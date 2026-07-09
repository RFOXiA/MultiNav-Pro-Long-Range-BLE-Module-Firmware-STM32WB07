/**
  ******************************************************************************
  * @file    zmod4510.c
  * @brief   ZMOD4510 Air Quality Sensor Driver Implementation
  * @details Simplified wrapper for Renesas ZMOD4510 ULP O3 sensor
  *          Provides STM32 HAL I2C integration
  ******************************************************************************
  */

#include "zmod4510.h"
#include <string.h>
#include <stdio.h>  // For printf debugging

/* Private variables */
static I2C_HandleTypeDef *zmod_i2c = NULL;
static bool initialized = false;
static uint32_t sample_counter = 0;
static uint8_t adc_result[8];  /* 8 ADC values from sensor */

/* DMA State Management */
static volatile uint8_t zmod_dma_buffer[8];  // Buffer for DMA: 8 ADC bytes
static volatile uint8_t zmod_dma_busy = 0;   // 0=idle, 1=reading, 2=complete, 3=error
static volatile uint32_t zmod_dma_start_time = 0;

/**
 * @brief Initialize ZMOD4510 sensor - simplified for raw data only
 * @note No algorithm initialization - just verify I2C communication
 */
HAL_StatusTypeDef ZMOD_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) {
        printf("ZMOD_Init: ERROR - NULL I2C handle\r\n");
        return HAL_ERROR;
    }
    
    zmod_i2c = hi2c;
    
    /* ZMOD4510 auto-starts measurements on power-up - no config needed */
    /* Just verify I2C communication is working */
    /* Use SAME parameters as I2C scanner: 1 retry, 10ms timeout */
    printf("ZMOD_Init: Checking device at I2C address 0x%02X (0x%02X in 8-bit)\r\n", 
           ZMOD4510_I2C_ADDR, ZMOD4510_I2C_ADDR << 1);
    
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(zmod_i2c, ZMOD4510_I2C_ADDR << 1, 1, 10);
    if (ret != HAL_OK) {
        printf("ZMOD_Init: Device not responding! (HAL status: 0x%02X)\r\n", ret);
        return ret;
    }
    
    printf("ZMOD_Init: Device detected successfully\r\n");
    initialized = true;
    sample_counter = 0;
    
    return HAL_OK;
}

/**
 * @brief Read raw ADC data from sensor (polling mode)
 * @param raw_data Pointer to 11-byte buffer: [8 ADC bytes] + [2 sample_count] + [1 valid]
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef ZMOD_ReadRaw(uint8_t *raw_data)
{
    if (!initialized || raw_data == NULL) {
        return HAL_ERROR;
    }
    
    /* Read 8 ADC bytes from sensor starting at register 0x00 */
    /* ZMOD4510 stores ADC results in sequential registers */
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(zmod_i2c, ZMOD4510_I2C_ADDR << 1, 
                                              0x00, I2C_MEMADD_SIZE_8BIT, 
                                              adc_result, 8, 100);
    if (ret != HAL_OK) {
        return ret;
    }
    
    /* Pack into 11-byte format for BLE transmission */
    /* Bytes 0-7: Raw ADC values */
    memcpy(raw_data, adc_result, 8);
    
    /* Bytes 8-9: Sample counter (LSB first) */
    sample_counter++;
    raw_data[8] = sample_counter & 0xFF;
    raw_data[9] = (sample_counter >> 8) & 0xFF;
    
    /* Byte 10: Valid flag (1 = valid data) */
    raw_data[10] = 0x01;
    
    return HAL_OK;
}

/**
 * @brief Check if sensor is initialized
 */
bool ZMOD_IsReady(void)
{
    return initialized;
}

/**
 * @brief Start DMA read for ZMOD4510 (Air Quality)
 * @note Uses HAL_I2C_Mem_Read_DMA to read 8 ADC bytes starting from register 0x00
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef ZMOD_StartDMARead(void)
{
    if (!initialized || zmod_i2c == NULL) {
        return HAL_ERROR;
    }
    
    // Check if previous transfer still ongoing
    if (zmod_dma_busy == 1) {
        // Check for timeout (should complete in <5ms)
        if ((HAL_GetTick() - zmod_dma_start_time) > 100) {
            zmod_dma_busy = 3;  // Mark as error
            return HAL_ERROR;
        }
        return HAL_BUSY;
    }
    
    // Start new DMA transfer
    zmod_dma_busy = 1;  // Mark as busy
    zmod_dma_start_time = HAL_GetTick();
    
    /* Read 8 ADC bytes from ZMOD4510 starting at register 0x00 */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(zmod_i2c, 
                                                     ZMOD4510_I2C_ADDR << 1,
                                                     0x00,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     (uint8_t*)zmod_dma_buffer, 
                                                     8);
    
    if (status != HAL_OK) {
        zmod_dma_busy = 3;  // Mark as error
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Check if ZMOD4510 DMA is currently busy
 * @retval 1 if busy (transfer in progress), 0 if idle
 */
uint8_t ZMOD_IsBusy(void)
{
    return (zmod_dma_busy == 1) ? 1 : 0;  // Return 1 if actively reading, 0 otherwise
}

/**
 * @brief Check if ZMOD4510 DMA read is complete and get results
 * @param raw_data Pointer to 11-byte buffer: [8 ADC] + [2 count] + [1 valid]
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef ZMOD_GetDMAResult(uint8_t *raw_data)
{
    if (raw_data == NULL) {
        return HAL_ERROR;
    }
    
    if (zmod_dma_busy == 1) {
        // Still reading
        return HAL_BUSY;
    }
    
    if (zmod_dma_busy == 3) {
        // Error occurred
        zmod_dma_busy = 0;  // Reset for next try
        return HAL_ERROR;
    }
    
    if (zmod_dma_busy == 2) {
        // Data ready - pack into 11-byte format
        memcpy(raw_data, (uint8_t*)zmod_dma_buffer, 8);  // Bytes 0-7: ADC values
        
        // Bytes 8-9: Sample counter (LSB first)
        sample_counter++;
        raw_data[8] = sample_counter & 0xFF;
        raw_data[9] = (sample_counter >> 8) & 0xFF;
        
        // Byte 10: Valid flag
        raw_data[10] = 0x01;
        
        zmod_dma_busy = 0;  // Reset for next read
        return HAL_OK;
    }
    
    // Not started yet
    return HAL_ERROR;
}

/**
 * @brief DMA RX Complete Callback for ZMOD4510
 * @note Called by HAL when DMA transfer completes
 * @note This should be called from HAL_I2C_MemRxCpltCallback in main.c
 */
void ZMOD_DMA_RxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == zmod_i2c && zmod_dma_busy == 1) {
        zmod_dma_busy = 2;  // Mark as complete
    }
}

/**
 * @brief DMA Error Callback for ZMOD4510
 * @note Called by HAL when DMA transfer fails
 */
void ZMOD_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == zmod_i2c && zmod_dma_busy == 1) {
        zmod_dma_busy = 3;  // Mark as error
    }
}
