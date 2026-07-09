/**
 * @file    mvh4000d.c
 * @brief   Initialize the MVH4000D sensor on periodic mode and Alert every 1 sec
 * @author  Mario Gamal Fayek
 * @date    29th April 2025
 *
 *
 * @details
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "mvh4000d.h"
#include <stdio.h>

/* Private macro -------------------------------------------------------------*/
#define MVH4000D_I2C_ADDR    (0x54 << 1)  // 0xA8 - matching original firmware exactly
/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef* i2c_handler =NULL;

/* DMA State Management */
static volatile uint8_t mvh_dma_buffer[5];  // Buffer for DMA: 5 bytes (hum_msb, hum_lsb, temp_msb, temp_lsb, CRC)
static volatile uint8_t mvh_dma_busy = 0;   // 0=idle, 1=reading, 2=complete, 3=error
static volatile uint32_t mvh_dma_start_time = 0;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef MVH4000D_EnablePeriodicMode(void);
static HAL_StatusTypeDef MVH4000D_StopPeriodicMode(void);
static HAL_StatusTypeDef MVH4000D_SetResolution(uint8_t humi_res, uint8_t temp_res);
static HAL_StatusTypeDef MVH4000D_SoftReset(void);
static void MVH4000D_ScanI2CBus(void);
static HAL_StatusTypeDef MVH4000D_SingleShotMeasurement(uint16_t* temp, uint16_t* hum);
/* Public function  -----------------------------------------------*/
HAL_StatusTypeDef MVH4000D_Init(I2C_HandleTypeDef* i2ch)
{
	printf("MVH4000D Init: addr=0x%02X\r\n", MVH4000D_I2C_ADDR);
	
	i2c_handler = i2ch;
	
	// Power-on delay like original firmware
	HAL_Delay(50);
	
	// Stop periodic mode first (original firmware approach)
	MVH4000D_StopPeriodicMode();
	HAL_Delay(10); // Small delay between stop and start
	
	// Enable periodic mode exactly like original firmware
	if(MVH4000D_EnablePeriodicMode() == HAL_OK) {
		printf("MVH4000D init SUCCESS\r\n");
		return HAL_OK;
	} else {
		printf("MVH4000D init FAILED\r\n");
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MVH4000D_ReadTempHum(uint16_t* raw_temperature,uint16_t* raw_humidity)
{
	  uint8_t data[5];
	  
	  // Increased timeout from 5ms to 100ms to prevent occasional blocking
	  if(HAL_I2C_Master_Receive(i2c_handler, MVH4000D_I2C_ADDR, data, 5, 100) == HAL_ERROR) {
		  return HAL_ERROR;
	  }

	  // Parse sensor data (big-endian) - EXACTLY LIKE ORIGINAL FIRMWARE
	  // Returns raw ADC values (0-65535) - Android app must do conversion
	  *raw_humidity = (data[0] << 8 | data[1]);
	  *raw_temperature = (data[2] << 8 | data[3]);
	  
	  return HAL_OK;
}


/* Static (private) helper functions---------------------------------------------------------*/
static HAL_StatusTypeDef MVH4000D_EnablePeriodicMode(void)
{
    uint8_t cmd[3];
    
    // First register write - exactly like original firmware
    cmd[0] = 0xA6; // Write Register command
    cmd[1] = 0x03; // Address of configuration register
    cmd[2] = 0x0;
    if(HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 3, 10) != HAL_OK) {
        return HAL_ERROR;
    }

    // Second register write - exactly like original firmware
    cmd[0] = 0xA6; // Write Register command
    cmd[1] = 0x02; // Address of configuration register
    cmd[2] = 0b11000001; // Periodic mode config
    if(HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 3, 10) == HAL_OK) {
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
}

static HAL_StatusTypeDef MVH4000D_StopPeriodicMode(void)
{
    uint8_t cmd[2];

    cmd[0] = 0x30; // Stop periodic measurement command

    if(HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 1, 10)==HAL_OK)
    {
    	return HAL_OK;
    }
    else
    {
    	return HAL_ERROR;
    }
}

static HAL_StatusTypeDef MVH4000D_SetResolution(uint8_t humi_res, uint8_t temp_res)
{
    uint8_t cmd[3];
    uint8_t resolution_reg = 0x0f;

    // Build resolution byte
    //resolution_reg |= (humi_res & 0x03) << 2; // Humidity resolution into bits 3:2
    //resolution_reg |= (temp_res & 0x03) << 0; // Temperature resolution into bits 1:0

    cmd[0] = 0xA6;    // Write Register command
    cmd[1] = 0x00;    // Resolution register address
    cmd[2] = resolution_reg;

    return HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 3, 10);
}

static HAL_StatusTypeDef MVH4000D_SoftReset(void)
{
    uint8_t cmd[2];
    
    // Try SHT3x/SHT4x style reset (0x30A2)
    cmd[0] = 0x30;
    cmd[1] = 0xA2;
    if(HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 2, 100) == HAL_OK) {
        return HAL_OK;
    }
    
    // Try alternative reset command (0x06)
    cmd[0] = 0x06;
    return HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 1, 100);
}

static void MVH4000D_ScanI2CBus(void)
{
    printf("Scanning I2C bus...\r\n");
    int found_count = 0;
    
    for(uint8_t addr = 0x40; addr <= 0x90; addr++) {
        if(HAL_I2C_IsDeviceReady(i2c_handler, addr, 1, 10) == HAL_OK) {
            printf("  Device found at 0x%02X\r\n", addr);
            found_count++;
        }
    }
    
    if(found_count == 0) {
        printf("  No I2C devices found!\r\n");
    } else {
        printf("  Total devices found: %d\r\n", found_count);
    }
}

static HAL_StatusTypeDef MVH4000D_SingleShotMeasurement(uint16_t* temp, uint16_t* hum)
{
    uint8_t cmd[2];
    uint8_t data[6];
    HAL_StatusTypeDef status;
    
    // Try SHT3x/SHT4x high repeatability single-shot
    cmd[0] = 0x24;
    cmd[1] = 0x00;
    status = HAL_I2C_Master_Transmit(i2c_handler, MVH4000D_I2C_ADDR, cmd, 2, 100);
    if(status != HAL_OK) {
        printf("Single-shot command failed (status=%d)\r\n", status);
        return HAL_ERROR;
    }
    
    // Wait for measurement (typical 15ms for high repeatability)
    HAL_Delay(20);
    
    // Read result
    status = HAL_I2C_Master_Receive(i2c_handler, MVH4000D_I2C_ADDR, data, 6, 100);
    if(status != HAL_OK) {
        printf("Single-shot read failed (status=%d)\r\n", status);
        return HAL_ERROR;
    }
    
    // Parse data (SHT3x format: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC)
    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_hum = (data[3] << 8) | data[4];
    
    // Convert to temperature and humidity values
    // SHT3x formula: T = -45 + 175 * (raw_temp / 65535)
    // H = 100 * (raw_hum / 65535)
    *temp = (uint16_t)((-4500 + (17500.0f * raw_temp) / 65535.0f));
    *hum = (uint16_t)((10000.0f * raw_hum) / 65535.0f);
    
    return HAL_OK;
}

/**
 * @brief Start DMA read for MVH4000D (Temperature + Humidity)
 * @note Uses HAL_I2C_Master_Receive_DMA (no register address - periodic mode)
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef MVH4000D_StartDMARead(void)
{
    if (i2c_handler == NULL) {
        return HAL_ERROR;
    }
    
    // Check if previous transfer still ongoing
    if (mvh_dma_busy == 1) {
        // Check for timeout (should complete in <5ms)
        if ((HAL_GetTick() - mvh_dma_start_time) > 100) {
            mvh_dma_busy = 3;  // Mark as error
            return HAL_ERROR;
        }
        return HAL_BUSY;
    }
    
    // Start new DMA transfer
    mvh_dma_busy = 1;  // Mark as busy
    mvh_dma_start_time = HAL_GetTick();
    
    /* CRITICAL: Use HAL_I2C_Master_Receive_DMA, NOT HAL_I2C_Mem_Read_DMA
     * MVH4000D runs in periodic mode - no register address needed
     * Format: 5 bytes = hum_msb, hum_lsb, temp_msb, temp_lsb, CRC */
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(i2c_handler, 
                                                          MVH4000D_I2C_ADDR, 
                                                          (uint8_t*)mvh_dma_buffer, 
                                                          5);
    
    if (status != HAL_OK) {
        mvh_dma_busy = 3;  // Mark as error
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Check if MVH4000D DMA is currently busy
 * @retval 1 if busy (transfer in progress), 0 if idle
 */
uint8_t MVH4000D_IsBusy(void)
{
    return (mvh_dma_busy == 1) ? 1 : 0;  // Return 1 if actively reading, 0 otherwise
}

/**
 * @brief Check if MVH4000D DMA read is complete and get results
 * @param raw_temperature  pointer to 2byte temp variable
 * @param raw_humidity pointer to 2byte hum variable
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef MVH4000D_GetDMAResult(uint16_t* raw_temperature, uint16_t* raw_humidity)
{
    if (mvh_dma_busy == 1) {
        // Still reading
        return HAL_BUSY;
    }
    
    if (mvh_dma_busy == 3) {
        // Error occurred
        mvh_dma_busy = 0;  // Reset for next try
        return HAL_ERROR;
    }
    
    if (mvh_dma_busy == 2) {
        // Data ready - parse it
        *raw_humidity = (mvh_dma_buffer[0] << 8 | mvh_dma_buffer[1]);
        *raw_temperature = (mvh_dma_buffer[2] << 8 | mvh_dma_buffer[3]);
        
        mvh_dma_busy = 0;  // Reset for next read
        return HAL_OK;
    }
    
    // Not started yet
    return HAL_ERROR;
}

/**
 * @brief DMA RX Complete Callback for MVH4000D
 * @note Called by HAL when DMA transfer completes
 * @note This should be called from HAL_I2C_MasterRxCpltCallback in main.c or stm32wbxx_it.c
 */
void MVH4000D_DMA_RxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == i2c_handler && mvh_dma_busy == 1) {
        mvh_dma_busy = 2;  // Mark as complete
    }
}

/**
 * @brief DMA Error Callback for MVH4000D
 * @note Called by HAL when DMA transfer fails
 */
void MVH4000D_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == i2c_handler && mvh_dma_busy == 1) {
        mvh_dma_busy = 3;  // Mark as error
    }
}

