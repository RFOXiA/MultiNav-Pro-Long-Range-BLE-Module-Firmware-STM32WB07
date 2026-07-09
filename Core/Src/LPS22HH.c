/*
 * LPS22HH.c
 *
 *  Created on: May 1, 2025
 *  Author: Mario Gamal Fayek
 */

//includes
#include "LPS22HH.h"

//private macros
#define CTRL_REG1_ADD			(0x10)
#define CTRL_REG2_ADD			(0x11)
#define CTRL_REG3_ADD			(0x12)
#define STATUS_REG_ADD			(0x27)
#define PRESS_OUT_XL_ADD		(0x28)
#define PRESS_OUT_L_ADD			(0x29)
#define PRESS_OUT_H_ADD			(0x2a)
//private variables
static I2C_HandleTypeDef* i2c_handler =NULL;

/* DMA State Management */
static volatile uint8_t lps_dma_buffer[3];  // Buffer for DMA: 3 bytes (XL, L, H)
static volatile uint8_t lps_dma_busy = 0;   // 0=idle, 1=reading, 2=complete, 3=error
static volatile uint32_t lps_dma_start_time = 0;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef LPS22HH_Config();

//Public Functions

/**
 * @brief Initialize the sensor with you config
 * @param i2ch pointer to I2C_handler used with sensor
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_Init(I2C_HandleTypeDef* i2ch)
{
	//set i2c handler
	i2c_handler=i2ch;
	
	// Test if device is present
	printf("LPS22HH_Init: Checking device at I2C address 0x%02X\r\n", LPS22HHTR_I2C_ADDR);
	if(HAL_I2C_IsDeviceReady(i2c_handler, LPS22HHTR_I2C_ADDR, 3, 100) != HAL_OK) {
		printf("LPS22HH_Init: Device not responding!\r\n");
		return HAL_ERROR;
	}
	printf("LPS22HH_Init: Device detected, configuring...\r\n");
	
	//config file
	HAL_StatusTypeDef status = LPS22HH_Config();
	if(status == HAL_OK) {
		printf("LPS22HH_Init: Configuration successful\r\n");
	} else {
		printf("LPS22HH_Init: Configuration failed (status=%d)\r\n", status);
	}
	return status;
}

/**
 * @brief raad the pressure in  hPA
 * @param pressure pointer to float to hold pressure
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_ReadRealPressure(float * p_val)
{

		uint8_t val_arr[3];
		if(LPS22HH_ReadRawPressure(val_arr)!= HAL_OK)
			return HAL_ERROR;
		int32_t raw_p= ( (int32_t)(val_arr[2])<<16)|( (int32_t)(val_arr[1])<<8) | (val_arr[0]);
		 *p_val =raw_p/4096.0f;
		 return HAL_OK;
}

/**
 * @brief raad the pressure raw data (3bytes)
 * @param pressure pointer to int array to hold 3 bytes of raw pressure value [0]->LSB, [1]->MID , [2]->MSB
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_ReadRawPressure(uint8_t * raw_arr)
{
	// Read pressure directly without checking STATUS register
	// At 1 Hz ODR, after 3 second intervals there will always be valid data
	HAL_StatusTypeDef state;
	
	// Read 3 bytes of pressure (24-bit value)
	state = HAL_I2C_Mem_Read(i2c_handler, LPS22HHTR_I2C_ADDR, PRESS_OUT_XL_ADD, 1, raw_arr + 0, 1, LPS22HHTR_TIMEOUT);
	if(state != HAL_OK)
		return state;
	
	state = HAL_I2C_Mem_Read(i2c_handler, LPS22HHTR_I2C_ADDR, PRESS_OUT_L_ADD, 1, raw_arr + 1, 1, LPS22HHTR_TIMEOUT);
	if(state != HAL_OK)
		return state;
	
	state = HAL_I2C_Mem_Read(i2c_handler, LPS22HHTR_I2C_ADDR, PRESS_OUT_H_ADD, 1, raw_arr + 2, 1, LPS22HHTR_TIMEOUT);
	return state;
}

//private function
static HAL_StatusTypeDef LPS22HH_Config()
{
	uint8_t cmd[2];
	    HAL_StatusTypeDef state;

	    // 1. Set ODR to 1 Hz , low-pass filter (CTRL_REG1)
	   // state=HAL_I2C_Mem_Read(&hi2c1, LPS22HHTR_I2C_ADDR, CTRL_REG1_ADD, 1, cmd, 1, 50);
	    cmd[0] = CTRL_REG1_ADD;
	    cmd[1] = 0x1c; // ODR[2:0] = 001 = 1 Hz, low-pass filter enabled
	    state = HAL_I2C_Master_Transmit(i2c_handler, LPS22HHTR_I2C_ADDR, cmd, 2, LPS22HHTR_TIMEOUT);

	    //Guard if not HAL_OK return Hal_state
	    if(state != HAL_OK) {
	    	printf("LPS22HH_Config: CTRL_REG1 write failed (status=%d)\r\n", state);
	    	return state;
	    }

// 2. Set INT_DRDY as active low + enable auto-increment (CTRL_REG2)
    //state=HAL_I2C_Mem_Read(&hi2c1, LPS22HHTR_I2C_ADDR, CTRL_REG2_ADD, 1, cmd, 1, 50);
    cmd[0] = CTRL_REG2_ADD;
    cmd[1] = 0x50; // Bit6=1 (active low), Bit4=1 (IF_ADD_INC - auto-increment for multi-byte reads)
	    state = HAL_I2C_Master_Transmit(i2c_handler, LPS22HHTR_I2C_ADDR, cmd, 2, LPS22HHTR_TIMEOUT);

	    //Guard if not HAL_OK return Hal_state
		if(state != HAL_OK)
			return state;
	    // 3. Enable DRDY interrupt (CTRL_REG3)
	    //state=HAL_I2C_Mem_Read(&hi2c1, LPS22HHTR_I2C_ADDR, CTRL_REG3_ADD, 1, cmd, 1, 50);
	    cmd[0] = CTRL_REG3_ADD;
	    cmd[1] = 0x04; // Bit 3 = 1 → DRDY_EN , Bit [0:1] -> INT_S[1:0] = 00
	    state = HAL_I2C_Master_Transmit(i2c_handler, LPS22HHTR_I2C_ADDR, cmd, 2, LPS22HHTR_TIMEOUT);

	    //Guard if not HAL_OK return Hal_state
	    return state;
}

/**
 * @brief Start DMA read for LPS22HH (Pressure)
 * @note Uses HAL_I2C_Mem_Read_DMA to read 3 bytes starting from PRESS_OUT_XL
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef LPS22HH_StartDMARead(void)
{
    if (i2c_handler == NULL) {
        return HAL_ERROR;
    }
    
    // Check if previous transfer still ongoing
    if (lps_dma_busy == 1) {
        // Check for timeout (should complete in <5ms)
        if ((HAL_GetTick() - lps_dma_start_time) > 100) {
            lps_dma_busy = 3;  // Mark as error
            return HAL_ERROR;
        }
        return HAL_BUSY;
    }
    
    // Start new DMA transfer
    lps_dma_busy = 1;  // Mark as busy
    lps_dma_start_time = HAL_GetTick();
    
    /* Read 3 bytes: PRESS_OUT_XL (0x28), PRESS_OUT_L (0x29), PRESS_OUT_H (0x2A)
     * LPS22HH supports auto-increment, so we can read all 3 bytes in one transaction */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(i2c_handler, 
                                                     LPS22HHTR_I2C_ADDR, 
                                                     PRESS_OUT_XL_ADD,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     (uint8_t*)lps_dma_buffer, 
                                                     3);
    
    if (status != HAL_OK) {
        lps_dma_busy = 3;  // Mark as error
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Check if LPS22HH DMA read is complete and get results
 * @param raw_arr pointer to 3-byte array for pressure data
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef LPS22HH_GetDMAResult(uint8_t *raw_arr)
{
    if (lps_dma_busy == 1) {
        // Still reading
        return HAL_BUSY;
    }
    
    if (lps_dma_busy == 3) {
        // Error occurred
        lps_dma_busy = 0;  // Reset for next try
        return HAL_ERROR;
    }
    
    if (lps_dma_busy == 2) {
        // Data ready - copy it
        raw_arr[0] = lps_dma_buffer[0];  // PRESS_OUT_XL (LSB)
        raw_arr[1] = lps_dma_buffer[1];  // PRESS_OUT_L (MID)
        raw_arr[2] = lps_dma_buffer[2];  // PRESS_OUT_H (MSB)
        
        lps_dma_busy = 0;  // Reset for next read
        return HAL_OK;
    }
    
    // Not started yet
    return HAL_ERROR;
}

/**
 * @brief DMA RX Complete Callback for LPS22HH
 * @note Called by HAL when DMA transfer completes
 */
void LPS22HH_DMA_RxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == i2c_handler && lps_dma_busy == 1) {
        lps_dma_busy = 2;  // Mark as complete
    }
}

/**
 * @brief DMA Error Callback for LPS22HH
 * @note Called by HAL when DMA transfer fails
 */
void LPS22HH_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == i2c_handler && lps_dma_busy == 1) {
        lps_dma_busy = 3;  // Mark as error
    }
}


