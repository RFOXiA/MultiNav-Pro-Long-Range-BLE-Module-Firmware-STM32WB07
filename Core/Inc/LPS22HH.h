/*
 * LPS22HH.h
 *
 *  Created on: May 1, 2025
 *      Author: Mario Gamal Fayek
 */

#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_

// Includes
#include <stdint.h>
#include "i2c.h"
#include <stdio.h>
// config macros
// LPS22HH has two possible I2C addresses based on SDO/SA0 pin:
#define LPS22HHTR_I2C_ADDR    (0x5C << 1) // SDO/SA0 = 0 (GND) - WORKING in backup
// #define LPS22HHTR_I2C_ADDR    (0x5D << 1) // SDO/SA0 = 1 (VDD) - Alternative
#define LPS22HHTR_TIMEOUT		(5)			//timeout duration used when communicate with i2c
//Public Functions

/**
 * @brief Initialize the sensor with you config
 * @param i2ch pointer to I2C_handler used with sensor
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_Init(I2C_HandleTypeDef*);

/**
 * @brief raad the pressure in  hPA
 * @param pressure pointer to float to hold pressure
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_ReadRealPressure(float *);

/**
 * @brief raad the pressure raw data (3bytes)
 * @param pressure pointer to int array to hold 3 bytes of raw pressure value [0]->LSB, [1]->MID , [2]->MSB
 * @retval return Hal status
 */
HAL_StatusTypeDef LPS22HH_ReadRawPressure(uint8_t *);

/**
 * @brief Start DMA read for LPS22HH pressure - NON-BLOCKING
 * @retval HAL_OK if DMA started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef LPS22HH_StartDMARead(void);

/**
 * @brief Check if LPS22HH DMA read is complete and get results
 * @param raw_arr pointer to 3-byte array for pressure data [0]=LSB, [1]=MID, [2]=MSB
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef LPS22HH_GetDMAResult(uint8_t *raw_arr);

/**
 * @brief DMA RX Complete Callback for LPS22HH
 */
void LPS22HH_DMA_RxCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief DMA Error Callback for LPS22HH
 */
void LPS22HH_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_LPS22HH_H_ */
