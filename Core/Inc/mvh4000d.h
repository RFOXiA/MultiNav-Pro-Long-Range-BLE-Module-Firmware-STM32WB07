/*
 * mvh4000d.h
 *
 *  Created on: Apr 29, 2025
 *      Author: Mario Gamal Fayek
 */

#ifndef INC_MVH4000D_H_
#define INC_MVH4000D_H_

// Includes
#include <stdint.h>
#include "i2c.h"
/**
 * @brief Initialize the sensor
 * @param pointer to I2C_hadler
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef MVH4000D_Init(I2C_HandleTypeDef*);

/**
 * @brief read raw date of temperature and humidity from the sensor 14-bit for each
 * @param raw_temperature  pointer to 2byte temp variable
 * @param raw_humidity pointer to 2byte hum variable
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef MVH4000D_ReadTempHum(uint16_t* raw_temperature,uint16_t* raw_humidity);

/**
 * @brief Start DMA read for MVH4000D - NON-BLOCKING (uses correct Master_Receive_DMA)
 * @retval HAL_OK if DMA started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef MVH4000D_StartDMARead(void);

/**
 * @brief Check if MVH4000D DMA read is complete and get results
 * @param raw_temperature  pointer to 2byte temp variable
 * @param raw_humidity pointer to 2byte hum variable
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef MVH4000D_GetDMAResult(uint16_t* raw_temperature, uint16_t* raw_humidity);

/**
 * @brief Check if MVH4000D DMA is currently busy
 * @retval 1 if busy (transfer in progress), 0 if idle
 */
uint8_t MVH4000D_IsBusy(void);

/**
 * @brief DMA RX Complete Callback - call from HAL_I2C_MasterRxCpltCallback
 */
void MVH4000D_DMA_RxCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief DMA Error Callback - call from HAL_I2C_ErrorCallback
 */
void MVH4000D_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_MVH4000D_H_ */
