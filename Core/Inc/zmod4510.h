/**
  ******************************************************************************
  * @file    zmod4510.h
  * @brief   ZMOD4510 Air Quality Sensor Driver
  * @details Simple wrapper for Renesas ZMOD4510 ULP O3 sensor
  *          Provides simplified API for STM32 I2C integration
  * @note    Uses Renesas ULP O3 algorithm library
  ******************************************************************************
  */

#ifndef ZMOD4510_H
#define ZMOD4510_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wb0x_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C Address - ZMOD4510 uses 0x33 (7-bit) on shared I2C2 bus */
#define ZMOD4510_I2C_ADDR    0x33

/**
 * @brief Initialize ZMOD4510 sensor (basic I2C setup only)
 * @param hi2c Pointer to I2C2 handle (shared bus)
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef ZMOD_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read raw ADC data from sensor (polling mode)
 * @param raw_data Pointer to 11-byte buffer:
 *                 [0-7]  = 8 ADC values (uint8_t each)
 *                 [8-9]  = sample counter (uint16_t, LSB first)
 *                 [10]   = valid flag (0x01 = valid)
 * @retval HAL_StatusTypeDef HAL_OK if successful
 * @note Uses shared I2C2 bus - coordinate with other sensor reads
 */
HAL_StatusTypeDef ZMOD_ReadRaw(uint8_t *raw_data);

/**
 * @brief Check if sensor is initialized
 * @retval bool true if initialized
 */
bool ZMOD_IsReady(void);

/**
 * @brief Start DMA read for ZMOD4510 (Air Quality)
 * @note Uses HAL_I2C_Mem_Read_DMA - coordinates with MVH4000D and LPS22HH on I2C2
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef ZMOD_StartDMARead(void);

/**
 * @brief Check if ZMOD4510 DMA is currently busy
 * @retval 1 if busy, 0 if idle
 */
uint8_t ZMOD_IsBusy(void);

/**
 * @brief Get DMA read results (non-blocking)
 * @param raw_data Pointer to 11-byte buffer (same format as ZMOD_ReadRaw)
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef ZMOD_GetDMAResult(uint8_t *raw_data);

/**
 * @brief DMA RX Complete Callback - call from HAL_I2C_MemRxCpltCallback in main.c
 * @param hi2c I2C handle
 */
void ZMOD_DMA_RxCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief DMA Error Callback - call from HAL_I2C_ErrorCallback in main.c
 * @param hi2c I2C handle
 */
void ZMOD_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* ZMOD4510_H */
