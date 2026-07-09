/**
  ******************************************************************************
  * @file    imu_dma.h
  * @brief   IMU Sensors DMA Reading (BMI270 + TMAG5273)
  * @date    January 17, 2026
  ******************************************************************************
  */

#ifndef __IMU_DMA_H
#define __IMU_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  IMU 3-axis data structure
  */
typedef struct {
    int16_t x;  /**< X-axis (int16_t little-endian) */
    int16_t y;  /**< Y-axis (int16_t little-endian) */
    int16_t z;  /**< Z-axis (int16_t little-endian) */
} IMU_Axis_Data_t;

/**
  * @brief  Complete IMU data set
  */
typedef struct {
    IMU_Axis_Data_t accel;  /**< Accelerometer data (m/s² * 1000) */
    IMU_Axis_Data_t gyro;   /**< Gyroscope data (°/s * 1000) */
    IMU_Axis_Data_t mag;    /**< Magnetometer data (µT * 100) */
    uint32_t timestamp;     /**< Last update timestamp (ms) */
} IMU_Data_t;

/* Exported constants --------------------------------------------------------*/

/* BMI270 I2C Address and Registers */
#define BMI270_I2C_ADDR         0x68    /* 7-bit address */
#define BMI270_REG_DATA         0x0C    /* Accelerometer data starts here (6 bytes accel + 6 bytes gyro) */
#define BMI270_DATA_SIZE        12      /* 6 bytes accel + 6 bytes gyro */

/* TMAG5273 I2C Address and Registers */
#define TMAG5273_I2C_ADDR       0x78    /* 7-bit address (actual PCB) */
#define TMAG5273_REG_X_MSB      0x12    /* X-axis MSB register */
#define TMAG5273_DATA_SIZE      6       /* 2 bytes X + 2 bytes Y + 2 bytes Z */

/* Update rate */
#define IMU_UPDATE_RATE_MS      100     /* 10Hz update rate (100ms interval) */

/* Exported variables --------------------------------------------------------*/

/* Global IMU data (updated by DMA) */
extern volatile IMU_Data_t g_imu_data;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize IMU DMA reading system
  * @retval HAL status
  */
HAL_StatusTypeDef IMU_DMA_Init(void);

/**
  * @brief  Start continuous IMU reading via DMA
  * @note   Reads BMI270 and TMAG5273 alternately
  * @retval HAL status
  */
HAL_StatusTypeDef IMU_DMA_StartReading(void);

/**
  * @brief  Stop IMU DMA reading
  * @retval HAL status
  */
HAL_StatusTypeDef IMU_DMA_StopReading(void);

/**
  * @brief  Get current IMU data (thread-safe copy)
  * @param  pData: Pointer to destination structure
  * @retval HAL_OK if data is valid
  */
HAL_StatusTypeDef IMU_GetData(IMU_Data_t *pData);

/**
  * @brief  Process IMU updates (call from main loop or timer)
  * @note   Triggers DMA read every 20ms (50Hz)
  * @retval None
  */
void IMU_Process(void);

/**
  * @brief  Send IMU notifications via BLE
  * @param  connHandle: BLE connection handle
  * @retval None
  */
void IMU_SendBLENotifications(uint16_t connHandle);

/**
  * @brief  Send Accelerometer notification via BLE
  * @param  connHandle: BLE connection handle
  * @retval None
  */
void IMU_SendAccelNotification(uint16_t connHandle);

/**
  * @brief  Send Gyroscope notification via BLE
  * @param  connHandle: BLE connection handle
  * @retval None
  */
void IMU_SendGyroNotification(uint16_t connHandle);

/**
  * @brief  Send Magnetometer notification via BLE
  * @param  connHandle: BLE connection handle
  * @retval None
  */
void IMU_SendMagNotification(uint16_t connHandle);

/**
  * @brief  Check if IMU data is fresh (updated recently)
  * @param  max_age_ms: Maximum age in milliseconds
  * @retval 1 if fresh, 0 if stale
  */
uint8_t IMU_IsDataFresh(uint32_t max_age_ms);

/**
  * @brief  IMU I2C DMA complete handler (call from HAL_I2C_MemRxCpltCallback)
  * @param  hi2c: I2C handle
  * @retval None
  */
void IMU_DMA_RxCallback(I2C_HandleTypeDef *hi2c);

/**
  * @brief  IMU I2C error handler (call from HAL_I2C_ErrorCallback)
  * @param  hi2c: I2C handle
  * @retval None
  */
void IMU_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DMA_H */
