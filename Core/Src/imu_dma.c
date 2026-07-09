/**
  ******************************************************************************
  * @file    imu_dma.c
  * @brief   IMU Sensors DMA Reading Implementation
  * @date    January 17, 2026
  ******************************************************************************
  * @details
  * This module implements continuous DMA-based reading of IMU sensors:
  * - BMI270: Accelerometer (3-axis) + Gyroscope (3-axis)
  * - TMAG5273: Magnetometer (3-axis)
  * 
  * Data format for BLE transmission (6 bytes per sensor):
  * - Bytes 0-1: X-axis (int16_t little-endian)
  * - Bytes 2-3: Y-axis (int16_t little-endian)  
  * - Bytes 4-5: Z-axis (int16_t little-endian)
  * 
  * Scaling factors (Android side):
  * - Accelerometer: value * 0.001 = m/s²
  * - Gyroscope: value * 0.001 = °/s
  * - Magnetometer: value * 0.01 = µT
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "imu_dma.h"
#include "bmi270.h"
#include "tmag5273.h"
#include "sensor_server.h"
#include "sensor_server_app.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    IMU_DMA_STATE_IDLE = 0,
    IMU_DMA_STATE_READING_BMI270,
    IMU_DMA_STATE_READING_TMAG5273,
    IMU_DMA_STATE_ERROR
} IMU_DMA_State_t;

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;  /* BMI270 and TMAG5273 are on I2C2 */

/* Global IMU data (updated by DMA callbacks) */
volatile IMU_Data_t g_imu_data = {0};

/* DMA buffers */
static uint8_t bmi270_dma_buffer[BMI270_DATA_SIZE];  /* 12 bytes: 6 accel + 6 gyro */
static uint8_t tmag5273_dma_buffer[TMAG5273_DATA_SIZE];  /* 6 bytes: 2*X + 2*Y + 2*Z */

/* State tracking */
static IMU_DMA_State_t imu_dma_state = IMU_DMA_STATE_IDLE;
static uint32_t last_update_time = 0;
static uint8_t imu_initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef IMU_ReadBMI270_DMA(void);
static HAL_StatusTypeDef IMU_ReadTMAG5273_DMA(void);
static void IMU_ProcessBMI270Data(void);
static void IMU_ProcessTMAG5273Data(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize IMU DMA reading system
  */
HAL_StatusTypeDef IMU_DMA_Init(void)
{
    printf("[IMU-DMA] Initializing IMU DMA system...\r\n");
    
    /* BMI270 and TMAG5273 should already be initialized in main.c */
    /* Just reset our state */
    imu_dma_state = IMU_DMA_STATE_IDLE;
    last_update_time = 0;
    memset((void*)&g_imu_data, 0, sizeof(IMU_Data_t));
    memset(bmi270_dma_buffer, 0, sizeof(bmi270_dma_buffer));
    memset(tmag5273_dma_buffer, 0, sizeof(tmag5273_dma_buffer));
    
    imu_initialized = 1;
    printf("[IMU-DMA] Initialization complete\r\n");
    
    return HAL_OK;
}

/**
  * @brief  Start continuous IMU reading
  */
HAL_StatusTypeDef IMU_DMA_StartReading(void)
{
    if (!imu_initialized) {
        return HAL_ERROR;
    }
    
    imu_dma_state = IMU_DMA_STATE_IDLE;
    printf("[IMU-DMA] Started continuous reading at 10Hz\r\n");
    
    return HAL_OK;
}

/**
  * @brief  Stop IMU DMA reading
  */
HAL_StatusTypeDef IMU_DMA_StopReading(void)
{
    /* Abort any ongoing DMA transfers */
    if (imu_dma_state == IMU_DMA_STATE_READING_BMI270 || 
        imu_dma_state == IMU_DMA_STATE_READING_TMAG5273) {
        HAL_I2C_Master_Abort_IT(&hi2c2, BMI270_I2C_ADDR << 1);
    }
    
    imu_dma_state = IMU_DMA_STATE_IDLE;
    printf("[IMU-DMA] Stopped reading\r\n");
    
    return HAL_OK;
}

/**
  * @brief  Get current IMU data (thread-safe copy)
  */
HAL_StatusTypeDef IMU_GetData(IMU_Data_t *pData)
{
    if (pData == NULL) {
        return HAL_ERROR;
    }
    
    /* Atomic copy of volatile data */
    __disable_irq();
    memcpy(pData, (void*)&g_imu_data, sizeof(IMU_Data_t));
    __enable_irq();
    
    return HAL_OK;
}

/**
  * @brief  Process IMU updates (call from main loop)
  * @note   Triggers DMA read every 100ms (10Hz)
  */
void IMU_Process(void)
{
    if (!imu_initialized) {
        return;
    }
    
    uint32_t now = HAL_GetTick();
    
    /* Check if it's time for next update (10Hz = 100ms interval) */
    if ((now - last_update_time) < IMU_UPDATE_RATE_MS) {
        return;
    }
    
    last_update_time = now;
    
    /* State machine for alternating sensor reads */
    switch (imu_dma_state) {
        case IMU_DMA_STATE_IDLE:
            /* Start with BMI270 (accel + gyro) */
            if (IMU_ReadBMI270_DMA() == HAL_OK) {
                imu_dma_state = IMU_DMA_STATE_READING_BMI270;
            }
            break;
            
        case IMU_DMA_STATE_READING_BMI270:
            /* BMI270 read complete, now read TMAG5273 */
            if (IMU_ReadTMAG5273_DMA() == HAL_OK) {
                imu_dma_state = IMU_DMA_STATE_READING_TMAG5273;
            } else {
                imu_dma_state = IMU_DMA_STATE_IDLE;
            }
            break;
            
        case IMU_DMA_STATE_READING_TMAG5273:
            /* Both sensors read, return to idle */
            imu_dma_state = IMU_DMA_STATE_IDLE;
            break;
            
        case IMU_DMA_STATE_ERROR:
            /* Reset to idle on error */
            printf("[IMU-DMA] Error state, resetting...\r\n");
            imu_dma_state = IMU_DMA_STATE_IDLE;
            break;
            
        default:
            imu_dma_state = IMU_DMA_STATE_IDLE;
            break;
    }
}

/**
  * @brief  Send IMU notifications via BLE
  * @param  connHandle: BLE connection handle
  * @note   This function is called from sensor_server_app.c which checks notification status
  */
void IMU_SendBLENotifications(uint16_t connHandle)
{
    if (connHandle == 0xFFFF) {
        return;  /* No connection */
    }
    
    SENSOR_SERVER_Data_t sensor_data;
    /* CRITICAL: Static buffers persist after function returns for async BLE transmission */
    static uint8_t accel_buffer[6];
    static uint8_t gyro_buffer[6];
    static uint8_t mag_buffer[6];
    
    /* Pack Accelerometer data */
    accel_buffer[0] = (uint8_t)(g_imu_data.accel.x & 0xFF);
    accel_buffer[1] = (uint8_t)((g_imu_data.accel.x >> 8) & 0xFF);
    accel_buffer[2] = (uint8_t)(g_imu_data.accel.y & 0xFF);
    accel_buffer[3] = (uint8_t)((g_imu_data.accel.y >> 8) & 0xFF);
    accel_buffer[4] = (uint8_t)(g_imu_data.accel.z & 0xFF);
    accel_buffer[5] = (uint8_t)((g_imu_data.accel.z >> 8) & 0xFF);
    
    /* Pack Gyroscope data */
    gyro_buffer[0] = (uint8_t)(g_imu_data.gyro.x & 0xFF);
    gyro_buffer[1] = (uint8_t)((g_imu_data.gyro.x >> 8) & 0xFF);
    gyro_buffer[2] = (uint8_t)(g_imu_data.gyro.y & 0xFF);
    gyro_buffer[3] = (uint8_t)((g_imu_data.gyro.y >> 8) & 0xFF);
    gyro_buffer[4] = (uint8_t)(g_imu_data.gyro.z & 0xFF);
    gyro_buffer[5] = (uint8_t)((g_imu_data.gyro.z >> 8) & 0xFF);
    
    /* Pack Magnetometer data */
    mag_buffer[0] = (uint8_t)(g_imu_data.mag.x & 0xFF);
    mag_buffer[1] = (uint8_t)((g_imu_data.mag.x >> 8) & 0xFF);
    mag_buffer[2] = (uint8_t)(g_imu_data.mag.y & 0xFF);
    mag_buffer[3] = (uint8_t)((g_imu_data.mag.y >> 8) & 0xFF);
    mag_buffer[4] = (uint8_t)(g_imu_data.mag.z & 0xFF);
    mag_buffer[5] = (uint8_t)((g_imu_data.mag.z >> 8) & 0xFF);
    
    /* Send Accelerometer notification */
    sensor_data.p_Payload = accel_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_ACCEL_CHAR, &sensor_data, connHandle);
    
    /* Send Gyroscope notification */
    sensor_data.p_Payload = gyro_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GYRO_CHAR, &sensor_data, connHandle);
    
    /* Send Magnetometer notification */
    sensor_data.p_Payload = mag_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_MAG_CHAR, &sensor_data, connHandle);
}

/**
  * @brief  Send Accelerometer notification to specific connection
  * @param  connHandle: BLE connection handle
  */
void IMU_SendAccelNotification(uint16_t connHandle)
{
    if (connHandle == 0xFFFF) {
        return;
    }
    
    SENSOR_SERVER_Data_t sensor_data;
    static uint8_t accel_buffer[6];
    
    accel_buffer[0] = (uint8_t)(g_imu_data.accel.x & 0xFF);
    accel_buffer[1] = (uint8_t)((g_imu_data.accel.x >> 8) & 0xFF);
    accel_buffer[2] = (uint8_t)(g_imu_data.accel.y & 0xFF);
    accel_buffer[3] = (uint8_t)((g_imu_data.accel.y >> 8) & 0xFF);
    accel_buffer[4] = (uint8_t)(g_imu_data.accel.z & 0xFF);
    accel_buffer[5] = (uint8_t)((g_imu_data.accel.z >> 8) & 0xFF);
    
    sensor_data.p_Payload = accel_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_ACCEL_CHAR, &sensor_data, connHandle);
}

/**
  * @brief  Send Gyroscope notification to specific connection
  * @param  connHandle: BLE connection handle
  */
void IMU_SendGyroNotification(uint16_t connHandle)
{
    if (connHandle == 0xFFFF) {
        return;
    }
    
    SENSOR_SERVER_Data_t sensor_data;
    static uint8_t gyro_buffer[6];
    
    gyro_buffer[0] = (uint8_t)(g_imu_data.gyro.x & 0xFF);
    gyro_buffer[1] = (uint8_t)((g_imu_data.gyro.x >> 8) & 0xFF);
    gyro_buffer[2] = (uint8_t)(g_imu_data.gyro.y & 0xFF);
    gyro_buffer[3] = (uint8_t)((g_imu_data.gyro.y >> 8) & 0xFF);
    gyro_buffer[4] = (uint8_t)(g_imu_data.gyro.z & 0xFF);
    gyro_buffer[5] = (uint8_t)((g_imu_data.gyro.z >> 8) & 0xFF);
    
    sensor_data.p_Payload = gyro_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GYRO_CHAR, &sensor_data, connHandle);
}

/**
  * @brief  Send Magnetometer notification to specific connection
  * @param  connHandle: BLE connection handle
  */
void IMU_SendMagNotification(uint16_t connHandle)
{
    if (connHandle == 0xFFFF) {
        return;
    }
    
    SENSOR_SERVER_Data_t sensor_data;
    static uint8_t mag_buffer[6];
    
    mag_buffer[0] = (uint8_t)(g_imu_data.mag.x & 0xFF);
    mag_buffer[1] = (uint8_t)((g_imu_data.mag.x >> 8) & 0xFF);
    mag_buffer[2] = (uint8_t)(g_imu_data.mag.y & 0xFF);
    mag_buffer[3] = (uint8_t)((g_imu_data.mag.y >> 8) & 0xFF);
    mag_buffer[4] = (uint8_t)(g_imu_data.mag.z & 0xFF);
    mag_buffer[5] = (uint8_t)((g_imu_data.mag.z >> 8) & 0xFF);
    
    sensor_data.p_Payload = mag_buffer;
    sensor_data.Length = 6;
    SENSOR_SERVER_NotifyValue(SENSOR_SERVER_MAG_CHAR, &sensor_data, connHandle);
}

/**
  * @brief  Check if IMU data is fresh
  */
uint8_t IMU_IsDataFresh(uint32_t max_age_ms)
{
    uint32_t age = HAL_GetTick() - g_imu_data.timestamp;
    return (age <= max_age_ms) ? 1 : 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read BMI270 data via DMA (blocking with timeout for now)
  * @note   Reads 12 bytes: 6 bytes accel (X,Y,Z) + 6 bytes gyro (X,Y,Z)
  */
static HAL_StatusTypeDef IMU_ReadBMI270_DMA(void)
{
    HAL_StatusTypeDef status;
    
    /* Read 12 bytes starting from data register (0x0C) */
    /* Using polling mode for simplicity - can be upgraded to DMA later */
    status = HAL_I2C_Mem_Read(&hi2c2, 
                              BMI270_I2C_ADDR << 1, 
                              BMI270_REG_DATA, 
                              I2C_MEMADD_SIZE_8BIT, 
                              bmi270_dma_buffer, 
                              BMI270_DATA_SIZE, 
                              100);
    
    if (status == HAL_OK) {
        IMU_ProcessBMI270Data();
    }
    
    return status;
}

/**
  * @brief  Read TMAG5273 data via DMA (blocking with timeout for now)
  * @note   Reads 6 bytes: X (2 bytes) + Y (2 bytes) + Z (2 bytes)
  */
static HAL_StatusTypeDef IMU_ReadTMAG5273_DMA(void)
{
    HAL_StatusTypeDef status;
    
    /* Read 6 bytes starting from X-axis MSB register (0x12) */
    status = HAL_I2C_Mem_Read(&hi2c2, 
                              TMAG5273_I2C_ADDR << 1, 
                              TMAG5273_REG_X_MSB, 
                              I2C_MEMADD_SIZE_8BIT, 
                              tmag5273_dma_buffer, 
                              TMAG5273_DATA_SIZE, 
                              100);
    
    if (status == HAL_OK) {
        IMU_ProcessTMAG5273Data();
    }
    
    return status;
}

/**
  * @brief  Process BMI270 raw data into global structure
  * @note   BMI270 data format: [AccX_L, AccX_H, AccY_L, AccY_H, AccZ_L, AccZ_H,
  *                               GyroX_L, GyroX_H, GyroY_L, GyroY_H, GyroZ_L, GyroZ_H]
  */
static void IMU_ProcessBMI270Data(void)
{
    /* Parse accelerometer data (bytes 0-5) */
    g_imu_data.accel.x = (int16_t)((bmi270_dma_buffer[1] << 8) | bmi270_dma_buffer[0]);
    g_imu_data.accel.y = (int16_t)((bmi270_dma_buffer[3] << 8) | bmi270_dma_buffer[2]);
    g_imu_data.accel.z = (int16_t)((bmi270_dma_buffer[5] << 8) | bmi270_dma_buffer[4]);
    
    /* Parse gyroscope data (bytes 6-11) */
    g_imu_data.gyro.x = (int16_t)((bmi270_dma_buffer[7] << 8) | bmi270_dma_buffer[6]);
    g_imu_data.gyro.y = (int16_t)((bmi270_dma_buffer[9] << 8) | bmi270_dma_buffer[8]);
    g_imu_data.gyro.z = (int16_t)((bmi270_dma_buffer[11] << 8) | bmi270_dma_buffer[10]);
    
    /* Update timestamp */
    g_imu_data.timestamp = HAL_GetTick();
}

/**
  * @brief  Process TMAG5273 raw data into global structure
  * @note   TMAG5273 data format: [X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB]
  */
static void IMU_ProcessTMAG5273Data(void)
{
    /* Parse magnetometer data (MSB first) */
    g_imu_data.mag.x = (int16_t)((tmag5273_dma_buffer[0] << 8) | tmag5273_dma_buffer[1]);
    g_imu_data.mag.y = (int16_t)((tmag5273_dma_buffer[2] << 8) | tmag5273_dma_buffer[3]);
    g_imu_data.mag.z = (int16_t)((tmag5273_dma_buffer[4] << 8) | tmag5273_dma_buffer[5]);
    
    /* Update timestamp */
    g_imu_data.timestamp = HAL_GetTick();
}

/**
  * @brief  IMU I2C DMA complete handler (called from main.c callback)
  */
void IMU_DMA_RxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2) {
        /* DMA transfer complete */
        if (imu_dma_state == IMU_DMA_STATE_READING_BMI270) {
            IMU_ProcessBMI270Data();
        } else if (imu_dma_state == IMU_DMA_STATE_READING_TMAG5273) {
            IMU_ProcessTMAG5273Data();
        }
    }
}

/**
  * @brief  IMU I2C error handler (called from main.c callback)
  */
void IMU_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2) {
        printf("[IMU-DMA] I2C error occurred!\r\n");
        imu_dma_state = IMU_DMA_STATE_ERROR;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
