/**
 * @file tmag5273.h
 * @brief TMAG5273 3-Axis Magnetometer Driver for STM32
 * @author RFOXiA Firmware Team
 * @date 2025
 * 
 * Simplified driver for TMAG5273C1QDBVR Hall-effect magnetometer
 * Based on TI example code, adapted for STM32 HAL
 */

#ifndef TMAG5273_H_
#define TMAG5273_H_

#include "stm32wb0x_hal.h"
#include <stdint.h>
#include <stdbool.h>

//****************************************************************************
// I2C Configuration
//****************************************************************************
#define TMAG5273_I2C_ADDR       0x78    // 7-bit I2C address (0xF0 8-bit write, actual PCB)
#define TMAG5273_I2C_TIMEOUT    100     // I2C timeout in milliseconds

//****************************************************************************
// Register Addresses
//****************************************************************************
#define TMAG5273_REG_DEVICE_CONFIG_1    0x00
#define TMAG5273_REG_DEVICE_CONFIG_2    0x01
#define TMAG5273_REG_SENSOR_CONFIG_1    0x02
#define TMAG5273_REG_SENSOR_CONFIG_2    0x03
#define TMAG5273_REG_X_THR_CONFIG       0x04
#define TMAG5273_REG_Y_THR_CONFIG       0x05
#define TMAG5273_REG_Z_THR_CONFIG       0x06
#define TMAG5273_REG_T_CONFIG           0x07
#define TMAG5273_REG_INT_CONFIG_1       0x08
#define TMAG5273_REG_MAG_GAIN_CONFIG    0x09
#define TMAG5273_REG_MAG_OFFSET_CONFIG_1 0x0A
#define TMAG5273_REG_MAG_OFFSET_CONFIG_2 0x0B
#define TMAG5273_REG_I2C_ADDRESS        0x0C
#define TMAG5273_REG_DEVICE_ID          0x0D
#define TMAG5273_REG_MANUFACTURER_ID_LSB 0x0E
#define TMAG5273_REG_MANUFACTURER_ID_MSB 0x0F
#define TMAG5273_REG_T_MSB_RESULT       0x10
#define TMAG5273_REG_T_LSB_RESULT       0x11
#define TMAG5273_REG_X_MSB_RESULT       0x12
#define TMAG5273_REG_X_LSB_RESULT       0x13
#define TMAG5273_REG_Y_MSB_RESULT       0x14
#define TMAG5273_REG_Y_LSB_RESULT       0x15
#define TMAG5273_REG_Z_MSB_RESULT       0x16
#define TMAG5273_REG_Z_LSB_RESULT       0x17
#define TMAG5273_REG_CONV_STATUS        0x18
#define TMAG5273_REG_ANGLE_RESULT_MSB   0x19
#define TMAG5273_REG_ANGLE_RESULT_LSB   0x1A
#define TMAG5273_REG_MAGNITUDE_RESULT   0x1B

//****************************************************************************
// Device Config Register Bit Definitions
//****************************************************************************
// DEVICE_CONFIG_1 (0x00)
#define TMAG5273_CONV_AVG_1X            0x00
#define TMAG5273_CONV_AVG_2X            0x04
#define TMAG5273_CONV_AVG_4X            0x08
#define TMAG5273_CONV_AVG_8X            0x0C
#define TMAG5273_CONV_AVG_16X           0x10
#define TMAG5273_CONV_AVG_32X           0x14

// DEVICE_CONFIG_2 (0x01)
#define TMAG5273_OPERATING_MODE_STANDBY         0x00
#define TMAG5273_OPERATING_MODE_SLEEP           0x01
#define TMAG5273_OPERATING_MODE_CONTINUOUS      0x02
#define TMAG5273_OPERATING_MODE_WAKEUP_SLEEP    0x03
#define TMAG5273_TRIGGER_MODE_I2C               0x00
#define TMAG5273_TRIGGER_MODE_INT_PIN           0x04
#define TMAG5273_LP_LN_LOW_POWER                0x00
#define TMAG5273_LP_LN_LOW_NOISE                0x10

// SENSOR_CONFIG_1 (0x02)
#define TMAG5273_MAG_CH_EN_XYZ          0x70    // Enable all 3 axes

// SENSOR_CONFIG_2 (0x03)
#define TMAG5273_RANGE_40mT             0x00    // ±40mT for X/Y, Z
#define TMAG5273_RANGE_80mT             0x03    // ±80mT for X/Y, Z
#define TMAG5273_ANGLE_EN_DISABLE       0x00
#define TMAG5273_ANGLE_EN_XY            0x04
#define TMAG5273_ANGLE_EN_YZ            0x08
#define TMAG5273_ANGLE_EN_XZ            0x0C

//****************************************************************************
// Data Structures
//****************************************************************************

/**
 * @brief TMAG5273 3-axis magnetic field data
 */
typedef struct {
    int16_t x_raw;      // Raw X-axis value (16-bit signed)
    int16_t y_raw;      // Raw Y-axis value (16-bit signed)
    int16_t z_raw;      // Raw Z-axis value (16-bit signed)
    float x_mT;         // X-axis magnetic field in milliTesla
    float y_mT;         // Y-axis magnetic field in milliTesla
    float z_mT;         // Z-axis magnetic field in milliTesla
    int16_t temp_raw;   // Raw temperature value
    float temp_C;       // Temperature in degrees Celsius
} TMAG5273_Data_t;

//****************************************************************************
// Function Prototypes
//****************************************************************************

/**
 * @brief Initialize TMAG5273 magnetometer
 * @param hi2c Pointer to I2C handle
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Read raw magnetic field data from all axes
 * @param data Pointer to data structure to store results
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_ReadRaw(TMAG5273_Data_t* data);

/**
 * @brief Read and convert magnetic field data to milliTesla
 * @param data Pointer to data structure to store results
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_ReadData(TMAG5273_Data_t* data);

/**
 * @brief Read device ID register
 * @param id Pointer to store device ID
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_ReadDeviceID(uint8_t* id);

/**
 * @brief Check if new data is available
 * @return true if new data ready, false otherwise
 */
bool TMAG5273_IsDataReady(void);

/**
 * @brief Write to a single register
 * @param reg Register address
 * @param value Value to write
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_WriteRegister(uint8_t reg, uint8_t value);

/**
 * @brief Read from a single register
 * @param reg Register address
 * @param value Pointer to store read value
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef TMAG5273_ReadRegister(uint8_t reg, uint8_t* value);

#endif /* TMAG5273_H_ */
