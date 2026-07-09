/**
 * @file tmag5273.c
 * @brief TMAG5273 3-Axis Magnetometer Driver Implementation
 * @author RFOXiA Firmware Team
 * @date 2025
 * 
 * Simplified driver for TMAG5273C1QDBVR Hall-effect magnetometer
 * Based on TI example code, adapted for STM32 HAL
 * 
 * Features:
 * - Continuous measurement mode
 * - 3-axis magnetic field sensing (X, Y, Z)
 * - ±40mT measurement range
 * - I2C polling interface
 * - Temperature measurement
 */

#include "tmag5273.h"
#include <stdio.h>
#include <string.h>

//****************************************************************************
// Private Variables
//****************************************************************************
static I2C_HandleTypeDef* tmag_i2c = NULL;
static uint16_t xy_range_mT = 40;   // ±40mT default range for X/Y axes
static uint16_t z_range_mT = 40;    // ±40mT default range for Z axis

//****************************************************************************
// Private Function Prototypes
//****************************************************************************
static float TMAG5273_ConvertToMilliTesla(int16_t raw_value, uint16_t range_mT);
static float TMAG5273_ConvertToTemperature(int16_t temp_raw);

//****************************************************************************
// Public Functions
//****************************************************************************

/**
 * @brief Initialize TMAG5273 magnetometer
 */
HAL_StatusTypeDef TMAG5273_Init(I2C_HandleTypeDef* hi2c)
{
    tmag_i2c = hi2c;
    HAL_StatusTypeDef status;
    uint8_t device_id = 0;
    
    printf("TMAG5273_Init: Initializing 3-axis magnetometer...\r\n");
    
    // Check if device is present on I2C bus
    status = HAL_I2C_IsDeviceReady(tmag_i2c, TMAG5273_I2C_ADDR << 1, 3, TMAG5273_I2C_TIMEOUT);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Device not responding at I2C address 0x%02X\r\n", TMAG5273_I2C_ADDR);
        return HAL_ERROR;
    }
    printf("TMAG5273_Init: Device detected at I2C address 0x%02X\r\n", TMAG5273_I2C_ADDR);
    
    // Read and verify device ID
    status = TMAG5273_ReadDeviceID(&device_id);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to read device ID\r\n");
        return HAL_ERROR;
    }
    printf("TMAG5273_Init: Device ID = 0x%02X\r\n", device_id);
    
    // Small delay for device stability
    HAL_Delay(10);
    
    // Configure DEVICE_CONFIG_1: 
    // - No averaging (fastest response)
    // - Standard I2C read mode
    // - No temperature compensation
    // - CRC disabled
    uint8_t config1 = TMAG5273_CONV_AVG_1X;
    status = TMAG5273_WriteRegister(TMAG5273_REG_DEVICE_CONFIG_1, config1);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to write DEVICE_CONFIG_1\r\n");
        return HAL_ERROR;
    }
    
    HAL_Delay(5);
    
    // Configure DEVICE_CONFIG_2:
    // - Continuous measurement mode
    // - I2C trigger mode
    // - Low noise mode (higher current but better accuracy)
    // - I2C glitch filter on
    uint8_t config2 = TMAG5273_OPERATING_MODE_CONTINUOUS | 
                      TMAG5273_TRIGGER_MODE_I2C | 
                      TMAG5273_LP_LN_LOW_NOISE;
    status = TMAG5273_WriteRegister(TMAG5273_REG_DEVICE_CONFIG_2, config2);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to write DEVICE_CONFIG_2\r\n");
        return HAL_ERROR;
    }
    
    HAL_Delay(5);
    
    // Configure SENSOR_CONFIG_1:
    // - Enable X, Y, Z magnetic channels
    // - No sleep time (continuous mode)
    uint8_t sensor_config1 = TMAG5273_MAG_CH_EN_XYZ;
    status = TMAG5273_WriteRegister(TMAG5273_REG_SENSOR_CONFIG_1, sensor_config1);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to write SENSOR_CONFIG_1\r\n");
        return HAL_ERROR;
    }
    
    HAL_Delay(5);
    
    // Configure SENSOR_CONFIG_2:
    // - ±40mT range for X/Y axes
    // - ±40mT range for Z axis
    // - Angle calculation disabled
    uint8_t sensor_config2 = TMAG5273_RANGE_40mT | TMAG5273_ANGLE_EN_DISABLE;
    status = TMAG5273_WriteRegister(TMAG5273_REG_SENSOR_CONFIG_2, sensor_config2);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to write SENSOR_CONFIG_2\r\n");
        return HAL_ERROR;
    }
    
    HAL_Delay(5);
    
    // Configure T_CONFIG:
    // - Enable temperature measurement
    uint8_t temp_config = 0x01;  // T_CH_EN = 1
    status = TMAG5273_WriteRegister(TMAG5273_REG_T_CONFIG, temp_config);
    if(status != HAL_OK) {
        printf("TMAG5273_Init: Failed to write T_CONFIG\r\n");
        return HAL_ERROR;
    }
    
    HAL_Delay(10);  // Allow first conversion to complete
    
    printf("TMAG5273_Init: Configuration complete\r\n");
    printf("TMAG5273_Init: Mode = Continuous, Range = ±%dmT, Channels = XYZ + Temp\r\n", xy_range_mT);
    
    return HAL_OK;
}

/**
 * @brief Read raw magnetic field data from all axes
 */
HAL_StatusTypeDef TMAG5273_ReadRaw(TMAG5273_Data_t* data)
{
    if(tmag_i2c == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status;
    uint8_t buffer[2];
    
    // Read X-axis (MSB first)
    status = HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1, 
                              TMAG5273_REG_X_MSB_RESULT, I2C_MEMADD_SIZE_8BIT,
                              buffer, 2, TMAG5273_I2C_TIMEOUT);
    if(status != HAL_OK) return HAL_ERROR;
    data->x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    // Read Y-axis
    status = HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1, 
                              TMAG5273_REG_Y_MSB_RESULT, I2C_MEMADD_SIZE_8BIT,
                              buffer, 2, TMAG5273_I2C_TIMEOUT);
    if(status != HAL_OK) return HAL_ERROR;
    data->y_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    // Read Z-axis
    status = HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1, 
                              TMAG5273_REG_Z_MSB_RESULT, I2C_MEMADD_SIZE_8BIT,
                              buffer, 2, TMAG5273_I2C_TIMEOUT);
    if(status != HAL_OK) return HAL_ERROR;
    data->z_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    // Read Temperature
    status = HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1, 
                              TMAG5273_REG_T_MSB_RESULT, I2C_MEMADD_SIZE_8BIT,
                              buffer, 2, TMAG5273_I2C_TIMEOUT);
    if(status != HAL_OK) return HAL_ERROR;
    data->temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    return HAL_OK;
}

/**
 * @brief Read and convert magnetic field data to milliTesla
 */
HAL_StatusTypeDef TMAG5273_ReadData(TMAG5273_Data_t* data)
{
    HAL_StatusTypeDef status = TMAG5273_ReadRaw(data);
    
    if(status == HAL_OK) {
        // Convert raw values to milliTesla
        data->x_mT = TMAG5273_ConvertToMilliTesla(data->x_raw, xy_range_mT);
        data->y_mT = TMAG5273_ConvertToMilliTesla(data->y_raw, xy_range_mT);
        data->z_mT = TMAG5273_ConvertToMilliTesla(data->z_raw, z_range_mT);
        
        // Convert raw temperature to Celsius
        data->temp_C = TMAG5273_ConvertToTemperature(data->temp_raw);
    }
    
    return status;
}

/**
 * @brief Read device ID register
 */
HAL_StatusTypeDef TMAG5273_ReadDeviceID(uint8_t* id)
{
    if(tmag_i2c == NULL || id == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1,
                           TMAG5273_REG_DEVICE_ID, I2C_MEMADD_SIZE_8BIT,
                           id, 1, TMAG5273_I2C_TIMEOUT);
}

/**
 * @brief Check if new data is available
 */
bool TMAG5273_IsDataReady(void)
{
    if(tmag_i2c == NULL) return false;
    
    uint8_t status_reg = 0;
    HAL_StatusTypeDef result;
    
    result = HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1,
                             TMAG5273_REG_CONV_STATUS, I2C_MEMADD_SIZE_8BIT,
                             &status_reg, 1, TMAG5273_I2C_TIMEOUT);
    
    if(result != HAL_OK) return false;
    
    // Check bit 0 (Data Ready flag)
    return (status_reg & 0x01) != 0;
}

/**
 * @brief Write to a single register
 */
HAL_StatusTypeDef TMAG5273_WriteRegister(uint8_t reg, uint8_t value)
{
    if(tmag_i2c == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Write(tmag_i2c, TMAG5273_I2C_ADDR << 1,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1, TMAG5273_I2C_TIMEOUT);
}

/**
 * @brief Read from a single register
 */
HAL_StatusTypeDef TMAG5273_ReadRegister(uint8_t reg, uint8_t* value)
{
    if(tmag_i2c == NULL || value == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Read(tmag_i2c, TMAG5273_I2C_ADDR << 1,
                           reg, I2C_MEMADD_SIZE_8BIT,
                           value, 1, TMAG5273_I2C_TIMEOUT);
}

//****************************************************************************
// Private Functions
//****************************************************************************

/**
 * @brief Convert raw magnetic field value to milliTesla
 * @param raw_value 16-bit signed raw value from sensor
 * @param range_mT Full-scale range in milliTesla (40 or 80)
 * @return Magnetic field strength in milliTesla
 */
static float TMAG5273_ConvertToMilliTesla(int16_t raw_value, uint16_t range_mT)
{
    // TMAG5273 uses 16-bit signed values
    // For ±40mT range: LSB = 40mT / 32768 = 1.22 µT/LSB
    // For ±80mT range: LSB = 80mT / 32768 = 2.44 µT/LSB
    float lsb_per_mT = (float)range_mT / 32768.0f;
    return (float)raw_value * lsb_per_mT;
}

/**
 * @brief Convert raw temperature value to degrees Celsius
 * @param temp_raw 16-bit signed raw temperature value
 * @return Temperature in degrees Celsius
 */
static float TMAG5273_ConvertToTemperature(int16_t temp_raw)
{
    // From TMAG5273 datasheet:
    // T_SENS_T0 = 25°C (reference temperature)
    // T_ADC_T0 = 17508 (ADC value at 25°C, typical)
    // T_ADC_RES = 60.1 LSB/°C (typical)
    const float T_SENS_T0 = 25.0f;
    const float T_ADC_T0 = 17508.0f;
    const float T_ADC_RES = 60.1f;
    
    return T_SENS_T0 + ((float)temp_raw - T_ADC_T0) / T_ADC_RES;
}
