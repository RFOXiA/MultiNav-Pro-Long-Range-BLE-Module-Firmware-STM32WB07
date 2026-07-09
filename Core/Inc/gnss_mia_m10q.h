/*
 * gnss_mia_m10q.h
 *
 *  Created on: December 22, 2025
 *  Author: RFOXiA AI Assistant
 *  
 *  MIA-M10Q GNSS Module Driver (I2C Interface)
 *  u-blox M10 GNSS receiver with concurrent GNSS support
 */

#ifndef INC_GNSS_MIA_M10Q_H_
#define INC_GNSS_MIA_M10Q_H_

#include "main.h"
#include "i2c.h"

/* MIA-M10Q I2C Address */
#define GNSS_I2C_ADDR          0x42  // 7-bit address (0x84 >> 1)

/* MIA-M10Q Register Addresses */
#define GNSS_REG_DATA_LENGTH_HIGH  0xFD  // Number of bytes available (MSB)
#define GNSS_REG_DATA_LENGTH_LOW   0xFE  // Number of bytes available (LSB)
#define GNSS_REG_DATA_STREAM       0xFF  // Data stream register

/* UBX Protocol Constants */
#define UBX_SYNC_CHAR_1        0xB5
#define UBX_SYNC_CHAR_2        0x62

/* UBX Message Classes */
#define UBX_CLASS_NAV          0x01  // Navigation Results
#define UBX_CLASS_CFG          0x06  // Configuration Input Messages

/* UBX Navigation Message IDs */
#define UBX_NAV_PVT            0x07  // Position Velocity Time Solution

/* UBX Configuration Message IDs */
#define UBX_CFG_RST            0x04  // Reset Receiver
#define UBX_CFG_RATE           0x08  // Navigation/Measurement Rate Settings
#define UBX_CFG_PRT            0x00  // Port Configuration
#define UBX_CFG_VALSET         0x8A  // Set configuration items (M10 generation)

/* GPS Fix Types */
#define GPS_FIX_NO_FIX         0
#define GPS_FIX_DEAD_RECKONING 1
#define GPS_FIX_2D             2
#define GPS_FIX_3D             3
#define GPS_FIX_GNSS_DEAD_RECKONING 4
#define GPS_FIX_TIME_ONLY      5

/* GNSS Data Structure */
typedef struct {
    uint8_t  fix_type;        // GPS fix type (0=no fix, 2=2D, 3=3D)
    uint8_t  num_satellites;  // Number of satellites used
    int32_t  latitude;        // Latitude in degrees * 1e-7
    int32_t  longitude;       // Longitude in degrees * 1e-7
    int32_t  altitude_msl;    // Altitude above MSL in mm
    uint32_t horizontal_acc;  // Horizontal accuracy in mm
    uint32_t vertical_acc;    // Vertical accuracy in mm
    int32_t  ground_speed;    // Ground speed in mm/s
    int32_t  heading;         // Heading of motion in degrees * 1e-5
    uint16_t year;            // UTC year
    uint8_t  month;           // UTC month (1-12)
    uint8_t  day;             // UTC day (1-31)
    uint8_t  hour;            // UTC hour (0-23)
    uint8_t  minute;          // UTC minute (0-59)
    uint8_t  second;          // UTC second (0-59)
    uint8_t  valid_flags;     // Validity flags
} GNSS_Data_t;

/* Function Prototypes */

/**
 * @brief Initialize MIA-M10Q GNSS module via I2C
 * @param i2ch Pointer to I2C handle
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef GNSS_Init(I2C_HandleTypeDef* i2ch);

/**
 * @brief Read GNSS position, velocity, time data
 * @param gnss_data Pointer to GNSS_Data_t structure to store results
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef GNSS_ReadPVT(GNSS_Data_t* gnss_data);

/**
 * @brief Start non-blocking DMA read for GNSS PVT data
 * @note Sends UBX-NAV-PVT poll and starts DMA to receive response
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef GNSS_StartDMARead(void);

/**
 * @brief Check if GNSS DMA read is complete and get parsed data
 * @param gnss_data Pointer to GNSS_Data_t structure to fill
 * @retval HAL_OK if data ready, HAL_BUSY if still reading, HAL_ERROR on error
 */
HAL_StatusTypeDef GNSS_GetDMAResult(GNSS_Data_t* gnss_data);

/**
 * @brief Check if GNSS DMA is currently busy
 * @retval 1 if busy (transfer in progress), 0 if idle
 */
uint8_t GNSS_IsBusy(void);

/**
 * @brief DMA RX Complete Callback for GNSS
 */
void GNSS_DMA_RxCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief DMA Error Callback for GNSS
 */
void GNSS_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief Check if GNSS module is responding
 * @retval HAL_StatusTypeDef HAL_OK if device is ready
 */
HAL_StatusTypeDef GNSS_IsReady(void);

/**
 * @brief Disable NMEA protocol output on the I2C (DDC) port (UBX-CFG-VALSET)
 * @retval HAL_StatusTypeDef HAL_OK if command sent successfully
 */
HAL_StatusTypeDef GNSS_DisableNMEAOnI2C(void);

/**
 * @brief Drain any pending bytes from the module's I2C stream buffer
 */
void GNSS_FlushStream(void);

/**
 * @brief Perform hardware reset of GNSS module
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef GNSS_HardwareReset(void);

/**
 * @brief Configure GNSS update rate
 * @param rate_ms Update rate in milliseconds (default 1000ms = 1Hz)
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef GNSS_SetUpdateRate(uint16_t rate_ms);

/**
 * @brief Get number of available bytes in GNSS buffer
 * @param num_bytes Pointer to store number of available bytes
 * @retval HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef GNSS_GetAvailableBytes(uint16_t* num_bytes);

#endif /* INC_GNSS_MIA_M10Q_H_ */
