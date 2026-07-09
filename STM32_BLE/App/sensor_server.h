/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SENSOR_Server.h
  * @author  MCD Application Team
  * @brief   Header for SENSOR_Server.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_SERVER_H
#define SENSOR_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ble_status.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  SENSOR_SERVER_TEMP_CHAR,
  SENSOR_SERVER_HUM_CHAR,
  SENSOR_SERVER_PRS_CHAR,
  SENSOR_SERVER_AQI_CHAR,

  /* USER CODE BEGIN Service1_CharOpcode_t */
  SENSOR_SERVER_GNSS_CHAR,
  SENSOR_SERVER_ACCEL_CHAR,
  SENSOR_SERVER_GYRO_CHAR,
  SENSOR_SERVER_MAG_CHAR,
  /* USER CODE END Service1_CharOpcode_t */

  SENSOR_SERVER_CHAROPCODE_LAST
} SENSOR_SERVER_CharOpcode_t;

typedef enum
{
  SENSOR_SERVER_TEMP_CHAR_READ_EVT,
  SENSOR_SERVER_TEMP_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_TEMP_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_HUM_CHAR_READ_EVT,
  SENSOR_SERVER_HUM_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_HUM_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_PRS_CHAR_READ_EVT,
  SENSOR_SERVER_PRS_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_PRS_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_AQI_CHAR_READ_EVT,
  SENSOR_SERVER_AQI_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_AQI_CHAR_NOTIFY_DISABLED_EVT,

  /* USER CODE BEGIN Service1_OpcodeEvt_t */
  SENSOR_SERVER_GNSS_CHAR_READ_EVT,
  SENSOR_SERVER_GNSS_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_GNSS_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_ACCEL_CHAR_READ_EVT,
  SENSOR_SERVER_ACCEL_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_ACCEL_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_GYRO_CHAR_READ_EVT,
  SENSOR_SERVER_GYRO_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_GYRO_CHAR_NOTIFY_DISABLED_EVT,
  SENSOR_SERVER_MAG_CHAR_READ_EVT,
  SENSOR_SERVER_MAG_CHAR_NOTIFY_ENABLED_EVT,
  SENSOR_SERVER_MAG_CHAR_NOTIFY_DISABLED_EVT,
  /* USER CODE END Service1_OpcodeEvt_t */

  SENSOR_SERVER_BOOT_REQUEST_EVT
} SENSOR_SERVER_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service1_Data_t */

  /* USER CODE END Service1_Data_t */

} SENSOR_SERVER_Data_t;

typedef struct
{
  SENSOR_SERVER_OpcodeEvt_t       EvtOpcode;
  SENSOR_SERVER_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;

  /* USER CODE BEGIN Service1_NotificationEvt_t */

  /* USER CODE END Service1_NotificationEvt_t */

} SENSOR_SERVER_NotificationEvt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SENSOR_SERVER_Init(void);
void SENSOR_SERVER_Notification(SENSOR_SERVER_NotificationEvt_t *p_Notification);
tBleStatus SENSOR_SERVER_UpdateValue(SENSOR_SERVER_CharOpcode_t CharOpcode, SENSOR_SERVER_Data_t *pData);
tBleStatus SENSOR_SERVER_NotifyValue(SENSOR_SERVER_CharOpcode_t CharOpcode, SENSOR_SERVER_Data_t *pData, uint16_t ConnectionHandle);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*SENSOR_SERVER_H */
