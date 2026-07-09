/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BConnection_Server.h
  * @author  MCD Application Team
  * @brief   Header for BConnection_Server.c
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
#ifndef BCONNECTION_SERVER_H
#define BCONNECTION_SERVER_H

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
  BCONNECTION_SERVER_BLIST_STATE_CHAR,
  BCONNECTION_SERVER_BLIST_CHAR,

  /* USER CODE BEGIN Service3_CharOpcode_t */

  /* USER CODE END Service3_CharOpcode_t */

  BCONNECTION_SERVER_CHAROPCODE_LAST
} BCONNECTION_SERVER_CharOpcode_t;

typedef enum
{
  BCONNECTION_SERVER_BLIST_STATE_CHAR_READ_EVT,
  BCONNECTION_SERVER_BLIST_STATE_CHAR_WRITE_NO_RESP_EVT,
  BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_ENABLED_EVT,
  BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_DISABLED_EVT,
  BCONNECTION_SERVER_BLIST_CHAR_READ_EVT,
  BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_ENABLED_EVT,
  BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_DISABLED_EVT,

  /* USER CODE BEGIN Service3_OpcodeEvt_t */

  /* USER CODE END Service3_OpcodeEvt_t */

  BCONNECTION_SERVER_BOOT_REQUEST_EVT
} BCONNECTION_SERVER_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service3_Data_t */

  /* USER CODE END Service3_Data_t */

} BCONNECTION_SERVER_Data_t;

typedef struct
{
  BCONNECTION_SERVER_OpcodeEvt_t       EvtOpcode;
  BCONNECTION_SERVER_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;

  /* USER CODE BEGIN Service3_NotificationEvt_t */

  /* USER CODE END Service3_NotificationEvt_t */

} BCONNECTION_SERVER_NotificationEvt_t;

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
void BCONNECTION_SERVER_Init(void);
void BCONNECTION_SERVER_Notification(BCONNECTION_SERVER_NotificationEvt_t *p_Notification);
tBleStatus BCONNECTION_SERVER_UpdateValue(BCONNECTION_SERVER_CharOpcode_t CharOpcode, BCONNECTION_SERVER_Data_t *pData);
tBleStatus BCONNECTION_SERVER_NotifyValue(BCONNECTION_SERVER_CharOpcode_t CharOpcode, BCONNECTION_SERVER_Data_t *pData, uint16_t ConnectionHandle);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*BCONNECTION_SERVER_H */
