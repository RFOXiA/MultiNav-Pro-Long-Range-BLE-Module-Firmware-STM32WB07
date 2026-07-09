/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chat_server.h
  * @author  RFOXiA Team
  * @brief   Header for chat_server.c - Phone-to-Phone messaging service
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
#ifndef CHAT_SERVER_H
#define CHAT_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ble_status.h"

/* Exported defines ----------------------------------------------------------*/
/* Maximum message size for phone-to-phone chat */
#define CHAT_MESSAGE_MAX_SIZE  50

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  CHAT_SERVER_WRITE_CHAR,    /* Write characteristic (receive from phone) */
  CHAT_SERVER_NOTIFY_CHAR,   /* Notify characteristic (send to phone) */
  CHAT_SERVER_CHAROPCODE_LAST
} CHAT_SERVER_CharOpcode_t;

typedef enum
{
  CHAT_SERVER_WRITE_CHAR_WRITE_NO_RESP_EVT,
  CHAT_SERVER_NOTIFY_CHAR_NOTIFY_ENABLED_EVT,
  CHAT_SERVER_NOTIFY_CHAR_NOTIFY_DISABLED_EVT,
  CHAT_SERVER_BOOT_REQUEST_EVT
} CHAT_SERVER_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;
} CHAT_SERVER_Data_t;

typedef struct
{
  CHAT_SERVER_OpcodeEvt_t  EvtOpcode;
  CHAT_SERVER_Data_t       DataTransfered;
  uint16_t                 ConnectionHandle;
  uint16_t                 AttributeHandle;
  uint8_t                  ServiceInstance;
} CHAT_SERVER_NotificationEvt_t;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CHAT_SERVER_Init(void);
void CHAT_SERVER_Notification(CHAT_SERVER_NotificationEvt_t *p_Notification);
tBleStatus CHAT_SERVER_UpdateValue(CHAT_SERVER_CharOpcode_t CharOpcode, CHAT_SERVER_Data_t *pData);
tBleStatus CHAT_SERVER_NotifyValue(CHAT_SERVER_CharOpcode_t CharOpcode, CHAT_SERVER_Data_t *pData, uint16_t ConnectionHandle);
uint16_t CHAT_SERVER_GetWriteCharValueHandle(void);
uint16_t CHAT_SERVER_GetNotifyCharValueHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* CHAT_SERVER_H */
