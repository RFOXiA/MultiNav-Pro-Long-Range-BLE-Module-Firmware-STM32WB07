/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MOTOR_Server.h
  * @author  MCD Application Team
  * @brief   Header for MOTOR_Server.c
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
#ifndef MOTOR_SERVER_H
#define MOTOR_SERVER_H

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
  MOTOR_SERVER_MCMD_CHAR,

  /* USER CODE BEGIN Service2_CharOpcode_t */

  /* USER CODE END Service2_CharOpcode_t */

  MOTOR_SERVER_CHAROPCODE_LAST
} MOTOR_SERVER_CharOpcode_t;

typedef enum
{
  MOTOR_SERVER_MCMD_CHAR_READ_EVT,
  MOTOR_SERVER_MCMD_CHAR_WRITE_NO_RESP_EVT,

  /* USER CODE BEGIN Service2_OpcodeEvt_t */

  /* USER CODE END Service2_OpcodeEvt_t */

  MOTOR_SERVER_BOOT_REQUEST_EVT
} MOTOR_SERVER_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service2_Data_t */

  /* USER CODE END Service2_Data_t */

} MOTOR_SERVER_Data_t;

typedef struct
{
  MOTOR_SERVER_OpcodeEvt_t       EvtOpcode;
  MOTOR_SERVER_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;

  /* USER CODE BEGIN Service2_NotificationEvt_t */

  /* USER CODE END Service2_NotificationEvt_t */

} MOTOR_SERVER_NotificationEvt_t;

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
void MOTOR_SERVER_Init(void);
void MOTOR_SERVER_Notification(MOTOR_SERVER_NotificationEvt_t *p_Notification);
tBleStatus MOTOR_SERVER_UpdateValue(MOTOR_SERVER_CharOpcode_t CharOpcode, MOTOR_SERVER_Data_t *pData);
tBleStatus MOTOR_SERVER_NotifyValue(MOTOR_SERVER_CharOpcode_t CharOpcode, MOTOR_SERVER_Data_t *pData, uint16_t ConnectionHandle);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*MOTOR_SERVER_H */
