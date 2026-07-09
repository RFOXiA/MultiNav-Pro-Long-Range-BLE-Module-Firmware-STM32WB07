/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MOTOR_Server_app.h
  * @author  MCD Application Team
  * @brief   Header for MOTOR_Server_app.c
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
#ifndef MOTOR_SERVER_APP_H
#define MOTOR_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MOTOR_SERVER_CENTR_CONN_HANDLE_EVT,
  MOTOR_SERVER_PERIPH_CONN_HANDLE_EVT,
  MOTOR_SERVER_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service2_OpcodeNotificationEvt_t */

  /* USER CODE END Service2_OpcodeNotificationEvt_t */

  MOTOR_SERVER_LAST_EVT,
} MOTOR_SERVER_APP_OpcodeNotificationEvt_t;

typedef struct
{
  MOTOR_SERVER_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN MOTOR_SERVER_APP_ConnHandleNotEvt_t */

  /* USER CODE END MOTOR_SERVER_APP_ConnHandleNotEvt_t */
} MOTOR_SERVER_APP_ConnHandleNotEvt_t;
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
void MOTOR_SERVER_APP_Init(void);
void MOTOR_SERVER_APP_EvtRx(MOTOR_SERVER_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*MOTOR_SERVER_APP_H */
