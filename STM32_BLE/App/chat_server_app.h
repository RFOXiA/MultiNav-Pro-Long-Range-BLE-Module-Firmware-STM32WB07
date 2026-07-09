/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chat_server_app.h
  * @author  RFOXiA Team
  * @brief   Header for chat_server_app.c - Application layer for chat service
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
#ifndef CHAT_SERVER_APP_H
#define CHAT_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CHAT_SERVER_APP_Init(void);
void CHAT_SERVER_APP_EvtRx(void *p_Payload);

#ifdef __cplusplus
}
#endif

#endif /* CHAT_SERVER_APP_H */
