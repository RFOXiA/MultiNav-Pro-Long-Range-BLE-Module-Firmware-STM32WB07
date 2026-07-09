/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MOTOR_Server_app.c
  * @author  MCD Application Team
  * @brief   MOTOR_Server_app application definition.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "ble.h"
#include "motor_server_app.h"
#include "motor_server.h"
#include "sensor_server_app.h"  /* For SENSOR_SERVER_APP_Handle_ViewChange() */
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  /* USER CODE BEGIN Service2_APP_SendInformation_t */

  /* USER CODE END Service2_APP_SendInformation_t */
  MOTOR_SERVER_APP_SENDINFORMATION_LAST
} MOTOR_SERVER_APP_SendInformation_t;

typedef struct
{
  uint16_t MOTOR_SERVER_periph_connHdl;
  /* USER CODE BEGIN Service2_APP_Context_t */

  /* USER CODE END Service2_APP_Context_t */
  uint16_t              ConnectionHandle;
} MOTOR_SERVER_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern uint8_t motor_cmd[5];
/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static MOTOR_SERVER_APP_Context_t MOTOR_SERVER_APP_Context;

uint8_t a_MOTOR_SERVER_UpdateCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void MOTOR_SERVER_Notification(MOTOR_SERVER_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_Notification_1 */

  /* USER CODE END Service2_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_Notification_Service2_EvtOpcode */

    /* USER CODE END Service2_Notification_Service2_EvtOpcode */

    case MOTOR_SERVER_MCMD_CHAR_READ_EVT:
      /* USER CODE BEGIN Service2Char1_READ_EVT */

      /* USER CODE END Service2Char1_READ_EVT */
      break;

    case MOTOR_SERVER_MCMD_CHAR_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN Service2Char1_WRITE_NO_RESP_EVT */
    	// Check if this is a view command (first byte = 'v' = 0x76)
    	if (p_Notification->DataTransfered.Length >= 2 && 
    	    p_Notification->DataTransfered.p_Payload[0] == 'v')
    	{
    		// View control command received via MOTOR_SERVICE
    		// View 0 = Map (environmental), View 1 = 3D (motion), View 2 = Cam
    		uint8_t new_view = p_Notification->DataTransfered.p_Payload[1];
    		
    		if (new_view <= 2) {  /* Valid views: 0, 1, 2 */
    			SENSOR_SERVER_APP_Handle_ViewChange(new_view);  /* This updates g_current_view and starts/stops TIM1 */
    			APP_DBG_MSG("📱 VIEW via MOTOR: %d (%s)\n", new_view,
    			           new_view == 0 ? "Map" : (new_view == 1 ? "3D" : "Cam"));
    		} else {
    			APP_DBG_MSG("⚠️ Invalid view value via MOTOR: %d\n", new_view);
    		}
    	}
    	else
    	{
    		// Regular motor/button command - store in buffer
    		for (uint8_t i = 0; i < p_Notification->DataTransfered.Length; i++)
    		{
    			if(i>4)
    			{
    				printf("index exceed limit");
    			}
    			else
    			{
    				motor_cmd[i]=p_Notification->DataTransfered.p_Payload[i];
    			}
    		}
    		UTIL_SEQ_SetTask(1U << TAKS_MOTOR_CMD,
    		CFG_SEQ_PRIO_0);
    	}
      /* USER CODE END Service2Char1_WRITE_NO_RESP_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_Notification_default */

      /* USER CODE END Service2_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service2_Notification_2 */

  /* USER CODE END Service2_Notification_2 */
  return;
}

void MOTOR_SERVER_APP_EvtRx(MOTOR_SERVER_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_APP_EvtRx_1 */

  /* USER CODE END Service2_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_APP_EvtRx_Service2_EvtOpcode */

    /* USER CODE END Service2_APP_EvtRx_Service2_EvtOpcode */
    case MOTOR_SERVER_CENTR_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_CENTR_CONN_HANDLE_EVT */

      /* USER CODE END Service2_APP_CENTR_CONN_HANDLE_EVT */
      break;

    case MOTOR_SERVER_PERIPH_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_PERIPH_CONN_HANDLE_EVT */

      /* USER CODE END Service2_APP_PERIPH_CONN_HANDLE_EVT */
      break;

    case MOTOR_SERVER_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_DISCON_HANDLE_EVT */

      /* USER CODE END Service2_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_APP_EvtRx_default */

      /* USER CODE END Service2_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service2_APP_EvtRx_2 */

  /* USER CODE END Service2_APP_EvtRx_2 */

  return;
}

void MOTOR_SERVER_APP_Init(void)
{
  MOTOR_SERVER_Init();

  /* USER CODE BEGIN Service2_APP_Init */
  /* USER CODE END Service2_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
