/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SENSOR_Server_app.h
  * @author  MCD Application Team
  * @brief   Header for SENSOR_Server_app.c
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
#ifndef SENSOR_SERVER_APP_H
#define SENSOR_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  SENSOR_SERVER_CENTR_CONN_HANDLE_EVT,
  SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT,
  SENSOR_SERVER_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service1_OpcodeNotificationEvt_t */

  /* USER CODE END Service1_OpcodeNotificationEvt_t */

  SENSOR_SERVER_LAST_EVT,
} SENSOR_SERVER_APP_OpcodeNotificationEvt_t;

typedef struct
{
  SENSOR_SERVER_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN SENSOR_SERVER_APP_ConnHandleNotEvt_t */

  /* USER CODE END SENSOR_SERVER_APP_ConnHandleNotEvt_t */
} SENSOR_SERVER_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Sensor types for availability tracking */
#define SENSOR_TYPE_TEMP     0
#define SENSOR_TYPE_HUM      1
#define SENSOR_TYPE_PRS      2
#define SENSOR_TYPE_AQI      3
#define SENSOR_TYPE_GNSS     4

/* Sensor availability structure (exported for gatt_client_app.c) */
typedef struct {
    uint8_t temp_available;
    uint8_t humidity_available;
    uint8_t pressure_available;
    uint8_t aqi_available;
    uint8_t gnss_available;
} SensorAvailability_t;

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SENSOR_SERVER_APP_Init(void);
void SENSOR_SERVER_APP_EvtRx(SENSOR_SERVER_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EF */
void SENSOR_SERVER_APP_ClearSubscriptionFlags(void);
void SENSOR_SERVER_Aqi_char_SendNotification(void);
void SENSOR_SERVER_SetSensorAvailability(uint8_t sensor_type, uint8_t available);
SensorAvailability_t* SENSOR_SERVER_GetSensorStatus(void);
void SENSOR_SERVER_RelayTempFromPeer(uint8_t *p_Payload);
void SENSOR_SERVER_RelayHumFromPeer(uint8_t *p_Payload);
void SENSOR_SERVER_RelayPrsFromPeer(uint8_t *p_Payload);
void SENSOR_SERVER_RelayAqiFromPeer(uint8_t *p_Payload, uint8_t length);
void SENSOR_SERVER_RelayGnssFromPeer(uint8_t *p_Payload, uint8_t length);
void SENSOR_SERVER_RelayAccelFromPeer(uint8_t *p_Payload);  /* Relay accelerometer from peer */
void SENSOR_SERVER_RelayGyroFromPeer(uint8_t *p_Payload);   /* Relay gyroscope from peer */
void SENSOR_SERVER_RelayMagFromPeer(uint8_t *p_Payload);    /* Relay magnetometer from peer */
void SENSOR_SERVER_SetPeerHandle(uint16_t peer_handle);  /* Set peer connection handle */
void SENSOR_SERVER_APP_Handle_ViewChange(uint8_t new_view);  /* Start/stop TIM1 based on view */

/* Additional sensor notification functions (called directly from main loop) */
void SEND_GNSS_Notification(void);  /* Send GNSS data notification */
void SEND_MVH_Notifications(void);  /* Send Temperature + Humidity notifications */
void SENSOR_SERVER_Temp_char_SendNotification(void);  /* Send Temperature notification */
void SENSOR_SERVER_Hum_char_SendNotification(void);   /* Send Humidity notification */
void SENSOR_SERVER_Prs_char_SendNotification(void);   /* Send Pressure notification */

/* CONNECTION STABILITY FIXES */
void SENSOR_SERVER_ProcessKeepalive(void);        /* Call from main loop to prevent timeout */
uint8_t SENSOR_SERVER_CheckRateLimit(void);       /* Check if notification rate limit allows sending */
void SENSOR_SERVER_SetKeepaliveEnabled(uint8_t enabled);  /* Enable/disable keepalive */
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*SENSOR_SERVER_APP_H */
