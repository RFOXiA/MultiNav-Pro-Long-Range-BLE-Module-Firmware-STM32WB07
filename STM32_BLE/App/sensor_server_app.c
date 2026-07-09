/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SENSOR_Server_app.c
  * @author  MCD Application Team
  * @brief   SENSOR_Server_app application definition.
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
#include "sensor_server_app.h"
#include "sensor_server.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint8_t G_aqi[11];  // Declared in main.c - AQI data from ZMOD4510
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Temp_char_NOTIFICATION_OFF,
  Temp_char_NOTIFICATION_ON,
  Hum_char_NOTIFICATION_OFF,
  Hum_char_NOTIFICATION_ON,
  Prs_char_NOTIFICATION_OFF,
  Prs_char_NOTIFICATION_ON,
  Aqi_char_NOTIFICATION_OFF,
  Aqi_char_NOTIFICATION_ON,
  Accel_char_NOTIFICATION_OFF,
  Accel_char_NOTIFICATION_ON,
  Gyro_char_NOTIFICATION_OFF,
  Gyro_char_NOTIFICATION_ON,
  Mag_char_NOTIFICATION_OFF,
  Mag_char_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */
  Gnss_char_NOTIFICATION_OFF,
  Gnss_char_NOTIFICATION_ON,
  /* USER CODE END Service1_APP_SendInformation_t */
  SENSOR_SERVER_APP_SENDINFORMATION_LAST
} SENSOR_SERVER_APP_SendInformation_t;

typedef struct
{
  SENSOR_SERVER_APP_SendInformation_t     Temp_char_Notification_Status;
  SENSOR_SERVER_APP_SendInformation_t     Hum_char_Notification_Status;
  SENSOR_SERVER_APP_SendInformation_t     Prs_char_Notification_Status;
  SENSOR_SERVER_APP_SendInformation_t     Aqi_char_Notification_Status;
  SENSOR_SERVER_APP_SendInformation_t     Accel_char_Notification_Status;  /* Motion sensors */
  SENSOR_SERVER_APP_SendInformation_t     Gyro_char_Notification_Status;
  SENSOR_SERVER_APP_SendInformation_t     Mag_char_Notification_Status;
  uint16_t SENSOR_SERVER_periph_connHdl;   /* Phone connection handle */
  uint16_t SENSOR_SERVER_peer_connHdl;      /* Peer connection handle (Module-to-Module) */
  /* USER CODE BEGIN Service1_APP_Context_t */
  SENSOR_SERVER_APP_SendInformation_t     Gnss_char_Notification_Status;  /* GNSS phone subscription */
  /* USER CODE END Service1_APP_Context_t */
  uint16_t              ConnectionHandle;
} SENSOR_SERVER_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG 1  // Enabled temporarily to debug subscription issues
#define STREAM 0

/* CONNECTION STABILITY FIXES */
#define KEEPALIVE_INTERVAL_MS    4000  /* Send keepalive every 4 seconds (within 5s timeout) */
#define NOTIFICATION_RATE_LIMIT  15    /* Max 15 notifications per second to prevent buffer overflow */
#define RATE_LIMIT_WINDOW_MS     1000  /* 1 second window for rate limiting */
#define NOTIFICATION_THROTTLE_MS 3000  /* 3 second throttle after connection to allow BLE stack to settle (CRITICAL for dual-connection) */
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern uint8_t G_temp[2],G_hum[2],G_prs[3];
// ZMOD4510 disabled to save flash
// extern uint8_t G_aqi[11];
/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static SENSOR_SERVER_APP_Context_t SENSOR_SERVER_APP_Context;

/* USER CODE BEGIN Connection Stability Variables */
/* Keepalive mechanism to prevent 5-second timeout */
static uint32_t last_keepalive_time = 0;
static uint8_t keepalive_enabled = 1;

/* Notification rate limiting to prevent 0x0C buffer exhaustion */
static uint32_t notification_count = 0;
static uint32_t rate_limit_window_start = 0;
/* USER CODE END Connection Stability Variables */

uint8_t a_SENSOR_SERVER_UpdateCharData[247];

/* USER CODE BEGIN PV */
/* CONNECTION STABILITY: Track phone connection time for notification throttling */
static uint32_t g_phone_connection_timestamp = 0;
#define NOTIFICATION_THROTTLE_MS 200  /* Minimal 200ms throttle for BLE stack stability */

/* CONNECTION STABILITY: Track peer connection time for notification throttling */
static uint32_t g_peer_connection_timestamp = 0;

/* Global flag to pause sensor notifications during peer connection */
volatile uint8_t g_pause_sensor_notifications = 0;

/* View-based sensor control
 * View 0 = Map (environmental sensors @ 1Hz, motion blocked)
 * View 1 = 3D (motion sensors @ 10Hz, environmental blocked)
 * View 2 = Cam
 * Default: Map view (environmental sensors active)
 */
volatile uint8_t g_current_view = 0;  /* Start in Map view */

/* ===== SMART SENSOR RELAY SYSTEM =====
 * Priority: Local sensors > Peer sensors
 * Only subscribe to peer for sensors that are missing/broken locally
 */

static SensorAvailability_t g_sensor_status = {0};

/* Peer sensor data buffers (used when local sensor missing) */
static uint8_t g_peer_temp[2] = {0};
static uint8_t g_peer_hum[2] = {0};
static uint8_t g_peer_prs[3] = {0};
static uint8_t g_peer_aqi[11] = {0};
static uint8_t g_peer_gnss[20] = {0};

/* Flags to track if peer data is valid */
static uint8_t g_peer_temp_valid = 0;
static uint8_t g_peer_hum_valid = 0;
static uint8_t g_peer_prs_valid = 0;
static uint8_t g_peer_aqi_valid = 0;
static uint8_t g_peer_gnss_valid = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Note: Functions below are now public (exported in header) for direct calls from main loop */
void SENSOR_SERVER_Temp_char_SendNotification(void);
void SENSOR_SERVER_Hum_char_SendNotification(void);
void SENSOR_SERVER_Prs_char_SendNotification(void);

/* USER CODE BEGIN PFP */
void SEND_MVH_Notifications(void);
void SEND_GNSS_Notification(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void SENSOR_SERVER_Notification(SENSOR_SERVER_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

    case SENSOR_SERVER_TEMP_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char3_READ_EVT */

      /* USER CODE END Service1Char3_READ_EVT */
      break;

    case SENSOR_SERVER_TEMP_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char3_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Temp_char_Notification_Status = Temp_char_NOTIFICATION_ON;
#if (DEBUG == 1)
    	      printf(">>Sensor_app Temp_char: NOTIFICATION ENABLED\n\r");
#endif
      /* USER CODE END Service1Char3_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_TEMP_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char3_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Temp_char_Notification_Status = Temp_char_NOTIFICATION_OFF;
#if (DEBUG == 1)
			  printf(">>Sensor_app Temp_char: NOTIFICATION DISABLED\n\r");
#endif
      /* USER CODE END Service1Char3_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_HUM_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char4_READ_EVT */

      /* USER CODE END Service1Char4_READ_EVT */
      break;

    case SENSOR_SERVER_HUM_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char4_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Hum_char_Notification_Status = Hum_char_NOTIFICATION_ON;
#if (DEBUG == 1)
			  printf(">>Sensor_app Hum_char: NOTIFICATION ENABLED\n\r");
#endif

      /* USER CODE END Service1Char4_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_HUM_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char4_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Hum_char_Notification_Status = Hum_char_NOTIFICATION_OFF;
#if (DEBUG == 1)
			  printf(">>Sensor_app Hum_char: NOTIFICATION DISABLED\n\r");
#endif
      /* USER CODE END Service1Char4_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_PRS_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char5_READ_EVT */

      /* USER CODE END Service1Char5_READ_EVT */
      break;

    case SENSOR_SERVER_PRS_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char5_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Prs_char_Notification_Status = Prs_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    				  printf(">>Sensor_app PRS_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char5_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_PRS_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char5_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Prs_char_Notification_Status = Prs_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    				  printf(">>Sensor_app PRS_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char5_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_AQI_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char6_READ_EVT */

      /* USER CODE END Service1Char6_READ_EVT */
      break;

    case SENSOR_SERVER_AQI_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char6_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Aqi_char_Notification_Status = Aqi_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    				  printf(">>Sensor_app AQI_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char6_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_AQI_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char6_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Aqi_char_Notification_Status = Aqi_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    				  printf(">>Sensor_app AQI_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char6_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_GNSS_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char7_READ_EVT */

      /* USER CODE END Service1Char7_READ_EVT */
      break;

    case SENSOR_SERVER_GNSS_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char7_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Gnss_char_Notification_Status = Gnss_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app GNSS_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char7_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_GNSS_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char7_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Gnss_char_Notification_Status = Gnss_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app GNSS_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char7_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_ACCEL_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char8_READ_EVT */

      /* USER CODE END Service1Char8_READ_EVT */
      break;

    case SENSOR_SERVER_ACCEL_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char8_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Accel_char_Notification_Status = Accel_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app ACCEL_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char8_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_ACCEL_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char8_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Accel_char_Notification_Status = Accel_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app ACCEL_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char8_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_GYRO_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char9_READ_EVT */

      /* USER CODE END Service1Char9_READ_EVT */
      break;

    case SENSOR_SERVER_GYRO_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char9_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Gyro_char_Notification_Status = Gyro_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app GYRO_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char9_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_GYRO_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char9_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Gyro_char_Notification_Status = Gyro_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app GYRO_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char9_NOTIFY_DISABLED_EVT */
      break;

    case SENSOR_SERVER_MAG_CHAR_READ_EVT:
      /* USER CODE BEGIN Service1Char10_READ_EVT */

      /* USER CODE END Service1Char10_READ_EVT */
      break;

    case SENSOR_SERVER_MAG_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char10_NOTIFY_ENABLED_EVT */
    	SENSOR_SERVER_APP_Context.Mag_char_Notification_Status = Mag_char_NOTIFICATION_ON;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app MAG_char: NOTIFICATION ENABLED\n\r");
    	#endif
      /* USER CODE END Service1Char10_NOTIFY_ENABLED_EVT */
      break;

    case SENSOR_SERVER_MAG_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char10_NOTIFY_DISABLED_EVT */
    	SENSOR_SERVER_APP_Context.Mag_char_Notification_Status = Mag_char_NOTIFICATION_OFF;
    	#if (DEBUG == 1)
    		printf(">>Sensor_app MAG_char: NOTIFICATION DISABLED\n\r");
    	#endif
      /* USER CODE END Service1Char10_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_Notification_default */

      /* USER CODE END Service1_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void SENSOR_SERVER_APP_EvtRx(SENSOR_SERVER_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

    /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
    case SENSOR_SERVER_CENTR_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_CENTR_CONN_HANDLE_EVT */

      /* USER CODE END Service1_APP_CENTR_CONN_HANDLE_EVT */
      break;

    case SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_PERIPH_CONN_HANDLE_EVT */
        SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl = p_Notification->ConnectionHandle;
        /* CONNECTION STABILITY: Record connection timestamp for notification throttling */
        g_phone_connection_timestamp = HAL_GetTick();
        APP_DBG_MSG("📱 Phone connected - throttling notifications for %d ms to allow BLE stack to stabilize\n", NOTIFICATION_THROTTLE_MS);
      /* USER CODE END Service1_APP_PERIPH_CONN_HANDLE_EVT */
      break;

    case SENSOR_SERVER_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */
      /* Clear ONLY the handle that actually disconnected - do NOT clear phone handle on peer disconnect */
      if (p_Notification->ConnectionHandle == SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl)
      {
        SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl = 0xFFFF;
        APP_DBG_MSG("📱 Phone disconnected - cleared handle (no more notifications until reconnect)\n");
      }
      else if (p_Notification->ConnectionHandle == SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl)
      {
        SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl = 0xFFFF;
        APP_DBG_MSG("🔄 Peer disconnected - cleared peer handle in sensor server\n");
      }
      /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_APP_EvtRx_default */

      /* USER CODE END Service1_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

/**
 * @brief Clear all notification subscription flags
 * @note Called when phone disconnects to prevent stale subscription state
 */
void SENSOR_SERVER_APP_ClearSubscriptionFlags(void)
{
  SENSOR_SERVER_APP_Context.Temp_char_Notification_Status = Temp_char_NOTIFICATION_OFF;
  SENSOR_SERVER_APP_Context.Hum_char_Notification_Status = Hum_char_NOTIFICATION_OFF;
  SENSOR_SERVER_APP_Context.Prs_char_Notification_Status = Prs_char_NOTIFICATION_OFF;
  SENSOR_SERVER_APP_Context.Aqi_char_Notification_Status = Aqi_char_NOTIFICATION_OFF;
  SENSOR_SERVER_APP_Context.Gnss_char_Notification_Status = Gnss_char_NOTIFICATION_OFF;
}

void SENSOR_SERVER_APP_Init(void)
{
  SENSOR_SERVER_Init();

  /* USER CODE BEGIN Service1_APP_Init */
  /* Initialize connection handles */
  SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl = 0xFFFF;
  SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl = 0xFFFF;
  
  UTIL_SEQ_RegTask( 1U << TASK_READ_MVH, UTIL_SEQ_RFU, SEND_MVH_Notifications);
  UTIL_SEQ_RegTask( 1U << TASK_READ_LPS22HH, UTIL_SEQ_RFU, SENSOR_SERVER_Prs_char_SendNotification);
  UTIL_SEQ_RegTask( 1U << TASK_READ_GNSS, UTIL_SEQ_RFU, SEND_GNSS_Notification);
  /* USER CODE END Service1_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

__USED void SENSOR_SERVER_Temp_char_SendNotification(void) /* Property Notification */
{
  SENSOR_SERVER_APP_SendInformation_t notification_on_off = Temp_char_NOTIFICATION_OFF;
  SENSOR_SERVER_Data_t sensor_server_notification_data;

  sensor_server_notification_data.p_Payload = (uint8_t*)a_SENSOR_SERVER_UpdateCharData;
  sensor_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char3_NS_1*/
  // APP_DBG_MSG("🌡️ Temp notification called\n");  // DISABLED: Reduces UART flooding
  
  /* CONNECTION STABILITY: Throttle notifications during first 200ms after phone OR peer connects
   * BLE stack needs settle time after ANY connection before notifications can be sent safely */
  if (g_phone_connection_timestamp > 0) {
    uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Phone connection still settling */
    }
  }
  
  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* Check if sensor notifications are paused (during peer connection) */
  extern volatile uint8_t g_pause_sensor_notifications;
  if (g_pause_sensor_notifications) {
    // APP_DBG_MSG("  ⏸️ BLOCKED: Notifications paused\n");
    return;  /* Skip notification during connection attempt */
  }
  
  /* View-based control applies to PHONE only, NOT peer connections
   * View 0 (Map) = Environmental sensors only (blocks motion)
   * View 1 (3D) = Motion sensors only (blocks environmental)
   */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_environmental = (g_current_view == 1);  /* 3D view blocks environmental sensors */
  
  /* SMART RELAY: Prioritize local sensor, fallback to peer, finally check if G_temp has real data */
  if (g_sensor_status.temp_available) {
    /* Module A with working local sensor */
    a_SENSOR_SERVER_UpdateCharData[0] = G_temp[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_temp[1];
    // APP_DBG_MSG("  ✅ LOCAL temp\n");
  } else if (g_peer_temp_valid) {
    /* Module A receiving from peer */
    a_SENSOR_SERVER_UpdateCharData[0] = g_peer_temp[0];
    a_SENSOR_SERVER_UpdateCharData[1] = g_peer_temp[1];
    // APP_DBG_MSG("  📡 PEER temp\n");
  } else if (G_temp[0] != 0 || G_temp[1] != 0) {
    /* Module B with physical sensors (g_sensor_status not initialized) - send G_temp directly */
    a_SENSOR_SERVER_UpdateCharData[0] = G_temp[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_temp[1];
    // APP_DBG_MSG("  ✅ G_temp\n");
  } else {
    // APP_DBG_MSG("  ❌ No temp data\n");
    return;
  }
  
    /*  Update notification data length */
  sensor_server_notification_data.Length +=  2;

    if(SENSOR_SERVER_APP_Context.Temp_char_Notification_Status ==Temp_char_NOTIFICATION_ON)
    {
      notification_on_off = Temp_char_NOTIFICATION_ON;
      // APP_DBG_MSG("  ✅ Notification ON\n");
#if (DEBUG == 1 && STREAM==1)
			  printf("--Sensor_app Temp_char: UPDATEVAL WITH NOTIFICATION\n\r");
#endif
    }
    else
    {
    	//just update char without send notification
    	  SENSOR_SERVER_UpdateValue(SENSOR_SERVER_TEMP_CHAR, &sensor_server_notification_data);
      // APP_DBG_MSG("  ❌ Not subscribed\n");
#if (DEBUG == 1 && STREAM==1)
			  printf("--Sensor_app Temp_char: UPDATEVAL WITHOUT NOTIFICATION\n\r");
#endif
    }
  /* USER CODE END Service1Char3_NS_1*/

  if (notification_on_off != Temp_char_NOTIFICATION_OFF)
  {
    /* Send to phone if connected AND view allows it */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF && !phone_view_blocks_environmental)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_TEMP_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Temp PHONE fail: 0x%02X\n", result);
      }
    }
    
    /* Send to peer if connected (for relay to other phone) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl != 0xFFFF)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_TEMP_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Temp PEER fail: 0x%02X\n", result);
      }
    }
  }
  else
  {
    // APP_DBG_MSG("  ⚠️ notification_on_off == OFF, not sending\n");  // DISABLED: Floods before connection
  }

  /* USER CODE BEGIN Service1Char3_NS_Last*/

  /* USER CODE END Service1Char3_NS_Last*/

  return;
}

__USED void SENSOR_SERVER_Hum_char_SendNotification(void) /* Property Notification */
{
  SENSOR_SERVER_APP_SendInformation_t notification_on_off = Hum_char_NOTIFICATION_OFF;
  SENSOR_SERVER_Data_t sensor_server_notification_data;

  sensor_server_notification_data.p_Payload = (uint8_t*)a_SENSOR_SERVER_UpdateCharData;
  sensor_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char4_NS_1*/
  /* Check if sensor notifications are paused (during peer connection) */
  extern volatile uint8_t g_pause_sensor_notifications;
  if (g_pause_sensor_notifications) {
    return;  /* Skip notification during connection attempt */
  }
  
  /* CRITICAL: Respect throttle after phone connection to prevent 0x0C errors */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF) {
    uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Still in throttle window - phone may not have subscribed yet */
    }
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* View-based control applies to PHONE only, NOT peer connections
   * View 0 (Map) = Environmental sensors only (blocks motion)
   * View 1 (3D) = Motion sensors only (blocks environmental)
   */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_environmental = (g_current_view == 1);  /* 3D view blocks environmental sensors */
  
  /* SMART RELAY: Prioritize local sensor, fallback to peer, finally check if G_hum has real data */
  if (g_sensor_status.humidity_available) {
    /* Module A with working local sensor */
    a_SENSOR_SERVER_UpdateCharData[0] = G_hum[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_hum[1];
  } else if (g_peer_hum_valid) {
    /* Module A receiving from peer */
    a_SENSOR_SERVER_UpdateCharData[0] = g_peer_hum[0];
    a_SENSOR_SERVER_UpdateCharData[1] = g_peer_hum[1];
  } else if (G_hum[0] != 0 || G_hum[1] != 0) {
    /* Module B with physical sensors (g_sensor_status not initialized) - send G_hum directly */
    a_SENSOR_SERVER_UpdateCharData[0] = G_hum[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_hum[1];
  } else {
    return;
  }
     /*  Update notification data length */
     sensor_server_notification_data.Length +=  2;

     if(SENSOR_SERVER_APP_Context.Hum_char_Notification_Status ==Hum_char_NOTIFICATION_ON)
     {
       notification_on_off = Hum_char_NOTIFICATION_ON;
#if (DEBUG == 1 && STREAM==1)
			  printf("--Sensor_app Hum_char: UPDATEVAL WITH NOTIFICATION\n\r");
#endif
     }
     else
     {
     	//just update char without send notification
     	  SENSOR_SERVER_UpdateValue(SENSOR_SERVER_HUM_CHAR, &sensor_server_notification_data);
#if (DEBUG == 1 && STREAM==1)
			  printf("--Sensor_app Hum_char: UPDATEVAL WITHOUT NOTIFICATION\n\r");
#endif
     }
  /* USER CODE END Service1Char4_NS_1*/

  if (notification_on_off != Hum_char_NOTIFICATION_OFF)
  {
    /* Send to phone if connected AND view allows environmental sensors (Map view) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF && !phone_view_blocks_environmental)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_HUM_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app Hum PHONE notification failed: 0x%02X\n\r", result);
      }
    }
    
    /* Send to peer if connected (for relay to other phone) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl != 0xFFFF)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_HUM_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app Hum PEER notification failed: 0x%02X\n\r", result);
      }
    }
  }

  /* USER CODE BEGIN Service1Char4_NS_Last*/

  /* USER CODE END Service1Char4_NS_Last*/

  return;
}

__USED void SENSOR_SERVER_Prs_char_SendNotification(void) /* Property Notification */
{
  SENSOR_SERVER_APP_SendInformation_t notification_on_off = Prs_char_NOTIFICATION_OFF;
  SENSOR_SERVER_Data_t sensor_server_notification_data;

  sensor_server_notification_data.p_Payload = (uint8_t*)a_SENSOR_SERVER_UpdateCharData;
  sensor_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char5_NS_1*/
  /* Check if sensor notifications are paused (during peer connection) */
  extern volatile uint8_t g_pause_sensor_notifications;
  if (g_pause_sensor_notifications) {
    return;  /* Skip notification during connection attempt */
  }
  
  /* CRITICAL: Respect 2000ms throttle after phone connection to prevent 0x0C errors */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF) {
    uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Still in throttle window - phone may not have subscribed yet */
    }
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* View-based control applies to PHONE only, NOT peer connections
   * View 0 (Map) = Environmental sensors only (blocks motion)
   * View 1 (3D) = Motion sensors only (blocks environmental)
   */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_environmental = (g_current_view == 1);  /* 3D view blocks environmental sensors */
  
  /* SMART RELAY: Prioritize local sensor, fallback to peer, finally check if G_prs has real data */
  if (g_sensor_status.pressure_available) {
    /* Module A with working local sensor */
    a_SENSOR_SERVER_UpdateCharData[0] = G_prs[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_prs[1];
    a_SENSOR_SERVER_UpdateCharData[2] = G_prs[2];
  } else if (g_peer_prs_valid) {
    /* Module A receiving from peer */
    a_SENSOR_SERVER_UpdateCharData[0] = g_peer_prs[0];
    a_SENSOR_SERVER_UpdateCharData[1] = g_peer_prs[1];
    a_SENSOR_SERVER_UpdateCharData[2] = g_peer_prs[2];
  } else if (G_prs[0] != 0 || G_prs[1] != 0 || G_prs[2] != 0) {
    /* Module B with physical sensors (g_sensor_status not initialized) - send G_prs directly */
    a_SENSOR_SERVER_UpdateCharData[0] = G_prs[0];
    a_SENSOR_SERVER_UpdateCharData[1] = G_prs[1];
    a_SENSOR_SERVER_UpdateCharData[2] = G_prs[2];
  } else {
    return;
  }
       /*  Update notification data length */
       sensor_server_notification_data.Length +=  3;

       if(SENSOR_SERVER_APP_Context.Prs_char_Notification_Status ==Prs_char_NOTIFICATION_ON)
       {
         notification_on_off = Prs_char_NOTIFICATION_ON;
  #if (DEBUG == 1 && STREAM==1)
  			  printf("--Sensor_app PRS_char: UPDATEVAL WITH NOTIFICATION\n\r");
  #endif
       }
       else
       {
       	//just update char without send notification
       	  SENSOR_SERVER_UpdateValue(SENSOR_SERVER_PRS_CHAR, &sensor_server_notification_data);
  #if (DEBUG == 1 && STREAM==1)
  			  printf("--Sensor_app Hum_char: UPDATEVAL WITHOUT NOTIFICATION\n\r");
  #endif
       }
  /* USER CODE END Service1Char5_NS_1*/

  if (notification_on_off != Prs_char_NOTIFICATION_OFF)
  {
    /* Send to phone if connected AND view allows environmental sensors (Map view) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF && !phone_view_blocks_environmental)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_PRS_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app Prs PHONE notification failed: 0x%02X\n\r", result);
      }
    }
    
    /* Send to peer if connected (for relay to other phone) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl != 0xFFFF)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_PRS_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app Prs PEER notification failed: 0x%02X\n\r", result);
      }
    }
  }

  /* USER CODE BEGIN Service1Char5_NS_Last*/

  /* USER CODE END Service1Char5_NS_Last*/

  return;
}

__USED void SENSOR_SERVER_Aqi_char_SendNotification(void) /* Property Notification */
{
  SENSOR_SERVER_APP_SendInformation_t notification_on_off = Aqi_char_NOTIFICATION_OFF;
  SENSOR_SERVER_Data_t sensor_server_notification_data;

  sensor_server_notification_data.p_Payload = (uint8_t*)a_SENSOR_SERVER_UpdateCharData;
  sensor_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char6_NS_1*/
  /* Check if sensor notifications are paused (during peer connection) */
  extern volatile uint8_t g_pause_sensor_notifications;
  if (g_pause_sensor_notifications) {
    return;  /* Skip notification during connection attempt */
  }
  
  /* CRITICAL: Respect 2000ms throttle after phone connection to prevent 0x0C errors */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF) {
    uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Still in throttle window - phone may not have subscribed yet */
    }
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* View-based control applies to PHONE only, NOT peer connections
   * View 0 (Map) = Environmental sensors only (blocks motion)
   * View 1 (3D) = Motion sensors only (blocks environmental)
   */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_environmental = (g_current_view == 1);  /* 3D view blocks environmental sensors */
  
  // Copy all 11 bytes of air quality data (8 ADC + 2 sample_count + 1 valid)
  for(int i = 0; i < 11; i++) {
    a_SENSOR_SERVER_UpdateCharData[i] = G_aqi[i];
  }
  /* Update notification data length */
  sensor_server_notification_data.Length = 11;  // Send ZMOD4510 AQI data

  if(SENSOR_SERVER_APP_Context.Aqi_char_Notification_Status == Aqi_char_NOTIFICATION_ON)
  {
    notification_on_off = Aqi_char_NOTIFICATION_ON;
#if (DEBUG == 1 && STREAM==1)
    printf("--Sensor_app AQI_char: UPDATEVAL WITH NOTIFICATION\n\r");
#endif
  }
  else
  {
    // just update char without send notification
    SENSOR_SERVER_UpdateValue(SENSOR_SERVER_AQI_CHAR, &sensor_server_notification_data);
#if (DEBUG == 1 && STREAM==1)
    printf("--Sensor_app AQI_char: UPDATEVAL WITHOUT NOTIFICATION\n\r");
#endif
  }
  /* USER CODE END Service1Char6_NS_1*/

  if (notification_on_off != Aqi_char_NOTIFICATION_OFF)
  {
    /* Send to phone if connected AND view allows environmental sensors (Map view) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF && !phone_view_blocks_environmental)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_AQI_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app AQI PHONE notification failed: 0x%02X\n\r", result);
      }
    }
    
    /* Send to peer if connected (for relay to other phone) */
    if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl != 0xFFFF)
    {
      tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_AQI_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl);
      if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("--Sensor_app AQI PEER notification failed: 0x%02X\n\r", result);
      }
    }
  }

  /* USER CODE BEGIN Service1Char6_NS_Last*/

  /* USER CODE END Service1Char6_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
__USED void SEND_MVH_Notifications(void)
{
    extern uint8_t G_temp[2], G_hum[2];
    // Debug disabled - causes UART flooding and BLE instability
    // printf("[TEMP-NOTIF] Sending TEMP: 0x%02X%02X (raw=%d)\r\n", G_temp[1], G_temp[0], (G_temp[1] << 8) | G_temp[0]);
    SENSOR_SERVER_Temp_char_SendNotification();
    // printf("[HUM-NOTIF] Sending HUM: 0x%02X%02X (raw=%d)\r\n", G_hum[1], G_hum[0], (G_hum[1] << 8) | G_hum[0]);
    SENSOR_SERVER_Hum_char_SendNotification();
}

__USED void SEND_GNSS_Notification(void)
{
  SENSOR_SERVER_Data_t sensor_server_notification_data;
  extern uint8_t G_gnss[20];
  
  /* Check if sensor notifications are paused (during peer connection) */
  extern volatile uint8_t g_pause_sensor_notifications;
  if (g_pause_sensor_notifications) {
    return;  /* Skip notification during connection attempt */
  }
  
  /* CRITICAL: Respect 2000ms throttle after phone connection to prevent 0x0C errors */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF) {
    uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Still in throttle window - phone may not have subscribed yet */
    }
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* View-based control applies to PHONE only, NOT peer connections
   * View 0 (Map) = Environmental sensors only (blocks motion)
   * View 1 (3D) = Motion sensors only (blocks environmental)
   */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_environmental = (g_current_view == 1);  /* 3D view blocks environmental sensors */
  
  /* SMART RELAY: Prioritize local sensor, fallback to peer, finally check if G_gnss has real data */
  if (g_sensor_status.gnss_available) {
    /* Module A with working local sensor */
    sensor_server_notification_data.p_Payload = G_gnss;
  } else if (g_peer_gnss_valid) {
    /* Module A receiving from peer */
    sensor_server_notification_data.p_Payload = g_peer_gnss;
  } else {
    /* Check if G_gnss has any non-zero data (Module B with physical sensor) */
    uint8_t has_data = 0;
    for(int i = 0; i < 20; i++) {
      if (G_gnss[i] != 0) {
        has_data = 1;
        break;
      }
    }
    if (has_data) {
      sensor_server_notification_data.p_Payload = G_gnss;
    } else {
      return;  /* No GNSS data available */
    }
  }
  
  sensor_server_notification_data.Length = 20;
  
  /* Send to phone if connected AND subscribed AND view allows environmental sensors (Map view) */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl != 0xFFFF && !phone_view_blocks_environmental
      && SENSOR_SERVER_APP_Context.Gnss_char_Notification_Status == Gnss_char_NOTIFICATION_ON)
  {
    tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GNSS_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
    if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
    {
      APP_DBG_MSG("--Sensor_app GNSS PHONE notification failed: 0x%02X\n\r", result);
    }
  }
  
  /* Send to peer if connected (for relay to other phone) */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl != 0xFFFF)
  {
    tBleStatus result = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GNSS_CHAR, &sensor_server_notification_data, SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl);
    if (result != BLE_STATUS_SUCCESS && result != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
    {
      APP_DBG_MSG("--Sensor_app GNSS PEER notification failed: 0x%02X\n\r", result);
    }
  }
}

/**
  * @brief  Send motion sensor notifications (Accel, Gyro, Mag)
  * @note   Called from TIM1 interrupt @ 10Hz (data updated by IMU DMA)
  */
__USED void SEND_IMU_Notifications(void)
{
  extern uint8_t G_accel[6], G_gyro[6], G_mag[6];
  
  /* CONNECTION STABILITY: Throttle notifications during first 3 seconds after phone connects */
  if (g_phone_connection_timestamp > 0) {
    uint32_t time_since_connect = HAL_GetTick() - g_phone_connection_timestamp;
    if (time_since_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Skip motion sensor notifications during throttle period */
    }
  }
  
  /* Check if sensor notifications are paused (during peer connection) */
  if (g_pause_sensor_notifications) {
    return;  /* Skip notification during connection attempt */
  }
  
  /* Send accelerometer */
  SENSOR_SERVER_RelayAccelFromPeer(G_accel);
  
  /* Send gyroscope */
  SENSOR_SERVER_RelayGyroFromPeer(G_gyro);
  
  /* Send magnetometer */
  SENSOR_SERVER_RelayMagFromPeer(G_mag);
}

/**
  * @brief  Handle view change to start/stop motion sensor timer
  * @param  new_view: 0=Map, 1=3D, 2=Cam
  * @retval None
  */
void SENSOR_SERVER_APP_Handle_ViewChange(uint8_t new_view)
{
  extern TIM_HandleTypeDef htim1;
  uint8_t old_view = g_current_view;
  
  APP_DBG_MSG("🔄 SENSOR_SERVER_APP_Handle_ViewChange called: %d → %d\r\n", old_view, new_view);
  
  g_current_view = new_view;
  
  /* Handle TIM1 (motion sensors) based on view */
  if (new_view == 1 && old_view != 1) {
    /* Entering 3D view - START motion sensor timer */
    /* Check if this module has working sensors (proxy for motion sensor availability) */
    if (g_sensor_status.temp_available || g_sensor_status.humidity_available || 
        g_sensor_status.pressure_available || g_sensor_status.aqi_available) {
      /* Module has sensors - start TIM1 for motion sampling */
      HAL_TIM_Base_Start_IT(&htim1);
      APP_DBG_MSG("🎬 TIM1 started - Motion sensors @ 10Hz\r\n");
    } else {
      /* Module has no sensors - skip TIM1 to prevent BLE interference */
      APP_DBG_MSG("⚠️  3D view: TIM1 NOT started (no sensors on this module)\r\n");
    }
  }
  else if (new_view != 1 && old_view == 1) {
    /* Leaving 3D view - STOP motion sensor timer */
    HAL_TIM_Base_Stop_IT(&htim1);
    APP_DBG_MSG("⏸️  TIM1 stopped - Motion sensors OFF\r\n");
  }
  
  APP_DBG_MSG("📱 View changed: %d → %d (0=Map, 1=3D, 2=Cam)\r\n", old_view, new_view);
}

/**
  * @brief  Get current BLE connection handle for autonomous DMA system
  * @retval Connection handle (0xFFFF if not connected)
  */
uint16_t SENSOR_SERVER_Get_ConnectionHandle(void)
{
  return SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl;
}
/**
  * @brief  Set peer connection handle (Module-to-Module connection)
  * @param  peer_handle: Peer connection handle (use 0xFFFF to clear)
  * @retval None
  */
void SENSOR_SERVER_SetPeerHandle(uint16_t peer_handle)
{
  SENSOR_SERVER_APP_Context.SENSOR_SERVER_peer_connHdl = peer_handle;
  APP_DBG_MSG("📡 Sensor server peer handle set to 0x%04X\n", peer_handle);
  
  /* Record peer connection time for 200ms throttle (same as phone)
   * BLE stack needs time to settle after peer connection before notifications */
  if (peer_handle != 0xFFFF) {
    g_peer_connection_timestamp = HAL_GetTick();
    APP_DBG_MSG("📱 Peer connected - throttling notifications for %d ms to allow BLE stack to stabilize\n", NOTIFICATION_THROTTLE_MS);
  }
}
/**
  * @brief  Set sensor availability status (called after I2C initialization)
  * @param  sensor_type: 0=TEMP, 1=HUM, 2=PRS, 3=AQI, 4=GNSS
  * @param  available: 1=sensor working locally, 0=need from peer
  * @retval None
  */
void SENSOR_SERVER_SetSensorAvailability(uint8_t sensor_type, uint8_t available)
{
  switch(sensor_type) {
    case 0: g_sensor_status.temp_available = available; break;
    case 1: g_sensor_status.humidity_available = available; break;
    case 2: g_sensor_status.pressure_available = available; break;
    case 3: g_sensor_status.aqi_available = available; break;
    case 4: g_sensor_status.gnss_available = available; break;
  }
  
  APP_DBG_MSG("📊 Sensor availability updated: TEMP=%d HUM=%d PRS=%d AQI=%d GNSS=%d\n",
              g_sensor_status.temp_available,
              g_sensor_status.humidity_available,
              g_sensor_status.pressure_available,
              g_sensor_status.aqi_available,
              g_sensor_status.gnss_available);
}

/**
  * @brief  Get sensor availability status (for peer subscription logic)
  * @retval Pointer to sensor status structure
  */
SensorAvailability_t* SENSOR_SERVER_GetSensorStatus(void)
{
  return &g_sensor_status;
}

/**
  * @brief  Relay temperature data from peer to phone (FALLBACK ONLY)
  * @param  p_Payload: Pointer to 2-byte temperature data from peer
  * @retval None
  */
void SENSOR_SERVER_RelayTempFromPeer(uint8_t *p_Payload)
{
  /* Store peer temperature data */
  g_peer_temp[0] = p_Payload[0];
  g_peer_temp[1] = p_Payload[1];
  g_peer_temp_valid = 1;
  
  /* LOGGING MINIMIZED: Was printing 5 lines per notification (every second)
   * This caused ~23ms of UART blocking during critical BLE parameter updates */
  
  /* Only relay to phone if local sensor is NOT available */
  if (g_sensor_status.temp_available) {
    return;  /* Priority to local sensor */
  }
  
  /* Check if phone is connected and subscribed to temperature notifications */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;  /* Phone not connected */
  }
  
  /* CRITICAL: Throttle notifications during phone reconnection window
   * When phone reconnects, it takes ~2 seconds to re-subscribe to characteristics
   * During this time, trying to send notifications causes hundreds of 0x0C errors
   * These errors block CPU and cause supervision timeout (0x28) disconnection */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;  /* Phone just connected, not subscribed yet */
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  if (SENSOR_SERVER_APP_Context.Temp_char_Notification_Status != Temp_char_NOTIFICATION_ON) {
    return;  /* Phone not subscribed */
  }
  
  /* Prepare notification data structure */
  SENSOR_SERVER_Data_t temp_notification;
  temp_notification.p_Payload = p_Payload;
  temp_notification.Length = 2;  /* Temperature is 2 bytes */
  
  /* Send notification to phone */
  tBleStatus ret = SENSOR_SERVER_NotifyValue(
    SENSOR_SERVER_TEMP_CHAR,
    &temp_notification,
    SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl
  );
  
  /* Only log on PERSISTENT errors (not transient 0x0C during connection setup) */
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C) {
    APP_DBG_MSG("⚠️ Relay temp failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay humidity data from peer to phone (FALLBACK ONLY)
  */
void SENSOR_SERVER_RelayHumFromPeer(uint8_t *p_Payload)
{
  g_peer_hum[0] = p_Payload[0];
  g_peer_hum[1] = p_Payload[1];
  g_peer_hum_valid = 1;
  
  /* LOGGING MINIMIZED for BLE timing stability */
  
  if (g_sensor_status.humidity_available) {
    return;
  }
  
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  if (SENSOR_SERVER_APP_Context.Hum_char_Notification_Status != Hum_char_NOTIFICATION_ON) {
    return;
  }
  
  SENSOR_SERVER_Data_t hum_notification;
  hum_notification.p_Payload = p_Payload;
  hum_notification.Length = 2;
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_HUM_CHAR, &hum_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C) {
    APP_DBG_MSG("⚠️ Relay hum failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay pressure data from peer to phone (FALLBACK ONLY)
  */
void SENSOR_SERVER_RelayPrsFromPeer(uint8_t *p_Payload)
{
  g_peer_prs[0] = p_Payload[0];
  g_peer_prs[1] = p_Payload[1];
  g_peer_prs[2] = p_Payload[2];
  g_peer_prs_valid = 1;
  
  /* LOGGING MINIMIZED for BLE timing stability */
  
  if (g_sensor_status.pressure_available) {
    return;
  }
  
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  if (SENSOR_SERVER_APP_Context.Prs_char_Notification_Status != Prs_char_NOTIFICATION_ON) {
    return;
  }
  
  SENSOR_SERVER_Data_t prs_notification;
  prs_notification.p_Payload = p_Payload;
  prs_notification.Length = 3;
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_PRS_CHAR, &prs_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C) {
    APP_DBG_MSG("⚠️ Relay prs failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay AQI data from peer to phone (FALLBACK ONLY)
  */
void SENSOR_SERVER_RelayAqiFromPeer(uint8_t *p_Payload, uint8_t length)
{
  memcpy(g_peer_aqi, p_Payload, (length > 11) ? 11 : length);
  g_peer_aqi_valid = 1;
  
  /* LOGGING MINIMIZED for BLE timing stability */
  
  if (g_sensor_status.aqi_available) {
    return;
  }
  
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  if (SENSOR_SERVER_APP_Context.Aqi_char_Notification_Status != Aqi_char_NOTIFICATION_ON) {
    return;
  }
  
  SENSOR_SERVER_Data_t aqi_notification;
  aqi_notification.p_Payload = g_peer_aqi;
  aqi_notification.Length = 11;
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_AQI_CHAR, &aqi_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C) {
    APP_DBG_MSG("⚠️ Relay aqi failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay GNSS data from peer to phone (FALLBACK ONLY)
  */
void SENSOR_SERVER_RelayGnssFromPeer(uint8_t *p_Payload, uint8_t length)
{
  memcpy(g_peer_gnss, p_Payload, (length > 20) ? 20 : length);
  g_peer_gnss_valid = 1;
  
  /* LOGGING MINIMIZED for BLE timing stability */
  
  if (g_sensor_status.gnss_available) {
    return;
  }
  
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;
  }
  
  /* CRITICAL FIX: Throttle notifications during reconnection window
   * Issue: When phone reconnects, it takes 2 seconds to re-subscribe
   * Relay attempts during this window cause 0x0C errors that block CPU
   * Solution: Block relay for 2 seconds after phone connects */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;  /* Throttle during reconnection window */
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* Skip if phone has not subscribed to GNSS notifications (prevents 0x0C flood) */
  if (SENSOR_SERVER_APP_Context.Gnss_char_Notification_Status != Gnss_char_NOTIFICATION_ON) {
    return;
  }
  
  SENSOR_SERVER_Data_t gnss_notification;
  gnss_notification.p_Payload = g_peer_gnss;
  gnss_notification.Length = 20;
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GNSS_CHAR, &gnss_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C) {
    APP_DBG_MSG("⚠️ Relay gnss failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay accelerometer data from peer to phone (motion sensor)
  */
void SENSOR_SERVER_RelayAccelFromPeer(uint8_t *p_Payload)
{
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;  /* Phone not connected */
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* Check if phone has subscribed to accelerometer notifications */
  if (SENSOR_SERVER_APP_Context.Accel_char_Notification_Status != Accel_char_NOTIFICATION_ON) {
    return;  /* Phone not subscribed to accelerometer */
  }
  
  /* View-based control: Motion sensors ONLY in View 0 (3D), BLOCKED in View 1 (Map) */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_motion = (g_current_view == 0);  /* Map view blocks motion sensors */
  
  if (phone_view_blocks_motion) {
    return;  /* Motion sensors blocked in Map view */
  }
  
  SENSOR_SERVER_Data_t accel_notification;
  accel_notification.p_Payload = p_Payload;
  accel_notification.Length = 6;  /* Accel is 6 bytes (X, Y, Z as int16_t) */
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_ACCEL_CHAR, &accel_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C && ret != 0x88) {
    APP_DBG_MSG("⚠️ Relay accel failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay gyroscope data from peer to phone (motion sensor)
  */
void SENSOR_SERVER_RelayGyroFromPeer(uint8_t *p_Payload)
{
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;  /* Phone not connected */
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* Check if phone has subscribed to gyroscope notifications */
  if (SENSOR_SERVER_APP_Context.Gyro_char_Notification_Status != Gyro_char_NOTIFICATION_ON) {
    return;  /* Phone not subscribed to gyroscope */
  }
  
  /* View-based control: Motion sensors ONLY in View 0 (3D), BLOCKED in View 1 (Map) */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_motion = (g_current_view == 0);  /* Map view blocks motion sensors */
  
  if (phone_view_blocks_motion) {
    return;  /* Motion sensors blocked in Map view */
  }
  
  SENSOR_SERVER_Data_t gyro_notification;
  gyro_notification.p_Payload = p_Payload;
  gyro_notification.Length = 6;  /* Gyro is 6 bytes (X, Y, Z as int16_t) */
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_GYRO_CHAR, &gyro_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C && ret != 0x88) {
    APP_DBG_MSG("⚠️ Relay gyro failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  Relay magnetometer data from peer to phone (motion sensor)
  */
void SENSOR_SERVER_RelayMagFromPeer(uint8_t *p_Payload)
{
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;  /* Phone not connected */
  }
  
  /* CRITICAL: Throttle during reconnection window */
  uint32_t time_since_phone_connect = HAL_GetTick() - g_phone_connection_timestamp;
  if (time_since_phone_connect < NOTIFICATION_THROTTLE_MS) {
    return;
  }

  if (g_peer_connection_timestamp > 0) {
    uint32_t time_since_peer_connect = HAL_GetTick() - g_peer_connection_timestamp;
    if (time_since_peer_connect < NOTIFICATION_THROTTLE_MS) {
      return;  /* Peer connection still settling - prevents 0x0C flood */
    }
  }
  
  /* Check if phone has subscribed to magnetometer notifications */
  if (SENSOR_SERVER_APP_Context.Mag_char_Notification_Status != Mag_char_NOTIFICATION_ON) {
    return;  /* Phone not subscribed to magnetometer */
  }
  
  /* View-based control: Motion sensors ONLY in View 0 (3D), BLOCKED in View 1 (Map) */
  extern volatile uint8_t g_current_view;
  uint8_t phone_view_blocks_motion = (g_current_view == 0);  /* Map view blocks motion sensors */
  
  if (phone_view_blocks_motion) {
    return;  /* Motion sensors blocked in Map view */
  }
  
  SENSOR_SERVER_Data_t mag_notification;
  mag_notification.p_Payload = p_Payload;
  mag_notification.Length = 6;  /* Mag is 6 bytes (X, Y, Z as int16_t) */
  
  tBleStatus ret = SENSOR_SERVER_NotifyValue(SENSOR_SERVER_MAG_CHAR, &mag_notification,
                            SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl);
  if (ret != BLE_STATUS_SUCCESS && ret != 0x0C && ret != 0x88) {
    APP_DBG_MSG("⚠️ Relay mag failed: 0x%02X\n", ret);
  }
}

/**
  * @brief  CONNECTION STABILITY FIX: Keepalive mechanism
  * @detail Sends dummy notification every 4 seconds to prevent 5-second supervision timeout
  *         Android rejected our 20s timeout request, so we keep connection alive with activity
  * @note   Call this from main loop every iteration
  */
void SENSOR_SERVER_ProcessKeepalive(void)
{
  if (!keepalive_enabled) {
    return;
  }
  
  /* Only send keepalive if phone is connected */
  if (SENSOR_SERVER_APP_Context.SENSOR_SERVER_periph_connHdl == 0xFFFF) {
    return;
  }
  
  uint32_t now = HAL_GetTick();
  
  /* Check if it's time to send keepalive */
  if (now - last_keepalive_time < KEEPALIVE_INTERVAL_MS) {
    return;
  }
  
  last_keepalive_time = now;
  
  /* Send a minimal notification to keep connection alive
   * We use temperature characteristic since it's always present
   * This prevents Android from timing out the connection at 5 seconds */
  if (SENSOR_SERVER_APP_Context.Temp_char_Notification_Status == Temp_char_NOTIFICATION_ON) {
    SENSOR_SERVER_Temp_char_SendNotification();
  }
}

/**
  * @brief  CONNECTION STABILITY FIX: Rate limiting check
  * @detail Prevents notification flooding that causes 0x0C (INSUFFICIENT_RESOURCES)
  * @param  None
  * @retval 1 if notification allowed, 0 if rate limit exceeded
  */
uint8_t SENSOR_SERVER_CheckRateLimit(void)
{
  uint32_t now = HAL_GetTick();
  
  /* Reset counter if window expired */
  if (now - rate_limit_window_start >= RATE_LIMIT_WINDOW_MS) {
    notification_count = 0;
    rate_limit_window_start = now;
  }
  
  /* Check if we're at the limit */
  if (notification_count >= NOTIFICATION_RATE_LIMIT) {
    return 0;  /* Rate limit exceeded */
  }
  
  notification_count++;
  return 1;  /* Notification allowed */
}

/**
  * @brief  Enable/disable keepalive mechanism
  * @param  enabled: 1 to enable, 0 to disable
  */
void SENSOR_SERVER_SetKeepaliveEnabled(uint8_t enabled)
{
  keepalive_enabled = enabled;
  if (enabled) {
    last_keepalive_time = HAL_GetTick();  /* Reset timer */
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
