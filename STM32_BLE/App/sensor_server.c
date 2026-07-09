/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SENSOR_Server.c
  * @author  MCD Application Team
  * @brief   SENSOR_Server definition.
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
#include "app_common.h"
#include "ble.h"
#include "sensor_server.h"
#include "sensor_server_app.h"
#include "ble_evt.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef struct{
  uint16_t  Sensor_serverSvcHdle;				/**< Sensor_server Service Handle */
  uint16_t  Temp_CharCharHdle;			/**< TEMP_CHAR Characteristic Handle */
  uint16_t  Hum_CharCharHdle;			/**< HUM_CHAR Characteristic Handle */
  uint16_t  Prs_CharCharHdle;			/**< PRS_CHAR Characteristic Handle */
  uint16_t  Aqi_CharCharHdle;			/**< AQI_CHAR Characteristic Handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/
  uint16_t  Gnss_CharCharHdle;			/**< GNSS_CHAR Characteristic Handle */
  uint16_t  Accel_CharCharHdle;			/**< ACCEL_CHAR Characteristic Handle */
  uint16_t  Gyro_CharCharHdle;			/**< GYRO_CHAR Characteristic Handle */
  uint16_t  Mag_CharCharHdle;			/**< MAG_CHAR Characteristic Handle */
/* USER CODE END Context */
}SENSOR_SERVER_Context_t;

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET        2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET             1
#define TEMP_CHAR_SIZE        2	/* TEMP Char Characteristic size */
#define HUM_CHAR_SIZE        2	/* HUM Char Characteristic size */
#define PRS_CHAR_SIZE        3	/* PRS Char Characteristic size */
#define AQI_CHAR_SIZE        11	/* AQI Char Characteristic size (TVOC+eCO2+AQI+raw ADC) */
/* USER CODE BEGIN PM */
#define GNSS_CHAR_SIZE       20	/* GNSS Char Characteristic size (lat+lon+alt+speed+heading+fix+sats) */
#define ACCEL_CHAR_SIZE      6	/* ACCEL Char Characteristic size (X, Y, Z as int16_t) */
#define GYRO_CHAR_SIZE       6	/* GYRO Char Characteristic size (X, Y, Z as int16_t) */
#define MAG_CHAR_SIZE        6	/* MAG Char Characteristic size (X, Y, Z as int16_t) */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static SENSOR_SERVER_Context_t SENSOR_SERVER_Context;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

/*
 * UUIDs for SENSOR Server service
 */
#define SENSOR_SERVER_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x20,0x00
#define TEMP_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x22,0x01
#define HUM_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x22,0x02
#define PRS_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x20,0x03
#define AQI_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x23,0x01
#define GNSS_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x24,0x01
#define ACCEL_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x25,0x01
#define GYRO_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x25,0x02
#define MAG_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x25,0x03

BLE_GATT_SRV_CCCD_DECLARE(temp_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(hum_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(prs_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(aqi_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* USER CODE BEGIN DESCRIPTORS DECLARATION */
BLE_GATT_SRV_CCCD_DECLARE(gnss_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(accel_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(gyro_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(mag_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
/* USER CODE END DESCRIPTORS DECLARATION */

uint8_t temp_char_val_buffer[TEMP_CHAR_SIZE];

static ble_gatt_val_buffer_def_t temp_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = TEMP_CHAR_SIZE,
  .buffer_len = sizeof(temp_char_val_buffer),
  .buffer_p = temp_char_val_buffer
};

uint8_t hum_char_val_buffer[HUM_CHAR_SIZE];

static ble_gatt_val_buffer_def_t hum_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = HUM_CHAR_SIZE,
  .buffer_len = sizeof(hum_char_val_buffer),
  .buffer_p = hum_char_val_buffer
};

uint8_t prs_char_val_buffer[PRS_CHAR_SIZE];

static ble_gatt_val_buffer_def_t prs_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = PRS_CHAR_SIZE,
  .buffer_len = sizeof(prs_char_val_buffer),
  .buffer_p = prs_char_val_buffer
};

uint8_t aqi_char_val_buffer[AQI_CHAR_SIZE];

static ble_gatt_val_buffer_def_t aqi_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = AQI_CHAR_SIZE,
  .buffer_len = sizeof(aqi_char_val_buffer),
  .buffer_p = aqi_char_val_buffer
};

/* USER CODE BEGIN VALUE BUFFERS */
uint8_t gnss_char_val_buffer[GNSS_CHAR_SIZE];

static ble_gatt_val_buffer_def_t gnss_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = GNSS_CHAR_SIZE,
  .buffer_len = sizeof(gnss_char_val_buffer),
  .buffer_p = gnss_char_val_buffer
};

uint8_t accel_char_val_buffer[ACCEL_CHAR_SIZE];
static ble_gatt_val_buffer_def_t accel_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = ACCEL_CHAR_SIZE,
  .buffer_len = sizeof(accel_char_val_buffer),
  .buffer_p = accel_char_val_buffer
};

uint8_t gyro_char_val_buffer[GYRO_CHAR_SIZE];
static ble_gatt_val_buffer_def_t gyro_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = GYRO_CHAR_SIZE,
  .buffer_len = sizeof(gyro_char_val_buffer),
  .buffer_p = gyro_char_val_buffer
};

uint8_t mag_char_val_buffer[MAG_CHAR_SIZE];
static ble_gatt_val_buffer_def_t mag_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = MAG_CHAR_SIZE,
  .buffer_len = sizeof(mag_char_val_buffer),
  .buffer_p = mag_char_val_buffer
};
/* USER CODE END VALUE BUFFERS */

/* SENSOR Server service: TEMP, HUM, PRS, AQI, GNSS characteristics */
static const ble_gatt_chr_def_t sensor_server_chars[] = {
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(TEMP_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(temp_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &temp_char_val_buffer_def
    },
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(HUM_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(hum_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &hum_char_val_buffer_def
    },
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(PRS_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(prs_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &prs_char_val_buffer_def
    },
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(AQI_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(aqi_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &aqi_char_val_buffer_def
    },
    /* USER CODE BEGIN CHARACTERISTICS */
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(GNSS_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(gnss_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &gnss_char_val_buffer_def
    },
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(ACCEL_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(accel_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &accel_char_val_buffer_def
    },
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(GYRO_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(gyro_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &gyro_char_val_buffer_def
    },
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(MAG_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(mag_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &mag_char_val_buffer_def
    },
    /* USER CODE END CHARACTERISTICS */
};

/* SENSOR Server service definition */
static const ble_gatt_srv_def_t sensor_server_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SENSOR_SERVER_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)sensor_server_chars,
       .chr_count = 8U,  /* TEMP, HUM, PRS, AQI, GNSS, ACCEL, GYRO, MAG */
   },
};

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  p_Event: Address of the buffer holding the p_Event
 * @retval Ack: Return whether the p_Event has been managed or not
 */
static BLEEVT_EvtAckStatus_t SENSOR_SERVER_EventHandler(aci_blecore_event *p_evt)
{
  BLEEVT_EvtAckStatus_t return_value = BLEEVT_NoAck;
  aci_gatt_srv_attribute_modified_event_rp0 *p_attribute_modified;
  SENSOR_SERVER_NotificationEvt_t notification;
  /* USER CODE BEGIN Service1_EventHandler_1 */

  /* USER CODE END Service1_EventHandler_1 */

  switch(p_evt->ecode)
  {
    case ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE:
    {
      /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

      /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
      p_attribute_modified = (aci_gatt_srv_attribute_modified_event_rp0*)p_evt->data;
      notification.ConnectionHandle         = p_attribute_modified->Connection_Handle;
      notification.AttributeHandle          = p_attribute_modified->Attr_Handle;
      notification.DataTransfered.Length    = p_attribute_modified->Attr_Data_Length;
      notification.DataTransfered.p_Payload = p_attribute_modified->Attr_Data;
      
      /* DEBUG: Print handle comparison - DISABLED to reduce UART flooding */
      #if 0  /* Disabled flooding logs */
      printf("[BLE-EVENT] Attr_Handle: 0x%04X, Data[0]: 0x%02X\r\n", 
             p_attribute_modified->Attr_Handle, p_attribute_modified->Attr_Data[0]);
      printf("[BLE-EVENT] Environmental: TEMP+2=0x%04X, HUM+2=0x%04X, PRS+2=0x%04X, AQI+2=0x%04X\r\n",
             SENSOR_SERVER_Context.Temp_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Hum_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Prs_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Aqi_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET);
      printf("[BLE-EVENT] Motion:        GNSS+2=0x%04X, ACCEL+2=0x%04X, GYRO+2=0x%04X, MAG+2=0x%04X\r\n",
             SENSOR_SERVER_Context.Gnss_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Accel_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Gyro_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET,
             SENSOR_SERVER_Context.Mag_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET);
      #endif
      
      if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Temp_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        /* USER CODE BEGIN Service1_Char_1 */

        /* USER CODE END Service1_Char_1 */
        switch(p_attribute_modified->Attr_Data[0])
		{
          /* USER CODE BEGIN Service1_Char_1_attribute_modified */

          /* USER CODE END Service1_Char_1_attribute_modified */

          /* Disabled Notification management */
        case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
          /* USER CODE BEGIN Service1_Char_1_Disabled_BEGIN */

          /* USER CODE END Service1_Char_1_Disabled_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_TEMP_CHAR_NOTIFY_DISABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_1_Disabled_END */

          /* USER CODE END Service1_Char_1_Disabled_END */
          break;

          /* Enabled Notification management */
        case BLE_GATT_SRV_CCCD_NOTIFICATION:
          /* USER CODE BEGIN Service1_Char_1_COMSVC_Notification_BEGIN */

          /* USER CODE END Service1_Char_1_COMSVC_Notification_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_TEMP_CHAR_NOTIFY_ENABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_1_COMSVC_Notification_END */

          /* USER CODE END Service1_Char_1_COMSVC_Notification_END */
          break;

        default:
          /* USER CODE BEGIN Service1_Char_1_default */

          /* USER CODE END Service1_Char_1_default */
          break;
        }
      }  /* if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Temp_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Hum_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        /* USER CODE BEGIN Service1_Char_4 */

        /* USER CODE END Service1_Char_4 */
        switch(p_attribute_modified->Attr_Data[0])
		{
          /* USER CODE BEGIN Service1_Char_4_attribute_modified */

          /* USER CODE END Service1_Char_4_attribute_modified */

          /* Disabled Notification management */
        case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
          /* USER CODE BEGIN Service1_Char_4_Disabled_BEGIN */

          /* USER CODE END Service1_Char_4_Disabled_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_HUM_CHAR_NOTIFY_DISABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_4_Disabled_END */

          /* USER CODE END Service1_Char_4_Disabled_END */
          break;

          /* Enabled Notification management */
        case BLE_GATT_SRV_CCCD_NOTIFICATION:
          /* USER CODE BEGIN Service1_Char_4_COMSVC_Notification_BEGIN */

          /* USER CODE END Service1_Char_4_COMSVC_Notification_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_HUM_CHAR_NOTIFY_ENABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_4_COMSVC_Notification_END */

          /* USER CODE END Service1_Char_4_COMSVC_Notification_END */
          break;

        default:
          /* USER CODE BEGIN Service1_Char_4_default */

          /* USER CODE END Service1_Char_4_default */
          break;
        }
      }  /* if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Hum_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Prs_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        /* USER CODE BEGIN Service1_Char_5 */

        /* USER CODE END Service1_Char_5 */
        switch(p_attribute_modified->Attr_Data[0])
		{
          /* USER CODE BEGIN Service1_Char_5_attribute_modified */

          /* USER CODE END Service1_Char_5_attribute_modified */

          /* Disabled Notification management */
        case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
          /* USER CODE BEGIN Service1_Char_5_Disabled_BEGIN */

          /* USER CODE END Service1_Char_5_Disabled_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_PRS_CHAR_NOTIFY_DISABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_5_Disabled_END */

          /* USER CODE END Service1_Char_5_Disabled_END */
          break;

          /* Enabled Notification management */
        case BLE_GATT_SRV_CCCD_NOTIFICATION:
          /* USER CODE BEGIN Service1_Char_5_COMSVC_Notification_BEGIN */

          /* USER CODE END Service1_Char_5_COMSVC_Notification_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_PRS_CHAR_NOTIFY_ENABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_5_COMSVC_Notification_END */

          /* USER CODE END Service1_Char_5_COMSVC_Notification_END */
          break;

        default:
          /* USER CODE BEGIN Service1_Char_5_default */

          /* USER CODE END Service1_Char_5_default */
          break;
        }
      }  /* if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Prs_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Aqi_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        /* USER CODE BEGIN Service1_Char_6 */

        /* USER CODE END Service1_Char_6 */
        switch(p_attribute_modified->Attr_Data[0])
		{
          /* USER CODE BEGIN Service1_Char_6_attribute_modified */

          /* USER CODE END Service1_Char_6_attribute_modified */

          /* Disabled Notification management */
        case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
          /* USER CODE BEGIN Service1_Char_6_Disabled_BEGIN */

          /* USER CODE END Service1_Char_6_Disabled_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_AQI_CHAR_NOTIFY_DISABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_6_Disabled_END */

          /* USER CODE END Service1_Char_6_Disabled_END */
          break;

          /* Enabled Notification management */
        case BLE_GATT_SRV_CCCD_NOTIFICATION:
          /* USER CODE BEGIN Service1_Char_6_COMSVC_Notification_BEGIN */

          /* USER CODE END Service1_Char_6_COMSVC_Notification_BEGIN */
          notification.EvtOpcode = SENSOR_SERVER_AQI_CHAR_NOTIFY_ENABLED_EVT;
          SENSOR_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service1_Char_6_COMSVC_Notification_END */

          /* USER CODE END Service1_Char_6_COMSVC_Notification_END */
          break;

        default:
          /* USER CODE BEGIN Service1_Char_6_default */

          /* USER CODE END Service1_Char_6_default */
          break;
        }
      }  /* if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Aqi_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

      /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
      /* GNSS Characteristic notification enable/disable handling */
      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Gnss_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        if(p_attribute_modified->Attr_Data[0] & 0x01)
        {
          /* Notification enabled */
          notification.EvtOpcode = SENSOR_SERVER_GNSS_CHAR_NOTIFY_ENABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
        else
        {
          /* Notification disabled */
          notification.EvtOpcode = SENSOR_SERVER_GNSS_CHAR_NOTIFY_DISABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
      }
      /* ACCEL Characteristic notification enable/disable handling */
      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Accel_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        if(p_attribute_modified->Attr_Data[0] & 0x01)
        {
          notification.EvtOpcode = SENSOR_SERVER_ACCEL_CHAR_NOTIFY_ENABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
        else
        {
          notification.EvtOpcode = SENSOR_SERVER_ACCEL_CHAR_NOTIFY_DISABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
      }
      /* GYRO Characteristic notification enable/disable handling */
      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Gyro_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        if(p_attribute_modified->Attr_Data[0] & 0x01)
        {
          notification.EvtOpcode = SENSOR_SERVER_GYRO_CHAR_NOTIFY_ENABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
        else
        {
          notification.EvtOpcode = SENSOR_SERVER_GYRO_CHAR_NOTIFY_DISABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
      }
      /* MAG Characteristic notification enable/disable handling */
      else if(p_attribute_modified->Attr_Handle == (SENSOR_SERVER_Context.Mag_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        if(p_attribute_modified->Attr_Data[0] & 0x01)
        {
          notification.EvtOpcode = SENSOR_SERVER_MAG_CHAR_NOTIFY_ENABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
        else
        {
          notification.EvtOpcode = SENSOR_SERVER_MAG_CHAR_NOTIFY_DISABLED_EVT;
          printf("EVENT OPCODE: 0x%04X\r\n", notification.EvtOpcode);
          SENSOR_SERVER_Notification(&notification);
        }
      }
      /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
      break;/* ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE */
    }
    case ACI_GATT_SRV_READ_VSEVT_CODE :
    {
      /* USER CODE BEGIN EVT_BLUE_GATT_SRV_READ_BEGIN */

      /* USER CODE END EVT_BLUE_GATT_SRV_READ_BEGIN */

      /* USER CODE BEGIN EVT_BLUE_GATT_SRV_READ_END */

      /* USER CODE END EVT_EVT_BLUE_GATT_SRV_READ_END */
      break;/* ACI_GATT_SRV_READ_VSEVT_CODE */
    }
    case ACI_GATT_SRV_WRITE_VSEVT_CODE:
    {
      /* USER CODE BEGIN EVT_BLUE_SRV_GATT_BEGIN */

      /* USER CODE END EVT_BLUE_SRV_GATT_BEGIN */

      /* USER CODE BEGIN EVT_BLUE_GATT_SRV_WRITE_END */

      /* USER CODE END EVT_BLUE_GATT_SRV_WRITE_END */
      break;/* ACI_GATT_SRV_WRITE_VSEVT_CODE */
    }
    case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
    {
      aci_gatt_tx_pool_available_event_rp0 *p_tx_pool_available_event;
      p_tx_pool_available_event = (aci_gatt_tx_pool_available_event_rp0 *) p_evt->data;
      UNUSED(p_tx_pool_available_event);

      /* USER CODE BEGIN ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */

      /* USER CODE END ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */
      break;/* ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE*/
    }
    case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
    {
      aci_att_exchange_mtu_resp_event_rp0 *p_exchange_mtu;
      p_exchange_mtu = (aci_att_exchange_mtu_resp_event_rp0 *)  p_evt->data;
      UNUSED(p_exchange_mtu);

      /* USER CODE BEGIN ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */

      /* USER CODE END ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */
      break;/* ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */
    }
    /* USER CODE BEGIN BLECORE_EVT */

    /* USER CODE END BLECORE_EVT */
  default:
    /* USER CODE BEGIN EVT_DEFAULT */

    /* USER CODE END EVT_DEFAULT */
    break;
  }

  /* USER CODE BEGIN Service1_EventHandler_2 */

  /* USER CODE END Service1_EventHandler_2 */

  return(return_value);
}/* end SENSOR_SERVER_EventHandler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SENSOR_SERVER_Init(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  UNUSED(SENSOR_SERVER_Context);

  /* USER CODE BEGIN InitService1Svc_1 */

  /* USER CODE END InitService1Svc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  BLEEVT_RegisterGattEvtHandler(SENSOR_SERVER_EventHandler);

  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&sensor_server_service);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_srv_add_service command: SENSOR_Server, error code: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_srv_add_service command: SENSOR_Server \n");
  }

  SENSOR_SERVER_Context.Sensor_serverSvcHdle = aci_gatt_srv_get_service_handle((ble_gatt_srv_def_t *) &sensor_server_service);
  SENSOR_SERVER_Context.Temp_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[0]);
  SENSOR_SERVER_Context.Hum_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[1]);
  SENSOR_SERVER_Context.Prs_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[2]);
  SENSOR_SERVER_Context.Aqi_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[3]);

  /* USER CODE BEGIN InitService1Svc_2 */
  SENSOR_SERVER_Context.Gnss_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[4]);
  SENSOR_SERVER_Context.Accel_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[5]);
  SENSOR_SERVER_Context.Gyro_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[6]);
  SENSOR_SERVER_Context.Mag_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sensor_server_chars[7]);
  
  /* Debug: Print motion sensor characteristic handles */
  APP_DBG_MSG("��� Motion Sensor Handles:\n");
  APP_DBG_MSG("  GNSS:  0x%04X (CCCD: 0x%04X)\n", SENSOR_SERVER_Context.Gnss_CharCharHdle, SENSOR_SERVER_Context.Gnss_CharCharHdle + 2);
  APP_DBG_MSG("  ACCEL: 0x%04X (CCCD: 0x%04X)\n", SENSOR_SERVER_Context.Accel_CharCharHdle, SENSOR_SERVER_Context.Accel_CharCharHdle + 2);
  APP_DBG_MSG("  GYRO:  0x%04X (CCCD: 0x%04X)\n", SENSOR_SERVER_Context.Gyro_CharCharHdle, SENSOR_SERVER_Context.Gyro_CharCharHdle + 2);
  APP_DBG_MSG("  MAG:   0x%04X (CCCD: 0x%04X)\n", SENSOR_SERVER_Context.Mag_CharCharHdle, SENSOR_SERVER_Context.Mag_CharCharHdle + 2);
  /* USER CODE END InitService1Svc_2 */

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail registering SENSOR_Server handlers\n");
  }

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the new data to be written in the characteristic
 *
 */
tBleStatus SENSOR_SERVER_UpdateValue(SENSOR_SERVER_CharOpcode_t CharOpcode, SENSOR_SERVER_Data_t *pData)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* USER CODE BEGIN Service1_App_Update_Char_1 */
  switch(CharOpcode)
  {
  case SENSOR_SERVER_TEMP_CHAR:
       memcpy(temp_char_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(temp_char_val_buffer)));
       break;
  case SENSOR_SERVER_HUM_CHAR:
       memcpy(hum_char_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(hum_char_val_buffer)));
       break;
  case SENSOR_SERVER_PRS_CHAR:
	  memcpy(prs_char_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(prs_char_val_buffer)));
	         break;
    default:
      break;
  }
  /* USER CODE END Service1_App_Update_Char_1 */

  switch(CharOpcode)
  {

    default:
      break;
  }

  /* USER CODE BEGIN Service1_App_Update_Char_2 */

  /* USER CODE END Service1_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic notification
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the data to be notified to the client
 * @param  ConnectionHandle: connection handle identifying the client to be notified.
 *
 */
tBleStatus SENSOR_SERVER_NotifyValue(SENSOR_SERVER_CharOpcode_t CharOpcode, SENSOR_SERVER_Data_t *pData, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Service1_App_Notify_Char_1 */

  /* USER CODE END Service1_App_Notify_Char_1 */

  switch(CharOpcode)
  {
    case SENSOR_SERVER_TEMP_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Temp_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify TEMP_CHAR command, error code: 0x%2X\n", ret);
      }
      // else { APP_DBG_MSG("  Success: aci_gatt_srv_notify TEMP_CHAR command\n"); }  // DISABLED
      /* USER CODE BEGIN Service1_Char_Value_3*/

      /* USER CODE END Service1_Char_Value_3*/
      break;

    case SENSOR_SERVER_HUM_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Hum_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify HUM_CHAR command, error code: 0x%2X\n", ret);
      }
      // else { APP_DBG_MSG("  Success: aci_gatt_srv_notify HUM_CHAR command\n"); }  // DISABLED
      /* USER CODE BEGIN Service1_Char_Value_4*/

      /* USER CODE END Service1_Char_Value_4*/
      break;

    case SENSOR_SERVER_PRS_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Prs_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify PRS_CHAR command, error code: 0x%2X\n", ret);
      }
      // else { APP_DBG_MSG("  Success: aci_gatt_srv_notify PRS_CHAR command\n"); }  // DISABLED
      /* USER CODE BEGIN Service1_Char_Value_5*/

      /* USER CODE END Service1_Char_Value_5*/
      break;

    case SENSOR_SERVER_AQI_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Aqi_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify AQI_CHAR command, error code: 0x%2X\n", ret);
      }
      // else { APP_DBG_MSG("  Success: aci_gatt_srv_notify AQI_CHAR command\n"); }  // DISABLED
      /* USER CODE BEGIN Service1_Char_Value_6*/

      /* USER CODE END Service1_Char_Value_6*/
      break;

    /* USER CODE BEGIN NOTIFICATION_CASES */
    case SENSOR_SERVER_GNSS_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Gnss_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)  /* Suppress 0x88 INSUFFICIENT_RESOURCES */
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify GNSS_CHAR command, error code: 0x%2X\n", ret);
      }
      // else { APP_DBG_MSG("  Success: aci_gatt_srv_notify GNSS_CHAR command\n"); }  // DISABLED
      break;
    case SENSOR_SERVER_ACCEL_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Accel_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length,
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify ACCEL_CHAR command, error code: 0x%2X\n", ret);
      }
      break;
    case SENSOR_SERVER_GYRO_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Gyro_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length,
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify GYRO_CHAR command, error code: 0x%2X\n", ret);
      }
      break;
    case SENSOR_SERVER_MAG_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                SENSOR_SERVER_Context.Mag_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length,
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS && ret != 0x88)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify MAG_CHAR command, error code: 0x%2X\n", ret);
      }
      break;
    /* USER CODE END NOTIFICATION_CASES */

    default:
      break;
  }

  /* USER CODE BEGIN Service1_App_Notify_Char_2 */

  /* USER CODE END Service1_App_Notify_Char_2 */

  return ret;
}
