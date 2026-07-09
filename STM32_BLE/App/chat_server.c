/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chat_server.c
  * @author  RFOXiA Team
  * @brief   Chat Server service implementation for phone-to-phone messaging
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
#include "chat_server.h"
#include "chat_server_app.h"
#include "ble_evt.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  ChatServerSvcHdle;          /**< Chat Server Service Handle */
  uint16_t  WriteCharHdle;              /**< Write Characteristic Handle */
  uint16_t  NotifyCharHdle;             /**< Notify Characteristic Handle */
}CHAT_SERVER_Context_t;

/* Private defines -----------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET        2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET             1

/* Private variables ---------------------------------------------------------*/
static CHAT_SERVER_Context_t CHAT_SERVER_Context;

/* Private function prototypes -----------------------------------------------*/

/*
 * UUIDs for Chat Server service (0xABC0, 0xABC1, 0xABC2)
 * Android app UUIDs: 0000ABC0-0000-1000-8000-00805f9b34fb
 */
#define CHAT_SERVER_UUID         0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0xc0,0xab,0x00,0x00
#define CHAT_WRITE_CHAR_UUID     0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0xc1,0xab,0x00,0x00
#define CHAT_NOTIFY_CHAR_UUID    0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0xc2,0xab,0x00,0x00

/* Declare CCCD for notify characteristic */
BLE_GATT_SRV_CCCD_DECLARE(chat_notify, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* Value buffers for characteristics */
uint8_t chat_write_char_val_buffer[CHAT_MESSAGE_MAX_SIZE];
uint8_t chat_notify_char_val_buffer[CHAT_MESSAGE_MAX_SIZE];

static ble_gatt_val_buffer_def_t chat_write_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = CHAT_MESSAGE_MAX_SIZE,
  .buffer_len = sizeof(chat_write_char_val_buffer),
  .buffer_p = chat_write_char_val_buffer
};

static ble_gatt_val_buffer_def_t chat_notify_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = CHAT_MESSAGE_MAX_SIZE,
  .buffer_len = sizeof(chat_notify_char_val_buffer),
  .buffer_p = chat_notify_char_val_buffer
};

/* Chat Server service characteristics definition */
static const ble_gatt_chr_def_t chat_server_chars[] = {
    {
        /* Write characteristic - phone sends messages here */
        .properties = BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(CHAT_WRITE_CHAR_UUID),
        .val_buffer_p = &chat_write_char_val_buffer_def
    },
    {
        /* Notify characteristic - module sends messages to phone */
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(CHAT_NOTIFY_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(chat_notify),
            .descr_count = 1U,
        },
        .val_buffer_p = &chat_notify_char_val_buffer_def
    },
};

/* Chat Server service definition */
static const ble_gatt_srv_def_t chat_server_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(CHAT_SERVER_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)chat_server_chars,
       .chr_count = 2U,
   },
};

/**
 * @brief  Event handler
 * @param  p_Event: Address of the buffer holding the p_Event
 * @retval Ack: Return whether the p_Event has been managed or not
 */
static BLEEVT_EvtAckStatus_t CHAT_SERVER_EventHandler(aci_blecore_event *p_evt)
{
  BLEEVT_EvtAckStatus_t return_value = BLEEVT_NoAck;
  aci_gatt_srv_attribute_modified_event_rp0 *p_attribute_modified;
  CHAT_SERVER_NotificationEvt_t notification;

  switch(p_evt->ecode)
  {
    case ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE:
    {
      p_attribute_modified = (aci_gatt_srv_attribute_modified_event_rp0*)p_evt->data;
      notification.ConnectionHandle         = p_attribute_modified->Connection_Handle;
      notification.AttributeHandle          = p_attribute_modified->Attr_Handle;
      notification.DataTransfered.Length    = p_attribute_modified->Attr_Data_Length;
      notification.DataTransfered.p_Payload = p_attribute_modified->Attr_Data;
      
      /* Check if it's the notify characteristic descriptor (CCCD) */
      if(p_attribute_modified->Attr_Handle == (CHAT_SERVER_Context.NotifyCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        switch(p_attribute_modified->Attr_Data[0])
        {
          /* Disabled Notification management */
          case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
            notification.EvtOpcode = CHAT_SERVER_NOTIFY_CHAR_NOTIFY_DISABLED_EVT;
            CHAT_SERVER_Notification(&notification);
            break;

          /* Enabled Notification management */
          case BLE_GATT_SRV_CCCD_NOTIFICATION:
            notification.EvtOpcode = CHAT_SERVER_NOTIFY_CHAR_NOTIFY_ENABLED_EVT;
            CHAT_SERVER_Notification(&notification);
            break;

          default:
            break;
        }
      }
      /* Check if it's the write characteristic (message from phone) */
      else if(p_attribute_modified->Attr_Handle == (CHAT_SERVER_Context.WriteCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        notification.EvtOpcode = CHAT_SERVER_WRITE_CHAR_WRITE_NO_RESP_EVT;
        CHAT_SERVER_Notification(&notification);
      }
      break;
    }
    
    case ACI_GATT_SRV_WRITE_VSEVT_CODE:
    {
      /* Additional write event handling if needed */
      break;
    }
    
    default:
      break;
  }

  return return_value;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void CHAT_SERVER_Init(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  UNUSED(CHAT_SERVER_Context);

  /**
   * Register the event handler to the BLE controller
   */
  BLEEVT_RegisterGattEvtHandler(CHAT_SERVER_EventHandler);

  /* Register Chat Server service */
  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&chat_server_service);

  /* Get service handle */
  CHAT_SERVER_Context.ChatServerSvcHdle = aci_gatt_srv_get_service_handle((ble_gatt_srv_def_t *)&chat_server_service);
  
  /* Get characteristic handles */
  CHAT_SERVER_Context.WriteCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&chat_server_chars[0]);
  CHAT_SERVER_Context.NotifyCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&chat_server_chars[1]);

  APP_DBG_MSG("  Chat Server handles: Write VALUE=0x%04X, Notify VALUE=0x%04X (add_service ret=0x%02X)\n",
              CHAT_SERVER_Context.WriteCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
              CHAT_SERVER_Context.NotifyCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
              ret);

  return;
}

/**
 * @brief  Get the chat WRITE characteristic VALUE attribute handle.
 * @note   Both modules run identical firmware, so the local handle equals
 *         the peer module's handle — used for peer-to-peer chat relay.
 */
uint16_t CHAT_SERVER_GetWriteCharValueHandle(void)
{
  return CHAT_SERVER_Context.WriteCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET;
}

/**
 * @brief  Get the chat NOTIFY characteristic VALUE attribute handle.
 */
uint16_t CHAT_SERVER_GetNotifyCharValueHandle(void)
{
  return CHAT_SERVER_Context.NotifyCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the new data to be written in the characteristic
 */
tBleStatus CHAT_SERVER_UpdateValue(CHAT_SERVER_CharOpcode_t CharOpcode, CHAT_SERVER_Data_t *pData)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  switch(CharOpcode)
  {
    case CHAT_SERVER_NOTIFY_CHAR:
      memcpy(chat_notify_char_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(chat_notify_char_val_buffer)));
      break;

    default:
      break;
  }

  return ret;
}

/**
 * @brief  Characteristic notification
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the data to be notified to the client
 * @param  ConnectionHandle: connection handle identifying the client to be notified.
 */
tBleStatus CHAT_SERVER_NotifyValue(CHAT_SERVER_CharOpcode_t CharOpcode, CHAT_SERVER_Data_t *pData, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  switch(CharOpcode)
  {
    case CHAT_SERVER_NOTIFY_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                CHAT_SERVER_Context.NotifyCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length,
                                (uint8_t *)pData->p_Payload);
      /* Silent operation - no debug output to avoid flooding UART */
      (void)ret;  /* Suppress unused variable warning */
      break;

    default:
      break;
  }

  return ret;
}
