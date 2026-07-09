/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chat_server_app.c
  * @author  RFOXiA Team
  * @brief   Chat Server application - Handles phone-to-phone message relay
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
#include "chat_server_app.h"
#include "chat_server.h"
#include "dual_role_manager.h"
#include "gatt_client_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  Chat_notify_NOTIFICATION_OFF,
  Chat_notify_NOTIFICATION_ON,
  CHAT_SERVER_APP_SENDINFORMATION_LAST
} CHAT_SERVER_APP_SendInformation_t;

typedef struct
{
  CHAT_SERVER_APP_SendInformation_t     Chat_notify_Notification_Status;
  uint16_t ConnectionHandle;
} CHAT_SERVER_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CHAT_SERVER_APP_Context_t CHAT_SERVER_APP_Context;

/* Message buffer for relay */
static uint8_t g_message_buffer[CHAT_MESSAGE_MAX_SIZE];
static uint8_t g_message_length = 0;

/* Private function prototypes -----------------------------------------------*/
static void CHAT_SERVER_Chat_notify_SendNotification(void);

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Notification handler called by chat_server.c
 * @param  p_Notification: pointer to the notification event
 * @retval None
 */
void CHAT_SERVER_Notification(CHAT_SERVER_NotificationEvt_t *p_Notification)
{
  switch(p_Notification->EvtOpcode)
  {
    case CHAT_SERVER_WRITE_CHAR_WRITE_NO_RESP_EVT:
      {
        /* Message received - check WHO sent it to prevent relay loops */
        if(p_Notification->DataTransfered.Length <= CHAT_MESSAGE_MAX_SIZE)
        {
          memcpy(g_message_buffer, p_Notification->DataTransfered.p_Payload, p_Notification->DataTransfered.Length);
          g_message_length = p_Notification->DataTransfered.Length;
          
          /* Get dual-role context to check connection handles */
          DualRole_Context_t *dual_ctx = DualRole_GetContext();
          uint16_t sender_handle = p_Notification->ConnectionHandle;
          
          /* CRITICAL: Determine message direction to prevent infinite loops
           * - If from PHONE → relay to PEER
           * - If from PEER → relay to PHONE (NOT back to peer!)
           */
          if(dual_ctx != NULL)
          {
            if(sender_handle == dual_ctx->peer_conn_handle && dual_ctx->phone_connected)
            {
              /* Message FROM peer → relay to PHONE via notification
               * CRITICAL: Check if phone has subscribed to notifications first!
               * If not subscribed, notification will fail with error 0x0C (NOT_ALLOWED)
               */
              if(CHAT_SERVER_APP_Context.Chat_notify_Notification_Status != Chat_notify_NOTIFICATION_ON)
              {
                APP_DBG_MSG("⚠️ Cannot relay to phone: Notifications NOT enabled by phone!\n");
                APP_DBG_MSG("   Phone app must subscribe to Chat Notify characteristic (0xABC2)\n");
                return;
              }
              
              uint16_t phone_chat_notify_handle = CHAT_SERVER_NOTIFY_CHAR;
              
              APP_DBG_MSG("📡→📱 Relaying %d bytes from peer to phone (handle 0x%04X)\n", 
                         g_message_length, dual_ctx->phone_conn_handle);
              
              CHAT_SERVER_Data_t notify_data;
              notify_data.p_Payload = g_message_buffer;
              notify_data.Length = g_message_length;
              
              tBleStatus ret = CHAT_SERVER_NotifyValue(
                  phone_chat_notify_handle,
                  &notify_data,
                  dual_ctx->phone_conn_handle
              );
              
              if (ret != BLE_STATUS_SUCCESS) {
                APP_DBG_MSG("  ⚠️ Relay to phone failed: 0x%02X\n", ret);
              } else {
                APP_DBG_MSG("  ✅ Relayed to phone successfully\n");
              }
            }
            else if(sender_handle == dual_ctx->phone_conn_handle && dual_ctx->peer_connected)
            {
              /* Message FROM phone → relay to PEER
               * Use the REAL chat write VALUE handle (both modules run identical
               * firmware → local handle == peer handle). The old hardcoded 0x002C
               * pointed at a BConnection attribute → messages silently dropped. */
              uint16_t peer_chat_write_handle = CHAT_SERVER_GetWriteCharValueHandle();
              
              APP_DBG_MSG("📱→📡 Relaying %d bytes from phone to peer (handle 0x%04X)\n", 
                         g_message_length, dual_ctx->peer_conn_handle);
              
              tBleStatus ret = aci_gatt_clt_write_without_resp(
                  dual_ctx->peer_conn_handle,
                  BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                  peer_chat_write_handle,
                  g_message_length,
                  g_message_buffer
              );
              
              if (ret != BLE_STATUS_SUCCESS) {
                APP_DBG_MSG("  ⚠️ Relay to peer failed: 0x%02X\n", ret);
              } else {
                APP_DBG_MSG("  ✅ Relayed to peer successfully\n");
              }
            }
            else
            {
              APP_DBG_MSG("⚠️ Message from unknown handle 0x%04X (peer=0x%04X, phone=0x%04X) - ignored\n",
                         sender_handle, dual_ctx->peer_conn_handle, dual_ctx->phone_conn_handle);
            }
          }
          else
          {
            APP_DBG_MSG("📱 Message received but dual-role context unavailable\n");
          }
        }
      }
      break;

    case CHAT_SERVER_NOTIFY_CHAR_NOTIFY_ENABLED_EVT:
      {
        CHAT_SERVER_APP_Context.Chat_notify_Notification_Status = Chat_notify_NOTIFICATION_ON;
        CHAT_SERVER_APP_Context.ConnectionHandle = p_Notification->ConnectionHandle;
      }
      break;

    case CHAT_SERVER_NOTIFY_CHAR_NOTIFY_DISABLED_EVT:
      {
        CHAT_SERVER_APP_Context.Chat_notify_Notification_Status = Chat_notify_NOTIFICATION_OFF;
      }
      break;

    default:
      break;
  }

  return;
}

/**
 * @brief  Application initialization
 * @param  None
 * @retval None
 */
void CHAT_SERVER_APP_Init(void)
{
  CHAT_SERVER_APP_Context.Chat_notify_Notification_Status = Chat_notify_NOTIFICATION_OFF;
  CHAT_SERVER_APP_Context.ConnectionHandle = 0;
  
  APP_DBG_MSG("Chat Server APP initialized\n");

  return;
}

/**
 * @brief  Called when message received from peer module (via GATT client notification)
 * @param  p_Payload: pointer to message data
 * @retval None
 */
void CHAT_SERVER_APP_EvtRx(void *p_Payload)
{
  tBleStatus ret;
  CHAT_SERVER_Data_t msg_data;
  
  /* This will be called when we receive a notification from peer module's chat characteristic */
  /* Forward the message to our connected phone */
  
  if(CHAT_SERVER_APP_Context.Chat_notify_Notification_Status == Chat_notify_NOTIFICATION_ON)
  {
    APP_DBG_MSG("📡→📱 Relaying message from peer to phone\n");
    
    /* Prepare message data */
    msg_data.p_Payload = (uint8_t *)p_Payload;
    msg_data.Length = strlen((char *)p_Payload);
    
    /* Send notification to phone */
    ret = CHAT_SERVER_NotifyValue(CHAT_SERVER_NOTIFY_CHAR, &msg_data, CHAT_SERVER_APP_Context.ConnectionHandle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Error sending message to phone: 0x%02X\n", ret);
    }
    else
    {
      APP_DBG_MSG("  Message sent to phone successfully\n");
    }
  }
  else
  {
    APP_DBG_MSG("⚠️  Phone not subscribed - message dropped\n");
  }
}

/**
 * @brief  Send chat notification to phone
 * @param  None
 * @retval None
 */
static void CHAT_SERVER_Chat_notify_SendNotification(void)
{
  tBleStatus ret;
  CHAT_SERVER_Data_t msg_data;

  if(CHAT_SERVER_APP_Context.Chat_notify_Notification_Status == Chat_notify_NOTIFICATION_ON)
  {
    msg_data.p_Payload = g_message_buffer;
    msg_data.Length = g_message_length;

    ret = CHAT_SERVER_NotifyValue(CHAT_SERVER_NOTIFY_CHAR, &msg_data, CHAT_SERVER_APP_Context.ConnectionHandle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail: CHAT_SERVER_NotifyValue, error code: 0x%2X\n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: Message sent to phone\n");
    }
  }

  return;
}
