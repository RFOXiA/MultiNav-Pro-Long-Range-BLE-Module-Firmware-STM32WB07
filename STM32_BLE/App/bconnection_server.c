/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BConnection_Server.c
  * @author  MCD Application Team
  * @brief   BConnection_Server definition.
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
#include "bconnection_server.h"
#include "bconnection_server_app.h"
#include "ble_evt.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef struct{
  uint16_t  Bconnection_serverSvcHdle;				/**< Bconnection_server Service Handle */
  uint16_t  Blist_State_CharCharHdle;			/**< BLIST_STATE_CHAR Characteristic Handle */
  uint16_t  Blist_CharCharHdle;			/**< BLIST_CHAR Characteristic Handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}BCONNECTION_SERVER_Context_t;

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET        2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET             1
#define BLIST_STATE_CHAR_SIZE        1	/* BList state Char Characteristic size */
#define BLIST_CHAR_SIZE        255	/* BList Char Characteristic size */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static BCONNECTION_SERVER_Context_t BCONNECTION_SERVER_Context;

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
 * UUIDs for BConnection Server service
 */
#define BCONNECTION_SERVER_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x40,0x00
#define BLIST_STATE_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x40,0x01
#define BLIST_CHAR_UUID			0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x9a,0xbc,0x40,0x02

BLE_GATT_SRV_CCCD_DECLARE(blist_state, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
BLE_GATT_SRV_CCCD_DECLARE(blist_char, CFG_BLE_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* USER CODE BEGIN DESCRIPTORS DECLARATION */

/* USER CODE END DESCRIPTORS DECLARATION */

uint8_t blist_state_char_val_buffer[BLIST_STATE_CHAR_SIZE];

static ble_gatt_val_buffer_def_t blist_state_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = BLIST_STATE_CHAR_SIZE,
  .buffer_len = sizeof(blist_state_char_val_buffer),
  .buffer_p = blist_state_char_val_buffer
};

uint8_t blist_char_val_buffer[BLIST_CHAR_SIZE];

static ble_gatt_val_buffer_def_t blist_char_val_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG | BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = BLIST_CHAR_SIZE,
  .buffer_len = sizeof(blist_char_val_buffer),
  .buffer_p = blist_char_val_buffer
};

/* BConnection Server service BLIST_STATE_CHAR (notification), BLIST_CHAR (notification) characteristics definition */
static const ble_gatt_chr_def_t bconnection_server_chars[] = {
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(BLIST_STATE_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(blist_state),
            .descr_count = 1U,
        },
        .val_buffer_p = &blist_state_char_val_buffer_def
    },
	{
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_128(BLIST_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(blist_char),
            .descr_count = 1U,
        },
        .val_buffer_p = &blist_char_val_buffer_def
    },
};

/* BConnection Server service definition */
static const ble_gatt_srv_def_t bconnection_server_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(BCONNECTION_SERVER_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)bconnection_server_chars,
       .chr_count = 2U,
   },
};

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  p_Event: Address of the buffer holding the p_Event
 * @retval Ack: Return whether the p_Event has been managed or not
 */
static BLEEVT_EvtAckStatus_t BCONNECTION_SERVER_EventHandler(aci_blecore_event *p_evt)
{
  BLEEVT_EvtAckStatus_t return_value = BLEEVT_NoAck;
  aci_gatt_srv_attribute_modified_event_rp0 *p_attribute_modified;
  BCONNECTION_SERVER_NotificationEvt_t notification;
  /* USER CODE BEGIN Service3_EventHandler_1 */

  /* USER CODE END Service3_EventHandler_1 */

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
      if(p_attribute_modified->Attr_Handle == (BCONNECTION_SERVER_Context.Blist_State_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        switch(p_attribute_modified->Attr_Data[0])
        {
          case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
            notification.EvtOpcode = BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_DISABLED_EVT;
            BCONNECTION_SERVER_Notification(&notification);
            break;

          case BLE_GATT_SRV_CCCD_NOTIFICATION:
            notification.EvtOpcode = BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_ENABLED_EVT;
            BCONNECTION_SERVER_Notification(&notification);
            break;

          default:
            break;
        }
      }
      else if(p_attribute_modified->Attr_Handle == (BCONNECTION_SERVER_Context.Blist_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;
        /* USER CODE BEGIN Service3_Char_2 */

        /* USER CODE END Service3_Char_2 */
        switch(p_attribute_modified->Attr_Data[0])
		{
          /* USER CODE BEGIN Service3_Char_2_attribute_modified */

          /* USER CODE END Service3_Char_2_attribute_modified */

          /* Disabled Notification management */
        case (!BLE_GATT_SRV_CCCD_NOTIFICATION):
          /* USER CODE BEGIN Service3_Char_2_Disabled_BEGIN */

          /* USER CODE END Service3_Char_2_Disabled_BEGIN */
          notification.EvtOpcode = BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_DISABLED_EVT;
          BCONNECTION_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service3_Char_2_Disabled_END */

          /* USER CODE END Service3_Char_2_Disabled_END */
          break;

          /* Enabled Notification management */
        case BLE_GATT_SRV_CCCD_NOTIFICATION:
          /* USER CODE BEGIN Service3_Char_2_COMSVC_Notification_BEGIN */

          /* USER CODE END Service3_Char_2_COMSVC_Notification_BEGIN */
          notification.EvtOpcode = BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_ENABLED_EVT;
          BCONNECTION_SERVER_Notification(&notification);
          /* USER CODE BEGIN Service3_Char_2_COMSVC_Notification_END */

          /* USER CODE END Service3_Char_2_COMSVC_Notification_END */
          break;

        default:
          /* USER CODE BEGIN Service3_Char_2_default */

          /* USER CODE END Service3_Char_2_default */
          break;
        }
      }  /* if(p_attribute_modified->Attr_Handle == (BCONNECTION_SERVER_Context.Blist_CharCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

      else if(p_attribute_modified->Attr_Handle == (BCONNECTION_SERVER_Context.Blist_State_CharCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
      {
        return_value = BLEEVT_Ack;

        notification.EvtOpcode = BCONNECTION_SERVER_BLIST_STATE_CHAR_WRITE_NO_RESP_EVT;
        /* USER CODE BEGIN Service3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
        APP_DBG_MSG("📝 B_STATE WRITE EVENT RECEIVED!\n");
        APP_DBG_MSG("   Connection Handle: 0x%04X\n", p_attribute_modified->Connection_Handle);
        APP_DBG_MSG("   Attr Handle: 0x%04X\n", p_attribute_modified->Attr_Handle);
        APP_DBG_MSG("   Data Length: %d bytes\n", p_attribute_modified->Attr_Data_Length);
        if (p_attribute_modified->Attr_Data_Length > 0) {
          APP_DBG_MSG("   Data: ");
          for (uint16_t i = 0; i < p_attribute_modified->Attr_Data_Length; i++) {
            APP_DBG_MSG("0x%02X ", p_attribute_modified->Attr_Data[i]);
          }
          APP_DBG_MSG("\n");
        }
        /* USER CODE END Service3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
        BCONNECTION_SERVER_Notification(&notification);
      } /* if(p_attribute_modified->Attr_Handle == (BCONNECTION_SERVER_Context.Blist_State_CharCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

      /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

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

  /* USER CODE BEGIN Service3_EventHandler_2 */

  /* USER CODE END Service3_EventHandler_2 */

  return(return_value);
}/* end BCONNECTION_SERVER_EventHandler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void BCONNECTION_SERVER_Init(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  UNUSED(BCONNECTION_SERVER_Context);

  /* USER CODE BEGIN InitService3Svc_1 */

  /* USER CODE END InitService3Svc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  BLEEVT_RegisterGattEvtHandler(BCONNECTION_SERVER_EventHandler);

  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&bconnection_server_service);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_srv_add_service command: BConnection_Server, error code: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_srv_add_service command: BConnection_Server \n");
  }

  BCONNECTION_SERVER_Context.Bconnection_serverSvcHdle = aci_gatt_srv_get_service_handle((ble_gatt_srv_def_t *) &bconnection_server_service);
  BCONNECTION_SERVER_Context.Blist_State_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&bconnection_server_chars[0]);
  BCONNECTION_SERVER_Context.Blist_CharCharHdle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&bconnection_server_chars[1]);

  /* USER CODE BEGIN InitService1Svc_2 */

  /* USER CODE END InitService1Svc_2 */

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail registering BConnection_Server handlers\n");
  }

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the new data to be written in the characteristic
 *
 */
tBleStatus BCONNECTION_SERVER_UpdateValue(BCONNECTION_SERVER_CharOpcode_t CharOpcode, BCONNECTION_SERVER_Data_t *pData)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* USER CODE BEGIN Service3_App_Update_Char_1 */
switch (CharOpcode) {
	case BCONNECTION_SERVER_BLIST_CHAR:
		memcpy(blist_char_val_buffer,pData->p_Payload, MIN(pData->Length, sizeof(blist_char_val_buffer)));
		break;
	default:
		break;
}
  /* USER CODE END Service3_App_Update_Char_1 */

  switch(CharOpcode)
  {
    case BCONNECTION_SERVER_BLIST_STATE_CHAR:
      memcpy(blist_state_char_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(blist_state_char_val_buffer)));
      /* USER CODE BEGIN Service3_Char_Value_1*/

      /* USER CODE END Service3_Char_Value_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Service3_App_Update_Char_2 */

  /* USER CODE END Service3_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic notification
 * @param  CharOpcode: Characteristic identifier
 * @param  pData: pointer to the data to be notified to the client
 * @param  ConnectionHandle: connection handle identifying the client to be notified.
 *
 */
tBleStatus BCONNECTION_SERVER_NotifyValue(BCONNECTION_SERVER_CharOpcode_t CharOpcode, BCONNECTION_SERVER_Data_t *pData, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Service3_App_Notify_Char_1 */

  /* USER CODE END Service3_App_Notify_Char_1 */

  switch(CharOpcode)
  {

    case BCONNECTION_SERVER_BLIST_STATE_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                BCONNECTION_SERVER_Context.Blist_State_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify BLIST_STATE_CHAR command, error code: 0x%2X\n", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_srv_notify BLIST_STATE_CHAR command (state=%d)\n", pData->p_Payload[0]);
      }
      break;

    case BCONNECTION_SERVER_BLIST_CHAR:
      ret = aci_gatt_srv_notify(ConnectionHandle,
                                BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                BCONNECTION_SERVER_Context.Blist_CharCharHdle + 1,
                                GATT_NOTIFICATION,
                                pData->Length, /* charValueLen */
                                (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_srv_notify BLIST_CHAR command, error code: 0x%2X\n", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_srv_notify BLIST_CHAR command\n");
      }
      /* USER CODE BEGIN Service3_Char_Value_2*/

      /* USER CODE END Service3_Char_Value_2*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Service3_App_Notify_Char_2 */

  /* USER CODE END Service3_App_Notify_Char_2 */

  return ret;
}
