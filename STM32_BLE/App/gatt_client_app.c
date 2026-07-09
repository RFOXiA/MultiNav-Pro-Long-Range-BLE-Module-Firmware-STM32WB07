/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gatt_client_app.c
  * @author  MCD Application Team
  * @brief   GATT Client Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ble.h"
#include "gatt_client_app.h"
#include "stm32_seq.h"
#include "app_ble.h"
#include "ble_evt.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dual_role_manager.h"
#include "chat_server.h"
#include "chat_server_app.h"
#include "sensor_server_app.h"  /* For temperature relay function */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* Subscription state machine for sequential sensor subscriptions */
typedef enum {
  SUB_STATE_IDLE = 0,
  SUB_STATE_TEMP,
  SUB_STATE_HUM,
  SUB_STATE_PRS,
  SUB_STATE_AQI,
  SUB_STATE_GNSS,
  SUB_STATE_COMPLETE
} SubscriptionState_t;

typedef struct {
  SubscriptionState_t state;
  uint16_t peer_conn_handle;
  uint8_t subscribed_count;
  uint8_t waiting_for_response;
} SubscriptionContext_t;

/* USER CODE END PTD */

typedef enum
{
  NOTIFICATION_INFO_RECEIVED_EVT,
  /* USER CODE BEGIN GATT_CLIENT_APP_Opcode_t */

  /* USER CODE END GATT_CLIENT_APP_Opcode_t */
}GATT_CLIENT_APP_Opcode_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t length;
}GATT_CLIENT_APP_Data_t;

typedef struct
{
  GATT_CLIENT_APP_Opcode_t Client_Evt_Opcode;
  GATT_CLIENT_APP_Data_t   DataTransfered;
}GATT_CLIENT_APP_Notification_evt_t;

typedef struct
{
  GATT_CLIENT_APP_State_t state;

  APP_BLE_ConnStatus_t connStatus;
  uint16_t connHdl;

  uint16_t ALLServiceStartHdl;
  uint16_t ALLServiceEndHdl;

  uint16_t GAPServiceStartHdl;
  uint16_t GAPServiceEndHdl;

  uint16_t GATTServiceStartHdl;
  uint16_t GATTServiceEndHdl;

  uint16_t ServiceChangedCharStartHdl;
  uint16_t ServiceChangedCharValueHdl;
  uint16_t ServiceChangedCharDescHdl;
  uint16_t ServiceChangedCharEndHdl;

  /* USER CODE BEGIN BleClientAppContext_t */

/* USER CODE END BleClientAppContext_t */

}BleClientAppContext_t;

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static BleClientAppContext_t a_ClientContext[GATT_CLT_MAX_NUM_SERVERS];
static uint16_t gattCharStartHdl = 0;
static uint16_t gattCharValueHdl = 0;

/* USER CODE BEGIN PV */
static SubscriptionContext_t g_sub_ctx = {
  .state = SUB_STATE_IDLE,
  .peer_conn_handle = 0xFFFF,
  .subscribed_count = 0,
  .waiting_for_response = 0
};
/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/

/* USER CODE BEGIN GV */
extern uint8_t scan;
/* USER CODE END GV */

/* Private function prototypes -----------------------------------------------*/

static BLEEVT_EvtAckStatus_t P2P_ROUTER_EventHandler(aci_blecore_event *p_evt);
static void gatt_parse_services(aci_att_clt_read_by_group_type_resp_event_rp0 *p_evt);
static void gatt_parse_services_by_UUID(aci_att_clt_find_by_type_value_resp_event_rp0 *p_evt);
static void gatt_parse_chars(aci_att_clt_read_by_type_resp_event_rp0 *p_evt);
static void gatt_parse_descs(aci_att_clt_find_info_resp_event_rp0 *p_evt);
static void gatt_parse_notification(aci_gatt_clt_notification_event_rp0 *p_evt);
static void gatt_Notification(GATT_CLIENT_APP_Notification_evt_t *p_Notif);
static void client_discover_all(void);
static void gatt_cmd_resp_release(void);
static void gatt_cmd_resp_wait(void);
/* USER CODE BEGIN PFP */
/* scanStart() is now declared in gatt_client_app.h as public function */
static void GATT_CLIENT_SubscribeNextSensor(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void GATT_CLIENT_APP_Init(void)
{
  uint8_t index =0;
  int ret;

  /* USER CODE BEGIN GATT_CLIENT_APP_Init_1 */

  /* USER CODE END GATT_CLIENT_APP_Init_1 */

  for(index = 0; index < GATT_CLT_MAX_NUM_SERVERS; index++)
  {
    a_ClientContext[index].connStatus = APP_BLE_IDLE;
  }

  /**
   *  Register the event handler to the BLE controller
   */
  ret = BLEEVT_RegisterGattEvtHandler(P2P_ROUTER_EventHandler);
  if (ret != 0)
  {
    Error_Handler();
  }
  /* Register a task allowing to discover all services and characteristics and enable all notifications */
  UTIL_SEQ_RegTask(1U << CFG_TASK_DISCOVER_SERVICES_ID, UTIL_SEQ_RFU, client_discover_all);

  /* USER CODE BEGIN GATT_CLIENT_APP_Init_2 */

  UTIL_SEQ_RegTask( 1U << START_SCAN, UTIL_SEQ_RFU, scanStart);
  APP_DBG_MSG("___Scan task set \n");

  /* USER CODE END GATT_CLIENT_APP_Init_2 */
  return;
}

void GATT_CLIENT_APP_Notification(GATT_CLIENT_APP_ConnHandle_Notif_evt_t *p_Notif)
{
  /* USER CODE BEGIN GATT_CLIENT_APP_Notification_1 */

  /* USER CODE END GATT_CLIENT_APP_Notification_1 */
  switch(p_Notif->ConnOpcode)
  {
    /* USER CODE BEGIN ConnOpcode */

    /* USER CODE END ConnOpcode */

    case PEER_CONN_HANDLE_EVT :
      /* USER CODE BEGIN PEER_CONN_HANDLE_EVT */

      /* USER CODE END PEER_CONN_HANDLE_EVT */
      break;

    case PEER_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */

      /* USER CODE END PEER_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN ConnOpcode_Default */

      /* USER CODE END ConnOpcode_Default */
      break;
  }
  /* USER CODE BEGIN GATT_CLIENT_APP_Notification_2 */

  /* USER CODE END GATT_CLIENT_APP_Notification_2 */
  return;
}

uint8_t GATT_CLIENT_APP_Set_Conn_Handle(uint8_t index, uint16_t connHdl)
{
  uint8_t ret;

  if (index < GATT_CLT_MAX_NUM_SERVERS)
  {
    a_ClientContext[index].connHdl = connHdl;
    ret = 0;
  }
  else
  {
    ret = 1;
  }

  return ret;
}

uint8_t GATT_CLIENT_APP_Clear_Conn_Handle(uint8_t index)
{
  uint8_t ret;

  if (index < GATT_CLT_MAX_NUM_SERVERS)
  {
    a_ClientContext[index].connHdl = 0xFFFF;
    a_ClientContext[index].state = GATT_CLIENT_APP_IDLE;
    ret = 0;
  }
  else
  {
    ret = 1;
  }
  return ret;
}

uint8_t GATT_CLIENT_APP_Get_State(uint8_t index)
{
  return a_ClientContext[index].state;
}

void GATT_CLIENT_APP_Discover_services(uint8_t index)
{
  GATT_CLIENT_APP_Procedure_Gatt(index, PROC_GATT_DISC_ALL_PRIMARY_SERVICES);
  GATT_CLIENT_APP_Procedure_Gatt(index, PROC_GATT_DISC_ALL_CHARS);
  GATT_CLIENT_APP_Procedure_Gatt(index, PROC_GATT_DISC_ALL_DESCS);
  GATT_CLIENT_APP_Procedure_Gatt(index, PROC_GATT_ENABLE_ALL_NOTIFICATIONS);

  return;
}

uint8_t GATT_CLIENT_APP_Procedure_Gatt(uint8_t index, ProcGattId_t GattProcId)
{
  tBleStatus result = BLE_STATUS_SUCCESS;
  uint8_t status;

  if (index >= GATT_CLT_MAX_NUM_SERVERS)
  {
    status = 1;
  }
  else
  {
    status = 0;
    switch (GattProcId)
    {
      case PROC_GATT_DISC_ALL_PRIMARY_SERVICES:
      {
        a_ClientContext[index].state = GATT_CLIENT_APP_DISCOVER_SERVICES;

        APP_DBG_MSG("GATT services discovery\n");
        result = aci_gatt_clt_disc_all_primary_services(a_ClientContext[index].connHdl, BLE_GATT_UNENHANCED_ATT_L2CAP_CID);

        if (result == BLE_STATUS_SUCCESS)
        {
          gatt_cmd_resp_wait();
          APP_DBG_MSG("PROC_GATT_CTL_DISC_ALL_PRIMARY_SERVICES services discovered Successfully\n\n");
        }
        else
        {
          APP_DBG_MSG("PROC_GATT_CTL_DISC_ALL_PRIMARY_SERVICES aci_gatt_clt_disc_all_primary_services cmd NOK status =0x%02X\n\n", result);
        }
      }

      break; /* PROC_GATT_DISC_ALL_PRIMARY_SERVICES */

      case PROC_GATT_DISC_ALL_CHARS:
      {
        a_ClientContext[index].state = GATT_CLIENT_APP_DISCOVER_CHARACS;

        APP_DBG_MSG("DISCOVER_ALL_CHARS ConnHdl=0x%04X ALLServiceHandle[0x%04X - 0x%04X]\n",
                          a_ClientContext[index].connHdl,
                          a_ClientContext[index].ALLServiceStartHdl,
                          a_ClientContext[index].ALLServiceEndHdl);
        result = aci_gatt_clt_disc_all_char_of_service(
                           a_ClientContext[index].connHdl,
                           BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                           a_ClientContext[index].ALLServiceStartHdl,
                           a_ClientContext[index].ALLServiceEndHdl);
        if (result == BLE_STATUS_SUCCESS)
        {
          gatt_cmd_resp_wait();
          APP_DBG_MSG("All characteristics discovered Successfully\n\n");
        }
        else
        {
          APP_DBG_MSG("All characteristics discovery Failed, status =0x%02X\n\n", result);
        }
      }
      break; /* PROC_GATT_DISC_ALL_CHARS */

      case PROC_GATT_DISC_ALL_DESCS:
      {
        a_ClientContext[index].state = GATT_CLIENT_APP_DISCOVER_WRITE_DESC;

        APP_DBG_MSG("DISCOVER_ALL_CHAR_DESCS [0x%04X - 0x%04X]\n",
                         a_ClientContext[index].ALLServiceStartHdl,
                         a_ClientContext[index].ALLServiceEndHdl);
        result = aci_gatt_clt_disc_all_char_desc(
                           a_ClientContext[index].connHdl,
						   BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                           a_ClientContext[index].ALLServiceStartHdl,
                           a_ClientContext[index].ALLServiceEndHdl);
        if (result == BLE_STATUS_SUCCESS)
        {
          gatt_cmd_resp_wait();
          APP_DBG_MSG("All characteristic descriptors discovered Successfully\n\n");
        }
        else
        {
          APP_DBG_MSG("All characteristic descriptors discovery Failed, status =0x%02X\n\n", result);
        }
      }
      break; /* PROC_GATT_DISC_ALL_DESCS */
      case PROC_GATT_ENABLE_ALL_NOTIFICATIONS:
      {
        uint16_t enable = 0x0001; /* Buffer must be kept valid for aci_gatt_clt_write until a gatt procedure complete is received. */

        if (a_ClientContext[index].ServiceChangedCharDescHdl != 0x0000)
        {
          result = aci_gatt_clt_write(a_ClientContext[index].connHdl,
                                      BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                      a_ClientContext[index].ServiceChangedCharDescHdl,
                                      2,
                                      (uint8_t *) &enable);
          gatt_cmd_resp_wait();
          APP_DBG_MSG(" ServiceChangedCharDescHdl =0x%04X\n",a_ClientContext[index].ServiceChangedCharDescHdl);
        }
        /* USER CODE BEGIN PROC_GATT_ENABLE_ALL_NOTIFICATIONS */

        /* USER CODE END PROC_GATT_ENABLE_ALL_NOTIFICATIONS */

        if (result == BLE_STATUS_SUCCESS)
        {
          APP_DBG_MSG("All notifications enabled Successfully\n\n");
        }
        else
        {
          APP_DBG_MSG("All notifications enabled Failed, status =0x%02X\n\n", result);
        }
      }
      break; /* PROC_GATT_ENABLE_ALL_NOTIFICATIONS */
    default:
      break;
    }
  }

  return status;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static BLEEVT_EvtAckStatus_t P2P_ROUTER_EventHandler(aci_blecore_event *p_evt)
{
  BLEEVT_EvtAckStatus_t return_value = BLEEVT_NoAck;
  GATT_CLIENT_APP_Notification_evt_t Notification;
  UNUSED(Notification);

  return_value = BLEEVT_NoAck;

  switch (p_evt->ecode)
  {
    case ACI_ATT_CLT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
    {
      aci_att_clt_read_by_group_type_resp_event_rp0 *p_evt_rsp = (void*)p_evt->data;
      gatt_parse_services((aci_att_clt_read_by_group_type_resp_event_rp0 *)p_evt_rsp);
    }
    break; /* ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE */
  case ACI_ATT_CLT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE:
    {
      aci_att_clt_find_by_type_value_resp_event_rp0 *p_evt_rsp = (void*) p_evt->data;
      gatt_parse_services_by_UUID((aci_att_clt_find_by_type_value_resp_event_rp0 *)p_evt_rsp);
    }
    break; /* ACI_ATT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE */
    case ACI_ATT_CLT_READ_BY_TYPE_RESP_VSEVT_CODE:
    {
      aci_att_clt_read_by_type_resp_event_rp0 *p_evt_rsp = (void*)p_evt->data;
      gatt_parse_chars((aci_att_clt_read_by_type_resp_event_rp0 *)p_evt_rsp);
    }
    break; /* ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE */
    case ACI_ATT_CLT_FIND_INFO_RESP_VSEVT_CODE:
    {
      aci_att_clt_find_info_resp_event_rp0 *p_evt_rsp = (void*)p_evt->data;
      gatt_parse_descs((aci_att_clt_find_info_resp_event_rp0 *)p_evt_rsp);
    }
    break; /* ACI_ATT_FIND_INFO_RESP_VSEVT_CODE */
    case ACI_GATT_CLT_NOTIFICATION_VSEVT_CODE:
    {
      aci_gatt_clt_notification_event_rp0 *p_evt_rsp = (void*)p_evt->data;
      gatt_parse_notification((aci_gatt_clt_notification_event_rp0 *)p_evt_rsp);
    }
    break;/* ACI_GATT_NOTIFICATION_VSEVT_CODE */
    case ACI_GATT_CLT_PROC_COMPLETE_VSEVT_CODE:
    {
      aci_gatt_clt_proc_complete_event_rp0 *p_evt_rsp = (void*)p_evt->data;
      
      /* USER CODE BEGIN ACI_GATT_CLT_PROC_COMPLETE_VSEVT_CODE */
      /* If we're in the middle of sensor subscriptions, trigger next state */
      if (g_sub_ctx.waiting_for_response && p_evt_rsp->Connection_Handle == g_sub_ctx.peer_conn_handle) {
        g_sub_ctx.waiting_for_response = 0;
        /* Trigger next subscription in state machine */
        GATT_CLIENT_SubscribeNextSensor();
      }
      /* USER CODE END ACI_GATT_CLT_PROC_COMPLETE_VSEVT_CODE */
      
      UNUSED(p_evt_rsp);
      gatt_cmd_resp_release();
    }
    break;/* ACI_GATT_PROC_COMPLETE_VSEVT_CODE */
    case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
    {
    aci_att_exchange_mtu_resp_event_rp0 *tx_pool_available;
    tx_pool_available = (aci_att_exchange_mtu_resp_event_rp0 *)p_evt->data;
    UNUSED(tx_pool_available);
    /* USER CODE BEGIN ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */

    /* USER CODE END ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */
    }
    break;/* ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE*/
    case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
    {
    aci_att_exchange_mtu_resp_event_rp0 * exchange_mtu_resp;
    exchange_mtu_resp = (aci_att_exchange_mtu_resp_event_rp0 *)p_evt->data;
    APP_DBG_MSG("  MTU exchanged size = %d\n",exchange_mtu_resp->MTU );
    UNUSED(exchange_mtu_resp);
    /* USER CODE BEGIN ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */

    /* USER CODE END ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */
    }
    break;
    /* USER CODE BEGIN BLECORE_EVT */

    /* USER CODE END BLECORE_EVT */
    default:
      APP_DBG_MSG("EVENT OPCODE: 0x%04X\n", p_evt->ecode);
      break;
  }/* end switch (p_evt->ecode) */

  return(return_value);
}

__USED static void gatt_Notification(GATT_CLIENT_APP_Notification_evt_t *p_Notif)
{
  /* USER CODE BEGIN gatt_Notification_1*/

  /* USER CODE END gatt_Notification_1 */
  switch (p_Notif->Client_Evt_Opcode)
  {
    /* USER CODE BEGIN Client_Evt_Opcode */

    /* USER CODE END Client_Evt_Opcode */

    case NOTIFICATION_INFO_RECEIVED_EVT:
      /* USER CODE BEGIN NOTIFICATION_INFO_RECEIVED_EVT */

      /* USER CODE END NOTIFICATION_INFO_RECEIVED_EVT */
      break;

    default:
      /* USER CODE BEGIN Client_Evt_Opcode_Default */

      /* USER CODE END Client_Evt_Opcode_Default */
      break;
  }
  /* USER CODE BEGIN gatt_Notification_2*/

  /* USER CODE END gatt_Notification_2 */
  return;
}

/**
* function of GATT service parse
*/
static void gatt_parse_services(aci_att_clt_read_by_group_type_resp_event_rp0 *p_evt)
{
  uint16_t uuid, ServiceStartHdl, ServiceEndHdl;
  uint8_t uuid_offset, uuid_size, uuid_short_offset;
  uint8_t i, idx, numServ, index;

  APP_DBG_MSG("ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE - ConnHdl=0x%04X\n",
                p_evt->Connection_Handle);

  for (index = 0 ; index < GATT_CLT_MAX_NUM_SERVERS ; index++)
  {
    if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
    {
      break;
    }
  }

  /* check connection handle related to response before processing */
  if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
  {
    /* Number of attribute value tuples */
    numServ = (p_evt->Data_Length) / p_evt->Attribute_Data_Length;

    /* event data in Attribute_Data_List contains:
    * 2 bytes for start handle
    * 2 bytes for end handle
    * 2 or 16 bytes data for UUID
    */
    if (p_evt->Attribute_Data_Length == 20) /* we are interested in the UUID is 128 bit.*/
    {
      idx = 16;                /*UUID index of 2 bytes read part in Attribute_Data_List */
      uuid_offset = 4;         /*UUID offset in bytes in Attribute_Data_List */
      uuid_size = 16;          /*UUID size in bytes */
      uuid_short_offset = 12;  /*UUID offset of 2 bytes read part in UUID field*/
    }
    if (p_evt->Attribute_Data_Length == 6) /* we are interested in the UUID is 16 bit.*/
    {
      idx = 4;
      uuid_offset = 4;
      uuid_size = 2;
      uuid_short_offset = 0;
    }
    UNUSED(idx);
    UNUSED(uuid_size);

    /* Loop on number of attribute value tuples */
    for (i = 0; i < numServ; i++)
    {
      ServiceStartHdl =  UNPACK_2_BYTE_PARAMETER(&p_evt->Attribute_Data_List[uuid_offset - 4]);
      ServiceEndHdl = UNPACK_2_BYTE_PARAMETER(&p_evt->Attribute_Data_List[uuid_offset - 2]);
      uuid = UNPACK_2_BYTE_PARAMETER(&p_evt->Attribute_Data_List[uuid_offset + uuid_short_offset]);
      APP_DBG_MSG("  %d/%d short UUID=0x%04X, handle [0x%04X - 0x%04X]",
                   i + 1, numServ, uuid, ServiceStartHdl,ServiceEndHdl);

      /* complete context fields */
      if ( (a_ClientContext[index].ALLServiceStartHdl == 0x0000) || (ServiceStartHdl < a_ClientContext[index].ALLServiceStartHdl) )
      {
        a_ClientContext[index].ALLServiceStartHdl = ServiceStartHdl;
      }
      if ( (a_ClientContext[index].ALLServiceEndHdl == 0x0000) || (ServiceEndHdl > a_ClientContext[index].ALLServiceEndHdl) )
      {
        a_ClientContext[index].ALLServiceEndHdl = ServiceEndHdl;
      }

      if (uuid == GAP_SERVICE_UUID)
      {
        a_ClientContext[index].GAPServiceStartHdl = ServiceStartHdl;
        a_ClientContext[index].GAPServiceEndHdl = ServiceEndHdl;

        APP_DBG_MSG(", GAP_SERVICE_UUID found\n");
      }
      else if (uuid == GATT_SERVICE_UUID)
      {
        a_ClientContext[index].GATTServiceStartHdl = ServiceStartHdl;
        a_ClientContext[index].GATTServiceEndHdl = ServiceEndHdl;

        APP_DBG_MSG(", GENERIC_ATTRIBUTE_SERVICE_UUID found\n");
      }
/* USER CODE BEGIN gatt_parse_services_1 */

/* USER CODE END gatt_parse_services_1 */
      else
      {
        APP_DBG_MSG("\n");
      }

      uuid_offset += p_evt->Attribute_Data_Length;
    }
  }
  else
  {
    APP_DBG_MSG("ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE, failed no free index in connection table !\n");
  }

  return;
}

/**
* function of GATT service parse by UUID
*/
static void gatt_parse_services_by_UUID(aci_att_clt_find_by_type_value_resp_event_rp0 *p_evt)
{
  uint8_t i;

  APP_DBG_MSG("ACI_ATT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE - ConnHdl=0x%04X, Num_of_Handle_Pair=%d\n",
                p_evt->Connection_Handle,
                p_evt->Num_of_Handle_Pair);

  for(i = 0 ; i < p_evt->Num_of_Handle_Pair ; i++)
  {
    APP_DBG_MSG("ACI_ATT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE - PaitId=%d Found_Attribute_Handle=0x%04X, Group_End_Handle=0x%04X\n",
                  i,
                  p_evt->Attribute_Group_Handle_Pair[i].Found_Attribute_Handle,
                  p_evt->Attribute_Group_Handle_Pair[i].Group_End_Handle);
  }

/* USER CODE BEGIN gatt_parse_services_by_UUID_1 */

/* USER CODE END gatt_parse_services_by_UUID_1 */

  return;
}

/**
* function of GATT characteristics parse
*/
static void gatt_parse_chars(aci_att_clt_read_by_type_resp_event_rp0 *p_evt)
{
  uint16_t uuid, CharStartHdl, CharValueHdl;
  uint8_t uuid_offset, uuid_size, uuid_short_offset;
  uint8_t i, idx, numHdlValuePair, index;
  uint8_t CharProperties;

  APP_DBG_MSG("ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE - ConnHdl=0x%04X\n",
                p_evt->Connection_Handle);

  for (index = 0 ; index < GATT_CLT_MAX_NUM_SERVERS ; index++)
  {
    if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
    {
      break;
    }
  }

  if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
  {
    /* event data in Attribute_Data_List contains:
    * 2 bytes for start handle
    * 1 byte char properties
    * 2 bytes handle
    * 2 or 16 bytes data for UUID
    */

    /* Number of attribute value tuples */
    numHdlValuePair = p_evt->Data_Length / p_evt->Handle_Value_Pair_Length;

    if (p_evt->Handle_Value_Pair_Length == 21) /* we are interested in  128 bit UUIDs */
    {
      idx = 17;                /* UUID index of 2 bytes read part in Attribute_Data_List */
      uuid_offset = 5;         /* UUID offset in bytes in Attribute_Data_List */
      uuid_size = 16;          /* UUID size in bytes */
      uuid_short_offset = 12;  /* UUID offset of 2 bytes read part in UUID field */
    }
    if (p_evt->Handle_Value_Pair_Length == 7) /* we are interested in  16 bit UUIDs */
    {
      idx = 5;
      uuid_offset = 5;
      uuid_size = 2;
      uuid_short_offset = 0;
    }
    UNUSED(idx);
    UNUSED(uuid_size);

    p_evt->Data_Length -= 1;

    APP_DBG_MSG("  ConnHdl=0x%04X, number of value pair = %d\n", a_ClientContext[index].connHdl, numHdlValuePair);
    /* Loop on number of attribute value tuples */
    for (i = 0; i < numHdlValuePair; i++)
    {
      CharStartHdl = UNPACK_2_BYTE_PARAMETER(&p_evt->Handle_Value_Pair_Data[uuid_offset - 5]);
      CharProperties = p_evt->Handle_Value_Pair_Data[uuid_offset - 3];
      CharValueHdl = UNPACK_2_BYTE_PARAMETER(&p_evt->Handle_Value_Pair_Data[uuid_offset - 2]);
      uuid = UNPACK_2_BYTE_PARAMETER(&p_evt->Handle_Value_Pair_Data[uuid_offset + uuid_short_offset]);

      if ( (uuid != 0x0) && (CharProperties != 0x0) && (CharStartHdl != 0x0) && (CharValueHdl != 0) )
      {
        APP_DBG_MSG("    %d/%d short UUID=0x%04X, Properties=0x%04X, CharHandle [0x%04X - 0x%04X]",
                     i + 1, numHdlValuePair, uuid, CharProperties, CharStartHdl, CharValueHdl);

        if (uuid == DEVICE_NAME_UUID)
        {
          APP_DBG_MSG(", GAP DEVICE_NAME charac found\n");
        }
        else if (uuid == APPEARANCE_UUID)
        {
          APP_DBG_MSG(", GAP APPEARANCE charac found\n");
        }
        else if (uuid == SERVICE_CHANGED_UUID)
        {
          a_ClientContext[index].ServiceChangedCharStartHdl = CharStartHdl;
          a_ClientContext[index].ServiceChangedCharValueHdl = CharValueHdl;
          APP_DBG_MSG(", GATT SERVICE_CHANGED_CHARACTERISTIC_UUID charac found\n");
        }
/* USER CODE BEGIN gatt_parse_chars_1 */

/* USER CODE END gatt_parse_chars_1 */
        else
        {
          APP_DBG_MSG("\n");
        }

      }
      uuid_offset += p_evt->Handle_Value_Pair_Length;
    }
  }
  else
  {
    APP_DBG_MSG("ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE, failed handle not found in connection table !\n");
  }

  return;
}
/**
* function of GATT descriptor parse
*/
static void gatt_parse_descs(aci_att_clt_find_info_resp_event_rp0 *p_evt)
{
  uint16_t uuid, handle;
  uint8_t uuid_offset, uuid_size, uuid_short_offset;
  uint8_t i, numDesc, handle_uuid_pair_size, index;

  APP_DBG_MSG("ACI_ATT_FIND_INFO_RESP_VSEVT_CODE - ConnHdl=0x%04X\n",
              p_evt->Connection_Handle);

  for (index = 0 ; index < GATT_CLT_MAX_NUM_SERVERS ; index++)
  {
    if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
    {
      break;
    }
  }

  if (a_ClientContext[index].connHdl == p_evt->Connection_Handle)
  {
    /* event data in Attribute_Data_List contains:
    * 2 bytes handle
    * 2 or 16 bytes data for UUID
    */
    if (p_evt->Format == UUID_TYPE_16)
    {
      uuid_size = 2;
      uuid_offset = 2;
      uuid_short_offset = 0;
      handle_uuid_pair_size = 4;
    }
    if (p_evt->Format == UUID_TYPE_128)
    {
      uuid_size = 16;
      uuid_offset = 2;
      uuid_short_offset = 12;
      handle_uuid_pair_size = 18;
    }
    UNUSED(uuid_size);

    /* Number of handle uuid pairs */
    numDesc = (p_evt->Event_Data_Length) / handle_uuid_pair_size;

    for (i = 0; i < numDesc; i++)
    {
      handle = UNPACK_2_BYTE_PARAMETER(&p_evt->Handle_UUID_Pair[uuid_offset - 2]);
      uuid = UNPACK_2_BYTE_PARAMETER(&p_evt->Handle_UUID_Pair[uuid_offset + uuid_short_offset]);

      if (uuid == PRIMARY_SERVICE_UUID)
      {
        APP_DBG_MSG("PRIMARY_SERVICE_UUID=0x%04X handle=0x%04X\n", uuid, handle);
      }
      else if (uuid == CHARACTERISTIC_UUID)
      {
        /* reset UUID & handle */
        gattCharStartHdl = 0;
        gattCharValueHdl = 0;

        gattCharStartHdl = handle;
        APP_DBG_MSG("reset - UUID & handle - CHARACTERISTIC_UUID=0x%04X CharStartHandle=0x%04X\n", uuid, handle);
      }
      else if ( (uuid == CHAR_EXTENDED_PROPERTIES_DESCRIPTOR_UUID)
             || (uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID) )
      {

        APP_DBG_MSG("Descriptor UUID=0x%04X, handle=0x%04X-0x%04X-0x%04X",
                      uuid,
                      gattCharStartHdl,
                      gattCharValueHdl,
                      handle);
        if (a_ClientContext[index].ServiceChangedCharValueHdl == gattCharValueHdl)
        {
          a_ClientContext[index].ServiceChangedCharDescHdl = handle;
          APP_DBG_MSG(", Service Changed found\n");
        }
/* USER CODE BEGIN gatt_parse_descs_1 */

/* USER CODE END gatt_parse_descs_1 */
        else
        {
          APP_DBG_MSG("\n");
        }
      }
      else
      {
        gattCharValueHdl = handle;

        APP_DBG_MSG("  UUID=0x%04X, handle=0x%04X", uuid, handle);

        if (uuid == DEVICE_NAME_UUID)
        {
          APP_DBG_MSG(", found GAP DEVICE_NAME_UUID\n");
        }
        else if (uuid == APPEARANCE_UUID)
        {
          APP_DBG_MSG(", found GAP APPEARANCE_UUID\n");
        }
        else if (uuid == SERVICE_CHANGED_UUID)
        {
          APP_DBG_MSG(", found GATT SERVICE_CHANGED_CHARACTERISTIC_UUID\n");
        }
/* USER CODE BEGIN gatt_parse_descs_2 */

/* USER CODE END gatt_parse_descs_2 */
        else
        {
          APP_DBG_MSG("\n");
        }
      }
    uuid_offset += handle_uuid_pair_size;
    }
  }
  else
  {
    APP_DBG_MSG("ACI_ATT_FIND_INFO_RESP_VSEVT_CODE, failed handle not found in connection table !\n");
  }

  return;
}

static void gatt_parse_notification(aci_gatt_clt_notification_event_rp0 *p_evt)
{
  /* LOGGING REMOVED: Prints on EVERY notification (every second) - blocks CPU ~8ms
   * During connection parameter negotiation, this delays firmware response to Android */
/* USER CODE BEGIN gatt_parse_notification_1 */
  
  /* Forward notifications from peer module to connected phone */
  /* Peer characteristic handles (notification VALUE handles, not CCCD handles) */
  const uint16_t PEER_CHAT_NOTIFY_HANDLE = CHAT_SERVER_GetNotifyCharValueHandle();  /* Chat Notify (identical firmware → same handle) */
  const uint16_t PEER_TEMP_VALUE_HANDLE = 0x0012;   /* Temperature VALUE (CCCD is 0x0013) */
  const uint16_t PEER_HUM_VALUE_HANDLE = 0x0015;    /* Humidity VALUE (CCCD is 0x0016) */
  const uint16_t PEER_PRS_VALUE_HANDLE = 0x0018;    /* Pressure VALUE (CCCD is 0x0019) */
  const uint16_t PEER_AQI_VALUE_HANDLE = 0x001B;    /* Air Quality VALUE (CCCD is 0x001C) */
  const uint16_t PEER_GNSS_VALUE_HANDLE = 0x001E;   /* GNSS VALUE (CCCD is 0x001F) */
  
  if (p_evt->Attribute_Handle == PEER_CHAT_NOTIFY_HANDLE)
  {
    /* This is a chat message notification from peer module
     * Forward it to the connected phone via our Chat Notify characteristic */
    
    DualRole_Context_t *ctx = DualRole_GetContext();
    if (ctx != NULL && ctx->phone_connected && ctx->phone_conn_handle != 0xFFFF)
    {
      /* Prepare data structure for notification */
      CHAT_SERVER_Data_t chat_data;
      chat_data.p_Payload = p_evt->Attribute_Value;
      chat_data.Length = p_evt->Attribute_Value_Length;
      
      /* Send notification to phone using public API */
      tBleStatus ret = CHAT_SERVER_NotifyValue(
        CHAT_SERVER_NOTIFY_CHAR,
        &chat_data,
        ctx->phone_conn_handle
      );
      
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("⚠️ Failed to relay peer message to phone: 0x%02X\n", ret);
      }
    }
  }
  else if (p_evt->Attribute_Handle == PEER_TEMP_VALUE_HANDLE)
  {
    /* Temperature notification from peer - relay with priority logic */
    /* LOG REMOVED: Prints every second, blocks CPU ~10ms during BLE operations */
    SENSOR_SERVER_RelayTempFromPeer(p_evt->Attribute_Value);
  }
  else if (p_evt->Attribute_Handle == PEER_HUM_VALUE_HANDLE)
  {
    /* Humidity notification from peer */
    SENSOR_SERVER_RelayHumFromPeer(p_evt->Attribute_Value);
  }
  else if (p_evt->Attribute_Handle == PEER_PRS_VALUE_HANDLE)
  {
    /* Pressure notification from peer */
    SENSOR_SERVER_RelayPrsFromPeer(p_evt->Attribute_Value);
  }
  else if (p_evt->Attribute_Handle == PEER_AQI_VALUE_HANDLE)
  {
    /* Air Quality notification from peer */
    SENSOR_SERVER_RelayAqiFromPeer(p_evt->Attribute_Value, p_evt->Attribute_Value_Length);
  }
  else if (p_evt->Attribute_Handle == PEER_GNSS_VALUE_HANDLE)
  {
    /* GNSS notification from peer */
    SENSOR_SERVER_RelayGnssFromPeer(p_evt->Attribute_Value, p_evt->Attribute_Value_Length);
  }
  else
  {
    /* Unknown handle - log for debugging */
    APP_DBG_MSG("⚠️ Notification from unknown handle 0x%04X (%d bytes)\n", 
                p_evt->Attribute_Handle, p_evt->Attribute_Value_Length);
  }

/* USER CODE END gatt_parse_notification_1 */

  return;
}

static void client_discover_all(void)
{
  /* Both modules run identical firmware so all sensor CCCD handles are known.
   * The standard discovery path (DISC_PRIMARY_SERVICES → DISC_CHARS → DISC_DESCS)
   * only finds GAP/GATT (handles 0x0001-0x000F) because the 128-bit UUID sensor
   * services arrive in a separate ATT response that races against the 0x08 disconnect.
   * Bypassing discovery and writing hardcoded CCCDs directly is faster, more
   * reliable, and eliminates the race condition. */
  uint16_t peer_conn_handle = a_ClientContext[0].connHdl;
  if (peer_conn_handle == 0xFFFF) {
    APP_DBG_MSG("\xE2\x9A\xA0\xEF\xB8\x8F client_discover_all: no peer handle set\n");
    return;
  }
  APP_DBG_MSG("\xF0\x9F\x93\xA1 Direct CCCD subscription to peer 0x%04X (skipping slow discovery)\n", peer_conn_handle);
  GATT_CLIENT_SubscribeToPeerTemperature(peer_conn_handle);
  return;
}

static void gatt_cmd_resp_release(void)
{
  UTIL_SEQ_SetEvt(1U << CFG_IDLEEVT_PROC_GATT_COMPLETE);
  return;
}

static void gatt_cmd_resp_wait(void)
{
  UTIL_SEQ_WaitEvt(1U << CFG_IDLEEVT_PROC_GATT_COMPLETE);
  return;
}

/* USER CODE BEGIN LF */
/* Made non-static for test scan from main.c */
void scanStart()
{
	/* Check if scan is already running */
	if (scan == 1) {
		APP_DBG_MSG("⚠️ Scan already in progress, ignoring duplicate request\n");
		return;
	}
	
	/* Check current dual-role state */
	DualRole_State_t current_state = DualRole_GetState();
	APP_DBG_MSG("📍 Current state before scan: %d\n", current_state);
	
	/* CRITICAL: Block scan if both connections are already established */
	if (current_state == DUAL_STATE_DUAL_ACTIVE) {
		APP_DBG_MSG("⚠️ Cannot scan: Both connections already established (state=6)\n");
		APP_DBG_MSG("   Peer is already connected at 0x%04X - no need to scan\n", DualRole_GetPeerHandle());
		return;  /* Abort scan */
	}
	
	/* Handle error state - reset to phone connected state */
	if (current_state == DUAL_STATE_ERROR) {
		APP_DBG_MSG("⚠️ Recovering from ERROR state\n");
		DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
		current_state = DUAL_STATE_PHONE_CONNECTED;
	}
	
	/* If already scanning, stop first */
	if (current_state == DUAL_STATE_SCANNING) {
		APP_DBG_MSG("⚠️ Already in SCANNING state, stopping existing scan first\n");
		APP_BLE_Procedure_Gap_Central(PROC_GAP_CENTRAL_SCAN_TERMINATE);
		HAL_Delay(100);
	}
	
	/* Clear device list before starting new scan */
	DualRole_ClearDeviceList();
	
	/* CRITICAL: Must stop advertising before scanning - BLE stack doesn't allow both */
	APP_DBG_MSG("🛑 Stopping advertising to enable scanning...\n");
	APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_STOP);
	
	/* Longer delay to ensure advertising fully stops - critical for scan success */
	HAL_Delay(100);
	
	/* Set dual-role state to SCANNING so DualRole_OnAdvertisingReport processes reports */
	DualRole_SetState(DUAL_STATE_SCANNING);
	
	/* Now start scanning - scan flag will be set by successful scan start event */
	APP_DBG_MSG("▶️ Starting BLE scan...\n");
	APP_BLE_Procedure_Gap_Central(PROC_GAP_CENTRAL_SCAN_START);
	
	/* Set scan flag to indicate scan is requested */
	scan=1;
}

/**
 * @brief  Start sequential sensor subscription to peer module
 * @note   Uses event-driven state machine to subscribe one sensor at a time
 * @param  peer_conn_handle: Connection handle of the peer module
 * @retval BLE status
 */
tBleStatus GATT_CLIENT_SubscribeToPeerTemperature(uint16_t peer_conn_handle)
{
  /* Guard: if already subscribing/subscribed to this same peer, skip restart.
   * This prevents double-subscription when both client_discover_all and the
   * subscription_pending path (triggered after MTU exchange) are active. */
  if (g_sub_ctx.state != SUB_STATE_IDLE &&
      g_sub_ctx.peer_conn_handle == peer_conn_handle) {
    APP_DBG_MSG("\xE2\x8F\xA9 Peer sensor subscription already in progress (state=%d, handle=0x%04X)\n",
                g_sub_ctx.state, peer_conn_handle);
    return BLE_STATUS_SUCCESS;
  }

  /* Initialize subscription state machine */
  g_sub_ctx.state = SUB_STATE_IDLE;
  g_sub_ctx.peer_conn_handle = peer_conn_handle;
  g_sub_ctx.subscribed_count = 0;
  g_sub_ctx.waiting_for_response = 0;
  
  APP_DBG_MSG("🔗 Starting sequential peer sensor subscriptions...\n");
  
  /* Start the state machine */
  GATT_CLIENT_SubscribeNextSensor();
  
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  State machine worker - subscribes to next sensor in sequence
 * @note   Called initially and then by ACI_GATT_CLT_PROC_COMPLETE event
 * @retval None
 */
static void GATT_CLIENT_SubscribeNextSensor(void)
{
  /* Get sensor availability status */
  SensorAvailability_t *status = SENSOR_SERVER_GetSensorStatus();
  
  /* Peer characteristic CCCD handles */
  const uint16_t PEER_TEMP_CCCD_HANDLE = 0x0013;
  const uint16_t PEER_HUM_CCCD_HANDLE = 0x0016;
  const uint16_t PEER_PRS_CCCD_HANDLE = 0x0019;
  const uint16_t PEER_AQI_CCCD_HANDLE = 0x001C;
  const uint16_t PEER_GNSS_CCCD_HANDLE = 0x001F;
  
  uint8_t enable_notification[] = {0x01, 0x00};
  tBleStatus ret;
  
  /* Advance to next state that needs subscription */
  while (g_sub_ctx.state < SUB_STATE_COMPLETE) {
    g_sub_ctx.state++;
    
    switch (g_sub_ctx.state) {
      case SUB_STATE_TEMP:
        if (!status->temp_available) {
          APP_DBG_MSG("  📡 [1/5] Temperature - subscribing...\n");
          ret = aci_gatt_clt_write(g_sub_ctx.peer_conn_handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                   PEER_TEMP_CCCD_HANDLE, sizeof(enable_notification), enable_notification);
          if (ret == BLE_STATUS_SUCCESS) {
            g_sub_ctx.waiting_for_response = 1;
            g_sub_ctx.subscribed_count++;
            return; /* Wait for ACI_GATT_CLT_PROC_COMPLETE event */
          } else {
            APP_DBG_MSG("     ❌ Write failed: 0x%02X\n", ret);
          }
        } else {
          APP_DBG_MSG("  ⏩ [1/5] Temperature - using local\n");
        }
        break;
        
      case SUB_STATE_HUM:
        if (!status->humidity_available) {
          APP_DBG_MSG("  📡 [2/5] Humidity - subscribing...\n");
          ret = aci_gatt_clt_write(g_sub_ctx.peer_conn_handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                   PEER_HUM_CCCD_HANDLE, sizeof(enable_notification), enable_notification);
          if (ret == BLE_STATUS_SUCCESS) {
            g_sub_ctx.waiting_for_response = 1;
            g_sub_ctx.subscribed_count++;
            return; /* Wait for event */
          } else {
            APP_DBG_MSG("     ❌ Write failed: 0x%02X\n", ret);
          }
        } else {
          APP_DBG_MSG("  ⏩ [2/5] Humidity - using local\n");
        }
        break;
        
      case SUB_STATE_PRS:
        if (!status->pressure_available) {
          APP_DBG_MSG("  📡 [3/5] Pressure - subscribing...\n");
          ret = aci_gatt_clt_write(g_sub_ctx.peer_conn_handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                   PEER_PRS_CCCD_HANDLE, sizeof(enable_notification), enable_notification);
          if (ret == BLE_STATUS_SUCCESS) {
            g_sub_ctx.waiting_for_response = 1;
            g_sub_ctx.subscribed_count++;
            return; /* Wait for event */
          } else {
            APP_DBG_MSG("     ❌ Write failed: 0x%02X\n", ret);
          }
        } else {
          APP_DBG_MSG("  ⏩ [3/5] Pressure - using local\n");
        }
        break;
        
      case SUB_STATE_AQI:
        if (!status->aqi_available) {
          APP_DBG_MSG("  📡 [4/5] Air Quality - subscribing...\n");
          ret = aci_gatt_clt_write(g_sub_ctx.peer_conn_handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                   PEER_AQI_CCCD_HANDLE, sizeof(enable_notification), enable_notification);
          if (ret == BLE_STATUS_SUCCESS) {
            g_sub_ctx.waiting_for_response = 1;
            g_sub_ctx.subscribed_count++;
            return; /* Wait for event */
          } else {
            APP_DBG_MSG("     ❌ Write failed: 0x%02X\n", ret);
          }
        } else {
          APP_DBG_MSG("  ⏩ [4/5] Air Quality - using local\n");
        }
        break;
        
      case SUB_STATE_GNSS:
        if (!status->gnss_available) {
          APP_DBG_MSG("  📡 [5/5] GNSS - subscribing...\n");
          ret = aci_gatt_clt_write(g_sub_ctx.peer_conn_handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                                   PEER_GNSS_CCCD_HANDLE, sizeof(enable_notification), enable_notification);
          if (ret == BLE_STATUS_SUCCESS) {
            g_sub_ctx.waiting_for_response = 1;
            g_sub_ctx.subscribed_count++;
            return; /* Wait for event */
          } else {
            APP_DBG_MSG("     ❌ Write failed: 0x%02X\n", ret);
          }
        } else {
          APP_DBG_MSG("  ⏩ [5/5] GNSS - using local\n");
        }
        break;
        
      case SUB_STATE_COMPLETE:
        /* All done */
        APP_DBG_MSG("✅ Sequential subscription complete: %d sensors subscribed\n", g_sub_ctx.subscribed_count);
        g_sub_ctx.state = SUB_STATE_IDLE;
        g_sub_ctx.peer_conn_handle = 0xFFFF;
        return;
        
      default:
        break;
    }
  }
  
  /* If we get here, we're done */
  APP_DBG_MSG("✅ All subscriptions processed: %d sensors\n", g_sub_ctx.subscribed_count);
  g_sub_ctx.state = SUB_STATE_IDLE;
  g_sub_ctx.peer_conn_handle = 0xFFFF;
}

/* USER CODE END LF */
