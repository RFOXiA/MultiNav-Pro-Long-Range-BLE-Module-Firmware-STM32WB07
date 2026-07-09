/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.c
  * @author  GPM WBL Application Team
  * @brief   BLE Application
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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32wb0x.h"
#include "ble.h"
#include "gatt_profile.h"
#include "gap_profile.h"
#include "app_ble.h"
#include "stm32wb0x_hal_radio_timer.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "blenvm.h"
#include "pka_manager.h"
#include "stm32_seq.h"
#include "sensor_server.h"
#include "sensor_server_app.h"
#include "motor_server.h"
#include "motor_server_app.h"
#include "bconnection_server.h"
#include "bconnection_server_app.h"
#include "chat_server.h"
#include "chat_server_app.h"
#include "gatt_client_app.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dual_role_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/**
 * Security parameters structure
 */
typedef struct
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
  /* USER CODE BEGIN tSecurityParams*/

  /* USER CODE END tSecurityParams */
}SecurityParams_t;

/**
 * Global context contains all BLE common variables.
 */
typedef struct
{
  /**
   * security requirements of the host
   */
  SecurityParams_t bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;
  /* USER CODE BEGIN BleGlobalContext_t*/
  uint8_t numOfConnections;
  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;
  /* USER CODE BEGIN PTD_1*/
  uint8_t deviceServerFound;
   uint8_t deviceServerBdAddrType;
   uint8_t a_deviceServerBdAddr[BD_ADDR_SIZE];
  /* USER CODE END PTD_1 */
}BleApplicationContext_t;

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/

NO_INIT(uint32_t dyn_alloc_a[BLE_DYN_ALLOC_SIZE>>2]);

static BleApplicationContext_t bleAppContext;

SENSOR_SERVER_APP_ConnHandleNotEvt_t SENSOR_SERVERHandleNotification;

MOTOR_SERVER_APP_ConnHandleNotEvt_t MOTOR_SERVERHandleNotification;

BCONNECTION_SERVER_APP_ConnHandleNotEvt_t BCONNECTION_SERVERHandleNotification;

GATT_CLIENT_APP_ConnHandle_Notif_evt_t clientHandleNotification;

static const char a_GapDeviceName[] = {  'R', 'F', 'O', 'X', 'i', 'A', ' ', 'B', 'L', 'E' }; /* Gap Device Name */

/**
 * Advertising Data
 */
uint8_t a_AdvData[] =
{
  2, AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  11, AD_TYPE_COMPLETE_LOCAL_NAME, 'R', 'F', 'O', 'X', 'i', 'A', ' ', 'B', 'L', 'E',  /* Complete name */
  /* Manufacturer Specific Data (AD type 0xFF) — carries the device's own BD address so   */
  /* iOS apps can display the real MAC (iOS does not expose BLE MAC addresses via CoreBluetooth). */
  /* Android apps already obtain the MAC from device.address and can safely ignore this field.  */
  /* Format: [len=9][0xFF][companyID_lo=0xFF][companyID_hi=0xFF][mac 6 bytes little-endian]    */
  /* Bytes [19..24] are patched at runtime with bd_address[] before advertising starts.         */
  9, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* MAC placeholder, patched below */
};

/* USER CODE BEGIN PV */
uint8_t scan;
static uint16_t new_ConnHdl;

/* Pending connection classification (for device name read) */
uint16_t g_pending_classify_handle = 0xFFFF;
uint8_t g_pending_classify_addr[6];
uint8_t g_pending_classify_addr_type;

/* Pending connection classification */
static uint16_t pending_conn_handle = 0xFFFF;
static uint8_t pending_peer_addr[6];
static uint8_t pending_peer_addr_type;
static uint8_t pending_role;
/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/
uint8_t bd_address[6] = {0};  /* BLE address for module identification */

/* USER CODE BEGIN GV */
extern uint8_t filter_name[100];
extern uint8_t filter_name_size;
/* USER CODE END GV */

/* Private function prototypes -----------------------------------------------*/
static void connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Connection_Interval,
                                      uint16_t Peripheral_Latency,
                                      uint16_t Supervision_Timeout);
static void gap_cmd_resp_wait(void);
static void gap_cmd_resp_release(void);

/* USER CODE BEGIN PFP */
static uint8_t analyse_adv_report(uint8_t adv_data_size, uint8_t *p_adv_data,
                                  uint8_t address_type, uint8_t *p_address);
static void Connect_Request(void);
static void Classify_Connection_Task(void);
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
uint8_t blst_devices_num=0;
uint8_t blst_index=1;
uint8_t blst[255]={'\0'};
/* USER CODE END EV */

/* Private functions ---------------------------------------------------------*/

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/* Functions Definition ------------------------------------------------------*/
void ModulesInit(void)
{
  BLENVM_Init();
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
    Error_Handler();
  }
}

void BLE_Init(void)
{
  uint8_t role;
  uint8_t privacy_type = 0;
  tBleStatus ret;
  uint16_t gatt_service_changed_handle;
  uint16_t gap_dev_name_char_handle;
  uint16_t gap_appearance_char_handle;
  uint16_t gap_periph_pref_conn_param_char_handle;
  /* bd_address is now global for module identification */
  uint8_t bd_address_len= 6;
  uint16_t appearance = CFG_GAP_APPEARANCE;

  /* CRITICAL: Debug output to verify configuration at compile time */
  APP_DBG_MSG("🔧 BLE Stack Config: NumRadioTasks=%d, MBLOCKS=%d, BufferSize=%d\n",
              CFG_BLE_NUM_RADIO_TASKS, CFG_BLE_MBLOCKS_COUNT, BLE_DYN_ALLOC_SIZE);

  BLE_STACK_InitTypeDef BLE_STACK_InitParams = {
    .BLEStartRamAddress = (uint8_t*)dyn_alloc_a,
    .TotalBufferSize = BLE_DYN_ALLOC_SIZE,
    .NumAttrRecords = CFG_BLE_NUM_GATT_ATTRIBUTES,
    .MaxNumOfClientProcs = CFG_BLE_NUM_OF_CONCURRENT_GATT_CLIENT_PROC,
    .NumOfRadioTasks = CFG_BLE_NUM_RADIO_TASKS,
    .NumOfEATTChannels = CFG_BLE_NUM_EATT_CHANNELS,
    .NumBlockCount = CFG_BLE_MBLOCKS_COUNT,
    .ATT_MTU = CFG_BLE_ATT_MTU_MAX,
    .MaxConnEventLength = CFG_BLE_CONN_EVENT_LENGTH_MAX,
    .SleepClockAccuracy = CFG_BLE_SLEEP_CLOCK_ACCURACY,
    .NumOfAdvDataSet = CFG_BLE_NUM_ADV_SETS,
    .NumOfSubeventsPAwR = CFG_BLE_NUM_PAWR_SUBEVENTS,
    .MaxPAwRSubeventDataCount = CFG_BLE_PAWR_SUBEVENT_DATA_COUNT_MAX,
    .NumOfAuxScanSlots = CFG_BLE_NUM_AUX_SCAN_SLOTS,
    .FilterAcceptListSizeLog2 = CFG_BLE_FILTER_ACCEPT_LIST_SIZE_LOG2,
    .L2CAP_MPS = CFG_BLE_COC_MPS_MAX,
    .L2CAP_NumChannels = CFG_BLE_COC_NBR_MAX,
    .NumOfSyncSlots = CFG_BLE_NUM_SYNC_SLOTS,
    .CTE_MaxNumAntennaIDs = CFG_BLE_NUM_CTE_ANTENNA_IDS_MAX,
    .CTE_MaxNumIQSamples = CFG_BLE_NUM_CTE_IQ_SAMPLES_MAX,
    .NumOfSyncBIG = CFG_BLE_NUM_SYNC_BIG_MAX,
    .NumOfBrcBIG = CFG_BLE_NUM_BRC_BIG_MAX,
    .NumOfSyncBIS = CFG_BLE_NUM_SYNC_BIS_MAX,
    .NumOfBrcBIS = CFG_BLE_NUM_BRC_BIS_MAX,
    .NumOfCIG = CFG_BLE_NUM_CIG_MAX,
    .NumOfCIS = CFG_BLE_NUM_CIS_MAX,
    .isr0_fifo_size = CFG_BLE_ISR0_FIFO_SIZE,
    .isr1_fifo_size = CFG_BLE_ISR1_FIFO_SIZE,
    .user_fifo_size = CFG_BLE_USER_FIFO_SIZE
  };

  /* Bluetooth LE stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    APP_DBG_MSG("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    Error_Handler();
  }

#if (CFG_BD_ADDRESS_TYPE == HCI_ADDR_PUBLIC)

  bd_address[0] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0x0000000000FF));
  bd_address[1] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0x00000000FF00) >> 8);
  bd_address[2] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0x000000FF0000) >> 16);
  bd_address[3] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0x0000FF000000) >> 24);
  bd_address[4] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0x00FF00000000) >> 32);
  bd_address[5] = (uint8_t)((CFG_PUBLIC_BD_ADDRESS & 0xFF0000000000) >> 40);
  (void)bd_address_len;

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bd_address);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET\n");
  }
#endif

  /**
   * Set TX Power.
   */
  ret = aci_hal_set_tx_power_level(0, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_srv_profile_init(GATT_INIT_SERVICE_CHANGED_BIT, &gatt_service_changed_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_srv_profile_init command, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_srv_profile_init command\n");
  }

  /**
   * Initialize GAP interface
   */
  role = 0U;
  /* CRITICAL FIX: Use ONLY GAP_CENTRAL_ROLE (sub firmware approach)
   * Setting BOTH PERIPHERAL+CENTRAL causes resource conflicts (0x12 error)
   * Advertising/peripheral connections still work with CENTRAL_ONLY role!
   * This matches the working p2pRouter sub firmware configuration.
   */
  role |= GAP_CENTRAL_ROLE;     /* Initiate outgoing connections (to peer) */
  
  APP_DBG_MSG("📋 GAP Role: CENTRAL_ONLY (value=0x%02X) - sub firmware approach\n", role);
  APP_DBG_MSG("   (Peripheral connections via advertising still work)\n");
  
#if CFG_BLE_PRIVACY_ENABLED
  privacy_type = 0x02;
#endif

/* USER CODE BEGIN Role_Mngt*/

/* USER CODE END Role_Mngt */

  ret = aci_gap_init(privacy_type, CFG_BD_ADDRESS_TYPE);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_init command\n");
  }

  ret = aci_gap_profile_init(role, privacy_type,
                             &gap_dev_name_char_handle,
                             &gap_appearance_char_handle,
                             &gap_periph_pref_conn_param_char_handle);

#if (CFG_BD_ADDRESS_TYPE == HCI_ADDR_STATIC_RANDOM_ADDR)
  ret = aci_hal_read_config_data(CONFIG_DATA_STORED_STATIC_RANDOM_ADDRESS,
                                 &bd_address_len, bd_address);
  APP_DBG_MSG("  Static Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n",bd_address[5],bd_address[4],bd_address[3],bd_address[2],bd_address[1],bd_address[0]);
#elif (CFG_BD_ADDRESS_TYPE == HCI_ADDR_PUBLIC)
  APP_DBG_MSG("  Public Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n",bd_address[5],bd_address[4],bd_address[3],bd_address[2],bd_address[1],bd_address[0]);
#else
#error "Invalid CFG_BD_ADDRESS_TYPE"
#endif

  ret = Gap_profile_set_dev_name(0, sizeof(a_GapDeviceName), (uint8_t*)a_GapDeviceName);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : Gap_profile_set_dev_name - Device Name, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: Gap_profile_set_dev_name - Device Name\n");
  }

  ret = Gap_profile_set_appearance(0, sizeof(appearance), (uint8_t*)&appearance);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : Gap_profile_set_appearance - Appearance, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: Gap_profile_set_appearance - Appearance\n");
  }

  /* CRITICAL: Enable Coded PHY for long-range support
   * Disable 2M PHY to avoid conflicts, but ENABLE Coded PHY for long range
   * TX_PHYS: 0x01 (1M) | 0x04 (Coded) = 0x05
   * This allows PHY updates to switch to Coded S=8 for 400-1500m range
   */
  APP_DBG_MSG("🔧 Setting default PHY to 1M + Coded (disable 2M, enable long range)...\n");
  ret = hci_le_set_default_phy(
    0x00,           /* ALL_PHYS: 0 = use TX_PHYS and RX_PHYS preferences */
    0x05,           /* TX_PHYS: bit 0 (1M) + bit 2 (Coded) = 0x05 */
    0x05            /* RX_PHYS: bit 0 (1M) + bit 2 (Coded) = 0x05 */
  );
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  WARN: hci_le_set_default_phy failed: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  SUCCESS: Default PHY set to 1M + Coded (long-range enabled)\n");
  }

  /* NOTE: Connection configuration will be done at RUNTIME (sub firmware approach)
   * Sub firmware calls aci_gap_set_connection_configuration() in DualRole_Peer_Connect_Task()
   * right before aci_gap_create_connection() - NOT during init!
   * See: sub firmware/STM32_BLE/App/dual_role_app.c lines 277-287
   * Calling it during init fails with 0x12 - API seems to require different controller state
   */
  APP_DBG_MSG("📝 Connection parameters will be configured at runtime (sub firmware approach)\n");

  /**
   * Initialize IO capability
   */
  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(bleAppContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  /**
   * Initialize authentication
   */
  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode             = CFG_MITM_PROTECTION;
  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin  = CFG_ENCRYPTION_KEY_SIZE_MIN;
  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax  = CFG_ENCRYPTION_KEY_SIZE_MAX;
  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode          = CFG_BONDING_MODE;

  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/
  ret = aci_gap_set_security_requirements(bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               bleAppContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               GAP_PAIRING_RESP_NONE);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_security_requirements command, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_security_requirements command\n");
  }

  /**
   * Initialize Filter Accept List
   * COMMENTED OUT to allow peer discovery - we don't use filter accept list
   */
  /* DISABLED FOR PEER DISCOVERY
  if (bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_filter_accept_and_resolving_list(0x01);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_filter_accept_and_resolving_list command, result: 0x%02X\n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_filter_accept_and_resolving_list command\n");
    }
  }
  */
  APP_DBG_MSG("  Filter Accept List: DISABLED for peer discovery\n");
  APP_DBG_MSG("==>> End BLE_Init function\n");

}

void BLEStack_Process_Schedule(void)
{
  /* Keep BLE Stack Process priority low, since there are limited cases
     where stack wants to be rescheduled for busy waiting.  */
  UTIL_SEQ_SetTask( 1U << CFG_TASK_BLE_STACK, CFG_SEQ_PRIO_1);
}
static void BLEStack_Process(void)
{
  APP_DEBUG_SIGNAL_SET(APP_STACK_PROCESS);
  BLE_STACK_Tick();

  APP_DEBUG_SIGNAL_RESET(APP_STACK_PROCESS);
}

void VTimer_Process(void)
{
  HAL_RADIO_TIMER_Tick();
}

void VTimer_Process_Schedule(void)
{
  UTIL_SEQ_SetTask( 1U << CFG_TASK_VTIMER, CFG_SEQ_PRIO_0);
}
void NVM_Process(void)
{
  NVMDB_Tick();
}

void NVM_Process_Schedule(void)
{
  UTIL_SEQ_SetTask( 1U << CFG_TASK_NVM, CFG_SEQ_PRIO_1);
}

/* Function called from RADIO_TIMER_TXRX_WKUP_IRQHandler() context. */
void HAL_RADIO_TIMER_TxRxWakeUpCallback(void)
{
  VTimer_Process_Schedule();
}

/* Function called from RADIO_TIMER_CPU_WKUP_IRQHandler() context. */
void HAL_RADIO_TIMER_CpuWakeUpCallback(void)
{
  VTimer_Process_Schedule();
}

/* Function called from RADIO_TXRX_IRQHandler() context. */
void HAL_RADIO_TxRxCallback(uint32_t flags)
{
  BLE_STACK_RadioHandler(flags);

  VTimer_Process_Schedule();
  NVM_Process_Schedule();
}

void BLE_STACK_ProcessRequest(void)
{
  BLEStack_Process_Schedule();
}

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  /* USER CODE BEGIN APP_BLE_Init_1 */

  /* USER CODE END APP_BLE_Init_1 */
  UTIL_SEQ_RegTask(1U << CFG_TASK_BLE_STACK, UTIL_SEQ_RFU, BLEStack_Process);
  UTIL_SEQ_RegTask(1U << CFG_TASK_VTIMER, UTIL_SEQ_RFU, VTimer_Process);
  UTIL_SEQ_RegTask(1U << CFG_TASK_NVM, UTIL_SEQ_RFU, NVM_Process);
  ModulesInit();

  /* Initialization of HCI & GATT & GAP layer */
  BLE_Init();

  /**
  * Initialization of the BLE App Context
  */
  bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
  bleAppContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /* From here, all initialization are BLE application specific */

  /* USER CODE BEGIN APP_BLE_Init_4 */

  /* USER CODE END APP_BLE_Init_4 */

  /**
  * Initialize Services and Characteristics.
  */
  APP_DBG_MSG("\n");
  APP_DBG_MSG("Services and Characteristics creation\n");
  SENSOR_SERVER_APP_Init();
  MOTOR_SERVER_APP_Init();
  BCONNECTION_SERVER_APP_Init();  // Now enabled with increased GATT attributes
  CHAT_SERVER_Init();              // Chat service for phone-to-phone messaging
  CHAT_SERVER_APP_Init();          // Chat service application layer
  // UART_RELAY_SERVICE_Init();      // TODO: Fix UART service compilation errors
  APP_DBG_MSG("End of Services and Characteristics creation\n");
  APP_DBG_MSG("\n");
  
  /* REMOVED: Connection configuration during init
   * aci_gap_set_connection_configuration() MUST be called when creating a connection,
   * NOT during initialization. Calling it here fails with 0x12 COMMAND_DISALLOWED
   * because there's no connection context.
   * 
   * Dual PHY configuration is now in dual_role_manager.c DualRole_ConnectToPeer()
   * where it's called RIGHT BEFORE aci_gap_create_connection() - this is the
   * correct timing per ST's P2P Router reference design.
   */
  APP_DBG_MSG("📝 Connection parameters will be configured at runtime (sub firmware approach)\n");
  
  /* Initialize dual-role manager */
  DualRole_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);

  /* USER CODE END APP_BLE_Init_3 */

  /**
  * Initialize GATT Client Application
  */
  GATT_CLIENT_APP_Init();

  /* USER CODE BEGIN APP_BLE_Init_2 */
  UTIL_SEQ_RegTask( 1u << TASK_CONNECT_REQUEST, UTIL_SEQ_RFU, Connect_Request);
  UTIL_SEQ_RegTask( 1u << TASK_CLASSIFY_CONNECTION, UTIL_SEQ_RFU, Classify_Connection_Task);
  /* USER CODE END APP_BLE_Init_2 */

  return;
}

void BLEEVT_App_Notification(const hci_pckt *hci_pckt)
{
  tBleStatus ret = BLE_STATUS_ERROR;
  hci_event_pckt    *p_event_pckt;
  hci_le_meta_event *p_meta_evt;
  void *event_data;

  UNUSED(ret);
  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  if(hci_pckt->type != HCI_EVENT_PKT_TYPE && hci_pckt->type != HCI_EVENT_EXT_PKT_TYPE)
  {
    /* Not an event */
    return;
  }

  p_event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type == HCI_EVENT_PKT_TYPE){
    event_data = p_event_pckt->data;
  }
  else { /* hci_pckt->type == HCI_EVENT_EXT_PKT_TYPE */
    hci_event_ext_pckt *p_event_pckt = (hci_event_ext_pckt*)hci_pckt->data;
    event_data = p_event_pckt->data;
  }

  switch (p_event_pckt->evt) /* evt field is at same offset in hci_event_pckt and hci_event_ext_pckt */
  {
  case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      hci_disconnection_complete_event_rp0 *p_disconnection_complete_event;
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_3 */

        /* USER CODE END EVT_DISCONN_COMPLETE_3 */

      if (p_disconnection_complete_event->Connection_Handle == bleAppContext.BleApplicationContext_legacy.connectionHandle)
      {
        bleAppContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
        bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
        APP_DBG_MSG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE\n");
        APP_DBG_MSG("     - Connection Handle:   0x%04X\n     - Reason:    0x%02X\n",
                    p_disconnection_complete_event->Connection_Handle,
                    p_disconnection_complete_event->Reason);

        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_2 */
        /* CRITICAL: Check if this is phone or peer disconnection */
        if (p_disconnection_complete_event->Connection_Handle == DualRole_GetContext()->phone_conn_handle)
        {
          APP_DBG_MSG("     - Type: PHONE disconnection\n");
          DualRole_OnPhoneDisconnected();
        }
        else if (p_disconnection_complete_event->Connection_Handle == DualRole_GetContext()->peer_conn_handle)
        {
          APP_DBG_MSG("     - Type: PEER disconnection\n");
          DualRole_OnPeerDisconnected(p_disconnection_complete_event->Reason);
        }
        /* USER CODE END EVT_DISCONN_COMPLETE_2 */
      }
      else
      {
        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_PEER */
        /* CRITICAL: Handle may not match bleAppContext.connectionHandle (dual connections!)
         * Check if it's phone or peer and route accordingly */
        APP_DBG_MSG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE (non-primary)\n");
        APP_DBG_MSG("     - Connection Handle:   0x%04X\n     - Reason:    0x%02X\n",
                    p_disconnection_complete_event->Connection_Handle,
                    p_disconnection_complete_event->Reason);
        
        if (p_disconnection_complete_event->Connection_Handle == DualRole_GetContext()->phone_conn_handle)
        {
          APP_DBG_MSG("     - Type: PHONE disconnection\n");
          bleAppContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
          bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
          DualRole_OnPhoneDisconnected();
        }
        else if (p_disconnection_complete_event->Connection_Handle == DualRole_GetContext()->peer_conn_handle)
        {
          APP_DBG_MSG("     - Type: PEER disconnection\n");
          DualRole_OnPeerDisconnected(p_disconnection_complete_event->Reason);
        }
        /* USER CODE END EVT_DISCONN_COMPLETE_PEER */
      }
      gap_cmd_resp_release();
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */

      /* USER CODE END EVT_DISCONN_COMPLETE_1 */
      SENSOR_SERVERHandleNotification.EvtOpcode = SENSOR_SERVER_DISCON_HANDLE_EVT;
      SENSOR_SERVERHandleNotification.ConnectionHandle = p_disconnection_complete_event->Connection_Handle;
      SENSOR_SERVER_APP_EvtRx(&SENSOR_SERVERHandleNotification);
      MOTOR_SERVERHandleNotification.EvtOpcode = MOTOR_SERVER_DISCON_HANDLE_EVT;
      MOTOR_SERVERHandleNotification.ConnectionHandle = p_disconnection_complete_event->Connection_Handle;
      MOTOR_SERVER_APP_EvtRx(&MOTOR_SERVERHandleNotification);
      BCONNECTION_SERVERHandleNotification.EvtOpcode = BCONNECTION_SERVER_DISCON_HANDLE_EVT;
      BCONNECTION_SERVERHandleNotification.ConnectionHandle = p_disconnection_complete_event->Connection_Handle;
      BCONNECTION_SERVER_APP_EvtRx(&BCONNECTION_SERVERHandleNotification);
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */
      /* REMOVED: Redundant advertising restart — DualRole_OnPhoneDisconnected() and
       * DualRole_OnPeerDisconnected() already restart advertising. Firing it again
       * here causes a 2nd rapid DISABLE+ENABLE cycle that floods the HCI queue
       * and disrupts connection events on the next phone connection. */
      /*
      bleAppContext.BleApplicationContext_legacy.numOfConnections--;
      */
      /* USER CODE END EVT_DISCONN_COMPLETE */
    }
    break;

  case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (hci_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
      case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_update_complete_event_rp0 *p_conn_update_complete;
          p_conn_update_complete = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          /* LOGGING MINIMIZED: Connection parameter updates happen during negotiation
           * Verbose logging (140+ chars) blocks CPU ~12ms during critical timing window
           * This delayed firmware response to Android, causing parameter fighting and 0x28 disconnects */
          #if 0  /* Disabled - enable only for debugging connection parameter issues */
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %d.%02d ms\n     - Connection latency:    %d\n     - Supervision Timeout:   %d ms\n",
                      INT(p_conn_update_complete->Connection_Interval*1.25),
                      FRACTIONAL_2DIGITS(p_conn_update_complete->Connection_Interval*1.25),
                      p_conn_update_complete->Peripheral_Latency,
                      p_conn_update_complete->Supervision_Timeout*10);
          #endif
          UNUSED(p_conn_update_complete);
          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */
          /*
          if(bleAppContext.BleApplicationContext_legacy.numOfConnections <= MAX_NUM_CONNECTIONS)
            {
          	  APP_DBG_MSG("\t\t Connected to %d device and advertise\n",bleAppContext.BleApplicationContext_legacy.numOfConnections);
          	  APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
            }
            else
            {
          	  APP_DBG_MSG("\t\t Connected to %d device and max no advertise.\n",bleAppContext.BleApplicationContext_legacy.numOfConnections);
            }
            */
          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
        }
        break;
      case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
        {
          hci_le_phy_update_complete_event_rp0 *p_le_phy_update_complete;
          p_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)p_meta_evt->data;
          UNUSED(p_le_phy_update_complete);

          gap_cmd_resp_release();

          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */
          /* Route to dual-role manager if it's the peer connection */
          if (p_le_phy_update_complete->Connection_Handle == DualRole_GetPeerHandle())
          {
            DualRole_OnPeerPhyUpdated(
              p_le_phy_update_complete->Status,
              p_le_phy_update_complete->TX_PHY,
              p_le_phy_update_complete->RX_PHY
            );
          }
          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
        }
        break;
      case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_conn_complete;
          p_enhanced_conn_complete = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;

          connection_complete_event(p_enhanced_conn_complete->Status,
                                    p_enhanced_conn_complete->Connection_Handle,
									p_enhanced_conn_complete->Role,
                                    p_enhanced_conn_complete->Peer_Address_Type,
                                    p_enhanced_conn_complete->Peer_Address,
                                    p_enhanced_conn_complete->Connection_Interval,
                                    p_enhanced_conn_complete->Peripheral_Latency,
                                    p_enhanced_conn_complete->Supervision_Timeout);
        }
        break;
      case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_complete_event_rp0 *p_conn_complete;
          p_conn_complete = (hci_le_connection_complete_event_rp0 *) p_meta_evt->data;

          connection_complete_event(p_conn_complete->Status,
                                    p_conn_complete->Connection_Handle,
									p_conn_complete->Role,
                                    p_conn_complete->Peer_Address_Type,
                                    p_conn_complete->Peer_Address,
                                    p_conn_complete->Connection_Interval,
                                    p_conn_complete->Peripheral_Latency,
                                    p_conn_complete->Supervision_Timeout);
        }
        break;
      case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
        {
          hci_le_advertising_report_event_rp0 *p_adv_report;
          p_adv_report = (hci_le_advertising_report_event_rp0 *) p_meta_evt->data;
          /* USER CODE BEGIN EVT_LE_ADVERTISING_REPORT */
          
          /* Route to dual-role manager for peer discovery (silent operation) */
          DualRole_OnAdvertisingReport(
            p_adv_report->Advertising_Report.Address_Type,
            p_adv_report->Advertising_Report.Address,
            p_adv_report->Advertising_Report.Data_Length,
            p_adv_report->Advertising_Report.Data_RSSI,
            p_adv_report->Advertising_Report.Event_Type
          );
          
          /* USER CODE END EVT_LE_ADVERTISING_REPORT */
	  UNUSED(p_adv_report);

        }
        break;
      case HCI_LE_EXTENDED_ADVERTISING_REPORT_SUBEVT_CODE:
        {
          hci_le_extended_advertising_report_event_rp0 *p_ext_adv_report;
          p_ext_adv_report = (hci_le_extended_advertising_report_event_rp0 *) p_meta_evt->data;
          /* USER CODE BEGIN EVT_LE_EXT_ADVERTISING_REPORT */
          
          /* Route to dual-role manager for peer discovery */
          DualRole_OnAdvertisingReport(
            p_ext_adv_report->Extended_Advertising_Report.Address_Type,
            p_ext_adv_report->Extended_Advertising_Report.Address,
            p_ext_adv_report->Extended_Advertising_Report.Data_Length,
            p_ext_adv_report->Extended_Advertising_Report.Data,
            0  /* Event type - use 0 for now */
          );
          
          /* USER CODE END EVT_LE_EXT_ADVERTISING_REPORT */
          UNUSED(p_ext_adv_report);

        }
        break;
      /* USER CODE BEGIN EVT_LE_META_EVENT_1 */
      case HCI_LE_DATA_LENGTH_CHANGE_SUBEVT_CODE:
        {
          hci_le_data_length_change_event_rp0 *p_dle_change;
          p_dle_change = (hci_le_data_length_change_event_rp0 *) p_meta_evt->data;
          
          /* Route to dual-role manager if it's the peer connection */
          if (p_dle_change->Connection_Handle == DualRole_GetPeerHandle())
          {
            DualRole_OnPeerDleEnabled(
              p_dle_change->MaxTxOctets,
              p_dle_change->MaxRxOctets
            );
          }
        }
        break;
      /* USER CODE END EVT_LE_META_EVENT_1 */

      default:
        /* USER CODE BEGIN SUBEVENT_DEFAULT */

        /* USER CODE END SUBEVENT_DEFAULT */
        break;
      }
    } /* HCI_LE_META_EVT_CODE */
    break;

  case HCI_VENDOR_EVT_CODE:
    {
      aci_blecore_event *p_blecore_evt = (aci_blecore_event*) event_data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (p_blecore_evt->ecode)
      {
        /* USER CODE BEGIN ecode */

        /* USER CODE END ecode */
      case ACI_L2CAP_CONNECTION_UPDATE_REQ_VSEVT_CODE:
        {
          aci_l2cap_connection_update_req_event_rp0 *p_l2cap_conn_update_req;
          p_l2cap_conn_update_req = (aci_l2cap_connection_update_req_event_rp0 *) p_blecore_evt->data;
          tBleStatus ret;
          uint8_t req_resp = 0x01;

          /* USER CODE BEGIN EVT_L2CAP_CONNECTION_UPDATE_REQ */
          
          APP_DBG_MSG(">>== ACI_L2CAP_CONNECTION_UPDATE_REQ_VSEVT_CODE\n");
          APP_DBG_MSG("     Connection_Handle: 0x%04X\n", p_l2cap_conn_update_req->Connection_Handle);
          APP_DBG_MSG("     Requested intervals: [%d.%02d - %d.%02d] ms\n",
                      INT(p_l2cap_conn_update_req->Connection_Interval_Min*1.25),
                      FRACTIONAL_2DIGITS(p_l2cap_conn_update_req->Connection_Interval_Min*1.25),
                      INT(p_l2cap_conn_update_req->Connection_Interval_Max*1.25),
                      FRACTIONAL_2DIGITS(p_l2cap_conn_update_req->Connection_Interval_Max*1.25));
          
          /* CONNECTION STABILITY FIX: Enforce minimum connection intervals for dual-role operation
           * Fast intervals (< 30ms) cause resource exhaustion and disconnections in dual-connection mode.
           * REJECT overly aggressive requests entirely instead of adjusting them.
           */
          #define MIN_CONN_INTERVAL_UNITS  24  /* 30ms = 24 * 1.25ms */
          #define MAX_CONN_INTERVAL_UNITS  40  /* 50ms = 40 * 1.25ms */
          
          uint16_t requested_min = p_l2cap_conn_update_req->Connection_Interval_Min;
          uint16_t requested_max = p_l2cap_conn_update_req->Connection_Interval_Max;
          
          /* SUPERVISION TIMEOUT FIX: Android defaults to 5000ms supervision timeout.
           * During the peer scan phase the BLE radio runs the scan procedure for up to
           * 2 seconds; if supervision timeout < 10s the phone can disconnect with 0x28.
           * Enforce a minimum of 20s (2000 × 10ms) on every L2CAP response so the
           * phone link survives the full scan+connect sequence uninterrupted. */
          #define MIN_SUPERVISION_TIMEOUT_UNITS  2000   /* 20000ms = 2000 × 10ms */
          uint16_t enforced_timeout =
              (p_l2cap_conn_update_req->Timeout_Multiplier < MIN_SUPERVISION_TIMEOUT_UNITS)
                  ? MIN_SUPERVISION_TIMEOUT_UNITS
                  : p_l2cap_conn_update_req->Timeout_Multiplier;
          APP_DBG_MSG("   Supervision timeout: requested=%u ms  enforced=%u ms\n",
                      p_l2cap_conn_update_req->Timeout_Multiplier * 10,
                      enforced_timeout * 10);
          
          /* Check if request is acceptable */
          if (requested_min < MIN_CONN_INTERVAL_UNITS || requested_max < MIN_CONN_INTERVAL_UNITS) {
            /* REJECT - intervals too fast for dual-role operation */
            req_resp = 0x00;  /* 0x00 = REJECT */
            APP_DBG_MSG("🚫 REJECTING fast interval request: [%d.%02d - %d.%02d] ms (minimum allowed: %d ms)\n",
                        INT(requested_min*1.25), FRACTIONAL_2DIGITS(requested_min*1.25),
                        INT(requested_max*1.25), FRACTIONAL_2DIGITS(requested_max*1.25),
                        INT(MIN_CONN_INTERVAL_UNITS*1.25));
            
            /* When rejecting, we must still provide acceptable alternative parameters */
            /* The phone will see rejection and may retry or use our suggested parameters */
            requested_min = MIN_CONN_INTERVAL_UNITS;
            requested_max = MAX_CONN_INTERVAL_UNITS;
          } else {
            /* ACCEPT - intervals are reasonable */
            req_resp = 0x01;  /* 0x01 = ACCEPT */
            APP_DBG_MSG("✅ ACCEPTING interval request: [%d.%02d - %d.%02d] ms\n",
                        INT(requested_min*1.25), FRACTIONAL_2DIGITS(requested_min*1.25),
                        INT(requested_max*1.25), FRACTIONAL_2DIGITS(requested_max*1.25));
          }

          /* USER CODE END EVT_L2CAP_CONNECTION_UPDATE_REQ */

          ret = aci_l2cap_connection_parameter_update_resp(p_l2cap_conn_update_req->Connection_Handle,
                                                           requested_min,
                                                           requested_max,
                                                           p_l2cap_conn_update_req->Max_Latency,
                                                           enforced_timeout,
                                                           CONN_CE_LENGTH_MS(10),
                                                           CONN_CE_LENGTH_MS(10),
                                                           p_l2cap_conn_update_req->Identifier,
                                                           req_resp);
          if(ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("  Fail   : aci_l2cap_connection_parameter_update_resp command (error: 0x%02X)\n", ret);
          }
          else
          {
            if (req_resp == 0x00) {
              APP_DBG_MSG("  ✅ Rejection sent - phone must use intervals >= %d ms\n", INT(MIN_CONN_INTERVAL_UNITS*1.25));
            } else {
              APP_DBG_MSG("  ✅ Connection parameters accepted\n");
            }
          }

          /* USER CODE BEGIN EVT_L2CAP_CONNECTION_UPDATE_RESP */

          /* USER CODE END EVT_L2CAP_CONNECTION_UPDATE_RESP */
        }
        break;
      case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE\n");
          aci_gap_proc_complete_event_rp0 *p_gap_proc_complete;
          p_gap_proc_complete = (aci_gap_proc_complete_event_rp0*) p_blecore_evt->data;

          /* USER CODE BEGIN EVT_GAP_PROCEDURE_COMPLETE */
          /* CRITICAL FIX: Gate DualRole_BuildDeviceListData() on GAP_GENERAL_DISCOVERY_PROC.
           * Previously scan==1 fired for ANY procedure completion including aci_gap_create_connection.
           * When the peer connection PROC_COMPLETE fired with scan==1 (stuck from a failed scan),
           * DualRole_BuildDeviceListData() reset state to PHONE_CONNECTED(1), armed the 10s timer,
           * which then fired immediately and restarted FAST advertising — dropping the peer with 0x08.
           */
          if ((p_gap_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC) &&
			 (p_gap_proc_complete->Status == 0x00))
		 {
		   if (scan == 1)
		   {
			   /* Build device list data for notification */
			   DualRole_BuildDeviceListData();
			   UTIL_SEQ_SetTask(1U << UPDATE_BLST, CFG_SEQ_PRIO_1);
			   scan = 0;
		   }
		   APP_DBG_MSG("-- GAP_GENERAL_DISCOVERY_PROC completed__TASK_CONNECT_REQUEST\n");
		   UTIL_SEQ_SetTask(1u << TASK_CONNECT_REQUEST, CFG_SEQ_PRIO_0);
		 }
          /* USER CODE END EVT_GAP_PROCEDURE_COMPLETE */
        }
        break;
      case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
        /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/

        /* USER CODE END RADIO_ACTIVITY_EVENT*/
        break;
      case ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
          /* USER CODE BEGIN ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/
        }
        break;
      case ACI_GAP_PASSKEY_REQ_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GAP_PASSKEY_REQ_VSEVT_CODE\n");

          ret = aci_gap_passkey_resp(bleAppContext.BleApplicationContext_legacy.connectionHandle, CFG_FIXED_PIN);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_passkey_resp : Fail, reason: 0x%02X\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_passkey_resp : Success\n");
          }
          /* USER CODE BEGIN ACI_GAP_PASSKEY_REQ_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_PASSKEY_REQ_VSEVT_CODE*/
        }
        break;
      case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
        {
          uint8_t confirm_value;
          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
          APP_DBG_MSG("     - numeric_value = %d\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          APP_DBG_MSG("     - Hex_value = %x\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);

          /* Set confirm value to 1(YES) */
          confirm_value = 1;
          /* USER CODE BEGIN ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE_0*/

          /* USER CODE END ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE_0*/

          ret = aci_gap_numeric_comparison_value_confirm_yesno(bleAppContext.BleApplicationContext_legacy.connectionHandle, confirm_value);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno : Fail, reason: 0x%02X\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno : Success\n");
          }
          /* USER CODE BEGIN ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/
        }
        break;
      case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
          aci_gap_pairing_complete_event_rp0 *p_pairing_complete;
          p_pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;

          if (p_pairing_complete->Status != 0)
          {
            APP_DBG_MSG("     - Pairing KO\n     - Status: 0x%02X\n     - Reason: 0x%02X\n",
                        p_pairing_complete->Status, p_pairing_complete->Reason);
          }
          else
          {
            APP_DBG_MSG("     - Pairing Success\n");
          }
          APP_DBG_MSG("\n");

          /* USER CODE BEGIN ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/
        }
        break;
      case ACI_GATT_SRV_READ_VSEVT_CODE :
        {
          APP_DBG_MSG(">>== ACI_GATT_SRV_READ_VSEVT_CODE\n");

          aci_gatt_srv_read_event_rp0    *p_read;
          p_read = (aci_gatt_srv_read_event_rp0*)p_blecore_evt->data;
          uint8_t error_code = BLE_ATT_ERR_INSUFF_AUTHORIZATION;

          APP_DBG_MSG("Handle 0x%04X\n",  p_read->Attribute_Handle);

          /* USER CODE BEGIN ACI_GATT_SRV_READ_VSEVT_CODE_1*/

          /* USER CODE END ACI_GATT_SRV_READ_VSEVT_CODE_1*/

          aci_gatt_srv_resp(p_read->Connection_Handle,
                            p_read->CID,
                            p_read->Attribute_Handle,
                            error_code,
                            0,
                            NULL);

          /* USER CODE BEGIN ACI_GATT_SRV_READ_VSEVT_CODE_2*/

          /* USER CODE END ACI_GATT_SRV_READ_VSEVT_CODE_2*/
          break;
        }
        /* USER CODE BEGIN EVT_VENDOR_1 */
      case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
        {
          aci_att_exchange_mtu_resp_event_rp0 *p_mtu_resp;
          p_mtu_resp = (aci_att_exchange_mtu_resp_event_rp0 *) p_blecore_evt->data;
          
          /* Route to dual-role manager if it's the peer connection */
          if (p_mtu_resp->Connection_Handle == DualRole_GetPeerHandle())
          {
            DualRole_OnPeerMtuExchanged(p_mtu_resp->MTU);
          }
        }
        break;
        
      case ACI_ATT_CLT_READ_RESP_VSEVT_CODE:
        {
          /* Device name read response for connection classification */
          aci_att_clt_read_resp_event_rp0 *p_read_resp;
          p_read_resp = (aci_att_clt_read_resp_event_rp0 *) p_blecore_evt->data;
          
          if (p_read_resp->Connection_Handle == g_pending_classify_handle)
          {
            /* Extract device name from response */
            char device_name[32] = {0};
            uint8_t name_len = p_read_resp->Event_Data_Length;
            if (name_len > 31) name_len = 31;
            memcpy(device_name, p_read_resp->Attribute_Value, name_len);
            device_name[name_len] = '\0';
            
            APP_DBG_MSG(">>== DEVICE NAME READ RESPONSE (Connection Classification)\n");
            APP_DBG_MSG("     - Connection Handle: 0x%04X\n", g_pending_classify_handle);
            APP_DBG_MSG("     - Device Name: '%s'\n", device_name);
            
            /* Check if it's a peer module (contains "RFOXiA") */
            if (strstr(device_name, "RFOXiA") != NULL)
            {
              /* It's a peer module - reclassify! */
              APP_DBG_MSG("     - ✅ RECLASSIFIED AS PEER (device name contains 'RFOXiA')\n");
              
              /* CRITICAL: Need to undo phone connection and establish peer connection */
              DualRole_Context_t *dual_ctx = DualRole_GetContext();
              
              /* If this was classified as phone, disconnect from phone state */
              if (dual_ctx->phone_conn_handle == g_pending_classify_handle)
              {
                APP_DBG_MSG("     - Removing incorrect PHONE classification\n");
                dual_ctx->phone_connected = 0;
                dual_ctx->phone_conn_handle = 0xFFFF;
              }
              
              /* Now properly classify as peer */
              DualRole_OnPeerConnected(g_pending_classify_handle, g_pending_classify_addr_type, g_pending_classify_addr);
            }
            else
            {
              /* It's a phone - classification was correct */
              APP_DBG_MSG("     - ✅ CONFIRMED AS PHONE (device name: %s)\n", device_name);
              /* Already classified as phone, no action needed */
            }
            
            /* Clear pending classification */
            g_pending_classify_handle = 0xFFFF;
          }
        }
        break;
        /* USER CODE END EVT_VENDOR_1 */
      default:
        /* USER CODE BEGIN EVT_VENDOR_DEFAULT */

        /* USER CODE END EVT_VENDOR_DEFAULT */
        break;
      }
    } /* HCI_VENDOR_EVT_CODE */
    break;

  case HCI_HARDWARE_ERROR_EVT_CODE:
    {
      hci_hardware_error_event_rp0 *p_hci_hardware_error_event;
      p_hci_hardware_error_event = (hci_hardware_error_event_rp0*)p_event_pckt->data;

      if (p_hci_hardware_error_event->Hardware_Code <= 0x03)
      {
        NVIC_SystemReset();
      }
    }
    break;

    /* USER CODE BEGIN EVENT_PCKT */

    /* USER CODE END EVENT_PCKT */

  default:
    /* USER CODE BEGIN ECODE_DEFAULT*/

    /* USER CODE END ECODE_DEFAULT*/
    break;
  }
}

static void connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Peripheral_Latency,
                                      uint16_t Supervision_Timeout)
{
  if(Status != 0)
  {
    APP_DBG_MSG("==>> connection_complete_event Fail, Status: 0x%02X\n", Status);
    bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
    return;
  }
  /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE_1 */

  /* USER CODE END HCI_EVT_LE_CONN_COMPLETE_1 */
  APP_DBG_MSG(">>== hci_le_connection_complete_event - Connection handle: 0x%04X\n", Connection_Handle);
  APP_DBG_MSG("     - Connection established with @:%02x:%02x:%02x:%02x:%02x:%02x\n",
              Peer_Address[5],
              Peer_Address[4],
              Peer_Address[3],
              Peer_Address[2],
              Peer_Address[1],
              Peer_Address[0]);
   APP_DBG_MSG("     - Connection Interval:   %d.%02d ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n",
               INT(Conn_Interval*1.25),
               FRACTIONAL_2DIGITS(Conn_Interval*1.25),
               Peripheral_Latency,
               Supervision_Timeout * 10
                 );

  if (bleAppContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
  {
   /* Connection as client */
   bleAppContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
  }
  else
  {
   /* Connection as server */
   bleAppContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
  }
  bleAppContext.BleApplicationContext_legacy.connectionHandle = Connection_Handle;
  if (Role == HCI_ROLE_PERIPHERAL)
  {
    SENSOR_SERVERHandleNotification.EvtOpcode = SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT;
  }
  else
  {
    SENSOR_SERVERHandleNotification.EvtOpcode = SENSOR_SERVER_CENTR_CONN_HANDLE_EVT;
  }

  SENSOR_SERVERHandleNotification.ConnectionHandle = Connection_Handle;
  SENSOR_SERVER_APP_EvtRx(&SENSOR_SERVERHandleNotification);
  if (Role == HCI_ROLE_PERIPHERAL)
  {
    MOTOR_SERVERHandleNotification.EvtOpcode = MOTOR_SERVER_PERIPH_CONN_HANDLE_EVT;
  }
  else
  {
    MOTOR_SERVERHandleNotification.EvtOpcode = MOTOR_SERVER_CENTR_CONN_HANDLE_EVT;
  }

  MOTOR_SERVERHandleNotification.ConnectionHandle = Connection_Handle;
  MOTOR_SERVER_APP_EvtRx(&MOTOR_SERVERHandleNotification);
  if (Role == HCI_ROLE_PERIPHERAL)
  {
    BCONNECTION_SERVERHandleNotification.EvtOpcode = BCONNECTION_SERVER_PERIPH_CONN_HANDLE_EVT;
  }
  else
  {
    BCONNECTION_SERVERHandleNotification.EvtOpcode = BCONNECTION_SERVER_CENTR_CONN_HANDLE_EVT;
  }

  BCONNECTION_SERVERHandleNotification.ConnectionHandle = Connection_Handle;
  BCONNECTION_SERVER_APP_EvtRx(&BCONNECTION_SERVERHandleNotification);

  /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */
  
  /* Route connection to dual-role manager */
  if (Role == HCI_ROLE_PERIPHERAL)
  {
    /* We are PERIPHERAL - someone connected TO us
     * Could be Module B (peer) or Phone A
     * 
     * Strategy: Check MAC address against known Module B address
     * If no match, default to PHONE classification
     */
    
    DualRole_Context_t *dual_ctx = DualRole_GetContext();
    uint8_t is_peer_connection = 0;
    
    /* Check 0: BD address heuristic — RFOXiA modules use STATIC RANDOM
     * addresses (address type = random, top 2 bits of MSB = 11, i.e. first
     * printed octet 0xC0-0xFF). Phones ALWAYS connect with Resolvable
     * Private Addresses (top 2 bits = 01, first octet 0x40-0x7F) or public
     * addresses (type 0). A random address with top bits 11 can only be
     * another RFOXiA module connecting to us as a peer.
     * This is symmetric: it works on BOTH modules without hardcoded MACs,
     * fixing the case where the ACCEPTING module misclassified the inbound
     * peer connection as a phone (breaking chat + Phone B's scan result). */
    if ((Peer_Address_Type & 0x01) == 0x01 && (Peer_Address[5] & 0xC0) == 0xC0)
    {
      is_peer_connection = 1;
      APP_DBG_MSG("     - Type: PEER (static random address = RFOXiA module)\n");
    }
    
    /* Check 1: Is this Module B? (MAC: ca:df:2b:e0:17:be) */
    const uint8_t module_b_addr[6] = {0xBE, 0x17, 0xE0, 0x2B, 0xDF, 0xCA};
    if (!is_peer_connection && memcmp(Peer_Address, module_b_addr, 6) == 0)
    {
      is_peer_connection = 1;
      APP_DBG_MSG("     - Type: PEER (recognized Module B by MAC address)\n");
    }
    
    /* Check 2: Existing peer connection */
    if (!is_peer_connection && dual_ctx && dual_ctx->peer_connected && dual_ctx->peer_conn_handle != 0xFFFF)
    {
      /* Check if this address matches our outgoing peer connection */
      if (memcmp(Peer_Address, dual_ctx->peer_bd_addr, 6) == 0)
      {
        is_peer_connection = 1;
        APP_DBG_MSG("     - Type: PEER (return connection from peer we're connected to)\n");
      }
    }
    
    /* Check 3: Scanned device list */
    if (!is_peer_connection)
    {
      for (uint8_t i = 0; i < device_count; i++)
      {
        if (memcmp(Peer_Address, device_list[i].addr, 6) == 0)
        {
          if (strstr(device_list[i].name, "RFOXiA") != NULL)
          {
            is_peer_connection = 1;
            APP_DBG_MSG("     - Type: PEER (RFOXiA module from scan list: %s)\n", device_list[i].name);
            break;
          }
        }
      }
    }
    
    if (is_peer_connection)
    {
      /* This is a peer module */
      DualRole_OnPeerConnected(Connection_Handle, Peer_Address_Type, Peer_Address);
      
      /* Also register this connection for sensor notifications */
      SENSOR_SERVER_SetPeerHandle(Connection_Handle);
      APP_DBG_MSG("✅ Peer registered for sensor notifications (handle 0x%04X)\n", Connection_Handle);
      
      /* CRITICAL: The generic dispatch above (SENSOR/BCONNECTION
       * PERIPH_CONN_HANDLE_EVT) already stamped THIS peer's handle as the
       * "phone" peripheral handle in the sensor and bconnection servers.
       * Restore the REAL phone handle (0xFFFF if no phone connected) so
       * B_LIST responses and phone sensor notifications target the phone,
       * not the peer (was causing 0x0C floods + empty scan results). */
      {
        uint16_t restore_phone_hdl = DualRole_GetPhoneHandle();
        SENSOR_SERVERHandleNotification.EvtOpcode = SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT;
        SENSOR_SERVERHandleNotification.ConnectionHandle = restore_phone_hdl;
        SENSOR_SERVER_APP_EvtRx(&SENSOR_SERVERHandleNotification);
        BCONNECTION_SERVERHandleNotification.EvtOpcode = BCONNECTION_SERVER_PERIPH_CONN_HANDLE_EVT;
        BCONNECTION_SERVERHandleNotification.ConnectionHandle = restore_phone_hdl;
        BCONNECTION_SERVER_APP_EvtRx(&BCONNECTION_SERVERHandleNotification);
        APP_DBG_MSG("✅ Restored phone handle 0x%04X in sensor/bconnection servers\n", restore_phone_hdl);
      }
      
      /* REMOVED TEST CODE: Was triggering immediate notifications causing 0x0C flood
       * Sensor notifications will start automatically after 200ms throttle period */
    }
    else
    {
      /* Default to phone */
      APP_DBG_MSG("     - Type: PHONE (not matched to known peer)\n");
      DualRole_OnPhoneConnected(Connection_Handle);
    }
  }
  else if (Role == HCI_ROLE_CENTRAL)
  {
    /* We are CENTRAL - we connected to peer (we initiated the connection) */
    APP_DBG_MSG("     - Type: PEER (we are central, connected TO peer)\n");
    DualRole_OnPeerConnected(Connection_Handle, Peer_Address_Type, Peer_Address);
    /* Mark central role: only the central initiates the Coded S=8 PHY upgrade
     * (see main.c) to avoid LL procedure collisions with the peer. */
    DualRole_GetContext()->peer_is_central = 1;
    /* Set GATT client handle and trigger service discovery so we can subscribe
     * to peer sensor notifications and relay them to the phone.
     * This is required because the deferred connection in main.c bypasses
     * Connect_Request() which normally sets APP_BLE_LP_CONNECTING. */
    /* Register the peer handle so GATT writes use the right connection.
     * Do NOT schedule client_discover_all here — subscription happens via
     * the PHY→MTU→subscription_pending chain to avoid GATT procedure collisions. */
    GATT_CLIENT_APP_Set_Conn_Handle(0, Connection_Handle);
    APP_DBG_MSG("🔗 Peer handle registered (0x%04X) — subscription via MTU chain\n", Connection_Handle);
  }

  if (bleAppContext.Device_Connection_Status == APP_BLE_CONNECTED_CLIENT)
 {
   UTIL_SEQ_SetTask( 1U << CFG_TASK_DISCOVER_SERVICES_ID, CFG_SEQ_PRIO_0);
 }

 if (Role == HCI_ROLE_CENTRAL)
 {
   new_ConnHdl = Connection_Handle;
   UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_CONNECTION_COMPLETE);
 }

  /* Keep advertising after phone connection to allow peer discovery */
  if (Role == HCI_ROLE_PERIPHERAL)
  {
    /* Phone just connected - restart advertising so peers can find us
     * CRITICAL FIX: Don't advertise if peer is already connected (dual-role state)
     * Advertising with 2 active connections causes 0x0C INSUFFICIENT_RESOURCES */
    if (DualRole_GetPeerHandle() == 0xFFFF) {
      /* No peer connected - safe to advertise.
       * Use LP (low-power) interval (1000ms) instead of FAST (20ms):
       * - Phone is already connected; no urgency for fast advertising
       * - 20ms advertising alongside a 45ms phone connection overloads the
       *   radio scheduler and causes missed connection events (→ 0x28 drops)
       * - 1000ms LP interval reduces radio overhead with minimal discovery impact */
      APP_DBG_MSG("\t\t Phone connected - restarting advertising (LP 1000ms) for peer discovery\n");
      APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_LP);
    } else if (DualRole_GetPhoneHandle() == 0xFFFF) {
      /* Peer connected but no phone yet - LP advertising already restarted by
       * DualRole_OnPeerConnected() so our phone can still connect. Don't start again. */
      APP_DBG_MSG("\t\t Peer connected without phone - LP advertising active for phone connection\n");
    } else {
      /* Both phone and peer connected - skip advertising to prevent 0x0C */
      APP_DBG_MSG("\t\t Phone + peer both active - skipping advertising (would cause 0x0C)\n");
    }
  }
  /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
}/* end hci_le_connection_complete_event() */

/* USER CODE BEGIN EVT_VENDOR_3 */

/* USER CODE END EVT_VENDOR_3 */

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return bleAppContext.Device_Connection_Status;
}

APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(uint16_t Connection_Handle)
{
  APP_BLE_ConnStatus_t conn_status;

  if (bleAppContext.BleApplicationContext_legacy.connectionHandle == Connection_Handle)
  {
    conn_status = bleAppContext.Device_Connection_Status;
  }
  else
  {
    conn_status = APP_BLE_IDLE;
  }

  return conn_status;
}

void APP_BLE_Procedure_Gap_General(ProcGapGeneralId_t ProcGapGeneralId)
{
  tBleStatus status;

  switch(ProcGapGeneralId)
  {
#if (CFG_BLE_CONTROLLER_2M_CODED_PHY_ENABLED == 1)
    case PROC_GAP_GEN_PHY_TOGGLE:
    {
      uint8_t phy_tx, phy_rx;

      status = hci_le_read_phy(bleAppContext.BleApplicationContext_legacy.connectionHandle, &phy_tx, &phy_rx);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("hci_le_read_phy failure: reason=0x%02X\n",status);
      }
      else
      {
        APP_DBG_MSG("==>> hci_le_read_phy - Success\n");
        APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d\n", phy_tx, phy_rx);
        if ((phy_tx == HCI_TX_PHY_LE_2M) && (phy_rx == HCI_RX_PHY_LE_2M))
        {
          APP_DBG_MSG("==>> hci_le_set_phy PHY Param  TX= %d, RX= %d - ", HCI_TX_PHY_LE_1M, HCI_RX_PHY_LE_1M);
          status = hci_le_set_phy(bleAppContext.BleApplicationContext_legacy.connectionHandle, 0, HCI_TX_PHYS_LE_1M_PREF, HCI_RX_PHYS_LE_1M_PREF, 0);
          if (status != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("Fail\n");
          }
          else
          {
            APP_DBG_MSG("Success\n");
            gap_cmd_resp_wait();/* waiting for HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE */
          }
        }
        else
        {
          APP_DBG_MSG("==>> hci_le_set_phy PHY Param  TX= %d, RX= %d - ", HCI_TX_PHYS_LE_2M_PREF, HCI_RX_PHYS_LE_2M_PREF);
          status = hci_le_set_phy(bleAppContext.BleApplicationContext_legacy.connectionHandle, 0, HCI_TX_PHYS_LE_2M_PREF, HCI_RX_PHYS_LE_2M_PREF, 0);
          if (status != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("Fail\n");
          }
          else
          {
            APP_DBG_MSG("Success\n");
            gap_cmd_resp_wait();/* waiting for HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE */
          }
        }
      }
      break;
    }/* PROC_GAP_GEN_PHY_TOGGLE */
#endif
    case PROC_GAP_GEN_CONN_TERMINATE:
    {
      status = aci_gap_terminate(bleAppContext.BleApplicationContext_legacy.connectionHandle, BLE_ERROR_TERMINATED_REMOTE_USER);
      if (status != BLE_STATUS_SUCCESS)
      {
         APP_DBG_MSG("aci_gap_terminate failure: reason=0x%02X\n", status);
      }
      else
      {
        APP_DBG_MSG("==>> aci_gap_terminate : Success\n");
        gap_cmd_resp_wait();/* waiting for HCI_DISCONNECTION_COMPLETE_EVT_CODE */
      }
      break;
    }/* PROC_GAP_GEN_CONN_TERMINATE */
    case PROC_GATT_EXCHANGE_CONFIG:
    {
      status =aci_gatt_clt_exchange_config(bleAppContext.BleApplicationContext_legacy.connectionHandle);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("aci_gatt_clt_exchange_config failure: reason=0x%02X\n", status);
      }
      else
      {
        APP_DBG_MSG("==>> aci_gatt_clt_exchange_config : Success\n");
      }
      break;
    }
    /* USER CODE BEGIN GAP_GENERAL */

    /* USER CODE END GAP_GENERAL */
    default:
      break;
  }
  return;
}

void APP_BLE_Procedure_Gap_Peripheral(ProcGapPeripheralId_t ProcGapPeripheralId)
{
  tBleStatus status;
  uint32_t paramA = ADV_INTERVAL_MIN;
  uint32_t paramB = ADV_INTERVAL_MAX;
  uint32_t paramC, paramD;

  /* First set parameters before calling ACI APIs, only if needed */
  switch(ProcGapPeripheralId)
  {
    case PROC_GAP_PERIPH_ADVERTISE_START_FAST:
    {
      paramA = ADV_INTERVAL_MIN;
      paramB = ADV_INTERVAL_MAX;
      paramC = APP_BLE_ADV_FAST;

      /* USER CODE BEGIN PROC_GAP_PERIPH_ADVERTISE_START_FAST */

      /* USER CODE END PROC_GAP_PERIPH_ADVERTISE_START_FAST */
      break;
    }/* PROC_GAP_PERIPH_ADVERTISE_START_FAST */
    case PROC_GAP_PERIPH_ADVERTISE_START_LP:
    {
      paramA = ADV_LP_INTERVAL_MIN;
      paramB = ADV_LP_INTERVAL_MAX;
      paramC = APP_BLE_ADV_LP;

      /* USER CODE BEGIN PROC_GAP_PERIPH_ADVERTISE_START_LP */

      /* USER CODE END PROC_GAP_PERIPH_ADVERTISE_START_LP */
      break;
    }/* PROC_GAP_PERIPH_ADVERTISE_START_LP */
    case PROC_GAP_PERIPH_ADVERTISE_STOP:
    {
      paramC = APP_BLE_IDLE;

      /* USER CODE BEGIN PROC_GAP_PERIPH_ADVERTISE_STOP */

      /* USER CODE END PROC_GAP_PERIPH_ADVERTISE_STOP */
      break;
    }/* PROC_GAP_PERIPH_ADVERTISE_STOP */
    case PROC_GAP_PERIPH_CONN_PARAM_UPDATE:
    {
      paramA = CONN_INT_MS(1000);
      paramB = CONN_INT_MS(1000);
      paramC = 0x0000;
      paramD = 0x03E8;  // 1000 = 10 seconds supervision timeout (was 0x01F4 = 5s)

      /* USER CODE BEGIN CONN_PARAM_UPDATE */

      /* USER CODE END CONN_PARAM_UPDATE */
      break;
    }/* PROC_GAP_PERIPH_CONN_PARAM_UPDATE */
    case PROC_GAP_PERIPH_CONN_TERMINATE:
    {
      status = aci_gap_terminate(bleAppContext.BleApplicationContext_legacy.connectionHandle, 0x13);
      if (status != BLE_STATUS_SUCCESS)
      {
         APP_DBG_MSG("aci_gap_terminate failure: reason=0x%02X\n", status);
      }
      else
      {
        APP_DBG_MSG("==>> aci_gap_terminate : Success\n");
        gap_cmd_resp_wait();/* waiting for HCI_DISCONNECTION_COMPLETE_EVT_CODE */
      }
      break;
    }
    /* PROC_GAP_PERIPH_CONN_TERMINATE */
    /* USER CODE BEGIN GAP_PERIPHERAL_1 */

    /* USER CODE END GAP_PERIPHERAL_1 */
    default:
      break;
  }

  /* Call ACI APIs */
  switch(ProcGapPeripheralId)
  {
    case PROC_GAP_PERIPH_ADVERTISE_START_FAST:
    case PROC_GAP_PERIPH_ADVERTISE_START_LP:
    {

      /* USER CODE BEGIN PROC_GAP_PERIPHERAL_ID */

      /* USER CODE END PROC_GAP_PERIPHERAL_ID */

      Advertising_Set_Parameters_t Advertising_Set_Parameters = {0};

      /* Start Fast or Low Power Advertising */

      /* CRITICAL FIX: Disable advertising first if it's already running
       * aci_gap_set_advertising_configuration returns 0x0C (COMMAND_DISALLOWED)
       * if advertising is already active. Always disable first to ensure clean state. */
      tBleStatus disable_status = aci_gap_set_advertising_enable(DISABLE, 0, NULL);
      if (disable_status != BLE_STATUS_SUCCESS && disable_status != 0x0C) {
        APP_DBG_MSG("==>> Pre-disable advertising returned 0x%02X (ignoring if 0x0C)\n", disable_status);
      }

      /* Set advertising configuration for legacy advertising */
      status = aci_gap_set_advertising_configuration(0,
                                                     GAP_MODE_GENERAL_DISCOVERABLE,
                                                     ADV_TYPE,
                                                     paramA,
                                                     paramB,
                                                     HCI_ADV_CH_ALL,
                                                     0,
                                                     NULL, /* No peer address */
                                                     HCI_ADV_FILTER_NONE,
                                                     0, /* 0 dBm */
                                                     HCI_PHY_LE_1M, /* Primary advertising PHY */
                                                     0, /* 0 skips */
                                                     HCI_PHY_LE_1M, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                     0, /* SID */
                                                     0 /* No scan request notifications */);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("==>> aci_gap_set_advertising_configuration - fail, result: 0x%02X\n", status);
      }
      else
      {
        bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
        APP_DBG_MSG("==>> Success: aci_gap_set_advertising_configuration\n");
      }

      /* Patch bd_address into manufacturer specific data bytes [19..24] before advertising */
      a_AdvData[19] = bd_address[0];
      a_AdvData[20] = bd_address[1];
      a_AdvData[21] = bd_address[2];
      a_AdvData[22] = bd_address[3];
      a_AdvData[23] = bd_address[4];
      a_AdvData[24] = bd_address[5];
      APP_DBG_MSG("==>> BD address in advert: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  bd_address[5], bd_address[4], bd_address[3],
                  bd_address[2], bd_address[1], bd_address[0]);

      status = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(a_AdvData), (uint8_t*) a_AdvData);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("==>> aci_gap_set_advertising_data Failed, result: 0x%02X\n", status);
      }
      else
      {
        APP_DBG_MSG("==>> Success: aci_gap_set_advertising_data\n");
      }

      /* Enable advertising */
      status = aci_gap_set_advertising_enable(ENABLE, 1, &Advertising_Set_Parameters);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("==>> aci_gap_set_advertising_enable Failed, result: 0x%02X\n", status);
      }
      else
      {
        APP_DBG_MSG("==>> Success: aci_gap_set_advertising_enable\n");
      }
      break;
    }
    case PROC_GAP_PERIPH_ADVERTISE_STOP:
    {
      status = aci_gap_set_advertising_enable(DISABLE, 0, NULL);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("Disable advertising - fail, result: 0x%02X\n",status);
      }
      else
      {
        bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
        APP_DBG_MSG("==>> Disable advertising - Success\n");
      }
      break;
    }/* PROC_GAP_PERIPH_ADVERTISE_STOP */
    case PROC_GAP_PERIPH_CONN_PARAM_UPDATE:
    {
       status = aci_l2cap_connection_parameter_update_req(
                                                       bleAppContext.BleApplicationContext_legacy.connectionHandle,
                                                       paramA,
                                                       paramB,
                                                       paramC,
                                                       paramD);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("aci_l2cap_connection_parameter_update_req - fail, result: 0x%02X\n",status);
      }
      else
      {
        APP_DBG_MSG("==>> aci_l2cap_connection_parameter_update_req - Success\n");
      }

      break;
    }/* PROC_GAP_PERIPH_CONN_PARAM_UPDATE */

    case PROC_GAP_PERIPH_SET_BROADCAST_MODE:
    {

      break;
    }/* PROC_GAP_PERIPH_SET_BROADCAST_MODE */
    /* USER CODE BEGIN GAP_PERIPHERAL_2 */

    /* USER CODE END GAP_PERIPHERAL_2 */
    default:
      break;
  }
  return;
}

void APP_BLE_Procedure_Gap_Central(ProcGapCentralId_t ProcGapCentralId)
{
  tBleStatus status;
  uint32_t paramA, paramB, paramC, paramD;

  UNUSED(paramA);
  UNUSED(paramB);
  UNUSED(paramC);
  UNUSED(paramD);

  /* First set parameters before calling ACI APIs, only if needed */
  switch(ProcGapCentralId)
  {
    case PROC_GAP_CENTRAL_SCAN_START:
    {
      /* CRITICAL: VERY gentle scan to maintain phone connection during scan
       * 100ms interval + 20ms window = 20% duty cycle
       * Leaves 80ms free for phone connection maintenance - prevents 0x28 timeout disconnect
       * Slower discovery but connection stays stable
       */
      /* 90 % duty cycle: scan for 90 ms in every 100 ms window.
       * Previous 20 ms / 100 ms (20%) caused ~8% hit-rate per advertising
       * period → peer required 3+ scans to appear.  At 90% duty cycle one
       * 2-second scan covers >99% of a 1000ms advertising interval. */
      paramA = SCAN_INT_MS(100);
      paramB = SCAN_WIN_MS(90);
      paramC = APP_BLE_SCANNING;

      break;
    }/* PROC_GAP_CENTRAL_SCAN_START */
    case PROC_GAP_CENTRAL_SCAN_TERMINATE:
    {
      paramA = 1;
      paramB = 1;
      paramC = APP_BLE_IDLE;

      break;
    }/* PROC_GAP_CENTRAL_SCAN_TERMINATE */
    /* USER CODE BEGIN GAP_CENTRAL_1 */

    /* USER CODE END GAP_CENTRAL_1 */
    default:
      break;
  }

  /* Call ACI APIs */
  switch(ProcGapCentralId)
  {
    case PROC_GAP_CENTRAL_SCAN_START:
    {
      /* DISABLE duplicate filter to detect all devices (even with same name) */
      status = aci_gap_set_scan_configuration(DUPLICATE_FILTER_DISABLED, 0x00, LE_1M_PHY_BIT, HCI_SCAN_TYPE_ACTIVE, paramA, paramB);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("❌ aci_gap_set_scan_configuration - fail, result: 0x%02X\n", status);
        /* Reset scan flag so user can retry */
        extern uint8_t scan;
        scan = 0;
        /* Return to phone connected state */
        DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
        /* Only restart advertising if no peer connection is pending.
         * Restarting advertising while aci_gap_create_connection is pending
         * adds a third radio task and can disrupt the pending connection. */
        {
            extern DualRole_Context_t* DualRole_GetContext(void);
            DualRole_Context_t *_ctx = DualRole_GetContext();
            if (!_ctx || !_ctx->pending_peer_connection) {
                APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
            } else {
                APP_DBG_MSG("⚠️ Skipping advertising restart - peer connection pending\n");
            }
        }
      }
      else
      {
        APP_DBG_MSG("==>> aci_gap_set_scan_configuration - Success\n");
      
        APP_DBG_MSG(">>SCAN_START: GAP_GENERAL_DISCOVERY_PROC on 1M PHY\n");
        /* Duration = 200 × 10ms = 2000ms cap.
         * Duration=0 means unbounded and can exceed the phone's supervision
         * timeout (5s default), causing 0x28 disconnection.  2s is enough to
         * find any nearby RFOXiA BLE module at the distances involved. */
        status = aci_gap_start_procedure(GAP_GENERAL_DISCOVERY_PROC, LE_1M_PHY_BIT, 200, 0);
        if (status != BLE_STATUS_SUCCESS)
        {
          APP_DBG_MSG("❌ aci_gap_start_procedure - fail, result: 0x%02X\n", status);
          /* Reset scan flag so user can retry */
          extern uint8_t scan;
          scan = 0;
          /* Return to phone connected state */
          DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
          /* Only restart advertising if no peer connection is pending.
           * Restarting advertising while aci_gap_create_connection is pending
           * adds a third radio task and can disrupt the pending connection. */
          {
              extern DualRole_Context_t* DualRole_GetContext(void);
              DualRole_Context_t *_ctx = DualRole_GetContext();
              if (!_ctx || !_ctx->pending_peer_connection) {
                  APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
              } else {
                  APP_DBG_MSG("⚠️ Skipping advertising restart - peer connection pending\n");
              }
          }
        }
        else
        {
          bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
          APP_DBG_MSG("==>> aci_gap_start_procedure - Success\n");
          /* Scan running silently - waiting for devices */
        }
      }
      break;
    }/* PROC_GAP_CENTRAL_SCAN_START */
    case PROC_GAP_CENTRAL_SCAN_TERMINATE:
    {
      status = aci_gap_terminate_proc(GAP_GENERAL_DISCOVERY_PROC);
      if (status != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("aci_gap_terminate_gap_proc - fail, result: 0x%02X\n",status);
      }
      else
      {
        bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
        APP_DBG_MSG("==>> aci_gap_terminate_gap_proc - Success\n");
      }
      break;
    }/* PROC_GAP_CENTRAL_SCAN_TERMINATE */
    /* USER CODE BEGIN GAP_CENTRAL_2 */

    /* USER CODE END GAP_CENTRAL_2 */
    default:
      break;
  }
  return;
}

static void gap_cmd_resp_release(void)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_PROC_GAP_COMPLETE);
  return;
}

static void gap_cmd_resp_wait(void)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_PROC_GAP_COMPLETE);
  return;
}
/* USER CODE BEGIN FD_LOCAL_FUNCTION */
static uint8_t analyse_adv_report(uint8_t adv_data_size, uint8_t *p_adv_data,
                                  uint8_t address_type, uint8_t *p_address)
{
  uint8_t found_status = 0;
  uint16_t i = 0;
  uint8_t ad_length, ad_type;

  /* Filter out empty scan response packets to reduce log flooding */
  if (adv_data_size == 0) {
    return 0;
  }

  while(i < adv_data_size)
  {
	  uint8_t found_status = 0;
    ad_length = p_adv_data[i];
    ad_type = p_adv_data[i + 1];

    switch (ad_type)
    {
    case AD_TYPE_COMPLETE_LOCAL_NAME:
    	//found device with local name in advertise

    	blst_devices_num++;
    	//APP_DBG_MSG("|%d __ %d __ ",ad_length,filter_name_size);
    	if(ad_length==filter_name_size+1)
    	{
			found_status=1;
    	}
    	for(int j=0;j<ad_length-1;j++)
    	{
    		if(p_adv_data[i+2+j]!=filter_name[j])
    		{
    			//APP_DBG_MSG(">> %c != %c  , index:%d\n",p_adv_data[i+2+j],filter_name[j],j);
    			found_status=0;
    		}
    		else
    		{
    		//	APP_DBG_MSG(">> %c == %c\n",p_adv_data[i+2+j],filter_name[j]);
    		}
    		blst[ blst_index]=p_adv_data[i+2+j];
    		blst_index++;
    		//APP_DBG_MSG("%c",p_adv_data[i+2+j]);
    	}
    	APP_DBG_MSG("|");
    	blst[ blst_index]='\n';
    	    		blst_index++;
    	APP_DBG_MSG("||||>>>found:%d\n",found_status);
    	if(found_status==1)
    	{
    		/* p2pServer name  detected */
    		            bleAppContext.deviceServerBdAddrType = address_type;
    		            memcpy(bleAppContext.a_deviceServerBdAddr, p_address, BD_ADDR_SIZE);

    		            bleAppContext.deviceServerFound = 0x01;

    		            APP_DBG_MSG("server found, db addr 0x%02X:%02X:%02X:%02X:%02X:%02X\n",
    		                        bleAppContext.a_deviceServerBdAddr[5],
    		                        bleAppContext.a_deviceServerBdAddr[4],
    		                        bleAppContext.a_deviceServerBdAddr[3],
    		                        bleAppContext.a_deviceServerBdAddr[2],
    		                        bleAppContext.a_deviceServerBdAddr[1],
    		                        bleAppContext.a_deviceServerBdAddr[0]);
    		            found_status = 1;
    	}
    		break;
    case AD_TYPE_SHORTENED_LOCAL_NAME:

    	APP_DBG_MSG("|");
    	    	for(int j=0;j<ad_length-1;j++)
    	    	{
    	    		APP_DBG_MSG("%c",p_adv_data[i+2+j]);
    	    	}
    	    	APP_DBG_MSG("|");
    	    		break;
    case AD_TYPE_FLAGS:
    	APP_DBG_MSG("|");
    	    	    	for(int j=0;j<ad_length-1;j++)
    	    	    	{
    	    	    		APP_DBG_MSG("%X",p_adv_data[i+2+j]);
    	    	    	}
    	    	    	APP_DBG_MSG("|");
    	    	    		break;
      break;
    case AD_TYPE_TX_POWER_LEVEL:
    	APP_DBG_MSG("|");
    	    	    	for(int j=0;j<ad_length-1;j++)
    	    	    	{
    	    	    		APP_DBG_MSG("%X",p_adv_data[i+2+j]);
    	    	    	}
    	    	    	APP_DBG_MSG("|");
    	    	    		break;

    default:
      break;
    }/* end of switch*/

    i += ad_length + 1; /* increment the iterator to go on next element*/

    if (found_status != 0)
    {
      break;
    }
  }/* end of while*/
  APP_DBG_MSG("\n");
  return found_status;
}

static void Connect_Request(void)
{
  tBleStatus result;

  if (bleAppContext.deviceServerFound != 0)
  {
    APP_DBG_MSG("Create connection to P2Pserver\n");

    result = aci_gap_set_connection_configuration(LE_1M_PHY_BIT,
                                                  CONN_INT_MS(50u), CONN_INT_MS(100u),
                                                  0u,
                                                  CONN_SUP_TIMEOUT_MS(10000u),  // 10 seconds (was 5000)
                                                  CONN_CE_LENGTH_MS(10u), CONN_CE_LENGTH_MS(10u));

    if (result == BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_connection_configuration Success , result: 0x%02x\n", result);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_connection_configuration Failed , result: 0x%02x\n", result);
    }
    result = aci_gap_create_connection(LE_1M_PHY_BIT,
                                       bleAppContext.deviceServerBdAddrType,
                                       &bleAppContext.a_deviceServerBdAddr[0]);
    if (result == BLE_STATUS_SUCCESS)
    {
      bleAppContext.Device_Connection_Status = APP_BLE_LP_CONNECTING;
      APP_DBG_MSG("  wait for event HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE\n");
      UTIL_SEQ_WaitEvt(1u << CFG_IDLEEVT_CONNECTION_COMPLETE);
      if(new_ConnHdl != 0u)
      {
        GATT_CLIENT_APP_Set_Conn_Handle(0, new_ConnHdl);
        new_ConnHdl = 0u;
      }
    }
    else
    {
      APP_DBG_MSG("==>> GAP Create connection Failed , result: 0x%02x\n", result);
    }
  }

  return;
}

/**
 * @brief  Task to classify incoming connection as phone or peer by reading device name
 * @param  None
 * @retval None
 */
static void Classify_Connection_Task(void)
{
  tBleStatus ret;
  
  if (pending_conn_handle == 0xFFFF) {
    APP_DBG_MSG("⚠️ Classify_Connection_Task: No pending connection\n");
    return;
  }
  
  APP_DBG_MSG("📝 Reading GAP Device Name to classify connection 0x%04X...\n", pending_conn_handle);
  
  /* Read GAP Device Name characteristic (UUID 0x2A00, typically at handle 0x0003)
   * CID = 0x0004 for unenhanced ATT bearer
   * The response will come via ACI_ATT_CLT_READ_RESP_VSEVT_CODE event */
  ret = aci_gatt_clt_read(pending_conn_handle, 0x0004, 0x0003);
  
  if (ret != BLE_STATUS_SUCCESS) {
    APP_DBG_MSG("⚠️ Failed to read device name: 0x%02X\n", ret);
    APP_DBG_MSG("     Defaulting to PHONE classification\n");
    
    /* If read fails, assume it's a phone (safer default) */
    DualRole_OnPhoneConnected(pending_conn_handle);
    pending_conn_handle = 0xFFFF;
  }
  /* Otherwise wait for ACI_GATT_CLT_READ_RESP_VSEVT_CODE event */
}

/**
 * @brief  Read device name characteristic to classify connection
 * @param  conn_handle: Connection handle to read from
 * @retval None
 */
void APP_BLE_ReadDeviceName(uint16_t conn_handle)
{
  tBleStatus ret;
  
  APP_DBG_MSG("📖 Initiating device name read on handle 0x%04X...\n", conn_handle);
  
  /* Read GAP Device Name characteristic (UUID 0x2A00)
   * Standard GATT handle for Device Name is usually 0x0003
   * CID 0x0004 = unenhanced ATT bearer
   * Response will come via ACI_GATT_CLT_READ_RESP_VSEVT_CODE */
  ret = aci_gatt_clt_read(conn_handle, 0x0004, 0x0003);
  
  if (ret != BLE_STATUS_SUCCESS) {
    APP_DBG_MSG("⚠️ Device name read request failed: 0x%02X\n", ret);
    APP_DBG_MSG("     Connection will remain classified as PHONE\n");
  }
}
/* USER CODE END FD_LOCAL_FUNCTION */

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */

/** \endcond
 */
