/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BConnection_Server_app.c
  * @author  MCD Application Team
  * @brief   BConnection_Server_app application definition.
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
#include "bconnection_server_app.h"
#include "bconnection_server.h"
#include "sensor_server_app.h"  /* For SENSOR_SERVER_APP_Handle_ViewChange() */
#include "stm32_seq.h"
#include "dual_role_manager.h"

/* External functions --------------------------------------------------------*/
extern void BLE_Init(void);  /* For clean BLE stack restart */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Blist_char_NOTIFICATION_OFF,
  Blist_char_NOTIFICATION_ON,
  /* USER CODE BEGIN Service3_APP_SendInformation_t */

  /* USER CODE END Service3_APP_SendInformation_t */
  BCONNECTION_SERVER_APP_SENDINFORMATION_LAST
} BCONNECTION_SERVER_APP_SendInformation_t;

typedef struct
{
  BCONNECTION_SERVER_APP_SendInformation_t     Blist_char_Notification_Status;
  uint16_t BCONNECTION_SERVER_periph_connHdl;
  /* USER CODE BEGIN Service3_APP_Context_t */

  /* USER CODE END Service3_APP_Context_t */
  uint16_t              ConnectionHandle;
} BCONNECTION_SERVER_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern uint8_t blst_devices_num;
extern uint8_t blst_index;
extern uint8_t blst[255];

 uint8_t filter_name[100];
 uint8_t filter_name_size=0;
 
/* Prevent duplicate B_LIST sends after subscription */
static uint8_t blist_already_sent = 0;
static uint32_t last_subscription_time = 0;

/**
 * @brief Reset the blist_already_sent flag when starting new scan or clearing device list
 */
void BCONNECTION_SERVER_APP_ResetBListSentFlag(void)
{
    blist_already_sent = 0;
}
/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static BCONNECTION_SERVER_APP_Context_t BCONNECTION_SERVER_APP_Context;

uint8_t a_BCONNECTION_SERVER_UpdateCharData[247];

volatile uint8_t g_reboot_requested = 0;  /* Flag to trigger reboot from main loop - MUST be initialized to 0 */

/* USER CODE BEGIN PV */
/**
 * @brief Check if phone has subscribed to BLIST_STATE notifications
 * @retval 1 if subscribed, 0 if not
 */
uint8_t BCONNECTION_SERVER_APP_IsBListStateSubscribed(void)
{
    return (BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status == Blist_char_NOTIFICATION_ON);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BCONNECTION_SERVER_Blist_char_SendNotification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void BCONNECTION_SERVER_Notification(BCONNECTION_SERVER_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service3_Notification_1 */
  APP_DBG_MSG("🔔 BCONNECTION_SERVER_Notification: EvtOpcode=0x%02X\n", p_Notification->EvtOpcode);
  /* USER CODE END Service3_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service3_Notification_Service3_EvtOpcode */

    /* USER CODE END Service3_Notification_Service3_EvtOpcode */

    case BCONNECTION_SERVER_BLIST_STATE_CHAR_READ_EVT:
      /* USER CODE BEGIN Service3Char1_READ_EVT */

      /* USER CODE END Service3Char1_READ_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_STATE_CHAR_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN Service3Char1_WRITE_NO_RESP_EVT */
		APP_DBG_MSG("🔥🔥🔥 BLIST_STATE_CHAR_WRITE_NO_RESP_EVT - Write received! 🔥🔥🔥\n");
		APP_DBG_MSG("     Handle: 0x%04X\n", p_Notification->AttributeHandle);
		APP_DBG_MSG("     Length: %d bytes\n", p_Notification->DataTransfered.Length);
		
		//index to select
    	//read char send
    	uint8_t cmd=p_Notification->DataTransfered.p_Payload[0];
    	APP_DBG_MSG(">>>>>>>>>>> cmd : 0x%x <<<<<<<<<<<<<\n",cmd);
    	switch (cmd) {
    	case 0:
    	case 1 ... 10: {
		uint8_t device_index=0;
		uint8_t name_index=1;
    	for(name_index=1;device_index< cmd;name_index++ )
    	{
    		if(blst[name_index]=='\n')
    		{
    			device_index++;
    		}
    	}
    	//copy name of selected device
    	filter_name_size=0;
    	while (blst[name_index]!='\n' && blst[name_index]!='\0' )
    	{
    		filter_name[filter_name_size]=blst[name_index];
    		name_index++;
    		filter_name_size++;
    	}
    	filter_name[filter_name_size]='\0';
		APP_DBG_MSG("___INDEX select Device#%d  Signal\n",cmd);
		APP_DBG_MSG("__DeviceName:");
		int i=0;
    	//selected (local name)
		while(filter_name[i] != '\0')
		{
			APP_DBG_MSG("%c",filter_name[i]);
					i++;
		}
		APP_DBG_MSG("\n");
		
	/* ============================================================
	 * DUAL CONNECTION: PHONE + PEER (SUB FIRMWARE APPROACH)
	 * ============================================================
	 * Strategy: Connect to peer WHILE maintaining phone connection
	 * This is what sub firmware does - true dual-role
	 * 
	 * CRITICAL LESSON FROM ST EXAMPLE (BLE_p2pClient):
	 * DO NOT connect immediately in the notification handler!
	 * Instead, save the peer info and let the connection happen
	 * in the main loop after the notification handler returns.
	 * This gives the BLE stack time to fully settle.
	 * ============================================================
	 */
	{
		/* CRITICAL FIX: Block device selection while scan is still running
		 * Problem: App can send device selection BEFORE scan completes
		 * This causes reading corrupted/incomplete device list data
		 * Result: Invalid MAC addresses (e.g., 34:9C:00:07:00:02) and connection failure (error 0x0C)
		 * Solution: Reject selection if state is SCANNING, require app to wait for scan completion
		 */
		DualRole_State_t current_state = DualRole_GetState();
		if (current_state == DUAL_STATE_SCANNING) {
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			APP_DBG_MSG("⚠️ REJECTED: Device selection while scan in progress!\n");
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			APP_DBG_MSG("   Current state: %d (2=SCANNING)\n", current_state);
			APP_DBG_MSG("   Device list is incomplete - data would be corrupted\n");
			APP_DBG_MSG("   Wait for state change to PHONE_CONNECTED before selecting\n");
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			return;  /* Abort - scan must complete first */
		}
		
		/* CRITICAL FIX: Validate device list is not empty before connection attempt
		 * Problem: App can send device index 0 BEFORE performing any scan
		 * Result: MAC address is 00:00:00:00:00:00, causing 0xD1 COMMAND_DISALLOWED error
		 * Solution: Check device_count and reject if zero (no scan performed yet)
		 */
		extern uint8_t device_count;
		if (device_count == 0) {
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			APP_DBG_MSG("⚠️ REJECTED: No devices in list - scan first!\n");
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			APP_DBG_MSG("   Device index: %d\n", cmd);
			APP_DBG_MSG("   Device list is empty (device_count=0)\n");
			APP_DBG_MSG("   Perform scan (command 0x63) before connecting\n");
			APP_DBG_MSG("════════════════════════════════════════════════\n");
			return;  /* Abort - no devices to connect to */
		}
		
		APP_DBG_MSG("════════════════════════════════════════════════\n");
		APP_DBG_MSG("🔗 DUAL CONNECTION: Preparing peer connection\n");
		APP_DBG_MSG("════════════════════════════════════════════════\n");
		
		/* Get peer device info */
		uint8_t peer_addr[6] = {0};
		uint8_t peer_addr_type = 0;
		
		int result = DualRole_GetDeviceInfo(cmd, peer_addr, &peer_addr_type);
		
		if (result != 0) {
			APP_DBG_MSG("   ❌ Failed to get device info for index %d\n", cmd);
			APP_DBG_MSG("   Device list has %d devices\n", device_count);
			return;
		}
		
		APP_DBG_MSG("   Peer: %02X:%02X:%02X:%02X:%02X:%02X (type %d)\n",
		           peer_addr[5], peer_addr[4], peer_addr[3],
		           peer_addr[2], peer_addr[1], peer_addr[0],
		           peer_addr_type);
		
		/* Save to context for deferred connection */
		DualRole_Context_t *ctx = DualRole_GetContext();
		APP_DBG_MSG("🔍 DEBUG: ctx=%p\n", (void*)ctx);
		if (ctx) {
			/* CRITICAL: Check if this device is already connected as peer */
			if (ctx->peer_connected && ctx->peer_conn_handle != 0xFFFF) {
				/* Compare MAC addresses */
				int addr_match = 1;
				for (int i = 0; i < 6; i++) {
					if (ctx->peer_bd_addr[i] != peer_addr[i]) {
						addr_match = 0;
						break;
					}
				}
				
				if (addr_match) {
					APP_DBG_MSG("⚠️ This device is ALREADY CONNECTED as peer (handle 0x%04X)!\n", ctx->peer_conn_handle);
					APP_DBG_MSG("   Cannot connect to the same device twice.\n");
					APP_DBG_MSG("   Current state: %d (6=DUAL_ACTIVE means both connections active)\n", ctx->current_state);
					APP_DBG_MSG("════════════════════════════════════════════════\n");
					return;  /* Abort connection attempt */
				} else {
					APP_DBG_MSG("⚠️ Different device selected, but peer already connected!\n");
					APP_DBG_MSG("   Current peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
					           ctx->peer_bd_addr[5], ctx->peer_bd_addr[4], ctx->peer_bd_addr[3],
					           ctx->peer_bd_addr[2], ctx->peer_bd_addr[1], ctx->peer_bd_addr[0]);
					APP_DBG_MSG("   New peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
					           peer_addr[5], peer_addr[4], peer_addr[3],
					           peer_addr[2], peer_addr[1], peer_addr[0]);
					APP_DBG_MSG("   TODO: Implement peer switching (disconnect old, connect new)\n");
					APP_DBG_MSG("════════════════════════════════════════════════\n");
					return;  /* For now, abort - need to disconnect first */
				}
			}
			
			/* Guard: reject connect command when already in CONNECTING state.
			 * Root cause of stuck-at-state-4 bug:
			 *   auto-retry clears pending_peer_connection=0 BEFORE calling
			 *   aci_gap_create_connection. If phone sends 0x00 at that exact
			 *   moment it re-sets pending=1, main loop calls create_connection
			 *   again → 0x0C failure → state stuck at CONNECTING, scan blocked. */
			if (ctx->current_state == DUAL_STATE_CONNECTING) {
				APP_DBG_MSG("⚠️ Already CONNECTING (state=4) - ignoring connect command\n");
				APP_DBG_MSG("   Auto-retry will handle the connection automatically\n");
				break;
			}

			APP_DBG_MSG("🔍 DEBUG: Before set - pending=%d, device_index=%d\n", 
			           ctx->pending_peer_connection, ctx->pending_peer_device_index);
			memcpy(ctx->peer_bd_addr, peer_addr, 6);
			ctx->peer_bd_addr_type = peer_addr_type;
			ctx->peer_found = 1;
			ctx->pending_peer_connection = 1;  // Flag for main loop
			ctx->pending_peer_device_index = cmd;  // CRITICAL: Store device index!
			/* CRITICAL FIX: Set state to CONNECTING and disarm the 10s timer.
			 * Without these, state stays at PHONE_CONNECTED(1), the 10s timer fires
			 * during connection setup, restarts FAST advertising (20ms), and the
			 * radio scheduler can't fit phone(45ms)+advertising(20ms)+peer(50ms)
			 * simultaneously → peer supervision timeout → disconnect reason 0x08. */
			ctx->waiting_for_user_connect = 0;   /* Disarm 10s timer */
			ctx->retry_count = 0;                /* First attempt */
			DualRole_SetState(DUAL_STATE_CONNECTING);
			APP_DBG_MSG("🔍 DEBUG: After set - pending=%d, device_index=%d\n", 
			           ctx->pending_peer_connection, ctx->pending_peer_device_index);
			
			uint16_t phone_handle = DualRole_GetPhoneHandle();
			APP_DBG_MSG("📱 Phone connection: %s (handle 0x%04X)\n",
			           (phone_handle != 0xFFFF) ? "ACTIVE" : "disconnected", phone_handle);
			APP_DBG_MSG("⏰ Peer connection will be initiated from main loop\n");
			APP_DBG_MSG("   (This allows BLE stack to fully settle first)\n");
		}
		else {
			APP_DBG_MSG("❌ Invalid context!\n");
		}
	}
		break;
	}

	case 'c':
		/* Scan all devices OR return connected peer info */
		APP_DBG_MSG(">>>>>>>>>>>>>> SCAN COMMAND RECEIVED <<<<<<<<<<<<<<<<\n");
		APP_DBG_MSG("___SCAN/QUERY Devices Command\n");
		
		/* Check if peer is already connected */
		if (DualRole_IsPeerConnected()) {
			APP_DBG_MSG("📱 Peer already connected - sending connected device info (no scan needed)\n");
			
			/* Get connected peer info */
			char peer_name[32] = {0};
			uint8_t peer_mac[6] = {0};
			int8_t peer_rssi = 0;
			
			int result = DualRole_GetConnectedPeerInfo(peer_name, peer_mac, &peer_rssi);
			
			if (result == 0) {
				/* Build B_LIST with single connected device */
				blst_devices_num = 1;
				blst_index = 1;
				memset(blst, 0, sizeof(blst));
				
				/* Format: [count][len][name][MAC6][RSSI+128] */
				blst[0] = 1;  /* Device count */
				
				uint8_t name_len = strlen(peer_name);
				blst[blst_index++] = name_len;
				memcpy(&blst[blst_index], peer_name, name_len);
				blst_index += name_len;
				
			/* Add MAC address (6 bytes) - peer_mac is already reversed by DualRole_GetConnectedPeerInfo() */
			for (uint8_t i = 0; i < 6; i++) {
				blst[blst_index++] = peer_mac[i];
			}
				
				/* Add RSSI (offset by 128 to make it unsigned: 0-255 range) */
				blst[blst_index++] = (uint8_t)(peer_rssi + 128);
				
				APP_DBG_MSG("   Connected device: %s\n", peer_name);
				APP_DBG_MSG("   MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
				           peer_mac[0], peer_mac[1], peer_mac[2],
				           peer_mac[3], peer_mac[4], peer_mac[5]);
				APP_DBG_MSG("   RSSI: %d dBm (encoded as %d)\n", peer_rssi, (uint8_t)(peer_rssi + 128));
				
				/* Send B_LIST notification */
				blist_already_sent = 1;
				UTIL_SEQ_SetTask(1U << UPDATE_BLST, CFG_SEQ_PRIO_1);
			} else {
				APP_DBG_MSG("❌ Failed to get connected peer info\n");
			}
			
			break;  /* Don't perform scan */
		}
		
		/* Check if both connections are already established (should not reach here due to above check) */
		DualRole_Context_t *scan_ctx = DualRole_GetContext();
		if (scan_ctx && scan_ctx->current_state == DUAL_STATE_DUAL_ACTIVE) {
			APP_DBG_MSG("⚠️ Cannot scan: Both connections already established (state=6)\n");
			APP_DBG_MSG("   Peer is already connected! No need to scan.\n");
			break;  /* Ignore scan command */
		}
		
		/* Block scan when peer connection is in progress or already connected */
		if (scan_ctx && (scan_ctx->current_state == DUAL_STATE_CONNECTING ||
		                 scan_ctx->current_state == DUAL_STATE_PEER_CONNECTED)) {
			APP_DBG_MSG("⚠️ Cannot scan: peer connection in progress (state=%d)\n",
			            scan_ctx->current_state);
			break;
		}
		
		/* Check if scan is already in progress */
		extern uint8_t scan;
		if (scan) {
			APP_DBG_MSG("⚠️ Scan already in progress, ignoring duplicate request\n");
			break;
		}
		
		/* Scan cooldown: prevent rapid back-to-back scans.
		 * The Android app re-subscribes after each scan, which triggers another 0x63.
		 * Two consecutive 2-second scans (~5s total) exceed Android's 5000ms supervision
		 * timeout, causing 0x28 disconnections. Enforce a 5-second gap between scans.
		 * During cooldown, re-send the cached device list so the app isn't left empty. */
		{
			extern uint8_t device_count;
			DualRole_Context_t *cool_ctx = DualRole_GetContext();
			if (cool_ctx && cool_ctx->scan_complete_time > 0) {
				uint32_t since_last_scan = HAL_GetTick() - cool_ctx->scan_complete_time;
				if (since_last_scan < 5000) {
					APP_DBG_MSG("⏳ Scan cooldown: %lu ms since last scan (5s gap required)\n",
					            (unsigned long)since_last_scan);
					if (device_count > 0) {
						blist_already_sent = 0;
						UTIL_SEQ_SetTask(1U << UPDATE_BLST, CFG_SEQ_PRIO_1);
						APP_DBG_MSG("📋 Re-sending cached list (%d devices) instead of re-scanning\n",
						            device_count);
					}
					break;
				}
			}
		}

		/* Reset the B_LIST sent flag when starting new scan */
		blist_already_sent = 0;
		
		/* Start scanning for ALL devices using DualRole_StartPeerScan() function */
		extern void DualRole_StartPeerScan(const char *peer_name);
		DualRole_StartPeerScan(NULL);  /* NULL = scan for all devices */
		break;
	
	case 'd': {
		/* Disconnect from peer */
		APP_DBG_MSG("════════════════════════════════════════════════\n");
		APP_DBG_MSG("🔴 PEER DISCONNECT Command Received!\n");
		APP_DBG_MSG("════════════════════════════════════════════════\n");
		
		if (DualRole_IsPeerConnected()) {
			uint16_t peer_handle = DualRole_GetPeerHandle();
			APP_DBG_MSG("✅ Peer is connected (handle: 0x%04X)\n", peer_handle);
			APP_DBG_MSG("📡 Cleaning up peer connection state...\n");
			/* 0x13 = local user requested disconnect - no auto-retry will be scheduled */
			DualRole_OnPeerDisconnected(0x13);
			
			APP_DBG_MSG("📤 Sending GAP terminate command...\n");
			tBleStatus result = aci_gap_terminate(peer_handle, 0x13);
			APP_DBG_MSG("📋 aci_gap_terminate result: 0x%02X %s\n", 
			           result, 
			           result == BLE_STATUS_SUCCESS ? "(SUCCESS)" : "(FAILED)");
		} else {
			APP_DBG_MSG("⚠️ No peer connected - nothing to disconnect\n");
		}
		APP_DBG_MSG("════════════════════════════════════════════════\n");
		break;
	}
	
	case 's': {
			/* NEW: Status query */
			APP_DBG_MSG("___STATUS QUERY Command\n");
			
			uint8_t current_state = (uint8_t)DualRole_GetState();
			
			BCONNECTION_SERVER_Data_t status_data;
			status_data.p_Payload = &current_state;
			status_data.Length = 1;
			
			BCONNECTION_SERVER_NotifyValue(
				BCONNECTION_SERVER_BLIST_STATE_CHAR,
				&status_data,
				p_Notification->ConnectionHandle
			);
			
			APP_DBG_MSG("Current state: 0x%02X\n", current_state);
			break;
		}
		
		case 'v': {
			/* NEW: View change command
			 * View 0 = Map (environmental sensors @ 1Hz)
			 * View 1 = 3D (motion sensors @ 10Hz)
			 * View 2 = Cam
			 */
			if (p_Notification->DataTransfered.Length >= 2) {
				uint8_t new_view = p_Notification->DataTransfered.p_Payload[1];
				
				if (new_view <= 2) {  /* Valid views: 0, 1, 2 */
					SENSOR_SERVER_APP_Handle_ViewChange(new_view);  /* This updates g_current_view and starts/stops TIM1 */
					APP_DBG_MSG("📱 VIEW CHANGED: %d (%s)\n", new_view,
					           new_view == 0 ? "Map" : (new_view == 1 ? "3D" : "Cam"));
				} else {
					APP_DBG_MSG("⚠️ Invalid view value: %d (must be 0, 1, or 2)\n", new_view);
				}
			} else {
				APP_DBG_MSG("⚠️ View command requires 2 bytes (cmd + view)\n");
			}
			break;
		}
		
		default:
			/* Not a command - might be message data for relay */
			if (cmd >= 0x20 && cmd <= 0x7E) {
				/* Printable ASCII - treat as message relay */
				APP_DBG_MSG("___MESSAGE RELAY: %d bytes from phone\n", 
				           p_Notification->DataTransfered.Length);
				
				/* Relay message from phone to peer */
				DualRole_RelayFromPhoneToPeer(
					p_Notification->DataTransfered.p_Payload,
					p_Notification->DataTransfered.Length
				);
			} else {
				APP_DBG_MSG("Unknown command: 0x%02X\n", cmd);
			}
			break;
		}

      /* USER CODE END Service3Char1_WRITE_NO_RESP_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service3Char1_NOTIFY_ENABLED_EVT */
      APP_DBG_MSG("📱 B_STATE notifications ENABLED - sending current dual-role state\n");
      
      // Send current dual-role state immediately when phone subscribes
      {
        extern DualRole_State_t DualRole_GetState(void);
        uint8_t current_state = (uint8_t)DualRole_GetState();
        BCONNECTION_SERVER_Data_t state_data;
        state_data.p_Payload = &current_state;
        state_data.Length = 1;
        
        tBleStatus result = BCONNECTION_SERVER_NotifyValue(
          BCONNECTION_SERVER_BLIST_STATE_CHAR,
          &state_data,
          p_Notification->ConnectionHandle
        );
        
        if (result == BLE_STATUS_SUCCESS) {
          APP_DBG_MSG("✅ Sent current state: %d to phone\n", current_state);
        } else {
          APP_DBG_MSG("⚠️ Failed to send current state, error: 0x%02X\n", result);
        }
      }
      /* USER CODE END Service3Char1_NOTIFY_ENABLED_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_STATE_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service3Char1_NOTIFY_DISABLED_EVT */
      APP_DBG_MSG("📱 B_STATE notifications DISABLED\n");
      /* USER CODE END Service3Char1_NOTIFY_DISABLED_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_CHAR_READ_EVT:
      /* USER CODE BEGIN Service3Char2_READ_EVT */

      /* USER CODE END Service3Char2_READ_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service3Char2_NOTIFY_ENABLED_EVT */
    	BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status = Blist_char_NOTIFICATION_ON;
    	APP_DBG_MSG("________Blist_char_NOTIFICATION_ON .\n");
    	
    	/* Strict debouncing: Ignore repeated subscription events within 3 seconds */
    	uint32_t current_time = HAL_GetTick();
    	if ((current_time - last_subscription_time) < 3000) {
    		APP_DBG_MSG("⚠️ Debouncing duplicate subscription event (%.1fs since last)\n", 
    		           (current_time - last_subscription_time) / 1000.0f);
    		return;
    	}
    	last_subscription_time = current_time;
    	
    	/* If device list already exists and not yet sent, send it now */
    	extern uint8_t blst_devices_num;
    	if (blst_devices_num > 0 && blist_already_sent == 0) {
    		APP_DBG_MSG("📤 Device list exists (%d devices), sending to newly subscribed app...\n", blst_devices_num);
    		blist_already_sent = 1;  // Mark as sent
    		UTIL_SEQ_SetTask(1U << UPDATE_BLST, CFG_SEQ_PRIO_1);
    	} else if (blist_already_sent == 1) {
    		APP_DBG_MSG("⚠️ Device list already sent for this scan, ignoring\n");
    	}
      /* USER CODE END Service3Char2_NOTIFY_ENABLED_EVT */
      break;

    case BCONNECTION_SERVER_BLIST_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service3Char2_NOTIFY_DISABLED_EVT */
    	BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status = Blist_char_NOTIFICATION_OFF;
    	APP_DBG_MSG("________Blist_char_NOTIFICATION_OFF .\n");
    	
    	/* Reset the sent flag when notifications are disabled */
    	blist_already_sent = 0;
      /* USER CODE END Service3Char2_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service3_Notification_default */

      /* USER CODE END Service3_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service3_Notification_2 */

  /* USER CODE END Service3_Notification_2 */
  return;
}

void BCONNECTION_SERVER_APP_EvtRx(BCONNECTION_SERVER_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service3_APP_EvtRx_1 */

  /* USER CODE END Service3_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service3_APP_EvtRx_Service3_EvtOpcode */

    /* USER CODE END Service3_APP_EvtRx_Service3_EvtOpcode */
    case BCONNECTION_SERVER_CENTR_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service3_APP_CENTR_CONN_HANDLE_EVT */

      /* USER CODE END Service3_APP_CENTR_CONN_HANDLE_EVT */
      break;

    case BCONNECTION_SERVER_PERIPH_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service3_APP_PERIPH_CONN_HANDLE_EVT */
    	 BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl = p_Notification->ConnectionHandle;
      /* USER CODE END Service3_APP_PERIPH_CONN_HANDLE_EVT */
      break;

    case BCONNECTION_SERVER_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service3_APP_DISCON_HANDLE_EVT */

      /* USER CODE END Service3_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service3_APP_EvtRx_default */

      /* USER CODE END Service3_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service3_APP_EvtRx_2 */

  /* USER CODE END Service3_APP_EvtRx_2 */

  return;
}

/**
 * @brief Clear all notification subscription flags
 * @note Called when phone disconnects to prevent stale subscription state
 */
void BCONNECTION_SERVER_APP_ClearSubscriptionFlags(void)
{
  BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status = Blist_char_NOTIFICATION_OFF;
}

void BCONNECTION_SERVER_APP_Init(void)
{
  BCONNECTION_SERVER_Init();

  /* USER CODE BEGIN Service3_APP_Init */
  UTIL_SEQ_RegTask( 1U << UPDATE_BLST, UTIL_SEQ_RFU, BCONNECTION_SERVER_Blist_char_SendNotification);
  /* USER CODE END Service3_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void BCONNECTION_SERVER_Blist_char_SendNotification(void) /* Property Notification */
{
  BCONNECTION_SERVER_APP_SendInformation_t notification_on_off = Blist_char_NOTIFICATION_OFF;
  BCONNECTION_SERVER_Data_t bconnection_server_notification_data;

  bconnection_server_notification_data.p_Payload = (uint8_t*)a_BCONNECTION_SERVER_UpdateCharData;
  bconnection_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service3Char2_NS_1*/
  /* FIXED: Use blst_index (actual data length) instead of searching for '\0'
   * Original bug: while(blst[i] != '\0') terminated early (sent only 2 bytes)
   * Fix: Copy exactly blst_index-1 bytes (blst[1] through blst[blst_index-1])
   */
  extern uint8_t blst_index;  /* Actual end position of data in blst[] */
  extern uint8_t device_count;  /* Number of devices found during scan */
  
  APP_DBG_MSG("🔍 DEBUG BEFORE SEND:\n");
  APP_DBG_MSG("  device_count = %d\n", device_count);
  APP_DBG_MSG("  blst_devices_num = %d\n", blst_devices_num);
  APP_DBG_MSG("  blst_index = %d\n", blst_index);
  
  //first byte hold num of devices found
  a_BCONNECTION_SERVER_UpdateCharData[0] = blst_devices_num; /* Number of devices found */
  
  APP_DBG_MSG(">>>>>>>>>>>>>>Start updating blst with data:\n%d\n", blst_devices_num);
  APP_DBG_MSG("📏 blst_index = %d (total data length)\n", blst_index);

  /* Copy device list data: blst[1] through blst[blst_index-1] */
  for (int i = 1; i < blst_index; i++) {
      a_BCONNECTION_SERVER_UpdateCharData[i] = blst[i];
  }
  
  /* Total length = 1 byte (count) + (blst_index - 1) bytes (device data) */
  bconnection_server_notification_data.Length = blst_index;
  
  APP_DBG_MSG("<<<<<<<<<<<<<<<<<<<<<<End updating blst with data: %d devices, %d total bytes\n", 
              blst_devices_num, bconnection_server_notification_data.Length);
  
  /* DEBUG: Check notification status and connection handle */
  APP_DBG_MSG("🔍 DEBUG: Notification Status = %d (0=OFF, 1=ON)\n", BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status);
  APP_DBG_MSG("🔍 DEBUG: Connection Handle = 0x%04X (0xFFFF=invalid)\n", BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl);
  
      if(BCONNECTION_SERVER_APP_Context.Blist_char_Notification_Status ==Blist_char_NOTIFICATION_ON)
      {
    	  APP_DBG_MSG("__update blst with notification.\n");
        notification_on_off = Blist_char_NOTIFICATION_ON;
      }
      else
      {
      	//just update char without send notification
    	  APP_DBG_MSG("__update blst without notification.\n");
    	  BCONNECTION_SERVER_UpdateValue(BCONNECTION_SERVER_BLIST_CHAR, &bconnection_server_notification_data);

      }
  /* USER CODE END Service3Char2_NS_1*/

  if (notification_on_off != Blist_char_NOTIFICATION_OFF && BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl != 0xFFFF)
  {
    tBleStatus ret = BCONNECTION_SERVER_NotifyValue(BCONNECTION_SERVER_BLIST_CHAR, &bconnection_server_notification_data, BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl);
    APP_DBG_MSG("📤 B_LIST notification attempt 1, result = 0x%02X\n", ret);
    
    /* Retry if notification queue is full (0x0C = BLE_STATUS_INSUFFICIENT_RESOURCES) */
    if (ret == 0x0C) {
      APP_DBG_MSG("⚠️ Queue full, retrying in 50ms...\n");
      HAL_Delay(50);
      ret = BCONNECTION_SERVER_NotifyValue(BCONNECTION_SERVER_BLIST_CHAR, &bconnection_server_notification_data, BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl);
      APP_DBG_MSG("📤 B_LIST notification attempt 2, result = 0x%02X\n", ret);
      
      if (ret == 0x0C) {
        APP_DBG_MSG("⚠️ Still full, retrying in 100ms...\n");
        HAL_Delay(100);
        ret = BCONNECTION_SERVER_NotifyValue(BCONNECTION_SERVER_BLIST_CHAR, &bconnection_server_notification_data, BCONNECTION_SERVER_APP_Context.BCONNECTION_SERVER_periph_connHdl);
        APP_DBG_MSG("📤 B_LIST notification attempt 3, result = 0x%02X\n", ret);
      }
    }
    
    if (ret == BLE_STATUS_SUCCESS) {
      APP_DBG_MSG("✅ B_LIST notification delivered successfully!\n");
    } else {
      APP_DBG_MSG("❌ B_LIST notification failed after retries: 0x%02X\n", ret);
    }
  }

  /* USER CODE BEGIN Service3Char2_NS_Last*/
   //set state to 'R' (Ready)
   a_BCONNECTION_SERVER_UpdateCharData[0]='R';
   bconnection_server_notification_data.Length = 1;
   BCONNECTION_SERVER_UpdateValue(BCONNECTION_SERVER_BLIST_STATE_CHAR, &bconnection_server_notification_data);

  /* USER CODE END Service3Char2_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
