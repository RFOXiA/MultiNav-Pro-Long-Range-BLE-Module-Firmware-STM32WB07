/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dual_role_manager.c
  * @author  Dual-Role Integration
  * @brief   Dual-role connection manager implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "dual_role_manager.h"
#include "uart_relay_service.h"
#include "bconnection_server.h"
#include "sensor_server_app.h"  /* For updating phone connection handle */
#include "gatt_client_app.h"     /* For subscribing to peer sensors */
#include "ble.h"
#include "stm32_seq.h"           /* For UTIL_SEQ_SetTask */
#include <string.h>
#include <stdlib.h>  /* For malloc/free in test data exchange */

/* Device list storage (defined here, declared as extern in header) */
DeviceInfo_t device_list[MAX_DEVICES];
uint8_t device_count = 0;

/* External device list variables */
extern uint8_t blst_devices_num;
extern uint8_t blst_index;
extern uint8_t blst[255];

/* Global context */
static DualRole_Context_t g_dual_role_ctx = {0};

void DualRole_Init(void)
{
    memset(&g_dual_role_ctx, 0, sizeof(DualRole_Context_t));
    g_dual_role_ctx.phone_conn_handle = 0xFFFF;
    g_dual_role_ctx.peer_conn_handle = 0xFFFF;
    g_dual_role_ctx.current_state = DUAL_STATE_IDLE;
    
    /* Initialize pending connection flags */
    g_dual_role_ctx.pending_peer_connection = 0;
    g_dual_role_ctx.pending_peer_device_index = 0xFF;
    
    /* Initialize auto-retry settings */
    g_dual_role_ctx.auto_retry_enabled = 1;  /* Enable by default */
    g_dual_role_ctx.retry_count = 0;
    g_dual_role_ctx.max_retries = 3;  /* Retry up to 3 times */
    g_dual_role_ctx.last_retry_time = 0;
    
    APP_DBG_MSG("Dual-Role Manager initialized\n");
}

DualRole_State_t DualRole_GetState(void)
{
    return g_dual_role_ctx.current_state;
}

void DualRole_SetState(DualRole_State_t new_state)
{
    APP_DBG_MSG("Dual-Role State: %d → %d\n", g_dual_role_ctx.current_state, new_state);
    g_dual_role_ctx.current_state = new_state;
    
    /* Notify phone via B_STATE characteristic */
    /* CRITICAL: Only send if phone is connected AND subscribed to prevent 0x0C errors */
    extern uint8_t BCONNECTION_SERVER_APP_IsBListStateSubscribed(void);
    if (g_dual_role_ctx.phone_connected && BCONNECTION_SERVER_APP_IsBListStateSubscribed()) {
        /* Validate connection handle before sending */
        if (g_dual_role_ctx.phone_conn_handle == 0xFFFF) {
            APP_DBG_MSG("⚠️ Skipping B_STATE notification - phone handle invalid\n");
            return;
        }
        
        uint8_t state_data = (uint8_t)new_state;
        BCONNECTION_SERVER_Data_t data;
        data.p_Payload = &state_data;
        data.Length = 1;
        
        tBleStatus ret = BCONNECTION_SERVER_NotifyValue(
            BCONNECTION_SERVER_BLIST_STATE_CHAR,
            &data,
            g_dual_role_ctx.phone_conn_handle
        );
        
        /* Log failures but don't crash - connection may be transitioning */
        if (ret != BLE_STATUS_SUCCESS && ret != BLE_STATUS_INSUFFICIENT_RESOURCES) {
            APP_DBG_MSG("⚠️ B_STATE notification failed: 0x%02X (non-fatal, continuing)\n", ret);
        }
    }
}

void DualRole_OnPhoneConnected(uint16_t conn_handle)
{
    /* Check if phone was already connected with different handle */
    if (g_dual_role_ctx.phone_connected && g_dual_role_ctx.phone_conn_handle != conn_handle) {
        APP_DBG_MSG("⚠️ Phone reconnected with NEW handle (old=0x%04X, new=0x%04X)\n",
                   g_dual_role_ctx.phone_conn_handle, conn_handle);
        APP_DBG_MSG("   This indicates the phone disconnected and reconnected.\n");
    }
    
    g_dual_role_ctx.phone_conn_handle = conn_handle;
    g_dual_role_ctx.phone_connected = 1;
    
    /* CRITICAL FIX: Update sensor server's peripheral connection handle
     * Issue: When phone reconnects with new handle (0x0802, 0x0803, etc.),
     * sensor server still has old handle, causing notification error 0x0C
     * Solution: Explicitly notify sensor server to update its handle */
    SENSOR_SERVER_APP_ConnHandleNotEvt_t sensor_notif;
    sensor_notif.EvtOpcode = SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT;
    sensor_notif.ConnectionHandle = conn_handle;
    SENSOR_SERVER_APP_EvtRx(&sensor_notif);
    APP_DBG_MSG("✅ Updated sensor server with phone handle 0x%04X\n", conn_handle);
    
    /* CRITICAL: Check if peer is still connected before changing state */
    if (g_dual_role_ctx.peer_connected) {
        /* Phone reconnected while peer is still active - stay in DUAL_ACTIVE */
        APP_DBG_MSG("✅ Phone reconnected with peer still active (handle 0x%04X)\n", 
                   g_dual_role_ctx.peer_conn_handle);
        APP_DBG_MSG("   Maintaining DUAL_ACTIVE state (6)\n");
        DualRole_SetState(DUAL_STATE_DUAL_ACTIVE);
    } else {
        /* Normal phone-only connection */
        DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
    }
    
    APP_DBG_MSG("Phone connected: handle 0x%04X\n", conn_handle);
    
    /* NOTE: The L2CAP supervision-timeout update (was: change 5s→20s) is
     * intentionally NOT sent.  The scan is capped at 2 s (duration=200×10ms)
     * which fits comfortably within Android's default 5000ms supervision
     * timeout.  Sending the L2CAP update causes 0x28 "Instant Passed"
     * disconnections because the BLE LL tries to apply the parameter change
     * at a future "instant" that races with scan startup. */
    g_dual_role_ctx.l2cap_update_pending = 0;  /* keep clear — update not needed */
    g_dual_role_ctx.phone_connect_time = HAL_GetTick();
}

void DualRole_OnPhoneDisconnected(void)
{
    g_dual_role_ctx.phone_conn_handle = 0xFFFF;
    g_dual_role_ctx.phone_connected = 0;
    
    /* Clear deferred-request and 10s-timer flags so they don't fire after reconnection */
    g_dual_role_ctx.l2cap_update_pending = 0;
    g_dual_role_ctx.waiting_for_user_connect = 0;
    
    /* CRITICAL FIX: Reset view to Map (environmental sensors) on disconnection
     * Issue: If phone was in 3D view (motion sensors) and disconnects,
     * g_current_view stays at 1 and TIM1 keeps running. When phone reconnects,
     * app may start in Map view but firmware still has TIM1 running.
     * Solution: Reset to Map view (0) and stop TIM1 on disconnection. */
    SENSOR_SERVER_APP_Handle_ViewChange(0);  /* Reset to Map view - stops TIM1 */
    APP_DBG_MSG("🔄 Reset view to Map (TIM1 stopped for reconnection)\n");
    
    /* CRITICAL FIX: Clear all subscription flags on disconnection
     * Issue: When phone disconnects and reconnects with new handle,
     * old subscription flags are still ON, causing firmware to send
     * notifications before phone re-subscribes (0x0C errors)
     * Solution: Clear all subscription flags so they must be re-enabled
     * on the new connection */
    extern void SENSOR_SERVER_APP_ClearSubscriptionFlags(void);
    extern void BCONNECTION_SERVER_APP_ClearSubscriptionFlags(void);
    SENSOR_SERVER_APP_ClearSubscriptionFlags();
    BCONNECTION_SERVER_APP_ClearSubscriptionFlags();
    APP_DBG_MSG("🔄 Cleared all subscription flags (will resubscribe on reconnection)\n");
    
    /* Reset B_LIST sent flag so new subscription will trigger device list send */
    extern void BCONNECTION_SERVER_APP_ResetBListSentFlag(void);
    BCONNECTION_SERVER_APP_ResetBListSentFlag();
    APP_DBG_MSG("🔄 Reset B_LIST sent flag (ready for reconnection)\n");
    
    /* Check if we have a pending peer connection */
    if (g_dual_role_ctx.pending_peer_connection) {
        APP_DBG_MSG("═══════════════════════════════════════════════════\n");
        APP_DBG_MSG("⚠️ Phone disconnected with pending peer connection!\n");
        APP_DBG_MSG("🔄 Restarting advertising so phone can reconnect...\n");
        APP_DBG_MSG("═══════════════════════════════════════════════════\n");
        
        /* CRITICAL: Restart advertising immediately so phone can reconnect
         * Advertising was disabled during scan, and if phone disconnects
         * before peer connection happens, advertising stays OFF forever
         * and phone can't reconnect. Restart it now. */
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
        
        /* Set state to IDLE and wait for phone to reconnect
         * Keep pending flags set so main loop will connect after reconnection */
        DualRole_SetState(DUAL_STATE_IDLE);
        APP_DBG_MSG("💤 Waiting for phone reconnection (pending flags preserved)\n");
        APP_DBG_MSG("📌 pending_peer_connection=%d, device_index=%d\n",
                   g_dual_role_ctx.pending_peer_connection,
                   g_dual_role_ctx.pending_peer_device_index);
        
        return;  /* Don't clear flags - main loop needs them after reconnection */
    }
    
    /* CRITICAL: Check if peer is still connected */
    if (g_dual_role_ctx.peer_connected) {
        APP_DBG_MSG("📱 Phone disconnected, but peer still connected (handle 0x%04X)\n", 
                   g_dual_role_ctx.peer_conn_handle);
        APP_DBG_MSG("   Transitioning to PEER_CONNECTED state (5)\n");
        DualRole_SetState(DUAL_STATE_PEER_CONNECTED);
        
        /* Restart advertising so phone can reconnect */
        APP_DBG_MSG("🔄 Restarting advertising for phone reconnection...\n");
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
        
        return;  /* Don't disconnect peer or go to IDLE */
    }
    
    /* No peer connected - normal shutdown */
    DualRole_SetState(DUAL_STATE_IDLE);
    APP_DBG_MSG("Phone disconnected\n");
    
    /* CRITICAL: ALWAYS restart advertising after phone disconnect
     * This matches the backup firmware behavior (always advertise after disconnect)
     * Phone needs to be able to reconnect immediately */
    APP_DBG_MSG("🔄 Restarting advertising for phone reconnection...\n");
    APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
    APP_DBG_MSG("✅ Advertising restarted - phone can reconnect\n");
}

void DualRole_StartPeerScan(const char *peer_name)
{
    if (!g_dual_role_ctx.phone_connected) {
        APP_DBG_MSG("Cannot scan: Phone not connected\n");
        DualRole_SetState(DUAL_STATE_ERROR);
        return;
    }
    
    /* Store peer name for filtering (NULL = scan for all devices)
     * Do NOT clear device list yet — only clear it once we confirm scan starts.
     * If scan fails (0x0C resources busy), the old results remain usable. */
    if (peer_name != NULL) {
        strncpy(g_dual_role_ctx.peer_name, peer_name, sizeof(g_dual_role_ctx.peer_name) - 1);
        g_dual_role_ctx.peer_name[sizeof(g_dual_role_ctx.peer_name) - 1] = '\0';
    } else {
        g_dual_role_ctx.peer_name[0] = '\0';  /* Empty = scan for all devices */
    }
    g_dual_role_ctx.peer_found = 0;
    
    DualRole_SetState(DUAL_STATE_SCANNING);
    
    /* CRITICAL: Pause sensor notifications during scan to prevent radio congestion */
    extern volatile uint8_t g_pause_sensor_notifications;
    g_pause_sensor_notifications = 1;
    APP_DBG_MSG("⏸️ Sensor notifications paused for scan duration\n");
    
    /* CRITICAL: Stop advertising before scanning - BLE stack doesn't allow both simultaneously */
    APP_DBG_MSG("🛑 Stopping advertising to enable scanning...\n");
    APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_STOP);
    
    /* Use the working APP_BLE_Procedure_Gap_Central function which handles all the BLE stack calls */
    APP_DBG_MSG("📡 Starting peer scan via APP_BLE_Procedure_Gap_Central...\n");
    APP_BLE_Procedure_Gap_Central(PROC_GAP_CENTRAL_SCAN_START);
    
    /* CRITICAL: Check if scan actually started.
     * APP_BLE_Procedure_Gap_Central resets state to PHONE_CONNECTED on failure.
     * If state is no longer SCANNING, the scan failed (e.g. 0x0C INSUFFICIENT_RESOURCES
     * because aci_gap_create_connection is still pending from auto-retry).
     * Do NOT set scan=1 or clear the device list — preserve previous scan results
     * so the user can still press connect on the cached device. */
    if (g_dual_role_ctx.current_state != DUAL_STATE_SCANNING) {
        g_pause_sensor_notifications = 0;  /* Resume notifications */
        APP_DBG_MSG("❌ Scan start failed - preserving device list, scan flag NOT set\n");
        return;
    }
    
    /* Scan started successfully — NOW clear device list and set scan flag */
    device_count = 0;
    blst_devices_num = 0;
    blst_index = 1;
    memset(device_list, 0, sizeof(device_list));
    memset(blst, 0, sizeof(blst));
    
    /* Reset B_LIST notification flag for new scan */
    extern void BCONNECTION_SERVER_APP_ResetBListSentFlag(void);
    BCONNECTION_SERVER_APP_ResetBListSentFlag();
    
    /* Set scan flag so app_ble.c will call DualRole_BuildDeviceListData() on completion */
    extern uint8_t scan;
    scan = 1;
    APP_DBG_MSG("✅ Scan flag set (will trigger device list build on completion)\n");
}

void DualRole_StopPeerScan(void)
{
    /* CRITICAL: Terminate the SAME procedure type we started!
     * We start with GAP_GENERAL_DISCOVERY_PROC, so terminate that
     * Sub firmware does: aci_gap_terminate_proc(GAP_GENERAL_DISCOVERY_PROC)
     */
    tBleStatus ret = aci_gap_terminate_proc(GAP_GENERAL_DISCOVERY_PROC);
    APP_DBG_MSG("Peer scan stopped (status: 0x%02X)\n", ret);
}

void DualRole_OnAdvertisingReport(uint8_t addr_type, uint8_t *addr, uint8_t data_len, uint8_t *data_rssi, uint8_t event_type)
{
    /* Only process if we're scanning for peer */
    if (g_dual_role_ctx.current_state != DUAL_STATE_SCANNING) {
        return;
    }
    
    /* Check if device list is full */
    if (device_count >= MAX_DEVICES) {
        return;
    }
    
    /* Parse advertising data for device name */
    char device_name[32] = {0};
    uint8_t name_len = 0;
    
    /* CRITICAL: Extract RSSI from last byte of data_rssi buffer
     * BLE advertising report format: [AD_Data...][RSSI_byte]
     * RSSI is signed 8-bit value (-127 to +20 dBm) */
    int8_t rssi = (data_len > 0) ? (int8_t)data_rssi[data_len] : 0;
    
    for (uint8_t i = 0; i < data_len;) {
        uint8_t len = data_rssi[i];
        if (len == 0) break;
        
        uint8_t type = data_rssi[i + 1];
        
        if (type == 0x08 || type == 0x09) {  /* Complete/Shortened Local Name */
            name_len = (len - 1 > 31) ? 31 : (len - 1);
            memcpy(device_name, &data_rssi[i + 2], name_len);
            device_name[name_len] = '\0';
        }
        
        i += (len + 1);
        if (i >= data_len) break;
    }
    
    /* Only add devices with names */
    if (name_len == 0) {
        return;
    }
    
    /* Check if device already in list (avoid duplicates) */
    for (uint8_t i = 0; i < device_count; i++) {
        if (memcmp(device_list[i].addr, addr, 6) == 0) {
            return;  /* Already in list */
        }
    }
    
    /* Add device to list */
    memcpy(device_list[device_count].addr, addr, 6);
    device_list[device_count].addr_type = addr_type;
    memcpy(device_list[device_count].name, device_name, name_len);
    device_list[device_count].name[name_len] = '\0';
    device_list[device_count].name_len = name_len;
    device_list[device_count].rssi = rssi;
    
    APP_DBG_MSG("[%d] Found: %s (%02X:%02X:%02X:%02X:%02X:%02X) RSSI: %d dBm\n", 
                device_count + 1,
                device_name,
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
                rssi);
    
    /* CRITICAL: Disable per-device logging - log flood blocks CPU ~5-10ms per device
     * During scan, 20+ devices = 100-200ms of blocked processing
     * This prevents connection supervision events from being handled, causing 0x28 timeout
     * Solution: Log only total count at scan completion (in DualRole_BuildDeviceListData) */
    // APP_DBG_MSG("[%d] Found: %s (%02X:%02X:%02X:%02X:%02X:%02X) RSSI: %d dBm\n", 
    //             device_count + 1,
    //             device_name,
    //             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
    //             rssi);
    
    device_count++;
    blst_devices_num = device_count;  // FIXED: Removed duplicate increment
}

void DualRole_ClearDeviceList(void)
{
    device_count = 0;
    blst_devices_num = 0;
    blst_index = 1;
    memset(device_list, 0, sizeof(device_list));
    memset(blst, 0, sizeof(blst));
    
    /* Reset B_LIST notification flag when clearing device list */
    extern void BCONNECTION_SERVER_APP_ResetBListSentFlag(void);
    BCONNECTION_SERVER_APP_ResetBListSentFlag();
}

void DualRole_ConnectToPeer(void)
{
    tBleStatus ret;
    
    if (!g_dual_role_ctx.peer_found) {
        APP_DBG_MSG("Cannot connect: Peer not found\n");
        DualRole_SetState(DUAL_STATE_ERROR);
        return;
    }
    
    /* CRITICAL: Only stop scan if we're still in scanning state
     * If scan completed naturally (ACI_GAP_PROC_COMPLETE_VSEVT_CODE), state already
     * transitioned to PHONE_CONNECTED. Trying to stop already-completed scan
     * puts controller in confused state, causing 0xD1 on connection attempts
     * Check current state BEFORE changing it
     */
    uint8_t scan_was_active = (g_dual_role_ctx.current_state == DUAL_STATE_SCANNING);
    
    DualRole_SetState(DUAL_STATE_CONNECTING);
    
    if (scan_was_active) {
        APP_DBG_MSG("🛑 Scan still active, stopping...\n");
        DualRole_StopPeerScan();
        /* No HAL_Delay here - scan completion arrives as ACI_GAP_PROC_COMPLETE_VSEVT_CODE.
         * The main loop will retry aci_gap_create_connection if it returns 0x12 (busy). */
        APP_DBG_MSG("✅ Scan stop requested (non-blocking - event confirms completion)\n");
    } else {
        APP_DBG_MSG("✅ Scan already completed, skipping stop\n");
    }
    
    /* Check if we're in event-driven mode (advertising already disabled before phone disconnect) */
    if (g_dual_role_ctx.pending_peer_connection == 0) {
        /* Normal mode: disable advertising now */
        APP_DBG_MSG("🚫 Cancelling advertising timeout and disabling advertising...\n");
        g_dual_role_ctx.waiting_for_user_connect = 0;
        
        ret = aci_gap_set_advertising_enable(DISABLE, 0, NULL);
        APP_DBG_MSG("   aci_gap_set_advertising_enable(DISABLE) result: 0x%02X\n", ret);
        /* No HAL_Delay here - advertising disable is synchronous at HCI level.
         * The main loop handles the 100ms settle wait non-blockingly. */
    } else {
        /* Event-driven mode: advertising already disabled, waiting_for_user_connect already 0 */
        APP_DBG_MSG("✅ Event-driven mode: advertising already disabled\n");
    }
    
    /* CRITICAL: Pause sensor notifications to reduce radio congestion
     * Phone connection at 15ms intervals + sensor notifications every 1s
     * creates too much ATT traffic. Pause notifications during connection.
     */
    extern volatile uint8_t g_pause_sensor_notifications;
    APP_DBG_MSG("⏸️  Pausing sensor notifications to reduce radio congestion...\n");
    g_pause_sensor_notifications = 1;
    
    APP_DBG_MSG("\n🔌 Connecting to peer from clean BLE state...\n");
    APP_DBG_MSG("   Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                g_dual_role_ctx.peer_bd_addr[5],
                g_dual_role_ctx.peer_bd_addr[4],
                g_dual_role_ctx.peer_bd_addr[3],
                g_dual_role_ctx.peer_bd_addr[2],
                g_dual_role_ctx.peer_bd_addr[1],
                g_dual_role_ctx.peer_bd_addr[0]);
    
    /* After clean BLE_Init(), stack is guaranteed idle - no need to terminate procedures */
    APP_DBG_MSG("📝 Configuring connection parameters (clean idle state)...\n");
    ret = aci_gap_set_connection_configuration(
        LE_1M_PHY_BIT,                     /* 1M PHY only (matches our init) */
        CONN_INT_MS(50),                   /* Min interval: 50ms */
        CONN_INT_MS(100),                  /* Max interval: 100ms */
        0,                                  /* Latency: 0 */
        CONN_SUP_TIMEOUT_MS(10000),        /* Timeout: 10s (was 5s - increased for stability) */
        CONN_CE_LENGTH_MS(10),             /* Min CE length */
        CONN_CE_LENGTH_MS(10)              /* Max CE length */
    );
    APP_DBG_MSG("   aci_gap_set_connection_configuration result: 0x%02X %s\n", ret,
               (ret == BLE_STATUS_SUCCESS) ? "✅" : "❌");
    
    if (ret != BLE_STATUS_SUCCESS) {
        APP_DBG_MSG("❌ Connection configuration failed! Cannot proceed.\n");
        g_pause_sensor_notifications = 0;
        DualRole_SetState(DUAL_STATE_ERROR);
        return;
    }
    
    /* Now create the connection (1M PHY only to match config) */
    APP_DBG_MSG("   Creating connection with 1M PHY only...\n");
    
    ret = aci_gap_create_connection(
        LE_1M_PHY_BIT,                     /* 1M PHY only (matches config) */
        g_dual_role_ctx.peer_bd_addr_type,
        g_dual_role_ctx.peer_bd_addr
    );
    
    APP_DBG_MSG("🔌 aci_gap_create_connection result: 0x%02X\n", ret);
    
    /* Resume sensor notifications after connection attempt */
    APP_DBG_MSG("▶️  Resuming sensor notifications...\n");
    g_pause_sensor_notifications = 0;
    
    if (ret == BLE_STATUS_SUCCESS) {
        APP_DBG_MSG("✅ Connection initiated successfully\n");
    } else {
        APP_DBG_MSG("❌ CONNECTION CREATION FAILED\n");
        APP_DBG_MSG("   Error code: 0x%02X\n", ret);
        APP_DBG_MSG("   Error analysis:\n");
        
        if (ret == 0x12) {
            APP_DBG_MSG("   → 0x12 = COMMAND_DISALLOWED\n");
            APP_DBG_MSG("   Possible causes:\n");
            APP_DBG_MSG("     1. Advertising still active (not fully stopped)\n");
            APP_DBG_MSG("     2. Scanning still active (not terminated)\n");
            APP_DBG_MSG("     3. Connection already in progress\n");
            APP_DBG_MSG("     4. Controller in invalid state for initiator role\n");
            APP_DBG_MSG("     5. PHY update in progress on phone connection\n");
            APP_DBG_MSG("   📌 CRITICAL CLUE: Check UART for recent HCI events\n");
            APP_DBG_MSG("   📌 PHY UPDATE events during connection attempt = timing issue\n");
        } else if (ret == 0xD2) {
            APP_DBG_MSG("   → 0xD2 = OUT_OF_MEMORY\n");
            APP_DBG_MSG("     - NumRadioTasks insufficient\n");
            APP_DBG_MSG("     - Connection parameters not configured\n");
        } else if (ret == 0xD1) {
            APP_DBG_MSG("   → 0xD1 = COMMAND_DISALLOWED (state)\n");
            APP_DBG_MSG("     - Stack not idle after scan/advertising\n");
        } else if (ret == 0x0C) {
            APP_DBG_MSG("   → 0x0C = INVALID_PARAMS\n");
            APP_DBG_MSG("     - Address type or PHY invalid\n");
        } else {
            APP_DBG_MSG("   → 0x%02X = UNKNOWN ERROR\n", ret);
        }
        
        APP_DBG_MSG("\n💡 RECOMMENDATION: If 0x12 persists:\n");
        APP_DBG_MSG("   1. Check if HCI_LE_PHY_UPDATE events occur during connection\n");
        APP_DBG_MSG("   2. Try adding hci_disconnect_flush before connection\n");
        APP_DBG_MSG("   3. Verify no background GAP procedures active\n");
        APP_DBG_MSG("   4. Consider longer delays if PHY negotiation interferes\n");
        
        /* Return to phone-connected state since phone is still connected */
        if (g_dual_role_ctx.phone_connected) {
            DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
        } else {
            DualRole_SetState(DUAL_STATE_ERROR);
        }
    }
}

void DualRole_OnPeerConnected(uint16_t conn_handle, uint8_t peer_addr_type, uint8_t *peer_addr)
{
    /* CRITICAL FIX: Do NOT reject peer connections when no phone is connected.
     * Old behavior: hci_disconnect(0x13) when phone not connected.
     * That broke the intended topology: Phone A → Module A → Module B ← Phone B.
     * Module B typically has NO phone connected when Module A initiates the peer
     * link, so Module B rejected every attempt → endless 0x13 connect/disconnect
     * loop on Module A (destabilizing the phone link too, and no sensor relay).
     * New behavior: always accept the peer link. If no phone is connected yet,
     * keep advertising so our own phone can connect later (see below). */
    if (!g_dual_role_ctx.phone_connected || g_dual_role_ctx.phone_conn_handle == 0xFFFF) {
        APP_DBG_MSG("📡 Peer connected with no phone attached - accepting (phone can join later)\n");
    }
    
    g_dual_role_ctx.peer_conn_handle = conn_handle;
    g_dual_role_ctx.peer_connected = 1;
    g_dual_role_ctx.peer_phy_updated = 0;
    g_dual_role_ctx.peer_mtu_exchanged = 0;
    g_dual_role_ctx.peer_dle_enabled = 0;
    g_dual_role_ctx.peer_is_central = 0;     /* overridden to 1 by app_ble.c when we initiated */
    g_dual_role_ctx.coded_phy_requested = 0;
    
    /* Store peer address info */
    if (peer_addr) {
        memcpy(g_dual_role_ctx.peer_bd_addr, peer_addr, 6);
        g_dual_role_ctx.peer_bd_addr_type = peer_addr_type;
    }
    
    /* CRITICAL FIX: If peer_name is empty, try to find it in device_list
     * This handles the case where peer is already connected when phone reconnects
     * and no fresh scan/selection cycle occurred */
    if (g_dual_role_ctx.peer_name[0] == '\0' && peer_addr) {
        /* Search device_list for this peer's MAC address */
        for (uint8_t i = 0; i < device_count && i < MAX_DEVICES; i++) {
            if (memcmp(device_list[i].addr, peer_addr, 6) == 0) {
                /* Found the peer in the device list - copy its name AND RSSI */
                strncpy(g_dual_role_ctx.peer_name, device_list[i].name, 
                       sizeof(g_dual_role_ctx.peer_name) - 1);
                g_dual_role_ctx.peer_name[sizeof(g_dual_role_ctx.peer_name) - 1] = '\0';
                g_dual_role_ctx.peer_rssi = device_list[i].rssi;  /* Store RSSI */
                APP_DBG_MSG("📝 Restored peer name from device list: %s (RSSI: %d dBm)\n", 
                           g_dual_role_ctx.peer_name, g_dual_role_ctx.peer_rssi);
                break;
            }
        }
        
        /* If still empty, use default name */
        if (g_dual_role_ctx.peer_name[0] == '\0') {
            strcpy(g_dual_role_ctx.peer_name, "RFOXiA BLE");
            APP_DBG_MSG("⚠️ Peer name unknown, using default: %s\n", 
                       g_dual_role_ctx.peer_name);
        }
    }
    
    /* Read actual RSSI from the connection */
    int8_t rssi_value = 127;  /* Invalid/unknown RSSI */
    tBleStatus rssi_ret = hci_read_rssi(conn_handle, &rssi_value);
    if (rssi_ret == BLE_STATUS_SUCCESS) {
        g_dual_role_ctx.peer_rssi = rssi_value;
        APP_DBG_MSG("📡 Peer RSSI: %d dBm\n", rssi_value);
    } else {
        g_dual_role_ctx.peer_rssi = -128;  /* Unknown/invalid RSSI */
        APP_DBG_MSG("⚠️ Failed to read RSSI: 0x%02X (will retry later)\n", rssi_ret);
    }
    
    DualRole_SetState(DUAL_STATE_PEER_CONNECTED);
    
    /* CRITICAL FIX #1: Advertising management depends on whether a phone is connected.
     * - Phone + peer connected (2 links): STOP advertising to prevent 0x0C
     *   INSUFFICIENT_RESOURCES (advertising with 2 active connections exhausts resources).
     * - Peer only (no phone): advertising auto-stopped when the peer connected as
     *   central. We must RESTART it (LP interval) so our own phone can still find
     *   and connect to us — required for the Phone B → Module B chat path. */
    if (g_dual_role_ctx.phone_connected) {
        APP_DBG_MSG("🛑 Stopping advertising - peer connection established (phone + peer)\n");
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_STOP);
    } else {
        APP_DBG_MSG("📡 Peer connected without phone - restarting LP advertising for phone connection\n");
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_LP);
    }
    
    /* CRITICAL FIX #3: Update sensor_server_app with peer connection handle
     * This allows sensor_server_app to send notifications to peer (environmental sensors only)
     */
    extern void SENSOR_SERVER_SetPeerHandle(uint16_t peer_handle);
    SENSOR_SERVER_SetPeerHandle(conn_handle);
    APP_DBG_MSG("✅ Updated sensor_server_app peer handle: 0x%04X\n", conn_handle);
    
    /* Reset retry counter on successful connection */
    g_dual_role_ctx.retry_count = 0;
    APP_DBG_MSG("🔄 Reset retry counter (connection successful)\n");
    
    APP_DBG_MSG("✅ Peer connected: 0x%04X — waiting for DLE event to trigger MTU exchange\n", conn_handle);
    /* MTU exchange is initiated from DualRole_OnPeerDleEnabled (after DLE auto-fires).
     * This ensures all BLE stack post-connection procedures finish before MTU exchange.
     * Calling aci_gatt_clt_exchange_config here (at connection time) races with DLE
     * negotiation and leaves the ATT client locked, causing 0x0C on CCCD writes. */
}

void DualRole_OnPeerPhyUpdated(uint8_t status, uint8_t tx_phy, uint8_t rx_phy)
{
    if (status != BLE_STATUS_SUCCESS) {
        APP_DBG_MSG("⚠️ Peer PHY update failed (0x%02X) — link stays on current PHY\n", status);
    } else if (tx_phy == 3 && rx_phy == 3) {
        APP_DBG_MSG("🚀 Peer link now on CODED PHY (S=8) — LONG-RANGE mode active\n");
    } else {
        APP_DBG_MSG("ℹ️ Peer PHY updated: TX=%d RX=%d (1=1M 2=2M 3=Coded)\n", tx_phy, rx_phy);
    }
    g_dual_role_ctx.peer_phy_updated = 1;
    /* MTU exchange is initiated from DualRole_OnPeerDleEnabled after DLE fires. */
}

void DualRole_OnPeerDleEnabled(uint16_t max_tx_octets, uint16_t max_rx_octets)
{
    g_dual_role_ctx.peer_dle_enabled = 1;
    /* Guard: DLE fires again after the Coded PHY S=8 switch (data length
     * re-negotiation). MTU exchange is only allowed ONCE per connection. */
    if (g_dual_role_ctx.peer_mtu_exchanged) {
        APP_DBG_MSG("🔄 DLE update (Tx=%d Rx=%d) — MTU already exchanged, no action\n",
                    max_tx_octets, max_rx_octets);
        return;
    }
    APP_DBG_MSG("🔄 DLE ready (Tx=%d Rx=%d) — starting MTU exchange\n",
                max_tx_octets, max_rx_octets);
    /* Trigger MTU exchange here, AFTER DLE auto-fires.
     * The DLE event is the natural synchronization point: by the time it fires
     * all BLE stack post-connection setup is complete and the ATT client is idle.
     * Initiating MTU exchange at connection time (OnPeerConnected) races with
     * internal stack DLE procedures and causes 0x0C errors on CCCD writes. */
    aci_gatt_clt_exchange_config(g_dual_role_ctx.peer_conn_handle);
}

void DualRole_OnPeerMtuExchanged(uint16_t mtu)
{
    /* Prevent duplicate processing */
    if (g_dual_role_ctx.peer_mtu_exchanged == 1) {
        return;  /* Already processed, skip */
    }
    
    g_dual_role_ctx.peer_mtu_exchanged = 1;
    APP_DBG_MSG("✅ Peer MTU: %d bytes\n", mtu);
    
    /* All optimizations complete - ready for dual-role operation */
    DualRole_SetState(DUAL_STATE_DUAL_ACTIVE);
    APP_DBG_MSG("🎉 Peer connection established\n");
    
    /* Advertising was already stopped in OnPeerConnected() to prevent 0x0C errors */
    
    /* Set flag for delayed sensor subscription (processed in main loop) */
    if (g_dual_role_ctx.subscription_pending == 0) {
        g_dual_role_ctx.subscription_pending = 1;
    }
}

void DualRole_OnPeerDisconnected(uint8_t reason)
{
    /* Capture initiator role BEFORE clearing: only the side that CREATED the
     * connection (central) should auto-retry. The acceptor (peripheral) must
     * NOT call aci_gap_create_connection — doing so races the initiator's
     * retry (fails 0xD1) and delays reconnection. The acceptor just restarts
     * advertising below so the initiator can reconnect to it. */
    uint8_t was_central = g_dual_role_ctx.peer_is_central;

    g_dual_role_ctx.peer_conn_handle = 0xFFFF;
    g_dual_role_ctx.peer_connected = 0;
    g_dual_role_ctx.peer_found = 0;
    g_dual_role_ctx.peer_phy_updated = 0;
    g_dual_role_ctx.peer_mtu_exchanged = 0;
    g_dual_role_ctx.peer_dle_enabled = 0;
    g_dual_role_ctx.peer_is_central = 0;
    g_dual_role_ctx.coded_phy_requested = 0;
    
    /* CRITICAL FIX #3: Clear peer handle in sensor_server_app to prevent 0x12 NOT_ALLOWED errors
     * Issue: Firmware continues trying to send notifications to stale peer handle after disconnect
     * Solution: Notify sensor_server_app to clear its peer handle (set to 0xFFFF)
     */
    extern void SENSOR_SERVER_SetPeerHandle(uint16_t peer_handle);
    SENSOR_SERVER_SetPeerHandle(0xFFFF);
    APP_DBG_MSG("🔄 Cleared peer handle in sensor_server_app\n");
    
    /* CRITICAL FIX: Do NOT auto-retry when the peer DELIBERATELY terminated the link.
     * 0x13 = Remote User Terminated Connection (peer explicitly disconnected us)
     * 0x16 = Connection Terminated by Local Host / unacceptable parameters
     * Retrying after a deliberate termination created an endless connect/disconnect
     * loop (retry counter was reset on each successful connect, so "1/3" repeated
     * forever), which also destabilized the phone connection. */
    if (reason == 0x13 || reason == 0x16) {
        APP_DBG_MSG("🚫 Peer terminated connection deliberately (reason 0x%02X) - NO auto-retry\n", reason);
        g_dual_role_ctx.retry_count = 0;
        g_dual_role_ctx.pending_peer_connection = 0;
    }
    /* AUTO-RETRY LOGIC: Automatically retry peer connection if enabled and under max retries
     * This fixes the issue where connection fails with 0x08 timeout and requires manual retry
     * Only retry if phone is still connected (can't connect peer without phone)
     * Only the INITIATOR (central) retries — the acceptor advertises and waits.
     */
    else if (was_central &&
        g_dual_role_ctx.auto_retry_enabled && 
        g_dual_role_ctx.phone_connected && 
        g_dual_role_ctx.retry_count < g_dual_role_ctx.max_retries) {
        
        g_dual_role_ctx.retry_count++;
        g_dual_role_ctx.last_retry_time = HAL_GetTick();
        g_dual_role_ctx.pending_peer_connection = 1;
        
        APP_DBG_MSG("🔄 AUTO-RETRY %d/%d: Will retry peer connection in 2 seconds\n", 
                    g_dual_role_ctx.retry_count, g_dual_role_ctx.max_retries);
        
        /* CRITICAL FIX: Set state to CONNECTING (not PHONE_CONNECTED) so the 10s timer
         * guard fires and prevents advertising from restarting during the 2s wait + connection.
         * Also clear waiting_for_user_connect so the timer exits early. */
        g_dual_role_ctx.waiting_for_user_connect = 0;   /* Disarm 10s timer */
        DualRole_SetState(DUAL_STATE_CONNECTING);
        
        /* Don't restart advertising - we're about to retry peer connection */
        return;
    }
    
    /* If retry count exceeded, reset for next manual attempt */
    if (g_dual_role_ctx.retry_count >= g_dual_role_ctx.max_retries) {
        APP_DBG_MSG("❌ Max retries (%d) exceeded - giving up. User must manually retry.\n", 
                    g_dual_role_ctx.max_retries);
        g_dual_role_ctx.retry_count = 0;  /* Reset for next manual attempt */
    }
    
    if (g_dual_role_ctx.phone_connected) {
        DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
        
        /* CRITICAL FIX #1: Restart advertising when peer disconnects (allow new peer or phone reconnection)
         * This ensures the device is discoverable again after losing peer connection
         */
        APP_DBG_MSG("🔄 Restarting advertising - peer disconnected, phone still connected\n");
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
    } else {
        DualRole_SetState(DUAL_STATE_IDLE);
        
        /* Restart advertising in IDLE state as well (standard behavior) */
        APP_DBG_MSG("🔄 Restarting advertising - both connections lost\n");
        APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
    }
    
    APP_DBG_MSG("Peer disconnected\n");
}

void DualRole_BuildDeviceListData(void)
{
    /* Build device list in blst[] format for B_LIST characteristic */
    /* Format: [num_devices][dev1_name_len][dev1_name...][MAC 6 bytes][RSSI 1 byte][dev2_name_len][dev2_name...][MAC 6 bytes][RSSI 1 byte]... */
    
    /* CRITICAL FIX: Set device count at blst[0] - this was missing! */
    blst_devices_num = device_count;
    blst[0] = device_count;
    blst_index = 1;  /* Start at index 1 (index 0 = device count) */
    
    APP_DBG_MSG("🔨 Building device list: %d devices found\n", device_count);
    
    for (uint8_t i = 0; i < device_count && i < MAX_DEVICES; i++) {
        uint8_t name_len = device_list[i].name_len;
        
        /* Add name length */
        if (blst_index < 254) {
            blst[blst_index++] = name_len;
        }
        
        /* Add device name */
        for (uint8_t j = 0; j < name_len && blst_index < 254; j++) {
            blst[blst_index++] = device_list[i].name[j];
        }
        
        /* Add MAC address (6 bytes) - REVERSE byte order for big-endian display */
        for (uint8_t j = 0; j < 6 && blst_index < 254; j++) {
            blst[blst_index++] = device_list[i].addr[5 - j];  // Reverse: send MSB first
        }
        
        /* Add RSSI (1 byte, offset by 128 to make it unsigned) */
        if (blst_index < 254) {
            blst[blst_index++] = (uint8_t)(device_list[i].rssi + 128);
        }
    }
    
    APP_DBG_MSG("📋 Device list built: %d devices, %d bytes\n", device_count, blst_index);
    
    /* Resume sensor notifications - scan is complete */
    extern volatile uint8_t g_pause_sensor_notifications;
    g_pause_sensor_notifications = 0;
    APP_DBG_MSG("▶️ Sensor notifications resumed (scan complete)\n");
    
    /* Reset scan flag - scan is now complete */
    extern uint8_t scan;
    scan = 0;
    APP_DBG_MSG("✅ Scan flag reset (scan complete)\n");
    
    /* Reset B_LIST notification flag - allow sending to new subscribers after scan completes */
    extern void BCONNECTION_SERVER_APP_ResetBListSentFlag(void);
    BCONNECTION_SERVER_APP_ResetBListSentFlag();
    APP_DBG_MSG("🔄 B_LIST sent flag reset (ready for new subscriptions)\n");
    
    /* CRITICAL: Preserve pending_peer_connection flag!
     * If user already selected device (pending_peer_connection=1), don't clear it.
     * Main loop needs this flag to initiate deferred connection.
     */
    uint8_t preserve_pending = g_dual_role_ctx.pending_peer_connection;
    APP_DBG_MSG("🔍 DEBUG: Preserving pending_peer_connection=%d\n", preserve_pending);
    
    /* Return to PHONE_CONNECTED state if phone is still connected
     * Start a 10-second timer - if user doesn't connect, restart advertising
     * GUARD: Only reset state if peer is NOT already connected.
     * If peer is connected (or connecting), this scan result is stale/spurious.
     */
    if (g_dual_role_ctx.phone_connected) {
        if (g_dual_role_ctx.peer_connected ||
            g_dual_role_ctx.current_state == DUAL_STATE_PEER_CONNECTED ||
            g_dual_role_ctx.current_state == DUAL_STATE_DUAL_ACTIVE ||
            g_dual_role_ctx.current_state == DUAL_STATE_CONNECTING) {
            /* Peer connection is in progress or established - don't reset state */
            APP_DBG_MSG("⚠️ DualRole_BuildDeviceListData: peer connection active (state=%d) - skipping state reset\n",
                       g_dual_role_ctx.current_state);
        } else {
        DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
        APP_DBG_MSG("📱 Returning to PHONE_CONNECTED state\n");
        
        /* CRITICAL: Restore pending_peer_connection after state change
         * DualRole_SetState might trigger other operations, so restore flag after
         */
        g_dual_role_ctx.pending_peer_connection = preserve_pending;
        APP_DBG_MSG("🔍 DEBUG: Restored pending_peer_connection=%d\n", g_dual_role_ctx.pending_peer_connection);
        
        /* Only set timeout if NOT already pending connection */
        if (!preserve_pending) {
            APP_DBG_MSG("⏲️ User has 10 seconds to connect, or advertising will restart\n");
            g_dual_role_ctx.scan_complete_time = HAL_GetTick();
            g_dual_role_ctx.waiting_for_user_connect = 1;
        } else {
            APP_DBG_MSG("⚡ Pending connection detected - skipping timeout, main loop will connect\n");
            g_dual_role_ctx.waiting_for_user_connect = 0;
        }
        }  /* end else (peer not connected) */
    } else {
        DualRole_SetState(DUAL_STATE_IDLE);
        APP_DBG_MSG("💤 Returning to IDLE state\n");
        
        /* REMOVED: Redundant advertising restart — DualRole_OnPhoneDisconnected() already
         * restarted advertising moments before this function was called (from ACI_GAP_PROC_COMPLETE).
         * A second DISABLE+ENABLE cycle here causes HCI queue flooding that disrupts
         * the upcoming phone reconnection (0x0802) connection events. */
    }
}

int DualRole_GetDeviceInfo(uint8_t index, uint8_t *addr_out, uint8_t *addr_type_out)
{
    /* Validate index (support both 0-based and 1-based for compatibility) */
    if (index >= device_count && (index - 1) >= device_count) {
        return -1;  /* Invalid index */
    }
    
    /* Determine if index is 0-based or 1-based */
    uint8_t actual_index = (index < device_count) ? index : (index - 1);
    
    /* Get device info */
    DeviceInfo_t *selected = &device_list[actual_index];
    
    /* Copy to output buffers */
    if (addr_out) {
        memcpy(addr_out, selected->addr, 6);
    }
    if (addr_type_out) {
        *addr_type_out = selected->addr_type;
    }
    
    return 0;  /* Success */
}

void DualRole_SelectDevice(uint8_t index)
{
    /* Validate index (support both 0-based and 1-based for compatibility) */
    if (index >= device_count && (index - 1) >= device_count) {
        APP_DBG_MSG("❌ Invalid device index: %d (have %d devices)\n", index, device_count);
        DualRole_SetState(DUAL_STATE_ERROR);
        return;
    }
    
    /* Determine if index is 0-based or 1-based */
    uint8_t actual_index = (index < device_count) ? index : (index - 1);
    APP_DBG_MSG("✅ Selecting device #%d (array index %d)\n", index, actual_index);
    
    /* Get selected device */
    DeviceInfo_t *selected = &device_list[actual_index];
    
    /* Store selected peer address, name, AND RSSI */
    memcpy(g_dual_role_ctx.peer_bd_addr, selected->addr, 6);
    g_dual_role_ctx.peer_bd_addr_type = selected->addr_type;
    
    /* CRITICAL: Store peer name AND RSSI for later retrieval */
    strncpy(g_dual_role_ctx.peer_name, selected->name, sizeof(g_dual_role_ctx.peer_name) - 1);
    g_dual_role_ctx.peer_name[sizeof(g_dual_role_ctx.peer_name) - 1] = '\0';
    g_dual_role_ctx.peer_rssi = selected->rssi;  /* Store scan-time RSSI */
    
    g_dual_role_ctx.peer_found = 1;
    
    APP_DBG_MSG("✅ Selected device %d: %s\n", index, selected->name);
    
    /* User is connecting - clear the timeout flag */
    g_dual_role_ctx.waiting_for_user_connect = 0;
    
    /* NOTE: Don't set CONNECTING state here - DualRole_ConnectToPeer() will do it
     * This prevents double state transitions
     */
    
    /* NOTE: Don't call DualRole_StopPeerScan() here - DualRole_ConnectToPeer() handles it
     * Double-calling causes 0x0C status (procedure already terminated) which confuses BLE stack
     */
    
    /* NOTE: Advertising was already stopped during scan and should stay OFF
     * until connection succeeds or fails. Don't restart it here!
     */

    /* CRITICAL FIX: Queue connection for the main loop instead of calling
     * DualRole_ConnectToPeer() directly here.
     * This function runs inside the BLE event handler (GATT write callback).
     * Calling aci_gap_create_connection() from inside an event handler is unsafe:
     * the BLE stack cannot process phone supervision events while blocked here,
     * causing the phone link to drop with 0x28 (supervision timeout).
     * The main loop runs DualRole_ConnectToPeer() OUTSIDE the event context,
     * with a non-blocking 100ms settle wait - sensors keep streaming uninterrupted. */
    DualRole_SetState(DUAL_STATE_CONNECTING);
    g_dual_role_ctx.retry_count = 0;          /* First attempt, not auto-retry */
    g_dual_role_ctx.pending_peer_connection = 1;
    APP_DBG_MSG("🔄 Connection queued for main loop (non-blocking, sensor flow preserved)\n");
}

int DualRole_GetConnectedPeerInfo(char *name_out, uint8_t *mac_out, int8_t *rssi_out)
{
    /* Check if peer is actually connected */
    if (!g_dual_role_ctx.peer_connected) {
        return -1;  /* Peer not connected */
    }
    
    /* Copy peer name */
    if (name_out && g_dual_role_ctx.peer_name[0] != '\0') {
        strcpy(name_out, g_dual_role_ctx.peer_name);
    } else if (name_out) {
        name_out[0] = '\0';  /* No name stored */
    }
    
    /* Copy peer MAC address - REVERSE byte order for big-endian display
     * BLE stack stores MAC in little-endian, but Android app expects big-endian */
    if (mac_out) {
        for (uint8_t i = 0; i < 6; i++) {
            mac_out[i] = g_dual_role_ctx.peer_bd_addr[5 - i];  // Reverse byte order
        }
    }
    
    /* Get peer RSSI */
    if (rssi_out) {
        /* Read live RSSI from the connection */
        int8_t live_rssi = 127;  /* Invalid/unknown RSSI */
        tBleStatus rssi_ret = hci_read_rssi(g_dual_role_ctx.peer_conn_handle, &live_rssi);
        
        if (rssi_ret == BLE_STATUS_SUCCESS) {
            g_dual_role_ctx.peer_rssi = live_rssi;  /* Update cached value */
            *rssi_out = live_rssi;
            APP_DBG_MSG("📡 Live RSSI read: %d dBm\n", live_rssi);
        } else {
            /* If read fails, use cached value */
            *rssi_out = g_dual_role_ctx.peer_rssi;
            APP_DBG_MSG("⚠️ RSSI read failed (0x%02X), using cached: %d dBm\n", 
                       rssi_ret, g_dual_role_ctx.peer_rssi);
        }
    }
    
    return 0;  /* Success */
}

void DualRole_RelayFromPhoneToPeer(uint8_t *data, uint16_t length)
{
    if (!g_dual_role_ctx.peer_connected) {
        APP_DBG_MSG("Cannot relay: Peer not connected\n");
        return;
    }
    
    if (g_dual_role_ctx.current_state != DUAL_STATE_DUAL_ACTIVE) {
        APP_DBG_MSG("Cannot relay: Dual-role not active (state=%d)\n", g_dual_role_ctx.current_state);
        return;
    }
    
    /* TODO: Re-enable when UART service compilation is fixed */
    APP_DBG_MSG("→ Would relay %d bytes: Phone → Peer (UART service disabled)\n", length);
    /* UART_RELAY_SERVICE_NotifyValue(data, length, g_dual_role_ctx.peer_conn_handle); */
}

void DualRole_RelayFromPeerToPhone(uint8_t *data, uint16_t length)
{
    if (!g_dual_role_ctx.phone_connected) {
        APP_DBG_MSG("Cannot relay: Phone not connected\n");
        return;
    }
    
    /* Send to phone via BConnection B_LIST characteristic notification */
    BCONNECTION_SERVER_Data_t relay_data;
    relay_data.p_Payload = data;
    relay_data.Length = length;
    
    /* TODO: Re-enable when ready */
    APP_DBG_MSG("← Would relay %d bytes: Peer → Phone (disabled)\n", length);
}

uint16_t DualRole_GetPhoneHandle(void)
{
    return g_dual_role_ctx.phone_conn_handle;
}

uint16_t DualRole_GetPeerHandle(void)
{
    return g_dual_role_ctx.peer_conn_handle;
}

uint8_t DualRole_IsPeerConnected(void)
{
    return g_dual_role_ctx.peer_connected;
}

DualRole_Context_t* DualRole_GetContext(void)
{
    return &g_dual_role_ctx;
}

void DualRole_CheckAdvertisingTimeout(void)
{
    /* Check if we're waiting for user to connect after scan */
    if (!g_dual_role_ctx.waiting_for_user_connect) {
        return;
    }
    
    /* If peer connection is in progress or established, never restart advertising */
    if (g_dual_role_ctx.current_state == DUAL_STATE_DUAL_ACTIVE ||
        g_dual_role_ctx.current_state == DUAL_STATE_PEER_CONNECTED ||
        g_dual_role_ctx.current_state == DUAL_STATE_CONNECTING) {
        g_dual_role_ctx.waiting_for_user_connect = 0;  /* Clear timer flag */
        return;  /* Peer connection in progress/active - no timeout needed */
    }
    
    /* Check if 10 seconds have passed */
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - g_dual_role_ctx.scan_complete_time;
    
    if (elapsed >= 10000) {  /* 10 seconds */
        APP_DBG_MSG("⏱️ User didn't connect within 10 seconds - restarting advertising\n");
        g_dual_role_ctx.waiting_for_user_connect = 0;
        
        /* Restart advertising for peer discovery */
        if (g_dual_role_ctx.phone_connected && 
            g_dual_role_ctx.current_state == DUAL_STATE_PHONE_CONNECTED) {
            APP_DBG_MSG("🔄 Restarting advertising for peer discovery\n");
            APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
        }
    }
}
