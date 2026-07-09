/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dual_role_manager.h
  * @author  Dual-Role Integration
  * @brief   Header for dual-role connection manager
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DUAL_ROLE_MANAGER_H
#define DUAL_ROLE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ble_status.h"
#include <stdint.h>

/* BLE timing conversion macros (from ST examples) */
#define CONN_INT_MS(x) ((uint16_t)((x)/1.25f))
#define CONN_SUP_TIMEOUT_MS(x) ((uint16_t)((x)/10.0f))
#define CONN_CE_LENGTH_MS(x) ((uint16_t)((x)/0.625f))

/* Device info structure for scan results */
#define MAX_DEVICES 10
typedef struct {
    uint8_t addr[6];
    uint8_t addr_type;
    char name[32];
    uint8_t name_len;
    int8_t rssi;
} DeviceInfo_t;

/* External device list variables (defined in dual_role_manager.c) */
extern DeviceInfo_t device_list[MAX_DEVICES];
extern uint8_t device_count;

/* Dual-role states (matches Android app expectations) */
typedef enum {
    DUAL_STATE_IDLE = 0x00,
    DUAL_STATE_PHONE_CONNECTED = 0x01,
    DUAL_STATE_SCANNING = 0x02,
    DUAL_STATE_PEER_FOUND = 0x03,
    DUAL_STATE_CONNECTING = 0x04,
    DUAL_STATE_PEER_CONNECTED = 0x05,
    DUAL_STATE_DUAL_ACTIVE = 0x06,
    DUAL_STATE_ERROR = 0xFF
} DualRole_State_t;

/* Connection context */
typedef struct {
    uint16_t phone_conn_handle;
    uint16_t peer_conn_handle;
    
    uint8_t phone_connected;
    uint8_t peer_connected;
    uint8_t peer_phy_updated;
    uint8_t peer_mtu_exchanged;
    uint8_t peer_dle_enabled;
    uint8_t peer_is_central;       /* 1 = we initiated the peer link (central role) */
    uint8_t coded_phy_requested;   /* 1 = Coded S=8 upgrade already requested on this link */
    
    uint8_t peer_found;
    uint8_t peer_bd_addr[6];
    uint8_t peer_bd_addr_type;
    char peer_name[32];
    int8_t peer_rssi;  /* Peer RSSI from scan or last known value */
    
    DualRole_State_t current_state;
    
    /* Timeout management for advertising restart */
    uint32_t scan_complete_time;
    uint8_t waiting_for_user_connect;

    /* Deferred L2CAP supervision timeout request (BT spec: must wait ≥1s after connection) */
    uint32_t phone_connect_time;    /* Tick when phone connected */
    uint8_t l2cap_update_pending;   /* 1 = send L2CAP update once 1.5s has elapsed */
    
    /* Pending peer connection (for event-driven isolated test) */
    uint8_t pending_peer_connection;
    uint8_t pending_peer_device_index;
    
    /* Sensor subscription flag (delayed subscription from main loop) */
    uint8_t subscription_pending;
    
    /* Auto-retry for peer connection failures */
    uint8_t auto_retry_enabled;
    uint8_t retry_count;
    uint32_t last_retry_time;
    uint8_t max_retries;
} DualRole_Context_t;

/* Initialization */
void DualRole_Init(void);

/* State machine */
DualRole_State_t DualRole_GetState(void);
void DualRole_SetState(DualRole_State_t new_state);

/* Connection management */
void DualRole_OnPhoneConnected(uint16_t conn_handle);
void DualRole_OnPhoneDisconnected(void);
void DualRole_StartPeerScan(const char *peer_name);
void DualRole_StopPeerScan(void);
void DualRole_ConnectToPeer(void);
void DualRole_OnPeerConnected(uint16_t conn_handle, uint8_t peer_addr_type, uint8_t *peer_addr);
void DualRole_OnPeerDisconnected(uint8_t reason);

/* PHY/MTU/DLE handlers */
void DualRole_OnPeerPhyUpdated(uint8_t status, uint8_t tx_phy, uint8_t rx_phy);
void DualRole_OnPeerMtuExchanged(uint16_t mtu);
void DualRole_OnPeerDleEnabled(uint16_t max_tx_octets, uint16_t max_rx_octets);

/* Message relay */
void DualRole_RelayFromPhoneToPeer(uint8_t *data, uint16_t length);
void DualRole_RelayFromPeerToPhone(uint8_t *data, uint16_t length);

/* Advertising report handler (for peer discovery) */
void DualRole_OnAdvertisingReport(uint8_t addr_type, uint8_t *addr, uint8_t data_len, uint8_t *data_rssi, uint8_t event_type);

/* Device list management */
void DualRole_BuildDeviceListData(void);
void DualRole_SelectDevice(uint8_t index);
void DualRole_ClearDeviceList(void);
int DualRole_GetDeviceInfo(uint8_t index, uint8_t *addr_out, uint8_t *addr_type_out);
int DualRole_GetConnectedPeerInfo(char *name_out, uint8_t *mac_out, int8_t *rssi_out);

/* Getters */
uint16_t DualRole_GetPhoneHandle(void);
uint16_t DualRole_GetPeerHandle(void);
uint8_t DualRole_IsPeerConnected(void);
DualRole_Context_t* DualRole_GetContext(void);

/* Timeout management */
void DualRole_CheckAdvertisingTimeout(void);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_ROLE_MANAGER_H */
