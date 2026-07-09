/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    uart_relay_service.h
  * @author  Dual-Role Integration
  * @brief   Header for UART relay service (Nordic UART compatible)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UART_RELAY_SERVICE_H
#define UART_RELAY_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ble_status.h"
#include <stdint.h>

/* Service UUID: Nordic UART Service */
#define UART_SERVICE_UUID_128   0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E

/* RX Characteristic (Write from peer) */
#define UART_RX_CHAR_UUID_128   0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E

/* TX Characteristic (Notify to peer) */
#define UART_TX_CHAR_UUID_128   0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E

typedef enum {
    UART_RELAY_RX_CHAR,
    UART_RELAY_TX_CHAR
} UART_RELAY_CharOpcode_t;

typedef enum {
    UART_RELAY_RX_WRITE_EVT,
    UART_RELAY_TX_NOTIFY_ENABLED_EVT,
    UART_RELAY_TX_NOTIFY_DISABLED_EVT
} UART_RELAY_OpcodeEvt_t;

typedef struct {
    uint8_t *p_Payload;
    uint8_t Length;
} UART_RELAY_Data_t;

typedef struct {
    UART_RELAY_OpcodeEvt_t EvtOpcode;
    UART_RELAY_Data_t DataTransfered;
    uint16_t ConnectionHandle;
    uint16_t AttributeHandle;
} UART_RELAY_NotificationEvt_t;

/* Service Initialization */
void UART_RELAY_SERVICE_Init(void);

/* Notification handler (called by BLE stack) */
void UART_RELAY_SERVICE_Notification(UART_RELAY_NotificationEvt_t *p_Notification);

/* Send notification to peer */
tBleStatus UART_RELAY_SERVICE_NotifyValue(uint8_t *pData, uint16_t Length, uint16_t ConnectionHandle);

/* Update characteristic value */
tBleStatus UART_RELAY_SERVICE_UpdateValue(UART_RELAY_CharOpcode_t CharOpcode, uint8_t *pData, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif /* UART_RELAY_SERVICE_H */
