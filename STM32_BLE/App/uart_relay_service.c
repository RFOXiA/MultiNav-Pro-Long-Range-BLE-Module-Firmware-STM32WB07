/* USER CODE BEGIN Header */
/**
  * @file    uart_relay_service.c
  * @brief   UART relay service stub (temporarily disabled)
  */
/* USER CODE END Header */

#include "main.h"
#include "app_common.h"
#include "ble.h"
#include "uart_relay_service.h"

static uint16_t UART_RELAY_ServiceHandle = 0;
static uint16_t UART_RELAY_RxCharHandle = 0;
static uint16_t UART_RELAY_TxCharHandle = 0;

void UART_RELAY_SERVICE_Init(void)
{
    APP_DBG_MSG("UART_RELAY_SERVICE_Init: Stub (disabled)\n");
}

tBleStatus UART_RELAY_SERVICE_NotifyValue(uint8_t *pData, uint16_t length, uint16_t conn_handle)
{
    APP_DBG_MSG("UART_RELAY_SERVICE_NotifyValue: Stub - %d bytes\n", length);
    return BLE_STATUS_SUCCESS;
}

void UART_RELAY_SERVICE_SendToDualRole(uint8_t *data, uint16_t length)
{
    APP_DBG_MSG("UART_RELAY_SERVICE_SendToDualRole: Stub - %d bytes\n", length);
}
