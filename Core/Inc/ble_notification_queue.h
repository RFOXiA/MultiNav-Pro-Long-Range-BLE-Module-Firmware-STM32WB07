/**
  ******************************************************************************
  * @file    ble_notification_queue.h
  * @brief   BLE Notification Queue Header
  * @date    December 22, 2025
  ******************************************************************************
  * @description
  * This module implements a lock-free queue for BLE notifications.
  * 
  * Key Features:
  * - Lock-free implementation safe for ISR context
  * - Multiple sensor data in single queue
  * - Prevents notification blocking
  * - Zero-copy when possible
  * 
  * Queue Architecture:
  * - Circular buffer with head/tail pointers
  * - Each entry contains sensor ID + formatted data
  * - Queue depth: 16 entries (supports 3+ full sensor cycles)
  * - Overflow protection with statistics
  * 
  * Usage:
  * 1. DMA complete callback calls BLE_Queue_Enqueue()
  * 2. Main loop calls BLE_Queue_Process() when idle
  * 3. Queue automatically calls aci_gatt_srv_notify()
  ******************************************************************************
  */

#ifndef __BLE_NOTIFICATION_QUEUE_H
#define __BLE_NOTIFICATION_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include "dma_config.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief BLE notification entry
  */
typedef struct {
    uint16_t charHandle;          /* BLE characteristic handle */
    uint8_t  data[32];            /* Formatted sensor data (max 32 bytes) */
    uint16_t dataLen;             /* Actual data length */
    SensorID_t sensorID;          /* Source sensor */
    uint32_t timestamp;           /* Enqueue timestamp (ms) */
} BLE_QueueEntry_t;

/**
  * @brief BLE queue status
  */
typedef struct {
    uint32_t totalEnqueued;       /* Total notifications enqueued */
    uint32_t totalDequeued;       /* Total notifications sent */
    uint32_t totalDropped;        /* Dropped due to overflow */
    uint32_t currentDepth;        /* Current queue depth */
    uint32_t maxDepth;            /* Maximum depth reached */
    uint32_t overflowCount;       /* Number of overflow events */
    uint32_t bleErrorCount;       /* BLE notification errors */
    uint32_t avgLatency;          /* Average queue latency (ms) */
    uint32_t maxLatency;          /* Maximum queue latency (ms) */
} BLE_QueueStatus_t;

/**
  * @brief Queue error codes
  */
typedef enum {
    BLE_QUEUE_OK = 0,
    BLE_QUEUE_FULL,
    BLE_QUEUE_EMPTY,
    BLE_QUEUE_ERROR,
    BLE_QUEUE_BLE_BUSY
} BLE_QueueResult_t;

/* Exported constants --------------------------------------------------------*/

#define BLE_QUEUE_DEPTH         16    /* Must be power of 2 for efficient modulo */
#define BLE_QUEUE_ALMOST_FULL   12    /* Warning threshold (75%) */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize BLE notification queue
  * @retval BLE_QueueResult_t
  */
BLE_QueueResult_t BLE_Queue_Init(void);

/**
  * @brief  Enqueue notification (called from ISR safe)
  * @param  sensorID: Source sensor
  * @param  charHandle: BLE characteristic handle
  * @param  data: Formatted sensor data
  * @param  len: Data length
  * @retval BLE_QueueResult_t
  */
BLE_QueueResult_t BLE_Queue_Enqueue(SensorID_t sensorID,
                                     uint16_t charHandle,
                                     const uint8_t* data,
                                     uint16_t len);

/**
  * @brief  Process queue (send one notification if available)
  * @note   Call this in main loop when MCU has free time
  * @retval BLE_QueueResult_t
  */
BLE_QueueResult_t BLE_Queue_Process(void);

/**
  * @brief  Process all pending notifications
  * @note   Blocks until queue is empty or BLE busy
  * @retval Number of notifications sent
  */
uint32_t BLE_Queue_ProcessAll(void);

/**
  * @brief  Get queue status
  * @param  status: Pointer to status structure
  */
void BLE_Queue_GetStatus(BLE_QueueStatus_t* status);

/**
  * @brief  Reset queue statistics
  */
void BLE_Queue_ResetStats(void);

/**
  * @brief  Check if queue is empty
  * @retval 1 if empty, 0 otherwise
  */
uint8_t BLE_Queue_IsEmpty(void);

/**
  * @brief  Check if queue is full
  * @retval 1 if full, 0 otherwise
  */
uint8_t BLE_Queue_IsFull(void);

/**
  * @brief  Get current queue depth
  * @retval Number of pending notifications
  */
uint32_t BLE_Queue_GetDepth(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_NOTIFICATION_QUEUE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
