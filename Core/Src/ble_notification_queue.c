/**
  ******************************************************************************
  * @file    ble_notification_queue.c
  * @brief   BLE Notification Queue Implementation
  * @date    December 22, 2025
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_notification_queue.h"
#include "debug_log.h"
#include "ble.h"
#include "ble_gatt.h"
#include "app_ble.h"
#include "sensor_server.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static BLE_QueueEntry_t queue[BLE_QUEUE_DEPTH];
static volatile uint32_t headIndex = 0;  /* Write index (modified by ISR) */
static volatile uint32_t tailIndex = 0;  /* Read index (modified by main) */
static BLE_QueueStatus_t queueStatus = {0};

/* Private function prototypes -----------------------------------------------*/
static inline uint32_t BLE_Queue_NextIndex(uint32_t index);
static void BLE_Queue_UpdateLatency(uint32_t latency);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize BLE notification queue
  */
BLE_QueueResult_t BLE_Queue_Init(void)
{
    /* Reset indices */
    headIndex = 0;
    tailIndex = 0;
    
    /* Clear statistics */
    memset(&queueStatus, 0, sizeof(BLE_QueueStatus_t));
    
    DEBUG_LOG_INFO("BLE_Queue_Init", "Queue initialized (depth=%d)", BLE_QUEUE_DEPTH);
    
    return BLE_QUEUE_OK;
}

/**
  * @brief  Enqueue notification (ISR safe)
  */
BLE_QueueResult_t BLE_Queue_Enqueue(SensorID_t sensorID,
                                     uint16_t charHandle,
                                     const uint8_t* data,
                                     uint16_t len)
{
    if (data == NULL || len == 0 || len > 32) {
        DEBUG_LOG_ERROR("BLE_Queue_Enqueue", "Invalid parameters");
        return BLE_QUEUE_ERROR;
    }
    
    /* Get next head index */
    uint32_t nextHead = BLE_Queue_NextIndex(headIndex);
    
    /* Check if queue is full */
    if (nextHead == tailIndex) {
        queueStatus.totalDropped++;
        queueStatus.overflowCount++;
        DEBUG_LOG_WARNING("BLE_Queue_Enqueue", "Queue full! Dropped sensor %d", sensorID);
        return BLE_QUEUE_FULL;
    }
    
    /* Copy data to queue */
    queue[headIndex].charHandle = charHandle;
    queue[headIndex].sensorID = sensorID;
    queue[headIndex].dataLen = len;
    queue[headIndex].timestamp = HAL_GetTick();
    memcpy(queue[headIndex].data, data, len);
    
    /* Advance head index (must be last to ensure data consistency) */
    headIndex = nextHead;
    
    /* Update statistics */
    queueStatus.totalEnqueued++;
    uint32_t depth = BLE_Queue_GetDepth();
    queueStatus.currentDepth = depth;
    if (depth > queueStatus.maxDepth) {
        queueStatus.maxDepth = depth;
    }
    
    /* Warning if queue almost full */
    if (depth >= BLE_QUEUE_ALMOST_FULL) {
        DEBUG_LOG_WARNING("BLE_Queue_Enqueue", "Queue almost full (%lu/%d)",
                         depth, BLE_QUEUE_DEPTH);
    }
    
    DEBUG_LOG_VERBOSE("BLE_Queue_Enqueue", "Enqueued sensor %d (len=%d, depth=%lu)",
                     sensorID, len, depth);
    
    return BLE_QUEUE_OK;
}

/**
  * @brief  Process one notification
  */
BLE_QueueResult_t BLE_Queue_Process(void)
{
    /* Check if queue is empty */
    if (headIndex == tailIndex) {
        return BLE_QUEUE_EMPTY;
    }
    
    /* Get entry from tail */
    BLE_QueueEntry_t* entry = &queue[tailIndex];
    
    /* Calculate latency */
    uint32_t latency = HAL_GetTick() - entry->timestamp;
    BLE_Queue_UpdateLatency(latency);
    
    /* Get connection handle from SENSOR_SERVER_APP_Context 
     * Using extern to access the internal context - this is a simplified approach
     * In production code, consider adding a getter function in sensor_server_app.c
     */
    extern uint16_t SENSOR_SERVER_Get_ConnectionHandle(void);
    
    /* Send BLE notification using correct 6-argument signature */
    tBleStatus ret = aci_gatt_srv_notify(
        SENSOR_SERVER_Get_ConnectionHandle(),     /* Get current connection handle */
        BLE_GATT_UNENHANCED_ATT_L2CAP_CID,       /* Standard L2CAP CID */
        entry->charHandle,                        /* Characteristic handle */
        GATT_NOTIFICATION,                        /* Notification flag */
        entry->dataLen,                           /* Data length */
        entry->data                               /* Data pointer */
    );
    
    if (ret != BLE_STATUS_SUCCESS) {
        queueStatus.bleErrorCount++;
        
        /* If BLE stack is busy, retry later */
        if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES) {
            DEBUG_LOG_VERBOSE("BLE_Queue_Process", "BLE busy, retry later");
            return BLE_QUEUE_BLE_BUSY;
        }
        
        DEBUG_LOG_ERROR("BLE_Queue_Process", "BLE notify failed: 0x%02X", ret);
        
        /* Advance tail anyway to avoid blocking queue */
        tailIndex = BLE_Queue_NextIndex(tailIndex);
        return BLE_QUEUE_ERROR;
    }
    
    /* Success - advance tail */
    tailIndex = BLE_Queue_NextIndex(tailIndex);
    queueStatus.totalDequeued++;
    queueStatus.currentDepth = BLE_Queue_GetDepth();
    
    DEBUG_LOG_VERBOSE("BLE_Queue_Process", "Sent sensor %d (latency=%lu ms)",
                     entry->sensorID, latency);
    
    return BLE_QUEUE_OK;
}

/**
  * @brief  Process all pending notifications
  */
uint32_t BLE_Queue_ProcessAll(void)
{
    uint32_t count = 0;
    BLE_QueueResult_t result;
    
    while (1) {
        result = BLE_Queue_Process();
        
        if (result == BLE_QUEUE_EMPTY) {
            /* Queue is empty, done */
            break;
        }
        else if (result == BLE_QUEUE_BLE_BUSY) {
            /* BLE stack busy, stop and retry later */
            break;
        }
        else if (result == BLE_QUEUE_OK) {
            count++;
        }
        else {
            /* Error, but continue to drain queue */
            count++;
        }
        
        /* Safety timeout to prevent infinite loop */
        if (count >= BLE_QUEUE_DEPTH) {
            DEBUG_LOG_WARNING("BLE_Queue_ProcessAll", "Safety limit reached");
            break;
        }
    }
    
    if (count > 0) {
        DEBUG_LOG_DEBUG("BLE_Queue_ProcessAll", "Processed %lu notifications", count);
    }
    
    return count;
}

/**
  * @brief  Get queue status
  */
void BLE_Queue_GetStatus(BLE_QueueStatus_t* status)
{
    if (status != NULL) {
        memcpy(status, (void*)&queueStatus, sizeof(BLE_QueueStatus_t));
        status->currentDepth = BLE_Queue_GetDepth();
    }
}

/**
  * @brief  Reset statistics
  */
void BLE_Queue_ResetStats(void)
{
    queueStatus.totalEnqueued = 0;
    queueStatus.totalDequeued = 0;
    queueStatus.totalDropped = 0;
    queueStatus.maxDepth = 0;
    queueStatus.overflowCount = 0;
    queueStatus.bleErrorCount = 0;
    queueStatus.avgLatency = 0;
    queueStatus.maxLatency = 0;
    
    DEBUG_LOG_INFO("BLE_Queue_ResetStats", "Statistics reset");
}

/**
  * @brief  Check if queue is empty
  */
uint8_t BLE_Queue_IsEmpty(void)
{
    return (headIndex == tailIndex) ? 1 : 0;
}

/**
  * @brief  Check if queue is full
  */
uint8_t BLE_Queue_IsFull(void)
{
    uint32_t nextHead = BLE_Queue_NextIndex(headIndex);
    return (nextHead == tailIndex) ? 1 : 0;
}

/**
  * @brief  Get current queue depth
  */
uint32_t BLE_Queue_GetDepth(void)
{
    uint32_t head = headIndex;
    uint32_t tail = tailIndex;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return BLE_QUEUE_DEPTH - tail + head;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get next queue index (circular)
  */
static inline uint32_t BLE_Queue_NextIndex(uint32_t index)
{
    return (index + 1) & (BLE_QUEUE_DEPTH - 1);  /* Efficient modulo for power of 2 */
}

/**
  * @brief  Update latency statistics
  */
static void BLE_Queue_UpdateLatency(uint32_t latency)
{
    /* Update max latency */
    if (latency > queueStatus.maxLatency) {
        queueStatus.maxLatency = latency;
    }
    
    /* Update average latency (exponential moving average) */
    if (queueStatus.avgLatency == 0) {
        queueStatus.avgLatency = latency;
    } else {
        queueStatus.avgLatency = (queueStatus.avgLatency * 7 + latency) / 8;  /* Alpha = 0.125 */
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
