/**
  ******************************************************************************
  * @file    sensor_scheduler.c
  * @brief   Time-Division Sensor Scheduler Implementation
  * @date    December 22, 2025
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor_scheduler.h"
#include "debug_log.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

volatile TimeSlot_t currentTimeSlot = TIMESLOT_BMI270;
volatile SchedulerState_t schedulerState = SCHEDULER_STOPPED;

SchedulerStats_t schedulerStats = {0};  // Non-static: accessible from DMA callbacks
static uint8_t sensorEnabled[SENSOR_COUNT] = {0, 1, 1, 0, 1};  // BMI270=OFF(polling), MVH=ON, LPS=ON, ZMOD=OFF, GNSS=ON
static uint32_t slotStartTime = 0;
static uint32_t lastCycleTime = 0;
static uint32_t lastSlotCheckTime = 0;  // SysTick-based slot timing

/* Private function prototypes -----------------------------------------------*/
static void Scheduler_ExecuteSlot(TimeSlot_t slot);
static void Scheduler_CheckSlotCompletion(void);
static const char* Scheduler_GetSlotName(TimeSlot_t slot);
static const char* Scheduler_GetSensorName(SensorID_t sensor);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize sensor scheduler (SysTick-based)
  * @note   Uses HAL_GetTick() for timing, TIM1 remains free for PWM
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Init(void)
{
    DEBUG_LOG("[SCHEDULER] Initializing SysTick-based scheduler...");
    DEBUG_LOG("[SCHEDULER] TIM1 preserved for PWM motor control");
    
    /* Reset timing variables */
    lastSlotCheckTime = 0;
    slotStartTime = 0;
    lastCycleTime = 0;
    
    /* Reset statistics */
    Scheduler_ResetStats();
    
    DEBUG_LOG("[SCHEDULER] Init complete. Using HAL_GetTick() for 100ms intervals.");
    
    return HAL_OK;
}

/**
  * @brief  Start sensor scheduling (SysTick-based)
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Start(void)
{
    DEBUG_LOG("[SCHEDULER] Starting SysTick-based sensor scheduling...");
    
    uint32_t now = HAL_GetTick();
    currentTimeSlot = TIMESLOT_MVH4000D;  // Start with first enabled sensor (MVH4000D), skip BMI270
    schedulerState = SCHEDULER_RUNNING;
    slotStartTime = now;
    lastCycleTime = now;
    lastSlotCheckTime = now;
    
    /* Execute first slot immediately */
    Scheduler_ExecuteSlot(currentTimeSlot);
    
    DEBUG_LOG("[SCHEDULER] Scheduler started. First slot: %s (call Scheduler_Process() from main loop)", 
              Scheduler_GetSlotName(currentTimeSlot));
    
    return HAL_OK;
}

/**
  * @brief  Stop sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Stop(void)
{
    DEBUG_LOG("[SCHEDULER] Stopping scheduler...");
    
    schedulerState = SCHEDULER_STOPPED;
    
    DEBUG_LOG("[SCHEDULER] Scheduler stopped. Total cycles: %lu", schedulerStats.totalCycles);
    
    return HAL_OK;
}

/**
  * @brief  Pause sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Pause(void)
{
    DEBUG_LOG("[SCHEDULER] Pausing scheduler...");
    
    schedulerState = SCHEDULER_PAUSED;
    
    return HAL_OK;
}

/**
  * @brief  Resume sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Resume(void)
{
    DEBUG_LOG("[SCHEDULER] Resuming scheduler...");
    
    schedulerState = SCHEDULER_RUNNING;
    lastSlotCheckTime = HAL_GetTick();
    
    return HAL_OK;
}

/**
  * @brief  Check if slot should advance (SysTick polling)
  * @note   Called from main loop via Scheduler_Process()
  * @retval None
  */
static void Scheduler_CheckSlotAdvance(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - lastSlotCheckTime;
    
    /* Check if 100ms elapsed */
    if (elapsed < TIMESLOT_DURATION_MS) return;
    
    lastSlotCheckTime = now;
    uint32_t slotDuration = now - slotStartTime;
    
    DEBUG_LOG_FAST("[SCHED] Slot %s completed in %lu ms", 
                   Scheduler_GetSlotName(currentTimeSlot), slotDuration);
    
    /* Check if current slot DMA transfer completed */
    Scheduler_CheckSlotCompletion();
    
    /* Advance to next slot */
    currentTimeSlot = (TimeSlot_t)((currentTimeSlot + 1) % TIMESLOT_COUNT);
    slotStartTime = now;
    
    /* Track cycle completion */
    if (currentTimeSlot == TIMESLOT_BMI270) {
        schedulerStats.totalCycles++;
        uint32_t cycleTime = now - lastCycleTime;
        lastCycleTime = now;
        
        DEBUG_LOG("[SCHED] === Cycle %lu complete. Cycle time: %lu ms ===", 
                  schedulerStats.totalCycles, cycleTime);
    }
    
    /* Execute next slot */
    Scheduler_ExecuteSlot(currentTimeSlot);
}

/**
  * @brief  Process scheduler (MUST be called from main loop)
  * @note   Handles SysTick-based slot timing and DMA monitoring
  * @retval None
  */
void Scheduler_Process(void)
{
    if (schedulerState != SCHEDULER_RUNNING) return;
    
    /* Check if time to advance to next slot */
    Scheduler_CheckSlotAdvance();
    
    /* Check for DMA timeouts */
    DMA_CheckTimeouts();
    
    /* Check if slot is taking too long */
    uint32_t elapsed = HAL_GetTick() - slotStartTime;
    if (elapsed > (TIMESLOT_DURATION_MS + 20)) {  // 20ms grace period
        DEBUG_LOG("[SCHED] WARNING: Slot %s exceeded time limit (%lu ms)", 
                  Scheduler_GetSlotName(currentTimeSlot), elapsed);
    }
}

/**
  * @brief  Get scheduler statistics
  */
void Scheduler_GetStats(SchedulerStats_t *pStats)
{
    if (pStats) {
        memcpy(pStats, &schedulerStats, sizeof(SchedulerStats_t));
    }
}

/**
  * @brief  Reset scheduler statistics
  */
void Scheduler_ResetStats(void)
{
    memset(&schedulerStats, 0, sizeof(SchedulerStats_t));
    DEBUG_LOG("[SCHEDULER] Statistics reset.");
}

/**
  * @brief  Get time remaining in current slot
  */
uint32_t Scheduler_GetTimeRemainingInSlot(void)
{
    uint32_t elapsed = HAL_GetTick() - slotStartTime;
    if (elapsed >= TIMESLOT_DURATION_MS) return 0;
    return TIMESLOT_DURATION_MS - elapsed;
}

/**
  * @brief  Force advance to next time slot
  */
void Scheduler_ForceNextSlot(void)
{
    DEBUG_LOG("[SCHEDULER] Force advancing to next slot...");
    lastSlotCheckTime = HAL_GetTick() - TIMESLOT_DURATION_MS;  // Trigger immediate advance
    Scheduler_CheckSlotAdvance();
}

/**
  * @brief  Enable/disable specific sensor slot
  */
void Scheduler_EnableSensor(SensorID_t sensor, uint8_t enable)
{
    if (sensor < SENSOR_COUNT) {
        sensorEnabled[sensor] = enable;
        DEBUG_LOG("[SCHEDULER] Sensor %s %s", 
                  Scheduler_GetSensorName(sensor),
                  enable ? "ENABLED" : "DISABLED");
    }
}

/**
  * @brief  Check if sensor is enabled
  */
uint8_t Scheduler_IsSensorEnabled(SensorID_t sensor)
{
    if (sensor < SENSOR_COUNT) {
        return sensorEnabled[sensor];
    }
    return 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Execute time slot operations
  */
static void Scheduler_ExecuteSlot(TimeSlot_t slot)
{
    HAL_StatusTypeDef status;
    SensorID_t sensor;
    
    /* Map time slot to sensor */
    switch (slot) {
        case TIMESLOT_BMI270:
            sensor = SENSOR_BMI270;
            break;
        case TIMESLOT_MVH4000D:
            sensor = SENSOR_MVH4000D;
            break;
        case TIMESLOT_LPS22HH:
            sensor = SENSOR_LPS22HH;
            break;
        case TIMESLOT_ZMOD4510:
            sensor = SENSOR_ZMOD4510;
            break;
        case TIMESLOT_GNSS:
            sensor = SENSOR_GNSS;
            break;
        case TIMESLOT_BLE_FREE:
            DEBUG_LOG_FAST("[SCHED] Slot: BLE_FREE (500ms for BLE processing)");
            return;  // No sensor operation
        default:
            return;
    }
    
    /* Check if sensor is enabled */
    if (!sensorEnabled[sensor]) {
        printf("[SCHED] Slot %d DISABLED, skipping sensor %d\r\n", slot, sensor);
        DEBUG_LOG_FAST("[SCHED] Slot %s: Sensor disabled, skipping.", 
                       Scheduler_GetSlotName(slot));
        return;
    }
    
    /* Start DMA read */
    printf("[SCHED] Slot %d: Starting DMA for sensor %d...\r\n", slot, sensor);
    DEBUG_LOG_FAST("[SCHED] Slot %s: Starting DMA read for %s...", 
                   Scheduler_GetSlotName(slot),
                   Scheduler_GetSensorName(sensor));
    
    status = DMA_Sensor_StartRead(sensor);
    
    if (status == HAL_OK) {
        printf("[SCHED] DMA S%d started OK\r\n", sensor);
        DEBUG_LOG_FAST("[SCHED] DMA transfer started successfully.");
    } else if (status == HAL_BUSY) {
        printf("[SCHED] S%d BUSY!\r\n", sensor);
        DEBUG_LOG("[SCHED] WARNING: Previous transfer still busy!");
        schedulerStats.missedSlots[sensor]++;
    } else {
        printf("[SCHED] S%d START FAILED: %d\r\n", sensor, status);
        DEBUG_LOG("[SCHED] ERROR: Failed to start DMA transfer! Status: %d", status);
        schedulerStats.errorReads[sensor]++;
    }
}

/**
  * @brief  Check if current slot completed successfully
  */
static void Scheduler_CheckSlotCompletion(void)
{
    SensorID_t sensor;
    
    /* Get sensor for current slot */
    switch (currentTimeSlot) {
        case TIMESLOT_BMI270:   sensor = SENSOR_BMI270;   break;
        case TIMESLOT_MVH4000D: sensor = SENSOR_MVH4000D; break;
        case TIMESLOT_LPS22HH:  sensor = SENSOR_LPS22HH;  break;
        case TIMESLOT_ZMOD4510: sensor = SENSOR_ZMOD4510; break;
        case TIMESLOT_GNSS:     sensor = SENSOR_GNSS;     break;
        default: return;
    }
    
    /* Check DMA status */
    DMA_SensorContext_t *ctx = DMA_Sensor_GetContext(sensor);
    
    if (ctx->state == DMA_TRANSFER_COMPLETE) {
        schedulerStats.successfulReads[sensor]++;
        uint32_t readTime = HAL_GetTick() - slotStartTime;
        
        /* Update timing statistics */
        schedulerStats.averageReadTime[sensor] = 
            (schedulerStats.averageReadTime[sensor] + readTime) / 2;
        
        if (readTime > schedulerStats.maxReadTime[sensor]) {
            schedulerStats.maxReadTime[sensor] = readTime;
        }
        
        DEBUG_LOG_FAST("[SCHED] %s: Transfer complete in %lu ms. Success count: %lu",
                       Scheduler_GetSensorName(sensor),
                       readTime,
                       schedulerStats.successfulReads[sensor]);
    } 
    else if (ctx->state == DMA_TRANSFER_BUSY) {
        DEBUG_LOG("[SCHED] WARNING: %s still busy after slot time!",
                  Scheduler_GetSensorName(sensor));
        schedulerStats.missedSlots[sensor]++;
    }
    else if (ctx->state == DMA_TRANSFER_ERROR || ctx->state == DMA_TRANSFER_TIMEOUT) {
        DEBUG_LOG("[SCHED] ERROR: %s transfer failed! State: %d",
                  Scheduler_GetSensorName(sensor), ctx->state);
        schedulerStats.errorReads[sensor]++;
    }
}

/**
  * @brief  Get time slot name for logging
  */
static const char* Scheduler_GetSlotName(TimeSlot_t slot)
{
    switch (slot) {
        case TIMESLOT_BMI270:   return "BMI270";
        case TIMESLOT_MVH4000D: return "MVH4000D";
        case TIMESLOT_LPS22HH:  return "LPS22HH";
        case TIMESLOT_ZMOD4510: return "ZMOD4510";
        case TIMESLOT_GNSS:     return "GNSS";
        case TIMESLOT_BLE_FREE: return "BLE_FREE";
        default: return "UNKNOWN";
    }
}

/**
  * @brief  Get sensor name for logging
  */
static const char* Scheduler_GetSensorName(SensorID_t sensor)
{
    switch (sensor) {
        case SENSOR_BMI270:   return "BMI270";
        case SENSOR_MVH4000D: return "MVH4000D";
        case SENSOR_LPS22HH:  return "LPS22HH";
        case SENSOR_ZMOD4510: return "ZMOD4510";
        case SENSOR_GNSS:     return "GNSS";
        default: return "UNKNOWN";
    }
}

/**
  * @brief  Timer IRQ Handler
  */
void SCHEDULER_TIMER_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_scheduler);
}

/**
  * @brief  HAL Timer Period Elapsed Callback
  * @note   Moved to main.c to handle both TIM1 (IMU) and scheduler timer
  */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     Scheduler_TimerCallback(htim);
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
