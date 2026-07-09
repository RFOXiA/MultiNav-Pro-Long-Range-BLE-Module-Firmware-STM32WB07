/**
  ******************************************************************************
  * @file    sensor_scheduler.h
  * @brief   Time-Division Sensor Scheduler for Autonomous DMA Operations
  * @date    December 22, 2025
  ******************************************************************************
  */

#ifndef __SENSOR_SCHEDULER_H
#define __SENSOR_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include "dma_config.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Time Slot Definition
  */
typedef enum {
    TIMESLOT_BMI270 = 0,        // 0-100ms: Gyro + Accel
    TIMESLOT_MVH4000D,          // 100-200ms: Temp + Humidity
    TIMESLOT_LPS22HH,           // 200-300ms: Pressure
    TIMESLOT_ZMOD4510,          // 300-400ms: Air Quality
    TIMESLOT_GNSS,              // 400-500ms: GPS
    TIMESLOT_BLE_FREE,          // 500-1000ms: BLE Processing
    TIMESLOT_COUNT
} TimeSlot_t;

/**
  * @brief  Scheduler State
  */
typedef enum {
    SCHEDULER_STOPPED = 0,
    SCHEDULER_RUNNING,
    SCHEDULER_PAUSED,
    SCHEDULER_ERROR
} SchedulerState_t;

/**
  * @brief  Scheduler Statistics
  */
typedef struct {
    uint32_t totalCycles;           // Total number of 1-second cycles
    uint32_t missedSlots[SENSOR_COUNT];  // Missed time slots per sensor
    uint32_t successfulReads[SENSOR_COUNT];  // Successful reads per sensor
    uint32_t errorReads[SENSOR_COUNT];    // Failed reads per sensor
    uint32_t averageReadTime[SENSOR_COUNT];  // Average read time in ms
    uint32_t maxReadTime[SENSOR_COUNT];   // Maximum read time in ms
} SchedulerStats_t;

/* Exported constants --------------------------------------------------------*/

/* Time slot duration in milliseconds */
#define TIMESLOT_DURATION_MS        100
#define CYCLE_DURATION_MS           1000
#define BLE_FREE_DURATION_MS        500

/* SysTick-based timing (TIM1 preserved for PWM motor control) */
/* No hardware timer needed - uses HAL_GetTick() polling from main loop */

/* Exported macro ------------------------------------------------------------*/

/**
  * @brief  Check if scheduler is running
  */
#define SCHEDULER_IS_RUNNING()  (schedulerState == SCHEDULER_RUNNING)

/**
  * @brief  Get current time slot
  */
#define SCHEDULER_GET_CURRENT_SLOT()  currentTimeSlot

/* Exported variables --------------------------------------------------------*/

extern TIM_HandleTypeDef htim_scheduler;
extern volatile TimeSlot_t currentTimeSlot;
extern volatile SchedulerState_t schedulerState;
extern SchedulerStats_t schedulerStats;  // Accessible from DMA callbacks

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize sensor scheduler
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Init(void);

/**
  * @brief  Start sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Start(void);

/**
  * @brief  Stop sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Stop(void);

/**
  * @brief  Pause sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Pause(void);

/**
  * @brief  Resume sensor scheduling
  * @retval HAL status
  */
HAL_StatusTypeDef Scheduler_Resume(void);

/**
  * @brief  Timer period elapsed callback (called from TIM IRQ)
  * @param  htim: Timer handle
  * @retval None
  */
void Scheduler_TimerCallback(TIM_HandleTypeDef *htim);

/**
  * @brief  Process current time slot (call from main loop for monitoring)
  * @retval None
  */
void Scheduler_Process(void);

/**
  * @brief  Get scheduler statistics
  * @param  pStats: Pointer to statistics structure
  * @retval None
  */
void Scheduler_GetStats(SchedulerStats_t *pStats);

/**
  * @brief  Reset scheduler statistics
  * @retval None
  */
void Scheduler_ResetStats(void);

/**
  * @brief  Get time remaining in current slot (milliseconds)
  * @retval Time remaining in ms
  */
uint32_t Scheduler_GetTimeRemainingInSlot(void);

/**
  * @brief  Force advance to next time slot (for testing)
  * @retval None
  */
void Scheduler_ForceNextSlot(void);

/**
  * @brief  Enable/disable specific sensor slot
  * @param  sensor: Sensor ID
  * @param  enable: 1 to enable, 0 to disable
  * @retval None
  */
void Scheduler_EnableSensor(SensorID_t sensor, uint8_t enable);

/**
  * @brief  Check if sensor is enabled
  * @param  sensor: Sensor ID
  * @retval 1 if enabled, 0 if disabled
  */
uint8_t Scheduler_IsSensorEnabled(SensorID_t sensor);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SCHEDULER_H */
