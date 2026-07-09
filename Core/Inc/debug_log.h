/**
  ******************************************************************************
  * @file    debug_log.h
  * @brief   Debug Logging System for Autonomous DMA Debugging
  * @date    December 22, 2025
  ******************************************************************************
  */

#ifndef __DEBUG_LOG_H
#define __DEBUG_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include <stdio.h>
#include <stdarg.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Debug log levels
  */
typedef enum {
    LOG_LEVEL_NONE = 0,     // No logging
    LOG_LEVEL_ERROR,        // Errors only
    LOG_LEVEL_WARNING,      // Warnings and errors
    LOG_LEVEL_INFO,         // Info, warnings, and errors
    LOG_LEVEL_DEBUG,        // All messages including debug
    LOG_LEVEL_VERBOSE       // Everything including fast logs
} LogLevel_t;

/**
  * @brief  Debug log module
  */
typedef enum {
    LOG_MODULE_MAIN = 0,
    LOG_MODULE_SCHEDULER,
    LOG_MODULE_DMA,
    LOG_MODULE_BLE,
    LOG_MODULE_SENSOR,
    LOG_MODULE_ALL
} LogModule_t;

/* Exported constants --------------------------------------------------------*/

/* Default log level */
#ifndef DEBUG_LOG_LEVEL
#define DEBUG_LOG_LEVEL     LOG_LEVEL_ERROR
#endif

/* Log buffer size */
#define LOG_BUFFER_SIZE     256

/* Enable/disable timestamps */
#define LOG_ENABLE_TIMESTAMP    1

/* Enable/disable module names */
#define LOG_ENABLE_MODULE       1

/* Exported macro ------------------------------------------------------------*/

/**
  * @brief  Main debug log macro (with printf-style formatting)
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO)
    #define DEBUG_LOG(fmt, ...)  DebugLog_Print(LOG_LEVEL_INFO, __func__, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG(fmt, ...)  ((void)0)
#endif

/**
  * @brief  Fast debug log (minimal overhead, for high-frequency logs)
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_VERBOSE)
    #define DEBUG_LOG_FAST(fmt, ...)  DebugLog_PrintFast(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_FAST(fmt, ...)  ((void)0)
#endif

/**
  * @brief  Error log
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR)
    #define DEBUG_LOG_ERROR(fmt, ...)  DebugLog_Print(LOG_LEVEL_ERROR, __func__, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_ERROR(fmt, ...)  ((void)0)
#endif

/**
  * @brief  Warning log
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_WARNING)
    #define DEBUG_LOG_WARNING(fmt, ...)  DebugLog_Print(LOG_LEVEL_WARNING, __func__, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_WARNING(fmt, ...)  ((void)0)
#endif

/**
  * @brief  Debug log
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG)
    #define DEBUG_LOG_DEBUG(fmt, ...)  DebugLog_Print(LOG_LEVEL_DEBUG, __func__, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_DEBUG(fmt, ...)  ((void)0)
#endif

/**
  * @brief  Info log
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO)
    #define DEBUG_LOG_INFO(module, fmt, ...)  DebugLog_Print(LOG_LEVEL_INFO, module, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_INFO(module, fmt, ...)  ((void)0)
#endif

/**
  * @brief  Verbose log
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_VERBOSE)
    #define DEBUG_LOG_VERBOSE(module, fmt, ...)  DebugLog_Print(LOG_LEVEL_VERBOSE, module, fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG_VERBOSE(module, fmt, ...)  ((void)0)
#endif

/**
  * @brief  Hex dump utility
  */
#if (DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG)
    #define DEBUG_LOG_HEX(label, data, len)  DebugLog_HexDump(label, data, len)
#else
    #define DEBUG_LOG_HEX(label, data, len)  ((void)0)
#endif

/**
  * @brief  Performance timing macros
  */
#define DEBUG_TIMER_START()         uint32_t _debug_timer_start = HAL_GetTick()
#define DEBUG_TIMER_ELAPSED()       (HAL_GetTick() - _debug_timer_start)
#define DEBUG_TIMER_LOG(label)      DEBUG_LOG("%s: %lu ms", label, DEBUG_TIMER_ELAPSED())

/* Exported variables --------------------------------------------------------*/

extern volatile LogLevel_t currentLogLevel;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize debug log system
  * @retval None
  */
void DebugLog_Init(void);

/**
  * @brief  Set debug log level
  * @param  level: New log level
  * @retval None
  */
void DebugLog_SetLevel(LogLevel_t level);

/**
  * @brief  Print formatted debug log
  * @param  level: Log level
  * @param  func: Function name
  * @param  fmt: Format string
  * @param  ...: Variable arguments
  * @retval None
  */
void DebugLog_Print(LogLevel_t level, const char* func, const char* fmt, ...);

/**
  * @brief  Print fast debug log (minimal overhead)
  * @param  fmt: Format string
  * @param  ...: Variable arguments
  * @retval None
  */
void DebugLog_PrintFast(const char* fmt, ...);

/**
  * @brief  Print hex dump
  * @param  label: Label for the dump
  * @param  data: Pointer to data
  * @param  len: Length of data
  * @retval None
  */
void DebugLog_HexDump(const char* label, const uint8_t* data, uint16_t len);

/**
  * @brief  Log DMA statistics
  * @retval None
  */
void DebugLog_DMAStats(void);

/**
  * @brief  Log scheduler statistics
  * @retval None
  */
void DebugLog_SchedulerStats(void);

/**
  * @brief  Log BLE notification queue status
  * @retval None
  */
void DebugLog_BLEQueueStatus(void);

/**
  * @brief  Enable/disable specific module logging
  * @param  module: Module to enable/disable
  * @param  enable: 1 to enable, 0 to disable
  * @retval None
  */
void DebugLog_EnableModule(LogModule_t module, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_LOG_H */
