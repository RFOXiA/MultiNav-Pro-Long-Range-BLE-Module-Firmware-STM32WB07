/**
  ******************************************************************************
  * @file    debug_log.c
  * @brief   Debug Logging System Implementation
  * @date    December 22, 2025
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "debug_log.h"
#include "sensor_scheduler.h"
#include "dma_config.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define USART_TIMEOUT_MS    10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

volatile LogLevel_t currentLogLevel = DEBUG_LOG_LEVEL;
static char logBuffer[LOG_BUFFER_SIZE];
static uint8_t moduleEnabled[LOG_MODULE_ALL] = {1, 1, 1, 1, 1};  // All enabled

extern UART_HandleTypeDef huart1;  // Assuming UART1 for debug output

/* Private function prototypes -----------------------------------------------*/
static const char* DebugLog_GetLevelString(LogLevel_t level);
static void DebugLog_Output(const char* message);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize debug log system
  */
void DebugLog_Init(void)
{
    /* Print startup banner */
    DebugLog_Output("\r\n");
    DebugLog_Output("=====================================\r\n");
    DebugLog_Output("  Autonomous DMA System Debug Log   \r\n");
    DebugLog_Output("  Date: December 22, 2025           \r\n");
    DebugLog_Output("=====================================\r\n");
    DebugLog_Output("\r\n");
    
    snprintf(logBuffer, LOG_BUFFER_SIZE, 
             "[INIT] Debug log initialized. Level: %s\r\n",
             DebugLog_GetLevelString(currentLogLevel));
    DebugLog_Output(logBuffer);
}

/**
  * @brief  Set debug log level
  */
void DebugLog_SetLevel(LogLevel_t level)
{
    currentLogLevel = level;
    snprintf(logBuffer, LOG_BUFFER_SIZE, 
             "[INIT] Log level changed to: %s\r\n",
             DebugLog_GetLevelString(level));
    DebugLog_Output(logBuffer);
}

/**
  * @brief  Print formatted debug log
  */
void DebugLog_Print(LogLevel_t level, const char* func, const char* fmt, ...)
{
    if (level > currentLogLevel) return;
    
    va_list args;
    int offset = 0;
    
    /* Add timestamp if enabled */
#if LOG_ENABLE_TIMESTAMP
    uint32_t timestamp = HAL_GetTick();
    offset = snprintf(logBuffer, LOG_BUFFER_SIZE, "[%6lu] ", timestamp);
#endif
    
    /* Add level indicator */
    offset += snprintf(logBuffer + offset, LOG_BUFFER_SIZE - offset, 
                      "%s ", DebugLog_GetLevelString(level));
    
    /* Add function name if provided */
#if LOG_ENABLE_MODULE
    if (func != NULL) {
        offset += snprintf(logBuffer + offset, LOG_BUFFER_SIZE - offset, 
                          "[%s] ", func);
    }
#endif
    
    /* Add user message */
    va_start(args, fmt);
    vsnprintf(logBuffer + offset, LOG_BUFFER_SIZE - offset, fmt, args);
    va_end(args);
    
    /* Add newline */
    strcat(logBuffer, "\r\n");
    
    DebugLog_Output(logBuffer);
}

/**
  * @brief  Print fast debug log (minimal overhead)
  */
void DebugLog_PrintFast(const char* fmt, ...)
{
    if (currentLogLevel < LOG_LEVEL_VERBOSE) return;
    
    va_list args;
    
    va_start(args, fmt);
    vsnprintf(logBuffer, LOG_BUFFER_SIZE, fmt, args);
    va_end(args);
    
    strcat(logBuffer, "\r\n");
    DebugLog_Output(logBuffer);
}

/**
  * @brief  Print hex dump
  */
void DebugLog_HexDump(const char* label, const uint8_t* data, uint16_t len)
{
    if (currentLogLevel < LOG_LEVEL_DEBUG) return;
    
    snprintf(logBuffer, LOG_BUFFER_SIZE, "[HEX] %s (%d bytes):\r\n", label, len);
    DebugLog_Output(logBuffer);
    
    for (uint16_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            if (i > 0) {
                DebugLog_Output("\r\n");
            }
            snprintf(logBuffer, LOG_BUFFER_SIZE, "%04X: ", i);
            DebugLog_Output(logBuffer);
        }
        snprintf(logBuffer, LOG_BUFFER_SIZE, "%02X ", data[i]);
        DebugLog_Output(logBuffer);
    }
    DebugLog_Output("\r\n\r\n");
}

/**
  * @brief  Log DMA statistics
  */
void DebugLog_DMAStats(void)
{
    DebugLog_Output("\r\n=== DMA Statistics ===\r\n");
    
    const char* sensorNames[] = {"BMI270", "MVH4000D", "LPS22HH", "ZMOD4510", "GNSS"};
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint32_t errorCount, timeoutCount;
        DMA_GetErrorStats((SensorID_t)i, &errorCount, &timeoutCount);
        
        snprintf(logBuffer, LOG_BUFFER_SIZE,
                "  %s: Errors=%lu, Timeouts=%lu\r\n",
                sensorNames[i], errorCount, timeoutCount);
        DebugLog_Output(logBuffer);
    }
    
    DebugLog_Output("====================\r\n\r\n");
}

/**
  * @brief  Log scheduler statistics
  */
void DebugLog_SchedulerStats(void)
{
    SchedulerStats_t stats;
    Scheduler_GetStats(&stats);
    
    DebugLog_Output("\r\n=== Scheduler Statistics ===\r\n");
    
    snprintf(logBuffer, LOG_BUFFER_SIZE,
            "  Total Cycles: %lu\r\n", stats.totalCycles);
    DebugLog_Output(logBuffer);
    
    const char* sensorNames[] = {"BMI270", "MVH4000D", "LPS22HH", "ZMOD4510", "GNSS"};
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        snprintf(logBuffer, LOG_BUFFER_SIZE,
                "  %s:\r\n", sensorNames[i]);
        DebugLog_Output(logBuffer);
        
        snprintf(logBuffer, LOG_BUFFER_SIZE,
                "    Success: %lu, Errors: %lu, Missed: %lu\r\n",
                stats.successfulReads[i],
                stats.errorReads[i],
                stats.missedSlots[i]);
        DebugLog_Output(logBuffer);
        
        snprintf(logBuffer, LOG_BUFFER_SIZE,
                "    Avg Time: %lu ms, Max Time: %lu ms\r\n",
                stats.averageReadTime[i],
                stats.maxReadTime[i]);
        DebugLog_Output(logBuffer);
    }
    
    DebugLog_Output("============================\r\n\r\n");
}

/**
  * @brief  Enable/disable specific module logging
  */
void DebugLog_EnableModule(LogModule_t module, uint8_t enable)
{
    if (module < LOG_MODULE_ALL) {
        moduleEnabled[module] = enable;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get log level string
  */
static const char* DebugLog_GetLevelString(LogLevel_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR:   return "[ERROR]";
        case LOG_LEVEL_WARNING: return "[WARN ]";
        case LOG_LEVEL_INFO:    return "[INFO ]";
        case LOG_LEVEL_DEBUG:   return "[DEBUG]";
        case LOG_LEVEL_VERBOSE: return "[VERB ]";
        default: return "[?????]";
    }
}

/**
  * @brief  Output log message via UART
  */
static void DebugLog_Output(const char* message)
{
    /* Send via UART */
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), USART_TIMEOUT_MS);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
