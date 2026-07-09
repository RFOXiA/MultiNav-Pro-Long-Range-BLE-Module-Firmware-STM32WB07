/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "pka.h"
#include "radio.h"
#include "radio_timer.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "stm32wb07.h"
#include "bmi270.h"  /* Motion sensor - Accelerometer + Gyroscope */
#include "common.h"
#include "mvh4000d.h"
#include "LPS22HH.h"
#include "bmi270rof.h"  // BMI270 wrapper for initialization
#include "stm32_seq.h"
#include "zmod4510.h"  // Air quality sensor with safe initialization
#include "gnss_mia_m10q.h"
#include "tmag5273.h"  // 3-axis magnetometer

/* Autonomous DMA System Includes */
#include "dma_config.h"
#include "sensor_scheduler.h"
#include "ble_notification_queue.h"
#include "debug_log.h"
#include "app_ble.h"
#include "imu_dma.h"  /* Motion sensors IMU system */

/* Dual Role Manager & BLE Includes */
#include "dual_role_manager.h"
#include "ble.h"
#include "gap_profile.h"
#include "sensor_server_app.h"  /* For sensor availability tracking */
#include "gatt_client_app.h"    /* For peer sensor subscription */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* TIM1 handle for 10Hz IMU timer interrupt */
TIM_HandleTypeDef htim1;

/* IMU Global Data (updated by DMA) */
extern volatile IMU_Data_t g_imu_data;
uint8_t G_accel[6] = {0};  /* Accelerometer: X, Y, Z (int16_t each) */
uint8_t G_gyro[6] = {0};   /* Gyroscope: X, Y, Z (int16_t each) */
uint8_t G_mag[6] = {0};    /* Magnetometer: X, Y, Z (int16_t each) */

uint8_t motor_cmd[5];
uint8_t spibuff[100]="hello world\r\n";

/* Air Quality Sensor (ZMOD4510) - Raw data only (11 bytes) */
uint8_t G_aqi[11] = {0};  // 8 ADC + 2 sample_count + 1 valid
bool zmod_ready = false;

/* Temperature & Humidity for ZMOD compensation */
uint16_t latest_temp = 2300;  // Default: 23.00°C (hundredths)
uint16_t latest_hum = 5000;   // Default: 50.00% (hundredths)

/* GNSS (MIA-M10Q) Variables */
GNSS_Data_t gnss_data = {0};
uint8_t G_gnss[20] = {0};  // GNSS data for BLE: lat(4) + lon(4) + alt(4) + speed(4) + heading(2) + fix(1) + sats(1)
bool gnss_ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void SendMotorCMD()
{
	printf(">>%c%c\r\n",motor_cmd[0],motor_cmd[1]);
	spibuff[0]=motor_cmd[0];
	spibuff[1]=motor_cmd[1];
	HAL_GPIO_WritePin(SPI2_NSS_to_Slave_CS_GPIO_Port, SPI2_NSS_to_Slave_CS_Pin, GPIO_PIN_RESET);
	    if(HAL_SPI_Transmit(&hspi2, spibuff, 2, 50)==HAL_OK)
	    {
	    	printf("^^^^^^send spi successfully^^^^^^^\r\n");
	    }

	    HAL_GPIO_WritePin(SPI2_NSS_to_Slave_CS_GPIO_Port, SPI2_NSS_to_Slave_CS_Pin, GPIO_PIN_SET);
}

void Scan_I2C_Bus()
{
  printf("\r\n=== Scanning I2C2 Bus ===\r\n");
  uint8_t devices_found = 0;
  
  for(uint8_t addr = 0x01; addr < 0x7F; addr++) {
    if(HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK) {
      printf("  Device found at 0x%02X (7-bit) / 0x%02X (8-bit)\r\n", addr, addr << 1);
      devices_found++;
    }
  }
  
  if(devices_found == 0) {
    printf("  ERROR: No I2C devices found!\r\n");
    printf("  Check: I2C clock, pull-ups, wiring\r\n");
  } else {
    printf("  Total devices: %d\r\n", devices_found);
  }
  printf("=========================\r\n\r\n");
}

void Init_Sensors_Task()
{
  static uint8_t sensors_initialized = 0;
  if(sensors_initialized) return; // Run only once
  
  printf("\r\nInitializing sensors...\r\n");
  
  // CRITICAL FIX: Don't scan first! It may put sensors in bad state.
  // Initialize sensors BEFORE scanning to avoid I2C bus conflicts.
  printf("NOTE: Initializing sensors WITHOUT pre-scan to avoid I2C conflicts\r\n");
  
  /* ===== SMART SENSOR RELAY: Check which sensors are available locally ===== */
  uint8_t mvh_ok = 0, lps_ok = 0, zmod_ok = 0, gnss_ok = 0;
  
  if(MVH4000D_Init(&hi2c2) == HAL_OK) {
    APP_DBG_MSG("MVH4000D init success\n");
    mvh_ok = 1;
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_TEMP, 1);  /* Temperature available */
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_HUM, 1);   /* Humidity available */
  }
  else {
    printf("MVH4000D init FAILED - will use peer for TEMP/HUM\r\n");
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_TEMP, 0);
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_HUM, 0);
  }
  
  // LPS22HH Pressure Sensor - Try both possible addresses
  printf("Attempting LPS22HH initialization...\r\n");
  HAL_Delay(200);  // Give sensor more time after MVH init
  
  // Clear I2C bus in case MVH left it in bad state
  __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_BUSY);
  
  // First, scan I2C bus to see what's actually there
  printf("Scanning I2C2 bus for LPS22HH...\r\n");
  uint8_t lps_found = 0;
  for(uint8_t addr = 0x5C; addr <= 0x5D; addr++) {
    if(HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 50) == HAL_OK) {
      printf("  LPS22HH candidate found at 0x%02X (7-bit) / 0x%02X (8-bit)\r\n", addr, addr << 1);
      lps_found = 1;
      break;
    }
  }
  
  if(!lps_found) {
    printf("  LPS22HH NOT detected at 0x5C or 0x5D - hardware may not be installed\r\n");
    printf("  Running full I2C2 scan to check what's connected...\r\n");
    Scan_I2C_Bus();
  }
  
  HAL_StatusTypeDef lps_status = LPS22HH_Init(&hi2c2);
  if(lps_status != HAL_OK && !lps_found) {
    printf("LPS22HH: First attempt failed and device not found in scan\r\n");
    printf("LPS22HH: Trying alternative initialization sequence...\r\n");
    HAL_Delay(500);  // Longer delay for second attempt
    lps_status = LPS22HH_Init(&hi2c2);
  }
  
  if(lps_status == HAL_OK) {
    APP_DBG_MSG("LPS22HH init success\n");
    lps_ok = 1;
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_PRS, 1);  /* Pressure available */
  }
  else {
    printf("LPS22HH init FAILED - will use peer for PRESSURE\r\n");
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_PRS, 0);
  }
  
  // BMI270 DISABLED - Not using accelerometer/gyroscope
  /*
  if(BMI270ROF_Init(&hi2c2)==HAL_OK)
    printf("BMI270 init success\r\n");
  else
    printf("BMI270 init FAILED\r\n");
  */
  
  // ZMOD4510 Air Quality Sensor - Re-enabled (hardware present at 0x33)
  // ZMOD may need warm-up time after I2C scanner, add delay and retry
  printf("Attempting ZMOD4510 initialization...\r\n");
  HAL_Delay(200);  // Give ZMOD sensor time to stabilize after scanner
  
  HAL_StatusTypeDef zmod_status = ZMOD_Init(&hi2c2);
  if(zmod_status != HAL_OK) {
    printf("ZMOD_Init: First attempt failed, retrying after delay...\r\n");
    HAL_Delay(500);  // Longer delay for warm-up
    zmod_status = ZMOD_Init(&hi2c2);
  }
  
  if(zmod_status == HAL_OK) {
    printf("ZMOD4510 air quality sensor init success\r\n");
    zmod_ready = true;
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_AQI, 1);
  }
  else {
    printf("ZMOD4510 init FAILED after retries (may need warm-up time)\r\n");
    zmod_ready = false;
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_AQI, 0);
  }
  
  /* GNSS - Check if available */
  if(GNSS_Init(&hi2c1) == HAL_OK) {
    printf("MIA-M10Q GNSS init success\r\n");
    gnss_ok = 1;
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_GNSS, 1);
  }
  else {
    printf("MIA-M10Q GNSS init FAILED - will use peer for GNSS\r\n");
    SENSOR_SERVER_SetSensorAvailability(SENSOR_TYPE_GNSS, 0);
  }
  
  /* BMI270 - Motion Sensor (Accelerometer + Gyroscope) */
  printf("Attempting BMI270 initialization...\r\n");
  HAL_Delay(100);
  uint8_t bmi270_ok = 0;
  if(BMI270ROF_Init(&hi2c2) == HAL_OK) {
    printf("BMI270 motion sensor init success\r\n");
    bmi270_ok = 1;
  }
  else {
    printf("BMI270 init FAILED - motion sensors unavailable\r\n");
  }
  
  /* TMAG5273 - Magnetometer */
  printf("Attempting TMAG5273 magnetometer initialization...\r\n");
  HAL_Delay(100);
  uint8_t tmag_ok = 0;
  if(TMAG5273_Init(&hi2c2) == HAL_OK) {
    printf("TMAG5273 magnetometer init success\r\n");
    tmag_ok = 1;
  }
  else {
    printf("TMAG5273 init FAILED - magnetometer unavailable\r\n");
  }
    
  sensors_initialized = 1;
  printf("  Motion sensors: ACCEL/GYRO=%d MAG=%d\r\n", bmi270_ok, tmag_ok);
  printf("Sensor initialization complete\r\n");
  printf("  Local sensors: TEMP=%d HUM=%d PRS=%d AQI=%d GNSS=%d\r\n",
         mvh_ok, mvh_ok, lps_ok, zmod_ready, gnss_ok);
  
  // NOTE: LPS22HH pressure sensor NOT detected on I2C2 bus (checked 0x5C and 0x5D)
  // Hardware may not be installed on this board
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Removed bare-metal test - will test AFTER clock config
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  // GPIO TEST - Toggle PB15 to verify pin is working
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;     // Enable GPIOB clock
  GPIOB->MODER &= ~(3 << (15*2));        // Clear mode bits
  GPIOB->MODER |= (1 << (15*2));         // Output mode
  
  // Blink PB15 10 times rapidly (should see on oscilloscope or LED)
  for(int i = 0; i < 10; i++) {
    GPIOB->ODR |= (1 << 15);   // Set high
    for(volatile int d = 0; d < 50000; d++);
    GPIOB->ODR &= ~(1 << 15);  // Set low
    for(volatile int d = 0; d < 50000; d++);
  }
  /* USER CODE END SysInit */

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();  // Initialize I2C1 for GNSS module
  MX_I2C2_Init();
  MX_RADIO_Init();
  MX_RADIO_TIMER_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_RNG_Init();
  MX_PKA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();  /* Initialize TIM1 for 10Hz IMU updates */
  /* USER CODE BEGIN 2 */
  // COMPREHENSIVE UART TEST
  // Test 1: Simple byte transmission
  for(int i = 0; i < 10; i++) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"U", 1, 100);
    HAL_Delay(100);
  }
  
  // Test 2: Full message
  char test_msg[] = "\r\n=== UART WORKING ===\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)test_msg, sizeof(test_msg)-1, 1000);
  
  // Test 3: Printf test
  printf("\r\n=== RFOXIA V1 STARTING ===\r\n");
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  
  // Initialize sensors BEFORE BLE (ensures I2C fully ready)
  printf("\r\n=== Initializing sensors (BEFORE BLE) ===\r\n");
  
  // Give I2C bus time to stabilize
  HAL_Delay(100);
  
  // Call proper sensor initialization function (includes I2C scanner + availability flags)
  Init_Sensors_Task();
  
  UTIL_SEQ_RegTask( 1U << TAKS_MOTOR_CMD, UTIL_SEQ_RFU, SendMotorCMD);
  printf("Starting BLE initialization...\r\n");
  
  /* ===== AUTONOMOUS DMA SYSTEM - DISABLED (Using simple polling instead) ===== */
  #if 0  // Disabled: DMA has issues with MVH4000D timing, reverting to polling
  printf("\r\n=== Initializing Autonomous DMA System ===\r\n");
  
  /* Initialize debug logging */
  DebugLog_Init();
  DebugLog_SetLevel(LOG_LEVEL_ERROR);  // Reduced logging for BLE stability
  
  /* Initialize DMA configuration */
  if (DMA_AutonomousSystem_Init() != HAL_OK) {
      DEBUG_LOG_ERROR("main", "DMA init failed!");
      Error_Handler();
  }
  DEBUG_LOG_INFO("main", "DMA configuration initialized");
  
  /* Initialize BLE notification queue */
  if (BLE_Queue_Init() != BLE_QUEUE_OK) {
      DEBUG_LOG_ERROR("main", "BLE queue init failed!");
      Error_Handler();
  }
  DEBUG_LOG_INFO("main", "BLE queue initialized (depth=16)");
  
  /* Initialize sensor scheduler */
  if (Scheduler_Init() != HAL_OK) {
      DEBUG_LOG_ERROR("main", "Scheduler init failed!");
      Error_Handler();
  }
  DEBUG_LOG_INFO("main", "Scheduler initialized (period=100ms)");
  
  DEBUG_LOG_INFO("main", "=== Autonomous DMA System Initialized ===");
  DEBUG_LOG_INFO("main", "CPU usage: <0.1%% for sensors, 99.9%% for BLE");
  printf("\r\n=== Rock Solid BLE Connection Mode Active ===\r\n\r\n");
  #endif
  /* ===== END AUTONOMOUS DMA INITIALIZATION ===== */
  
  /* USER CODE END 2 */

  /* Init code for STM32_BLE */
  MX_APPE_Init(NULL);
  printf("BLE initialization complete\r\n");
  
  /* ===== IMU DMA INITIALIZATION ===== */
  printf("\r\n=== IMU Initialization ===\r\n");
  
  /* Initialize IMU DMA system (BMI270 and TMAG5273 must be initialized first) */
  if (IMU_DMA_Init() == HAL_OK) {
      printf("[MAIN] IMU DMA initialization successful\r\n");
      printf("[MAIN] TIM1 ready - will start when Android sends view=1 (3D)\r\n");
      /* Note: TIM1 NOT started here - controlled by view changes from Android app */
  } else {
      printf("[MAIN] ERROR: IMU DMA initialization failed\r\n");
  }
  
  printf("=== END IMU INITIALIZATION ===\r\n\r\n");
  /* ===== END IMU INITIALIZATION ===== */
  
  /* ===== START AUTONOMOUS DMA SCHEDULER - DISABLED ===== */
  #if 0  // Disabled: Reverting to simple polling
  /* Start scheduler after BLE is initialized */
  if (Scheduler_Start() != HAL_OK) {
      DEBUG_LOG_ERROR("main", "Scheduler start failed!");
      Error_Handler();
  }
  DEBUG_LOG_INFO("main", "Scheduler started (SysTick-based, 100ms slots)");
  #endif
  /* ===== END SCHEDULER START ===== */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting main loop (autonomous mode)...\r\n");
  
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    
    /* ===== DEFERRED PEER CONNECTION PROCESSING ===== */
    /* CRITICAL: Process AFTER MX_APPE_Process() to ensure BLE stack fully settled
     * This implements the ST BLE_p2pClient pattern: connection called OUTSIDE
     * event handler context to avoid 0xD1 (COMMAND_DISALLOWED) errors.
     */
    {
        DualRole_Context_t *ctx = DualRole_GetContext();
        
        if (ctx && ctx->pending_peer_connection) {
            /* AUTO-RETRY DELAY: Wait 2 seconds between retry attempts
             * This prevents rapid failure loops and gives BLE stack time to recover
             * Only enforce delay if retry_count > 0 (i.e., this is a retry, not first attempt)
             */
            if (ctx->retry_count > 0) {
                uint32_t time_since_retry = HAL_GetTick() - ctx->last_retry_time;
                if (time_since_retry < 2000) {
                    /* Still waiting for retry delay - check again next loop iteration */
                    continue;  /* Skip connection attempt this loop */
                }
            }
            
            /* ── Phase 1 (first entry): disable advertising, start settle timer ──────
             * We do NOT block here.  After starting the timer we call continue so
             * MX_APPE_Process() runs and the phone keepalive/supervision events are
             * serviced normally.  This is the same non-blocking pattern used by the
             * subscription_delay_start block below. */
            static uint32_t peer_adv_disable_start = 0;

            if (peer_adv_disable_start == 0) {
                APP_DBG_MSG("════════════════════════════════════════════════\n");
                APP_DBG_MSG("🔌 MAIN LOOP: Initiating deferred peer connection\n");
                if (ctx->retry_count > 0) {
                    APP_DBG_MSG("   (AUTO-RETRY %d/%d)\n", ctx->retry_count, ctx->max_retries);
                }
                APP_DBG_MSG("════════════════════════════════════════════════\n");
                APP_DBG_MSG("   Peer: %02X:%02X:%02X:%02X:%02X:%02X (type %d)\n",
                           ctx->peer_bd_addr[5], ctx->peer_bd_addr[4], ctx->peer_bd_addr[3],
                           ctx->peer_bd_addr[2], ctx->peer_bd_addr[1], ctx->peer_bd_addr[0],
                           ctx->peer_bd_addr_type);
                uint16_t phone_handle = DualRole_GetPhoneHandle();
                APP_DBG_MSG("📱 Phone connection: %s (handle 0x%04X)\n",
                           (phone_handle != 0xFFFF) ? "ACTIVE" : "disconnected", phone_handle);
                APP_DBG_MSG("🛑 Disabling advertising - BLE stack settling 100ms (non-blocking)...\n");
                tBleStatus adv_result = aci_gap_set_advertising_enable(DISABLE, 0, NULL);
                APP_DBG_MSG("   aci_gap_set_advertising_enable(DISABLE) result: 0x%02X\n", adv_result);
                peer_adv_disable_start = HAL_GetTick();
                if (peer_adv_disable_start == 0) peer_adv_disable_start = 1; /* guard zero value */
                continue; /* ← MX_APPE_Process runs; phone supervision events serviced */
            }

            /* ── Phase 2: wait 100ms without blocking ───────────────────────────── */
            if (HAL_GetTick() - peer_adv_disable_start < 100) {
                continue; /* BLE stack keeps running */
            }

            /* ── Phase 3: BLE stack fully settled - proceed with connection ─────── */
            peer_adv_disable_start = 0; /* reset for next attempt */
            ctx->pending_peer_connection = 0; /* Clear flag */

            /* Configure connection parameters (peer role) */
            APP_DBG_MSG("📝 Configuring connection parameters...\n");
            
            /* Connection interval: 40*1.25ms = 50ms, 80*1.25ms = 100ms */
            /* Supervision timeout: 1000*10ms = 10s (reduced from 20s for consistency) */
            /* CE length: 8*0.625ms = 5ms */
            /* NOTE: Start with 1M PHY, will update to Coded S=8 after connection */
            tBleStatus result = aci_gap_set_connection_configuration(
                LE_1M_PHY_BIT,  /* Initiating_PHY: 1M PHY (PHY update will switch to Coded) */
                40,                                /* Connection_Interval_Min (50ms) */
                80,                                /* Connection_Interval_Max (100ms) */
                0,                                 /* Max_Latency */
                1000,                              /* Supervision_Timeout (10s) */
                8,                                 /* Min_CE_Length (5ms) */
                8                                  /* Max_CE_Length (5ms) */
            );
            APP_DBG_MSG("   aci_gap_set_connection_configuration result: 0x%02X %s\n",
                       result, (result == 0x00) ? "✅" : "❌");
            
            if (result == 0x00) {
                /* Create connection */
                APP_DBG_MSG("   Creating connection with 1M PHY (will upgrade to Coded S=8)...\n");
                result = aci_gap_create_connection(
                    LE_1M_PHY_BIT,  /* Initiating_PHY: 1M PHY (PHY update will switch to Coded) */
                    ctx->peer_bd_addr_type,           /* Peer address type */
                    ctx->peer_bd_addr                 /* Peer address */
                );
                APP_DBG_MSG("🔌 aci_gap_create_connection result: 0x%02X %s\n",
                           result, (result == 0x00) ? "✅ SUCCESS!" : "❌ FAILED");
                
                if (result == 0x00) {
                    APP_DBG_MSG("✨ Connection initiated successfully!\n");
                    APP_DBG_MSG("   Waiting for hci_le_connection_complete_event...\n");
                } else {
                    APP_DBG_MSG("💥 Connection failed with error 0x%02X\n", result);
                    if (result == 0x0C) {
                        /* 0x0C = COMMAND_DISALLOWED: a connection is already in-flight
                         * (e.g. from auto-retry). Keep state=CONNECTING so the in-flight
                         * attempt can complete and generate hci_le_connection_complete_event. */
                        APP_DBG_MSG("   ℹ️  0x0C = connection already in-flight, keeping CONNECTING state\n");
                        APP_DBG_MSG("   Waiting for hci_le_connection_complete_event...\n");
                    } else {
                        /* Genuine failure - recover so user can scan and retry manually */
                        ctx->peer_found = 0;
                        if (ctx->phone_connected) {
                            DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
                            APP_DBG_MSG("🔄 Recovering to PHONE_CONNECTED - user can scan and retry\n");
                            APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
                        } else {
                            DualRole_SetState(DUAL_STATE_ERROR);
                        }
                    }
                }
            } else {
                APP_DBG_MSG("💥 Configuration failed, aborting connection\n");
                ctx->peer_found = 0;
                if (ctx->phone_connected) {
                    DualRole_SetState(DUAL_STATE_PHONE_CONNECTED);
                    APP_DBG_MSG("🔄 Recovering to PHONE_CONNECTED - user can scan and retry\n");
                    APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
                } else {
                    DualRole_SetState(DUAL_STATE_ERROR);
                }
            }
            
            APP_DBG_MSG("════════════════════════════════════════════════\n\n");
        }
        
        /* ===== DEFERRED SENSOR SUBSCRIPTION PROCESSING ===== */
        /* Subscribe after BLE stack settled (100ms for faster connection) */
        static uint32_t subscription_delay_start = 0;
        
        if (ctx && ctx->subscription_pending) {
            if (subscription_delay_start == 0) {
                /* First time seeing the flag - start the delay timer */
                subscription_delay_start = HAL_GetTick();
                APP_DBG_MSG("⏳ Subscribing in 100ms...\n");
            } else if ((HAL_GetTick() - subscription_delay_start) >= 100) {
                /* 100ms has elapsed - now subscribe */
                ctx->subscription_pending = 0;
                subscription_delay_start = 0;
                
                APP_DBG_MSG("\n📡 Subscribing to peer sensors...\n");
                tBleStatus sensor_ret = GATT_CLIENT_SubscribeToPeerTemperature(ctx->peer_conn_handle);
                if (sensor_ret == BLE_STATUS_SUCCESS) {
                    APP_DBG_MSG("✅ Peer sensor subscriptions initiated\n");
                } else {
                    APP_DBG_MSG("⚠️ Peer sensor subscription failed: 0x%02X\n", sensor_ret);
                }
            }
        } else {
            /* No subscription pending - reset timer */
            subscription_delay_start = 0;
        }
        /* ===== END DEFERRED SENSOR SUBSCRIPTION PROCESSING ===== */

        /* ===== PEER MTU-CHAIN FALLBACK ===== */
        /* The MTU exchange is normally triggered by the DLE event
         * (DualRole_OnPeerDleEnabled). If the DLE event never fires, the chain
         * DLE -> MTU -> subscription stalls silently and NO peer sensor data
         * ever reaches the phone. Fallback: if the peer is connected but the
         * MTU exchange hasn't started within 2 seconds, trigger it manually. */
        {
            static uint16_t mtu_fallback_handle = 0xFFFF;
            static uint32_t mtu_fallback_start = 0;

            if (ctx && ctx->peer_connected && !ctx->peer_mtu_exchanged) {
                if (mtu_fallback_handle != ctx->peer_conn_handle) {
                    /* New peer connection - arm the fallback timer */
                    mtu_fallback_handle = ctx->peer_conn_handle;
                    mtu_fallback_start = HAL_GetTick();
                    if (mtu_fallback_start == 0) mtu_fallback_start = 1; /* guard zero */
                } else if (mtu_fallback_start != 0 && !ctx->peer_dle_enabled &&
                           (HAL_GetTick() - mtu_fallback_start) >= 500) {
                    mtu_fallback_start = 0; /* fire only once per connection */
                    APP_DBG_MSG("⏰ DLE event not received within 500ms - MTU exchange fallback\n");
                    tBleStatus mtu_ret = aci_gatt_clt_exchange_config(ctx->peer_conn_handle);
                    APP_DBG_MSG("   aci_gatt_clt_exchange_config result: 0x%02X\n", mtu_ret);
                }
            } else {
                mtu_fallback_handle = 0xFFFF;
                mtu_fallback_start = 0;
            }
        }
        /* ===== END PEER MTU-CHAIN FALLBACK ===== */

        /* ===== CODED PHY S=8 UPGRADE (LONG-RANGE) ===== */
        /* The module hardware (PA + LNA + high-gain antenna, -104 dBm sensitivity)
         * is designed for multi-km range which REQUIRES Coded PHY S=8.
         * Strategy: complete the fast GATT setup chain (DLE -> MTU -> CCCD
         * subscriptions) on 1M PHY first, then switch the peer link to Coded
         * S=8 for maximum link budget. Only the CENTRAL (initiator) requests
         * the switch to avoid LL procedure collisions; the peripheral side
         * accepts automatically (default PHY prefs include Coded). */
        {
            static uint16_t phy_up_handle = 0xFFFF;
            static uint32_t phy_up_start = 0;

            if (ctx && ctx->peer_connected && ctx->peer_mtu_exchanged &&
                ctx->peer_is_central && !ctx->coded_phy_requested) {
                if (phy_up_handle != ctx->peer_conn_handle) {
                    /* MTU chain done on this link - arm 500ms settle timer */
                    phy_up_handle = ctx->peer_conn_handle;
                    phy_up_start = HAL_GetTick();
                    if (phy_up_start == 0) phy_up_start = 1; /* guard zero */
                } else if (phy_up_start != 0 &&
                           (HAL_GetTick() - phy_up_start) >= 500) {
                    phy_up_start = 0;
                    ctx->coded_phy_requested = 1; /* request once per connection */
                    APP_DBG_MSG("\n📡 Upgrading peer link to CODED PHY S=8 (long-range)...\n");
                    tBleStatus phy_ret = hci_le_set_phy(ctx->peer_conn_handle,
                                                        0x00,   /* ALL_PHYS: use TX/RX prefs below */
                                                        0x04,   /* TX_PHYS: Coded only */
                                                        0x04,   /* RX_PHYS: Coded only */
                                                        0x0002  /* PHY options: prefer S=8 coding */);
                    APP_DBG_MSG("   hci_le_set_phy result: 0x%02X %s\n", phy_ret,
                                (phy_ret == BLE_STATUS_SUCCESS) ?
                                "(update requested)" : "(FAILED - staying on 1M)");
                }
            } else if (!(ctx && ctx->peer_connected)) {
                phy_up_handle = 0xFFFF;
                phy_up_start = 0;
            }
        }
        /* ===== END CODED PHY S=8 UPGRADE ===== */
    }
    /* ===== END DEFERRED PEER CONNECTION PROCESSING ===== */
    
/* ===== AUTONOMOUS DMA PROCESSING - DISABLED ===== */
  #if 0  // Disabled: Using simple polling instead
  /* Process scheduler (checks timing, advances slots) */
    Scheduler_Process();
    
    /* Process BLE notification queue */
    BLE_Queue_Process();
    
    /* If queue is getting full, drain it */
    if (BLE_Queue_GetDepth() >= BLE_QUEUE_ALMOST_FULL) {
        BLE_Queue_ProcessAll();
    }
    
    /* Optional: Periodic statistics (every 10 seconds) */
    static uint32_t last_stats_time = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_stats_time > 10000) {
        last_stats_time = now;
        DebugLog_SchedulerStats();
        
        /* Print queue status */
        BLE_QueueStatus_t queueStatus;
        BLE_Queue_GetStatus(&queueStatus);
        DEBUG_LOG_INFO("main", "Queue: depth=%lu, enqueued=%lu, sent=%lu, dropped=%lu",
                      queueStatus.currentDepth,
                      queueStatus.totalEnqueued,
                      queueStatus.totalDequeued,
                      queueStatus.totalDropped);
    }
    
    /* Optional: MCU can sleep here if desired (ultra low power mode) */
    // __WFI();  // Wait for interrupt
    #endif
    /* ===== END AUTONOMOUS DMA PROCESSING ===== */
    
    /* DMA MODE - All sensors use DMA for non-blocking reads */
    #if 1  
    static uint32_t last_sensor_read = 0;
    static uint32_t last_lps_read = 0;  // Separate timer for pressure (offset from main timer)
    static uint8_t mvh_dma_started = 0;  // Track MVH4000D DMA
    static uint8_t lps_dma_started = 0;  // Track LPS22HH DMA
    static uint8_t zmod_dma_started = 0; // Track ZMOD4510 DMA
    static uint8_t gnss_dma_started = 0; // Track GNSS DMA
    static uint32_t last_gnss_check = 0; // Throttle GNSS checks to prevent blocking
    uint32_t now = HAL_GetTick();
    
    // Every 1 second, read all sensors
    if((now - last_sensor_read) >= 1000 && now > 2000) {
      last_sensor_read = now;
      
      // ===== MVH4000D (Temperature & Humidity) - DMA =====
      if (!mvh_dma_started) {
        if(MVH4000D_StartDMARead() == HAL_OK) {
          mvh_dma_started = 1;
        }
      }
      
      // ===== GNSS - DMA (on separate I2C1 bus, can run in parallel) =====
      if (!gnss_dma_started) {
        if(GNSS_StartDMARead() == HAL_OK) {
          gnss_dma_started = 1;
        }
      }
    }
    
    // ===== LPS22HH (Pressure) - DMA =====
    // Read every 1 second, offset by 200ms from main timer to avoid I2C2 conflict with MVH4000D
    // This gives MVH4000D time to complete before pressure starts
    if (!lps_dma_started && (now - last_lps_read) >= 1000 && now > 2200) {
      if(LPS22HH_StartDMARead() == HAL_OK) {
        lps_dma_started = 1;
        last_lps_read = now;
      }
    }
    
    // Check if GNSS DMA is complete (throttled to every 100ms to avoid blocking)
    if (gnss_dma_started && (now - last_gnss_check) >= 100) {
      last_gnss_check = now;
      extern uint8_t G_gnss[20];
      GNSS_Data_t gnss_data = {0};
      HAL_StatusTypeDef gnss_status = GNSS_GetDMAResult(&gnss_data);
      
      if (gnss_status == HAL_OK) {
        // DMA read complete - pack data for BLE
        G_gnss[0] = gnss_data.latitude & 0xFF;
        G_gnss[1] = (gnss_data.latitude >> 8) & 0xFF;
        G_gnss[2] = (gnss_data.latitude >> 16) & 0xFF;
        G_gnss[3] = (gnss_data.latitude >> 24) & 0xFF;
        G_gnss[4] = gnss_data.longitude & 0xFF;
        G_gnss[5] = (gnss_data.longitude >> 8) & 0xFF;
        G_gnss[6] = (gnss_data.longitude >> 16) & 0xFF;
        G_gnss[7] = (gnss_data.longitude >> 24) & 0xFF;
        G_gnss[8] = gnss_data.altitude_msl & 0xFF;
        G_gnss[9] = (gnss_data.altitude_msl >> 8) & 0xFF;
        G_gnss[10] = (gnss_data.altitude_msl >> 16) & 0xFF;
        G_gnss[11] = (gnss_data.altitude_msl >> 24) & 0xFF;
        G_gnss[12] = gnss_data.ground_speed & 0xFF;
        G_gnss[13] = (gnss_data.ground_speed >> 8) & 0xFF;
        G_gnss[14] = (gnss_data.ground_speed >> 16) & 0xFF;
        G_gnss[15] = (gnss_data.ground_speed >> 24) & 0xFF;
        int16_t heading_deg = gnss_data.heading / 1000;
        G_gnss[16] = heading_deg & 0xFF;
        G_gnss[17] = (heading_deg >> 8) & 0xFF;
        G_gnss[18] = gnss_data.fix_type;
        G_gnss[19] = gnss_data.num_satellites;
        
        /* Send GNSS notification directly (TASK_READ_GNSS doesn't exist) */
        SEND_GNSS_Notification();
        
        gnss_dma_started = 0;  // Ready for next cycle
      }
      else if (gnss_status == HAL_ERROR) {
        gnss_dma_started = 0;  // Reset on error
      }
      // If HAL_BUSY, keep polling
    }
    
    // Check if MVH4000D DMA is complete (non-blocking check)
    if (mvh_dma_started) {
      uint16_t temp = 0, hum = 0;
      HAL_StatusTypeDef dma_status = MVH4000D_GetDMAResult(&temp, &hum);
      
      if (dma_status == HAL_OK) {
        // DMA read complete - update values
        // Silently ignore bad readings (0 or 0xFFFF)
        latest_temp = temp;
        latest_hum = hum;
        extern uint8_t G_temp[2], G_hum[2];
        G_temp[1] = temp >> 8; G_temp[0] = temp & 0xFF;
        G_hum[1] = (hum >> 8) & 0xFF; G_hum[0] = hum & 0xFF;
        UTIL_SEQ_SetTask(1U << TASK_READ_MVH, CFG_SEQ_PRIO_1);
        mvh_dma_started = 0;  // Ready for next cycle
      } 
      else if (dma_status == HAL_ERROR) {
        mvh_dma_started = 0;
      }
      else if (dma_status == HAL_BUSY && (now - last_sensor_read) > 500) {
        mvh_dma_started = 0;  // Reset stuck DMA
      }
    }
    
    // Check if LPS22HH DMA is complete (non-blocking check)
    if (lps_dma_started) {
      extern uint8_t G_prs[3];
      HAL_StatusTypeDef dma_status = LPS22HH_GetDMAResult(G_prs);
      
      if (dma_status == HAL_OK) {
        // DMA read complete - send notification
        UTIL_SEQ_SetTask(1U << TASK_READ_LPS22HH, CFG_SEQ_PRIO_1);
        lps_dma_started = 0;  // Ready for next cycle
      }
      else if (dma_status == HAL_ERROR) {
        lps_dma_started = 0;
      }
      else if (dma_status == HAL_BUSY && (now - last_lps_read) > 500) {
        lps_dma_started = 0;  // Reset stuck DMA
      }
    }
    
    // ===== ZMOD4510 (Air Quality) - DMA MODE =====
    // Start DMA read every 1 second when I2C2 bus is free (coordinate with MVH4000D and LPS22HH)
    // 1-second interval matches other sensors for fast, consistent data updates
    static uint32_t last_aqi_read = 0;
    if (!zmod_dma_started && zmod_ready && (now - last_aqi_read) >= 1000) {
      if (ZMOD_StartDMARead() == HAL_OK) {
        last_aqi_read = now;
        zmod_dma_started = 1;
      }
    }
    
    // Check ZMOD4510 DMA result
    if (zmod_dma_started) {
      HAL_StatusTypeDef zmod_status = ZMOD_GetDMAResult(G_aqi);
      if (zmod_status == HAL_OK) {
        // DMA read complete - send notification
        extern void SENSOR_SERVER_Aqi_char_SendNotification(void);
        SENSOR_SERVER_Aqi_char_SendNotification();
        zmod_dma_started = 0;  // Ready for next cycle
      }
      else if (zmod_status == HAL_ERROR) {
        zmod_dma_started = 0;
      }
      // HAL_BUSY = still reading, keep waiting
    }
    
    // BMI270 (Accelerometer & Gyroscope) - DISABLED
    // Not using these sensors in current application
    
    // TMAG5273 (Magnetometer) - One-time read completed during initialization
    // No continuous polling needed
    
    #endif
    
    // Check BLE dual role advertising timeout (restart if user didn't connect)
    extern void DualRole_CheckAdvertisingTimeout(void);
    DualRole_CheckAdvertisingTimeout();
    
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief I2C Master RX Complete Callback - for MVH4000D DMA
 * @note Called by HAL when I2C DMA receive completes (Master_Receive_DMA)
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // Forward to MVH4000D DMA handler
  MVH4000D_DMA_RxCallback(hi2c);
}

/**
 * @brief I2C Memory RX Complete Callback - for LPS22HH and GNSS DMA
 * @note Called by HAL when I2C DMA memory read completes (Mem_Read_DMA)
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // Forward to all handlers using Mem_Read_DMA (LPS22HH, ZMOD4510 on I2C2; GNSS on I2C1)
  LPS22HH_DMA_RxCallback(hi2c);
  ZMOD_DMA_RxCallback(hi2c);
  GNSS_DMA_RxCallback(hi2c);
}

/**
 * @brief I2C Error Callback - for all sensors using DMA
 * @note Called by HAL when I2C error occurs
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  // Forward to all error handlers
  MVH4000D_DMA_ErrorCallback(hi2c);
  LPS22HH_DMA_ErrorCallback(hi2c);
  ZMOD_DMA_ErrorCallback(hi2c);
  GNSS_DMA_ErrorCallback(hi2c);
}

/**
  * @brief TIM1 Initialization Function (10Hz for IMU updates)
  * @note  Timer configured for 100ms period (10Hz) to trigger IMU DMA reads
  */
static void MX_TIM1_Init(void)
{
  /* TIM1 clock is 32 MHz (from System Clock) */
  /* Prescaler: 32000-1 → Timer clock = 32MHz / 32000 = 1kHz (1ms tick) */
  /* Period: 100-1 → Interrupt every 100ms (10Hz) */
  
  __HAL_RCC_TIM1_CLK_ENABLE();
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000 - 1;  /* 32MHz / 32000 = 1kHz */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 - 1;  /* 100ms period (10Hz) */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  
  /* Enable TIM1 interrupt */
  HAL_NVIC_SetPriority(TIM1_IRQn, 5, 0);  /* Lower priority than BLE */
  HAL_NVIC_EnableIRQ(TIM1_IRQn);
}

/**
  * @brief TIM1 Period Elapsed Callback (10Hz IMU trigger)
  * @note  Called from TIM1_IRQHandler when timer reaches period
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Handle TIM1 for IMU updates @ 10Hz */
  if (htim->Instance == TIM1) {
    /* Trigger IMU processing (DMA reads of BMI270 and TMAG5273) */
    IMU_Process();
    
    /* Copy IMU data to global variables for BLE notifications */
    IMU_Data_t imu_data;
    if (IMU_GetData(&imu_data) == HAL_OK) {
      memcpy(G_accel, &imu_data.accel, 6);
      memcpy(G_gyro, &imu_data.gyro, 6);
      memcpy(G_mag, &imu_data.mag, 6);
      
      /* Send motion sensor notifications (with view-based filtering) */
      extern void SEND_IMU_Notifications(void);
      SEND_IMU_Notifications();
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("\r\n!!! ERROR HANDLER TRIGGERED !!!\r\n");
  __disable_irq();
  while (1)
  {
    // System halted - requires hardware reset
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
