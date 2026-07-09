# RFOXiA MultiNav Pro+ BLE Module — Sensors & Peripherals Reference

**Build:** 38 — Production Ready (July 2026)  
**Sensors:** 6 (Module A full), 2 (Module B subset)  
**DMA mode:** Polling-triggered I2C DMA (autonomous disabled)

---

## Table of Contents

1. [Sensor Architecture Overview](#1-sensor-architecture-overview)
2. [MVH4000D — Humidity Sensor](#2-mvh4000d--humidity-sensor)
3. [LPS22HH — Pressure & Temperature](#3-lps22hh--pressure--temperature)
4. [ZMOD4510 — Air Quality (AQI)](#4-zmod4510--air-quality-aqi)
5. [MIA-M10Q — GNSS](#5-mia-m10q--gnss)
6. [BMI270 — Accelerometer & Gyroscope](#6-bmi270--accelerometer--gyroscope)
7. [TMAG5273 — Magnetometer](#7-tmag5273--magnetometer)
8. [DMA Architecture](#8-dma-architecture)
9. [Sensor Scheduler (TIM2 TDMA)](#9-sensor-scheduler-tim2-tdma)
10. [IMU On-Demand System (TIM1)](#10-imu-on-demand-system-tim1)
11. [Long-Range RF Configuration](#11-long-range-rf-configuration)
12. [SPI Motor Control](#12-spi-motor-control)
13. [Function Reference](#13-function-reference)

---

## 1. Sensor Architecture Overview

### Bus Assignment

```
I2C2 (PA13=SCL, PA14=SDA, 100 kHz Standard Mode)
  ├── MVH4000D  @ 0x54  — Humidity
  ├── LPS22HH   @ 0x5D  — Pressure + Temperature
  ├── ZMOD4510  @ 0x33  — Air Quality / AQI
  ├── BMI270    @ 0x68  — Accel + Gyro (10 Hz on-demand)
  └── TMAG5273  @ 0x78  — Magnetometer (10 Hz on-demand)

I2C1 (PA0=SCL, PA1=SDA, 400 kHz Fast Mode)
  └── MIA-M10Q  @ 0x42  — GNSS (1 Hz polling DMA)

SPI2 (PA4=CS, PA5=SCK, PA6=MOSI, PA7=MISO, 1 MHz)
  └── Motor controller (2-byte command slave)

TIM1 (PB11/10/2/3 = CH1–4, 1 kHz PWM)
  └── Motor PWM drive outputs
```

### Rate Summary

| Sensor    | Update Rate | Trigger          | Method          |
|-----------|-------------|------------------|-----------------|
| MVH4000D  | 1 Hz        | TIM2 slot 0      | I2C DMA         |
| LPS22HH   | 1 Hz        | TIM2 slot 2      | I2C DMA +200ms  |
| ZMOD4510  | 1 Hz        | TIM2 slot 4      | I2C DMA         |
| MIA-M10Q  | 1 Hz        | TIM2 slot 6      | I2C DMA (UART)  |
| BMI270    | 10 Hz       | TIM1 ISR         | On-demand I2C   |
| TMAG5273  | 10 Hz       | TIM1 ISR         | On-demand I2C   |

### Module B Sensor Subset

Module B hardware only populates **ZMOD4510** and **MIA-M10Q**. Firmware running on Module B skips MVH4000D, LPS22HH, BMI270, and TMAG5273 initialization and DMA slots.

---

## 2. MVH4000D — Humidity Sensor

**I2C Address:** 0x54  
**Bus:** I2C2  
**Update Rate:** 1 Hz  

### Register Interface

| Register | Address | Description |
|----------|---------|-------------|
| MEAS_CFG | 0x44    | Measurement configuration |
| HUM_OUT_L| 0x28    | Humidity LSB |
| HUM_OUT_H| 0x29    | Humidity MSB |

### BLE Data Format (2 bytes)

```
Offset 0: uint16 little-endian raw humidity
Scaling: raw / 1000.0 → % RH
```

### Initialization

```c
// One-shot measurement mode
I2C_WriteReg(0x54, 0x44, 0x01);  // Enable measurement
```

### DMA Read Sequence

```c
// DMA reads 2 bytes from 0x28
I2C_ReadDMA(MVH_ADDR, HUM_OUT_L, mvh_buf, 2);
// Callback: pack mvh_buf into BLE characteristic
```

---

## 3. LPS22HH — Pressure & Temperature

**I2C Address:** 0x5D  
**Bus:** I2C2  
**Update Rate:** 1 Hz (+200 ms offset from MVH slot)  

### Register Interface

| Register    | Address | Description |
|-------------|---------|-------------|
| CTRL_REG1   | 0x10    | Output data rate, LPF |
| PRESS_OUT_XL| 0x28    | Pressure byte 0 (LSB) |
| PRESS_OUT_L | 0x29    | Pressure byte 1 |
| PRESS_OUT_H | 0x2A    | Pressure byte 2 (MSB) |
| TEMP_OUT_L  | 0x2B    | Temperature LSB |
| TEMP_OUT_H  | 0x2C    | Temperature MSB |

### BLE Data Formats

**Pressure (3 bytes):**
```
Offset 0–2: uint24 little-endian raw
Scaling: raw / 4096.0 → hPa
```

**Temperature (2 bytes):**
```
Offset 0–1: int16 little-endian raw (LPS22HH native)
Scaling: raw / 100.0 → °C
```

### Initialization

```c
// ODR = 1Hz, LPF enabled
I2C_WriteReg(0x5D, 0x10, 0x12);
```

### DMA Read Sequence

```c
// DMA reads 5 bytes starting at PRESS_OUT_XL (auto-increment)
I2C_ReadDMA(LPS_ADDR, PRESS_OUT_XL, lps_buf, 5);
// Callback: bytes 0-2 → pressure char, bytes 3-4 → temp char
```

---

## 4. ZMOD4510 — Air Quality (AQI)

**I2C Address:** 0x33  
**Bus:** I2C2  
**Update Rate:** 1 Hz (raw ADC only)  

### Important: Raw ADC Mode

The ZMOD4510 is used in **raw ADC output mode only**. The firmware does **not** compute an AQI index. The 8-byte ADC array is forwarded to the phone, where AQI calculation is performed using Renesas algorithm tables (liboaq library).

### BLE Data Format (11 bytes)

```
Offset  Size  Field
0       8     adc_readings[8]  — raw ADC values per MOX channel
8       2     sample_count     — uint16 little-endian, total samples
10      1     valid            — 0x01 = valid, 0x00 = warming up
```

### ZMOD4510 Initialization Sequence

```c
void ZMOD4510_Init(void) {
    // 1. Product ID check
    uint8_t pid;
    I2C_ReadReg(0x33, ZMOD4510_PID_REG, &pid, 1);
    
    // 2. Load configuration sequence (from datasheet)
    ZMOD4510_LoadConfig();
    
    // 3. Start measurement (OAQ_2ND_GEN_ULP algorithm recommended)
    ZMOD4510_StartMeasurement();
    
    // 4. Warm-up: skip first 60 samples before marking valid
}
```

### Warm-Up Period

The ZMOD4510 requires a warm-up period of ~60 seconds (60 samples at 1 Hz) before readings are stable. The `valid` byte in the BLE packet remains 0x00 during this period.

---

## 5. MIA-M10Q — GNSS

**I2C Address:** 0x42 (I2C mode)  
**Bus:** I2C1  
**Update Rate:** 1 Hz  
**Protocol:** UBX binary  

### Note on UART vs I2C

The MIA-M10Q supports both UART and I2C interfaces. In CubeMX, I2C1 is configured as the primary interface at 400 kHz. USART1 (PB14/PB15) was the original interface but the firmware was migrated to I2C1 for cleaner DMA integration.

### Control Pins (CRITICAL — Build 38)

| Pin | Function | Required boot level |
|-----|----------|---------------------|
| PA9 | RESET_N (active-low) | **HIGH** — releases the module |
| PB9 | VOL_SEL | **HIGH** — selects 3.3 V I/O (sampled at power-up) |

> **Root cause of "GNSS never works":** `MX_GPIO_Init()` originally drove PA9 LOW at boot, holding the M10Q in **permanent reset** — the module never ACKed on I2C and init always failed with "Device not responding at 0x42". PB9 was also driven LOW, selecting 1.8 V I/O. Both pins are now HIGH at boot; `GNSS_Init()` additionally performs a clean hardware reset pulse and retries device detection once after 500 ms extra boot time.

### Init Sequence (Build 38)

```
GNSS_Init()
  1. VOL_SEL → HIGH (3.3 V)
  2. GNSS_HardwareReset()          — clean reset pulse, release RESET_N
  3. HAL_I2C_IsDeviceReady ×3      — retry once after 500 ms if not ready
  4. GNSS_DisableNMEAOnI2C()       — UBX-CFG-VALSET: CFG-I2COUTPROT-NMEA = 0 (RAM+BBR)
  5. GNSS_FlushStream()            — drain up to 2 KB of stale (NMEA) bytes
  6. GNSS_SetUpdateRate(1000)      — 1 Hz navigation rate
```

> **Why NMEA must be disabled:** the M10 outputs NMEA sentences on I2C **by default**. The DMA read path reads the FRONT of the module's stream buffer and searches it for the UBX-NAV-PVT poll response — with NMEA enabled the response is queued behind hundreds of NMEA text bytes and is NEVER found, so no GNSS data ever reached BLE.

### DMA Read Path (Build 38)

- DMA buffer is **192 bytes** (was 92). The full UBX-NAV-PVT frame is 100 bytes (6 header + 92 payload + 2 checksum); the extra room consumes any stale/leading bytes.
- The read length is the full available byte count (capped at buffer size). **Reading less than the full frame leaves stale bytes queued in the module buffer, shifting every subsequent frame until parsing fails permanently** — this was the cause of "GNSS works then stops".
- The UBX header search is bounded so the parsed region stays inside the bytes actually read.
- Diagnostic: when UBX-NAV-PVT is not found, a rate-limited log prints the read length and first 4 bytes (miss #1, then every 30th).

### BLE Notification Gating (Build 38)

GNSS notifications (both the direct phone path in `SEND_GNSS_Notification()` and the peer-relay path in `SENSOR_SERVER_RelayGnssFromPeer()`) are sent **only when the phone has subscribed** to the GNSS characteristic (`Gnss_char_Notification_Status == ON`). Previously GNSS had no subscription flag and notified unconditionally, causing `0x0C` floods when the phone wasn't subscribed.

The sensor server also clears **only the handle that actually disconnected** (phone vs peer) — a peer disconnect no longer wipes the phone handle.

### BLE Data Format (20 bytes)

```
Offset  Size  Type     Field       Unit
0       4     float32  latitude    degrees (positive = North)
4       4     float32  longitude   degrees (positive = East)
8       4     float32  altitude    meters MSL
12      4     float32  speed       km/h
16      2     uint16   heading     degrees × 100 (0–36000)
18      1     uint8    fix_type    0=no fix, 2=2D, 3=3D
19      1     uint8    satellites  count
```

### UBX Parsing

The firmware requests `UBX-NAV-PVT` messages (0x01 0x07) which contain all navigation solution data in one 92-byte packet. The relevant fields are extracted and packed into the 20-byte BLE format.

```c
// UBX-NAV-PVT field offsets (within payload, after 6-byte header)
// lat:    byte 28  (int32, 1e-7 degrees)
// lon:    byte 24  (int32, 1e-7 degrees)
// height: byte 36  (int32, mm)
// gSpeed: byte 60  (uint32, mm/s)
// headMot:byte 64  (int32, 1e-5 degrees)
// fixType:byte 20  (uint8)
// numSV:  byte 23  (uint8)
```

---

## 6. BMI270 — Accelerometer & Gyroscope

**I2C Address:** 0x68  
**Bus:** I2C2  
**Update Rate:** 10 Hz  
**Trigger:** TIM1 ISR (on-demand)  

### BLE Data Formats

**Accelerometer (6 bytes):**
```
[X_L][X_H][Y_L][Y_H][Z_L][Z_H]  — int16 × 3 raw
Scaling: raw × 0.001 → m/s²
```

**Gyroscope (6 bytes):**
```
[X_L][X_H][Y_L][Y_H][Z_L][Z_H]  — int16 × 3 raw
Scaling: raw × 0.001 → °/s
```

### Initialization

```c
void BMI270_Init(void) {
    // Soft reset
    I2C_WriteReg(BMI_ADDR, 0x7E, 0xB6);
    HAL_Delay(10);
    
    // Load initialization sequence (mandatory for BMI270)
    BMI270_LoadConfigFile();  // 8KB config from Bosch SDK
    
    // Enable accel + gyro, ODR 100Hz, OSR4
    I2C_WriteReg(BMI_ADDR, 0x40, 0xA8);  // PWR_CTRL: accel+gyro enable
    I2C_WriteReg(BMI_ADDR, 0x41, 0x0A);  // ACCEL_CONF: ODR=100Hz, BW=normal
    I2C_WriteReg(BMI_ADDR, 0x42, 0x0A);  // GYRO_CONF: ODR=100Hz
}
```

### On-Demand Read (TIM1 ISR)

```c
void TIM1_ISR_Callback(void) {
    // Called at 10 Hz
    I2C_ReadDMA(BMI_ADDR, ACC_X_LSB, imu_buf, 12);
    // Callback: bytes 0-5 → accel, bytes 6-11 → gyro
}
```

---

## 7. TMAG5273 — Magnetometer

**I2C Address:** 0x78  
**Bus:** I2C2  
**Update Rate:** 10 Hz  
**Trigger:** TIM1 ISR (shared with BMI270)  

### BLE Data Format (6 bytes)

```
[X_L][X_H][Y_L][Y_H][Z_L][Z_H]  — int16 × 3 raw
Scaling: raw × 0.01 → µT (microtesla)
Range: ±163.84 µT default (±1 LSBT = 0.01µT)
```

### Initialization

```c
void TMAG5273_Init(void) {
    // Operating mode: continuous measurement
    I2C_WriteReg(TMAG_ADDR, 0x02, 0x07);  // X+Y+Z channels, 32-average
    // Range: ±163.84 µT
    I2C_WriteReg(TMAG_ADDR, 0x03, 0x01);
}
```

### DMA Read (10 Hz, after BMI270)

```c
// Reads 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
I2C_ReadDMA(TMAG_ADDR, MAG_X_MSB, mag_buf, 6);
```

---

## 8. DMA Architecture

### Current Implementation: Polling-Triggered DMA

> **The autonomous TIM2 DMA design (original) is disabled in firmware with `#if 0`.**  
> The current production implementation uses **polling-triggered DMA** via TIM2 ISR.

**How it works:**

```
TIM2 ISR (100 ms)
    │
    ├─ Slot 0: trigger I2C2 DMA read for MVH4000D
    │    └─ HAL_I2C_Master_Receive_DMA(&hi2c2, 0x54, buf, 2)
    │         └─ DMA complete → HAL_I2C_MasterRxCpltCallback()
    │              └─ pack BLE packet → aci_gatt_srv_notify()
    │
    ├─ Slot 2: trigger I2C2 DMA read for LPS22HH (+200ms)
    ├─ Slot 4: trigger I2C2 DMA read for ZMOD4510
    └─ Slot 6: trigger I2C1 DMA read for MIA-M10Q
```

### What "Autonomous" Meant (Disabled)

The original design used the STM32 DMA request router to have TIM2 directly trigger DMA without CPU intervention. This is commented out (`#if 0`) because it required more CubeMX configuration work and the polling approach was sufficiently stable for 1 Hz sensors.

### DMA Channels Used

| DMA Channel | I2C Bus | Sensor |
|-------------|---------|--------|
| DMA1 CH1    | I2C2 RX | MVH4000D, LPS22HH, ZMOD4510 (shared) |
| DMA1 CH2    | I2C1 RX | MIA-M10Q GNSS |

**Note:** I2C2 DMA channel is shared between three 1 Hz sensors. They are serialized by the TDMA slot schedule — never overlapping.

### Double Buffering

Each sensor has a double buffer `uint8_t raw[2][32]` to prevent data corruption if a new DMA completes before BLE notification is sent:

```c
static uint8_t mvh_buf[2][2];    // ping-pong for humidity
static uint8_t active_buf = 0;

void DMA_Complete_Callback(void) {
    uint8_t done = active_buf;
    active_buf ^= 1;  // switch to other buffer for next read
    // pack from mvh_buf[done] → BLE notify
}
```

---

## 9. Sensor Scheduler (TIM2 TDMA)

**Timer:** TIM2  
**Period:** 100 ms  
**Implementation file:** `Core/Src/sensor_scheduler.c` (~580 lines)

### Slot Assignment

```c
typedef struct {
    uint8_t  slot_id;
    uint32_t offset_ms;
    void    (*trigger_fn)(void);
} SensorSlot_t;

static const SensorSlot_t sensor_slots[] = {
    { 0,   0, MVH4000D_TriggerRead  },
    { 2, 200, LPS22HH_TriggerRead   },
    { 4, 400, ZMOD4510_TriggerRead  },
    { 6, 600, MIA_M10Q_TriggerRead  },
};
```

### TIM2 ISR

```c
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        sensor_tick++;
        for (int i = 0; i < NUM_SLOTS; i++) {
            if ((sensor_tick % (sensor_slots[i].offset_ms / 100)) == 0) {
                sensor_slots[i].trigger_fn();
            }
        }
    }
}
```

### NVIC Priority

BLE radio interrupts must always preempt sensor DMA. Priority configuration:

```
Priority 0: BLE Radio (RADIO_TXRX_IRQn)
Priority 1: BLE Stack
Priority 3: DMA1 CH1, CH2 (I2C RX)
Priority 5: TIM2 (scheduler), I2C1/I2C2 event/error
Priority 10: USART1 (debug)
```

---

## 10. IMU On-Demand System (TIM1)

**Timer:** TIM1  
**Rate:** 10 Hz (100 ms period between IMU reads)  
**Sensors:** BMI270 (accel + gyro) and TMAG5273 (mag)  

The IMU sensors are sampled at 10 Hz, higher than the 1 Hz DMA sensors. TIM1 is configured as an interrupt-driven periodic trigger separate from TIM2:

```c
// TIM1 configured: prescaler=6399 (64MHz/6400=10kHz), ARR=999 (100ms)
// TIM1_IRQHandler → imu_read_cycle()

void imu_read_cycle(void) {
    BMI270_ReadAccelGyro_DMA();  // 12 bytes: 6 accel + 6 gyro
    // Callback:
    //   → aci_gatt_srv_notify(accel_char, 6 bytes)
    //   → aci_gatt_srv_notify(gyro_char, 6 bytes)
    //   → TMAG5273_ReadMag_DMA()  (6 bytes)
    //     → aci_gatt_srv_notify(mag_char, 6 bytes)
}
```

Both BMI270 and TMAG5273 share I2C2. They are read sequentially (BMI270 first, then TMAG5273 in the BMI270 DMA callback) to avoid I2C bus conflict.

---

## 11. Long-Range RF Configuration

### Coded PHY S=8 (Maximum BLE Range)

Used for **peer-to-peer** (Module A ↔ Module B) connections. Phone connections remain on 1M PHY.

**PHY upgrade sequence (after initial 1M connection):**

```c
hci_le_set_phy(
    conn_handle,
    0x00,  // all PHYs preference
    0x04,  // TX: Coded PHY preferred
    0x04,  // RX: Coded PHY preferred
    0x02   // S=8 option (maximum coding gain)
);
```

**Confirmed in logs:**
```
✅ Peer PHY updated to Coded S=8
   TX_PHY = 3 (Coded), RX_PHY = 3 (Coded)
```

### TX Power — Maximum 8 dBm

```c
// app_conf.h
#define CFG_TX_POWER_MODE    1     // High-power mode
#define CFG_TX_POWER         0x1F  // +8 dBm (maximum for STM32WB07)
```

The **SKY66114-11** external PA/LNA (PA8 = RF RX enable, PA10 = RF TX enable) further boosts the effective output power.

### RX Sensitivity

| PHY              | RX Sensitivity |
|------------------|---------------|
| 1M PHY           | −96 dBm       |
| Coded PHY S=2    | −100 dBm      |
| Coded PHY S=8    | **−104 dBm**  |

### Data Length Extension (DLE)

DLE auto-fires after connection (and re-fires after the Coded PHY switch). **Build 38:** the DLE event is used as the synchronization point to start the peer MTU exchange — initiating MTU at connection time raced the stack's internal DLE negotiation and locked the ATT client (0x0C on CCCD writes). The Coded S=8 upgrade itself is requested only by the connection initiator (central) after the MTU/subscription chain completes.

```c
hci_le_set_data_length(
    peer_conn_handle,
    251,   // Max TX octets
    2120   // Max TX time µs (Coded PHY)
);
```

### Range Estimates

| Environment     | Estimated Range |
|-----------------|-----------------|
| Free space (LOS)| 1,200–1,500 m   |
| Suburban outdoor| 400–600 m       |
| Indoor obstructed| 100–200 m      |
| **Tested actual**| **5 km** (highway open area, LOS) |

Path loss margin = TX (+8 dBm) − RX sensitivity (−104 dBm) = **112 dB**

### Dual-PHY Scanning

To discover Coded PHY modules during scan as well as 1M devices:

```c
aci_gap_set_scan_configuration(
    SCAN_PASSIVE,
    GAP_GENERAL_DISCOVERY_PROC,
    LE_1M_PHY_BIT | LE_CODED_PHY_BIT,  // both PHYs
    SCAN_WIN_MS(100),
    SCAN_INT_MS(200)
);
```

---

## 12. SPI Motor Control

**Bus:** SPI2, full-duplex master  
**Pins:** PA4 (CS), PA5 (SCK), PA6 (MOSI), PA7 (MISO)  
**Clock:** 1 MHz (64 MHz / prescaler 64)  
**Mode:** CPOL=0, CPHA=0 (Mode 0)  
**Data:** 8-bit MSB-first  

### Command Format

Motor commands arrive via BLE GATT write (2 bytes) and are forwarded to SPI:

```
Byte 0: motor_id  (0x01–0x04)
Byte 1: value     (0x00–0xFF, duty cycle or direction encoding)
```

### Implementation

```c
void Motor_SendCommand(uint8_t motor_id, uint8_t value) {
    uint8_t tx[2] = { motor_id, value };
    uint8_t rx[2];
    
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 10);
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
}
```

### SPI2 CubeMX Configuration

| Parameter       | Value          |
|-----------------|----------------|
| Mode            | Full-Duplex Master |
| NSS             | Hardware Output |
| Data Size       | 8 bit          |
| First Bit       | MSB First      |
| Clock Polarity  | Low (CPOL=0)   |
| Clock Phase     | 1 Edge (CPHA=0)|
| Baud Rate       | 64 prescaler → 1 MHz |

### Motor PWM (TIM1)

Four PWM channels control motor speed/direction:

```
TIM1 CH1 ← PB11 — Motor 4
TIM1 CH2 ← PB10 — Motor 3
TIM1 CH3 ← PB3  — Motor 1 (note: CH3→PB3, CH4→PB2 mapping)
TIM1 CH4 ← PB2  — Motor 2

PWM frequency = 1 kHz (64 MHz / (PRE=63+1) / (ARR=999+1))
Duty cycle set via TIM1->CCR1–CCR4 (0–999 for 0–100%)
```

---

## 13. Function Reference

### Sensor Initialization

```c
void MVH4000D_Init(void);     // Initialize humidity sensor
void LPS22HH_Init(void);      // Initialize pressure + temperature sensor
void ZMOD4510_Init(void);     // Initialize AQI sensor (starts warm-up)
void MIA_M10Q_Init(void);     // Initialize GNSS, configure UBX-NAV-PVT
void BMI270_Init(void);       // Initialize IMU (loads 8KB config file)
void TMAG5273_Init(void);     // Initialize magnetometer

// GNSS control (Build 38)
HAL_StatusTypeDef GNSS_HardwareReset(void);      // Clean RESET_N pulse, release pin
HAL_StatusTypeDef GNSS_DisableNMEAOnI2C(void);   // UBX-CFG-VALSET: NMEA off on I2C port
void              GNSS_FlushStream(void);        // Drain stale bytes from module buffer
```

### DMA Trigger Functions (called by TIM2 ISR / scheduler)

```c
void MVH4000D_TriggerRead(void);   // Start I2C2 DMA read, 2 bytes
void LPS22HH_TriggerRead(void);    // Start I2C2 DMA read, 5 bytes
void ZMOD4510_TriggerRead(void);   // Start I2C2 DMA read, 11 bytes
void MIA_M10Q_TriggerRead(void);   // Start I2C1 DMA read, UBX packet
```

### DMA Callback Functions (called by HAL on DMA complete)

```c
void MVH_DMA_Complete(void);    // Parse humidity → BLE notify
void LPS_DMA_Complete(void);    // Parse pressure + temp → 2× BLE notify
void ZMOD_DMA_Complete(void);   // Pack AQI raw → BLE notify
void GNSS_DMA_Complete(void);   // Parse UBX-NAV-PVT → BLE notify
```

### IMU Functions (10 Hz, TIM1)

```c
void BMI270_ReadAccelGyro_DMA(void);  // Start 12-byte DMA; callback sends accel + gyro
void TMAG5273_ReadMag_DMA(void);      // Start 6-byte DMA; callback sends mag
void IMU_ReadCycle(void);             // Full 10 Hz cycle entry point (called from TIM1 ISR)
```

### BLE Notification Helpers

```c
// Send raw bytes to a GATT characteristic
tBleStatus Sensor_Notify(uint16_t char_handle, uint8_t *data, uint8_t len);

// Pause all sensor BLE notifications (used during BLE scan)
void Sensor_PauseNotifications(void);
void Sensor_ResumeNotifications(void);
```

### Dual-Role Manager

```c
DualRole_State_t DualRole_GetState(void);
void DualRole_SetState(DualRole_State_t state);
void DualRole_BuildDeviceListData(void);    // Build B_LIST binary payload
void DualRole_ClearDeviceList(void);
void DualRole_ScheduleAutoRetry(void);
void DualRole_OnPeerConnected(uint16_t conn_handle, uint8_t peer_addr_type, uint8_t *peer_addr);
void DualRole_OnPeerDisconnected(uint8_t reason);  // reason-gated retry (Build 38)
void DualRole_OnPhoneConnected(uint16_t conn_handle);
void DualRole_OnPhoneDisconnected(void);
```

### Low-Level I2C DMA

```c
// Wrapper around HAL_I2C_Master_Receive_DMA with error handling
HAL_StatusTypeDef I2C_ReadDMA(uint8_t addr, uint8_t reg,
                               uint8_t *buf, uint16_t len);

// Synchronous register write (used for init only)
HAL_StatusTypeDef I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val);
```

### SPI Motor

```c
void Motor_Init(void);
void Motor_SendCommand(uint8_t motor_id, uint8_t value);
void Motor_SetPWM(uint8_t motor_id, uint16_t duty_0_to_999);
```
