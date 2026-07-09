# RFOXiA MultiNav Pro+ BLE Module — Architecture & Design Reference

**MCU:** STM32WB07CC (ARM Cortex-M0+, 64 MHz)  
**BLE:** 5.4, dual-role (Peripheral + Central)  
**Build:** 38 — Production Ready  
**Date:** July 9, 2026  

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Components](#2-hardware-components)
3. [Software Stack](#3-software-stack)
4. [BLE Services & GATT Layout](#4-ble-services--gatt-layout)
5. [Sensor Data Formats](#5-sensor-data-formats)
6. [Connection Parameters](#6-connection-parameters)
7. [BLE Configuration Values](#7-ble-configuration-values)
8. [GPIO Pin Mapping](#8-gpio-pin-mapping)
9. [Module Topology](#9-module-topology)

---

## 1. System Overview

The RFOXiA firmware runs on an STM32WB07CC and implements a **dual-role BLE node** that:

- Acts as a **BLE Peripheral** (advertising, connecting to a phone app)
- Acts as a **BLE Central** (scanning for and connecting to peer modules)
- Reads **6 sensors** via I2C DMA
- Relays **motor commands** over SPI
- Relays **chat messages** between Module A phone and Module B phone

### High-Level Architecture

```
┌───────────────────┐        1M PHY / MTU 247
│   Phone App       │◄────────────────────────────┐
│  (Android/iOS)    │                             │
└───────────────────┘                             │
                                                  │
                                     ┌────────────┴──────────┐
                                     │     MODULE A           │
                                     │   STM32WB07CC          │
                                     │  (Peripheral + Central)│
                                     └────────────┬──────────┘
                                                  │
                                       Coded PHY S=8 / MTU 247
                                                  │
                                     ┌────────────┴──────────┐
                                     │     MODULE B           │
                                     │   STM32WB07CC          │
                                     │  (Peripheral to A)     │
                                     └───────────────────────┘
```

### Execution Model

The firmware runs in a **super-loop** driven by `UTIL_SEQ_Run()`. All BLE events are processed by registered tasks. Sensor reads are interrupt-driven (TIM2 ISR → I2C DMA). There is no RTOS.

```
main()
  └─ while(1) { UTIL_SEQ_Run(UTIL_SEQ_DEFAULT); }
```

---

## 2. Hardware Components

### Sensors (Module A — Full Configuration)

| Sensor     | Function          | I2C Address | Bus  | Rate    | DMA Mode      |
|------------|-------------------|-------------|------|---------|---------------|
| MVH4000D   | Humidity          | 0x54        | I2C2 | 1 Hz    | DMA each read |
| LPS22HH    | Pressure + Temp   | 0x5D        | I2C2 | 1 Hz    | DMA +200ms    |
| ZMOD4510   | Air Quality (AQI) | 0x33        | I2C2 | 1 Hz    | DMA raw ADC   |
| MIA-M10Q   | GNSS              | 0x42        | I2C1 | 1 Hz    | DMA polling   |
| BMI270     | Accel + Gyro      | 0x68        | I2C2 | 10 Hz   | On-demand TIM1|
| TMAG5273   | Magnetometer      | 0x78        | I2C2 | 10 Hz   | On-demand TIM1|

### Module B — Reduced Configuration
Module B only carries **ZMOD4510 (AQI)** and **MIA-M10Q (GNSS)**. The MVH4000D, LPS22HH, BMI270, and TMAG5273 sensors are absent on Module B hardware.

### RF Front-End

**SKY66114-11** PA/LNA on:
- **PA8** — RF RX (LNA enable)
- **PA10** — RF TX (PA enable)

This external PA gives the system +8 dBm TX power and ~-104 dBm RX sensitivity at Coded PHY S=8.

### Motor Control

- **SPI2** — 2-byte command to slave motor controller
- **TIM1 CH1–CH4 (PB11/10/2/3)** — PWM outputs for motor drive (1 kHz)

---

## 3. Software Stack

### File Organization

```
Core/
  Src/
    main.c                     — Super-loop, TIM2 ISR, initialization
    sensor_scheduler.c         — TDMA slot management per sensor
    zmod4510.c                 — ZMOD AQI sensor driver
    imu_dma.c                  — BMI270 + TMAG5273 driver (10Hz TIM1)
    app_entry.c                — Startup, BLE stack initialization
  Inc/
    app_conf.h                 — All tunable BLE config constants
STM32_BLE/App/
    app_ble.c                  — BLE events dispatcher, L2CAP handling
    dual_role_manager.c        — Dual-role state machine
    gatt_client_app.c          — Central role (scan, connect, GATT read)
    bconnection_server_app.c   — B_LIST / B_STATE / B_CMD characteristics
    sensor_server_app.c        — Sensor GATT notifications
    motor_server_app.c         — Motor GATT write handler
    chat_server_app.c          — Chat relay service
```

### Key Source Files — Line Counts
| File | Lines | Role |
|------|-------|------|
| main.c | ~800 | Entry point, TIM2 ISR, sensor polling loop |
| sensor_scheduler.c | ~580 | Sensor TDMA slots and DMA trigger |
| dual_role_manager.c | ~800 | Dual-role state machine |
| gatt_client_app.c | ~700 | Central scan + connect |
| app_ble.c | ~900 | All BLE event routing + L2CAP |

### Sensor Scheduler (TIM2 TDMA)

TIM2 fires every **100ms**. Each tick the scheduler assigns a time slot to a sensor:

```
Slot 0 (0ms):   MVH4000D humidity read (DMA)
Slot 2 (+200ms): LPS22HH pressure read (DMA)
Slot 4 (+400ms): ZMOD4510 AQI read (DMA)
Slot 6 (+600ms): MIA-M10Q GNSS read (DMA)

TIM1 (10Hz):    BMI270 accel/gyro + TMAG5273 mag — on-demand
```

**Note:** Autonomous TIM2-driven DMA (the original design) is **disabled** (`#if 0` in main.c). The current implementation uses **polling-DMA**: the TIM2 ISR triggers I2C reads, hardware DMA handles the byte transfer, and a completion callback posts the data to BLE.

---

## 4. BLE Services & GATT Layout

### Service Definitions

#### Sensor Service
**UUID:** `12345678-1234-5678-1234-56789abc2000`

| Characteristic | UUID Suffix | Properties | Bytes |
|----------------|-------------|------------|-------|
| Gyroscope      | ...abc2101  | Notify     | 6     |
| Accelerometer  | ...abc2102  | Notify     | 6     |
| Magnetometer   | ...abc2002  | Notify     | 6     |
| Pressure       | ...abc2003  | Notify     | 3     |
| Temperature    | ...abc2201  | Notify     | 2     |
| Humidity       | ...abc2202  | Notify     | 2     |
| Air Quality    | ...abc2005  | Notify     | 11    |

#### Motor Service
**UUID:** `12345678-1234-5678-1234-56789abc3000`

| Characteristic | UUID Suffix | Properties | Bytes |
|----------------|-------------|------------|-------|
| Motor Command  | ...abc3001  | Write/NoRsp| 2     |

#### Chat Service
**UUID:** `0x0000FE50-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID   | Properties | Bytes |
|----------------|--------|------------|-------|
| Chat RX        | 0xFE51 | Write/NoRsp| 244   |
| Chat TX        | 0xFE52 | Notify     | 244   |

#### B_CONNECTION Service (Dual-Role Control)

| Characteristic | Handle  | UUID   | Properties     |
|----------------|---------|--------|----------------|
| B_LIST         | 0x0240  | custom | Notify         |
| B_STATE        | 0x0241  | custom | Notify         |
| B_CMD          | 0x0242  | custom | Write          |

**B_CMD values:**
- `'c'` (0x63) — Start scan
- `'0'` (0x30) — Stop scan
- `'1'`–`'9'` — Connect to device at index 0–8
- `'10'` — Connect to device at index 9

### GATT Attribute Count

`CFG_BLE_NUM_GATT_ATTRIBUTES = 48`.

Actual usage: Sensor (25) + Motor (3) + B_CONNECTION (7) + Chat (6) = **41 attributes**, plus margin for future services.

> **History:** The previous value of 30 was too small — `aci_gatt_srv_add_service()` for the Chat service failed with `0x87 BLE_STATUS_OUT_OF_MEMORY`, so the Chat service silently never existed in the GATT database and chat was completely non-functional. Increased to 48 in Build 38.

### Chat Relay Handles (Runtime-Resolved)

Both modules run identical firmware, so the local chat characteristic handles equal the peer module's handles. The relay code obtains them at runtime:

```c
uint16_t CHAT_SERVER_GetWriteCharValueHandle(void);   // peer chat write VALUE handle
uint16_t CHAT_SERVER_GetNotifyCharValueHandle(void);  // peer chat notify VALUE handle
```

> **History:** The relay previously used a hardcoded handle `0x002C`, which actually pointed at a B_CONNECTION attribute — phone→peer chat messages were silently dropped. Fixed in Build 38.

---

## 5. Sensor Data Formats

All values transmitted via BLE notifications as raw binary (little-endian unless noted).

### GNSS — 20 bytes

```
Offset  Size  Field      Unit / Notes
0       4     latitude   float32, degrees (positive = N)
4       4     longitude  float32, degrees (positive = E)
8       4     altitude   float32, meters MSL
12      4     speed      float32, km/h
16      2     heading    uint16, degrees × 100
18      1     fix_type   0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4+=RTK
19      1     satellites uint8, number of tracked SVs
```

### Accelerometer — 6 bytes

```
Offset  Size  Field  Unit
0       2     X      int16, raw; multiply by 0.001 → m/s²
2       2     Y      int16, raw
4       2     Z      int16, raw
```

### Gyroscope — 6 bytes

```
Offset  Size  Field  Unit
0       2     X      int16, raw; multiply by 0.001 → °/s
2       2     Y      int16, raw
4       2     Z      int16, raw
```

### Magnetometer — 6 bytes

```
Offset  Size  Field  Unit
0       2     X      int16, raw; multiply by 0.01 → µT
2       2     Y      int16, raw
4       2     Z      int16, raw
```

### Pressure — 3 bytes

```
Offset  Size  Field     Unit
0       3     pressure  uint24 raw; divide by 4096.0 → hPa
```

### Temperature — 2 bytes

```
Offset  Size  Field       Unit
0       2     temperature int16 raw (LPS22HH native format)
```

### Humidity — 2 bytes

```
Offset  Size  Field    Unit
0       2     humidity uint16 raw (MVH4000D native format)
```

### Air Quality (ZMOD4510) — 11 bytes

```
Offset  Size  Field         Notes
0       8     adc_readings  8 × uint8 raw ADC channel values
8       2     sample_count  uint16 little-endian, total samples
10      1     valid         0x01 = valid read, 0x00 = warming up
```

> **Note:** The ZMOD4510 returns raw ADC results only. AQI index computation
> must be done on the Android/phone side using Renesas algorithm tables.

### Motor Command — 2 bytes

```
Offset  Size  Field    Notes
0       1     motor_id 0x01–0x04 (motor number)
1       1     value    0x00–0xFF (duty cycle or direction)
```

### Chat Message — up to 244 bytes

UTF-8 encoded text payload, no null terminator required. Maximum length = MTU (247) - 3 ATT header = 244 bytes.

### B_LIST Device List (variable length)

```
[device_count: 1 byte]
For each device:
  [name_len: 1 byte]
  [name: name_len bytes, UTF-8]
  [MAC: 6 bytes, big-endian]
  [RSSI: 1 byte, unsigned = actual_rssi + 128]
```

---

## 6. Connection Parameters

### Phone Connection (Module as Peripheral)

| Parameter          | Value                |
|--------------------|----------------------|
| Advertising PHY    | LE 1M                |
| Adv interval       | 20–30 ms (0x0020–0x0030) |
| Connection PHY     | LE 1M                |
| Connection interval| 60–90 ms             |
| Slave latency      | 0                    |
| Supervision timeout| 10 s (1000 units)    |
| MTU                | 247 bytes            |
| Preferred CE length| 20–40 ms             |

### Peer Connection (Module as Central)

| Parameter           | Value                          |
|---------------------|--------------------------------|
| Initial scan PHY    | LE 1M                         |
| Initial conn PHY    | LE 1M (upgrades after connect)|
| PHY after upgrade   | Coded PHY S=8                 |
| Connection interval | 50–100 ms                     |
| Slave latency       | 0                             |
| Supervision timeout | 10 s (1000 units)             |
| MTU                 | 247 bytes                     |
| Max TX octets       | 251                           |
| Max TX time         | 2120 µs (Coded PHY)           |

### PHY Upgrade (Long-Range)

After the peer connection is established, the link is brought up in a strict sequence to avoid GATT/LL procedure collisions (each step waits for the previous one to complete):

```
Connect (1M PHY)
  → DLE event auto-fires (stack finishes post-connection setup)
    → MTU exchange (aci_gatt_clt_exchange_config) — initiated from the DLE handler
      → CCCD subscriptions (direct writes, known handles)
        → Coded PHY S=8 upgrade — initiated by the CENTRAL side only
```

```c
hci_le_set_phy(
    conn_handle,
    0x00,  // all PHYs preference
    0x04,  // TX: Coded PHY preferred
    0x04,  // RX: Coded PHY preferred
    0x02   // S=8 option
);
```

**Rules (Build 38):**
- MTU exchange must NOT be started at connection time — it races the internal DLE negotiation and locks the ATT client, causing `0x0C` errors on CCCD writes. The DLE event is the synchronization point.
- DLE fires a second time after the Coded PHY switch (data length re-negotiation); MTU exchange is guarded to run only once per connection (`peer_mtu_exchanged`).
- Only the connection initiator (central, tracked by `peer_is_central`) requests the Coded S=8 upgrade, preventing LL procedure collisions when both sides try simultaneously.

This provides ~−104 dBm receiver sensitivity and 5 km tested range (highway open area, line-of-sight).

---

## 7. BLE Configuration Values

Defined in `Core/Inc/app_conf.h`:

```c
#define CFG_NUM_RADIO_TASKS             2       // Supports 2 simultaneous connections
#define CFG_BLE_NUM_GATT_ATTRIBUTES     48      // 41 used (Sensor 25 + Motor 3 + BConn 7 + Chat 6) + margin
#define CFG_BLE_ATT_MTU_MAX             247     // Maximum GATT payload
#define CFG_BLE_MBLOCK_COUNT_MARGIN     150     // Memory blocks (was 80, increased for stability)
#define CFG_TX_POWER_MODE               1       // High-power mode
#define CFG_TX_POWER                    0x1F    // +8 dBm (maximum)
#define ADV_INTERVAL_MIN                0x0020  // 20 ms
#define ADV_INTERVAL_MAX                0x0030  // 30 ms
```

**Memory block note:** Peak usage observed is ~36 MBLOCKs at dual-connection. Margin of 150 prevents 0x0C (buffer exhaustion) errors. Former value of 80 caused intermittent disconnects.

**GATT attribute note:** The former value of 30 caused the Chat service registration to fail with `0x87 BLE_STATUS_OUT_OF_MEMORY` (service silently missing from GATT DB).

---

## 8. GPIO Pin Mapping

| Pin  | Function        | Mode                        | Notes                        |
|------|-----------------|-----------------------------|------------------------------|
| PA0  | I2C1 SCL        | Open-drain alternate        | GNSS bus (MIA-M10Q)          |
| PA1  | I2C1 SDA        | Open-drain alternate        | GNSS bus                     |
| PA4  | SPI2 NSS/CS     | Alternate (hardware NSS)    | Motor CS                     |
| PA5  | SPI2 SCK        | Push-pull alternate         | Motor clock                  |
| PA6  | SPI2 MOSI       | Push-pull alternate         | Motor data out               |
| PA7  | SPI2 MISO       | Push-pull alternate         | Motor data in                |
| PA8  | RF RX           | Output                      | SKY66114-11 LNA enable       |
| PA9  | GNSS RESET_N    | Output, **HIGH at boot**    | MIA-M10Q reset (active-low)  |
| PA10 | RF TX           | Output                      | SKY66114-11 PA enable        |
| PA13 | I2C2 SCL        | Open-drain alternate        | Sensor bus (all except GNSS) |
| PA14 | I2C2 SDA        | Open-demand alternate       | Sensor bus                   |
| PB2  | TIM1 CH4        | Push-pull alternate         | Motor 2 PWM                  |
| PB3  | TIM1 CH3        | Push-pull alternate         | Motor 1 PWM                  |
| PB9  | GNSS VOL_SEL    | Output, **HIGH at boot**    | MIA-M10Q I/O voltage: HIGH = 3.3 V |
| PB10 | TIM1 CH2        | Push-pull alternate         | Motor 3 PWM                  |
| PB11 | TIM1 CH1        | Push-pull alternate         | Motor 4 PWM                  |
| PB14 | UART1 RX        | Alternate                   | Debug serial (SWO)           |
| PB15 | UART1 TX        | Alternate                   | Debug serial (115200 baud)   |

**Hardware requirement:** I2C2 (PA13/PA14) and I2C1 (PA0/PA1) require external 4.7 kΩ pull-up resistors to 3.3 V.

> **CRITICAL (Build 38):** PA9 (GNSS RESET_N) is active-low and MUST be driven HIGH in `MX_GPIO_Init()`. The old code drove it LOW at boot, holding the M10Q in permanent reset — the module never ACKed on I2C and GNSS init always failed with "Device not responding at 0x42". Similarly, PB9 (VOL_SEL) is sampled by the M10Q at power-up and must be HIGH from the very start to select 3.3 V I/O.

---

## 9. Module Topology

### Module A — Full Node

```
Module A
├── Sensors: MVH4000D · LPS22HH · ZMOD4510 · MIA-M10Q · BMI270 · TMAG5273
├── BLE role: Peripheral (to phone) + Central (to Module B)
├── Services: Sensor · Motor · Chat · B_CONNECTION
└── Phone A ↔ Module A ↔ Module B ↔ Phone B   (relay both ways)
```

### Module B — Reduced Node

```
Module B
├── Sensors: ZMOD4510 · MIA-M10Q only
├── BLE role: Peripheral (to Module A) + Peripheral (to Phone B)
├── Services: Sensor · Chat
└── Note: MVH4000D, LPS22HH, BMI270, TMAG5273 absent in hardware
```

### Data Relay Path

```
Phone A writes motor command / chat
  → Module A receives via Sensor/Motor/Chat service
    → Module A forwards over peer BLE connection
      → Module B receives, executes locally
        → Module B sensor data flows back via peer connection
          → Module A relays to Phone A notifications
```
