# RFOXiA MultiNav Pro+ BLE Module — Integration, Build & Operations Guide

**Build:** 38 — Production Ready (July 2026)  
**Toolchain:** STM32CubeIDE + STM32CubeMX  
**Target:** STM32WB07CCV6  
**Flash tool:** STM32CubeProgrammer

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [CubeMX Configuration](#2-cubemx-configuration)
3. [Build Instructions](#3-build-instructions)
4. [Flash Instructions](#4-flash-instructions)
5. [Debug Setup](#5-debug-setup)
6. [Test Procedures](#6-test-procedures)
7. [Troubleshooting](#7-troubleshooting)
8. [Changelog](#8-changelog)

---

## 1. Quick Start

Minimum steps to get the firmware running:

1. **Open in CubeIDE:** `File → Open Projects from File System` → select the `Firmware/` folder.
2. **Build:** `Project → Build Project` (Ctrl+B). Output: `Debug/BLE Pro test Debug.elf`.
3. **Connect hardware:** ST-Link debugger to Module A via SWD.
4. **Flash:** Open STM32CubeProgrammer → Connect → Open .elf → Download.
5. **Verify:** Open serial terminal at **115200 baud** on the debug UART (PB14/PB15). You should see:
   ```
   BLE initialization complete
   ==>> Success: aci_gap_set_advertising_enable
   Advertising enabled on 1M PHY...
   Device name: RFOXiA BLE
   ```
6. **Connect phone:** Open the Android app, scan for "RFOXiA BLE", connect. Sensor data flows automatically.

---

## 2. CubeMX Configuration

Open `BLE Pro test.ioc` in STM32CubeMX. All settings below should already be correct in the project file, but verify before regenerating code.

### 2.1 I2C2 — Main Sensor Bus

**Path:** Connectivity → I2C2  
**Pins:** PA13 (SCL), PA14 (SDA)

| Parameter          | Value                 | Reason                    |
|--------------------|-----------------------|---------------------------|
| Mode               | I2C                   | I2C master                |
| I2C Speed Mode     | Standard Mode         | 100 kHz                   |
| Clock Speed        | 100,000 Hz            | Conservative for reliability |
| Analog Filter      | Enable                | Reduces I2C bus noise     |
| Digital Filter     | 0                     | Analog filter sufficient  |
| GPIO Mode          | Open-Drain AF         | Both pins                 |
| GPIO Pull          | No pull               | External 4.7 kΩ required |

> **Hardware:** Install external 4.7 kΩ pull-up resistors on PA13 and PA14 to 3.3 V. Missing pull-ups are the #1 cause of I2C initialization failures.

**Expected timing register value:** `0x10707DBC` (64 MHz SYSCLK, Standard Mode 100 kHz)

> **Known issue:** An older configuration had `hi2c2.Init.Timing = 0x00000000`. This is invalid. If sensors fail to initialize, check this value in `i2c.c`.

---

### 2.2 I2C1 — GNSS Bus

**Path:** Connectivity → I2C1  
**Pins:** PA0 (SCL), PA1 (SDA)

| Parameter    | Value     | Reason                     |
|--------------|-----------|----------------------------|
| Speed Mode   | Fast Mode | 400 kHz (MIA-M10Q supports)|
| GPIO Mode    | Open-Drain AF | Both pins              |
| GPIO Pull    | No pull   | External 4.7 kΩ required   |

---

### 2.3 USART1 — Debug Serial

**Path:** Connectivity → USART1  
**Pins:** PB14 (RX), PB15 (TX)

| Parameter      | Value      |
|---------------|------------|
| Mode           | Asynchronous |
| Baud Rate      | 9600 (GNSS) / **115200 (debug)** |
| Word Length    | 8 bits     |
| Parity         | None       |
| Stop Bits      | 1          |
| Flow Control   | None       |

> **Note:** USART1 serves double duty — originally wired to MIA-M10Q GNSS UART; now used as the debug printf output at 115200 baud. GNSS is on I2C1 in production.

---

### 2.4 SPI2 — Motor Control

**Path:** Connectivity → SPI2  
**Pins:** PA4 (NSS/CS), PA5 (SCK), PA6 (MOSI), PA7 (MISO)

| Parameter       | Value              |
|-----------------|--------------------|
| Mode            | Full-Duplex Master |
| Hardware NSS    | Output Signal      |
| Data Size       | 8 Bits             |
| First Bit       | MSB First          |
| Clock Polarity  | Low (CPOL=0)       |
| Clock Phase     | 1 Edge (CPHA=0)    |
| Baud Rate Prescaler | 64 → 1 MHz (64 MHz / 64) |
| NVIC            | Enable SPI2 global interrupt |

---

### 2.5 TIM1 — Motor PWM

**Path:** Timers → TIM1  
**Pins:** PB11 (CH1), PB10 (CH2), PB3 (CH3), PB2 (CH4)

| Parameter    | Value  | Calculation                  |
|--------------|--------|------------------------------|
| Prescaler    | 63     | 64 MHz / 64 = 1 MHz timer    |
| Counter Period (ARR) | 999 | 1 MHz / 1000 = 1 kHz PWM |
| Channel 1–4  | PWM Generation | All four motors    |
| PWM Mode     | PWM Mode 1 | Rising edge active     |
| CH Polarity  | High   |                              |

**Motor mapping:**

| TIM1 Channel | GPIO Pin | Motor |
|--------------|----------|-------|
| CH1          | PB11     | Motor 4 |
| CH2          | PB10     | Motor 3 |
| CH3          | PB3      | Motor 1 |
| CH4          | PB2      | Motor 2 |

---

### 2.6 TIM2 — Sensor Scheduler

**Path:** Timers → TIM2

| Parameter    | Value   | Calculation          |
|--------------|---------|----------------------|
| Prescaler    | 999     | 64 MHz / 1000 = 64 kHz |
| Counter Period | 6399  | 64 kHz / 6400 = 10 Hz (100ms) |
| Global interrupt | Enable |                   |

---

### 2.7 NVIC Priority Table

| Interrupt Source          | Priority | Sub-priority | Reason |
|---------------------------|----------|--------------|--------|
| BLE Radio (RADIO_*)       | 0        | 0            | Highest — never preempt BLE |
| BLE Stack                 | 1        | 0            | Stack events |
| DMA1 CH1, CH2             | 3        | 0            | Sensor DMA callbacks |
| TIM2                      | 5        | 0            | Sensor scheduler |
| I2C1, I2C2 Event/Error    | 5        | 0            | I2C bus management |
| USART1                    | 10       | 0            | Debug, non-critical |

> **Critical:** BLE must always preempt sensors. If DMA priorities exceed BLE radio priorities, packet loss and disconnects occur.

---

### 2.8 BLE Stack Settings

In CubeMX Wireless → BLE:

| Parameter                  | Value | Notes |
|----------------------------|-------|-------|
| NUM_RADIO_TASKS            | 2     | Two simultaneous connections |
| BLE_NUM_GATT_ATTRIBUTES    | 48    | 41 used; 30 caused Chat service 0x87 OUT_OF_MEMORY |
| BLE_ATT_MTU_MAX            | 247   | Maximum GATT payload |
| MBLOCK_COUNT_MARGIN        | 150   | Increased from 80 — prevents 0x0C |
| TX_POWER_MODE              | 1     | High-power mode |
| TX_POWER                   | 0x1F  | +8 dBm |

After changing CubeMX settings, click **Generate Code** — do NOT overwrite manually edited files unless you diff first.

---

## 3. Build Instructions

### STM32CubeIDE GUI (Recommended)

1. Open **STM32CubeIDE**
2. `File → Open Projects from File System` → select `Firmware/`
3. `Project → Clean Project`
4. `Project → Build Project` (Ctrl+B)
5. Check Console — look for `Build Finished. 0 errors.`
6. Output: `Firmware/Debug/BLE Pro test Debug.elf`

### Common Build Errors

| Error | Cause | Fix |
|-------|-------|-----|
| `undefined reference to BMI270_LoadConfigFile` | Missing Bosch SDK file | Ensure `bmi270_config.c` is in project |
| `CFG_BLE_NUM_GATT_ATTRIBUTES` redefined | CubeMX regenerated `app_conf.h` | Restore value to 48 |
| Linker region overflow | Debug symbols too large | Use Release build configuration |

---

## 4. Flash Instructions

### Using STM32CubeProgrammer (GUI)

1. Connect **ST-Link** to Module A via SWD (4 pins: SWDIO, SWDCLK, GND, 3.3V)
2. Open **STM32CubeProgrammer**
3. Click **Connect** (verify ST-Link detected in top-right)
4. `File → Open File` → select `Debug/BLE Pro test Debug.elf`
5. Click **Download** (blue arrow icon)
6. Wait for `File download complete` message
7. Click **Disconnect**
8. Power cycle Module A (or press RST button)

### Using SWD via OpenOCD (command line)

```bash
openocd -f interface/stlink.cfg -f target/stm32wbx.cfg \
  -c "program {BLE Pro test Debug.elf} verify reset exit"
```

### Verify Flash

After flashing, the serial monitor should print the boot greeting within 2 seconds:

```
=== RFOXiA MultiNav Pro+ Build 38 ===
Initializing sensors...
  MVH4000D: OK
  LPS22HH:  OK
  ZMOD4510: OK (warming up)
  MIA-M10Q: OK
  BMI270:   OK
  TMAG5273: OK
BLE initialization complete
Advertising enabled on 1M PHY...
```

If you see `I2C Error` for any sensor, check pull-up resistors and I2C timing register.

---

## 5. Debug Setup

### 5.1 Serial Monitor (Primary Debug Method)

**Connection:** PB15 = UART TX → USB-UART adapter → PC  
**Settings:** 115200 baud, 8N1, no flow control  
**Recommended tool:** PuTTY, Tera Term, or VS Code Serial Monitor

All `APP_DBG_MSG()` calls output to this UART.

### 5.2 SWD / ST-Link Debugging in CubeIDE

1. Open `BLE Pro test Debug.launch` in CubeIDE
2. `Run → Debug Configurations` → select `BLE Pro test Debug`
3. Click **Debug** — CubeIDE connects to ST-Link, flashes, and breaks at main()
4. Use standard debugger controls: Step Over (F6), Step Into (F5), Resume (F8)

**For sensor issues:** Set a breakpoint in `HAL_I2C_MasterRxCpltCallback()` to inspect raw DMA buffers.

**For BLE issues:** Set a breakpoint in `app_ble.c` at the received event switch statement.

### 5.3 SWV / ITM Trace (NUCLEO boards)

If using a NUCLEO-WB07CC evaluation board:

1. Enable SWV in `BLE Pro test Debug.launch` → SWO settings → enable ITM port 0
2. Open `Window → Show View → SWV → SWV ITM Data Console`
3. Configure SWO clock = 64 MHz

> **Note:** SWV is only available when connected via ST-Link with SWO support. Custom hardware without SWO routing must use UART debug.

### 5.4 Useful Debug Log Patterns

**Scan command receipt:**
```
🔔 BCONNECTION_SERVER_Notification: EvtOpcode=0x03
🔥🔥🔥 BLIST_STATE_CHAR_WRITE_NO_RESP_EVT - Write received!
     cmd : 0x63 → SCAN COMMAND RECEIVED
```

**Stable phone connection:**
```
>>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE
     Connection Interval: 75.00 ms     ← stable
```

**PHY upgrade to long-range:**
```
✅ Peer PHY updated to Coded S=8
   TX_PHY = 3 (Coded), RX_PHY = 3 (Coded)
```

**Dual-active state:**
```
State: 6 (DUAL_ACTIVE) — Phone + Peer both connected ✅
```

---

## 6. Test Procedures

### TEST 1 — Sensor Data Flow

**Objective:** Verify all 6 sensors initialize and send BLE notifications.

1. Flash Module A, open serial monitor
2. Connect phone via Android app
3. Enable all sensor notifications in the app
4. **Expected:** Serial shows DMA complete callbacks for each sensor, app shows live values

**Pass criteria:**
- Humidity (MVH4000D): updating at 1 Hz, value ~30–80% RH reasonable
- Pressure (LPS22HH): updating at 1 Hz, value ~980–1020 hPa reasonable
- AQI (ZMOD4510): updating at 1 Hz, valid byte = 0x00 first 60s then 0x01
- GNSS (MIA-M10Q): updating at 1 Hz, fix_type = 3 outdoors, 0 indoors
- Accel (BMI270): updating at 10 Hz, Z ≈ +9800 when flat (gravity)
- Gyro (BMI270): updating at 10 Hz, values near 0 when stationary
- Mag (TMAG5273): updating at 10 Hz, magnitude ~45–65 µT typical

---

### TEST 2 — Scan Command Reaches Firmware

**Objective:** Verify Android app successfully sends scan command to firmware.

1. Connect phone to Module A
2. Press "Scan for Modules" in the Android app
3. **Check serial monitor immediately**

**Expected output:**
```
🔥🔥🔥 BLIST_STATE_CHAR_WRITE_NO_RESP_EVT - Write received!
     Handle: 0x0025
     Length: 1 bytes
     cmd : 0x63 → SCAN COMMAND RECEIVED
```

**If NOT seen:** Android app is not writing to the correct characteristic handle (0x0025) or the phone is disconnected. Check app BLE write logic.

---

### TEST 3 — Dual Connection (Module A + Module B)

**Prerequisite:** Module B must be powered on and advertising.

1. Power on Module B (verify serial shows "Advertising enabled")
2. Power on Module A, connect Phone A
3. In app, press "Scan for Modules"
4. Within 3 seconds, Module B should appear in the device list
5. Press "Connect" on Module B in the app
6. Wait for connection

**Expected serial on Module A:**
```
Scan complete — 1 device found: RFOXiA BLE
Connecting to peer...
✅ Peer connected: handle 0x0802
✅ Peer PHY updated to Coded S=8
State: 6 (DUAL_ACTIVE)
```

**Pass criteria:** Serial shows DUAL_ACTIVE, app shows both connections active, sensor data continues flowing from both modules.

---

### TEST 4 — Motor Command Relay

1. With dual connection active (Module A + Module B)
2. Use the Android app motor controls to send a command
3. **Expected:** Motor on Module B responds

**Expected serial on Module A:**
```
Motor command received: motor=1, value=128
Forwarding to peer via SPI...
```

---

### TEST 5 — Chat Relay

1. Dual connection active
2. Phone A sends a chat message via the app
3. **Expected:** Phone B receives the message

**Expected serial on Module A:**
```
Chat RX from phone: ["Hello from Phone A"]
Forwarding chat to peer module B...
```

---

## 7. Troubleshooting

### Sensors Not Initializing

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| `I2C Error: MVH4000D` | Missing pull-up resistors on I2C2 | Add 4.7 kΩ on PA13/PA14 |
| `I2C Error: ZMOD4510` | I2C timing register = 0x00000000 | Set proper timing in `i2c.c` |
| All sensors fail | Wrong I2C address in firmware | Verify addresses match hardware variant |
| BMI270 init fails | Missing config file load | Check `BMI270_LoadConfigFile()` call |
| GNSS "Device not responding at 0x42" | PA9 RESET_N held LOW at boot (module in permanent reset) | Drive PA9 HIGH in `MX_GPIO_Init()`; `GNSS_Init()` performs a clean reset pulse |
| GNSS detected but no position data ever | NMEA output on I2C floods the stream; UBX response buried | `GNSS_DisableNMEAOnI2C()` (UBX-CFG-VALSET) + `GNSS_FlushStream()` at init |
| GNSS works then stops permanently | Partial frame reads leave stale bytes queued, shifting all later frames | Read the full available data (192-byte DMA buffer), not just 92 bytes |

### BLE Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| 0xD1 on connection | `aci_gap_create_connection` called from event handler | Use `UTIL_SEQ_SetTask()` deferred pattern |
| 0x28 supervision timeout | Short timeout + Android 7.5ms phase | Set supervision timeout to 10s |
| 0x28 "Instant Passed" after connect | L2CAP parameter-update request racing scan startup | Do not send the L2CAP update — 2s scan cap fits the default 5s timeout |
| 0x0C buffer exhaustion | MBLOCKS too low | Set `CFG_BLE_MBLOCK_COUNT_MARGIN = 150` |
| 0x0C notify flood after peer connects | Peer handle stamped as phone handle in sensor/bconnection servers | Restore real phone handle after peer registration (Build 38) |
| 0x0C GNSS notify flood | GNSS notified without phone subscription | Gate on `Gnss_char_Notification_Status` (Build 38) |
| Chat service missing from GATT DB | `aci_gatt_srv_add_service` → 0x87 OUT_OF_MEMORY | `CFG_BLE_NUM_GATT_ATTRIBUTES = 48` (was 30) |
| Phone→peer chat silently dropped | Hardcoded relay handle 0x002C pointed at BConnection attribute | Use `CHAT_SERVER_GetWriteCharValueHandle()` (Build 38) |
| Peer misclassified as phone on acceptor | MAC-based classification asymmetric | Static-random-address heuristic (Build 38) |
| Endless peer connect/disconnect loop | Acceptor rejected peer without phone; retry after deliberate 0x13 | Accept peer without phone; reason-gated, initiator-only retry (Build 38) |
| Stuck at state 4 (CONNECTING), scan blocked | Phone connect cmd raced auto-retry | Ignore connect cmd while state = CONNECTING (Build 38) |
| Peer not found in scan | Module B not advertising | Power on Module B, check its serial |
| Peer needs 3+ scans to appear | 20% scan duty cycle | 90 ms window / 100 ms interval (Build 38) |
| Peer not found (long-range) | Scan using 1M PHY only | Add `LE_CODED_PHY_BIT` to scan config |
| BLIST empty after reconnect | `blist_already_sent` not reset | Reset flag on phone reconnection event |

### Android App Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Scan button does nothing | Writing to wrong handle | Verify write goes to handle 0x0025 |
| 7.5ms interval for >10s | Android L2CAP bypass | Normal — wait, it self-corrects |
| Sensor data stops during scan | Scan/sensor I2C conflict | Enable `sensor_notifications_paused` logic |

---

## 8. Changelog

### Build 38 — July 2026 (Production Release)

**Theme: Symmetric peer topology, working cross-platform chat, GNSS bring-up**

**BLE / dual-role:**
- **GATT attributes 30 → 48** — Chat service registration failed with `0x87 OUT_OF_MEMORY` at 30 (service silently missing). Actual usage: 41 (Sensor 25 + Motor 3 + BConnection 7 + Chat 6)
- **Peer classification by static random address** — symmetric heuristic (top address bits `11` = RFOXiA module); no more hardcoded MACs; fixes acceptor misclassifying the peer as a phone
- **Accept peer connection without a phone** — required for the Phone A → Module A → Module B ← Phone B topology; LP advertising restarts so the module's own phone can still connect
- **Phone handle restore after peer connect** — generic dispatch stamped the peer handle as the phone handle in sensor/bconnection servers, causing 0x0C floods and empty B_LIST
- **DLE → MTU → subscribe → Coded PHY sequencing** — MTU exchange moved to the DLE event (was racing at connect time, locking the ATT client → 0x0C on CCCD writes); MTU guarded to once per connection; Coded S=8 requested by the central only
- **Direct CCCD subscription** — peer service discovery bypassed (identical firmware → known handles); eliminates the discovery/disconnect race; double-subscription guard
- **Reason-gated, initiator-only auto-retry** — `DualRole_OnPeerDisconnected(reason)`: no retry on deliberate 0x13/0x16 termination (was an endless connect/disconnect loop); only the central retries, the acceptor re-advertises
- **B_CMD connect guard** — connect command ignored while already CONNECTING (fixed stuck-at-state-4)
- **L2CAP supervision-timeout update removed** — caused 0x28 "Instant Passed"; unnecessary since the 2 s scan cap fits Android's default 5 s timeout
- **Scan duty cycle 20% → 90%** — 90 ms window / 100 ms interval; peer now found on the first scan

**Chat:**
- **Runtime-resolved relay handles** — `CHAT_SERVER_GetWriteCharValueHandle()` / `GetNotifyCharValueHandle()`; the old hardcoded 0x002C pointed at a BConnection attribute so phone→peer messages were silently dropped
- **App-level ACK delivery receipts** — phones send `ACK:<djb2-hash8>` on receipt; relayed transparently by firmware (12-byte relays in logs); enables ✓✓ on Android + iOS

**GNSS (MIA-M10Q):**
- **PA9 RESET_N released at boot** — was driven LOW in `MX_GPIO_Init()`, holding the module in permanent reset ("Device not responding at 0x42"); now HIGH + clean reset pulse in `GNSS_Init()` with a 500 ms retry
- **PB9 VOL_SEL HIGH at boot** — sampled by the module at power-up; HIGH = 3.3 V I/O
- **NMEA disabled on I2C** — `GNSS_DisableNMEAOnI2C()` via UBX-CFG-VALSET (CFG-I2COUTPROT-NMEA = 0, RAM+BBR); NMEA flooding buried the UBX-NAV-PVT poll response so no fix ever reached BLE
- **Stream flush at init** — `GNSS_FlushStream()` drains up to 2 KB of stale bytes
- **DMA buffer 92 → 192 bytes, full-frame reads** — partial reads left stale bytes queued, shifting every later frame until parsing failed permanently; header search bounded to bytes actually read; periodic miss diagnostics added
- **GNSS notification gating** — `Gnss_char_Notification_Status` tracks the phone's CCCD subscription; both the direct path and the peer-relay path skip notify when unsubscribed (fixes 0x0C flood)
- **Per-handle disconnect clearing** — sensor server clears only the handle that actually disconnected (peer disconnect no longer wipes the phone handle)

---

### Build 37 — December 31, 2025

**Theme: Stability fixes for dual connection**

- **MBLOCKS 80 → 150** — Eliminates 0x0C buffer exhaustion errors
- **Supervision timeout 5s → 10s** — Tolerates Android's fast-interval phase
- **Keepalive timer 4s** — Prevents supervision timeout during quiet periods
- **Sensor pause during scan** — Fixes I2C/radio contention causing sensor drops
- **BLIST reconnect fix** — Reset `blist_already_sent` on phone reconnection
- **0x13/0x16 retry fix** — Do not retry after peer explicit rejection
- **Dual-PHY scan** — Added `LE_CODED_PHY_BIT` so Coded PHY modules are discoverable

---

### Build 36 — December 30, 2025 (Breakthrough)

**Theme: First working dual connection**

- **CRITICAL FIX: Deferred connection execution** — Moved `aci_gap_create_connection()` to `UTIL_SEQ_SetTask()` task, resolving 35-build streak of 0xD1 errors
- All connection attempt variants from builds 1–35 eliminated
- Dual-connection state machine fully functional: IDLE → PHONE → SCAN → PEER → DUAL_ACTIVE
- PHY upgrade to Coded S=8 working post-connection

---

### Build 35 and Earlier — November–December 2025 (Development)

**Major milestones:**

| Build Range | Achievement |
|-------------|-------------|
| 1–5         | Basic BLE peripheral (phone connection only) |
| 6–10        | Sensor DMA integration (MVH, LPS, ZMOD) |
| 11–15       | GNSS integration (MIA-M10Q, UBX parsing) |
| 16–20       | IMU integration (BMI270 + TMAG5273 at 10Hz) |
| 21–25       | Dual-role framework + B_CONNECTION service |
| 26–30       | Chat service integration (FE50 UUID) |
| 31–35       | Scan working, connection attempts failing (0xD1) |
| 36          | Dual connection working (deferred execution fix) |
| 37          | Production stable (MBLOCKS + timeout + scan pause) |
| 38          | Symmetric peer topology, cross-platform chat + ACKs, GNSS bring-up |

---

### December 2025 — Key Configuration Changes

| Parameter | Old Value | New Value | Reason |
|-----------|-----------|-----------|--------|
| CFG_BLE_MBLOCK_COUNT_MARGIN | 80 | 150 | Prevent 0x0C |
| CFG_BLE_NUM_GATT_ATTRIBUTES | 20 | 30 | Chat service |
| Supervision timeout | 5000ms | 10000ms | Android compatibility |
| Scan PHY | 1M only | 1M + Coded | Discover long-range devices |
| I2C2 timing | 0x00000000 | 0x10707DBC | Fix invalid timing |

### July 2026 — Key Configuration Changes (Build 38)

| Parameter | Old Value | New Value | Reason |
|-----------|-----------|-----------|--------|
| CFG_BLE_NUM_GATT_ATTRIBUTES | 30 | 48 | 30 → Chat service 0x87 OUT_OF_MEMORY |
| Peer scan window | 20 ms / 100 ms | 90 ms / 100 ms | Peer found on first scan |
| Chat relay write handle | 0x002C hardcoded | Runtime-resolved | 0x002C was a BConnection attribute |
| L2CAP 20s timeout request | Sent after 1.5 s | Removed | Caused 0x28 "Instant Passed" |
| PA9 (GNSS RESET_N) boot level | LOW | HIGH | LOW held M10Q in permanent reset |
| PB9 (GNSS VOL_SEL) boot level | LOW | HIGH | HIGH = 3.3 V I/O, sampled at power-up |
| GNSS DMA buffer | 92 bytes | 192 bytes | Full-frame reads, no stale bytes |
| GNSS NMEA on I2C | Enabled (default) | Disabled (UBX only) | NMEA buried UBX poll responses |

---

### 2026 — Future Work

| Priority | Item |
|----------|------|
| Medium   | Enable autonomous TIM2 DMA (currently `#if 0`) |
| Medium   | Module B sensor expansion (MVH, LPS hardware swap) |
| Low      | Multi-peer support (Module A connects to multiple Module B) |
| Low      | OTA firmware update support via BLE |
| Low      | GPS geofence alerts via peer link |
