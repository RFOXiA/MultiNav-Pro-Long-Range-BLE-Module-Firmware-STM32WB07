# RFOXiA MultiNav Pro+ BLE Module — Dual-Role & Connection Reference

**Status:** Production Ready (Build 38 — July 2026)  
**Breakthrough:** Dual connections working after 36-build journey; symmetric peer topology + cross-platform chat completed in Build 38  
**Range tested:** 5 km (highway open area, LOS, Coded PHY S=8 + SKY66114-11)

---

## Table of Contents

1. [Dual-Role Architecture](#1-dual-role-architecture)
2. [State Machine](#2-state-machine)
3. [The 36-Build Journey to Success](#3-the-36-build-journey-to-success)
4. [Critical Bug Fixes](#4-critical-bug-fixes)
5. [Device Name Detection](#5-device-name-detection)
6. [Scan & Device List](#6-scan--device-list)
7. [Auto-Retry Logic](#7-auto-retry-logic)
8. [Chat Relay Service](#8-chat-relay-service)
9. [Long-Range Scanning Fix](#9-long-range-scanning-fix)
10. [B_LIST Reconnection Fix](#10-blist-reconnection-fix)
11. [Sensor Pause During Scan](#11-sensor-pause-during-scan)
12. [Connection Diagnostics](#12-connection-diagnostics)
13. [Known Limitations](#13-known-limitations)

---

## 1. Dual-Role Architecture

The firmware simultaneously operates as:

| Role        | Connection target | PHY used        |
|-------------|-------------------|-----------------|
| Peripheral  | Phone app         | 1M PHY          |
| Central     | Peer module (B)   | 1M → Coded S=8  |

Both connections are supported by `CFG_NUM_RADIO_TASKS = 2` in `app_conf.h`.

### Key Source Files

| File | Responsibility |
|------|---------------|
| `dual_role_manager.c/h` | State machine, connection coordination |
| `gatt_client_app.c` | Central-role scan + connect |
| `bconnection_server_app.c` | B_LIST / B_STATE / B_CMD characteristics |
| `app_ble.c` | BLE event routing, L2CAP rejection |

---

## 2. State Machine

```c
typedef enum {
    DUAL_STATE_IDLE           = 0x00,  // No connections
    DUAL_STATE_PHONE_CONNECTED = 0x01, // Phone connected only
    DUAL_STATE_SCANNING       = 0x02,  // Active scanning for peers
    DUAL_STATE_PEER_FOUND     = 0x03,  // Peer discovered, not yet connected
    DUAL_STATE_CONNECTING     = 0x04,  // Connection in progress
    DUAL_STATE_PEER_CONNECTED = 0x05,  // Peer connected, phone may be off
    DUAL_STATE_DUAL_ACTIVE    = 0x06,  // Both phone and peer connected ✅
    DUAL_STATE_ERROR          = 0xFF   // Error state
} DualRole_State_t;
```

### Typical Transition Flow

```
IDLE → PHONE_CONNECTED
       → (phone sends 'c') → SCANNING
                             → PEER_FOUND → CONNECTING → PEER_CONNECTED
                                                           → (phone reconnects) → DUAL_ACTIVE
```

State changes are reported to the phone via the **B_STATE notification** (handle 0x0241).

---

## 3. The 36-Build Journey to Success

### The Persistent Error

Every connection attempt returned:

```
aci_gap_create_connection() → 0xD1 (BLE_STATUS_COMMAND_DISALLOWED)
```

**Translation:** The BLE controller rejected the command — not because of wrong parameters, but because of **when and where it was called**.

### Failed Approaches (Builds 1–35)

| Builds | Theory Tested | Result |
|--------|--------------|--------|
| 1–5    | Add delays (100ms–5s) before connecting | ❌ Still 0xD1 |
| 6–12   | Full BLE stack re-initialization | ❌ Still 0xD1 |
| 13–20  | Different connection parameters (PHY, intervals, timeout) | ❌ Still 0xD1 |
| 21–28  | Stop advertising/scanning first, then connect | ❌ Still 0xD1 |
| 29–32  | Cache peer MAC address in global variable | ❌ Still 0xD1 |
| 33–35  | Flag + call from main loop (BUT flag cleared by event queue) | ❌ Still 0xD1 |

### The Breakthrough — Build 36: Task-Based Deferred Execution

**Root cause:** `aci_gap_create_connection()` **cannot be called from within a BLE event callback**. The radio control is not re-entrant at that point. The flag approach in builds 33–35 failed because the event queue continued processing between the flag being set and the main loop reading it, leaving the controller still in an invalid state.

**Solution discovered from ST's official BLE_p2pClient example:**

```c
// WRONG — called from within event handler (0xD1):
case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
    aci_gap_create_connection(...);   // ❌ 0xD1
    break;

// CORRECT — schedule as a separate sequencer task:
case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
    UTIL_SEQ_SetTask(1u << CFG_TASK_CONN_DEV_ID, CFG_SEQ_PRIO_0);
    break;  // return immediately!

// Task runs OUTSIDE event handler context:
void Connect_Request(void) {
    ret = aci_gap_create_connection(...);  // ✅ 0x00 SUCCESS
}
```

The `UTIL_SEQ_SetTask()` call returns control to the BLE event dispatcher immediately. The sequencer runs the connection task only after the event stack is fully unwound.

### Build 37: Stability Fix

Build 36 achieved dual connection, but it was unstable under load. Build 37 added:

1. **MBLOCKS 80 → 150** — peak usage ~36 MBLOCKs at dual connection
2. **Keepalive 4s** — prevents supervision timeout disconnect
3. **Supervision timeout 10s** — tolerates Android's fast-interval phase without dropping

---

## 4. Critical Bug Fixes

### Error 0xD1 — Command Disallowed (Build 36)

**Symptom:** `aci_gap_create_connection()` always returned 0xD1.  
**Cause:** Called from within a BLE event handler callback.  
**Fix:** Use `UTIL_SEQ_SetTask()` to defer connection to outside the event context.

---

### Error 0x28 — Supervision Timeout (Build 37)

**Symptom:** Peer connection dropped within 1–3 seconds.  
**Cause:** Android initially forces 7.5 ms connection intervals (L2CAP bypass). At high BLE load with dual connections, this causes supervision timeout at short timeout values.  
**Fix:**
```c
// Phone connection parameters
#define PHONE_CONN_INTERVAL_MIN   CONN_INT_MS(60)
#define PHONE_CONN_INTERVAL_MAX   CONN_INT_MS(90)
#define PHONE_SUPERVISION_TIMEOUT CONN_SUP_TIMEOUT_MS(10000)  // was 5s

// Peer connection parameters
#define PEER_SUPERVISION_TIMEOUT  CONN_SUP_TIMEOUT_MS(10000)
```

**Android behavior:** Android may momentarily use 7.5 ms intervals (bypassing L2CAP negotiation). This is normal. The 10s timeout is generous enough to survive this transient period.

---

### Error 0x0C — Memory Buffer Exhaustion (Build 37)

**Symptom:** Random disconnects, particularly when both phone and peer are active and sending sensor data.  
**Cause:** `CFG_BLE_MBLOCK_COUNT_MARGIN = 80` insufficient for dual connections. Peak observed: ~36 MBLOCKs.  
**Fix:**
```c
// app_conf.h
#define CFG_BLE_MBLOCK_COUNT_MARGIN  150  // was 80
```

---

### Errors 0x13 / 0x16 — Deliberate Termination Is Not Retried (Build 38)

**Symptom:** After one side explicitly disconnected the peer link, auto-retry immediately reconnected, creating an endless connect/disconnect loop (the retry counter was reset on each successful connect, so "1/3" repeated forever) which also destabilized the phone connection.  
**Cause:** `DualRole_OnPeerDisconnected()` had no disconnect-reason information and retried unconditionally.  
**Fix:** The disconnect reason is now passed in (`DualRole_OnPeerDisconnected(uint8_t reason)`). Reasons `0x13` (Remote User Terminated) and `0x16` (Terminated by Local Host) are deliberate terminations — no auto-retry, and any pending connection request is cleared.

```c
void DualRole_OnPeerDisconnected(uint8_t reason) {
    ...
    if (reason == 0x13 || reason == 0x16) {
        // deliberate termination - NO auto-retry
        retry_count = 0;
        pending_peer_connection = 0;
    }
    else if (was_central && auto_retry_enabled && ...) {
        // only the INITIATOR (central) retries; the acceptor just re-advertises
        DualRole_ScheduleAutoRetry();
    }
}
```

**Initiator-only retry:** Only the side that CREATED the connection (central, tracked by `peer_is_central`) auto-retries. If the acceptor also called `aci_gap_create_connection()`, the two retries raced each other (0xD1 failures) and delayed reconnection. The acceptor simply restarts advertising so the initiator can reconnect.

---

### Peer Classification by Address Type (Build 38)

**Symptom:** When the ACCEPTING module received an inbound peer connection, it sometimes misclassified the peer as a phone — breaking chat relay and Phone B's scan results.  
**Cause:** Classification relied on a hardcoded Module B MAC plus scan-context heuristics, which are asymmetric (only work on the initiating side).  
**Fix:** RFOXiA modules use STATIC RANDOM BLE addresses (address type = random, top two bits of the MSB = `11`, i.e. first printed octet `0xC0–0xFF`). Phones always connect with Resolvable Private Addresses (top bits `01`) or public addresses. A random address with top bits `11` can only be another RFOXiA module:

```c
if ((Peer_Address_Type & 0x01) == 0x01 && (Peer_Address[5] & 0xC0) == 0xC0) {
    is_peer_connection = 1;  // static random address = RFOXiA module
}
```

This is symmetric — it works on BOTH modules without hardcoded MACs.

---

### Phone Handle Corruption on Peer Connect (Build 38)

**Symptom:** After the peer link came up, B_LIST responses went to the peer instead of the phone (phone saw empty scan results) and sensor notifications targeted the wrong link, causing 0x0C floods.  
**Cause:** The generic connection-event dispatch stamped the NEW peer's handle as the "phone" peripheral handle in the sensor and bconnection servers.  
**Fix:** Immediately after registering the peer, the REAL phone handle (0xFFFF if no phone connected) is restored in both servers via `SENSOR_SERVER_PERIPH_CONN_HANDLE_EVT` / `BCONNECTION_SERVER_PERIPH_CONN_HANDLE_EVT`.

---

### Peer Connection Without a Phone Is Accepted (Build 38)

**Symptom:** Module B rejected every peer connection attempt from Module A (endless 0x13 connect/disconnect loop on Module A) whenever Phone B was not yet connected.  
**Cause:** Old code disconnected any inbound peer when no phone was attached (`hci_disconnect(0x13)`), assuming peer links were useless without a phone. That broke the intended topology **Phone A → Module A → Module B ← Phone B**, where Module B typically has no phone when the peer link is established.  
**Fix:** Peer links are always accepted. If no phone is connected yet, LP advertising is restarted so the module's own phone can still find and connect to it later. Advertising is only stopped when BOTH phone and peer are connected (2 links = advertising would cause 0x0C).

---

### Peer Link Bring-Up Sequencing: DLE → MTU → Subscribe → Coded PHY (Build 38)

**Symptom:** Intermittent 0x0C errors on CCCD writes right after peer connection; occasional LL procedure collisions during PHY upgrade.  
**Cause:** MTU exchange was initiated at connection time, racing the BLE stack's internal DLE negotiation and leaving the ATT client locked. Both sides also requested the Coded PHY upgrade simultaneously.  
**Fix:**
- MTU exchange (`aci_gatt_clt_exchange_config`) is initiated from the **DLE event handler** — by the time DLE auto-fires, all post-connection stack procedures are complete and the ATT client is idle.
- DLE fires again after the Coded S=8 switch (data-length re-negotiation); a `peer_mtu_exchanged` guard ensures MTU exchange runs only once per connection.
- Only the central (initiator) requests the Coded S=8 upgrade (`peer_is_central` + `coded_phy_requested` flags).
- Peer sensor subscription happens via direct CCCD writes with known handles (see below), chained after MTU exchange — with a guard against double subscription when multiple trigger paths are active.

---

### Direct CCCD Subscription — Discovery Bypass (Build 38)

**Symptom:** Standard GATT service discovery on the peer link only found GAP/GATT services (handles 0x0001–0x000F); the 128-bit sensor services arrived in a later ATT response that raced against a 0x08 disconnect.  
**Fix:** Both modules run identical firmware, so all sensor CCCD handles are known constants. `client_discover_all()` now skips discovery entirely and writes the CCCDs directly (`GATT_CLIENT_SubscribeToPeerTemperature()` starts the subscription state machine). Faster, more reliable, race-free.

---

### B_CMD Connect Guard While CONNECTING (Build 38)

**Symptom:** State machine stuck at state 4 (CONNECTING) with scans blocked.  
**Cause:** Auto-retry clears `pending_peer_connection` BEFORE calling `aci_gap_create_connection()`. If the phone sent a connect command (0x00) at that exact moment, it re-set the pending flag, the main loop called create_connection again → 0x0C failure → stuck in CONNECTING.  
**Fix:** Connect commands received while `current_state == DUAL_STATE_CONNECTING` are ignored (auto-retry handles the connection).

---

### L2CAP Supervision-Timeout Update Removed (Build 38)

**Symptom:** 0x28 "Instant Passed" disconnections shortly after phone connect.  
**Cause:** The firmware sent an L2CAP connection-parameter update request (5 s → 20 s supervision timeout) 1.5 s after connect. The LL applies parameter changes at a future "instant" that raced with scan startup.  
**Fix:** The update is no longer sent at all. The peer scan is capped at 2 s (duration = 200 × 10 ms), which fits comfortably within Android's default 5000 ms supervision timeout, making the update unnecessary.

---

### L2CAP Fast Interval (Android Behavior)

**Symptom:** After phone connects, connection interval drops to 7.5 ms for 5–10 seconds, causing sensor drop.  
**Observation:** Android bypasses L2CAP negotiation and forces 7.5 ms directly via HCI. The peripheral cannot enforce parameters when the Central bypasses L2CAP.  
**Mitigation:**
- 10s supervision timeout tolerates the fast-interval period
- Module requests preferred intervals in advertising data
- Android adjusts to stable 60–90 ms after ~10 seconds automatically

> **Do not fight this.** The 7.5 ms phase is a normal Android initialization pattern and self-resolves. Trying to reject it via L2CAP fails because Android bypasses L2CAP on these platforms.

---

## 5. Device Name Detection

### Architecture

Device name detection is needed to identify which BLE device is a peer RFOXiA module vs. a generic phone or other BLE device in the scan list.

**Device name = "RFOXiA BLE"** (set in firmware advertising data)

### How Scan Results Include Names

The STM32WB07 BLE stack returns device names in **scan response events** (`ACI_GAP_ADV_REPORT_VSEVT_CODE`). The firmware parses the AD structure to extract the Complete Local Name (type 0x09) or Shortened Local Name (type 0x08).

```c
void GattClient_ParseAdData(uint8_t *data, uint8_t len, DeviceInfo_t *device) {
    uint8_t i = 0;
    while (i < len) {
        uint8_t ad_len  = data[i];
        uint8_t ad_type = data[i + 1];
        
        if (ad_type == 0x09 || ad_type == 0x08) {  // Complete/Shortened local name
            uint8_t name_len = MIN(ad_len - 1, MAX_DEVICE_NAME_LEN);
            memcpy(device->name, &data[i + 2], name_len);
            device->name_len = name_len;
        }
        i += ad_len + 1;
    }
}
```

### Android Integration

The Android app receives device names in the B_LIST characteristic. Name bytes are decoded as UTF-8. The app filters the scan list to show only devices whose name contains "RFOXiA" and presents them for connection.

**Key discovery (Build ~30):** Early builds tried to retrieve names via GATT reads *after* connection. This added ~500ms latency and failed when the peer didn't support the Generic Access characteristic. The fix was to extract names **from advertisement packets during scan** (before connecting).

---

## 6. Scan & Device List

### Starting a Scan

The phone writes `'c'` (0x63) to the **B_CMD characteristic** (handle 0x0242).

Internally:

```c
void scanStart(void) {
    if (scan == 1) return;  // Prevent duplicate scan
    
    // Stop advertising temporarily (BLE stack limitation on STM32WB07)
    APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_STOP);
    
    DualRole_ClearDeviceList();
    scan = 1;
    
    // Configure scan parameters
    aci_gap_set_scan_configuration(
        SCAN_PASSIVE,
        GAP_GENERAL_DISCOVERY_PROC,
        LE_1M_PHY_BIT | LE_CODED_PHY_BIT,  // scan both PHYs
        SCAN_WIN_MS(100),
        SCAN_INT_MS(200)
    );
    
    // Start scan via sequencer task (outside event handler)
    UTIL_SEQ_SetTask(1u << CFG_TASK_SCAN_DEV_ID, CFG_SEQ_PRIO_0);
}
```

### B_LIST Device List Format

After scan completion, the device list is sent as a binary notification:

```
[device_count: 1 byte]
Per device:
  [name_len: 1 byte]
  [name: name_len bytes UTF-8]
  [MAC: 6 bytes BIG-ENDIAN]
  [RSSI: 1 byte, unsigned = actual_rssi + 128]
```

**MAC endian note:** The BLE stack stores addresses little-endian. The firmware reverses them (`addr[5-j]`) for display-ready big-endian ordering. Android displays as `AA:BB:CC:DD:EE:FF`.

### Connecting to a Device

Phone writes `'1'`–`'9'` (or `'10'`) to B_CMD. The firmware maps this index to the device list and calls `aci_gap_create_connection()` via the sequencer task pattern.

---

## 7. Auto-Retry Logic

When the peer link drops with a NON-deliberate reason (anything other than 0x13/0x16), the firmware schedules a retry — but **only on the side that initiated the connection** (central):

```c
#define PEER_AUTO_RETRY_MAX    3    // maximum retry attempts
#define PEER_AUTO_RETRY_DELAY  2000 // ms between retries

void DualRole_OnPeerDisconnected(uint8_t reason) {
    uint8_t was_central = ctx->peer_is_central;  // capture BEFORE clearing
    ...
    if (reason == 0x13 || reason == 0x16) {
        // deliberate termination - no retry, clear pending request
        ctx->retry_count = 0;
        ctx->pending_peer_connection = 0;
    }
    else if (was_central && ctx->auto_retry_enabled &&
             ctx->phone_connected && ctx->retry_count < ctx->max_retries) {
        DualRole_ScheduleAutoRetry();
    }
    // The acceptor (peripheral) never retries - it restarts advertising
    // so the initiator can reconnect to it.
}

// Reset on successful connection
void DualRole_OnPeerConnected(...) {
    retry_count = 0;
    // ...
}
```

---

## 8. Chat Relay Service

### Purpose

Relay text messages between Phone A (connected to Module A) and Phone B (connected to Module B), via the Module A → Module B BLE link.

### Architecture

```
Phone A → Module A Chat RX (0xFE51 write)
   → Module A reads → forwards to Module B via peer GATT write
     → Module B Chat TX (0xFE52 notify) → Phone B

Phone B → Module B Chat RX write
   → Module B forwards → Module A relay
     → Module A Chat TX notify → Phone A
```

### Chat Service UUIDs

| Item           | UUID / Handle                            |
|----------------|------------------------------------------|
| Service        | 0x0000FE50-0000-1000-8000-00805F9B34FB  |
| RX (write)     | 0x0000FE51-0000-1000-8000-00805F9B34FB  |
| TX (notify)    | 0x0000FE52-0000-1000-8000-00805F9B34FB  |
| Max payload    | 244 bytes (MTU 247 − 3 ATT overhead)    |

### Key Implementation Details

- The Chat service uses the **standard FE50 UUID** (registered with Bluetooth SIG). This ensures Android's generic BLE stack does not impose special filtering on it. A custom 128-bit UUID was tried first but caused discovery issues on some Android versions.
- **Runtime-resolved relay handles (Build 38):** the phone→peer relay writes to `CHAT_SERVER_GetWriteCharValueHandle()` and peer notifications are recognized by `CHAT_SERVER_GetNotifyCharValueHandle()`. Both modules run identical firmware, so the local handle equals the peer's handle. The old hardcoded handle `0x002C` actually pointed at a B_CONNECTION attribute — phone→peer messages were **silently dropped**.
- **Delivery receipts (app-level, Build 38):** when a phone app receives a chat message, it sends back `ACK:<hash8>` where `hash8` is the 8-hex-char djb2 hash of the message's UTF-8 bytes. The firmware relays these ACKs transparently like any chat message (a 12-byte relay in the logs = an ACK). Both Android and iOS use this to display ✓✓ double check marks. Sensor relay texts (`SENSOR:` prefix) are never ACKed.

### GATT Attribute Count

Adding the Chat service requires GATT database capacity. `CFG_BLE_NUM_GATT_ATTRIBUTES = 48` (Build 38). **The former value of 30 was too small: `aci_gatt_srv_add_service()` failed with `0x87 BLE_STATUS_OUT_OF_MEMORY` and the Chat service silently never existed in the GATT DB.** Actual usage is 41 attributes (Sensor 25 + Motor 3 + BConnection 7 + Chat 6).

---

## 9. Long-Range Scanning Fix

### Problem

After the system was configured for Coded PHY S=8 (long-range peer connections), scan stopped finding nearby devices.

**Root cause:** The scan was configured to use only `LE_1M_PHY_BIT`. Coded PHY devices were broadcasting on Coded PHY primary channels and not visible to 1M-only scans.

### Fix

Scan now uses both PHYs:

```c
aci_gap_set_scan_configuration(
    SCAN_PASSIVE,
    GAP_GENERAL_DISCOVERY_PROC,
    LE_1M_PHY_BIT | LE_CODED_PHY_BIT,  // ← added CODED PHY
    SCAN_WIN_MS(100),
    SCAN_INT_MS(200)
);
```

This allows Module A to discover both:
- Nearby BLE devices advertising on 1M PHY
- Long-range RFOXiA modules advertising on Coded PHY

### Scan Duty Cycle 20% → 90% (Build 38)

The connected-mode peer scan originally used a 20 ms window per 100 ms interval (20% duty cycle), giving only ~8% hit-rate per advertising period — the peer typically required 3+ scans to appear. The window was increased to 90 ms per 100 ms interval (90% duty cycle):

```c
paramA = SCAN_INT_MS(100);
paramB = SCAN_WIN_MS(90);   // was SCAN_WIN_MS(20)
```

At 90% duty cycle a single 2-second scan covers >99% of a 1000 ms advertising interval, so the peer reliably appears on the first scan. The scan remains capped at 2 s to stay within the phone link's 5 s supervision timeout.

---

## 10. B_LIST Reconnection Fix

### Problem

After Phone A disconnected and reconnected, the B_LIST characteristic showed empty or stale device data, even if Module B was still connected.

**Root cause:** The `blist_already_sent` flag in `bconnection_server_app.c` was set to `1` when the device list was first sent. On phone reconnection, the flag remained `1` so the list was not re-sent.

### Fix

Reset `blist_already_sent` when the phone reconnects:

```c
// In phone connection event handler (app_ble.c):
case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
    if (conn_is_phone) {
        blist_already_sent = 0;  // ← reset flag so list is resent
        DualRole_TriggerBlistUpdate();
    }
    break;
```

---

## 11. Sensor Pause During Scan

### Problem

When Module A started BLE scanning while simultaneously sending sensor notifications, sensor data dropped or BLE transfer failed intermittently.

**Root cause:** The I2C DMA operations and BLE radio scanning compete for the same radio timeslots on STM32WB07. The combined interrupt load caused I2C DMA callbacks to be delayed, resulting in partial reads and corrupt sensor packets.

### Fix

Pause sensor notifications during the scan window:

```c
void scanStart(void) {
    sensor_notifications_paused = 1;  // ← pause before scan
    // ... start scan ...
}

void scanComplete(void) {
    sensor_notifications_paused = 0;  // ← resume after scan
    // ... send device list ...
}
```

Sensor data is not lost — the DMA still reads, but notifications are held until scan completes (~2 seconds maximum).

---

## 12. Connection Diagnostics

### Reading Scan Command Receipt

When the phone writes `'c'` to B_CMD, the serial monitor should show:

```
🔔 BCONNECTION_SERVER_Notification: EvtOpcode=0x03
🔥🔥🔥 BLIST_STATE_CHAR_WRITE_NO_RESP_EVT - Write received!
     Handle: 0x0025
     Length: 1 bytes
>>>>>>>>>>> cmd : 0x63 <<<<<<<<<<<<<
>>>>>>>>>>>>>> SCAN COMMAND RECEIVED <<<<<<<<<<<<<<<<
```

If you don't see `🔥🔥🔥 BLIST_STATE_CHAR_WRITE_NO_RESP_EVT`, the command did not reach firmware — check that the Android app is writing to the correct handle (0x0025) and that the phone is still connected at time of write.

### Connection Interval Logs

A stable phone connection shows:

```
>>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE
     Connection Interval: 75.00 ms    ← stable (60–90 ms range)
```

If you see 7.5 ms and it never changes, the Android app may not have been granted the interval update. Wait 10 seconds — it should self-correct.

### Peer PHY Upgrade Confirmation

After peer connects and PHY upgrades:

```
✅ Peer PHY updated to Coded S=8
   TX_PHY = 3 (Coded)
   RX_PHY = 3 (Coded)
```

### Dual-Active Confirmation

```
State: 6 (DUAL_ACTIVE) — Phone + Peer both connected ✅
```

---

## 13. Known Limitations

| Limitation | Details |
|------------|---------|
| Peer detection requires scan-first | Module B must be advertising before Module A scans. If Module B is powered on after Module A has already scanned, a new scan is required to detect it. |
| Android 7.5 ms interval | Android Central devices bypass L2CAP and force 7.5 ms initially. This is not controllable from the peripheral side. It self-corrects in 5–10 seconds. |
| Scan pauses sensors | During a scan (~2s), sensor BLE notifications are paused to avoid I2C/radio contention. |
| Module B sensor subset | Module B hardware only carries AQI and GNSS sensors. Accel/Gyro/Mag/Pressure/Temp/Humidity are absent. |
| Range varies by environment | 5 km confirmed on highway open area (LOS). Expect less in urban or indoor environments due to attenuation. |
| Single peer connection | Module A connects to exactly one peer (Module B). No mesh or multi-peer support. |
