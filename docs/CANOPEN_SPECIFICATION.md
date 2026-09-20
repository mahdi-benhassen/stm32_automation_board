# CANopen CiA 301 & CiA 401 Protocol Stack Specification

This document specifies the CANopen protocol architecture, Object Dictionary layout, hardware integration, and CanFestival `objdictgen` compatibility for the **STM32 Automation Board** (STM32F407) and external targets such as **STM32F767**.

---

## 1. Architectural Overview

The CANopen stack is designed to be modular, zero-dynamic-allocation (deterministic memory footprint), and hardware-decoupled:

```
+-------------------------------------------------------------------------+
|                         Application / User Code                         |
|     (DIP switch node ID, relay switching, ADC filtering, RTOS tasks)   |
+-------------------------------------------------------------------------+
       |                                                    |
       v                                                    v
+-----------------------+                         +-----------------------+
|  canopen_app_sync_*   |                         |   canopen_od_read/    |
|  (Weak I/O Callbacks) |                         |   canopen_od_write    |
+-----------------------+                         +-----------------------+
       ^                                                    ^
       |                                                    |
+-------------------------------------------------------------------------+
|                             CANopen Core                                |
|  - NMT State Machine (CiA 301 §7.2)                                     |
|  - Heartbeat Producer (CiA 301 §7.2.8.2)                                |
|  - SDO Server (Expedited & Segmented Upload, CiA 301 §7.2.4)            |
|  - PDO Engine (TPDO1/2, RPDO1/2, Event/Inhibit Timers, SYNC §7.2.2)     |
|  - Object Dictionary (CiA 401 Automation I/O Profile)                   |
+-------------------------------------------------------------------------+
       |
       v
+-------------------------------------------------------------------------+
|                       CAN Driver (bxCAN Wrapper)                        |
|  - APB1 Prescaler / Bit Timing (500 kbps @ 42MHz F4 / 54MHz F7)         |
|  - Dual Filter Bank Configuration (Broadcast + Node-Specific)           |
|  - Interrupt / Polled TX Mailbox & RX FIFO0                             |
+-------------------------------------------------------------------------+
```

---

## 2. Predefined Connection Set (CiA 301)

For an assigned Node ID \(N \in [1, 127]\):

| Function | COB-ID (Hex) | Direction | Payload (DLC) | Description |
|---|---|---|---|---|
| **NMT** | `0x000` | Rx | 2 bytes | Network management commands from NMT Master |
| **SYNC** | `0x080` | Rx | 0 bytes | Synchronization pulse triggering synchronous PDOs |
| **EMCY** | `0x080 + N` | Tx | 8 bytes | Emergency error message |
| **TPDO1** | `0x180 + N` | Tx | 1 byte | CiA 401: 8 Digital Inputs (`0x6000 sub 1`) |
| **RPDO1** | `0x200 + N` | Rx | 1 byte | CiA 401: 8 Digital Outputs (`0x6200 sub 1`) |
| **TPDO2** | `0x280 + N` | Tx | 8 bytes | CiA 401: 4 Analog Inputs 16-bit (`0x6401 sub 1..4`) |
| **RPDO2** | `0x300 + N` | Rx | 4 bytes | CiA 401: 2 Analog Outputs 16-bit (`0x6411 sub 1..2`) |
| **SDO Tx (Server)** | `0x580 + N` | Tx | 8 bytes | SDO response / upload data to Client |
| **SDO Rx (Server)** | `0x600 + N` | Rx | 8 bytes | SDO request / download data from Client |
| **Heartbeat / Bootup** | `0x700 + N` | Tx | 1 byte | Boot-up (`0x00`) or Heartbeat (`0x04`, `0x05`, `0x7F`) |

---

## 3. Object Dictionary (OD) Structure

The Object Dictionary data structures strictly conform to CanFestival `objdictgen` (`gen_cfile.py`):

```c
typedef struct {
    UNS8   bAccessType;   /* ro (0x05), wo (0x03), rw (0x07), const (0x01) */
    UNS8   bDataType;     /* uint8, uint16, uint32, visible_string, etc. */
    UNS32  size;          /* Object size in bytes */
    void  *pObject;       /* Direct pointer to memory variable */
} subindex;

typedef struct {
    subindex *pSubindex;   /* Pointer to array of subindices */
    UNS8      bEndpoints;  /* Number of subindices */
    UNS16     index;       /* CANopen 16-bit Index */
} indextable;
```

### 3.1 CiA 301 Communication Objects

| Index | Subindex | Name | Type | Access | Default Value / Description |
|---|---|---|---|---|---|
| `0x1000` | 0 | Device Type | `uint32` | `ro` | `0x00020191` (CiA 401 Digital & Analog I/O) |
| `0x1001` | 0 | Error Register | `uint8` | `ro` | `0x00` (Generic error bit 0) |
| `0x1008` | 0 | Device Name | `visible_string` | `ro` | `"STM32 Automation Board"` |
| `0x1009` | 0 | Hardware Version | `visible_string` | `ro` | `"v1.0"` |
| `0x100A` | 0 | Software Version | `visible_string` | `ro` | `"v1.0"` |
| `0x1017` | 0 | Heartbeat Producer Time | `uint16` | `rw` | `1000` (ms) |
| `0x1018` | 0 | Identity Object Length | `uint8` | `ro` | `4` |
| | 1 | Vendor ID | `uint32` | `ro` | `0x00000000` |
| | 2 | Product Code | `uint32` | `ro` | `0x00004070` |
| | 3 | Revision Number | `uint32` | `ro` | `0x00010000` |
| | 4 | Serial Number | `uint32` | `ro` | `0x12345678` |
| `0x1400` | 0..2 | RPDO1 Communication | - | - | COB-ID `0x200 + NodeID`, Transmission Type `254` |
| `0x1401` | 0..2 | RPDO2 Communication | - | - | COB-ID `0x300 + NodeID`, Transmission Type `254` |
| `0x1600` | 0..1 | RPDO1 Mapping | - | - | Maps `0x6200 sub 1` (8 bits DO) |
| `0x1601` | 0..2 | RPDO2 Mapping | - | - | Maps `0x6411 sub 1 & 2` (2x 16 bits AO) |
| `0x1800` | 0..5 | TPDO1 Communication | - | - | COB-ID `0x180 + NodeID`, Inhibit 100ms, Event 1000ms |
| `0x1801` | 0..5 | TPDO2 Communication | - | - | COB-ID `0x280 + NodeID`, Inhibit 100ms, Event 1000ms |
| `0x1A00` | 0..1 | TPDO1 Mapping | - | - | Maps `0x6000 sub 1` (8 bits DI) |
| `0x1A01` | 0..4 | TPDO2 Mapping | - | - | Maps `0x6401 sub 1..4` (4x 16 bits AI) |

### 3.2 CiA 401 Automation I/O Objects

| Index | Subindex | Name | Type | Access | Mapping | Board Hardware |
|---|---|---|---|---|---|---|
| `0x6000` | 0 | Number of 8-bit DI groups | `uint8` | `ro` | `1` | - |
| | 1 | Digital Inputs 1..8 | `uint8` | `ro` | **TPDO1** | 8x Opto-isolated inputs (PE0..7) |
| `0x6200` | 0 | Number of 8-bit DO groups | `uint8` | `ro` | `1` | - |
| | 1 | Digital Outputs 1..8 | `uint8` | `rw` | **RPDO1** | 4x Relays (PC8..11) + 4x DO (PB0,1,5,6) |
| `0x6401` | 0 | Number of 16-bit AI channels | `uint8` | `ro` | `4` | - |
| | 1 | Analog Input Channel 1 | `int16` | `ro` | **TPDO2** | AI0 (0-10V -> ADC1_IN0) |
| | 2 | Analog Input Channel 2 | `int16` | `ro` | **TPDO2** | AI1 (0-10V -> ADC1_IN10) |
| | 3 | Analog Input Channel 3 | `int16` | `ro` | **TPDO2** | AI2 (0-10V -> ADC1_IN12) |
| | 4 | Analog Input Channel 4 | `int16` | `ro` | **TPDO2** | AI3 (0-10V -> ADC1_IN13) |
| `0x6411` | 0 | Number of 16-bit AO channels | `uint8` | `ro` | `2` | - |
| | 1 | Analog Output Channel 1 | `int16` | `rw` | **RPDO2** | AO0 (DAC_OUT1 -> 0-10V Op-Amp) |
| | 2 | Analog Output Channel 2 | `int16` | `rw` | **RPDO2** | AO1 (DAC_OUT2 -> 0-10V Op-Amp) |

---

## 4. Hardware Driver (bxCAN) & Clock Tree Setup

### 4.1 Bit Timing Calculation (500 kbps)

Both STM32F407 and STM32F767 utilize the ST bxCAN controller attached to the **APB1 peripheral bus**.

$$\text{Bitrate} = \frac{f_{\text{APB1}}}{\text{Prescaler} \times (1 + \text{BS1} + \text{BS2})}$$

$$\text{Sample Point} = \frac{1 + \text{BS1}}{1 + \text{BS1} + \text{BS2}}$$

| Target MCU | \(f_{\text{SYSCLK}}\) | \(f_{\text{APB1}}\) | Prescaler | TimeSeg1 (BS1) | TimeSeg2 (BS2) | Total \(T_q\) | Bit Rate | Sample Point |
|---|---|---|---|---|---|---|---|---|
| **STM32F407** | 168 MHz | 42 MHz | **6** | 10 \(T_q\) | 3 \(T_q\) | 14 \(T_q\) | **500 kbps** | **78.57%** |
| **STM32F767** | 216 MHz | 54 MHz | **6** | 13 \(T_q\) | 4 \(T_q\) | 18 \(T_q\) | **500 kbps** | **77.78%** |

*Note: CiA 301 recommends a sample point between 75.0% and 87.5%. Both target configurations comply.*

### 4.2 GPIO Alternate Function Pin Mapping

- **STM32F407 (Automation Board)**:
  - CAN1_RX: `PA11` (AF9)
  - CAN1_TX: `PA12` (AF9)
  - *Zero pin conflicts verified via `scripts/check_pin_conflicts.py`.*
- **STM32F767 (Nucleo-144 / Custom)**:
  - Default: CAN1_RX `PD0` (AF9), CAN1_TX `PD1` (AF9).
  - Alternate: CAN1_RX `PB8` (AF9), CAN1_TX `PB9` (AF9).

---

## 5. Comparison against Open-Source Reference Stacks

| Metric / Feature | **Our Implementation** | **CanFestival (objdictgen)** | **CANopenNode** | **Easy-CANopen** |
|---|---|---|---|---|
| **objdictgen Compatible** | **Native 1:1** (`subindex`, `indextable`) | Native (source format) | Requires OD conversion | Incompatible format |
| **Memory Allocation** | **100% Static** (Zero malloc) | Configurable / malloc | Static (CO_OD) | Static |
| **Hardware Coupling** | **Fully Decoupled** (weak hooks + function pointers) | Driver-coupled | Driver-coupled | Highly coupled |
| **CiA 401 Automation I/O** | **Built-in** (8xDI, 8xDO, 4xAI, 2xAO) | Manual OD setup | Manual OD setup | Incomplete |
| **STM32F4 & F7 Support** | **Turnkey** (500k timing for 42M & 54M) | Manual porting needed | Contrib drivers | Partial |
| **Native Host Test Suite** | **16 Unit Tests** (100% automated) | Difficult to mock | Unit tests present | None |

---

## 6. Verification and Test Results

The stack includes a comprehensive native test harness in [`tests/test_canopen.c`](file:///c:/Users/MAHDI/Desktop/Agent_AI/stm32_automation_board/tests/test_canopen.c). All 16 automated tests execute in &lt; 0.05s on host:

1. `test_nmt_init_and_bootup`: Verifies CiA 301 Boot-up frame generation (`0x700 + ID`, data `0x00`).
2. `test_nmt_state_transitions`: Verifies Start (0x01), Stop (0x02), Pre-op (0x80), and Reset (0x81).
3. `test_heartbeat_producer`: Verifies periodic countdown and state encoding (`0x04`, `0x05`, `0x7F`).
4. `test_sdo_expedited_read`: Reads 0x1000, 0x1017, 0x1018 subindices with correct SCS/size flags.
5. `test_sdo_expedited_write`: Writes 0x6200 and 0x6411 and verifies positive response.
6. `test_sdo_abort_codes`: Verifies aborts on missing objects, missing subindices, and RO write violations.
7. `test_sdo_segmented_read_string`: Reads 22-byte device name across 4 segments with toggle validation.
8. `test_rpdo_operational_gating`: Verifies RPDO rejection in Pre-Op and acceptance in Operational state.
9. `test_tpdo_transmission_and_sync`: Verifies TPDO packaging, inhibit cooldown, and SYNC frame triggers.
10. `test_canfestival_objdictgen_compat`: Tests custom `indextable` and `subindex` generated by `gen_cfile.py`.
11. `test_sdo_segmented_toggle_error`: Validates abort `0x05030000` on toggle mismatch.
12. `test_sdo_stopped_state_gating`: Enforces SDO rejection when in STOPPED state.
13. `test_sdo_client_abort`: Verifies immediate transfer teardown on client abort (`0x80`).
14. `test_od_post_write_callback`: Validates dynamic write notifications.
15. `test_cia401_analog_io_sdo_access`: Tests signed 16-bit analog input/output scaling.
16. `test_node_id_reconfiguration`: Re-allocates node ID and verifies all COB-ID recalculations.
