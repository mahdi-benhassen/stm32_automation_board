# STM32 Automation Board — User Manual

## Table of Contents

1. [Board Overview](#1-board-overview)
2. [Physical Connections](#2-physical-connections)
3. [Power-Up Sequence](#3-power-up-sequence)
4. [Network Configuration](#4-network-configuration)
5. [Modbus Communication](#5-modbus-communication)
6. [Register Map](#6-register-map)
7. [Digital Inputs](#7-digital-inputs)
8. [Digital Outputs](#8-digital-outputs)
9. [Analog Inputs](#9-analog-inputs)
10. [Analog Outputs](#10-analog-outputs)
11. [Relay Control](#11-relay-control)
12. [Practical Examples](#12-practical-examples)
13. [LED Indicators](#13-led-indicators)
14. [Troubleshooting](#14-troubleshooting)
15. [Electrical Specifications](#15-electrical-specifications)

---

## 1. Board Overview

The STM32 Automation Board is an industrial controller based on the STM32F407 microcontroller (ARM Cortex-M4, 168 MHz). It provides:

| Interface         | Channels | Type                                 |
|-------------------|----------|--------------------------------------|
| Digital Inputs    | 8        | 24V opto-isolated, sinking           |
| Digital Outputs   | 8        | 24V high-side, push-pull             |
| Analog Inputs     | 4        | 0–10V / 4–20mA (12-bit ADC)          |
| Analog Outputs    | 2        | 0–10V (12-bit DAC)                   |
| Relays            | 4        | NO contacts, 250VAC/30VDC 5A         |
| RS485             | 1        | Modbus RTU, half-duplex              |
| Ethernet          | 1        | Modbus TCP, 10/100M RMII             |

The board acts as a **Modbus slave/server** on both RS485 and Ethernet simultaneously.

---

## 2. Physical Connections

### Power Supply

| Pin  | Label | Voltage  | Notes                   |
|------|-------|----------|-------------------------|
| 1    | +24V  | 24V DC   | Main power input        |
| 2    | GND   | 0V       | Power ground            |
| 3    | +3.3V | 3.3V DC  | MCU core (internal reg) |

### Digital Inputs (Terminal Block DI)

```
DI0 DI1 DI2 DI3 DI4 DI5 DI6 DI7 COM
 │   │   │   │   │   │   │   │   │
 └───┴───┴───┴───┴───┴───┴───┴───┴── GND (common)
```

Each input is opto-isolated with internal pull-up. Connect the input pin to **GND** to activate (logic 1). Inputs are active-low.

**Wiring a push button:**
```
Push button ─┬─ DI0
             │
             └─ COM (GND)
```
Press → DI0 reads 1. Release → DI0 reads 0.

**Wiring a 24V sensor (NPN sinking):**
```
24V ──── Sensor ──── DI0
                    COM ──── 0V
```
Sensor ON → DI0 pulled to GND → logic 1.

### Digital Outputs (Terminal Block DO)

```
DO0 DO1 DO2 DO3 DO4 DO5 DO6 DO7 COM
 │   │   │   │   │   │   │   │   │
 └───┴───┴───┴───┴───┴───┴───┴───┴── +24V (common)
```

Outputs switch +24V. Max current: 100 mA per channel.

**Wiring a 24V indicator lamp:**
```
DO0 ──── Lamp ──── 0V
```
DO0 ON → 24V at output → lamp illuminates.

### Analog Inputs (Terminal Block AI)

```
AI0 AI1 AI2 AI3 AGND
 │   │   │   │   │
 └───┴───┴───┴───┴── Analog Ground
```

**0–10V sensor wiring:**
```
Sensor Vout ──── AI0
Sensor GND ──── AGND
```

**4–20mA sensor wiring (requires external 250Ω resistor):**
```
                         ┌─ AI0
24V ──── Sensor ────┬───┤
                    │   └─ 250Ω ──── AGND
                    │
                    └───────────────┘
```
Voltage across 250Ω: 1V at 4mA, 5V at 20mA. Scaled internally.

### Analog Outputs (Terminal Block AO)

```
AO0 AO1 AGND
 │   │   │
 └───┴───┴── Analog Ground
```

0–10V output, max 10 mA. Connect to VFD speed reference, valve positioner, etc.

### Relays (Terminal Block RLY)

```
RLY1_NO RLY1_COM RLY2_NO RLY2_COM RLY3_NO RLY3_COM RLY4_NO RLY4_COM
   │       │        │       │        │       │        │       │
   └───┬───┘        └───┬───┘        └───┬───┘        └───┬───┘
   Normally Open      Normally Open   Normally Open   Normally Open
```

Each relay provides a dry contact (NO-COM). Rating: 250VAC/5A or 30VDC/5A.

### RS485 (Terminal Block)

```
A (D+)  B (D-)  GND
 │       │       │
 └───────┴───────┴── Shield/Common
```

Connect A to A, B to B across all RS485 devices. Use 120Ω termination resistor at both ends of a long bus.

### Ethernet (RJ45)

Standard 10/100M Ethernet. Connect to a switch or directly to a PC with a crossover cable.

**Default IP:** `192.168.1.100`  
**Subnet mask:** `255.255.255.0`  
**Gateway:** `192.168.1.1`

---

## 3. Power-Up Sequence

1. Apply 24V DC power
2. **Power LED** illuminates (solid green)
3. Board initializes all peripherals (~100 ms)
4. **Status LED** blinks at 1 Hz (heartbeat)
5. Ethernet link LED on RJ45 lights up when cable connected
6. Board is ready for Modbus communication

```
Power ON
   │
   ▼
SystemInit() → FPU enable, vector table
   │
   ▼
system_clock_config() → HSE 8MHz → PLL → 168MHz
   │
   ▼
HAL_Init() → SysTick 1ms
   │
   ▼
All peripherals init (GPIO, ADC, DAC, USART2, ETH)
   │
   ▼
Modbus stacks init (RTU slave 1, TCP unit 1)
   │
   ▼
FreeRTOS scheduler starts
   │
   ├── IO_Scan task (100 Hz, scans inputs)
   ├── ModbusRTU task (event-driven, RS485)
   └── ModbusTCP task (event-driven, Ethernet)
```

---

## 4. Network Configuration

### Changing IP Address

Edit `inc/board_config.h` before building:

```c
#define IP_ADDR0    192
#define IP_ADDR1    168
#define IP_ADDR2    1
#define IP_ADDR3    100      // ← change this

#define GATEWAY0    192
#define GATEWAY1    168
#define GATEWAY2    1
#define GATEWAY3    1
```

### Changing RS485 Baud Rate

```c
#define RS485_BAUDRATE      115200U    // Options: 9600, 19200, 38400, 57600, 115200
```

### Changing Modbus Slave ID

```c
#define MODBUS_SLAVE_ID         1      // Used for TCP (unit identifier)
#define MODBUS_RTU_ADDRESS      1      // Used for RTU (slave address)
```

After changing, rebuild the firmware:
```bash
cmake --build build -j$(nproc)
```

---

## 5. Modbus Communication

The board responds to **Modbus RTU** (RS485) and **Modbus TCP** (Ethernet) on the same register map.

### RS485 (Modbus RTU)

| Parameter    | Value              |
|--------------|--------------------|
| Baud rate    | 115200 (default)   |
| Data bits    | 8                  |
| Stop bits    | 1                  |
| Parity       | Even (8E1)         |
| Slave ID     | 1 (default)        |
| Connector    | Terminal block A/B |

**Test with `mbpoll` (Linux):**
```bash
# Read 8 coils (digital outputs status)
mbpoll -a 1 -b 115200 -p e -d 8 -s 1 -t 0 -r 1 -c 8 /dev/ttyUSB0

# Read 8 discrete inputs
mbpoll -a 1 -b 115200 -p e -d 8 -s 1 -t 1 -r 1 -c 8 /dev/ttyUSB0

# Read 4 input registers (analog inputs)
mbpoll -a 1 -b 115200 -p e -d 8 -s 1 -t 3 -r 1 -c 4 /dev/ttyUSB0

# Write single coil (turn ON DO0)
mbpoll -a 1 -b 115200 -p e -d 8 -s 1 -t 0 -r 1 /dev/ttyUSB0 1

# Write single register (set AO0 to 2047 ≈ 5.0V)
mbpoll -a 1 -b 115200 -p e -d 8 -s 1 -t 4 -r 1 /dev/ttyUSB0 2047
```

### Ethernet (Modbus TCP)

> **Note:** Modbus TCP frame processing (MBAP header strip/wrap) is implemented. However, a full TCP/IP stack (e.g., lwIP) must be integrated for complete ARP/IP/TCP protocol handling. Currently, Modbus communication is fully functional over RS485 (RTU). The Modbus TCP code path is ready for use once lwIP or an equivalent stack is added.

| Parameter    | Value                  |
|--------------|------------------------|
| IP address   | 192.168.1.100 (default)|
| Port         | 502                    |
| Unit ID      | 1 (default)            |
| Connector    | RJ45                   |

**Test with `mbpoll` (Linux):**
```bash
# Read coils via TCP
mbpoll -a 1 -t 0 -r 1 -c 8 192.168.1.100

# Read input registers via TCP
mbpoll -a 1 -t 3 -r 1 -c 4 192.168.1.100

# Write holding register via TCP
mbpoll -a 1 -t 4 -r 1 192.168.1.100 2047
```

**Test with Python (`pymodbus`):**
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient("192.168.1.100", port=502)
client.connect()

# Read coils (digital outputs)
result = client.read_coils(0, 8)
print(f"DO states: {result.bits}")

# Write coil (turn ON DO2)
client.write_coil(2, True)

# Read input registers (analog inputs)
result = client.read_input_registers(0, 4)
for i, val in enumerate(result.registers):
    voltage = (val / 4095.0) * 10.0
    print(f"AI{i}: {val} raw = {voltage:.2f}V")

# Write holding register (set AO0 to 5V)
client.write_register(0, 2047)

client.close()
```

### Supported Function Codes

| FC  | Name                       | Used For                                  |
|-----|----------------------------|-------------------------------------------|
| 0x01| Read Coils                 | Read DO status (0–7), relay status (8–11) |
| 0x02| Read Discrete Inputs       | Read DI status (0–7)                      |
| 0x03| Read Holding Registers     | Read AO values, general-purpose registers |
| 0x04| Read Input Registers       | Read AI raw ADC values (0–3)              |
| 0x05| Write Single Coil          | Turn ON/OFF one DO or relay               |
| 0x06| Write Single Register      | Set one AO value                          |
| 0x0F| Write Multiple Coils       | Set multiple DOs/relays at once           |
| 0x10| Write Multiple Registers   | Set multiple AO values at once            |

---

## 6. Register Map

### Coils (FC 0x01, 0x05, 0x0F) — Read/Write

| Address | Label    | Hardware       | Values                 |
|---------|----------|----------------|------------------------|
| 0       | DO0      | PB0 output     | 0 = OFF, 1 = ON        |
| 1       | DO1      | PB1 output     | 0 = OFF, 1 = ON        |
| 2       | DO2      | PB5 output     | 0 = OFF, 1 = ON        |
| 3       | DO3      | PB6 output     | 0 = OFF, 1 = ON        |
| 4       | DO4      | PB8 output     | 0 = OFF, 1 = ON        |
| 5       | DO5      | PB9 output     | 0 = OFF, 1 = ON        |
| 6       | DO6      | PB10 output    | 0 = OFF, 1 = ON        |
| 7       | DO7      | PB14 output    | 0 = OFF, 1 = ON        |
| 8       | RELAY1   | PC8 + PD0 LED  | 0 = OFF, 1 = ON        |
| 9       | RELAY2   | PC9 + PD1 LED  | 0 = OFF, 1 = ON        |
| 10      | RELAY3   | PC10 + PD2 LED | 0 = OFF, 1 = ON        |
| 11      | RELAY4   | PC11 + PD3 LED | 0 = OFF, 1 = ON        |

### Discrete Inputs (FC 0x02) — Read-Only

| Address | Label    | Hardware       | Values                 |
|---------|----------|----------------|------------------------|
| 0       | DI0      | PE0 input      | 0 = open, 1 = closed   |
| 1       | DI1      | PE1 input      | 0 = open, 1 = closed   |
| 2       | DI2      | PE2 input      | 0 = open, 1 = closed   |
| 3       | DI3      | PE3 input      | 0 = open, 1 = closed   |
| 4       | DI4      | PE4 input      | 0 = open, 1 = closed   |
| 5       | DI5      | PE5 input      | 0 = open, 1 = closed   |
| 6       | DI6      | PE6 input      | 0 = open, 1 = closed   |
| 7       | DI7      | PE7 input      | 0 = open, 1 = closed   |

### Input Registers (FC 0x04) — Read-Only

| Address | Label    | Source               | Range   | Scaled Value         |
|---------|----------|----------------------|---------|----------------------|
| 0       | AI0      | PA0, ADC1_IN0        | 0–4095  | 0 = 0V, 4095 = 10V   |
| 1       | AI1      | PC0, ADC1_IN10       | 0–4095  | 0 = 0V, 4095 = 10V   |
| 2       | AI2      | PC2, ADC1_IN12       | 0–4095  | 0 = 0V, 4095 = 10V   |
| 3       | AI3      | PC3, ADC1_IN13       | 0–4095  | 0 = 0V, 4095 = 10V   |

### Holding Registers (FC 0x03, 0x06, 0x10) — Read/Write

| Address | Label    | Hardware            | Range   | Scaled Value          |
|---------|----------|---------------------|---------|-----------------------|
| 0       | AO0      | PA4, DAC_OUT1       | 0–4095  | 0 = 0V, 4095 = 10V    |
| 1       | AO1      | PA5, DAC_OUT2       | 0–4095  | 0 = 0V, 4095 = 10V    |
| 100     | AI0_copy | Mirror of input reg | 0–4095  | Read-only mirror of AI0 |
| 101     | AI1_copy | Mirror of input reg | 0–4095  | Read-only mirror of AI1 |
| 102     | AI2_copy | Mirror of input reg | 0–4095  | Read-only mirror of AI2 |
| 103     | AI3_copy | Mirror of input reg | 0–4095  | Read-only mirror of AI3 |

---

## 7. Digital Inputs

### How They Work

Digital inputs are scanned every **10 ms** by the IO_Scan FreeRTOS task. Each input has a **6-sample software debounce** — the input must remain stable for 6 consecutive samples (~60 ms) before the state changes.

```
GPIOE IDR register
    │
    ▼ (every 10ms)
digital_inputs_scan()
    │
    ├── read PE0 → if LOW for 5 samples → DI0 = 1
    ├── read PE1 → if LOW for 5 samples → DI1 = 1
    ├── ...
    └── read PE7 → if LOW for 5 samples → DI7 = 1
    │
    ▼
modbus_sync_registers()
    │ writes to discrete_bits[0-7]
    │
    ▼
Available via FC 0x02 (Read Discrete Inputs)
```

### Reading via Modbus

**Read all 8 DI states (FC 0x02):**
```
Request:  [01][02][0000][0008][CRC]
Response: [01][02][01][XX][CRC]
```
The byte value represents bits: bit0=DI0, bit1=DI1, ..., bit7=DI7.

### Conversion to Physical State

| DI State | GPIOE Pin Level | Physical Meaning         |
|----------|-----------------|--------------------------|
| 0        | HIGH (3.3V)     | Input open / sensor OFF  |
| 1        | LOW (0V)        | Input closed / sensor ON |

---

## 8. Digital Outputs

### How They Work

Writing a coil via Modbus immediately updates the output pin through `modbus_sync_registers()`:

```
Modbus FC 0x05/0x0F (Write Coil)
    │
    ▼
modbus_write_coil() → coil_bits[addr] = value
    │
    ▼
modbus_sync_registers()
    │
    ▼
digital_output_write(channel, state)
    │
    ├── state=1 → GPIOB->BSRR = pin    (set HIGH → 24V output)
    └── state=0 → GPIOB->BSRR = pin<<16 (reset LOW → 0V output)
```

### Writing via Modbus

**Turn ON DO0 (FC 0x05):**
```
Request:  [01][05][0000][FF00][CRC]     addr=0, value=0xFF00 (ON)
Response: [01][05][0000][FF00][CRC]     echo
```

**Turn OFF DO3 (FC 0x05):**
```
Request:  [01][05][0003][0000][CRC]     addr=3, value=0x0000 (OFF)
Response: [01][05][0003][0000][CRC]     echo
```

**Turn ON DO0, DO1, DO2 simultaneously (FC 0x0F):**
```
Request:  [01][0F][0000][0003][01][07][CRC]
          │    │    │    │    │  │  └── byte: bits 0,1,2 = 1
          │    │    │    │    │  └──── byte_count = 1
          │    │    │    │    └─────── quantity = 3
          │    │    │    └──────────── start_addr = 0
          │    │    └───────────────── Write Multiple Coils
          │    └────────────────────── slave ID = 1
          └─────────────────────────── FC
Response: [01][0F][0000][0003][CRC]
```

---

## 9. Analog Inputs

### How They Work

ADC1 scans all 4 channels every **10 ms**. The raw 12-bit value (0–4095) is stored in holding registers 100–103 and mirrored to input registers 0–3.

```
PA0 (AI0) ──► ADC1_IN0
PC0 (AI1) ──► ADC1_IN10     ┌────────────────────┐
PC2 (AI2) ──► ADC1_IN12 ───►│ 12-bit ADC (0-4095)│
PC3 (AI3) ──► ADC1_IN13     └────────────────────┘
                                      │
                                      ▼
                             holding_regs[100-103]
                             input_regs[0-3]
```

### Voltage Conversion

```
V_input = (raw_value / 4095.0) × 10.0V
```

| Raw Value | Voltage at Terminal | Typical Use |
|-----------|---------------------|-------------|
| 0         | 0.00 V              | Sensor at minimum |
| 1024      | 2.50 V              | |
| 2047      | 5.00 V              | Mid-range |
| 3071      | 7.50 V              | |
| 4095      | 10.00 V             | Sensor at maximum |

### Reading via Modbus

**Read AI0 (FC 0x04, addr 0):**
```
Request:  [01][04][0000][0001][CRC]
Response: [01][04][02][07][FF][CRC]
```
Raw value = 0x07FF = 2047 → voltage = 2047/4095 × 10 = 5.00V

### 4–20mA Conversion

With an external 250Ω precision resistor:
```
I_loop = (V_measured / 250Ω)
       = ((raw / 4095) × 10V) / 250Ω
       = (raw × 10) / (4095 × 250) amperes

In mA: I_mA = (raw × 10000) / (4095 × 250)
```

| Raw Value | Voltage | Current (250Ω) |
|-----------|---------|----------------|
| 410       | 1.00V   | 4 mA           |
| 2047      | 5.00V   | 20 mA          |

---

## 10. Analog Outputs

### How They Work

Writing a holding register via Modbus sets the DAC output voltage:

```
Modbus FC 0x06/0x10 (Write Holding Register)
    │
    ▼
holding_regs[addr] = value
    │
    ▼
modbus_sync_registers()
    │
    ▼
analog_output_write_raw(channel, value)
    │
    ▼
HAL_DAC_SetValue(DAC, DAC_CHANNEL_X, DAC_ALIGN_12B_R, value)
    │
    ▼
PA4/PA5 → internal op-amp ×3 → 0–10V at terminal
```

### Setting Voltage via Modbus

**Set AO0 to 5.0V (FC 0x06):**
```
raw_value = (5.0 / 10.0) × 4095 = 2047 = 0x07FF

Request:  [01][06][0000][07FF][CRC]     holding reg 0 = 2047
Response: [01][06][0000][07FF][CRC]     echo
```

**Common setpoints:**

| Desired Output | Raw Value (decimal) | Raw Value (hex) |
|----------------|---------------------|-----------------|
| 0.0 V          | 0                   | 0x0000          |
| 2.5 V          | 1023                | 0x03FF          |
| 5.0 V          | 2047                | 0x07FF          |
| 7.5 V          | 3071                | 0x0BFF          |
| 10.0 V         | 4095                | 0x0FFF          |

**Set both AO0 and AO1 simultaneously (FC 0x10):**
```
AO0 = 5.0V (2047 = 0x07FF), AO1 = 2.5V (1023 = 0x03FF)

Request: [01][10][0000][0002][04][07][FF][03][FF][CRC]
          │    │    │    │    │  └── AO0 hi/lo ─┘ └── AO1 hi/lo
          │    │    │    │    └──── byte_count = 4
          │    │    │    └───────── quantity = 2 registers
          │    │    └────────────── start_addr = 0
Response: [01][10][0000][0002][CRC]
```

---

## 11. Relay Control

### How They Work

Each relay is controlled as a Modbus coil (addresses 8–11). When activated, both the relay coil energizes AND the corresponding LED illuminates.

```
Coil 8 (RELAY1) ON
    │
    ├── GPIOC->BSRR = GPIO_PIN_8    → 3.3V → transistor → relay coil → NO-COM closes
    │
    └── GPIOD->BSRR = GPIO_PIN_0    → 3.3V → LED1 ON (red)
```

**Relay protection:** In case of power loss, all relays return to OFF (normally open).

### Controlling via Modbus

**Turn ON Relay 3 (FC 0x05, addr 10):**
```
Request:  [01][05][000A][FF00][CRC]     coil 10 = ON
Response: [01][05][000A][FF00][CRC]     echo
```
Result: Green LED 3 illuminates, NO-COM contact closes.

**Turn ON Relays 1+4, OFF Relays 2+3 (FC 0x0F):**
```
coil 8=1, coil 9=0, coil 10=0, coil 11=1 → byte = 0b00001001 = 0x09

Request: [01][0F][0008][0004][01][09][CRC]
Response: [01][0F][0008][0004][CRC]
```

### Relay LED Indicators

| Relay | Coil Pin | LED Pin | LED Color | Meaning          |
|-------|----------|---------|-----------|------------------|
| 1     | PC8      | PD0     | Red       | ON when energized |
| 2     | PC9      | PD1     | Red       | ON when energized |
| 3     | PC10     | PD2     | Red       | ON when energized |
| 4     | PC11     | PD3     | Red       | ON when energized |

---

## 12. Practical Examples

### Example A: Read All Inputs, Control One Output

**Scenario:** Read DI0-DI3 (limit switches), read AI0 (pressure sensor 0-10V = 0-10 bar), turn ON DO0 if pressure > 5 bar and limit switch DI0 is closed.

```python
from pymodbus.client import ModbusTcpClient
import time

client = ModbusTcpClient("192.168.1.100")
client.connect()

while True:
    # Read digital inputs
    di = client.read_discrete_inputs(0, 4)
    di0 = di.bits[0]  # limit switch

    # Read analog input (pressure sensor)
    ai = client.read_input_registers(0, 1)
    raw = ai.registers[0]
    pressure_bar = (raw / 4095.0) * 10.0
    
    print(f"DI0={di0}, Pressure={pressure_bar:.1f} bar")
    
    # Control decision
    if di0 and pressure_bar > 5.0:
        client.write_coil(0, True)   # DO0 ON
    else:
        client.write_coil(0, False)  # DO0 OFF
    
    time.sleep(0.1)
```

### Example B: Ramp Analog Output (Motor Speed Control)

**Scenario:** Slowly ramp AO0 from 0V to 10V over 10 seconds, then back down.

```python
import time

client = ModbusTcpClient("192.168.1.100")
client.connect()

# Ramp up
for raw in range(0, 4096, 40):
    client.write_register(0, raw)
    time.sleep(0.1)  # 100ms per step → ~10s total

# Ramp down
for raw in range(4095, -1, -40):
    client.write_register(0, raw)
    time.sleep(0.1)
```

### Example C: Sequence Control with Relays

**Scenario:** Start motor 1 (Relay 1), wait 2 seconds, start motor 2 (Relay 2), wait for DI0 (motor running feedback), then start motor 3 (Relay 3).

```python
import time

def write_relay(client, num, state):
    client.write_coil(7 + num, state)  # coils 8-11

write_relay(client, 1, True)           # Start motor 1
time.sleep(2)

write_relay(client, 2, True)           # Start motor 2

# Wait for feedback
while True:
    di = client.read_discrete_inputs(0, 1)
    if di.bits[0]:                     # DI0 = motor running
        break
    time.sleep(0.1)

write_relay(client, 3, True)           # Start motor 3
```

### Example D: Monitoring via Modbus RTU (RS485)

**Scenario:** Read all inputs and outputs continuously via RS485 for a dashboard.

```python
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(method="rtu", port="/dev/ttyUSB0",
                            baudrate=115200, parity="E",
                            stopbits=1, bytesize=8)
client.connect()

while True:
    # Read inputs
    di = client.read_discrete_inputs(0, 8)
    ai = client.read_input_registers(0, 4)
    
    # Read outputs
    do = client.read_coils(0, 8)
    relay = client.read_coils(8, 4)
    ao = client.read_holding_registers(0, 2)
    
    print(f"DI: {di.bits}")
    print(f"AI: {[f'{v/4095*10:.1f}V' for v in ai.registers]}")
    print(f"DO: {do.bits}")
    print(f"Relays: {relay.bits}")
    print(f"AO: {[f'{v/4095*10:.1f}V' for v in ao.registers]}")
    print("---")
    
    time.sleep(0.5)
```

---

## 13. LED Indicators

| LED         | Color  | Meaning                                      |
|-------------|--------|----------------------------------------------|
| Power       | Green  | Board powered (3.3V regulator output)         |
| Status      | Yellow | Firmware heartbeat (toggles every 500ms)     |
| Relay 1     | Red    | ON when Relay 1 energized                     |
| Relay 2     | Red    | ON when Relay 2 energized                     |
| Relay 3     | Red    | ON when Relay 3 energized                     |
| Relay 4     | Red    | ON when Relay 4 energized                     |
| ETH Link    | Green  | ON when Ethernet cable connected (on RJ45)    |
| ETH Activity| Yellow | Blinks on Ethernet TX/RX (on RJ45)           |

---

## 14. Troubleshooting

### No Modbus Response (RS485)

| Symptom                          | Possible Cause              | Solution                                   |
|----------------------------------|-----------------------------|--------------------------------------------|
| No response at all               | Wrong baud rate             | Verify 115200 8N1                          |
|                                   | Slave ID mismatch           | Default is 1, check MODBUS_RTU_ADDRESS     |
|                                   | A/B wires swapped           | Swap A and B at terminal                   |
|                                   | Missing GND connection      | Connect RS485 GND between devices          |
| CRC errors                       | Noisy bus / long cable      | Add 120Ω termination at both ends          |
| Response with wrong data         | Register address off-by-1   | Modbus addressing: coil 0 = addr 0         |

### No Modbus Response (TCP)

| Symptom                          | Possible Cause              | Solution                                   |
|----------------------------------|-----------------------------|--------------------------------------------|
| Connection refused               | Wrong IP                    | Default 192.168.1.100; check with ping     |
|                                   | Wrong port                  | Default 502                                |
|                                   | Ethernet not linked         | Check cable, check ETH Link LED on RJ45    |
| Connection timeout               | Firewall blocking port 502  | Allow TCP port 502                         |
|                                   | IP on different subnet      | Ensure PC IP is 192.168.1.x                |

### Analog Input Reads 0

| Symptom                          | Possible Cause              | Solution                                   |
|----------------------------------|-----------------------------|--------------------------------------------|
| AI always 0                      | No sensor connected         | Verify wiring to AI pin and AGND           |
|                                   | Sensor output type mismatch | 4-20mA sensor needs 250Ω resistor to AGND  |
| AI value unstable                | Noisy signal                | Add 0.1µF capacitor between AI and AGND    |

### Relay Not Switching

| Symptom                          | Possible Cause              | Solution                                   |
|----------------------------------|-----------------------------|--------------------------------------------|
| LED on but no contact closure    | Faulty relay                | Replace relay                              |
| No LED, no click                 | Coil not driven             | Verify Modbus write to coil 8-11          |
|                                   | Power issue                 | Check 24V supply to relay driver circuit   |

### Digital Output Always OFF

| Symptom                          | Possible Cause              | Solution                                   |
|----------------------------------|-----------------------------|--------------------------------------------|
| DO stays LOW                     | Load exceeds 100mA          | Add external relay for high-current loads  |
|                                   | Short circuit               | Disconnect load, test with multimeter      |

---

## 15. Electrical Specifications

| Parameter                  | Min    | Typical | Max    | Unit  |
|----------------------------|--------|---------|--------|-------|
| Supply voltage             | 18     | 24      | 30     | V DC  |
| Supply current (idle)      | —      | 120     | 200    | mA    |
| Digital input threshold    | —      | —       | —      | —     |
|  - Logic 0 (open)          | 10     | —       | 30     | V     |
|  - Logic 1 (closed to GND) | 0      | —       | 3      | V     |
| Digital output max current | —      | —       | 100    | mA    |
| Analog input range         | 0      | —       | 10     | V     |
| Analog input impedance     | —      | 100     | —      | kΩ    |
| Analog output range        | 0      | —       | 10     | V     |
| Analog output max current  | —      | —       | 10     | mA    |
| Relay contact rating       | —      | —       | 250/30 | VAC/VDC|
| Relay contact current      | —      | —       | 5      | A     |
| RS485 bus voltage          | -7     | —       | +12    | V     |
| Operating temperature      | -20    | —       | +70    | °C    |

---

*Document version: 1.0 — Compatible with firmware v1.0+*
