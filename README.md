# STM32 Automation Board

Industrial automation controller based on STM32F407 with Ethernet, RS485, and Modbus protocol support.

## Features

- **8 Digital Inputs** (opto-isolated, active-low, with software debouncing)
- **8 Digital Outputs** (push-pull, high-side drive)
- **4 Analog Inputs** (12-bit ADC, 0-10V range)
- **2 Analog Outputs** (12-bit DAC, 0-10V range)
- **4 Relays** with individual LED status indicators
- **RS485** half-duplex interface with DE/RE direction control
- **Ethernet** 10/100M RMII with built-in MAC (external PHY: LAN8720/DP83848)
- **Modbus RTU** (slave) over RS485
- **Modbus TCP** (server) over Ethernet, port 502

## Modbus Register Map

| Type               | Address Range | Description               |
|---------------------|---------------|---------------------------|
| Coils (0x)          | 0-7           | Digital outputs DO0-DO7   |
| Coils (0x)          | 8-11          | Relays 1-4                |
| Discrete Inputs (1x)| 0-7           | Digital inputs DI0-DI7    |
| Input Registers (3x)| 0-3           | Analog inputs (raw ADC)   |
| Holding Registers (4x)| 0-1         | Analog outputs (raw DAC)  |
| Holding Registers (4x)| 100-103     | Analog inputs (mirrored)  |

## Building

### Prerequisites

- ARM GCC Toolchain (`arm-none-eabi-gcc` 12.3+)
- CMake 3.20+
- STM32CubeF4 HAL library (downloaded automatically in CI)

### Local Build

```bash
git clone https://github.com/your-org/stm32_automation_board.git
cd stm32_automation_board

# Download STM32CubeF4 HAL
./scripts/fetch_hal.sh

# Build
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### CI/CD

GitHub Actions automatically builds on push/PR to `main`/`master`. 
Tags starting with `v` trigger a release with firmware artifacts.

## Pin Assignments

### Digital I/O
| Signal | Pin  | Signal | Pin  |
|--------|------|--------|------|
| DI0    | PE0  | DO0    | PB0  |
| DI1    | PE1  | DO1    | PB1  |
| DI2    | PE2  | DO2    | PB5  |
| DI3    | PE3  | DO3    | PB6  |
| DI4    | PE4  | DO4    | PB8  |
| DI5    | PE5  | DO5    | PB9  |
| DI6    | PE6  | DO6    | PB10 |
| DI7    | PE7  | DO7    | PB12 |

### Analog I/O
| Signal | Pin     | ADC Channel  |
|--------|---------|-------------|
| AI0    | PA0     | ADC1_IN0    |
| AI1    | PC0     | ADC1_IN10   |
| AI2    | PC2     | ADC1_IN12   |
| AI3    | PC3     | ADC1_IN13   |
| AO0    | PA4     | DAC_OUT1    |
| AO1    | PA5     | DAC_OUT2    |

### Relays
| Relay | Coil Pin | LED Pin |
|-------|----------|---------|
| 1     | PC8      | PD0     |
| 2     | PC9      | PD1     |
| 3     | PC10     | PD2     |
| 4     | PC11     | PD3     |

### Communication
| Signal   | Pin  | Function        |
|----------|------|-----------------|
| RS485 TX | PD5  | USART2_TX       |
| RS485 RX | PD6  | USART2_RX       |
| RS485 DE | PD7  | Direction ctrl  |

### Ethernet (RMII)
| Signal      | Pin  |
|-------------|------|
| REF_CLK     | PA1  |
| MDIO        | PA2  |
| CRS_DV      | PA7  |
| MDC         | PC1  |
| RXD0        | PC4  |
| RXD1        | PC5  |
| TX_EN       | PB11 |
| TXD0        | PB12 |
| TXD1        | PB13 |

## Flashing

### Option 1: STM32CubeProgrammer (CLI)

```bash
# Download from https://www.st.com/en/development-tools/stm32cubeprog.html

# Flash via ST-Link
STM32_Programmer_CLI -c port=SWD -w build/stm32_automation_board.hex -v

# Flash via UART (bootloader)
STM32_Programmer_CLI -c port=COM3 br=115200 -w build/stm32_automation_board.hex -v
```

### Option 2: OpenOCD + ST-Link

```bash
# Install OpenOCD (Ubuntu/Debian)
sudo apt-get install openocd

# Connect ST-Link and flash
openocd -f openocd.cfg -c "program build/stm32_automation_board.hex verify reset exit"
```

### Option 3: STM32 ST-LINK Utility (Windows)

1. Download [ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html)
2. Connect ST-Link to the board
3. Target → Connect
4. Target → Program & Verify → select `build/stm32_automation_board.hex`

### Entering Bootloader (for UART flash)

1. Set BOOT0 pin HIGH, BOOT1 pin LOW
2. Reset the MCU
3. Connect USART1 (PA9=TX, PA10=RX) via USB-UART adapter
4. Flash via STM32CubeProgrammer or `stm32flash`
5. Set BOOT0 LOW and reset to run firmware

## RTOS Task Architecture

The firmware runs on **FreeRTOS V11** with a preemptive scheduler at 1 kHz tick rate. The application is split into three tasks, each responsible for a specific subsystem.

### Task Overview

| Task          | Priority | Stack  | Period        | Description                              |
|---------------|----------|--------|---------------|------------------------------------------|
| `IO_Scan`     | 3 (high) | 512 B  | 10 ms         | Scans all digital and analog inputs      |
| `ModbusRTU`   | 2 (med)  | 768 B  | event-driven  | Processes Modbus RTU frames from RS485   |
| `ModbusTCP`   | 2 (med)  | 1024 B | event-driven  | Processes Modbus TCP frames from Ethernet|

### Task States

Every task transitions through FreeRTOS's standard states during its lifetime:

```
                    ┌─────────┐
          vTaskCreate│         │
         ───────────►│  Ready  │◄─────────────┐
                    │         │               │
                    └────┬────┘               │
                         │                    │
              Scheduler │ picks              │ Yield / Preempt
                         │                    │
                    ┌────▼────┐               │
                    │ Running │───────────────┘
                    └────┬────┘
                         │
           ┌─────────────┼──────────────┐
           │             │              │
    vTaskDelay()  xQueueReceive()  xSemaphoreTake()
    (IO_Scan)     (ModbusRTU/TCP)  (RS485 TX mutex)
           │             │              │
     ┌─────▼─────┐ ┌────▼─────┐ ┌─────▼──────┐
     │  Blocked  │ │ Blocked  │ │  Blocked   │
     │ (timeout) │ │(queue rx)│ │ (mutex)    │
     └─────┬─────┘ └────┬─────┘ └─────┬──────┘
           │             │              │
           └─────────────┼──────────────┘
                         │ timeout / event / mutex acquired
                         ▼
                     ┌──────┐
                     │Ready │  (returns to scheduler queue)
                     └──────┘
```

**State descriptions:**

- **Running** — Only one task runs at a time on the Cortex-M4. The scheduler picks the highest-priority Ready task.
- **Ready** — Task is ready to run but a higher or equal priority task is currently Running.
- **Blocked** — Task is waiting for an event:
  - `IO_Scan` blocks on `vTaskDelayUntil()` for 10 ms periods
  - `ModbusRTU` blocks on `xQueueReceive()` waiting for RS485 frames
  - `ModbusTCP` blocks on `xQueueReceive()` waiting for Ethernet frames
- **Suspended** — Not used in this application (only via explicit `vTaskSuspend()`).

### Data Flow & IPC

```
┌──────────┐  ISR callback    ┌─────────────┐  xQueueReceive  ┌─────────────┐
│  RS485   │─────────────────►│ rs485_rx_q  │───────────────►│  ModbusRTU  │
│  USART2  │  xQueueSendFrom  │  (queue 8)  │                │    Task     │
└──────────┘     ISR()        └─────────────┘                └──────┬──────┘
                                                                    │
                                                          rs485_tx_mutex
                                                          (mutex, guards TX)
                                                                    │
┌──────────┐  ISR callback    ┌─────────────┐  xQueueReceive  ┌─────▼──────┐
│ Ethernet │─────────────────►│  eth_rx_q   │───────────────►│ ModbusTCP  │
│   MAC    │  xQueueSendFrom  │  (queue 8)  │                │    Task    │
└──────────┘     ISR()        └─────────────┘                └────────────┘

┌──────────┐  vTaskDelayUntil(10ms)  ┌─────────────┐
│ SysTick  │────────────────────────►│   IO_Scan   │──► Modbus registers
│  1 kHz   │                         │    Task     │    (holding regs 100-103)
└──────────┘                         └─────────────┘
```

**Key IPC primitives:**

| Mechanism       | Type  | Purpose                                           |
|-----------------|-------|---------------------------------------------------|
| `rs485_rx_q`   | Queue | Passes raw RS485 frames from ISR to ModbusRTU task|
| `eth_rx_q`     | Queue | Passes raw Ethernet frames from ISR to ModbusTCP task|
| `rs485_tx_mutex`| Mutex | Prevents concurrent RS485 TX (half-duplex bus)     |

**Why queues from ISR?** The RS485 and Ethernet RX callbacks execute in interrupt context. `xQueueSendFromISR()` is the only FreeRTOS API safe to call from ISRs. The task-level `xQueueReceive()` then picks up the frame in thread mode where blocking is allowed.

### Memory Management

FreeRTOS `heap_4.c` manages a **32 KB heap** (`configTOTAL_HEAP_SIZE`). Allocations use a first-fit algorithm with coalescing of adjacent free blocks.

| Allocation       | Size       | Type     |
|------------------|------------|----------|
| Task stacks      | ~2.3 KB    | Static   |
| Queues (2x8)     | ~4.1 KB    | Dynamic  |
| Mutex            | ~200 B     | Dynamic  |
| Kernel structures| ~1 KB      | Dynamic  |

Heap usage is monitored via `xPortGetFreeHeapSize()` — if it drops below a threshold, `vApplicationMallocFailedHook()` traps execution.

## License

MIT
