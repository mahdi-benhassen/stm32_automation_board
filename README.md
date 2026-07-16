# STM32 Automation Board

Industrial automation controller based on STM32F407 with Ethernet, RS485, and Modbus protocol support.

## Features

- **8 Digital Inputs** (opto-isolated, active-low, with software debouncing)
- **8 Digital Outputs** (push-pull, high-side drive)
- **4 Analog Inputs** (12-bit ADC, 0-10V range)
- **2 Analog Outputs** (12-bit DAC, 0-10V range)
- **4 Relays** with individual LED status indicators
- **RS485** half-duplex interface with DE/RE direction control (8E1, Modbus-compliant)
- **Ethernet** 10/100M RMII with built-in MAC (external PHY: LAN8720/DP83848)
- **Modbus RTU** (slave) over RS485
- **Modbus TCP** slave over Ethernet (lwIP, static IP, TCP port 502)
- **FreeRTOS V11** real-time OS with preemptive scheduler
- **IWDG watchdog** with per-task check-in monitoring
- **PVD brown-out detection** at 2.9V threshold
- **Buffer overflow protection** on Modbus register reads (max 125 registers)

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

# Download STM32CubeF4 HAL, FreeRTOS, and lwIP (or use CI which fetches all three)
./scripts/fetch_hal.sh
# FreeRTOS: clone Kernel V11.1.0 into lib/FreeRTOS (see .github/workflows/build.yml)
./scripts/fetch_lwip.sh

# Build
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Default network (static): **192.168.1.100/24**, gateway **192.168.1.1**, Modbus TCP **port 502** (see `board_config.h`).

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

## Validation Status

| Category | Status |
|----------|--------|
| Build (CI) | READY — compiles clean, CI green |
| Code quality | READY — modular, documented, reviewed |
| Safety (watchdog/PVD/fail-safe) | READY — per-task IWDG check-ins |
| Firmware hardening | READY — symmetric DI debounce, ADC last-good, RTU timebase, TCP ADU size |
| Modbus RTU | READY — protocol-compliant, T1.5/T3.5 framing, buffer-safe |
| Modbus TCP | CODE READY — lwIP + TCP :502 (static IP); validate PHY/link on hardware |
| FreeRTOS | READY — correct config, hooks, priorities |
| Production deployment | NOT READY — needs bootloader, NVRAM, self-test, certification |

See [docs/INDUSTRIAL_READINESS_REVIEW.md](docs/INDUSTRIAL_READINESS_REVIEW.md) for industrial readiness and roadmap.  
See [docs/VALIDATION_CHECKLIST.md](docs/VALIDATION_CHECKLIST.md) for full checklist.  
See [docs/TEST_PLAN.md](docs/TEST_PLAN.md) for bench test procedures.

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

| Task          | Priority | Stack (words) | Stack (bytes) | Period        | Description                              |
|---------------|----------|---------------|---------------|---------------|------------------------------------------|
| `IO_Scan`     | 3 (high) | 512           | 2048          | 10 ms         | Scans all digital and analog inputs      |
| `ModbusRTU`   | 2 (med)  | 768           | 3072          | event-driven  | Processes Modbus RTU frames from RS485   |
| `ModbusTCP`   | 2 (med)  | 1024          | 4096          | event-driven  | Processes Modbus TCP frames from Ethernet|
| `Watchdog`    | 1 (low)  | 256           | 1024          | 200 ms        | Refreshes IWDG if all tasks checked in   |

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

- **Running** — Only one task runs at a time. The scheduler picks the highest-priority Ready task.
- **Ready** — Task is ready to run but a higher or equal priority task is currently Running.
- **Blocked** — Task waits for an event: `IO_Scan` on `vTaskDelayUntil()`, `ModbusRTU`/`ModbusTCP` on `xQueueReceive()`.
- **Suspended** — Not used in this application.

### How Each Task Works

#### IO_Scan — Periodic, Time-Driven

```
vTaskDelayUntil(&xLastWakeTime, 10ms)
┌─────────────────────────────────────────────────┐
│ 1. digital_inputs_scan() → reads GPIOE IDR      │
│    with 6-sample debounce per channel (~60ms)    │
│                                                  │
│ 2. analog_inputs_scan_all() → starts ADC1,       │
│    loops 4 channels (polling conv), stores raw   │
│                                                  │
│ 3. modbus_write_holding_register(100+i, raw)     │
│    copies ADC values to Modbus holding regs      │
│    100-103 for external read access              │
│                                                  │
│ 4. vTaskDelayUntil() blocks for exactly 10ms     │
│    (not 10ms from unblock — fixed 100 Hz rate)   │
└─────────────────────────────────────────────────┘
```

`vTaskDelayUntil()` ensures a fixed 100 Hz scan rate regardless of processing jitter. If the scan takes 2 ms, it sleeps 8 ms. If it takes 0.5 ms, it sleeps 9.5 ms. The period does not drift.

#### ModbusRTU — Event-Driven, Queue-Blocked

```
xQueueReceive(rs485_rx_q, 50ms timeout)
┌─────────────────────────────────────────────────┐
│ 1. Blocks on queue for up to 100 ms              │
│    Timeout → loops back (prevents tight spin)    │
│                                                  │
│ 2. On frame arrival:                             │
│    modbus_rtu_process(frame, response) → CRC     │
│    check, function code dispatch, read/write     │
│    coils/registers, build response               │
│                                                  │
│ 3. If response built (resp_len > 0):             │
│    xSemaphoreTake(rs485_tx_mutex, 50ms)          │
│    ├─ acquired: rs485_set_tx_mode()              │
│    │            1000 NOPs (PHY settle)            │
│    │            rs485_send(response)              │
│    │            rs485_set_rx_mode()               │
│    │            xSemaphoreGive(mutex)             │
│    └─ timeout: drops response (bus busy)         │
│                                                  │
│ 4. Loops back to queue receive                   │
└─────────────────────────────────────────────────┘
```

The RS485 tx_mutex is critical — RS485 is half-duplex, only one device transmits at a time. The 1000-NOP delay after switching the DE pin allows the transceiver to stabilize (~1-5 µs). Response is silently dropped if TX is busy (50 ms timeout), preventing queue buildup.

#### ModbusTCP — Hybrid Polling + Event-Driven

```
xQueueReceive(eth_rx_q, 10ms timeout)
┌─────────────────────────────────────────────────┐
│ 1. ethernet_process() called EVERY loop          │
│    iteration, not only on queue event. This      │
│    drains ETH DMA for received frames.           │
│                                                  │
│    Why poll? ETH IRQ fires once per packet       │
│    burst, not per individual frame. Polling      │
│    ensures the DMA ring is fully drained.        │
│                                                  │
│ 2. xQueueReceive(100ms) — blocks until a         │
│    frame arrives or timeout                      │
│                                                  │
│ 3. On frame: modbus_tcp_build_response() →       │
│    strips MBAP header, delegates to RTU engine,  │
│    wraps response with same transaction ID       │
│                                                  │
│ 4. ethernet_send(response) → configures          │
│    ETH_TxPacketConfig, calls HAL_ETH_Transmit    │
│                                                  │
│ 5. Loops back                                    │
└─────────────────────────────────────────────────┘
```

Ethernet RX uses a dual-path mechanism — `ethernet_process()` polls the DMA ring and pushes frames into the queue via the ISR callback, while `xQueueReceive()` dequeues and processes them in task context. This decouples DMA timing (interrupt context) from Modbus processing (task context).

### Inter-Task Data Flow

```
IO_Scan (periodic)           ModbusRTU (event)         ModbusTCP (event)
     │                            │                         │
     │ writes regs 100-103        │ reads regs on           │
     ├───────────────────────────►│ FC 03/04 request        │
     │                            │                         │
     │ reads DO states            │ writes DO coils on      │
     │◄───────────────────────────┤ FC 05/0F                │
     │                            │                         │
     │                            │                         │ reads regs / writes
     │                            │                         │ coils on TCP requests
     │ reads/writes shared        │                         │
     │ Modbus registers           │                         │
     │◄───────────────────────────┼────────────────────────►│
     │                            │                         │
     ▼                            ▼                         ▼
  Modbus register array (shared memory, no lock needed —
  uint16_t reads/writes are atomic on Cortex-M4)
```

All three tasks share the Modbus register array without explicit locking because `uint16_t` reads/writes are atomic on ARM Cortex-M4 (single aligned 16-bit store instruction). The IO_Scan task only writes to holding registers 100-103 (AI mirrors), never reads from them — ModbusRTU and ModbusTCP only read from these locations. For coils and DO registers, task writes are mediated through dedicated `modbus_bit_write()` and `modbus_regs_write()` functions, with hardware sync deferred to `modbus_sync_registers()` inside each task's request handler.

### Data Flow & IPC

```
┌──────────┐  ISR fills     ┌──────────┐  task drains    ┌─────────────┐  xQueueSend    ┌─────────────┐
│  RS485   │───────────────►│  ring    │────────────────►│ rs485_       │──────────────►│ rs485_rx_q  │
│  USART2  │   rx_head++    │  buffer  │  rs485_process()│ process()    │               │  (queue 8)  │
└──────────┘                └──────────┘                 └─────────────┘               └──────┬──────┘
                                                                                              │
                                                                                     xQueueReceive
                                                                                              │
                                                                                      ┌───────▼──────┐
                                                                                      │  ModbusRTU   │
                                                                                      │    Task      │
                                                                                      └──────────────┘

┌──────────┐  DMA IRQ       ┌─────────────┐  task polls     ┌─────────────┐  xQueueSend    ┌─────────────┐
│ Ethernet │───────────────►│  ETH DMA    │────────────────►│ ethernet_    │──────────────►│  eth_rx_q   │
│   MAC    │  ETH_IRQHandler│  descriptors│ethernet_process()│ process()   │               │  (queue 8)  │
└──────────┘                └─────────────┘                 └─────────────┘               └──────┬──────┘
                                                                                                 │
                                                                                        xQueueReceive
                                                                                                 │
                                                                                         ┌───────▼──────┐
                                                                                         │  ModbusTCP   │
                                                                                         │    Task      │
                                                                                         └──────────────┘
```

┌──────────┐  vTaskDelayUntil(10ms)  ┌─────────────┐
│ SysTick  │────────────────────────►│   IO_Scan   │──► Modbus registers
│  1 kHz   │                         │    Task     │    (holding regs 100-103)
└──────────┘                         └─────────────┘
```

| Mechanism        | Type  | Purpose                                           |
|------------------|-------|---------------------------------------------------|
| `rs485_rx_q`    | Queue | Passes raw RS485 frames from ISR to ModbusRTU task|
| `eth_rx_q`      | Queue | Passes raw Ethernet frames from ISR to ModbusTCP task|
| `rs485_tx_mutex` | Mutex | Prevents concurrent RS485 TX (half-duplex bus)     |

**Why queues from ISR?** The RS485 and Ethernet RX callbacks execute in interrupt context. `xQueueSendFromISR()` is the only FreeRTOS API safe to call from ISRs. The task-level `xQueueReceive()` then picks up the frame in thread mode where blocking is allowed.

### Memory Management

FreeRTOS `heap_4.c` manages a **32 KB heap** (`configTOTAL_HEAP_SIZE`). Allocations use a first-fit algorithm with coalescing of adjacent free blocks.

| Allocation        | Size       | Type     |
|-------------------|------------|----------|
| Task stacks       | ~9.2 KB    | Dynamic  |
| Queues (2x8)      | ~4.1 KB    | Dynamic  |
| Mutex             | ~200 B     | Dynamic  |
| Kernel structures | ~1 KB      | Dynamic  |

When FreeRTOS `pvPortMalloc()` fails to allocate memory, `vApplicationMallocFailedHook()` traps execution (infinite loop with interrupts disabled). Heap usage can be inspected at runtime via `xPortGetFreeHeapSize()`.

## License

MIT
