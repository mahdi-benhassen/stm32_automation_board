# Industrial Validation Checklist

## Build Validation

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | Compiles with ARM GCC 12.3 (Release) | PASS | CI green on `feature/freertos-integration` |
| 2 | Compiles with ARM GCC 12.3 (Debug) | PASS | Both build types in CI matrix |
| 3 | No compiler errors | PASS | |
| 4 | No compiler warnings (excluding HAL) | PASS | `-Wall -Wextra` enabled |
| 5 | Linker succeeds with memory report | PASS | FLASH 3.3%, RAM 44.8% |
| 6 | `.hex` and `.bin` artifacts generated | PASS | CI uploads artifacts |
| 7 | `cppcheck` static analysis | PASS | Lint job in CI with `--error-exitcode=1` |
| 8 | Modbus unit tests (CRC, quantity, buffer) | PASS | 20 native tests in CI |
| 9 | FreeRTOS config validation tests | PASS | 11 native tests in CI |
| 10 | Linker script validation tests | PASS | 7 native tests in CI |
| 11 | Firmware size guard (<80% flash) | PASS | CI rejects oversized builds |

## Safety Features

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | IWDG watchdog initialized | PASS | LSI, `/64` prescaler, reload `0x0FFF` (~8 s @ 32 kHz LSI) |
| 2 | Watchdog task with check-in pattern | PASS | Per-task flags (`checkin_io_scan` / `_rtu` / `_tcp`); all must check in before refresh |
| 3 | PVD brown-out detection | PASS | `PWR_PVDLEVEL_2` (~2.9V) |
| 4 | Default_Handler triggers system reset | PASS | Writes to `NVIC_AIR` register |
| 5 | Fail-safe output states on reset | PASS | All DO/relay/AO initialized to OFF/0V |
| 6 | Stack overflow hook | PASS | `vApplicationStackOverflowHook` traps |
| 7 | Malloc failed hook | PASS | `vApplicationMallocFailedHook` traps |
| 8 | `INCLUDE_uxTaskGetStackHighWaterMark` | PASS | Enabled for runtime monitoring |

## Firmware Hardening (industrial code review)

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | Symmetric DI debounce (on and off) | PASS | 5-sample integration (~50 ms @ 10 ms scan) in `digital_io.c` |
| 2 | ADC last-good on conversion fail | PASS | Seed buffer from `ai_raw_values`; update only on `HAL_OK` |
| 3 | Watchdog check-in race-free | PASS | Separate volatile bytes (no shared RMW `\|=`) |
| 4 | RTU µs timebase re-entrant | PASS | `PENDSTSET` (not COUNTFLAG); FreeRTOS tick hook + `rs485_on_systick` |
| 5 | Modbus TCP ADU queue size | PASS | `modbus_frame_t` / eth path use 260-byte max ADU |
| 6 | RTU T1.5 / T3.5 framing | PASS | SysTick soft timer (portable; no TIM6) |

## Modbus Protocol Compliance

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | FC 0x01 Read Coils | PASS | Max 2000 coils per request |
| 2 | FC 0x02 Read Discrete Inputs | PASS | Max 2000 inputs per request |
| 3 | FC 0x03 Read Holding Registers | PASS | Max 125 registers per request |
| 4 | FC 0x04 Read Input Registers | PASS | Max 125 registers per request |
| 5 | FC 0x05 Write Single Coil | PASS | Validates 0xFF00/0x0000 only |
| 6 | FC 0x06 Write Single Register | PASS | |
| 7 | FC 0x0F Write Multiple Coils | PASS | Max 1968 coils, byte_count validated |
| 8 | FC 0x10 Write Multiple Registers | PASS | Max 123 registers, byte_count validated |
| 9 | CRC16 calculation (table-driven) | PASS | Standard Modbus polynomial 0xA001 |
| 10 | Broadcast (slave ID 0) — no response | PASS | Executes write, suppresses response |
| 11 | Exception code 0x01 (Illegal Function) | PASS | Unknown FC |
| 12 | Exception code 0x02 (Illegal Data Address) | PASS | Out-of-range addresses |
| 13 | Exception code 0x03 (Illegal Data Value) | PASS | Invalid coil value, byte_count mismatch, quantity=0 |
| 14 | UART 8E1 framing (11-bit char) | PASS | `UART_WORDLENGTH_9B` + `UART_PARITY_EVEN` |
| 15 | PDU/CRC separation for TCP | PASS | `modbus_pdu_process()` called directly by TCP |
| 16 | Mutex protects shared registers | PASS | `modbus_mutex` wraps all PDU processing |
| 17 | 3.5-char frame detection | PASS | T1.5 in USART IRQ; T3.5 via tick hook + `rs485_process()` poll (`rs485.c`) |

## Security

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | Buffer overflow on read registers | PASS | Enforced `quantity <= 125` |
| 2 | Buffer overflow on read coils | PASS | Enforced `quantity <= 2000` |
| 3 | Buffer overflow on write registers | PASS | Enforced `quantity <= 123` |
| 4 | Buffer overflow on write coils | PASS | Enforced `quantity <= 1968` |
| 5 | Input length validation | PASS | `rx_len < 4` check, `rx_pdu_len` check |
| 6 | Register address bounds check | PASS | `start_addr + quantity > MODBUS_MAX_*` |
| 7 | Network authentication | **LIMITATION** | No auth on Modbus TCP (by protocol design) |

## FreeRTOS Configuration

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | Preemptive scheduling | PASS | `configUSE_PREEMPTION = 1` |
| 2 | 1 kHz tick rate | PASS | `configTICK_RATE_HZ = 1000` |
| 3 | 32 KB heap | PASS | `configTOTAL_HEAP_SIZE = 32768` |
| 4 | Mutex with priority inheritance | PASS | `configUSE_MUTEXES = 1` |
| 5 | Timers enabled | PASS | `configUSE_TIMERS = 1` |
| 6 | Stack overflow detection (method 2) | PASS | `configCHECK_FOR_STACK_OVERFLOW = 2` |
| 7 | Task notification API | PASS | `configUSE_TASK_NOTIFICATIONS = 1` |
| 8 | 4 tasks with correct priorities | PASS | IO(3) > RTU(2) = TCP(2) > WD(1) |
| 9 | SysTick/PendSV/SVC mapped to FreeRTOS | PASS | Via `FreeRTOSConfig.h` macros |
| 10 | `HAL_GetTick` uses FreeRTOS tick | PASS | `xTaskGetTickCount()` |

## Hardware Pin Assignments

| # | Check | Status | Notes |
|---|-------|--------|-------|
| 1 | No pin conflicts (DI/DO/AI/AO/relay/RS485/ETH) | PASS | DO7 moved from PB12 to PB14 |
| 2 | Ethernet RMII pins correctly assigned | PASS | PA1/PA2/PA7/PC1/PC4/PC5/PB11/PB12/PB13 |
| 3 | RS485 USART2 pins with AF7 | PASS | PD5(TX)/PD6(RX)/PD7(DE) |
| 4 | ADC channels mapped correctly | PASS | IN0/IN10/IN12/IN13 |
| 5 | DAC channels mapped correctly | PASS | DAC1(PA4)/DAC2(PA5) |
| 6 | Status LED on PC13 | PASS | 500ms toggle via FreeRTOS timer |

## Known Limitations (Not Production-Ready)

| # | Limitation | Impact | Mitigation |
|---|-----------|--------|------------|
| 1 | ~~No TCP/IP stack (lwIP)~~ | **FIXED (static IP)** | lwIP + netconn server on port 502; PHY/link still need hardware validation |
| 2 | ~~No 3.5-char frame detection on RS485~~ | **FIXED** | SysTick soft T1.5/T3.5 framing |
| 3 | No configuration persistence (NVRAM) | Settings lost on power cycle | Add I2C/flash-based parameter storage |
| 4 | No firmware bootloader | No field updates without SWD | Implement UART or Ethernet bootloader |
| 5 | No versioning/fingerprinting | Cannot identify firmware version in field | Add version string in flash + Modbus register |
| 6 | No self-test diagnostics | No power-on hardware verification | Add ADC/DAC/GPIO self-test (POST) |
| 7 | 64KB CCMRAM unused | Wasted fast RAM | Move FreeRTOS heap or hot data to CCMRAM |
| 8 | ~~Asymmetric DI debounce~~ | **FIXED** | Symmetric integration debounce (both edges) |
| 9 | No Modbus security (TLS) | Unauthenticated network access | Implement Modbus Security (RFC 9433) if required |
| 10 | No CE/UL safety certification | Cannot be deployed in certified equipment | Submit for IEC 61010 / IEC 61326 testing |

See [INDUSTRIAL_READINESS_REVIEW.md](INDUSTRIAL_READINESS_REVIEW.md) for full gap analysis and production roadmap.

## Validation Verdict

| Category | Status |
|----------|--------|
| **Build** | READY — compiles clean, CI green |
| **Code quality** | READY — modular, documented, reviewed |
| **Safety features** | READY — watchdog, PVD, fail-safe, reset-on-fault |
| **Security** | PARTIAL — buffer overflow protected, no network auth |
| **Modbus RTU** | READY — protocol-compliant, buffer-safe, mutex-protected |
| **Modbus TCP** | CODE READY — lwIP + :502; validate on hardware |
| **FreeRTOS** | READY — correct priorities, stacks, hooks |
| **Hardware** | READY — no pin conflicts, correct peripheral config |
| **Production** | NOT READY — missing bootloader, NVRAM, self-test, certification |

**Overall: Bench Ready / Field Prototype** — ready for hardware bench validation and Modbus **RTU** field testing. **Not** ready for industrial Modbus **TCP** or certified deployment until lwIP, NVRAM, bootloader, POST, and certification milestones are complete.
