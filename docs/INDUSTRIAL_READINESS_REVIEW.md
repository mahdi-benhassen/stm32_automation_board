# Industrial Readiness Review — STM32 Automation Board

**Branch reviewed:** `feature/freertos-integration`  
**Review date:** 2026-07-16  
**Verdict:** Bench Ready / Field Prototype (Modbus RTU) — **not** certified industrial Modbus TCP

This document maps a full industrial evaluation against the **current firmware**. Several issues called out in earlier reviews are already fixed in tree; residual gaps for production remain explicit.

---

## 1. Executive Summary

| Area | Status |
|------|--------|
| Code quality / modularity | **PASS** |
| CI/CD (build, unit tests, cppcheck, size guard) | **PASS** |
| Documentation (user manual, test plan, validation) | **PASS** |
| Modbus RTU (framing, CRC, exceptions, broadcast) | **PASS** — ready for bench / RTU field trials |
| Safety (IWDG multi-task check-in, PVD, stack/malloc hooks, fail-safe I/O) | **PASS** |
| Firmware hardening (debounce, ADC fail-safe, RTU timebase, TCP frame size) | **PASS** (see §2) |
| End-to-end Modbus TCP over Ethernet | **IN TREE** — lwIP + TCP :502 server (bench validation on hardware still required) |
| Production deployment (bootloader, NVRAM, POST, certification) | **NOT READY** |

The most important remaining blocker for industrial **Modbus TCP** is the absence of a run-time TCP/IP stack. The MBAP/PDU path (`modbus_tcp_build_response`) is implemented and tested; `ethernet.c` is only a MAC/DMA layer and cannot terminate TCP port 502.

---

## 2. Technical Evaluation vs Current Code

### A. Hardware drivers & I/O

| Topic | Earlier finding | Current status |
|-------|-----------------|----------------|
| DI debounce | Turn-off path instantaneous (noise risk) | **FIXED** — symmetric 5-sample integration debounce (~50 ms both edges @ 10 ms scan) in `digital_io.c` |
| ADC scan buffer | Uninitialized on poll timeout → garbage to Modbus | **FIXED** — seed from `ai_raw_values`; update cache only on `HAL_OK` in `analog_io.c` |
| Relays | Safe OFF default | **PASS** — unchanged, conformant |

### B. RTOS & safety

| Topic | Earlier finding | Current status |
|-------|-----------------|----------------|
| Task priorities | IO(3) > RTU/TCP(2) > WD(1) | **PASS** — correct hierarchy |
| Watchdog check-in | Shared `task_checkin \|=` RMW race | **FIXED** — separate `checkin_io_scan` / `checkin_modbus_rtu` / `checkin_modbus_tcp` flags |
| RS485 µs timebase | COUNTFLAG read-to-clear → 1 ms backward jump | **FIXED** — `SCB->ICSR` `PENDSTSET` (side-effect free); FreeRTOS tick hook advances private `rtu_ms_tick` |
| IWDG | Multi-task refresh | **PASS** — refresh only when all three tasks check in |

### C. Modbus protocol & security

| Topic | Earlier finding | Current status |
|-------|-----------------|----------------|
| PDU validation / CRC / broadcast | Strong | **PASS** — see `MODBUS_CONFORMANCE_REVIEW.md` |
| Buffer quantity limits | Strong | **PASS** |
| Queue frame size for TCP | 256 truncated full ADU | **FIXED** — `modbus_frame_t` sized to `MODBUS_TCP_FRAME_MAX` (260); eth path caps at `MODBUS_TCP_MAX_ADU` |
| TCP/IP + port 502 | Missing | **ADDED** — lwIP 2.2 + `modbus_tcp_server` netconn listen on `MODBUS_TCP_PORT` (502); static IP from `board_config.h` |

---

## 3. Documentation

| Document | Assessment |
|----------|------------|
| `USER_MANUAL.md` | Field-technician ready; notes lwIP gap for TCP |
| `TEST_PLAN.md` | Bench / stress / brown-out procedures |
| `VALIDATION_CHECKLIST.md` | Build, safety, protocol tracking |
| `MODBUS_CONFORMANCE_REVIEW.md` | Parser vs official PDFs |
| This file | Industrial readiness + gap roadmap |

---

## 4. Industrial Roadmap (production image)

| Step | Feature | Description | Importance | Status |
|------|---------|-------------|------------|--------|
| 1 | **lwIP TCP/IP stack** | ARP, static IP, ICMP, TCP listen **:502** → `modbus_tcp_build_response()` | **Critical** | **Done (static IP)** — DHCP still optional/open |
| 2 | NVRAM config | Persist IP, gateway, mask, Modbus slave ID (I2C/SPI EEPROM or flash) | High | Open |
| 3 | Firmware bootloader | Secure update over RS485 and/or Ethernet | High | Open |
| 4 | Power-On Self-Test | Supply (ADC/PVD), DAC↔ADC loopback, register integrity | Medium | Open |
| 5 | CCMRAM use | FreeRTOS heap / hot data in 64 KB CCM | Medium | Open |
| 6 | IEC 61010 / 61326 | EMC + electrical safety certification | Required for certified sale | Open |

```mermaid
flowchart LR
  A[Bench Ready<br/>Modbus RTU] --> B[lwIP + TCP:502]
  B --> C[NVRAM + Bootloader]
  C --> D[POST + CCMRAM]
  D --> E[EMC / Safety cert]
  E --> F[Production industrial image]
```

---

## 5. Sign-off

| Category | Status |
|----------|--------|
| Code quality | **PASS** |
| Build pipeline | **PASS** |
| Documentation | **PASS** |
| Safety features | **PASS** |
| Network service (Modbus TCP stack) | **PASS (code)** — lwIP + :502; **needs on-target link/PHY validation** |
| Overall | **Bench Ready / Field Prototype** — RTU solid; TCP ready for hardware bring-up |

**Recommendation:** Bench-test Modbus **RTU** and bring up Ethernet PHY + ping + Modbus TCP clients against static IP `192.168.1.100:502`. Complete NVRAM, bootloader, POST, and certification before production.

### Architecture (Modbus TCP path)

1. `ethernet_init()` — STM32 MAC/RMII DMA  
2. `net_init()` — `tcpip_init` + netif (static IP) + `net_in` RX task  
3. `modbus_tcp_server_task` — lwIP `netconn` listen **:502**, MBAP reassembly, `modbus_tcp_build_response()`  

### Related commits

- lwIP integration + Modbus TCP server (this work)
- `3412522` / `918b510` — industrial firmware hardening
- `ab1de89` — SysTick tick-hook RTU timeouts (no TIM6)
