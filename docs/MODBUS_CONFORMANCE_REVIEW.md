# Modbus Conformance Review

Review date: 2026-07-13
Scope: firmware source on `feature/freertos-integration`, plus the three reference PDFs in this directory.

## Reference material

| Document | Revision | Firmware area reviewed |
|---|---:|---|
| `modbusprotocolspecification.pdf` | V1.1b3 | PDU encoding, exception responses, implemented function-code limits |
| `modbusoverserial.pdf` | V1.02 | RTU framing, CRC order, broadcast behavior, RS-485 timing |
| `messagingimplementationguide.pdf` | V1.0b | MBAP header and Modbus/TCP ADU parsing |

## Two-pass results

The first pass compared every implemented request path to the protocol rules. The second pass focused on malformed/truncated inputs, arithmetic overflow, no-response paths, and the advertised transport boundary.

| Area | Result | Evidence |
|---|---|---|
| Implemented PDU functions | Conformant after fixes | FC 01, 02, 03, 04, 05, 06, 0F, and 10 validate exact request structure, quantity limits, values, and table ranges. |
| Exception responses | Conformant after fixes | Invalid function, value, address, and malformed request lengths return the appropriate exception PDU. |
| RTU framing | Conformant for the configured 8E1 link | `src/rs485.c` uses CRC-16 low-byte-first, T1.5 rejection, and T3.5 frame completion; above 19,200 bit/s it uses the recommended fixed 750/1,750 microsecond timings. |
| Serial broadcast | Conformant after fixes | Address 0 executes only the implemented write functions and emits no response. |
| MBAP handling | Conformant after fixes | Protocol ID must be zero; Length must include Unit Identifier and PDU and match the received ADU exactly; the response copies transaction and unit identifiers. |
| End-to-end Modbus/TCP service | Implemented (code) | lwIP netconn server listens on port 502 (`modbus_tcp_server.c`); MBAP reassembly feeds `modbus_tcp_build_response()`. Validate on hardware with live PHY. See `INDUSTRIAL_READINESS_REVIEW.md`. |
| TCP ADU sizing | Conformant | Full 260-byte Modbus TCP ADUs supported end-to-end. |

## Corrections made by this review

- Set all response lengths to zero before parsing so invalid input cannot cause a stale response to be transmitted.
- Reject truncated, overlong, and internally inconsistent PDUs before accessing their fields.
- Prevent 16-bit `start + quantity` wraparound and return exception `02` for invalid single-write addresses.
- Validate the byte-count and the complete payload length of FC 0F and FC 10 requests.
- Enforce Modbus/TCP Protocol Identifier `0`, the MBAP length field, and the 253-byte maximum PDU length.
- Treat Unit Identifier `0` on TCP as an identifier to echo, not as a serial-line broadcast.

## Extended function codes (issue #3) vs V1.1b3

Reviewed against `Modbus_Application_Protocol_V1_1b3.pdf` §6.7, §6.14–6.15, §6.17, §6.21.

| FC | Spec section | Conformance notes |
|----|--------------|-------------------|
| 0x07 | §6.7 | PDU OK (FC + 1 status byte). Device-specific status = discrete inputs 0–7 (LSB = DI0). Spec marks serial-line-only; we also answer on TCP for dual-stack convenience. |
| 0x14 | §6.14 | Sub-req 7 bytes, ref type 06, byte count 0x07–0xF5. Virtual files 1–4 × 128 regs; record number > 0x270F or out of store → exc 02. |
| 0x15 | §6.15 | Request data length 0x09–0xFB; response echo of request. Same virtual file store. |
| 0x17 | §6.17 | Write-before-read; read qty 1–0x007D; write qty 1–0x0079; BC = 2×write. |
| 0x2B/0x0E | §6.21 | Basic objects 0–2 mandatory strings; conformity **0x81** (basic + individual); stream 01/02/03 and individual 04; unknown stream Object Id restarts at 0; unknown individual Object Id → exc 02. |

## Residual limitations and next action

This review does not certify the hardware installation or electrical layer. RS-485 termination, biasing, cable topology, EMC, and system-level timing must be verified on the target board.

Modbus/TCP transport is implemented via lwIP (static IP, port 502). On-target Ethernet PHY link, ARP, and multi-client soak tests remain lab work.

File records are a **RAM virtual store** (not a filesystem); device identification is basic-level only (no regular/extended optional objects 0x03–0x06).

## Integration review (RTU / TCP / FreeRTOS)

| Path | Integration |
|------|-------------|
| RTU | USART IRQ → soft T1.5/T3.5 → `rs485_process` → queue → `modbus_rtu_process` under `modbus_mutex` |
| TCP | lwIP netconn :502 → MBAP reassembly → `modbus_tcp_build_response` → `modbus_pdu_process` under same mutex |
| IO scan | DI/AI updates take `modbus_mutex` so table writes do not race PDU handlers |
| FC 0x02 / 0x07 | Refresh debounced DI snapshot at PDU time |
| FC 0x04 | Refresh input-reg AI mirror from holding 100+ at PDU time |
| Response size | RTU PDU ≤ 253; TCP ADU ≤ 260 enforced before framing |

Both branches share the same `modbus_pdu_process` core for the extended FCs.

## Modbus RTU master (client)

| Item | Notes |
|------|-------|
| Module | `src/modbus_master.c` + bare-metal RS485 glue `src/modbus_master_rtu.c` |
| Role | Dual-role: slave remains default RX path; while a master transaction is armed, frames are delivered to the master buffer instead of `modbus_rtu_process` |
| FCs | Master builders/parsers for 0x01–0x06, 0x07, 0x0F, 0x10, 0x14, 0x15, 0x17, 0x2B/0x0E |
| Broadcast | Slave address 0: write FCs send ADU and expect no response |
| API | High-level `modbus_master_read_*` / `write_*` / file / device-id; low-level `modbus_master_transaction()` |
| Transport | Injectable `modbus_master_transport_t` (unit-testable without UART) |

```c
uint16_t regs[4];
uint8_t exc = 0;
modbus_status_t st = modbus_master_read_holding_registers(2, 0, 4, regs, &exc);
```

## Verification

- `git diff --check` passed.
- Native and ARM compiler binaries are not installed in this workspace, so local compilation could not be run. The existing GitHub Actions workflow builds the STM32 firmware and runs the native Modbus, FreeRTOS, and linker tests after push.
