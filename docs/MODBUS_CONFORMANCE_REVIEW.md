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
| End-to-end Modbus/TCP service | Not yet implemented | The repository has no TCP/IP stack or TCP port-502 listener. `ethernet.c` is a MAC/DMA layer; it cannot turn Ethernet frames into TCP payloads. Integrating lwIP (or equivalent) remains required before claiming Modbus/TCP operation. See `INDUSTRIAL_READINESS_REVIEW.md`. |
| TCP ADU queue sizing | Fixed for future stack | Application queues and eth callback allow full 260-byte Modbus TCP ADUs (`MODBUS_TCP_FRAME_MAX` / `MODBUS_TCP_MAX_ADU`). |

## Corrections made by this review

- Set all response lengths to zero before parsing so invalid input cannot cause a stale response to be transmitted.
- Reject truncated, overlong, and internally inconsistent PDUs before accessing their fields.
- Prevent 16-bit `start + quantity` wraparound and return exception `02` for invalid single-write addresses.
- Validate the byte-count and the complete payload length of FC 0F and FC 10 requests.
- Enforce Modbus/TCP Protocol Identifier `0`, the MBAP length field, and the 253-byte maximum PDU length.
- Treat Unit Identifier `0` on TCP as an identifier to echo, not as a serial-line broadcast.

## Residual limitations and next action

This review does not certify the hardware installation or electrical layer. RS-485 termination, biasing, cable topology, EMC, and system-level timing must be verified on the target board.

For Modbus/TCP, add an lwIP TCP server bound to port 502 and invoke `modbus_tcp_build_response()` only with complete TCP payloads. That work is deliberately outside this parser-hardening change because the required network stack is not present in this repository.

## Verification

- `git diff --check` passed.
- Native and ARM compiler binaries are not installed in this workspace, so local compilation could not be run. The existing GitHub Actions workflow builds the STM32 firmware and runs the native Modbus, FreeRTOS, and linker tests after push.
