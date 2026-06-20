# Test Plan — STM32 Automation Board

## 1. Power-On Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 1.1 | Apply 24V DC | Power LED illuminates | |
| 1.2 | Wait 2 seconds | Status LED (PC13) blinks at 1 Hz | |
| 1.3 | Connect Ethernet cable | ETH Link LED on RJ45 lights | |
| 1.4 | Measure 3.3V rail | 3.25V–3.35V | |
| 1.5 | Measure board current | < 200 mA (no loads) | |

## 2. Digital Input Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 2.1 | Read DI0-DI7 via Modbus FC 0x02 (all open) | All bits = 0 | |
| 2.2 | Short DI0 to GND | DI0 = 1 within 60ms (6-sample debounce) | |
| 2.3 | Release DI0 | DI0 = 0 within 60ms | |
| 2.4 | Short DI0-DI7 to GND simultaneously | All bits = 1 | |
| 2.5 | Release all | All bits = 0 | |
| 2.6 | Toggle DI3 at 10 Hz | Modbus reads alternate 0/1 at ~5 Hz max (debounce limit) | |

## 3. Digital Output Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 3.1 | Write coil 0 (DO0) = ON via FC 0x05 | PB0 = HIGH (3.3V), 24V at terminal | |
| 3.2 | Write coil 0 = OFF | PB0 = LOW, 0V at terminal | |
| 3.3 | Write coils 0-7 = ON via FC 0x0F | All DO terminals = 24V | |
| 3.4 | Write coils 0-7 = OFF | All DO terminals = 0V | |
| 3.5 | Write coil 7 (DO7, PB14) = ON | PB14 = HIGH | |
| 3.6 | Read coils 0-7 via FC 0x01 | Matches written states | |
| 3.7 | Power cycle, read coils | All = 0 (fail-safe default) | |

## 4. Analog Input Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 4.1 | Apply 0V to AI0 | Input register 0 = 0 (±5 LSB) | |
| 4.2 | Apply 5.0V to AI0 | Input register 0 = 2047 (±20 LSB) | |
| 4.3 | Apply 10.0V to AI0 | Input register 0 = 4095 (±20 LSB) | |
| 4.4 | Apply 2.5V to AI1 | Input register 1 = 1023 (±15 LSB) | |
| 4.5 | Apply 7.5V to AI2 | Input register 2 = 3071 (±20 LSB) | |
| 4.6 | Read holding regs 100-103 via FC 0x03 | Matches input registers 0-3 | |
| 4.7 | Disconnect AI0 | Input register 0 = 0 (floating pulls low) | |
| 4.8 | Apply 10V to all 4 AI simultaneously | All 4 registers read 4095 (±20) | |

## 5. Analog Output Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 5.1 | Write holding reg 0 = 0 (AO0) | PA4 terminal = 0V (±0.05V) | |
| 5.2 | Write holding reg 0 = 2047 | PA4 terminal = 5.0V (±0.1V) | |
| 5.3 | Write holding reg 0 = 4095 | PA4 terminal = 10.0V (±0.1V) | |
| 5.4 | Write holding reg 1 = 1023 (AO1) | PA5 terminal = 2.5V (±0.1V) | |
| 5.5 | Write holding regs 0+1 via FC 0x10 | Both outputs update simultaneously | |
| 5.6 | Power cycle, measure AO0/AO1 | Both = 0V (fail-safe default) | |
| 5.7 | Write 4095 then 0 rapidly (10x) | Output tracks without lag > 2ms | |

## 6. Relay Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 6.1 | Write coil 8 (RELAY1) = ON via FC 0x05 | Relay 1 contact closes, LED1 ON | |
| 6.2 | Write coil 8 = OFF | Relay 1 contact opens, LED1 OFF | |
| 6.3 | Write coils 8-11 = ON via FC 0x0F | All 4 relays energize, all LEDs ON | |
| 6.4 | Write coils 8-11 = OFF | All 4 relays de-energize, all LEDs OFF | |
| 6.5 | Write coil 10 (RELAY3) = ON, others OFF | Only relay 3 + LED3 ON | |
| 6.6 | Read coils 8-11 via FC 0x01 | Matches written states | |
| 6.7 | Power cycle | All relays OFF (fail-safe) | |
| 6.8 | Continuity test: RLY1 NO-COM with relay ON | < 1Ω (closed contact) | |
| 6.9 | Continuity test: RLY1 NO-COM with relay OFF | > 10MΩ (open contact) | |

## 7. RS485 Modbus RTU Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 7.1 | Send FC 0x01 read 8 coils at slave ID 1, 115200 8E1 | Response with 1 byte coil data | |
| 7.2 | Send FC 0x02 read 8 discrete inputs | Response with 1 byte DI data | |
| 7.3 | Send FC 0x03 read holding reg 0 | Response with 2 bytes (AO0 value) | |
| 7.4 | Send FC 0x04 read input reg 0 | Response with 2 bytes (AI0 raw) | |
| 7.5 | Send FC 0x05 write coil 0 = ON | Echo response, DO0 energizes | |
| 7.6 | Send FC 0x06 write holding reg 0 = 2047 | Echo response, AO0 = 5V | |
| 7.7 | Send FC 0x0F write 8 coils | Response with addr+quantity | |
| 7.8 | Send FC 0x10 write 2 holding regs | Response with addr+quantity | |
| 7.9 | Send to slave ID 2 (wrong) | No response | |
| 7.10 | Send broadcast (ID 0) write coil 0 = ON | DO0 energizes, NO response | |
| 7.11 | Send FC 0x05 with value 0x1234 (invalid) | Exception response 0x03 | |
| 7.12 | Send FC 0x03 with quantity = 200 (>125) | Exception response 0x03 | |
| 7.13 | Send FC 0x99 (invalid function) | Exception response 0x01 | |
| 7.14 | Send frame with bad CRC | No response (silent drop) | |
| 7.15 | Send 100 requests/second for 60s | No missed responses, no hangs | |
| 7.16 | Disconnect RS485 mid-transfer | System recovers, next request works | |

## 8. Watchdog Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 8.1 | Normal operation for 10 minutes | No watchdog reset (status LED steady blink) | |
| 8.2 | Force IO_Scan task to hang (debug breakpoint) | System resets within ~1 second | |
| 8.3 | Force ModbusRTU task to hang | System resets within ~1 second | |
| 8.4 | Remove RS485 and Ethernet (no traffic) | System stays alive (tasks check in on timeout) | |

## 9. Brown-Out (PVD) Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 9.1 | Slowly reduce supply voltage from 24V to 18V | No reset, normal operation | |
| 9.2 | Reduce voltage further until 3.3V rail drops below 2.9V | System resets (PVD trigger) | |
| 9.3 | Restore voltage | System boots normally, all outputs OFF | |

## 10. FreeRTOS Stress Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 10.1 | Flood RS485 with 1000 req/s for 5 minutes | No crash, no stack overflow | |
| 10.2 | Simultaneous RS485 + Ethernet traffic | Both paths process independently | |
| 10.3 | Check `uxTaskGetStackHighWaterMark` after stress | > 30% stack free on all tasks | |
| 10.4 | Check `xPortGetFreeHeapSize` after stress | > 20KB free | |

## 11. Environmental Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 11.1 | Operate at -20°C for 1 hour | All functions normal | |
| 11.2 | Operate at +70°C for 1 hour | All functions normal | |
| 11.3 | Power cycle 100 times at room temp | Every boot succeeds, outputs fail-safe | |
| 11.4 | ESD test: 4kV contact discharge on DI terminals | No crash, no false input readings | |
| 11.5 | ESD test: 8kV air discharge on RS485 terminals | No crash, communication resumes | |

## 12. Modbus Conformance Test

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 12.1 | Run Modbus conformance test suite (Modbus.org) | All mandatory FCs pass | |
| 12.2 | Test with 3 different Modbus masters | Interoperable with all | |
| 12.3 | Test at 9600, 19200, 38400, 57600, 115200 baud | All baud rates work | |
| 12.4 | Test on 1000m RS485 bus with 32 nodes | Reliable communication | |

## Test Equipment Required

| Equipment | Purpose |
|-----------|---------|
| 24V DC power supply (adjustable) | Power + brown-out test |
| Multimeter (4½ digit) | Voltage measurements |
| Modbus RTU test tool (mbpoll / Modbus Poll) | Protocol testing |
| Oscilloscope (100 MHz) | Signal quality, timing |
| Logic analyzer (8+ channels) | RS485 timing, GPIO verification |
| RS485-USB adapter | PC-to-board RS485 |
| Ethernet switch + PC | TCP testing |
| Function generator | Analog input simulation |
| DC load (electronic) | DO/AO load testing |
| Climate chamber | Temperature testing |
| ESD simulator | EMC testing |

## Sign-Off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Test Engineer | | | |
| Firmware Developer | | | |
| Hardware Engineer | | | |
| QA Manager | | | |
