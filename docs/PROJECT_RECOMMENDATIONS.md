# STM32 Project Recommendations — Smart IoT & Industrial Control

Recommended projects building on the STM32 Automation Board codebase.

---

## Smart Home & Lighting

### 1. DALI Gateway Controller (STM32F407)

DALI bus master for smart lighting control.

| Aspect | Details |
|--------|---------|
| **Protocol** | DALI (Manchester encoding, 1200 bps) via USART |
| **Capacity** | Up to 64 DALI ballasts/LED drivers per bus |
| **Interface** | Web dashboard (Ethernet) for room/scene/time scheduling |
| **Integration** | Modbus RTU bridge for BMS/SCADA |
| **Reuses** | RS485 + Ethernet stack, FreeRTOS tasks |
| **Adds** | DALI physical layer (USART TX with Manchester), DALI command set |

---

### 2. KNX/Modbus Smart Bridge

Bidirectional protocol translator for building automation.

| Aspect | Details |
|--------|---------|
| **KNX** | TPUART (Twisted Pair UART, 9600 bps) physical interface |
| **Mapping** | KNX group objects ↔ Modbus coils/registers (configurable table) |
| **Storage** | NVRAM/eeprom for persistent scenes and schedules |
| **Reuses** | FreeRTOS tasks + Modbus engine, RS485 driver |
| **Adds** | KNX TPUART driver, object-to-register mapping table |

---

## Energy & Power Monitoring

### 3. 3-Phase Power Analyzer (STM32F407)

Real-time electrical parameter measurement and logging.

| Aspect | Details |
|--------|---------|
| **Inputs** | 3× CT (current) + 3× voltage dividers → 6 ADC channels, synchronized via DMA |
| **Measurements** | Vrms, Irms, kW, kVAR, kVA, power factor, THD, frequency, kWh |
| **Analysis** | Software FFT or zero-crossing detection |
| **Storage** | SD card logging (FATFS on SPI) with RTC timestamps |
| **Protocol** | Modbus RTU + TCP to SCADA |
| **Reuses** | ADC driver, Modbus stack, Ethernet |
| **Adds** | 6-channel DMA ADC, RMS/FFT computation, FATFS filesystem |

---

### 4. Energy Data Concentrator

Multi-protocol gateway collecting data from sub-meters.

| Aspect | Details |
|--------|---------|
| **Downstream** | RS485 Modbus master polling electricity/water/gas meters |
| **Upstream** | Ethernet Modbus TCP server to SCADA, MQTT client to cloud |
| **Storage** | 30-day rolling buffer in external SPI flash |
| **Cloud** | MQTT (paho.mqtt.embedded) over lwIP for AWS IoT / Azure |
| **Reuses** | Modbus RTU engine (master mode), RS485, Ethernet |
| **Adds** | Modbus master polling scheduler, MQTT client, lwIP TCP/IP stack |

---

## Industrial & Building Automation

### 5. VFD Controller & Motor Management

Variable frequency drive control with PID regulation.

| Aspect | Details |
|--------|---------|
| **Outputs** | 2× AO (0-10V speed reference), 2× relays (fwd/rev contactors) |
| **Inputs** | 4× DI (run/stop/feedback/estop), 2× AI (4-20mA process variable) |
| **Control** | Software PID loop with auto-tune, ramp profiles, emergency stop |
| **Features** | Acceleration/deceleration curves, motor current monitoring, fault logging |
| **Reuses** | AO/DO/relay drivers, analog I/O, FreeRTOS tasks |
| **Adds** | PID controller (floating-point or fixed-point), ramp generator, state machine |

---

### 6. Multi-Channel Temperature Controller

Industrial temperature regulation with PID auto-tuning.

| Aspect | Details |
|--------|---------|
| **Sensors** | 8× thermocouple (MAX31855) or PT100 (MAX31865) over SPI |
| **Outputs** | 4× PID channels → AO (0-10V SSR drive) or PWM/relay |
| **Registers** | Setpoint, PV, PID params (Kp, Ki, Kd), output%, auto-tune status |
| **Dashboard** | Web UI with real-time temperature trend charts via Ethernet |
| **Reuses** | Relay control, analog outputs, Modbus, Ethernet |
| **Adds** | MAX31855/MAX31865 SPI driver, PID + auto-tune (Cohen-Coon/Ziegler-Nichols), lwIP HTTP server |

---

## Protocol Bridge & Gateway

### 7. Modbus RTU ↔ Modbus TCP Bridge (Transparent)

Turn any RS485 Modbus device into a network-accessible one.

| Aspect | Details |
|--------|---------|
| **Function** | Transparent frame-level bridge between RS485 RTU and Ethernet TCP |
| **Operation** | TCP client sends MBAP → strip → forward as RTU → wait reply → wrap MBAP → return |
| **Routing** | Multiple RTU slaves behind one gateway, routed by unit ID in MBAP header |
| **Value** | Enables legacy RS485-only equipment on Ethernet networks with zero device changes |
| **Reuses** | All existing Modbus RTU + TCP + RS485 + Ethernet code |
| **Adds** | Modbus master polling, address-based routing table, response timeout handling |

---

### 8. OPC UA Server on STM32F407

Industry 4.0 gateway — expose I/O as standardized OPC UA nodes.

| Aspect | Details |
|--------|---------|
| **Library** | open62541 ported to FreeRTOS + lwIP |
| **Nodes** | All DO/DI/AI/AO/relay registers as browseable OPC UA nodes |
| **Features** | Read, write, subscription (publish on change), method calls |
| **Integration** | Companion to Modbus — SCADA speaks OPC UA natively |
| **Reuses** | All I/O drivers, FreeRTOS, Ethernet |
| **Adds** | open62541 integration, OPC UA node model, subscriptions engine |

---

## Agriculture & Environment

### 9. Smart Greenhouse Controller

Automated climate and irrigation management.

| Aspect | Details |
|--------|---------|
| **Sensors** | DHT22/SHT31 (temp/humidity via I2C/1-Wire), 4× soil moisture (analog) |
| **Actuators** | 2× relays (irrigation pump, exhaust fan), 2× AO (LED dimming, vent position) |
| **Logic** | Threshold-based rule engine: "moisture < 30% → open irrigation for 60s" |
| **Network** | Modbus + MQTT for remote dashboards and alerts |
| **Reuses** | AI/AO/relay drivers, Modbus, FreeRTOS tasks |
| **Adds** | DHT22/SHT31 I2C driver, rule engine (priority-ordered conditions), MQTT |

---

### 10. Weather Station Gateway

Environmental field monitoring with cloud upload.

| Aspect | Details |
|--------|---------|
| **Sensors** | RS485 Modbus master polling: anemometer, pyranometer, rain gauge, barometer |
| **Local** | BME280 (temp/pressure/humidity I2C) + BH1750 (lux I2C) |
| **Logging** | SD card CSV logging with RTC timestamps |
| **API** | HTTP REST API via lwIP for dashboard queries |
| **Reuses** | RS485 Modbus master, Ethernet, FreeRTOS |
| **Adds** | BME280/BH1750 I2C drivers, lwIP HTTP server, REST endpoint routing, FATFS |

---

## Getting Started — Recommended Order

| Priority | Project | Difficulty | New Code | Why First |
|----------|---------|------------|----------|-----------|
| **1st** | #7 RTU↔TCP Bridge | Low | ~200 lines | Minimal new code, maximum value. Turns your slave into a gateway. |
| **2nd** | #3 Power Analyzer | Medium | ~800 lines | Uses existing ADC + DMA. Adds FFT/calculation layer. |
| **3rd** | #5 VFD Controller | Medium | ~500 lines | Uses existing AO + relay code. Adds PID loop. |
| **4th** | #9 Greenhouse | Low | ~400 lines | New sensor drivers + simple rules. Good IoT portfolio piece. |
| **5th** | #8 OPC UA Server | High | ~2000 lines | Most valuable for Industry 4.0 but complex library integration. |
