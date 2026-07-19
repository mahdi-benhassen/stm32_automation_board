# Customizing the Modbus Register Map and `modbus_sync_registers()`

This document explains how the slave's data model works, what
`modbus_sync_registers()` actually does, whether *you* need it for a custom
project, and how to customize it step by step.

**Short answer to the most common question:** `modbus_sync_registers()` is
**not** part of the Modbus protocol. It is board-specific glue that copies
values between the four Modbus data tables and this board's physical I/O
(digital outputs, relays, digital inputs, analog I/O). If your register map is
pure software data, you don't need it at all.

---

## 1. The Modbus data model (what the protocol layer owns)

The slave core (`src/modbus.c`) keeps four flat tables in RAM:

| Modbus table        | Storage (static in `modbus.c`) | Size define (in `inc/board_config.h`) | Base offset define          |
|---------------------|-------------------------------|----------------------------------------|-----------------------------|
| Coils (0x)          | `coil_bits[]` (bit-packed)    | `MODBUS_MAX_COILS` (128)               | `MODBUS_COIL_OFFSET`        |
| Discrete Inputs (1x)| `discrete_bits[]` (bit-packed)| `MODBUS_MAX_COILS` (128)               | `MODBUS_DISCRETE_INPUT_OFFSET` |
| Input Registers (3x)| `input_regs[]` (uint16)       | `MODBUS_MAX_REGISTERS` (256)           | `MODBUS_INPUT_REG_OFFSET`   |
| Holding Registers(4x)| `holding_regs[]` (uint16)    | `MODBUS_MAX_REGISTERS` (256)           | `MODBUS_HOLDING_REG_OFFSET` |

All function-code handlers (FC 0x01/0x02/0x03/0x04/0x05/0x06/0x0F/0x10/0x16/0x17)
read and write **only these tables**. A Modbus master writing coil 5 changes
`coil_bits[5]` — nothing else happens by itself. That is the key point: the
protocol layer manages the *data model*; it does not know what your addresses
*mean*.

Your application accesses the same tables through the public accessor API
(`inc/modbus.h`):

```c
uint16_t modbus_read_coil(uint16_t addr);
void     modbus_write_coil(uint16_t addr, uint16_t value);
uint16_t modbus_read_discrete_input(uint16_t addr);
uint16_t modbus_read_input_register(uint16_t addr);
uint16_t modbus_read_holding_register(uint16_t addr);
void     modbus_write_holding_register(uint16_t addr, uint16_t value);
```

## 2. What `modbus_sync_registers()` does on this board

It is a `static` function in `src/modbus.c` with four blocks:

```c
static void modbus_sync_registers(void)
{
    /* (A) Modbus coils  -> physical digital outputs */
    for (uint8_t i = 0; i < DO_COUNT; i++)
        digital_output_write(i, modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + i));

    /* (B) Modbus coils 8..8+RELAY_COUNT -> physical relays */
    for (uint8_t i = 0; i < RELAY_COUNT; i++)
        relay_set(i, modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + 8 + i));

    /* (C) physical digital inputs -> Modbus discrete inputs */
    uint8_t di = digital_inputs_read_all();
    for (uint8_t i = 0; i < DI_COUNT; i++)
        modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i, (di >> i) & 0x01);

    /* (D) holding regs 100.. -> input regs (analog-in mirror) */
    for (uint8_t i = 0; i < AI_COUNT; i++)
        input_regs[MODBUS_INPUT_REG_OFFSET + i] =
            holding_regs[MODBUS_HOLDING_REG_OFFSET + 100 + i];

    /* (E) holding regs 0..AO_COUNT -> physical analog outputs */
    for (uint8_t i = 0; i < AO_COUNT; i++)
        analog_output_write_raw(i, holding_regs[MODBUS_HOLDING_REG_OFFSET + i]);
}
```

Direction summary:

| Block | Direction                     | Trigger it answers                          |
|-------|-------------------------------|---------------------------------------------|
| A, B  | Modbus table → hardware       | "A master wrote a coil — drive the pin"     |
| E     | Modbus table → hardware       | "A master wrote a holding reg — set the DAC"|
| C     | hardware → Modbus table       | "A master may read DIs — refresh them first"|
| D     | app area → input registers    | "Publish internal values as readable 3x regs"|

**When it runs:** it is called from the FC handlers *after every successful
write* (FC 0x05, 0x06, 0x0F, 0x10, 0x15, 0x16, 0x17), so a master's write takes
effect on the hardware immediately, inside the same request/response cycle.
There is also a public `modbus_sync_inputs()` (blocks C+D only) that the main
loop / RTOS task can call periodically so input-side tables stay fresh between
requests.

## 3. Do you need it? Decision guide

**Case 1 — your register map is pure software data** (setpoints, parameters,
status words your own firmware produces/consumes):
**No sync function is needed.** Read and write the tables with the accessor
API from your application code, wherever and whenever you want:

```c
/* your control loop */
uint16_t setpoint = modbus_read_holding_register(10);   /* written by master */
modbus_write_holding_register(20, measured_temperature); /* readable by master */
```

You can delete the `modbus_sync_registers()` calls (or leave the function
body empty) — the protocol layer is fully functional without it.

**Case 2 — some addresses must reflect or drive real hardware** (GPIO, ADC,
DAC, relays, UART-attached devices):
**Yes, keep a sync function — but rewrite its body for your board.** Replace
the `digital_output_write()` / `relay_set()` / `digital_inputs_read_all()` /
`analog_output_write_raw()` calls with your own drivers, and define your own
address mapping.

**Case 3 — mixed:** keep sync only for the hardware-bound ranges and use the
accessor API for everything else.

## 4. How to customize it (recipe)

1. **Define your map first.** Write down which Modbus addresses mean what, e.g.:

   | Address (protocol) | Table   | Meaning            | Direction        |
   |--------------------|---------|--------------------|------------------|
   | 0–7                | Coil    | GPIO outputs PC0–7 | master → pin     |
   | 0–3                | Discrete| GPIO inputs PE0–3  | pin → master     |
   | 0                  | Holding | PWM duty cycle     | master → TIM1    |
   | 0–1                | Input   | ADC channels 0–1   | ADC → master     |

2. **Size the tables** in `inc/board_config.h` (`MODBUS_MAX_COILS`,
   `MODBUS_MAX_REGISTERS`) and set the base offsets if you don't want the map
   to start at 0.

3. **Rewrite the body** of `modbus_sync_registers()` with your drivers:

   ```c
   static void modbus_sync_registers(void)
   {
       /* master -> hardware (public accessors work fine here) */
       for (uint8_t i = 0; i < 8; i++)
           gpio_write(GPIOC, i, modbus_read_coil(i));
       pwm_set_duty(modbus_read_holding_register(0));

       /* hardware -> master (internal helpers: you are inside modbus.c) */
       for (uint8_t i = 0; i < 4; i++)
           modbus_bit_write(discrete_bits, i, gpio_read(GPIOE, i));
       input_regs[0] = adc_read(0);
       input_regs[1] = adc_read(1);
   }
   ```

   **Note:** the public API intentionally has *readers* for discrete inputs /
   input registers but no *writers* — those tables are meant to be refreshed
   from inside the slave (the static helpers above, or `modbus_sync_inputs()`).
   Application code that must publish a value from outside `modbus.c` can
   either write it to a holding register (readable by the master via FC 0x03),
   mirror it into input registers the way the stock firmware mirrors
   `holding_regs[100+i]` → `input_regs[i]`, or add a small public writer
   wrapper in `modbus.c`/`modbus.h`.

4. **Keep it fast and non-blocking.** On the bare-metal `main` branch it runs
   inside the request/response cycle; on `feature/freertos-integration` it runs
   while holding `modbus_mutex`. Never wait on slow peripherals inside it —
   sample slow inputs (ADC, external buses) in your main loop / a task and just
   copy the latest values here.

5. **Choose your refresh strategy for inputs:**
   - *On-request* (what the stock firmware does): refresh input tables inside
     sync, so every read sees fresh data. Simple; adds latency per request.
   - *Periodic*: call `modbus_sync_inputs()` (or your own C/D-only function)
     from the main loop / a low-priority task every N ms, and keep only the
     output blocks (A/B/E) in `modbus_sync_registers()`. Best for slow inputs.

6. **Sparse or computed mappings:** you are not limited to loops — a
   `switch (addr)` or a lookup-table struct (`{modbus_addr, gpio_port, pin}`)
   is fine. The only contract is: after a write FC, hardware state matches the
   tables; before a read FC, the tables match reality.

## 5. Customization without editing `modbus.c`

If you want to keep `src/modbus.c` untouched across upgrades, move your mapping
into a separate file: declare a weak hook or a registered callback in your own
code, e.g.

```c
/* in modbus.c — default does nothing */
__weak void modbus_app_sync(void) { }

/* called from modbus_sync_registers() */
static void modbus_sync_registers(void) { modbus_app_sync(); }

/* in your app file — strong definition overrides the weak one */
void modbus_app_sync(void) { /* your mapping here */ }
```

That keeps all board knowledge out of the protocol file. (The stock firmware
currently keeps the mapping in `modbus.c` directly for simplicity — both
patterns are valid.)

## 6. Summary

- The four Modbus tables live in `modbus.c`; FC handlers only touch the tables.
- `modbus_sync_registers()` = application glue, not protocol. Customize or
  delete it freely.
- Pure software map → use `modbus_read_holding_register()` /
  `modbus_write_holding_register()` / `modbus_write_coil()`, no sync.
- Hardware-bound map → rewrite the sync body with your drivers, keep it fast,
  and pick on-request vs. periodic input refresh.
- Sizes and base offsets are in `inc/board_config.h`.
