/*
 * Host-side stubs for the board IO drivers used by src/modbus.c.
 *
 * The protocol core calls these to mirror coils/registers onto physical
 * IO. In native unit tests there is no hardware, so they are no-ops.
 * This is what lets CI compile the REAL src/modbus.c with host gcc.
 */
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"

uint8_t digital_inputs_read_all(void)
{
    return 0;
}

void digital_output_write(uint8_t channel, uint8_t state)
{
    (void)channel;
    (void)state;
}

void relay_set(uint8_t channel, uint8_t state)
{
    (void)channel;
    (void)state;
}

void analog_output_write_raw(uint8_t channel, uint16_t value)
{
    (void)channel;
    (void)value;
}
