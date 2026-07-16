#include "digital_io.h"

static const uint16_t di_pins[DI_COUNT] = {
    DI_PIN_0, DI_PIN_1, DI_PIN_2, DI_PIN_3,
    DI_PIN_4, DI_PIN_5, DI_PIN_6, DI_PIN_7
};

static const uint16_t do_pins[DO_COUNT] = {
    DO_PIN_0, DO_PIN_1, DO_PIN_2, DO_PIN_3,
    DO_PIN_4, DO_PIN_5, DO_PIN_6, DO_PIN_7
};

static uint8_t digital_input_states = 0;
static uint8_t digital_output_states = 0;
static uint8_t di_debounce[DI_COUNT] = {0};

void digital_inputs_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    DI_CLK_ENABLE();
    gpio.Pin  = DI_PIN_MASK;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DI_PORT, &gpio);
}

void digital_inputs_scan(void)
{
    /* Symmetric integration debounce: 5 samples (~50 ms @ 10 ms scan) each way */
    uint32_t port = DI_PORT->IDR;
    uint8_t current_states = digital_input_states;

    for (uint8_t i = 0; i < DI_COUNT; i++) {
        uint8_t pin_active = !(port & di_pins[i]);

        if (pin_active) {
            if (di_debounce[i] < 5) {
                di_debounce[i]++;
            }
            if (di_debounce[i] == 5) {
                current_states |= (uint8_t)(1U << i);
            }
        } else {
            if (di_debounce[i] > 0) {
                di_debounce[i]--;
            }
            if (di_debounce[i] == 0) {
                current_states &= (uint8_t)~(1U << i);
            }
        }
    }
    digital_input_states = current_states;
}

uint8_t digital_inputs_read_all(void)
{
    return digital_input_states;
}

uint8_t digital_input_read(uint8_t channel)
{
    if (channel >= DI_COUNT) return 0;
    return (digital_input_states >> channel) & 0x01;
}

void digital_outputs_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    DO_CLK_ENABLE();
    gpio.Pin   = DO_PIN_MASK;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DO_PORT, &gpio);
    DO_PORT->BSRR = DO_PIN_MASK << 16U;
    digital_output_states = 0;
}

void digital_outputs_write_all(uint8_t value)
{
    digital_output_states = value;
    uint32_t bsrr_set = 0;
    uint32_t bsrr_reset = 0;

    for (uint8_t i = 0; i < DO_COUNT; i++) {
        if (value & (1 << i)) {
            bsrr_set |= do_pins[i];
        } else {
            bsrr_reset |= do_pins[i];
        }
    }
    DO_PORT->BSRR = bsrr_reset << 16U;
    DO_PORT->BSRR = bsrr_set;
}

void digital_output_write(uint8_t channel, uint8_t state)
{
    if (channel >= DO_COUNT) return;

    if (state) {
        digital_output_states |= (1 << channel);
        DO_PORT->BSRR = do_pins[channel];
    } else {
        digital_output_states &= ~(1 << channel);
        DO_PORT->BSRR = (uint32_t)do_pins[channel] << 16U;
    }
}

uint8_t digital_output_read(uint8_t channel)
{
    if (channel >= DO_COUNT) return 0;
    return (digital_output_states >> channel) & 0x01;
}

uint8_t digital_outputs_read_all(void)
{
    return digital_output_states;
}
