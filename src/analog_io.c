#include "analog_io.h"

static ADC_HandleTypeDef hadc1;
static DAC_HandleTypeDef hdac;
static uint16_t ai_raw_values[AI_COUNT] = {0};

void analog_inputs_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    AI_ADC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = AI0_GPIO_PIN;
    HAL_GPIO_Init(AI0_GPIO_PORT, &gpio);
    gpio.Pin = AI1_GPIO_PIN;
    HAL_GPIO_Init(AI1_GPIO_PORT, &gpio);
    gpio.Pin = AI2_GPIO_PIN;
    HAL_GPIO_Init(AI2_GPIO_PORT, &gpio);
    gpio.Pin = AI3_GPIO_PIN;
    HAL_GPIO_Init(AI3_GPIO_PORT, &gpio);

    hadc1.Instance                   = AI_ADC_INSTANCE;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = AI_COUNT;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);
}

void analog_inputs_scan_all(uint16_t *buffer)
{
    for (uint8_t i = 0; i < AI_COUNT; i++) {
        /* Fallback to last good value if this conversion fails/times out */
        buffer[i] = ai_raw_values[i];

        ADC_ChannelConfTypeDef sConfig = {0};
        uint8_t channel = 0;

        if (i == 0) channel = AI0_ADC_CHANNEL;
        else if (i == 1) channel = AI1_ADC_CHANNEL;
        else if (i == 2) channel = AI2_ADC_CHANNEL;
        else channel = AI3_ADC_CHANNEL;

        sConfig.Channel      = channel;
        sConfig.Rank         = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
            buffer[i] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    for (uint8_t i = 0; i < AI_COUNT; i++) {
        ai_raw_values[i] = buffer[i];
    }
}

uint16_t analog_input_read_raw(uint8_t channel)
{
    if (channel >= AI_COUNT) return 0;
    return ai_raw_values[channel];
}

float analog_input_read_voltage(uint8_t channel)
{
    if (channel >= AI_COUNT) return 0.0f;
    float v_adc = ((float)ai_raw_values[channel] / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    return v_adc * AI_VOLTAGE_SCALE;
}

float analog_input_read_eng_unit(uint8_t channel)
{
    return analog_input_read_voltage(channel);
}

void analog_outputs_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    AO_DAC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = AO0_GPIO_PIN;
    HAL_GPIO_Init(AO0_GPIO_PORT, &gpio);
    gpio.Pin = AO1_GPIO_PIN;
    HAL_GPIO_Init(AO1_GPIO_PORT, &gpio);

    hdac.Instance = AO_DAC;
    HAL_DAC_Init(&hdac);

    DAC_ChannelConfTypeDef dac_ch = {0};
    dac_ch.DAC_Trigger      = DAC_TRIGGER_NONE;
    dac_ch.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &dac_ch, AO0_DAC_CHANNEL);
    HAL_DAC_ConfigChannel(&hdac, &dac_ch, AO1_DAC_CHANNEL);

    analog_output_write_raw(0, 0);
    analog_output_write_raw(1, 0);
}

void analog_output_write_raw(uint8_t channel, uint16_t value)
{
    if (channel >= AO_COUNT) return;

    if (channel == 0) {
        HAL_DAC_SetValue(&hdac, AO0_DAC_CHANNEL, DAC_ALIGN_12B_R, value);
        HAL_DAC_Start(&hdac, AO0_DAC_CHANNEL);
    } else {
        HAL_DAC_SetValue(&hdac, AO1_DAC_CHANNEL, DAC_ALIGN_12B_R, value);
        HAL_DAC_Start(&hdac, AO1_DAC_CHANNEL);
    }
}

void analog_output_write_voltage(uint8_t channel, float voltage)
{
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 10.0f) voltage = 10.0f;

    float v_dac = voltage / AI_VOLTAGE_SCALE;
    uint16_t raw = (uint16_t)((v_dac / ADC_REF_VOLTAGE) * (float)DAC_RESOLUTION);
    if (raw > DAC_RESOLUTION) raw = DAC_RESOLUTION;

    analog_output_write_raw(channel, raw);
}

aio_status_t analog_output_write_eng_unit(uint8_t channel, float value)
{
    if (channel >= AO_COUNT) return AIO_INVALID_CHANNEL;
    analog_output_write_voltage(channel, value);
    return AIO_OK;
}
