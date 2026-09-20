/**
 * @file custom_register_map_example.c
 * @brief Implementation of custom Modbus register mapping using hooks.
 *
 * Shows how to bind custom application logic to Modbus write/read cycles
 * without modifying any core stack files.
 */

#include "custom_register_map_example.h"
#include "modbus.h"
#include <string.h>

static app_process_data_t *s_app_data = NULL;

/**
 * Called automatically by the Modbus engine whenever a master writes to coils
 * or holding registers (FC 0x05, 0x06, 0x0F, 0x10, 0x15, 0x16, 0x17).
 */
static void on_modbus_registers_written(void *user_ctx)
{
    app_process_data_t *data = (app_process_data_t *)user_ctx;
    if (data == NULL) {
        return;
    }

    /* 1. Update coil-driven controls */
    data->motor_enabled         = (modbus_read_coil(APP_COIL_MOTOR_ENABLE) != 0);
    data->alarm_reset_requested = (modbus_read_coil(APP_COIL_ALARM_RESET) != 0);
    data->auto_mode             = (modbus_read_coil(APP_COIL_AUTO_MODE) != 0);
    data->cooling_valve_open    = (modbus_read_coil(APP_COIL_COOLING_VALVE) != 0);

    /* If alarm reset was pulsed by master, clear it */
    if (data->alarm_reset_requested) {
        modbus_write_coil(APP_COIL_ALARM_RESET, 0);
        data->alarm_reset_requested = false;
    }

    /* 2. Update holding register parameters / setpoints */
    data->target_temp_sp     = modbus_read_holding_register(APP_HR_TARGET_TEMP_SP);
    data->max_pressure_limit = modbus_read_holding_register(APP_HR_MAX_PRESSURE_LIMIT);
    data->pid_kp             = modbus_read_holding_register(APP_HR_PID_KP);
    data->pid_ki             = modbus_read_holding_register(APP_HR_PID_KI);
    data->pid_kd             = modbus_read_holding_register(APP_HR_PID_KD);

    /* 3. Here you can directly drive physical actuators:
     * e.g., HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, data->motor_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
     */
}

/**
 * Called automatically before responding to input-reading commands or
 * when modbus_sync_inputs() is explicitly called by your main loop.
 */
static void on_modbus_inputs_refresh(void *user_ctx)
{
    app_process_data_t *data = (app_process_data_t *)user_ctx;
    if (data == NULL) {
        return;
    }

    /* 1. Write Discrete Inputs (1x) into Modbus memory */
    /* Note: Discrete inputs are bit-packed, accessed via internal tables or holding registers */
    /* If publishing hardware status words to holding registers: */
    modbus_write_holding_register(100 + APP_IR_TEMPERATURE_RAW, data->temperature_raw);
    modbus_write_holding_register(100 + APP_IR_PRESSURE_MBAR, data->pressure_mbar);
    modbus_write_holding_register(100 + APP_IR_MOTOR_CURRENT_MA, data->motor_current_ma);
    modbus_write_holding_register(100 + APP_IR_SUPPLY_VOLTAGE_MV, data->supply_voltage_mv);
}

void app_custom_register_map_init(app_process_data_t *app_data)
{
    s_app_data = app_data;

    /* Initialize default parameter values */
    if (s_app_data) {
        memset(s_app_data, 0, sizeof(*s_app_data));
        s_app_data->target_temp_sp = 250U;       /* 25.0 deg C */
        s_app_data->max_pressure_limit = 5000U;  /* 5000 mbar */
        s_app_data->pid_kp = 120U;
        s_app_data->pid_ki = 15U;
        s_app_data->pid_kd = 45U;

        /* Write default setpoints to Modbus holding registers */
        modbus_write_holding_register(APP_HR_TARGET_TEMP_SP, s_app_data->target_temp_sp);
        modbus_write_holding_register(APP_HR_MAX_PRESSURE_LIMIT, s_app_data->max_pressure_limit);
        modbus_write_holding_register(APP_HR_PID_KP, s_app_data->pid_kp);
        modbus_write_holding_register(APP_HR_PID_KI, s_app_data->pid_ki);
        modbus_write_holding_register(APP_HR_PID_KD, s_app_data->pid_kd);
    }

    /* Register custom hooks with the core Modbus engine */
    static const modbus_sync_hooks_t hooks = {
        .on_registers_written = on_modbus_registers_written,
        .on_inputs_refresh    = on_modbus_inputs_refresh,
        .user_ctx             = (void *)s_app_data
    };
    modbus_register_sync_hooks(&hooks);
}

void app_custom_process_step(app_process_data_t *app_data)
{
    if (app_data == NULL) {
        return;
    }

    /* Simulate or read physical sensors (ADC / GPIO / I2C) */
    /* e.g., app_data->temperature_raw = read_adc_channel(0); */
    app_data->temperature_raw = 248U; /* 24.8 deg C */
    app_data->pressure_mbar = 1013U;
    app_data->motor_current_ma = app_data->motor_enabled ? 1450U : 0U;
    app_data->supply_voltage_mv = 24120U; /* 24.12 V */

    /* Refresh Modbus input tables */
    modbus_sync_inputs();
}
