/**
 * @file custom_register_map_example.h
 * @brief Example of custom Modbus register mapping for STM32 / embedded platforms.
 *
 * This example demonstrates how to define custom coils, discrete inputs,
 * input registers, and holding registers without modifying core Modbus files.
 */

#ifndef CUSTOM_REGISTER_MAP_EXAMPLE_H
#define CUSTOM_REGISTER_MAP_EXAMPLE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Custom Application Register Map Offsets:
 * You can organize your addresses logically by function.
 */

/* Coils (0x) - Read / Write bits */
#define APP_COIL_MOTOR_ENABLE       0U   /* 1 = Motor run, 0 = Motor stop */
#define APP_COIL_ALARM_RESET        1U   /* Write 1 to clear active alarm */
#define APP_COIL_AUTO_MODE          2U   /* 1 = Automatic, 0 = Manual */
#define APP_COIL_COOLING_VALVE      3U   /* 1 = Open valve, 0 = Close */

/* Discrete Inputs (1x) - Read-only bits */
#define APP_DI_EMERGENCY_STOP       0U   /* 1 = E-stop pressed */
#define APP_DI_DOOR_INTERLOCK       1U   /* 1 = Safety door closed */
#define APP_DI_PRESSURE_SWITCH      2U   /* 1 = Pressure OK */
#define APP_DI_MOTOR_FAULT          3U   /* 1 = Thermal overload tripped */

/* Input Registers (3x) - Read-only 16-bit analog / telemetry values */
#define APP_IR_TEMPERATURE_RAW      0U   /* Temperature sensor (0.1 deg C) */
#define APP_IR_PRESSURE_MBAR        1U   /* Line pressure (mbar) */
#define APP_IR_MOTOR_CURRENT_MA     2U   /* Motor load current (mA) */
#define APP_IR_SUPPLY_VOLTAGE_MV    3U   /* DC bus voltage (mV) */

/* Holding Registers (4x) - Read / Write 16-bit parameters and setpoints */
#define APP_HR_TARGET_TEMP_SP       0U   /* Target temperature setpoint (0.1 deg C) */
#define APP_HR_MAX_PRESSURE_LIMIT   1U   /* Max allowable pressure (mbar) */
#define APP_HR_PID_KP               2U   /* PID Proportional gain */
#define APP_HR_PID_KI               3U   /* PID Integral gain */
#define APP_HR_PID_KD               4U   /* PID Derivative gain */

/**
 * Application state structure representing physical or software variables.
 */
typedef struct {
    /* Setpoints & Controls */
    bool     motor_enabled;
    bool     alarm_reset_requested;
    bool     auto_mode;
    bool     cooling_valve_open;
    uint16_t target_temp_sp;
    uint16_t max_pressure_limit;
    uint16_t pid_kp;
    uint16_t pid_ki;
    uint16_t pid_kd;

    /* Telemetry & Inputs */
    bool     e_stop_active;
    bool     door_closed;
    bool     pressure_ok;
    bool     motor_fault;
    uint16_t temperature_raw;
    uint16_t pressure_mbar;
    uint16_t motor_current_ma;
    uint16_t supply_voltage_mv;
} app_process_data_t;

/**
 * Initialize custom register mapping and bind synchronization hooks.
 * @param app_data Pointer to application process data structure.
 */
void app_custom_register_map_init(app_process_data_t *app_data);

/**
 * Periodic update function to be called from main loop or RTOS task.
 * Updates internal sensors and refreshes Modbus input tables.
 */
void app_custom_process_step(app_process_data_t *app_data);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_REGISTER_MAP_EXAMPLE_H */
