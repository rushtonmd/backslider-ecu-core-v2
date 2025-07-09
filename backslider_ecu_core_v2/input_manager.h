// input_manager.h
// Main input manager interface
//
// This provides the public API for the input manager system.
// Modules use these functions to register sensors and get status.

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "input_manager_types.h"
#include "msg_definitions.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    // Mock functions for desktop testing (only if not already defined)
    #ifndef MOCK_ARDUINO_H
        inline uint16_t analogRead(int pin) { return 2048; }
        inline uint32_t micros() { return 0; }
        inline void pinMode(int pin, int mode) {}
        #define INPUT 0
        #define INPUT_PULLUP 1
    #endif
#endif

// =============================================================================
// PUBLIC API
// =============================================================================

// Initialize the input manager system
void input_manager_init(void);

// Register sensors from a module
// Returns number of sensors successfully registered
uint8_t input_manager_register_sensors(const sensor_definition_t* sensors, uint8_t count);

// Main update function - call from main loop
void input_manager_update(void);

// =============================================================================
// STATUS AND DIAGNOSTICS
// =============================================================================

// Get system status
uint8_t input_manager_get_sensor_count(void);
uint8_t input_manager_get_valid_sensor_count(void);
uint32_t input_manager_get_total_updates(void);
uint32_t input_manager_get_total_errors(void);

// Get individual sensor status (by index)
uint8_t input_manager_get_sensor_status(uint8_t sensor_index, sensor_runtime_t* status);

// Find sensor by message ID
int8_t input_manager_find_sensor_by_msg_id(uint32_t msg_id);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Convert ADC counts to voltage (inline for performance)
static inline float adc_counts_to_voltage(uint16_t counts) {
    return (counts * ADC_VOLTAGE_REF) / ADC_RESOLUTION;
}

// Validate voltage reading
static inline uint8_t is_voltage_valid(float voltage) {
    return (voltage >= SENSOR_VOLTAGE_MIN && voltage <= SENSOR_VOLTAGE_MAX) ? 1 : 0;
}

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

// Helper macro to define a linear sensor
#define DEFINE_LINEAR_SENSOR(pin_name, msg_id_name, min_v, max_v, min_val, max_val, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_ANALOG_LINEAR, \
        .config.linear = { \
            .min_voltage = min_v, \
            .max_voltage = max_v, \
            .min_value = min_val, \
            .max_value = max_val, \
            .pullup_ohms = 0 \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 32, \
        .name = sensor_name \
    }

// Helper macro to define a thermistor sensor
#define DEFINE_THERMISTOR_SENSOR(pin_name, msg_id_name, pullup_value, v_table, t_table, size, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_THERMISTOR, \
        .config.thermistor = { \
            .pullup_ohms = pullup_value, \
            .voltage_table = v_table, \
            .temp_table = t_table, \
            .table_size = size \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 128, \
        .name = sensor_name \
    }

#endif