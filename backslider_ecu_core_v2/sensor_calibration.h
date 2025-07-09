// sensor_calibration.h
// Sensor calibration tables and functions
//
// This file contains all the calibration data and algorithms.
// Separated so it's easy to modify calibration without touching core logic.

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include "input_manager_types.h"

// =============================================================================
// STANDARD THERMISTOR CALIBRATION TABLES
// =============================================================================

// Standard automotive thermistor (like Bosch sensors)
// Voltage points (V) - assumes 2.2kΩ pullup to 5V
extern const float STANDARD_THERMISTOR_VOLTAGE_TABLE[];
extern const float STANDARD_THERMISTOR_TEMP_TABLE[];
extern const uint8_t STANDARD_THERMISTOR_TABLE_SIZE;

// GM-style coolant temperature sensor
extern const float GM_CTS_VOLTAGE_TABLE[];
extern const float GM_CTS_TEMP_TABLE[];
extern const uint8_t GM_CTS_TABLE_SIZE;

// Generic IAT sensor (similar to CTS but might have different curve)
extern const float GENERIC_IAT_VOLTAGE_TABLE[];
extern const float GENERIC_IAT_TEMP_TABLE[];
extern const uint8_t GENERIC_IAT_TABLE_SIZE;

// =============================================================================
// CALIBRATION FUNCTIONS
// =============================================================================

// Linear calibration (for TPS, MAP, pressure sensors, etc.)
float calibrate_linear(const linear_config_t* config, float voltage);

// Thermistor calibration with lookup table interpolation
float calibrate_thermistor(const thermistor_config_t* config, float voltage);

// Digital sensor calibration (mostly just validation)
float calibrate_digital(const digital_config_t* config, uint8_t digital_value);

// Frequency sensor calibration
float calibrate_frequency(const frequency_config_t* config, uint32_t frequency_hz);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Generic table interpolation function
float interpolate_table(const float* x_table, const float* y_table, 
                       uint8_t table_size, float x_value);

// Calculate thermistor resistance from voltage and pullup
float calculate_thermistor_resistance(float voltage, uint16_t pullup_ohms, float vcc);

// Validate calibrated reading based on sensor type
uint8_t validate_calibrated_reading(sensor_type_t type, float value);

#endif