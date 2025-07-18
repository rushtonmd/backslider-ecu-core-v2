// sensor_calibration.cpp
// Sensor calibration implementation
//
// This file contains all the calibration algorithms and lookup tables.
// Easy to modify calibration without affecting core sensor logic.

#include "sensor_calibration.h"
#include <math.h>

// =============================================================================
// CALIBRATION LOOKUP TABLES
// =============================================================================

// Standard automotive thermistor calibration (Bosch-style)
// Assumes 2.2kΩ pullup resistor to 5V supply
const float STANDARD_THERMISTOR_VOLTAGE_TABLE[] = {
    0.25f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.5f
};

const float STANDARD_THERMISTOR_TEMP_TABLE[] = {
    120.0f, 100.0f, 80.0f, 60.0f, 40.0f, 20.0f, 0.0f, -20.0f, -40.0f, -60.0f
};

const uint8_t STANDARD_THERMISTOR_TABLE_SIZE = 
    sizeof(STANDARD_THERMISTOR_VOLTAGE_TABLE) / sizeof(float);

// GM-style coolant temperature sensor
const float GM_CTS_VOLTAGE_TABLE[] = {
    0.3f, 0.6f, 1.2f, 1.8f, 2.4f, 3.0f, 3.6f, 4.2f, 4.7f
};

const float GM_CTS_TEMP_TABLE[] = {
    130.0f, 110.0f, 85.0f, 60.0f, 35.0f, 15.0f, -5.0f, -25.0f, -40.0f
};

const uint8_t GM_CTS_TABLE_SIZE = 
    sizeof(GM_CTS_VOLTAGE_TABLE) / sizeof(float);

// Generic IAT sensor (similar characteristics to CTS)
const float GENERIC_IAT_VOLTAGE_TABLE[] = {
    0.25f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.5f
};

const float GENERIC_IAT_TEMP_TABLE[] = {
    120.0f, 100.0f, 80.0f, 60.0f, 40.0f, 20.0f, 0.0f, -20.0f, -40.0f, -60.0f
};

const uint8_t GENERIC_IAT_TABLE_SIZE = 
    sizeof(GENERIC_IAT_VOLTAGE_TABLE) / sizeof(float);

// =============================================================================
// CALIBRATION FUNCTIONS
// =============================================================================

float calibrate_linear(const linear_config_t* config, float voltage) {
    if (config == nullptr) return 0.0f;
    
    // Handle out-of-range inputs
    if (voltage <= config->min_voltage) {
        return config->min_value;
    }
    if (voltage >= config->max_voltage) {
        return config->max_value;
    }
    
    // Linear interpolation
    float voltage_range = config->max_voltage - config->min_voltage;
    float value_range = config->max_value - config->min_value;
    float ratio = (voltage - config->min_voltage) / voltage_range;
    
    return config->min_value + (ratio * value_range);
}

float calibrate_thermistor(const thermistor_config_t* config, float voltage) {
    if (config == nullptr || config->voltage_table == nullptr || config->temp_table == nullptr) {
        return 20.0f;  // Default room temperature
    }
    
    return interpolate_table(config->voltage_table, config->temp_table, 
                           config->table_size, voltage);
}

float calibrate_digital(const digital_config_t* config, uint8_t digital_value) {
    if (config == nullptr) return 0.0f;
    
    // Convert to 0 or 1 (normalize any non-zero value to 1)
    uint8_t normalized_value = digital_value ? 1 : 0;
    
    // Apply inversion if configured
    if (config->invert_logic) {
        normalized_value = !normalized_value;
    }
    
    return static_cast<float>(normalized_value);
}

float calibrate_frequency(const frequency_config_t* config, uint32_t frequency_hz) {
    if (config == nullptr) return 0.0f;
    
    // Convert frequency to meaningful units (RPM, speed, etc.)
    // For RPM: RPM = (frequency_hz * 60 seconds/minute) / pulses_per_revolution
    // For other sensors: adjust based on scaling_factor
    float base_value = (static_cast<float>(frequency_hz) * 60.0f) / config->pulses_per_unit;
    return base_value * config->scaling_factor;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

float interpolate_table(const float* x_table, const float* y_table, 
                       uint8_t table_size, float x_value) {
    if (x_table == nullptr || y_table == nullptr || table_size < 2) {
        return 0.0f;
    }
    
    // Handle edge cases
    if (x_value <= x_table[0]) {
        return y_table[0];
    }
    if (x_value >= x_table[table_size - 1]) {
        return y_table[table_size - 1];
    }
    
    // Find interpolation points
    for (uint8_t i = 0; i < table_size - 1; i++) {
        if (x_value >= x_table[i] && x_value <= x_table[i + 1]) {
            // Linear interpolation between points
            float x_range = x_table[i + 1] - x_table[i];
            float y_range = y_table[i + 1] - y_table[i];
            float ratio = (x_value - x_table[i]) / x_range;
            
            return y_table[i] + (ratio * y_range);
        }
    }
    
    // Should never reach here with valid input
    return 0.0f;
}

float calculate_thermistor_resistance(float voltage, uint16_t pullup_ohms, float vcc) {
    if (voltage <= 0.01f || voltage >= (vcc - 0.01f)) {
        return 0.0f;  // Invalid voltage reading
    }
    
    // Calculate thermistor resistance using voltage divider equation
    // V_out = V_cc * R_thermistor / (R_pullup + R_thermistor)
    // Solving for R_thermistor:
    // R_thermistor = R_pullup * V_out / (V_cc - V_out)
    
    float thermistor_resistance = pullup_ohms * voltage / (vcc - voltage);
    return thermistor_resistance;
}

uint8_t validate_calibrated_reading(sensor_type_t type, float value) {
    switch (type) {
        case SENSOR_ANALOG_LINEAR:
            // Generic validation - could be made more specific per sensor
            return (value >= -1000.0f && value <= 10000.0f) ? 1 : 0;
            
        case SENSOR_THERMISTOR:
            // Temperature range validation
            return (value >= -60.0f && value <= 200.0f) ? 1 : 0;
            
        case SENSOR_DIGITAL_PULLUP:
        case SENSOR_I2C_GPIO:
            // Digital values should be 0 or 1
            return (value == 0.0f || value == 1.0f) ? 1 : 0;
            
        case SENSOR_FREQUENCY_COUNTER:
            // Frequency-based sensors should be non-negative
            return (value >= 0.0f && value <= 50000.0f) ? 1 : 0;
            
        default:
            return 0;  // Unknown sensor type
    }
}