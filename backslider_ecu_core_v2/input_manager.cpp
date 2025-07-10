// input_manager.cpp
// Core input manager implementation
//
// This file contains the main logic for sensor management.
// Kept focused on the core update loop and sensor registration.

#include "input_manager.h"
#include "sensor_calibration.h"
#include "msg_bus.h"

// =============================================================================
// PRIVATE DATA
// =============================================================================

// Registered sensors
static sensor_definition_t sensors[MAX_SENSORS];
static sensor_runtime_t sensor_runtime[MAX_SENSORS];
static uint8_t sensor_count = 0;

// Statistics
static uint32_t total_updates = 0;
static uint32_t total_errors = 0;

// =============================================================================
// PRIVATE FUNCTION DECLARATIONS
// =============================================================================

static void update_single_sensor(uint8_t sensor_index);
static void configure_sensor_pin(const sensor_definition_t* sensor);
static float apply_sensor_filtering(uint8_t sensor_index, float new_value);
static void publish_sensor_value(uint32_t msg_id, float value);
static void handle_sensor_error(uint8_t sensor_index);

// =============================================================================
// PUBLIC FUNCTIONS
// =============================================================================

void input_manager_init(void) {
    // Reset all data
    sensor_count = 0;
    total_updates = 0;
    total_errors = 0;
    
    // Initialize runtime data
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        sensor_runtime[i].calibrated_value = 0.0f;
        sensor_runtime[i].raw_voltage = 0.0f;
        sensor_runtime[i].raw_counts = 0;
        sensor_runtime[i].last_update_us = 0;
        sensor_runtime[i].update_count = 0;
        sensor_runtime[i].is_valid = 0;
        sensor_runtime[i].error_count = 0;
        sensor_runtime[i].first_reading = 1;
    }
    
    #ifdef ARDUINO
    // Configure ADC for performance
    analogReadResolution(12);
    analogReadAveraging(1);  // No averaging - we'll do our own filtering
    
    Serial.println("InputManager: Initialized");
    #endif
}

uint8_t input_manager_register_sensors(const sensor_definition_t* new_sensors, uint8_t count) {
    uint8_t registered = 0;
    
    for (uint8_t i = 0; i < count && sensor_count < MAX_SENSORS; i++) {
        // Copy sensor definition
        sensors[sensor_count] = new_sensors[i];
        
        // Configure the pin
        configure_sensor_pin(&sensors[sensor_count]);
        
        // Reset runtime data for this sensor
        sensor_runtime[sensor_count].calibrated_value = 0.0f;
        sensor_runtime[sensor_count].last_update_us = 0;
        sensor_runtime[sensor_count].is_valid = 0;
        sensor_runtime[sensor_count].first_reading = 1;
        
        #ifdef ARDUINO
        Serial.print("InputManager: Registered sensor '");
        Serial.print(sensors[sensor_count].name);
        Serial.print("' on pin ");
        Serial.print(sensors[sensor_count].pin);
        Serial.print(" with msg_id 0x");
        Serial.println(sensors[sensor_count].msg_id, HEX);
        #endif
        
        sensor_count++;
        registered++;
    }
    
    return registered;
}

void input_manager_update(void) {
    uint32_t now_us = micros();
    
    // Remove these debug lines:
    // #ifndef ARDUINO
    // std::cout << "DEBUG: input_manager_update() starting, current time: " << now_us << "us" << std::endl;
    // std::cout << "DEBUG: Processing " << (int)sensor_count << " sensors" << std::endl;
    // #endif
    
    // Update each sensor on its own schedule
    for (uint8_t i = 0; i < sensor_count; i++) {
        uint32_t time_since_last = now_us - sensor_runtime[i].last_update_us;
        
        // Remove these debug lines too:
        // #ifndef ARDUINO
        // std::cout << "DEBUG: Sensor " << (int)i << " - time since last: " << time_since_last 
        //           << "us, interval: " << sensors[i].update_interval_us << "us" << std::endl;
        // #endif
        
        if (time_since_last >= sensors[i].update_interval_us) {
            // Remove debug line:
            // #ifndef ARDUINO
            // std::cout << "DEBUG: Updating sensor " << (int)i << " (pin " << sensors[i].pin << ")" << std::endl;
            // #endif
            
            update_single_sensor(i);
            sensor_runtime[i].last_update_us = now_us;
            total_updates++;
            
            // Remove debug line:
            // #ifndef ARDUINO
            // std::cout << "DEBUG: Sensor " << (int)i << " updated, total_updates: " << total_updates << std::endl;
            // #endif
        }
    }
    
    // Remove debug line:
    // #ifndef ARDUINO
    // std::cout << "DEBUG: input_manager_update() completed" << std::endl;
    // #endif
}

// =============================================================================
// STATUS FUNCTIONS
// =============================================================================

uint8_t input_manager_get_sensor_count(void) {
    return sensor_count;
}

uint8_t input_manager_get_valid_sensor_count(void) {
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensor_runtime[i].is_valid) {
            valid_count++;
        }
    }
    return valid_count;
}

uint32_t input_manager_get_total_updates(void) {
    return total_updates;
}

uint32_t input_manager_get_total_errors(void) {
    return total_errors;
}

uint8_t input_manager_get_sensor_status(uint8_t sensor_index, sensor_runtime_t* status) {
    if (sensor_index >= sensor_count || status == NULL) {
        return 0;  // Invalid index or null pointer
    }
    
    *status = sensor_runtime[sensor_index];
    return 1;  // Success
}

int8_t input_manager_find_sensor_by_msg_id(uint32_t msg_id) {
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i].msg_id == msg_id) {
            return i;
        }
    }
    return -1;  // Not found
}

// =============================================================================
// PRIVATE FUNCTIONS
// =============================================================================

static void update_single_sensor(uint8_t sensor_index) {
    const sensor_definition_t* sensor = &sensors[sensor_index];
    sensor_runtime_t* runtime = &sensor_runtime[sensor_index];
    
    // Read raw sensor data
    if (sensor->type == SENSOR_DIGITAL_PULLUP) {
        #ifdef ARDUINO
        uint8_t digital_value = digitalRead(sensor->pin);
        if (sensor->config.digital.invert_logic) {
            digital_value = !digital_value;
        }
        runtime->raw_counts = digital_value;
        runtime->raw_voltage = digital_value ? ADC_VOLTAGE_REF : 0.0f;
        #else
        runtime->raw_counts = 1;  // Mock reading
        runtime->raw_voltage = ADC_VOLTAGE_REF;
        #endif
    } else {
        // Analog sensors
        #ifdef ARDUINO
        runtime->raw_counts = analogRead(sensor->pin);
        #else
        //runtime->raw_counts = 2048;  // Mock 12-bit reading
        runtime->raw_counts = analogRead(sensor->pin);  // Use mock reading
        #endif
        runtime->raw_voltage = adc_counts_to_voltage(runtime->raw_counts);
    }
    
    // Validate raw reading
    if (!is_voltage_valid(runtime->raw_voltage) && sensor->type != SENSOR_DIGITAL_PULLUP) {
        handle_sensor_error(sensor_index);
        return;
    }
    
    // Calibrate reading based on sensor type
    float calibrated_value;
    switch (sensor->type) {
        case SENSOR_ANALOG_LINEAR:
            calibrated_value = calibrate_linear(&sensor->config.linear, runtime->raw_voltage);
            break;
            
        case SENSOR_THERMISTOR:
            calibrated_value = calibrate_thermistor(&sensor->config.thermistor, runtime->raw_voltage);
            break;
            
        case SENSOR_DIGITAL_PULLUP:
            calibrated_value = calibrate_digital(&sensor->config.digital, runtime->raw_counts);
            break;
            
        case SENSOR_FREQUENCY_COUNTER:
            // Frequency sensors need special handling (not implemented yet)
            calibrated_value = 0.0f;
            break;
            
        default:
            calibrated_value = 0.0f;
            break;
    }
    
    // Apply filtering
    calibrated_value = apply_sensor_filtering(sensor_index, calibrated_value);
    
    // Validate calibrated reading
    if (!validate_calibrated_reading(sensor->type, calibrated_value)) {
        handle_sensor_error(sensor_index);
        return;
    }
    
    // Update runtime data
    runtime->calibrated_value = calibrated_value;
    runtime->is_valid = 1;
    runtime->error_count = 0;
    runtime->update_count++;
    
    // Publish to message bus
    publish_sensor_value(sensor->msg_id, calibrated_value);
}

static void configure_sensor_pin(const sensor_definition_t* sensor) {
    #ifdef ARDUINO
    switch (sensor->type) {
        case SENSOR_ANALOG_LINEAR:
        case SENSOR_THERMISTOR:
            // Analog pins don't need pinMode configuration
            break;
            
        case SENSOR_DIGITAL_PULLUP:
            if (sensor->config.digital.use_pullup) {
                pinMode(sensor->pin, INPUT_PULLUP);
            } else {
                pinMode(sensor->pin, INPUT);
            }
            break;
            
        case SENSOR_FREQUENCY_COUNTER:
            pinMode(sensor->pin, INPUT);
            break;
    }
    #endif
}

static float apply_sensor_filtering(uint8_t sensor_index, float new_value) {
    sensor_runtime_t* runtime = &sensor_runtime[sensor_index];
    const sensor_definition_t* sensor = &sensors[sensor_index];
    
    if (runtime->first_reading) {
        runtime->first_reading = 0;
        return new_value;  // No filtering on first reading
    }
    
    // Simple low-pass filter based on filter_strength
    // filter_strength: 0 = no filtering, 255 = maximum filtering
    float alpha = (255.0f - sensor->filter_strength) / 255.0f;
    
    return (alpha * new_value) + ((1.0f - alpha) * runtime->calibrated_value);
}

static void publish_sensor_value(uint32_t msg_id, float value) {
    // Publish to message bus (internal only for now)
    g_message_bus.publishFloat(msg_id, value, false);
}

static void handle_sensor_error(uint8_t sensor_index) {
    sensor_runtime_t* runtime = &sensor_runtime[sensor_index];
    
    runtime->error_count++;
    total_errors++;
    
    if (runtime->error_count >= MAX_CONSECUTIVE_ERRORS) {
        runtime->is_valid = 0;  // Mark sensor as failed
        
        #ifdef ARDUINO
        Serial.print("InputManager: Sensor '");
        Serial.print(sensors[sensor_index].name);
        Serial.println("' marked as failed");
        #endif
    }
}