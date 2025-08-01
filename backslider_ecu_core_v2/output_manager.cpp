// output_manager.cpp
// Implementation of ECU output management system

#include "output_manager.h"
#include "msg_bus.h"
#include "pin_assignments.h"
#include <Arduino.h>

// =============================================================================
// PRIVATE DATA
// =============================================================================

// Output definitions and state
static output_definition_t registered_outputs[OUTPUT_MANAGER_MAX_OUTPUTS];
static uint8_t output_count = 0;
static uint8_t outputs_enabled = 1;

// Statistics
static output_manager_stats_t stats;

// Fault tracking
static output_fault_record_t fault_records[OUTPUT_MANAGER_MAX_FAULTS];
static uint8_t fault_count = 0;

// =============================================================================
// PRIVATE FUNCTION DECLARATIONS
// =============================================================================

static void handle_pwm_output_message(const CANMessage* msg);
static void handle_digital_output_message(const CANMessage* msg);
static void handle_analog_output_message(const CANMessage* msg);
static void configure_output_pin(output_definition_t* output);
static void update_pwm_output(output_definition_t* output, float value);
static void update_digital_output(output_definition_t* output, float value);
static void update_analog_output(output_definition_t* output, float value);
static void update_spi_output(output_definition_t* output, float value);
static void update_virtual_output(output_definition_t* output, float value);
static float clamp_value(float value, float min_val, float max_val);
static uint8_t check_rate_limit(output_definition_t* output);
static void record_fault(uint8_t output_index, output_fault_t fault_type, float value);
static uint16_t find_output_by_msg_id(uint32_t msg_id);

// =============================================================================
// PUBLIC FUNCTIONS
// =============================================================================

uint8_t output_manager_init(void) {
    #ifdef ARDUINO
    Serial.println("Initializing output manager...");
    #endif
    
    // Initialize state
    output_count = 0;
    outputs_enabled = 1;
    fault_count = 0;
    
    // Clear statistics
    stats.total_outputs = 0;
    stats.pwm_outputs = 0;
    stats.digital_outputs = 0;
    stats.analog_outputs = 0;
    stats.spi_outputs = 0;
    stats.virtual_outputs = 0;
    stats.total_updates = 0;
    stats.rate_limited_updates = 0;
    stats.range_violations = 0;
    stats.fault_count = 0;
    stats.last_update_time_ms = 0;
    
    #ifdef ARDUINO
    Serial.println("Output manager initialized");
    #endif
    
    return 1;
}

uint8_t output_manager_register_outputs(const output_definition_t* outputs, uint8_t count) {
    if (outputs == NULL || count == 0) {
        return 0;
    }
    
    if (output_count + count > OUTPUT_MANAGER_MAX_OUTPUTS) {
        #ifdef ARDUINO
        Serial.println("ERROR: Too many outputs to register");
        #endif
        return 0;
    }
    
    // Copy output definitions and configure pins
    for (uint8_t i = 0; i < count; i++) {
        registered_outputs[output_count] = outputs[i];
        registered_outputs[output_count].current_value = registered_outputs[output_count].config.pwm.default_duty_cycle;
        registered_outputs[output_count].last_update_time_ms = 0;
        registered_outputs[output_count].fault_detected = 0;
        
            // Configure the hardware pin
    configure_output_pin(&registered_outputs[output_count]);
    

        
        // Subscribe to the output's control message
        g_message_bus.subscribe(registered_outputs[output_count].msg_id, handle_pwm_output_message);
        
        // Update statistics
        switch (registered_outputs[output_count].type) {
            case OUTPUT_PWM:
                stats.pwm_outputs++;
                break;
            case OUTPUT_DIGITAL:
                stats.digital_outputs++;
                break;
            case OUTPUT_ANALOG:
                stats.analog_outputs++;
                break;
            case OUTPUT_SPI:
                stats.spi_outputs++;
                break;
            case OUTPUT_VIRTUAL:
                stats.virtual_outputs++;
                break;
            case OUTPUT_TYPE_COUNT:
                // Should not happen
                break;
        }
        
        output_count++;
    }
    
    stats.total_outputs = output_count;
    
    #ifdef ARDUINO
    Serial.print("Registered ");
    Serial.print(count);
    Serial.print(" outputs. Total: ");
    Serial.println(output_count);
    #endif
    
    return count;
}

void output_manager_update(void) {
    if (!outputs_enabled) {
        return;
    }
    
    stats.last_update_time_ms = millis();
    
    // Periodic refresh of all outputs to maintain state
    // This ensures outputs stay in the correct state even if messages are lost
    static uint32_t last_refresh_time = 0;
    uint32_t now = millis();
    
    // Refresh outputs every 100ms (10Hz) to maintain state
    if (now - last_refresh_time >= 100) {
        // Debug output removed to reduce serial clutter
        for (uint8_t i = 0; i < output_count; i++) {
            output_definition_t* output = &registered_outputs[i];
            
            // Skip rate limiting for periodic refresh to ensure state is maintained
            switch (output->type) {
                case OUTPUT_PWM: {
                    // Re-apply current PWM value to maintain state
                    #ifdef ARDUINO
                    uint32_t max_value = (1 << output->config.pwm.resolution_bits) - 1;
                    uint32_t pwm_value = (uint32_t)(output->current_value * max_value);
                    analogWrite(output->pin, pwm_value);
                    #endif
                    break;
                }
                case OUTPUT_DIGITAL: {
                    // Re-apply current digital state
                    #ifdef ARDUINO
                    bool digital_state = (output->current_value > 0.5f);
                    if (!output->config.digital.active_high) {
                        digital_state = !digital_state;
                    }
                    digitalWrite(output->pin, digital_state);
                    #endif
                    break;
                }
                case OUTPUT_ANALOG:
                case OUTPUT_SPI:
                case OUTPUT_VIRTUAL:
                    // These don't need periodic refresh
                    break;
                case OUTPUT_TYPE_COUNT:
                    break;
            }
        }
        
        last_refresh_time = now;
        stats.total_updates += output_count; // Count refresh updates
    }
    
    // Process any pending message bus messages
    // (Message handling is done automatically via subscribers)
}

void output_manager_safe_state(void) {
    // Debug output removed to reduce serial clutter
    
    for (uint8_t i = 0; i < output_count; i++) {
        output_definition_t* output = &registered_outputs[i];
        
        switch (output->type) {
            case OUTPUT_PWM:
                update_pwm_output(output, output->config.pwm.default_duty_cycle);
                break;
            case OUTPUT_DIGITAL:
                update_digital_output(output, (float)output->config.digital.default_state);
                break;
            case OUTPUT_ANALOG:
                update_analog_output(output, output->config.analog.default_voltage);
                break;
            case OUTPUT_SPI:
                update_spi_output(output, (float)output->config.spi.default_state);
                break;
            case OUTPUT_VIRTUAL:
                update_virtual_output(output, output->config.virtual_out.default_value);
                break;
            case OUTPUT_TYPE_COUNT:
                // Should not happen
                break;
        }
    }
}

void output_manager_enable(uint8_t enable) {
    outputs_enabled = enable;
    
    if (!enable) {
        output_manager_safe_state();
    }
    
    // Debug output removed to reduce serial clutter
}

float output_manager_get_value(uint8_t output_index) {
    if (output_index >= output_count) {
        return 0.0f;
    }
    
    return registered_outputs[output_index].current_value;
}

void output_manager_set_value(uint8_t output_index, float value) {
    if (output_index >= output_count) {
        return;
    }
    
    output_definition_t* output = &registered_outputs[output_index];
    
    switch (output->type) {
        case OUTPUT_PWM:
            update_pwm_output(output, value);
            break;
        case OUTPUT_DIGITAL:
            update_digital_output(output, value);
            break;
        case OUTPUT_ANALOG:
            update_analog_output(output, value);
            break;
        case OUTPUT_SPI:
            update_spi_output(output, value);
            break;
        case OUTPUT_VIRTUAL:
            update_virtual_output(output, value);
            break;
        case OUTPUT_TYPE_COUNT:
            // Should not happen
            break;
    }
}

const output_manager_stats_t* output_manager_get_stats(void) {
    return &stats;
}

void output_manager_reset_stats(void) {
    stats.total_updates = 0;
    stats.rate_limited_updates = 0;
    stats.range_violations = 0;
    stats.fault_count = 0;
}

uint8_t output_manager_get_fault_count(void) {
    return fault_count;
}

const output_fault_record_t* output_manager_get_fault(uint8_t fault_index) {
    if (fault_index >= fault_count) {
        return NULL;
    }
    
    return &fault_records[fault_index];
}

void output_manager_clear_faults(void) {
    fault_count = 0;
    stats.fault_count = 0;
    
    // Clear fault flags on outputs
    for (uint8_t i = 0; i < output_count; i++) {
        registered_outputs[i].fault_detected = 0;
    }
}



// =============================================================================
// PRIVATE FUNCTIONS
// =============================================================================

static void handle_pwm_output_message(const CANMessage* msg) {
    if (!outputs_enabled) {
        return;
    }
    
    uint16_t output_index = find_output_by_msg_id(msg->id);
    if (output_index >= output_count) {
        return;
    }
    
    float value = MSG_UNPACK_FLOAT(msg);
    
    #ifdef ARDUINO
    // Debug for pressure solenoid messages
    if (msg->id == 0x10500113) {  // MSG_TRANS_PRESSURE_SOL
        Serial.print("OUTPUT MANAGER RECEIVED PRESSURE: ");
        Serial.print(value * 100.0f);
        Serial.println("%");
    }
    #endif
    
    output_definition_t* output = &registered_outputs[output_index];
    
    // Route to appropriate handler based on output type
    switch (output->type) {
        case OUTPUT_PWM:
            update_pwm_output(output, value);
            break;
        case OUTPUT_DIGITAL:
            update_digital_output(output, value);
            break;
        case OUTPUT_ANALOG:
            update_analog_output(output, value);
            break;
        case OUTPUT_SPI:
            update_spi_output(output, value);
            break;
        case OUTPUT_VIRTUAL:
            update_virtual_output(output, value);
            break;
        case OUTPUT_TYPE_COUNT:
            // Should not happen
            break;
    }
}

static void handle_digital_output_message(const CANMessage* msg) {
    handle_pwm_output_message(msg); // Same logic
}

static void handle_analog_output_message(const CANMessage* msg) {
    handle_pwm_output_message(msg); // Same logic
}

static void configure_output_pin(output_definition_t* output) {
    #ifdef ARDUINO
    switch (output->type) {
        case OUTPUT_PWM:
            pinMode(output->pin, OUTPUT);
            analogWriteFrequency(output->pin, output->config.pwm.frequency_hz);
            analogWriteResolution(output->config.pwm.resolution_bits);
            
            #ifdef ARDUINO
            // Debug pin configuration
            if (output->pin == 22) {
                Serial.print("CONFIGURED PIN 22: OUTPUT, freq=");
                Serial.print(output->config.pwm.frequency_hz);
                Serial.print("Hz, resolution=");
                Serial.print(output->config.pwm.resolution_bits);
                Serial.println(" bits");
            }
            #endif
            break;
        case OUTPUT_DIGITAL:
            pinMode(output->pin, OUTPUT);
            break;
        case OUTPUT_ANALOG:
            // For Teensy 4.1, this would typically be PWM with external filter
            pinMode(output->pin, OUTPUT);
            if (output->config.analog.use_pwm_filter) {
                analogWriteFrequency(output->pin, 20000); // High frequency for filtering
                analogWriteResolution(output->config.analog.resolution_bits);
            }
            break;
        case OUTPUT_SPI:
            // SPI outputs don't need individual pin configuration
            // SPI bus will be configured when first SPI output is registered
            break;
        case OUTPUT_VIRTUAL:
            // Virtual outputs have no physical pin
            break;
        case OUTPUT_TYPE_COUNT:
            // Should not happen
            break;
    }
    #else
    // Mock pin configuration for testing
    switch (output->type) {
        case OUTPUT_PWM:
        case OUTPUT_DIGITAL:
        case OUTPUT_ANALOG:
            pinMode(output->pin, OUTPUT);  // Call mock pinMode
            break;
        case OUTPUT_SPI:
        case OUTPUT_VIRTUAL:
            // No pin configuration needed
            break;
        case OUTPUT_TYPE_COUNT:
            break;
    }
    #endif
}

static void update_pwm_output(output_definition_t* output, float value) {
    // Clamp value to safe range
    float clamped_value = clamp_value(value, output->config.pwm.min_duty_cycle, output->config.pwm.max_duty_cycle);
    if (clamped_value != value) {
        stats.range_violations++;
        record_fault(output - registered_outputs, OUTPUT_FAULT_RANGE_VIOLATION, value);
    }
    
    // Only update if value has actually changed (with small tolerance for floating point)
    float value_change = fabs(clamped_value - output->current_value);
    
    #ifdef ARDUINO
    // Debug value change detection for pin 22
    if (output->pin == 22) {
        Serial.print("VALUE CHANGE CHECK: current=");
        Serial.print(output->current_value * 100.0f);
        Serial.print("% new=");
        Serial.print(clamped_value * 100.0f);
        Serial.print("% change=");
        Serial.print(value_change * 100.0f);
        Serial.print("% threshold=0.1% -> ");
        Serial.println(value_change < 0.001f ? "SKIP" : "UPDATE");
    }
    #endif
    
    if (value_change < 0.001f) {  // 0.1% tolerance
        return;  // Value hasn't changed, skip update
    }
    
    #ifdef ARDUINO
    // Debug for pressure solenoid - only show when value actually changes
    if (output->pin == 22) {
        Serial.print("PIN 22 VALUE CHANGED: ");
        Serial.print(output->current_value * 100.0f);
        Serial.print("% -> ");
        Serial.print(clamped_value * 100.0f);
        Serial.println("%");
    }
    #endif
    
    // Check rate limiting
    if (!check_rate_limit(output)) {
        stats.rate_limited_updates++;
        return;
    }
    
    output->current_value = clamped_value;
    output->last_update_time_ms = millis();
    stats.total_updates++;
    
    #ifdef ARDUINO
    // Debug for pressure solenoid
    if (output->pin == 22) {
        Serial.print("PIN 22: About to calculate PWM, clamped_value=");
        Serial.print(clamped_value * 100.0f);
        Serial.println("%");
    }
    
    // Convert duty cycle to PWM value
    uint32_t max_value = (1 << output->config.pwm.resolution_bits) - 1;
    uint32_t pwm_value = (uint32_t)(clamped_value * max_value);
    
    if (output->config.pwm.invert_output) {
        pwm_value = max_value - pwm_value;
    }
    
    // Simple debug for pin 22 (pressure solenoid)
    if (output->pin == 22) {
        Serial.print("PIN 22 PWM: ");
        Serial.print(pwm_value);
        Serial.print("/");
        Serial.print(max_value);
        Serial.print(" (");
        Serial.print((float)pwm_value / max_value * 100.0f);
        Serial.println("%)");
        Serial.print("CALLING analogWrite(");
        Serial.print(output->pin);
        Serial.print(", ");
        Serial.print(pwm_value);
        Serial.println(")");
    }
    
    analogWrite(output->pin, pwm_value);
    
    #ifdef ARDUINO
    // Debug after analogWrite
    if (output->pin == 22) {
        Serial.println("analogWrite() CALLED for pin 22");
    }
    #endif
    #endif
}

static void update_digital_output(output_definition_t* output, float value) {
    uint8_t digital_state = (value > 0.5f) ? 1 : 0;
    
    if (!output->config.digital.active_high) {
        digital_state = !digital_state;
    }
    
    // Only update if value has actually changed
    float new_value = (float)digital_state;
    if (fabs(new_value - output->current_value) < 0.1f) {  // Digital values should be exactly 0 or 1
        return;  // Value hasn't changed, skip update
    }
    
    // Check rate limiting
    if (!check_rate_limit(output)) {
        stats.rate_limited_updates++;
        return;
    }
    
    output->current_value = new_value;
    output->last_update_time_ms = millis();
    stats.total_updates++;
    
    #ifdef ARDUINO
    // Simple debug for pin 22 (pressure solenoid)
    if (output->pin == 22) {
        Serial.print("PIN 22 SET TO: ");
        Serial.println(digital_state ? "HIGH" : "LOW");
    }
    
    digitalWrite(output->pin, digital_state);
    #else
    std::cout << "DEBUG: digitalWrite(" << (int)output->pin << ", " << (int)digital_state << ")" << std::endl;
    digitalWrite(output->pin, digital_state);  // Call mock digitalWrite
    #endif
}

static void update_analog_output(output_definition_t* output, float value) {
    // Check rate limiting
    if (!check_rate_limit(output)) {
        stats.rate_limited_updates++;
        return;
    }
    
    // Clamp voltage to safe range
    float clamped_voltage = clamp_value(value, output->config.analog.min_voltage, output->config.analog.max_voltage);
    if (clamped_voltage != value) {
        stats.range_violations++;
        record_fault(output - registered_outputs, OUTPUT_FAULT_RANGE_VIOLATION, value);
    }
    
    output->current_value = clamped_voltage;
    output->last_update_time_ms = millis();
    stats.total_updates++;
    
    #ifdef ARDUINO
    if (output->config.analog.use_pwm_filter) {
        // Use PWM with external filter to approximate analog output
        uint32_t max_value = (1 << output->config.analog.resolution_bits) - 1;
        float voltage_ratio = clamped_voltage / output->config.analog.max_voltage;
        uint32_t pwm_value = (uint32_t)(voltage_ratio * max_value);
        analogWrite(output->pin, pwm_value);
    }
    // Note: True DAC output would require different handling on Teensy 4.1
    #endif
}

static float clamp_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static uint8_t check_rate_limit(output_definition_t* output) {
    uint32_t now = millis();
    uint8_t rate_ok = (now - output->last_update_time_ms) >= output->update_rate_limit_ms;
    
    #ifdef ARDUINO
    // Debug for pressure solenoid rate limiting
    if (output->pin == 22) {
        if (!rate_ok) {
            Serial.print("PIN 22 RATE LIMITED! Time since last update: ");
            Serial.print(now - output->last_update_time_ms);
            Serial.print("ms, limit: ");
            Serial.print(output->update_rate_limit_ms);
            Serial.println("ms");
        }
    }
    #endif
    
    return rate_ok;
}

static void record_fault(uint8_t output_index, output_fault_t fault_type, float value) {
    if (fault_count >= OUTPUT_MANAGER_MAX_FAULTS) {
        return; // Fault buffer full
    }
    
    fault_records[fault_count].fault_type = fault_type;
    fault_records[fault_count].output_index = output_index;
    fault_records[fault_count].fault_time_ms = millis();
    fault_records[fault_count].fault_value = value;
    fault_records[fault_count].description = "Range violation";
    
    fault_count++;
    stats.fault_count++;
    
    if (output_index < output_count) {
        registered_outputs[output_index].fault_detected = 1;
    }
}

static void update_spi_output(output_definition_t* output, float value) {
    // Check rate limiting
    if (!check_rate_limit(output)) {
        stats.rate_limited_updates++;
        return;
    }
    
    uint8_t digital_state = (value > 0.5f) ? 1 : 0;
    
    if (!output->config.spi.active_high) {
        digital_state = !digital_state;
    }
    
    output->current_value = (float)digital_state;
    output->last_update_time_ms = millis();
    stats.total_updates++;
    
    #ifdef ARDUINO
    // TODO: Implement SPI output control
    // This would involve:
    // 1. Reading current state of SPI device
    // 2. Setting/clearing the appropriate bit
    // 3. Writing new state back to SPI device
    // For now, just log the intended change
    Serial.print("SPI Output: Device ");
    Serial.print(output->config.spi.spi_device_id);
    Serial.print(", Bit ");
    Serial.print(output->config.spi.bit_position);
    Serial.print(" = ");
    Serial.println(digital_state);
    #endif
}

static void update_virtual_output(output_definition_t* output, float value) {
    // Check rate limiting
    if (!check_rate_limit(output)) {
        stats.rate_limited_updates++;
        return;
    }
    
    // Clamp value to configured range
    float clamped_value = clamp_value(value, output->config.virtual_out.min_value, output->config.virtual_out.max_value);
    if (clamped_value != value) {
        stats.range_violations++;
        record_fault(output - registered_outputs, OUTPUT_FAULT_RANGE_VIOLATION, value);
    }
    
    output->current_value = clamped_value;
    output->last_update_time_ms = millis();
    stats.total_updates++;
    
    #ifdef ARDUINO
    // Log to serial if configured
    if (output->config.virtual_out.log_to_serial) {
        Serial.print("Virtual Output: ");
        Serial.print(output->name);
        Serial.print(" = ");
        Serial.println(clamped_value);
    }
    
    // Send to CAN if configured
    if (output->config.virtual_out.send_to_can) {
        // Publish the virtual output value on the CAN bus
        g_message_bus.publishFloat(output->msg_id, clamped_value);
    }
    #endif
}

static uint16_t find_output_by_msg_id(uint32_t msg_id) {
    for (uint8_t i = 0; i < output_count; i++) {
        if (registered_outputs[i].msg_id == msg_id) {
            return i;
        }
    }
    
    #ifdef ARDUINO
    // Debug for pressure solenoid
    if (msg_id == 0x10500113) {  // MSG_TRANS_PRESSURE_SOL
        Serial.print("PRESSURE SOLENOID NOT FOUND! Output count: ");
        Serial.print(output_count);
        Serial.print(", Looking for msg_id: 0x");
        Serial.println(msg_id, HEX);
        
        // Show all registered outputs
        for (uint8_t i = 0; i < output_count; i++) {
            Serial.print("  Output ");
            Serial.print(i);
            Serial.print(": msg_id=0x");
            Serial.print(registered_outputs[i].msg_id, HEX);
            Serial.print(", pin=");
            Serial.print(registered_outputs[i].pin);
            Serial.print(", name=");
            Serial.println(registered_outputs[i].name);
        }
    }
    #endif
    
    return OUTPUT_MANAGER_MAX_OUTPUTS; // Not found
} 