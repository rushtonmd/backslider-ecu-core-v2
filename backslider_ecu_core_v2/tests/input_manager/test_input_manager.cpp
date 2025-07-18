// tests/input_manager/test_input_manager.cpp
// Test suite for the input manager system

#include <iostream>
#include <cassert>

// Include enhanced mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
// Serial mock object is defined in mock_arduino.cpp

// Include message bus and input manager for testing
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../input_manager.h"
#include "../../sensor_calibration.h"

// Simple test framework
int tests_run = 0;
int tests_passed = 0;

#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        std::cout << "  Running test: " #name "... "; \
        tests_run++; \
        test_##name(); \
        tests_passed++; \
        std::cout << "PASSED" << std::endl; \
    } \
    void test_##name()

// Test message reception for sensors
static float received_sensor_value = 0.0f;
static uint32_t received_msg_id = 0;
static bool sensor_message_received = false;

void test_sensor_message_handler(const CANMessage* msg) {
    received_sensor_value = MSG_UNPACK_FLOAT(msg);
    received_msg_id = msg->id;
    sensor_message_received = true;
}

// Test setup function to initialize mock environment
void test_setup() {
    mock_reset_all();
    
    // Set some realistic mock sensor values
    mock_set_analog_voltage(A0, 2.5f);  // TPS at 50% (mid-range)
    mock_set_analog_voltage(A1, 1.5f);  // MAP sensor
    mock_set_analog_voltage(A2, 2.0f);  // Temperature sensor
    
    // Reset test state
    sensor_message_received = false;
    received_sensor_value = 0.0f;
    received_msg_id = 0;
}

// Helper function to ensure mock analog readings are stable
void ensure_mock_stable(int pin) {
    // Force mock to settle by reading twice
    analogRead(pin);
    analogRead(pin);
}

// Test basic input manager initialization
TEST(input_manager_initialization) {
    test_setup();
    input_manager_init();
    
    // Check initial state
    assert(input_manager_get_sensor_count() == 0);
    assert(input_manager_get_valid_sensor_count() == 0);
    assert(input_manager_get_total_updates() == 0);
    assert(input_manager_get_total_errors() == 0);
}

// Test sensor registration
TEST(sensor_registration) {
    test_setup();
    input_manager_init();
    
    // Create a simple test sensor
    sensor_definition_t test_sensors[] = {
        DEFINE_LINEAR_SENSOR(
            A0,                        // pin
            MSG_THROTTLE_POSITION,     // message ID
            0.5f, 4.5f,               // voltage range
            0.0f, 100.0f,             // output range
            50000,                    // 50ms update interval
            "Test TPS"                // name
        )
    };
    
    // Register the sensor
    uint8_t registered = input_manager_register_sensors(test_sensors, 1);
    
    assert(registered == 1);
    assert(input_manager_get_sensor_count() == 1);
}

// Test multiple sensor registration
TEST(multiple_sensor_registration) {
    input_manager_init();
    
    // Create multiple test sensors
    sensor_definition_t test_sensors[] = {
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 50000, "TPS"),
        DEFINE_LINEAR_SENSOR(A1, MSG_MANIFOLD_PRESSURE, 0.5f, 4.5f, 20.0f, 300.0f, 25000, "MAP"),
        DEFINE_THERMISTOR_SENSOR(A2, MSG_COOLANT_TEMP, 2200, 
                               STANDARD_THERMISTOR_VOLTAGE_TABLE,
                               STANDARD_THERMISTOR_TEMP_TABLE,
                               STANDARD_THERMISTOR_TABLE_SIZE,
                               1000000, "CTS")
    };
    
    // Register all sensors
    uint8_t registered = input_manager_register_sensors(test_sensors, 3);
    
    assert(registered == 3);
    assert(input_manager_get_sensor_count() == 3);
}

// Test sensor update and message publishing
TEST(sensor_update_and_publishing) {
    // Reset mock environment and initialize systems
    test_setup();
    g_message_bus.init();
    input_manager_init();
    sensor_message_received = false;
    
    // Subscribe to sensor messages
    g_message_bus.subscribe(MSG_THROTTLE_POSITION, test_sensor_message_handler);
    
    // Create and register a test sensor with ZERO update interval (always update)
    sensor_definition_t test_sensor[] = {
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 0, "Test TPS")  // 0 = always update
    };
    
    uint8_t registered = input_manager_register_sensors(test_sensor, 1);
    assert(registered == 1);
    
    // Set a known analog voltage for testing and ensure it's stable
    mock_set_analog_voltage(A0, 2.5f);  // Should result in ~50% throttle
    ensure_mock_stable(A0);              // Force mock to settle
    
    // Verify mock is working correctly before proceeding
    uint16_t adc_reading = analogRead(A0);
    float voltage_reading = adc_counts_to_voltage(adc_reading);
    
    // Should be approximately 2.5V (±0.1V tolerance for mock precision)
    assert(voltage_reading >= 2.4f && voltage_reading <= 2.6f);
    
    // Force an update - with 0 interval, should always update
    input_manager_update();
    
    // Process message bus
    g_message_bus.process();
    
    // Verify sensor updated and published message
    assert(input_manager_get_total_updates() > 0);
    assert(sensor_message_received == true);
    assert(received_msg_id == MSG_THROTTLE_POSITION);
    
    // Verify we got a reasonable value (close to 50% with tolerance for mock precision)
    assert(received_sensor_value >= 45.0f && received_sensor_value <= 55.0f);
}

// Test linear calibration
TEST(linear_calibration) {
    linear_config_t config = {
        .min_voltage = 0.5f,
        .max_voltage = 4.5f,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .pullup_ohms = 0
    };
    
    // Test calibration at different points
    float result;
    
    // Test minimum
    result = calibrate_linear(&config, 0.5f);
    assert(result == 0.0f);
    
    // Test maximum
    result = calibrate_linear(&config, 4.5f);
    assert(result == 100.0f);
    
    // Test middle
    result = calibrate_linear(&config, 2.5f);
    assert(result == 50.0f);
    
    // Test out of range (should clamp)
    result = calibrate_linear(&config, 0.0f);
    assert(result == 0.0f);
    
    result = calibrate_linear(&config, 5.0f);
    assert(result == 100.0f);
}

// Test thermistor calibration
TEST(thermistor_calibration) {
    thermistor_config_t config = {
        .pullup_ohms = 2200,
        .voltage_table = STANDARD_THERMISTOR_VOLTAGE_TABLE,
        .temp_table = STANDARD_THERMISTOR_TEMP_TABLE,
        .table_size = STANDARD_THERMISTOR_TABLE_SIZE
    };
    
    // Test calibration at known points
    float result;
    
    // Test a point that should be in the table
    result = calibrate_thermistor(&config, 2.5f);
    assert(result == 20.0f);  // Should match table entry
    
    // Test interpolation between points
    result = calibrate_thermistor(&config, 1.75f);
    assert(result > 20.0f && result < 60.0f);  // Should be between table entries
}

// Test digital sensor calibration
TEST(digital_calibration) {
    digital_config_t config = {
        .use_pullup = 1,
        .invert_logic = 0
    };
    
    // Test digital values
    float result;
    
    result = calibrate_digital(&config, 0);
    assert(result == 0.0f);
    
    result = calibrate_digital(&config, 1);
    assert(result == 1.0f);
}

// Test sensor finding by message ID
TEST(sensor_find_by_message_id) {
    input_manager_init();
    
    // Register test sensors
    sensor_definition_t test_sensors[] = {
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 50000, "TPS"),
        DEFINE_LINEAR_SENSOR(A1, MSG_MANIFOLD_PRESSURE, 0.5f, 4.5f, 20.0f, 300.0f, 25000, "MAP")
    };
    
    input_manager_register_sensors(test_sensors, 2);
    
    // Test finding sensors
    int8_t tps_index = input_manager_find_sensor_by_msg_id(MSG_THROTTLE_POSITION);
    int8_t map_index = input_manager_find_sensor_by_msg_id(MSG_MANIFOLD_PRESSURE);
    int8_t nonexistent_index = input_manager_find_sensor_by_msg_id(0x999);
    
    assert(tps_index == 0);
    assert(map_index == 1);
    assert(nonexistent_index == -1);
}

// Test sensor status retrieval
TEST(sensor_status_retrieval) {
    input_manager_init();
    
    // Register a test sensor
    sensor_definition_t test_sensor[] = {
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 1000, "TPS")
    };
    
    input_manager_register_sensors(test_sensor, 1);
    
    // Get sensor status (even without updates)
    sensor_runtime_t status;
    uint8_t result = input_manager_get_sensor_status(0, &status);
    
    assert(result == 1);  // Should successfully get status
    // Don't check update_count since sensor hasn't been triggered to update yet
    
    // Test invalid index
    result = input_manager_get_sensor_status(99, &status);
    assert(result == 0);  // Should fail for invalid index
}

// Test utility functions
TEST(utility_functions) {
    // Test ADC to voltage conversion
    float voltage = adc_counts_to_voltage(2048);  // Half of 12-bit range
    assert(voltage > 1.6f && voltage < 1.7f);     // Should be ~1.65V (half of 3.3V)
    
    // Test voltage validation
    assert(is_voltage_valid(2.5f) == 1);          // Valid voltage
    assert(is_voltage_valid(0.05f) == 0);         // Too low
    assert(is_voltage_valid(4.95f) == 0);         // Too high
}

// Test table interpolation
TEST(table_interpolation) {
    const float x_table[] = {0.0f, 1.0f, 2.0f, 3.0f};
    const float y_table[] = {0.0f, 10.0f, 20.0f, 30.0f};
    
    // Test exact points
    float result = interpolate_table(x_table, y_table, 4, 1.0f);
    assert(result == 10.0f);
    
    // Test interpolation
    result = interpolate_table(x_table, y_table, 4, 1.5f);
    assert(result == 15.0f);
    
    // Test edge cases
    result = interpolate_table(x_table, y_table, 4, -1.0f);
    assert(result == 0.0f);  // Should clamp to first value
    
    result = interpolate_table(x_table, y_table, 4, 5.0f);
    assert(result == 30.0f);  // Should clamp to last value
}

// Main test runner
int main() {
    std::cout << "=== Input Manager Tests ===" << std::endl;
    
    // Run all tests
    run_test_input_manager_initialization();
    run_test_sensor_registration();
    run_test_multiple_sensor_registration();
    run_test_sensor_update_and_publishing();
    run_test_linear_calibration();
    run_test_thermistor_calibration();
    run_test_digital_calibration();
    run_test_sensor_find_by_message_id();
    run_test_sensor_status_retrieval();
    run_test_utility_functions();
    run_test_table_interpolation();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Input Manager Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL INPUT MANAGER TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME INPUT MANAGER TESTS FAILED!" << std::endl;
        return 1;
    }
}