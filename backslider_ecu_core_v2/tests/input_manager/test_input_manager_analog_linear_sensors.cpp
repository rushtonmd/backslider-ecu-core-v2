// tests/input_manager/test_input_manager_analog_linear_sensors.cpp
// Comprehensive test suite for analog linear sensor functionality in the input manager
//
// This test suite focuses specifically on SENSOR_ANALOG_LINEAR sensor logic, 
// configuration, calibration, filtering, and message publishing to ensure
// the analog linear sensor implementation is working correctly.
//
// Tests sensors like TPS, MAP, oil pressure, fuel pressure, boost pressure, etc.

#include <iostream>
#include <cassert>
#include <cmath>

// Include enhanced mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

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

// Test message reception for analog sensors
static float received_analog_value = 0.0f;
static uint32_t received_msg_id = 0;
static bool analog_message_received = false;

void test_analog_message_handler(const CANMessage* msg) {
    received_analog_value = MSG_UNPACK_FLOAT(msg);
    received_msg_id = msg->id;
    analog_message_received = true;
}

// Test setup function to initialize mock environment
void test_setup() {
    mock_reset_all();
    
    // Set realistic analog sensor voltages for testing
    mock_set_analog_voltage(A0, 2.5f);  // TPS at 50% (mid-range)
    mock_set_analog_voltage(A1, 1.5f);  // MAP at low pressure
    mock_set_analog_voltage(A2, 0.8f);  // Oil pressure at minimum
    mock_set_analog_voltage(A3, 4.2f);  // Fuel pressure at high
    mock_set_analog_voltage(A4, 3.0f);  // Boost pressure at moderate
    
    // Reset test state
    analog_message_received = false;
    received_analog_value = 0.0f;
    received_msg_id = 0;
}

// Helper function to ensure mock analog readings are stable
void ensure_analog_mock_stable(int pin) {
    // Force mock to settle by reading twice
    analogRead(pin);
    analogRead(pin);
}

// Helper function to check if two floats are approximately equal
bool float_equals(float a, float b, float tolerance = 0.1f) {
    return fabs(a - b) < tolerance;
}

// =============================================================================
// ANALOG LINEAR SENSOR CONFIGURATION TESTS
// =============================================================================

// Test basic analog linear sensor registration
TEST(analog_linear_sensor_registration) {
    test_setup();
    input_manager_init();
    
    // Create a basic TPS sensor (0-100% throttle position)
    sensor_definition_t tps_sensor[] = {
        DEFINE_LINEAR_SENSOR(
            A0,                        // pin
            MSG_THROTTLE_POSITION,     // message ID
            0.5f, 4.5f,               // voltage range (0.5V-4.5V)
            0.0f, 100.0f,             // output range (0-100%)
            50000,                    // 50ms update interval
            "TPS Sensor"              // name
        )
    };
    
    // Register the sensor
    uint8_t registered = input_manager_register_sensors(tps_sensor, 1);
    
    assert(registered == 1);
    assert(input_manager_get_sensor_count() == 1);
    
    // Verify sensor configuration
    sensor_runtime_t status;
    uint8_t result = input_manager_get_sensor_status(0, &status);
    assert(result == 1);
}

// Test multiple analog linear sensors with different configurations
TEST(multiple_analog_sensors) {
    test_setup();
    input_manager_init();
    
    // Create multiple automotive sensors with different ranges
    sensor_definition_t automotive_sensors[] = {
        // TPS: 0.5-4.5V -> 0-100%
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 50000, "TPS"),
        
        // MAP: 0.5-4.5V -> 20-300 kPa (typical automotive range)
        DEFINE_LINEAR_SENSOR(A1, MSG_MANIFOLD_PRESSURE, 0.5f, 4.5f, 20.0f, 300.0f, 25000, "MAP"),
        
        // Oil Pressure: 0.5-4.5V -> 0-100 PSI
        DEFINE_LINEAR_SENSOR(A2, MSG_OIL_PRESSURE, 0.5f, 4.5f, 0.0f, 100.0f, 100000, "Oil Pressure"),
        
        // Battery Voltage: 0.5-4.5V -> 8-16 V (scaled automotive voltage)
        DEFINE_LINEAR_SENSOR(A3, MSG_BATTERY_VOLTAGE, 0.5f, 4.5f, 8.0f, 16.0f, 100000, "Battery Voltage"),
        
        // Boost Target: 0.5-4.5V -> 0-300 kPa (boost pressure target)
        DEFINE_LINEAR_SENSOR(A4, MSG_BOOST_TARGET, 0.5f, 4.5f, 0.0f, 300.0f, 50000, "Boost Target")
    };
    
    // Register all sensors
    uint8_t registered = input_manager_register_sensors(automotive_sensors, 5);
    
    assert(registered == 5);
    assert(input_manager_get_sensor_count() == 5);
    
    // Verify each sensor can be found by message ID
    assert(input_manager_find_sensor_by_msg_id(MSG_THROTTLE_POSITION) == 0);
    assert(input_manager_find_sensor_by_msg_id(MSG_MANIFOLD_PRESSURE) == 1);
    assert(input_manager_find_sensor_by_msg_id(MSG_OIL_PRESSURE) == 2);
    assert(input_manager_find_sensor_by_msg_id(MSG_BATTERY_VOLTAGE) == 3);
    assert(input_manager_find_sensor_by_msg_id(MSG_BOOST_TARGET) == 4);
}

// Test sensor configuration with different voltage ranges
TEST(different_voltage_ranges) {
    test_setup();
    input_manager_init();
    
    // Create sensors with different voltage ranges (some sensors use 0-5V, others 0.5-4.5V)
    sensor_definition_t voltage_range_sensors[] = {
        // Standard 0.5-4.5V sensor (most automotive sensors)
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 50000, "Standard TPS"),
        
        // Full 0-5V range sensor (some aftermarket sensors)
        DEFINE_LINEAR_SENSOR(A1, MSG_MANIFOLD_PRESSURE, 0.0f, 5.0f, 0.0f, 500.0f, 25000, "Full Range MAP"),
        
        // Narrow range sensor (1-4V for precision)
        DEFINE_LINEAR_SENSOR(A2, MSG_OIL_PRESSURE, 1.0f, 4.0f, 0.0f, 150.0f, 100000, "Precision Oil"),
        
        // Wide range sensor (0.2-4.8V for maximum resolution)
        DEFINE_LINEAR_SENSOR(A3, MSG_BATTERY_VOLTAGE, 0.2f, 4.8f, 8.0f, 18.0f, 100000, "Wide Range Battery")
    };
    
    // Register all sensors
    uint8_t registered = input_manager_register_sensors(voltage_range_sensors, 4);
    
    assert(registered == 4);
    assert(input_manager_get_sensor_count() == 4);
}

// =============================================================================
// ANALOG LINEAR CALIBRATION TESTS
// =============================================================================

// Test linear calibration function directly
TEST(linear_calibration_function) {
    // Test standard TPS calibration (0.5-4.5V -> 0-100%)
    linear_config_t tps_config = {
        .min_voltage = 0.5f,
        .max_voltage = 4.5f,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .pullup_ohms = 0
    };
    
    float result;
    
    // Test minimum voltage
    result = calibrate_linear(&tps_config, 0.5f);
    assert(float_equals(result, 0.0f));
    
    // Test maximum voltage
    result = calibrate_linear(&tps_config, 4.5f);
    assert(float_equals(result, 100.0f));
    
    // Test mid-range voltage (should be 50%)
    result = calibrate_linear(&tps_config, 2.5f);
    assert(float_equals(result, 50.0f));
    
    // Test 25% point
    result = calibrate_linear(&tps_config, 1.5f);
    assert(float_equals(result, 25.0f));
    
    // Test 75% point
    result = calibrate_linear(&tps_config, 3.5f);
    assert(float_equals(result, 75.0f));
}

// Test linear calibration with different ranges
TEST(linear_calibration_different_ranges) {
    // Test MAP sensor calibration (0.5-4.5V -> 20-300 kPa)
    linear_config_t map_config = {
        .min_voltage = 0.5f,
        .max_voltage = 4.5f,
        .min_value = 20.0f,
        .max_value = 300.0f,
        .pullup_ohms = 0
    };
    
    float result;
    
    // Test minimum (should be 20 kPa)
    result = calibrate_linear(&map_config, 0.5f);
    assert(float_equals(result, 20.0f));
    
    // Test maximum (should be 300 kPa)
    result = calibrate_linear(&map_config, 4.5f);
    assert(float_equals(result, 300.0f));
    
    // Test mid-range (should be 160 kPa)
    result = calibrate_linear(&map_config, 2.5f);
    assert(float_equals(result, 160.0f));
    
    // Test boost target calibration (0.5-4.5V -> 0 to 300 kPa)
    linear_config_t boost_config = {
        .min_voltage = 0.5f,
        .max_voltage = 4.5f,
        .min_value = 0.0f,
        .max_value = 300.0f,
        .pullup_ohms = 0
    };
    
    // Test minimum (should be 0 kPa)
    result = calibrate_linear(&boost_config, 0.5f);
    assert(float_equals(result, 0.0f));
    
    // Test maximum (should be 300 kPa)
    result = calibrate_linear(&boost_config, 4.5f);
    assert(float_equals(result, 300.0f));
    
    // Test mid-range (should be 150 kPa)
    result = calibrate_linear(&boost_config, 2.5f);
    assert(float_equals(result, 150.0f));
}

// Test linear calibration edge cases and clamping
TEST(linear_calibration_edge_cases) {
    linear_config_t config = {
        .min_voltage = 0.5f,
        .max_voltage = 4.5f,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .pullup_ohms = 0
    };
    
    float result;
    
    // Test out-of-range low voltage (should clamp to minimum)
    result = calibrate_linear(&config, 0.0f);
    assert(float_equals(result, 0.0f));
    
    // Test out-of-range high voltage (should clamp to maximum)
    result = calibrate_linear(&config, 5.0f);
    assert(float_equals(result, 100.0f));
    
    // Test very low voltage (should clamp to minimum)
    result = calibrate_linear(&config, 0.1f);
    assert(float_equals(result, 0.0f));
    
    // Test very high voltage (should clamp to maximum)
    result = calibrate_linear(&config, 6.0f);
    assert(float_equals(result, 100.0f));
    
    // Test null pointer safety
    result = calibrate_linear(nullptr, 2.5f);
    assert(float_equals(result, 0.0f));
}

// =============================================================================
// ANALOG LINEAR SENSOR READING AND PUBLISHING TESTS
// =============================================================================

// Test analog sensor reading and message publishing
TEST(analog_sensor_reading_and_publishing) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to TPS messages
    g_message_bus.subscribe(MSG_THROTTLE_POSITION, test_analog_message_handler);
    
    // Create TPS sensor with zero update interval and no filtering
    sensor_definition_t tps_sensor[] = {
        {
            .pin = A0,
            .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {
                .min_voltage = 0.5f,
                .max_voltage = 4.5f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .pullup_ohms = 0
            },
            .msg_id = MSG_THROTTLE_POSITION,
            .update_interval_us = 0,
            .filter_strength = 0,  // No filtering for predictable test results
            .name = "TPS Test"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(tps_sensor, 1);
    assert(registered == 1);
    
    // Test at 50% throttle position (2.5V)
    mock_set_analog_voltage(A0, 2.5f);
    ensure_analog_mock_stable(A0);
    
    analog_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(analog_message_received == true);
    assert(received_msg_id == MSG_THROTTLE_POSITION);
    assert(float_equals(received_analog_value, 50.0f, 5.0f));  // 50% ± 5%
    
    // Test at 25% throttle position (1.5V)
    mock_set_analog_voltage(A0, 1.5f);
    ensure_analog_mock_stable(A0);
    
    analog_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(analog_message_received == true);
    assert(float_equals(received_analog_value, 25.0f, 5.0f));  // 25% ± 5%
    
    // Test at 75% throttle position (3.5V)
    mock_set_analog_voltage(A0, 3.5f);
    ensure_analog_mock_stable(A0);
    
    analog_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(analog_message_received == true);
    assert(float_equals(received_analog_value, 75.0f, 5.0f));  // 75% ± 5%
}

// Test multiple analog sensors publishing simultaneously
TEST(multiple_analog_sensors_publishing) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to multiple sensor messages
    g_message_bus.subscribe(MSG_THROTTLE_POSITION, test_analog_message_handler);
    g_message_bus.subscribe(MSG_MANIFOLD_PRESSURE, test_analog_message_handler);
    g_message_bus.subscribe(MSG_OIL_PRESSURE, test_analog_message_handler);
    
    // Create multiple sensors with zero update interval and no filtering
    sensor_definition_t multi_sensors[] = {
        {
            .pin = A0, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION, .update_interval_us = 0, .filter_strength = 0, .name = "TPS"
        },
        {
            .pin = A1, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 20.0f, 300.0f, 0},
            .msg_id = MSG_MANIFOLD_PRESSURE, .update_interval_us = 0, .filter_strength = 0, .name = "MAP"
        },
        {
            .pin = A2, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_OIL_PRESSURE, .update_interval_us = 0, .filter_strength = 0, .name = "Oil"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(multi_sensors, 3);
    assert(registered == 3);
    
    // Set different voltages for each sensor
    mock_set_analog_voltage(A0, 2.5f);  // TPS at 50% (should be ~50%)
    mock_set_analog_voltage(A1, 1.5f);  // MAP at 25% (should be ~90 kPa)
    mock_set_analog_voltage(A2, 4.0f);  // Oil at 87.5% (should be ~87.5 PSI)
    
    // Ensure all mocks are stable
    ensure_analog_mock_stable(A0);
    ensure_analog_mock_stable(A1);
    ensure_analog_mock_stable(A2);
    
    // Update all sensors
    input_manager_update();
    g_message_bus.process();
    
    // Check that all sensors published reasonable values
    assert(input_manager_get_total_updates() >= 3);
    
    // Check individual sensor values
    sensor_runtime_t status;
    
    // TPS should be around 50%
    input_manager_get_sensor_status(0, &status);
    assert(float_equals(status.calibrated_value, 50.0f, 5.0f));
    
    // MAP should be around 90 kPa (25% of 20-300 range)
    input_manager_get_sensor_status(1, &status);
    assert(float_equals(status.calibrated_value, 90.0f, 5.0f));
    
    // Oil pressure should be around 87.5 PSI
    input_manager_get_sensor_status(2, &status);
    assert(float_equals(status.calibrated_value, 87.5f, 5.0f));
}

// Test analog sensor response to changing voltages
TEST(analog_sensor_voltage_changes) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to TPS messages
    g_message_bus.subscribe(MSG_THROTTLE_POSITION, test_analog_message_handler);
    
    // Create TPS sensor with no filtering
    sensor_definition_t tps_sensor[] = {
        {
            .pin = A0, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION, .update_interval_us = 0, .filter_strength = 0, .name = "TPS"
        }
    };
    
    input_manager_register_sensors(tps_sensor, 1);
    
    // Test a sequence of different throttle positions
    float test_voltages[] = {0.5f, 1.5f, 2.5f, 3.5f, 4.5f};
    float expected_values[] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f};
    
    for (int i = 0; i < 5; i++) {
        mock_set_analog_voltage(A0, test_voltages[i]);
        ensure_analog_mock_stable(A0);
        
        analog_message_received = false;
        input_manager_update();
        g_message_bus.process();
        
        assert(analog_message_received == true);
        assert(float_equals(received_analog_value, expected_values[i], 5.0f));
    }
}

// =============================================================================
// ANALOG SENSOR FILTERING TESTS
// =============================================================================

// Test analog sensor filtering (low-pass filter)
TEST(analog_sensor_filtering) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Create TPS sensor with moderate filtering (filter_strength = 100)
    sensor_definition_t filtered_sensor[] = {
        {
            .pin = A0,
            .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {
                .min_voltage = 0.5f,
                .max_voltage = 4.5f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .pullup_ohms = 0
            },
            .msg_id = MSG_THROTTLE_POSITION,
            .update_interval_us = 0,
            .filter_strength = 100,  // Moderate filtering
            .name = "Filtered TPS"
        }
    };
    
    input_manager_register_sensors(filtered_sensor, 1);
    
    // Start with a steady voltage
    mock_set_analog_voltage(A0, 2.5f);  // 50% throttle
    ensure_analog_mock_stable(A0);
    
    // First reading should be unfiltered
    input_manager_update();
    sensor_runtime_t status;
    input_manager_get_sensor_status(0, &status);
    float first_reading = status.calibrated_value;
    assert(float_equals(first_reading, 50.0f, 5.0f));
    
    // Change voltage suddenly
    mock_set_analog_voltage(A0, 4.0f);  // 87.5% throttle
    ensure_analog_mock_stable(A0);
    
    // Second reading should be filtered (not jump immediately to 87.5%)
    input_manager_update();
    input_manager_get_sensor_status(0, &status);
    float second_reading = status.calibrated_value;
    
    // Should be somewhere between first reading and target
    assert(second_reading > first_reading);
    assert(second_reading < 87.5f);
    
    // Multiple updates should converge toward target
    for (int i = 0; i < 10; i++) {
        input_manager_update();
    }
    
    input_manager_get_sensor_status(0, &status);
    float final_reading = status.calibrated_value;
    
    // Should be closer to target after multiple updates
    assert(final_reading > second_reading);
    assert(float_equals(final_reading, 87.5f, 5.0f));
}

// Test different filter strengths
TEST(different_filter_strengths) {
    test_setup();
    input_manager_init();
    
    // Create sensors with different filter strengths
    sensor_definition_t filter_test_sensors[] = {
        {
            .pin = A0,
            .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION,
            .update_interval_us = 0,
            .filter_strength = 0,    // No filtering
            .name = "Unfiltered TPS"
        },
        {
            .pin = A1,
            .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_MANIFOLD_PRESSURE,
            .update_interval_us = 0,
            .filter_strength = 255,  // Maximum filtering
            .name = "Heavily Filtered MAP"
        }
    };
    
    input_manager_register_sensors(filter_test_sensors, 2);
    
    // Set initial voltages
    mock_set_analog_voltage(A0, 2.5f);  // 50%
    mock_set_analog_voltage(A1, 2.5f);  // 50%
    ensure_analog_mock_stable(A0);
    ensure_analog_mock_stable(A1);
    
    // First update (establishes baseline)
    input_manager_update();
    
    // Change voltages
    mock_set_analog_voltage(A0, 4.0f);  // 87.5%
    mock_set_analog_voltage(A1, 4.0f);  // 87.5%
    ensure_analog_mock_stable(A0);
    ensure_analog_mock_stable(A1);
    
    // Second update
    input_manager_update();
    
    sensor_runtime_t status0, status1;
    input_manager_get_sensor_status(0, &status0);
    input_manager_get_sensor_status(1, &status1);
    
    // Unfiltered sensor should jump to new value
    assert(float_equals(status0.calibrated_value, 87.5f, 5.0f));
    
    // Heavily filtered sensor should barely change
    assert(float_equals(status1.calibrated_value, 50.0f, 5.0f));
}

// =============================================================================
// ANALOG SENSOR TIMING TESTS
// =============================================================================

// Test analog sensor update timing (basic)
TEST(analog_sensor_timing) {
    test_setup();
    input_manager_init();
    
    // Create sensor with zero update interval and no filtering
    sensor_definition_t timing_sensor[] = {
        {
            .pin = A0, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION, .update_interval_us = 0, .filter_strength = 0, .name = "Timing Test"
        }
    };
    
    input_manager_register_sensors(timing_sensor, 1);
    
    // Set voltage
    mock_set_analog_voltage(A0, 2.5f);
    ensure_analog_mock_stable(A0);
    
    // Set initial time
    mock_set_micros(0);
    
    // First update (should work)
    input_manager_update();
    assert(input_manager_get_total_updates() == 1);
    
    // Second update immediately (should work with zero interval)
    input_manager_update();
    assert(input_manager_get_total_updates() == 2);
    
    // This test verifies basic timing logic works
}

// =============================================================================
// ANALOG SENSOR STATUS AND DIAGNOSTICS TESTS
// =============================================================================

// Test analog sensor status retrieval
TEST(analog_sensor_status) {
    test_setup();
    input_manager_init();
    
    // Create and register an analog sensor with no filtering
    sensor_definition_t analog_sensor[] = {
        {
            .pin = A0, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION, .update_interval_us = 0, .filter_strength = 0, .name = "Status Test"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(analog_sensor, 1);
    assert(registered == 1);
    
    // Set voltage and trigger update
    mock_set_analog_voltage(A0, 3.0f);  // 62.5% throttle
    ensure_analog_mock_stable(A0);
    
    // Get initial status
    sensor_runtime_t status;
    uint8_t result = input_manager_get_sensor_status(0, &status);
    assert(result == 1);
    
    // Update sensor
    input_manager_update();
    
    // Get updated status
    result = input_manager_get_sensor_status(0, &status);
    assert(result == 1);
    
    // Check status fields
    assert(status.is_valid == 1);
    assert(float_equals(status.calibrated_value, 62.5f, 5.0f));
    assert(status.raw_voltage > 2.9f && status.raw_voltage < 3.1f);
    assert(status.update_count > 0);
}

// Test finding analog sensors by message ID
TEST(analog_sensor_find_by_msg_id) {
    test_setup();
    input_manager_init();
    
    // Register multiple analog sensors
    sensor_definition_t analog_sensors[] = {
        DEFINE_LINEAR_SENSOR(A0, MSG_THROTTLE_POSITION, 0.5f, 4.5f, 0.0f, 100.0f, 50000, "TPS"),
        DEFINE_LINEAR_SENSOR(A1, MSG_MANIFOLD_PRESSURE, 0.5f, 4.5f, 20.0f, 300.0f, 25000, "MAP"),
        DEFINE_LINEAR_SENSOR(A2, MSG_OIL_PRESSURE, 0.5f, 4.5f, 0.0f, 100.0f, 100000, "Oil")
    };
    
    input_manager_register_sensors(analog_sensors, 3);
    
    // Test finding sensors by message ID
    int8_t tps_index = input_manager_find_sensor_by_msg_id(MSG_THROTTLE_POSITION);
    int8_t map_index = input_manager_find_sensor_by_msg_id(MSG_MANIFOLD_PRESSURE);
    int8_t oil_index = input_manager_find_sensor_by_msg_id(MSG_OIL_PRESSURE);
    int8_t nonexistent_index = input_manager_find_sensor_by_msg_id(0x999);
    
    assert(tps_index == 0);
    assert(map_index == 1);
    assert(oil_index == 2);
    assert(nonexistent_index == -1);
}

// Test analog sensor validation and error handling
TEST(analog_sensor_validation) {
    test_setup();
    input_manager_init();
    
    // Create analog sensor with no filtering
    sensor_definition_t test_sensor[] = {
        {
            .pin = A0, .type = SENSOR_ANALOG_LINEAR,
            .config.linear = {0.5f, 4.5f, 0.0f, 100.0f, 0},
            .msg_id = MSG_THROTTLE_POSITION, .update_interval_us = 0, .filter_strength = 0, .name = "Validation Test"
        }
    };
    
    input_manager_register_sensors(test_sensor, 1);
    
    // Test with valid voltage
    mock_set_analog_voltage(A0, 2.5f);
    ensure_analog_mock_stable(A0);
    
    input_manager_update();
    
    sensor_runtime_t status;
    input_manager_get_sensor_status(0, &status);
    assert(status.is_valid == 1);
    
    // Test with invalid voltage (too low - possible short circuit)
    mock_set_analog_voltage(A0, 0.05f);  // Very low voltage
    ensure_analog_mock_stable(A0);
    
    input_manager_update();
    
    // Should detect error condition
    assert(input_manager_get_total_errors() > 0);
}

// =============================================================================
// MAIN TEST RUNNER
// =============================================================================

int main() {
    std::cout << "=== Analog Linear Sensor Tests ===" << std::endl;
    
    // Run configuration tests
    std::cout << "\n--- Configuration Tests ---" << std::endl;
    run_test_analog_linear_sensor_registration();
    run_test_multiple_analog_sensors();
    run_test_different_voltage_ranges();
    
    // Run calibration tests
    std::cout << "\n--- Calibration Tests ---" << std::endl;
    run_test_linear_calibration_function();
    run_test_linear_calibration_different_ranges();
    run_test_linear_calibration_edge_cases();
    
    // Run reading and publishing tests
    std::cout << "\n--- Reading and Publishing Tests ---" << std::endl;
    run_test_analog_sensor_reading_and_publishing();
    run_test_multiple_analog_sensors_publishing();
    run_test_analog_sensor_voltage_changes();
    
    // Run filtering tests
    std::cout << "\n--- Filtering Tests ---" << std::endl;
    run_test_analog_sensor_filtering();
    run_test_different_filter_strengths();
    
    // Run timing tests
    std::cout << "\n--- Timing Tests ---" << std::endl;
    run_test_analog_sensor_timing();
    
    // Run status and diagnostics tests
    std::cout << "\n--- Status and Diagnostics Tests ---" << std::endl;
    run_test_analog_sensor_status();
    run_test_analog_sensor_find_by_msg_id();
    run_test_analog_sensor_validation();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Analog Linear Sensor Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL ANALOG LINEAR SENSOR TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME ANALOG LINEAR SENSOR TESTS FAILED!" << std::endl;
        return 1;
    }
} 