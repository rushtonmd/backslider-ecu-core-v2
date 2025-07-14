// tests/input_manager/test_input_manager_thermistors.cpp
// Comprehensive test suite for thermistor sensor functionality in the input manager
//
// This test suite focuses specifically on SENSOR_THERMISTOR sensor logic,
// configuration, calibration, lookup table interpolation, and message publishing
// to ensure the thermistor sensor implementation works correctly.
//
// Tests automotive temperature sensors like CTS, IAT, transmission fluid temp, etc.

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

// Test message reception for thermistor sensors
static float received_temp_value = 0.0f;
static uint32_t received_msg_id = 0;
static bool temp_message_received = false;

void test_temp_message_handler(const CANMessage* msg) {
    received_temp_value = MSG_UNPACK_FLOAT(msg);
    received_msg_id = msg->id;
    temp_message_received = true;
}

// Test setup function to initialize mock environment
void test_setup() {
    mock_reset_all();
    
    // Set realistic thermistor voltages for testing
    // These correspond to various temperature readings
    mock_set_analog_voltage(A3, 2.5f);  // CTS at ~20°C (room temp)
    mock_set_analog_voltage(A4, 2.0f);  // IAT at ~40°C (warm intake air)
    mock_set_analog_voltage(A13, 1.5f); // Trans fluid at ~60°C (operating temp)
    mock_set_analog_voltage(A17, 3.0f); // Ambient at ~0°C (cold day)
    
    // Reset test state
    temp_message_received = false;
    received_temp_value = 0.0f;
    received_msg_id = 0;
}

// Helper function to ensure mock analog readings are stable
void ensure_thermistor_mock_stable(int pin) {
    // Force mock to settle by reading twice
    analogRead(pin);
    analogRead(pin);
}

// Helper function to check if two temperatures are approximately equal
bool temp_equals(float a, float b, float tolerance = 2.0f) {
    return fabs(a - b) < tolerance;
}

// =============================================================================
// THERMISTOR SENSOR CONFIGURATION TESTS
// =============================================================================

// Test basic thermistor sensor registration
TEST(thermistor_sensor_registration) {
    test_setup();
    input_manager_init();
    
    // Create a basic CTS sensor using standard thermistor table
    sensor_definition_t cts_sensor[] = {
        DEFINE_THERMISTOR_SENSOR(
            A3,                                // pin (PIN_CTS)
            MSG_COOLANT_TEMP,                  // message ID
            2200,                              // 2.2kΩ pullup resistor
            STANDARD_THERMISTOR_VOLTAGE_TABLE, // voltage lookup table
            STANDARD_THERMISTOR_TEMP_TABLE,    // temperature lookup table
            STANDARD_THERMISTOR_TABLE_SIZE,    // table size
            100000,                            // 100ms update interval
            "CTS Sensor"                       // name
        )
    };
    
    // Register the sensor
    uint8_t registered = input_manager_register_sensors(cts_sensor, 1);
    
    assert(registered == 1);
    assert(input_manager_get_sensor_count() == 1);
    
    // Verify sensor configuration
    sensor_runtime_t status;
    uint8_t result = input_manager_get_sensor_status(0, &status);
    assert(result == 1);
}

// Test multiple thermistor sensors with different types
TEST(multiple_thermistor_sensors) {
    test_setup();
    input_manager_init();
    
    // Create multiple automotive temperature sensors
    sensor_definition_t temp_sensors[] = {
        // Coolant Temperature Sensor (CTS) - Standard thermistor
        DEFINE_THERMISTOR_SENSOR(A3, MSG_COOLANT_TEMP, 2200,
                               STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE,
                               STANDARD_THERMISTOR_TABLE_SIZE, 200000, "CTS"),
        
        // Intake Air Temperature (IAT) - Generic IAT thermistor  
        DEFINE_THERMISTOR_SENSOR(A4, MSG_AIR_INTAKE_TEMP, 2200,
                               GENERIC_IAT_VOLTAGE_TABLE, GENERIC_IAT_TEMP_TABLE,
                               GENERIC_IAT_TABLE_SIZE, 200000, "IAT"),
        
        // Transmission Fluid Temperature - Standard thermistor
        DEFINE_THERMISTOR_SENSOR(A13, MSG_TRANS_FLUID_TEMP, 2200,
                               STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE,
                               STANDARD_THERMISTOR_TABLE_SIZE, 500000, "Trans Fluid Temp")
    };
    
    // Register all sensors
    uint8_t registered = input_manager_register_sensors(temp_sensors, 3);
    
    assert(registered == 3);
    assert(input_manager_get_sensor_count() == 3);
    
    // Verify each sensor can be found by message ID
    assert(input_manager_find_sensor_by_msg_id(MSG_COOLANT_TEMP) == 0);
    assert(input_manager_find_sensor_by_msg_id(MSG_AIR_INTAKE_TEMP) == 1);
    assert(input_manager_find_sensor_by_msg_id(MSG_TRANS_FLUID_TEMP) == 2);
}

// Test different thermistor configurations (pullup resistors, table types)
TEST(different_thermistor_configurations) {
    test_setup();
    input_manager_init();
    
    // Create sensors with different configurations
    sensor_definition_t config_test_sensors[] = {
        // Standard 2.2kΩ pullup with standard table
        DEFINE_THERMISTOR_SENSOR(A3, MSG_COOLANT_TEMP, 2200,
                               STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE,
                               STANDARD_THERMISTOR_TABLE_SIZE, 100000, "Standard CTS"),
        
        // Different pullup value (1kΩ) with GM table
        DEFINE_THERMISTOR_SENSOR(A4, MSG_AIR_INTAKE_TEMP, 1000,
                               GM_CTS_VOLTAGE_TABLE, GM_CTS_TEMP_TABLE,
                               GM_CTS_TABLE_SIZE, 100000, "GM Style IAT"),
        
        // Higher pullup (4.7kΩ) with generic IAT table
        DEFINE_THERMISTOR_SENSOR(A13, MSG_TRANS_FLUID_TEMP, 4700,
                               GENERIC_IAT_VOLTAGE_TABLE, GENERIC_IAT_TEMP_TABLE,
                               GENERIC_IAT_TABLE_SIZE, 100000, "High Pullup Trans")
    };
    
    // Register all sensors
    uint8_t registered = input_manager_register_sensors(config_test_sensors, 3);
    
    assert(registered == 3);
    assert(input_manager_get_sensor_count() == 3);
}

// =============================================================================
// THERMISTOR CALIBRATION TESTS
// =============================================================================

// Test thermistor calibration function directly
TEST(thermistor_calibration_function) {
    // Test standard thermistor calibration
    thermistor_config_t standard_config = {
        .pullup_ohms = 2200,
        .voltage_table = STANDARD_THERMISTOR_VOLTAGE_TABLE,
        .temp_table = STANDARD_THERMISTOR_TEMP_TABLE,
        .table_size = STANDARD_THERMISTOR_TABLE_SIZE
    };
    
    float result;
    
    // Test exact table match (2.5V should give 20°C)
    result = calibrate_thermistor(&standard_config, 2.5f);
    assert(temp_equals(result, 20.0f, 0.1f));
    
    // Test another exact match (2.0V should give 40°C)
    result = calibrate_thermistor(&standard_config, 2.0f);
    assert(temp_equals(result, 40.0f, 0.1f));
    
    // Test interpolation between points (2.25V should be ~30°C)
    result = calibrate_thermistor(&standard_config, 2.25f);
    assert(temp_equals(result, 30.0f, 2.0f));
}

// Test different thermistor table types
TEST(different_thermistor_tables) {
    // Test GM CTS calibration
    thermistor_config_t gm_config = {
        .pullup_ohms = 2200,
        .voltage_table = GM_CTS_VOLTAGE_TABLE,
        .temp_table = GM_CTS_TEMP_TABLE,
        .table_size = GM_CTS_TABLE_SIZE
    };
    
    float result;
    
    // Test GM table specific points
    result = calibrate_thermistor(&gm_config, 2.4f);
    assert(temp_equals(result, 35.0f, 1.0f));  // Should be close to 35°C
    
    // Test Generic IAT calibration
    thermistor_config_t iat_config = {
        .pullup_ohms = 2200,
        .voltage_table = GENERIC_IAT_VOLTAGE_TABLE,
        .temp_table = GENERIC_IAT_TEMP_TABLE,
        .table_size = GENERIC_IAT_TABLE_SIZE
    };
    
    // Test IAT table (should be same as standard for this implementation)
    result = calibrate_thermistor(&iat_config, 2.5f);
    assert(temp_equals(result, 20.0f, 0.1f));
}

// Test thermistor calibration edge cases
TEST(thermistor_calibration_edge_cases) {
    thermistor_config_t config = {
        .pullup_ohms = 2200,
        .voltage_table = STANDARD_THERMISTOR_VOLTAGE_TABLE,
        .temp_table = STANDARD_THERMISTOR_TEMP_TABLE,
        .table_size = STANDARD_THERMISTOR_TABLE_SIZE
    };
    
    float result;
    
    // Test below minimum voltage (should clamp to highest temperature)
    result = calibrate_thermistor(&config, 0.1f);
    assert(temp_equals(result, 120.0f, 1.0f));
    
    // Test above maximum voltage (should clamp to lowest temperature)
    result = calibrate_thermistor(&config, 5.0f);
    assert(temp_equals(result, -60.0f, 1.0f));
    
    // Test null pointer safety
    result = calibrate_thermistor(nullptr, 2.5f);
    assert(temp_equals(result, 20.0f, 0.1f));  // Should return default room temp
}

// =============================================================================
// LOOKUP TABLE INTERPOLATION TESTS  
// =============================================================================

// Test the interpolate_table function directly
TEST(table_interpolation_function) {
    // Create simple test tables
    const float voltage_table[] = {1.0f, 2.0f, 3.0f, 4.0f};
    const float temp_table[] = {80.0f, 60.0f, 40.0f, 20.0f};
    const uint8_t table_size = 4;
    
    float result;
    
    // Test exact matches
    result = interpolate_table(voltage_table, temp_table, table_size, 2.0f);
    assert(temp_equals(result, 60.0f, 0.1f));
    
    result = interpolate_table(voltage_table, temp_table, table_size, 3.0f);
    assert(temp_equals(result, 40.0f, 0.1f));
    
    // Test interpolation between points (2.5V should be 50°C)
    result = interpolate_table(voltage_table, temp_table, table_size, 2.5f);
    assert(temp_equals(result, 50.0f, 0.1f));
    
    // Test interpolation between other points (3.5V should be 30°C)
    result = interpolate_table(voltage_table, temp_table, table_size, 3.5f);
    assert(temp_equals(result, 30.0f, 0.1f));
}

// Test table interpolation edge cases
TEST(table_interpolation_edge_cases) {
    const float voltage_table[] = {1.0f, 2.0f, 3.0f, 4.0f};
    const float temp_table[] = {80.0f, 60.0f, 40.0f, 20.0f};
    const uint8_t table_size = 4;
    
    float result;
    
    // Test below minimum (should clamp to first value)
    result = interpolate_table(voltage_table, temp_table, table_size, 0.5f);
    assert(temp_equals(result, 80.0f, 0.1f));
    
    // Test above maximum (should clamp to last value)
    result = interpolate_table(voltage_table, temp_table, table_size, 5.0f);
    assert(temp_equals(result, 20.0f, 0.1f));
    
    // Test null pointer safety
    result = interpolate_table(nullptr, temp_table, table_size, 2.0f);
    assert(temp_equals(result, 0.0f, 0.1f));
    
    result = interpolate_table(voltage_table, nullptr, table_size, 2.0f);
    assert(temp_equals(result, 0.0f, 0.1f));
    
    // Test invalid table size
    result = interpolate_table(voltage_table, temp_table, 1, 2.0f);
    assert(temp_equals(result, 0.0f, 0.1f));
}

// =============================================================================
// THERMISTOR READING AND PUBLISHING TESTS
// =============================================================================

// Test thermistor sensor reading and message publishing
TEST(thermistor_reading_and_publishing) {
    test_setup();
    g_message_bus.init();
    input_manager_init();
    
    // Subscribe to coolant temperature messages
    g_message_bus.subscribe(MSG_COOLANT_TEMP, test_temp_message_handler);
    
    // Create CTS sensor with no filtering for predictable results
    sensor_definition_t cts_sensor[] = {
        {
            .pin = A3,
            .type = SENSOR_THERMISTOR,
            .config.thermistor = {
                .pullup_ohms = 2200,
                .voltage_table = STANDARD_THERMISTOR_VOLTAGE_TABLE,
                .temp_table = STANDARD_THERMISTOR_TEMP_TABLE,
                .table_size = STANDARD_THERMISTOR_TABLE_SIZE
            },
            .msg_id = MSG_COOLANT_TEMP,
            .update_interval_us = 0,
            .filter_strength = 0,  // No filtering for predictable test results
            .name = "CTS Test"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(cts_sensor, 1);
    assert(registered == 1);
    
    // Test at room temperature (2.5V should give ~20°C)
    mock_set_analog_voltage(A3, 2.5f);
    ensure_thermistor_mock_stable(A3);
    
    temp_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(temp_message_received == true);
    assert(received_msg_id == MSG_COOLANT_TEMP);
    assert(temp_equals(received_temp_value, 20.0f, 2.0f));  // 20°C ± 2°C
    
    // Test at operating temperature (2.0V should give ~40°C)
    mock_set_analog_voltage(A3, 2.0f);
    ensure_thermistor_mock_stable(A3);
    
    temp_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(temp_message_received == true);
    assert(temp_equals(received_temp_value, 40.0f, 2.0f));  // 40°C ± 2°C
    
    // Test at hot temperature (1.5V should give ~60°C)
    mock_set_analog_voltage(A3, 1.5f);
    ensure_thermistor_mock_stable(A3);
    
    temp_message_received = false;
    input_manager_update();
    g_message_bus.process();
    
    assert(temp_message_received == true);
    assert(temp_equals(received_temp_value, 60.0f, 2.0f));  // 60°C ± 2°C
}

// Test multiple thermistor sensors publishing simultaneously
TEST(multiple_thermistors_publishing) {
    test_setup();
    g_message_bus.init();
    input_manager_init();
    
    // Subscribe to multiple temperature sensor messages
    g_message_bus.subscribe(MSG_COOLANT_TEMP, test_temp_message_handler);
    g_message_bus.subscribe(MSG_AIR_INTAKE_TEMP, test_temp_message_handler);
    g_message_bus.subscribe(MSG_TRANS_FLUID_TEMP, test_temp_message_handler);
    
    // Create multiple temperature sensors with no filtering
    sensor_definition_t temp_sensors[] = {
        {
            .pin = A3, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_COOLANT_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "CTS"
        },
        {
            .pin = A4, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, GENERIC_IAT_VOLTAGE_TABLE, GENERIC_IAT_TEMP_TABLE, GENERIC_IAT_TABLE_SIZE},
            .msg_id = MSG_AIR_INTAKE_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "IAT"
        },
        {
            .pin = A13, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_TRANS_FLUID_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "Trans Temp"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(temp_sensors, 3);
    assert(registered == 3);
    
    // Set different voltages for different temperatures
    mock_set_analog_voltage(A3, 2.0f);   // CTS at ~40°C (normal operating)
    mock_set_analog_voltage(A4, 2.5f);   // IAT at ~20°C (cool air)
    mock_set_analog_voltage(A13, 1.5f);  // Trans at ~60°C (warm fluid)
    
    // Ensure all mocks are stable
    ensure_thermistor_mock_stable(A3);
    ensure_thermistor_mock_stable(A4);
    ensure_thermistor_mock_stable(A13);
    
    // Update all sensors
    input_manager_update();
    g_message_bus.process();
    
    // Check that all sensors published reasonable values
    assert(input_manager_get_total_updates() >= 3);
    
    // Check individual sensor values
    sensor_runtime_t status;
    
    // CTS should be around 40°C
    input_manager_get_sensor_status(0, &status);
    assert(temp_equals(status.calibrated_value, 40.0f, 3.0f));
    
    // IAT should be around 20°C  
    input_manager_get_sensor_status(1, &status);
    assert(temp_equals(status.calibrated_value, 20.0f, 3.0f));
    
    // Trans temp should be around 60°C
    input_manager_get_sensor_status(2, &status);
    assert(temp_equals(status.calibrated_value, 60.0f, 3.0f));
}

// Test thermistor response to changing temperatures
TEST(thermistor_temperature_changes) {
    test_setup();
    g_message_bus.init();
    input_manager_init();
    
    // Subscribe to coolant temperature messages
    g_message_bus.subscribe(MSG_COOLANT_TEMP, test_temp_message_handler);
    
    // Create CTS sensor with no filtering
    sensor_definition_t cts_sensor[] = {
        {
            .pin = A3, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_COOLANT_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "CTS"
        }
    };
    
    input_manager_register_sensors(cts_sensor, 1);
    
    // Test a sequence of different temperature readings
    float test_voltages[] = {4.0f, 3.0f, 2.0f, 1.0f, 0.5f};
    float expected_temps[] = {-40.0f, 0.0f, 40.0f, 80.0f, 100.0f}; // Approximate temperatures
    
    for (int i = 0; i < 5; i++) {
        mock_set_analog_voltage(A3, test_voltages[i]);
        ensure_thermistor_mock_stable(A3);
        
        temp_message_received = false;
        input_manager_update();
        g_message_bus.process();
        
        assert(temp_message_received == true);
        assert(temp_equals(received_temp_value, expected_temps[i], 5.0f));
    }
}

// =============================================================================
// THERMISTOR FILTERING TESTS
// =============================================================================

// Test thermistor sensor filtering (temperature sensors typically have heavy filtering)
TEST(thermistor_filtering) {
    test_setup();
    g_message_bus.init();
    input_manager_init();
    
    // Create CTS sensor with heavy filtering (typical for temperature sensors)
    sensor_definition_t filtered_cts[] = {
        {
            .pin = A3,
            .type = SENSOR_THERMISTOR,
            .config.thermistor = {
                .pullup_ohms = 2200,
                .voltage_table = STANDARD_THERMISTOR_VOLTAGE_TABLE,
                .temp_table = STANDARD_THERMISTOR_TEMP_TABLE,
                .table_size = STANDARD_THERMISTOR_TABLE_SIZE
            },
            .msg_id = MSG_COOLANT_TEMP,
            .update_interval_us = 0,
            .filter_strength = 200,  // Heavy filtering typical for temperature sensors
            .name = "Filtered CTS"
        }
    };
    
    input_manager_register_sensors(filtered_cts, 1);
    
    // Start with room temperature
    mock_set_analog_voltage(A3, 2.5f);  // ~20°C
    ensure_thermistor_mock_stable(A3);
    
    // First reading should be unfiltered
    input_manager_update();
    sensor_runtime_t status;
    input_manager_get_sensor_status(0, &status);
    float first_reading = status.calibrated_value;
    assert(temp_equals(first_reading, 20.0f, 2.0f));
    
    // Change to hot temperature suddenly
    mock_set_analog_voltage(A3, 1.5f);  // ~60°C
    ensure_thermistor_mock_stable(A3);
    
    // Second reading should be heavily filtered (not jump immediately to 60°C)
    input_manager_update();
    input_manager_get_sensor_status(0, &status);
    float second_reading = status.calibrated_value;
    
    // Should be somewhere between first reading and target, but much closer to first
    assert(second_reading > first_reading);
    assert(second_reading < 40.0f);  // Should not reach halfway yet due to heavy filtering
    
    // Multiple updates should slowly converge toward target
    for (int i = 0; i < 20; i++) {
        input_manager_update();
    }
    
    input_manager_get_sensor_status(0, &status);
    float final_reading = status.calibrated_value;
    
    // Should be much closer to target after many updates
    assert(final_reading > second_reading);
    assert(temp_equals(final_reading, 60.0f, 10.0f));  // Should be approaching 60°C
}

// =============================================================================
// THERMISTOR STATUS AND DIAGNOSTICS TESTS
// =============================================================================

// Test thermistor sensor status retrieval
TEST(thermistor_sensor_status) {
    test_setup();
    input_manager_init();
    
    // Create and register a thermistor sensor
    sensor_definition_t thermistor_sensor[] = {
        {
            .pin = A3, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_COOLANT_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "Status Test"
        }
    };
    
    uint8_t registered = input_manager_register_sensors(thermistor_sensor, 1);
    assert(registered == 1);
    
    // Set voltage for normal operating temperature (2.0V = ~40°C)
    mock_set_analog_voltage(A3, 2.0f);
    ensure_thermistor_mock_stable(A3);
    
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
    assert(temp_equals(status.calibrated_value, 40.0f, 3.0f));
    assert(status.raw_voltage > 1.9f && status.raw_voltage < 2.1f);
    assert(status.update_count > 0);
}

// Test finding thermistor sensors by message ID
TEST(thermistor_find_by_msg_id) {
    test_setup();
    input_manager_init();
    
    // Register multiple thermistor sensors
    sensor_definition_t thermistor_sensors[] = {
        {
            .pin = A3, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_COOLANT_TEMP, .update_interval_us = 100000, .filter_strength = 128, .name = "CTS"
        },
        {
            .pin = A4, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, GENERIC_IAT_VOLTAGE_TABLE, GENERIC_IAT_TEMP_TABLE, GENERIC_IAT_TABLE_SIZE},
            .msg_id = MSG_AIR_INTAKE_TEMP, .update_interval_us = 200000, .filter_strength = 64, .name = "IAT"
        },
        {
            .pin = A13, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_TRANS_FLUID_TEMP, .update_interval_us = 500000, .filter_strength = 200, .name = "Trans Temp"
        }
    };
    
    input_manager_register_sensors(thermistor_sensors, 3);
    
    // Test finding sensors by message ID
    int8_t cts_index = input_manager_find_sensor_by_msg_id(MSG_COOLANT_TEMP);
    int8_t iat_index = input_manager_find_sensor_by_msg_id(MSG_AIR_INTAKE_TEMP);
    int8_t trans_index = input_manager_find_sensor_by_msg_id(MSG_TRANS_FLUID_TEMP);
    int8_t nonexistent_index = input_manager_find_sensor_by_msg_id(0x999);
    
    assert(cts_index == 0);
    assert(iat_index == 1);
    assert(trans_index == 2);
    assert(nonexistent_index == -1);
}

// Test thermistor sensor validation and error handling
TEST(thermistor_validation) {
    test_setup();
    input_manager_init();
    
    // Create thermistor sensor
    sensor_definition_t test_sensor[] = {
        {
            .pin = A3, .type = SENSOR_THERMISTOR,
            .config.thermistor = {2200, STANDARD_THERMISTOR_VOLTAGE_TABLE, STANDARD_THERMISTOR_TEMP_TABLE, STANDARD_THERMISTOR_TABLE_SIZE},
            .msg_id = MSG_COOLANT_TEMP, .update_interval_us = 0, .filter_strength = 0, .name = "Validation Test"
        }
    };
    
    input_manager_register_sensors(test_sensor, 1);
    
    // Test with valid voltage
    mock_set_analog_voltage(A3, 2.5f);
    ensure_thermistor_mock_stable(A3);
    
    input_manager_update();
    
    sensor_runtime_t status;
    input_manager_get_sensor_status(0, &status);
    assert(status.is_valid == 1);
    
    // Test with invalid voltage (too low - possible short circuit)
    mock_set_analog_voltage(A3, 0.05f);  // Very low voltage
    ensure_thermistor_mock_stable(A3);
    
    input_manager_update();
    
    // Should detect error condition
    assert(input_manager_get_total_errors() > 0);
}

// =============================================================================
// MAIN TEST RUNNER
// =============================================================================

int main() {
    std::cout << "=== Thermistor Sensor Tests ===" << std::endl;
    
    // Run configuration tests
    std::cout << "\n--- Configuration Tests ---" << std::endl;
    run_test_thermistor_sensor_registration();
    run_test_multiple_thermistor_sensors();
    run_test_different_thermistor_configurations();
    
    // Run calibration tests
    std::cout << "\n--- Calibration Tests ---" << std::endl;
    run_test_thermistor_calibration_function();
    run_test_different_thermistor_tables();
    run_test_thermistor_calibration_edge_cases();
    
    // Run lookup table interpolation tests
    std::cout << "\n--- Lookup Table Interpolation Tests ---" << std::endl;
    run_test_table_interpolation_function();
    run_test_table_interpolation_edge_cases();
    
    // Run reading and publishing tests
    std::cout << "\n--- Reading and Publishing Tests ---" << std::endl;
    run_test_thermistor_reading_and_publishing();
    run_test_multiple_thermistors_publishing();
    run_test_thermistor_temperature_changes();
    
    // Run filtering tests
    std::cout << "\n--- Filtering Tests ---" << std::endl;
    run_test_thermistor_filtering();
    
    // Run status and diagnostics tests
    std::cout << "\n--- Status and Diagnostics Tests ---" << std::endl;
    run_test_thermistor_sensor_status();
    run_test_thermistor_find_by_msg_id();
    run_test_thermistor_validation();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Thermistor Sensor Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL THERMISTOR SENSOR TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME THERMISTOR SENSOR TESTS FAILED!" << std::endl;
        return 1;
    }
} 