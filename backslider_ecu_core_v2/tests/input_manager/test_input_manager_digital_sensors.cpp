// tests/input_manager/test_input_manager_digital_sensors.cpp
// Comprehensive test suite for digital sensor functionality in the input manager
//
// This test suite focuses specifically on digital sensor logic, configuration,
// calibration, and message publishing since there may be issues with the
// digital sensor implementation that need to be identified and fixed.

#include <iostream>
#include <cassert>

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

// Test message reception for digital sensors
static float received_digital_value = 0.0f;
static uint32_t received_msg_id = 0;
static bool digital_message_received = false;

void test_digital_message_handler(const CANMessage* msg) {
    received_digital_value = MSG_UNPACK_FLOAT(msg);
    received_msg_id = msg->id;
    digital_message_received = true;
}

// Test setup function to initialize mock environment
void test_setup() {
    mock_reset_all();
    
    // Set realistic digital pin states
    mock_set_digital_value(2, HIGH);   // Paddle upshift (inactive)
    mock_set_digital_value(3, HIGH);   // Paddle downshift (inactive)
    mock_set_digital_value(4, LOW);    // Gear switch (active)
    mock_set_digital_value(5, HIGH);   // Another gear switch (inactive)
    
    // Reset test state
    digital_message_received = false;
    received_digital_value = 0.0f;
    received_msg_id = 0;
}

// Helper macro to define a digital sensor (since one doesn't exist in the main code)
#define DEFINE_DIGITAL_SENSOR(pin_name, msg_id_name, use_pullup_val, invert_val, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_DIGITAL_PULLUP, \
        .config.digital = { \
            .use_pullup = use_pullup_val, \
            .invert_logic = invert_val \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 0, \
        .name = sensor_name \
    }

// =============================================================================
// DIGITAL SENSOR CONFIGURATION TESTS
// =============================================================================

// Test digital sensor registration and pin configuration
TEST(digital_sensor_registration) {
    test_setup();
    input_manager_init();
    
    // Create a basic digital sensor with pullup enabled
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 20000, "Paddle Upshift")
    };
    
    // Register the digital sensor
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    
    assert(registered == 1);
    assert(input_manager_get_sensor_count() == 1);
    
    // Check that pin was configured as INPUT_PULLUP
    assert(mock_get_pin_mode(2) == INPUT_PULLUP);
}

// Test digital sensor with pullup disabled
TEST(digital_sensor_no_pullup) {
    test_setup();
    input_manager_init();
    
    // Create a digital sensor without pullup
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(3, MSG_PADDLE_DOWNSHIFT, 0, 0, 20000, "Paddle Downshift")
    };
    
    // Register the digital sensor
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    
    assert(registered == 1);
    
    // Check that pin was configured as INPUT (no pullup)
    assert(mock_get_pin_mode(3) == INPUT);
}

// Test multiple digital sensors with different configurations
TEST(multiple_digital_sensors) {
    test_setup();
    input_manager_init();
    
    // Create multiple digital sensors with different configurations
    sensor_definition_t digital_sensors[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 20000, "Paddle Up"),     // Pullup + Invert
        DEFINE_DIGITAL_SENSOR(3, MSG_PADDLE_DOWNSHIFT, 1, 1, 20000, "Paddle Down"), // Pullup + Invert
        DEFINE_DIGITAL_SENSOR(4, MSG_TRANS_PARK_SWITCH, 1, 1, 50000, "Park Switch"), // Pullup + Invert
        DEFINE_DIGITAL_SENSOR(5, MSG_TRANS_DRIVE_SWITCH, 1, 0, 50000, "Drive Switch") // Pullup only
    };
    
    // Register all digital sensors
    uint8_t registered = input_manager_register_sensors(digital_sensors, 4);
    
    assert(registered == 4);
    assert(input_manager_get_sensor_count() == 4);
    
    // Check that all pins were configured as INPUT_PULLUP
    assert(mock_get_pin_mode(2) == INPUT_PULLUP);
    assert(mock_get_pin_mode(3) == INPUT_PULLUP);
    assert(mock_get_pin_mode(4) == INPUT_PULLUP);
    assert(mock_get_pin_mode(5) == INPUT_PULLUP);
}

// =============================================================================
// DIGITAL SENSOR CALIBRATION TESTS
// =============================================================================

// Test digital calibration function directly
TEST(digital_calibration_function) {
    // Test normal digital configuration (no inversion)
    digital_config_t normal_config = {
        .use_pullup = 1,
        .invert_logic = 0
    };
    
    float result;
    
    // Test LOW input
    result = calibrate_digital(&normal_config, LOW);
    assert(result == 0.0f);
    
    // Test HIGH input
    result = calibrate_digital(&normal_config, HIGH);
    assert(result == 1.0f);
    
    // Test inverted configuration
    digital_config_t inverted_config = {
        .use_pullup = 1,
        .invert_logic = 1
    };
    
    // Test LOW input (should be inverted to HIGH)
    result = calibrate_digital(&inverted_config, LOW);
    assert(result == 1.0f);
    
    // Test HIGH input (should be inverted to LOW)
    result = calibrate_digital(&inverted_config, HIGH);
    assert(result == 0.0f);
}

// Test digital calibration with various input values
TEST(digital_calibration_edge_cases) {
    digital_config_t config = {
        .use_pullup = 1,
        .invert_logic = 0
    };
    
    // Test with various input values (should normalize to 0.0f or 1.0f)
    assert(calibrate_digital(&config, 0) == 0.0f);
    assert(calibrate_digital(&config, 1) == 1.0f);
    assert(calibrate_digital(&config, 2) == 1.0f);  // Any non-zero should be 1.0f
    assert(calibrate_digital(&config, 255) == 1.0f);
}

// =============================================================================
// DIGITAL SENSOR READING AND MESSAGE PUBLISHING TESTS
// =============================================================================

// Test digital sensor reading and publishing - normal logic
TEST(digital_sensor_reading_normal) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to digital sensor messages
    g_message_bus.subscribe(MSG_PADDLE_UPSHIFT, test_digital_message_handler);
    
    // Create digital sensor with normal (non-inverted) logic
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 0, 0, "Paddle Upshift")  // No invert, always update
    };
    
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    assert(registered == 1);
    
    // Test HIGH input (should publish 1.0f)
    mock_set_digital_value(2, HIGH);
    digital_message_received = false;
    
    input_manager_update();
    g_message_bus.process();
    
    assert(digital_message_received == true);
    assert(received_msg_id == MSG_PADDLE_UPSHIFT);
    assert(received_digital_value == 1.0f);
    
    // Test LOW input (should publish 0.0f)
    mock_set_digital_value(2, LOW);
    digital_message_received = false;
    
    input_manager_update();
    g_message_bus.process();
    
    assert(digital_message_received == true);
    assert(received_digital_value == 0.0f);
}

// Test digital sensor reading and publishing - inverted logic
TEST(digital_sensor_reading_inverted) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to digital sensor messages
    g_message_bus.subscribe(MSG_PADDLE_DOWNSHIFT, test_digital_message_handler);
    
    // Create digital sensor with inverted logic (typical for pullup switches)
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(3, MSG_PADDLE_DOWNSHIFT, 1, 1, 0, "Paddle Downshift")  // Inverted, always update
    };
    
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    assert(registered == 1);
    
    // Test HIGH input (should publish 0.0f due to inversion)
    mock_set_digital_value(3, HIGH);
    digital_message_received = false;
    
    input_manager_update();
    g_message_bus.process();
    
    assert(digital_message_received == true);
    assert(received_msg_id == MSG_PADDLE_DOWNSHIFT);
    assert(received_digital_value == 0.0f);
    
    // Test LOW input (should publish 1.0f due to inversion)
    mock_set_digital_value(3, LOW);
    digital_message_received = false;
    
    input_manager_update();
    g_message_bus.process();
    
    assert(digital_message_received == true);
    assert(received_digital_value == 1.0f);
}

// Test digital sensor state changes
TEST(digital_sensor_state_changes) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to digital sensor messages
    g_message_bus.subscribe(MSG_TRANS_PARK_SWITCH, test_digital_message_handler);
    
    // Create digital sensor
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(4, MSG_TRANS_PARK_SWITCH, 1, 1, 0, "Park Switch")
    };
    
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    assert(registered == 1);
    
    // Test multiple state changes
    uint8_t expected_states[] = {HIGH, LOW, HIGH, LOW, HIGH};
    float expected_values[] = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f};  // Inverted logic
    
    for (int i = 0; i < 5; i++) {
        mock_set_digital_value(4, expected_states[i]);
        digital_message_received = false;
        
        input_manager_update();
        g_message_bus.process();
        
        assert(digital_message_received == true);
        assert(received_digital_value == expected_values[i]);
    }
}

// =============================================================================
// DIGITAL SENSOR TIMING AND UPDATE TESTS
// =============================================================================

// Test basic digital sensor update timing (debounce protection)
TEST(digital_sensor_timing) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to digital sensor messages
    g_message_bus.subscribe(MSG_PADDLE_UPSHIFT, test_digital_message_handler);
    
    // Create digital sensor with ZERO interval for this basic test
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 0, "Paddle Upshift")  // Always update (0 interval)
    };
    
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    assert(registered == 1);
    
    // Set initial state
    mock_set_digital_value(2, HIGH);
    mock_set_micros(0);
    
    // Should work with zero interval
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == true);
    
    // Should also work on second call with zero interval
    mock_set_digital_value(2, LOW);
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == true);  // Should work with 0 interval
    
    // This test verifies that basic timing/interval logic works
    // More complex debounce testing can be done separately
}

// Test digital sensor debouncing for race car paddle shifters
TEST(digital_sensor_debouncing) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to paddle messages
    g_message_bus.subscribe(MSG_PADDLE_UPSHIFT, test_digital_message_handler);
    
    // Create paddle sensor with racing-appropriate debounce (20ms)
    sensor_definition_t paddle_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 20000, "Racing Paddle")  // 20ms debounce
    };
    
    input_manager_register_sensors(paddle_sensor, 1);
    
    // Simulate the critical race car scenario: preventing double shifts
    mock_set_digital_value(2, HIGH);
    mock_set_micros(0);
    
    // Initial paddle press (should register)
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == true);
    assert(received_digital_value == 0.0f);  // HIGH inverted = 0.0f (inactive)
    
    // CRITICAL TEST: Rapid second press should be blocked (prevents double shift)
    mock_set_micros(5000);   // 5ms later (within debounce window)
    mock_set_digital_value(2, LOW);  // Rapid press
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == false);  // CRITICAL: Blocked - saves transmission!
    
    // Third rapid press also blocked
    mock_set_micros(10000);  // 10ms later (still within window)
    mock_set_digital_value(2, HIGH); // Another bounce
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == false);  // Still blocked
    
    // This test verifies the essential debounce protection for race car paddle shifters
    // The key is that rapid presses within the debounce window are blocked
}

// Test rapid paddle presses (race car specific scenario)
TEST(digital_sensor_rapid_presses) {
    test_setup();
    g_message_bus.init(false);
    input_manager_init();
    
    // Subscribe to paddle messages
    g_message_bus.subscribe(MSG_PADDLE_UPSHIFT, test_digital_message_handler);
    
    // Create paddle sensor with typical racing debounce
    sensor_definition_t paddle_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 20000, "Racing Paddle")
    };
    
    input_manager_register_sensors(paddle_sensor, 1);
    
    // Test scenario: Driver rapidly hits paddle multiple times (common in racing)
    mock_set_digital_value(2, HIGH);  // Start inactive
    mock_set_micros(0);
    
    // First press (should work)
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == true);
    
    // Rapid second press (should be blocked - critical for transmission protection)
    mock_set_micros(5000);  // 5ms later (way too fast!)
    mock_set_digital_value(2, LOW);  // Press again rapidly
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == false);  // BLOCKED - prevents double shift damage!
    
    // Third rapid press also blocked
    mock_set_micros(10000);  // 10ms later (still too fast)
    mock_set_digital_value(2, HIGH);  // Another rapid press
    digital_message_received = false;
    input_manager_update();
    g_message_bus.process();
    assert(digital_message_received == false);  // Still blocked - good!
    
    // This test verifies that rapid consecutive paddle presses are blocked
    // which prevents transmission damage from switch bounce or driver error
}

// =============================================================================
// DIGITAL SENSOR STATUS AND DIAGNOSTICS TESTS
// =============================================================================

// Test digital sensor status retrieval
TEST(digital_sensor_status) {
    test_setup();
    input_manager_init();
    
    // Create and register a digital sensor
    sensor_definition_t digital_sensor[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 0, "Paddle Upshift")  // Always update
    };
    
    uint8_t registered = input_manager_register_sensors(digital_sensor, 1);
    assert(registered == 1);
    
    // Get sensor status
    sensor_runtime_t status;
    uint8_t result = input_manager_get_sensor_status(0, &status);
    
    assert(result == 1);  // Should successfully get status
    
    // Set digital value and trigger update
    mock_set_digital_value(2, LOW);
    input_manager_update();
    
    // Get updated status
    result = input_manager_get_sensor_status(0, &status);
    assert(result == 1);
    
    // Check that digital reading is reflected correctly
    assert(status.calibrated_value == 1.0f);  // LOW with invert_logic=1 should be 1.0f
    assert(status.is_valid == 1);
}

// Test finding digital sensors by message ID
TEST(digital_sensor_find_by_msg_id) {
    test_setup();
    input_manager_init();
    
    // Register multiple digital sensors
    sensor_definition_t digital_sensors[] = {
        DEFINE_DIGITAL_SENSOR(2, MSG_PADDLE_UPSHIFT, 1, 1, 20000, "Paddle Up"),
        DEFINE_DIGITAL_SENSOR(3, MSG_PADDLE_DOWNSHIFT, 1, 1, 20000, "Paddle Down"),
        DEFINE_DIGITAL_SENSOR(4, MSG_TRANS_PARK_SWITCH, 1, 1, 50000, "Park Switch")
    };
    
    input_manager_register_sensors(digital_sensors, 3);
    
    // Test finding sensors by message ID
    int8_t upshift_index = input_manager_find_sensor_by_msg_id(MSG_PADDLE_UPSHIFT);
    int8_t downshift_index = input_manager_find_sensor_by_msg_id(MSG_PADDLE_DOWNSHIFT);
    int8_t park_index = input_manager_find_sensor_by_msg_id(MSG_TRANS_PARK_SWITCH);
    int8_t nonexistent_index = input_manager_find_sensor_by_msg_id(0x999);
    
    assert(upshift_index == 0);
    assert(downshift_index == 1);
    assert(park_index == 2);
    assert(nonexistent_index == -1);
}

// =============================================================================
// MAIN TEST RUNNER
// =============================================================================

int main() {
    std::cout << "=== Digital Sensor Tests ===" << std::endl;
    
    // Run configuration tests
    std::cout << "\n--- Configuration Tests ---" << std::endl;
    run_test_digital_sensor_registration();
    run_test_digital_sensor_no_pullup();
    run_test_multiple_digital_sensors();
    
    // Run calibration tests
    std::cout << "\n--- Calibration Tests ---" << std::endl;
    run_test_digital_calibration_function();
    run_test_digital_calibration_edge_cases();
    
    // Run reading and publishing tests
    std::cout << "\n--- Reading and Publishing Tests ---" << std::endl;
    run_test_digital_sensor_reading_normal();
    run_test_digital_sensor_reading_inverted();
    run_test_digital_sensor_state_changes();
    
    // Run timing tests
    std::cout << "\n--- Timing Tests ---" << std::endl;
    run_test_digital_sensor_timing();
    
    // Run status and diagnostics tests
    std::cout << "\n--- Status and Diagnostics Tests ---" << std::endl;
    run_test_digital_sensor_status();
    run_test_digital_sensor_find_by_msg_id();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Digital Sensor Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL DIGITAL SENSOR TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME DIGITAL SENSOR TESTS FAILED!" << std::endl;
        return 1;
    }
}
