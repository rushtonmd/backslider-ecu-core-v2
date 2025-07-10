// tests/output_manager/test_output_manager.cpp
// Test suite for the output manager system

#include <iostream>
#include <cassert>

// Include enhanced mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

// Include message bus and output manager for testing
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../output_manager.h"
#include "../../output_manager_types.h"

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

// Test message reception for outputs
static float received_output_value = 0.0f;
static uint32_t received_msg_id = 0;
static bool output_message_received = false;

void test_output_message_handler(const CANMessage* msg) {
    received_output_value = MSG_UNPACK_FLOAT(msg);
    received_msg_id = msg->id;
    output_message_received = true;
}

// Test setup function to initialize mock environment
void test_setup() {
    mock_reset_all();
    
    // Reset test state
    output_message_received = false;
    received_output_value = 0.0f;
    received_msg_id = 0;
    
    // Reset mock pin states to known values
    for (int i = 0; i < 56; i++) {
        mock_digital_values[i] = LOW;
        mock_pin_modes[i] = INPUT;
    }
}

// Helper function to check if a pin was configured as output
bool is_pin_configured_as_output(int pin) {
    return mock_get_pin_mode(pin) == OUTPUT;
}

// Helper function to get current digital pin state
int get_pin_state(int pin) {
    return mock_digital_values[pin];
}

// Test basic output manager initialization
TEST(output_manager_initialization) {
    test_setup();
    
    uint8_t result = output_manager_init();
    
    // Check successful initialization
    assert(result == 1);
    
    // Check initial statistics
    const output_manager_stats_t* stats = output_manager_get_stats();
    assert(stats->total_outputs == 0);
    assert(stats->pwm_outputs == 0);
    assert(stats->digital_outputs == 0);
    assert(stats->analog_outputs == 0);
    assert(stats->spi_outputs == 0);
    assert(stats->virtual_outputs == 0);
    assert(stats->total_updates == 0);
    assert(stats->fault_count == 0);
    
    // Check initial fault state
    assert(output_manager_get_fault_count() == 0);
}

// Test PWM output registration
TEST(pwm_output_registration) {
    test_setup();
    output_manager_init();
    
    // Create a PWM output definition
    output_definition_t pwm_outputs[] = {
        {
            .pin = 23,
            .type = OUTPUT_PWM,
            .config = {.pwm = {1000, 10, 0.0f, 1.0f, 0.5f, 0}},
            .msg_id = MSG_TRANS_TCC_SOL,
            .current_value = 0.0f,
            .last_update_time_ms = 0,
            .update_rate_limit_ms = 50,
            .fault_detected = 0,
            .name = "Test_PWM"
        }
    };
    
    // Register the PWM output
    uint8_t registered = output_manager_register_outputs(pwm_outputs, 1);
    
    assert(registered == 1);
    
    // Check statistics updated
    const output_manager_stats_t* stats = output_manager_get_stats();
    assert(stats->total_outputs == 1);
    assert(stats->pwm_outputs == 1);
    assert(stats->digital_outputs == 0);
    
    // Check pin was configured as output
    assert(is_pin_configured_as_output(23));
}

// Test digital output registration
TEST(digital_output_registration) {
    test_setup();
    output_manager_init();
    
    // Create a digital output definition
    output_definition_t digital_outputs[] = {
        {
            .pin = 13,
            .type = OUTPUT_DIGITAL,
            .config = {.digital = {1, 0, 0}},
            .msg_id = MSG_SHIFT_LIGHT,
            .current_value = 0.0f,
            .last_update_time_ms = 0,
            .update_rate_limit_ms = 100,
            .fault_detected = 0,
            .name = "Test_Digital"
        }
    };
    
    // Register the digital output
    uint8_t registered = output_manager_register_outputs(digital_outputs, 1);
    
    assert(registered == 1);
    
    // Check statistics updated
    const output_manager_stats_t* stats = output_manager_get_stats();
    assert(stats->total_outputs == 1);
    assert(stats->digital_outputs == 1);
    assert(stats->pwm_outputs == 0);
    
    // Check pin was configured as output
    assert(is_pin_configured_as_output(13));
}

// Test direct output control
TEST(direct_output_control) {
    test_setup();
    output_manager_init();
    
    // Register a digital output
    output_definition_t test_output[] = {
        {
            .pin = 13,
            .type = OUTPUT_DIGITAL,
            .config = {.digital = {1, 0, 0}},
            .msg_id = MSG_SHIFT_LIGHT,
            .current_value = 0.0f,
            .last_update_time_ms = 0,
            .update_rate_limit_ms = 0,  // No rate limiting for test
            .fault_detected = 0,
            .name = "Test_LED"
        }
    };
    
    output_manager_register_outputs(test_output, 1);
    
    // Test direct control - turn on
    output_manager_set_value(0, 1.0f);
    
    // Check pin state changed
    assert(get_pin_state(13) == HIGH);
    assert(output_manager_get_value(0) == 1.0f);
    
    // Test direct control - turn off
    output_manager_set_value(0, 0.0f);
    
    // Check pin state changed
    assert(get_pin_state(13) == LOW);
    assert(output_manager_get_value(0) == 0.0f);
}

// Test message-driven output control
TEST(message_driven_control) {
    test_setup();
    g_message_bus.init(false);
    output_manager_init();
    
    // Register a digital output
    output_definition_t test_output[] = {
        {
            .pin = 13,
            .type = OUTPUT_DIGITAL,
            .config = {.digital = {1, 0, 0}},
            .msg_id = MSG_SHIFT_LIGHT,
            .current_value = 0.0f,
            .last_update_time_ms = 0,
            .update_rate_limit_ms = 0,  // No rate limiting
            .fault_detected = 0,
            .name = "Test_LED"
        }
    };
    
    output_manager_register_outputs(test_output, 1);
    
    // Publish message to turn on output
    g_message_bus.publishFloat(MSG_SHIFT_LIGHT, 1.0f, false);
    
    // Process message bus
    g_message_bus.process();
    
    // Check output was activated
    assert(get_pin_state(13) == HIGH);
    assert(output_manager_get_value(0) == 1.0f);
    
    // Publish message to turn off output
    g_message_bus.publishFloat(MSG_SHIFT_LIGHT, 0.0f, false);
    
    // Process message bus
    g_message_bus.process();
    
    // Check output was deactivated
    assert(get_pin_state(13) == LOW);
    assert(output_manager_get_value(0) == 0.0f);
}

// Main test runner
int main() {
    std::cout << "=== Output Manager Tests ===" << std::endl;
    
    // Run all tests
    run_test_output_manager_initialization();
    run_test_pwm_output_registration();
    run_test_digital_output_registration();
    run_test_direct_output_control();
    run_test_message_driven_control();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Output Manager Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL OUTPUT MANAGER TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME OUTPUT MANAGER TESTS FAILED!" << std::endl;
        return 1;
    }
} 