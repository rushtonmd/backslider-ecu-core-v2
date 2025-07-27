// tests/transmission_module/test_simple_output.cpp
// Simple test to debug output flow

#include <iostream>
#include <cassert>

// Include enhanced mock Arduino before any ECU code
#include "../mock_arduino.h"

// Include message bus and output manager
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../output_manager.h"

// Include storage manager for custom_canbus_manager
#include "../../storage_manager.h"
#include "../../spi_flash_storage_backend.h"

// Global instances for testing
static SPIFlashStorageBackend global_storage_backend;
static StorageManager global_storage_manager(&global_storage_backend);
StorageManager& g_storage_manager = global_storage_manager;

// Track pin state changes using mock Arduino's built-in tracking
extern uint8_t mock_digital_values[56];  // Access mock Arduino's pin state array

static int pin_21_changes = 0;
static int pin_22_changes = 0;
static int pin_21_previous_state = 0;
static int pin_22_previous_state = 0;

void check_pin_changes() {
    // Check if pin 21 changed
    if (mock_digital_values[21] != pin_21_previous_state) {
        pin_21_changes++;
        std::cout << "Pin 21 changed from " << pin_21_previous_state << " to " << (int)mock_digital_values[21] << std::endl;
        pin_21_previous_state = mock_digital_values[21];
    }
    
    // Check if pin 22 changed
    if (mock_digital_values[22] != pin_22_previous_state) {
        pin_22_changes++;
        std::cout << "Pin 22 changed from " << pin_22_previous_state << " to " << (int)mock_digital_values[22] << std::endl;
        pin_22_previous_state = mock_digital_values[22];
    }
}

int main() {
    std::cout << "=== Simple Output Test ===" << std::endl;
    
    // Initialize systems
    g_message_bus.init();
    output_manager_init();
    
    // Create a simple output definition
    output_definition_t test_outputs[2];
    
    // Shift Solenoid A (Digital ON/OFF) - Pin 21
    test_outputs[0].pin = 21;
    test_outputs[0].type = OUTPUT_DIGITAL;
    test_outputs[0].config.digital.active_high = 1;
    test_outputs[0].config.digital.default_state = 0;
    test_outputs[0].config.digital.open_drain = 0;
    test_outputs[0].msg_id = MSG_TRANS_SHIFT_SOL_A;
    test_outputs[0].current_value = 0.0f;
    test_outputs[0].last_update_time_ms = 0;
    test_outputs[0].update_rate_limit_ms = 10;
    test_outputs[0].fault_detected = 0;
    test_outputs[0].name = "Test Shift A";
    
    // Shift Solenoid B (Digital ON/OFF) - Pin 22
    test_outputs[1].pin = 22;
    test_outputs[1].type = OUTPUT_DIGITAL;
    test_outputs[1].config.digital.active_high = 1;
    test_outputs[1].config.digital.default_state = 0;
    test_outputs[1].config.digital.open_drain = 0;
    test_outputs[1].msg_id = MSG_TRANS_SHIFT_SOL_B;
    test_outputs[1].current_value = 0.0f;
    test_outputs[1].last_update_time_ms = 0;
    test_outputs[1].update_rate_limit_ms = 10;
    test_outputs[1].fault_detected = 0;
    test_outputs[1].name = "Test Shift B";
    
    // Register outputs
    uint8_t registered = output_manager_register_outputs(test_outputs, 2);
    std::cout << "Registered " << (int)registered << " outputs" << std::endl;
    
    // Reset pin tracking
    pin_21_changes = 0;
    pin_22_changes = 0;
    pin_21_previous_state = mock_digital_values[21];
    pin_22_previous_state = mock_digital_values[22];
    
    // Test 1: Direct message bus publishing
    std::cout << "\n1. Testing direct message bus publishing..." << std::endl;
    
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_A, 1.0f);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_B, 1.0f);
    
    std::cout << "Messages published, processing..." << std::endl;
    g_message_bus.process();
    
    check_pin_changes();
    std::cout << "Pin 21 changes: " << pin_21_changes << std::endl;
    std::cout << "Pin 22 changes: " << pin_22_changes << std::endl;
    
    // Test 2: Direct output manager calls
    std::cout << "\n2. Testing direct output manager calls..." << std::endl;
    
    pin_21_changes = 0;
    pin_22_changes = 0;
    pin_21_previous_state = mock_digital_values[21];
    pin_22_previous_state = mock_digital_values[22];
    
    // Check current time for rate limiting
    std::cout << "Current time: " << millis() << "ms" << std::endl;
    
    output_manager_set_value(0, 1.0f);  // Set output 0 (Pin 21) to 1.0f
    output_manager_set_value(1, 1.0f);  // Set output 1 (Pin 22) to 1.0f
    
    check_pin_changes();
    std::cout << "Pin 21 changes: " << pin_21_changes << std::endl;
    std::cout << "Pin 22 changes: " << pin_22_changes << std::endl;
    
    // Test 3: Wait and try again (rate limiting test)
    std::cout << "\n3. Testing after delay (rate limiting)..." << std::endl;
    
    // Advance time by 20ms to clear rate limiting
    mock_advance_time_ms(20);
    std::cout << "Advanced time to: " << millis() << "ms" << std::endl;
    
    pin_21_changes = 0;
    pin_22_changes = 0;
    pin_21_previous_state = mock_digital_values[21];
    pin_22_previous_state = mock_digital_values[22];
    
    output_manager_set_value(0, 0.0f);  // Set output 0 (Pin 21) to 0.0f
    output_manager_set_value(1, 0.0f);  // Set output 1 (Pin 22) to 0.0f
    
    check_pin_changes();
    std::cout << "Pin 21 changes: " << pin_21_changes << std::endl;
    std::cout << "Pin 22 changes: " << pin_22_changes << std::endl;
    
    std::cout << "\n✅ Simple output test completed!" << std::endl;
    
    return 0;
} 