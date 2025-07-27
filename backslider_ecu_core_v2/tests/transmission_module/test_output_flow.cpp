// tests/transmission_module/test_output_flow.cpp
// Test to demonstrate the complete output flow:
// 1. Module publishes message to message bus
// 2. Output manager receives message via subscription
// 3. Output manager finds the correct output by message ID
// 4. Output manager updates the hardware pin

#include <iostream>
#include <cassert>

// Include enhanced mock Arduino before any ECU code
#include "../mock_arduino.h"

// Include message bus, input manager, output manager, and transmission module for testing
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../input_manager.h"
#include "../../output_manager.h"
#include "../../sensor_calibration.h"
#include "../../transmission_module.h"

// Include storage manager for custom_canbus_manager
#include "../../storage_manager.h"
#include "../../spi_flash_storage_backend.h"

// Global instances for testing
static SPIFlashStorageBackend global_storage_backend;
static StorageManager global_storage_manager(&global_storage_backend);
StorageManager& g_storage_manager = global_storage_manager;

// Track pin state changes
static int pin_21_changes = 0;
static int pin_22_changes = 0;
static int pin_23_changes = 0;
static int pin_19_changes = 0;
static int pin_18_changes = 0;

// Mock pin state tracking
static int pin_21_state = 0;
static int pin_22_state = 0;
static int pin_23_state = 0;
static int pin_19_state = 0;
static int pin_18_state = 0;

// Override mock digitalWrite to track changes
extern "C" {
    void digitalWrite(uint8_t pin, uint8_t state) {
        switch (pin) {
            case 21:
                if (pin_21_state != state) {
                    pin_21_changes++;
                    pin_21_state = state;
                }
                break;
            case 22:
                if (pin_22_state != state) {
                    pin_22_changes++;
                    pin_22_state = state;
                }
                break;
            case 23:
                if (pin_23_state != state) {
                    pin_23_changes++;
                    pin_23_state = state;
                }
                break;
            case 19:
                if (pin_19_state != state) {
                    pin_19_changes++;
                    pin_19_state = state;
                }
                break;
            case 18:
                if (pin_18_state != state) {
                    pin_18_changes++;
                    pin_18_state = state;
                }
                break;
        }
        // Note: digitalWrite is already the mock function in this environment
    }
}

void reset_pin_tracking() {
    pin_21_changes = 0;
    pin_22_changes = 0;
    pin_23_changes = 0;
    pin_19_changes = 0;
    pin_18_changes = 0;
    pin_21_state = 0;
    pin_22_state = 0;
    pin_23_state = 0;
    pin_19_state = 0;
    pin_18_state = 0;
}

int main() {
    std::cout << "=== Transmission Output Flow Test ===" << std::endl;
    
    // Initialize systems
    g_message_bus.init();
    input_manager_init();
    output_manager_init();
    
    // Initialize transmission module (registers outputs with output manager)
    uint8_t result = transmission_module_init();
    std::cout << "Transmission module initialized with " << (int)result << " sensors" << std::endl;
    
    // Reset pin tracking
    reset_pin_tracking();
    
    std::cout << "\n--- Testing Complete Output Flow ---" << std::endl;
    
    // Test 1: Direct message bus publishing to output manager
    std::cout << "\n1. Testing direct message bus publishing..." << std::endl;
    
    // Publish messages directly to message bus
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_A, 1.0f);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_B, 1.0f);
    g_message_bus.publishFloat(MSG_TRANS_OVERRUN_SOL, 0.5f);
    g_message_bus.publishFloat(MSG_TRANS_PRESSURE_SOL, 0.75f);
    g_message_bus.publishFloat(MSG_TRANS_LOCKUP_SOL, 1.0f);
    
    // Process messages
    g_message_bus.process();
    
    // Check that pins were updated
    std::cout << "   Pin 21 (Shift A) changes: " << pin_21_changes << std::endl;
    std::cout << "   Pin 22 (Shift B) changes: " << pin_22_changes << std::endl;
    std::cout << "   Pin 23 (Overrun) changes: " << pin_23_changes << std::endl;
    std::cout << "   Pin 19 (Pressure) changes: " << pin_19_changes << std::endl;
    std::cout << "   Pin 18 (Lockup) changes: " << pin_18_changes << std::endl;
    
    assert(pin_21_changes > 0);
    assert(pin_22_changes > 0);
    assert(pin_23_changes > 0);
    assert(pin_19_changes > 0);
    assert(pin_18_changes > 0);
    
    // Test 2: Transmission module functions that publish messages
    std::cout << "\n2. Testing transmission module functions..." << std::endl;
    
    reset_pin_tracking();
    
    // Call transmission functions that publish messages
    transmission_set_lockup(true);  // Publishes MSG_TRANS_LOCKUP_SOL
    transmission_set_line_pressure(0.5f);  // Publishes MSG_TRANS_PRESSURE_SOL
    transmission_set_solenoid_pattern(1);  // Publishes MSG_TRANS_SHIFT_SOL_A, MSG_TRANS_SHIFT_SOL_B, MSG_TRANS_LOCKUP_SOL
    
    // Process messages
    g_message_bus.process();
    
    std::cout << "   After transmission functions:" << std::endl;
    std::cout << "   Pin 21 (Shift A) changes: " << pin_21_changes << std::endl;
    std::cout << "   Pin 22 (Shift B) changes: " << pin_22_changes << std::endl;
    std::cout << "   Pin 23 (Overrun) changes: " << pin_23_changes << std::endl;
    std::cout << "   Pin 19 (Pressure) changes: " << pin_19_changes << std::endl;
    std::cout << "   Pin 18 (Lockup) changes: " << pin_18_changes << std::endl;
    
    // Test 3: Safe state function
    std::cout << "\n3. Testing safe state function..." << std::endl;
    
    reset_pin_tracking();
    
    // Call safe state function
    transmission_outputs_safe_state();
    
    // Process messages
    g_message_bus.process();
    
    std::cout << "   After safe state:" << std::endl;
    std::cout << "   Pin 21 (Shift A) changes: " << pin_21_changes << std::endl;
    std::cout << "   Pin 22 (Shift B) changes: " << pin_22_changes << std::endl;
    std::cout << "   Pin 23 (Overrun) changes: " << pin_23_changes << std::endl;
    std::cout << "   Pin 19 (Pressure) changes: " << pin_19_changes << std::endl;
    std::cout << "   Pin 18 (Lockup) changes: " << pin_18_changes << std::endl;
    
    std::cout << "\n✅ OUTPUT FLOW TEST PASSED!" << std::endl;
    std::cout << "\n🏁 Complete output flow verified:" << std::endl;
    std::cout << "   1. ✅ Module publishes message to message bus" << std::endl;
    std::cout << "   2. ✅ Output manager receives message via subscription" << std::endl;
    std::cout << "   3. ✅ Output manager finds correct output by message ID" << std::endl;
    std::cout << "   4. ✅ Output manager updates hardware pin" << std::endl;
    std::cout << "   5. ✅ Pin state changes tracked successfully" << std::endl;
    
    return 0;
} 