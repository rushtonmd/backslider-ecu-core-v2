#include <iostream>
#include <cassert>
#include <cstring>
#include "../mock_arduino.h"
#include "../../external_message_broadcasting.h"
#include "../../msg_bus.h"
#include "../../external_canbus.h"
#include "../../external_serial.h"

// =============================================================================
// EXTERNAL MESSAGE BROADCASTING TESTS
// =============================================================================

// Mock external interfaces for testing
class MockExternalCanBus {
public:
    bool initialized = true;
    uint32_t messages_sent = 0;
    
    bool is_initialized() const { return initialized; }
    
    bool send_custom_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
        messages_sent++;
        return true;
    }
};

class MockExternalSerial {
public:
    uint32_t messages_sent = 0;
    
    bool send_message(const CANMessage& msg) {
        messages_sent++;
        return true;
    }
};

// Global instances
static ExternalCanBus real_canbus;
static ExternalSerial real_serial;

// Test message IDs
#define TEST_MSG_1 0x1000
#define TEST_MSG_2 0x2000
#define TEST_MSG_3 0x3000

// Test state
static uint32_t messages_received = 0;
static uint32_t last_received_msg_id = 0;
static float last_received_value = 0.0f;

// Message capture function
static void capture_message(const CANMessage* msg) {
    messages_received++;
    last_received_msg_id = msg->id;
    if (msg->len >= 4) {
        memcpy(&last_received_value, msg->buf, 4);
    }
}

// Test functions
static bool test_initialization() {
    std::cout << "  Running test: initialization... ";
    
    // Initialize the broadcasting module
    ExternalMessageBroadcasting::init();
    
    // Check initial state
    assert(ExternalMessageBroadcasting::get_messages_broadcast() == 0);
    assert(ExternalMessageBroadcasting::get_can_bus_broadcasts() == 0);
    assert(ExternalMessageBroadcasting::get_serial_broadcasts() == 0);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_message_registration() {
    std::cout << "  Running test: message_registration... ";
    
    // Register a test message
    bool result = ExternalMessageBroadcasting::register_broadcast_message(
        TEST_MSG_1, "Test Message 1", 0);
    
    assert(result == true);
    assert(ExternalMessageBroadcasting::is_message_registered(TEST_MSG_1) == true);
    
    // Try to register the same message again
    result = ExternalMessageBroadcasting::register_broadcast_message(
        TEST_MSG_1, "Test Message 1 Duplicate", 0);
    
    assert(result == false); // Should fail - already registered
    
    // Register another message
    result = ExternalMessageBroadcasting::register_broadcast_message(
        TEST_MSG_2, "Test Message 2", 0);
    
    assert(result == true);
    assert(ExternalMessageBroadcasting::is_message_registered(TEST_MSG_2) == true);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_message_unregistration() {
    std::cout << "  Running test: message_unregistration... ";
    
    // Unregister a message
    bool result = ExternalMessageBroadcasting::unregister_broadcast_message(TEST_MSG_1);
    
    assert(result == true);
    assert(ExternalMessageBroadcasting::is_message_registered(TEST_MSG_1) == false);
    assert(ExternalMessageBroadcasting::is_message_registered(TEST_MSG_2) == true);
    
    // Try to unregister a non-existent message
    result = ExternalMessageBroadcasting::unregister_broadcast_message(0x9999);
    
    assert(result == false);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_enable_disable_broadcasts() {
    std::cout << "  Running test: enable_disable_broadcasts... ";
    
    // Re-register the test message
    ExternalMessageBroadcasting::register_broadcast_message(TEST_MSG_1, "Test Message 1", 0);
    
    // Disable broadcasting for a specific message
    bool result = ExternalMessageBroadcasting::enable_broadcast_message(TEST_MSG_1, false);
    
    assert(result == true);
    
    // Disable all broadcasts
    ExternalMessageBroadcasting::enable_all_broadcasts(false);
    
    // Re-enable all broadcasts
    ExternalMessageBroadcasting::enable_all_broadcasts(true);
    
    // Re-enable specific message
    result = ExternalMessageBroadcasting::enable_broadcast_message(TEST_MSG_1, true);
    
    assert(result == true);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_message_broadcasting() {
    std::cout << "  Running test: message_broadcasting... ";
    
    // Initialize the real external interfaces
    external_canbus_config_t can_config = {
        .enabled = true,
        .baudrate = 500000,
        .enable_obdii = false,
        .enable_custom_messages = true,
        .can_bus_number = 1,
        .cache_default_max_age_ms = 1000
    };
    real_canbus.init(can_config);
    real_serial.init({true, 115200, true, true, true, true, true, true, true, true});
    
    // Set up external interfaces
    ExternalMessageBroadcasting::set_external_interfaces(&real_canbus, &real_serial);
    
    // Get initial statistics
    uint32_t initial_can_sent = real_canbus.get_statistics().messages_sent;
    
    // Publish a message that should be broadcast
    float test_value = 123.45f;
    g_message_bus.publishFloat(TEST_MSG_1, test_value);
    
    // Process the message bus to trigger broadcasting
    g_message_bus.process();
    

    
    // Check that the message was broadcast
    uint32_t final_can_sent = real_canbus.get_statistics().messages_sent;
    assert(final_can_sent > initial_can_sent);
    assert(ExternalMessageBroadcasting::get_can_bus_broadcasts() == 1);
    assert(ExternalMessageBroadcasting::get_messages_broadcast() == 1);
    
    // Publish a message that is NOT registered for broadcasting
    g_message_bus.publishFloat(TEST_MSG_3, 999.99f);
    g_message_bus.process();
    
    // Check that this message was NOT broadcast
    assert(ExternalMessageBroadcasting::get_messages_broadcast() == 1); // Should still be 1
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_convenience_functions() {
    std::cout << "  Running test: convenience_functions... ";
    
    // Test convenience functions
    register_engine_broadcast_messages();
    register_transmission_broadcast_messages();
    register_vehicle_state_broadcast_messages();
    
    // Check that common messages were registered
    assert(ExternalMessageBroadcasting::is_message_registered(BROADCAST_MSG_ENGINE_RPM));
    assert(ExternalMessageBroadcasting::is_message_registered(BROADCAST_MSG_COOLANT_TEMP));
    assert(ExternalMessageBroadcasting::is_message_registered(BROADCAST_MSG_TRANS_CURRENT_GEAR));
    assert(ExternalMessageBroadcasting::is_message_registered(BROADCAST_MSG_VEHICLE_SPEED));
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_statistics() {
    std::cout << "  Running test: statistics... ";
    
    // Get current statistics
    uint32_t total_broadcasts = ExternalMessageBroadcasting::get_messages_broadcast();
    uint32_t can_broadcasts = ExternalMessageBroadcasting::get_can_bus_broadcasts();
    uint32_t serial_broadcasts = ExternalMessageBroadcasting::get_serial_broadcasts();
    
    // Reset statistics
    ExternalMessageBroadcasting::reset_statistics();
    
    // Check that statistics were reset
    assert(ExternalMessageBroadcasting::get_messages_broadcast() == 0);
    assert(ExternalMessageBroadcasting::get_can_bus_broadcasts() == 0);
    assert(ExternalMessageBroadcasting::get_serial_broadcasts() == 0);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_configuration_access() {
    std::cout << "  Running test: configuration_access... ";
    
    uint8_t config_count = 0;
    const broadcast_message_config_t* configs = ExternalMessageBroadcasting::get_broadcast_configs(&config_count);
    
    // Should have some configurations
    assert(config_count > 0);
    assert(configs != nullptr);
    
    // Check that we can access the configurations
    bool found_engine_rpm = false;
    for (uint8_t i = 0; i < config_count; i++) {
        if (configs[i].msg_id == BROADCAST_MSG_ENGINE_RPM) {
            found_engine_rpm = true;
            assert(configs[i].description != nullptr);
            assert(strlen(configs[i].description) > 0);
            break;
        }
    }
    
    assert(found_engine_rpm);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

static bool test_frequency_based_broadcasting() {
    std::cout << "  Running test: frequency_based_broadcasting... ";
    
    // Register a message with 10Hz frequency
    ExternalMessageBroadcasting::register_broadcast_message(
        TEST_MSG_3, "Test Message 3", 10);
    
    // Set up external interfaces
    ExternalMessageBroadcasting::set_external_interfaces(&real_canbus, &real_serial);
    
    // Get initial statistics
    uint32_t initial_can_sent = real_canbus.get_statistics().messages_sent;
    
    // Publish a value once to cache it
    g_message_bus.publishFloat(TEST_MSG_3, 123.45f);
    g_message_bus.process();
    
    // Now call update multiple times to trigger frequency-based broadcasting
    for (int i = 0; i < 10; i++) {
        ExternalMessageBroadcasting::update();
    }
    
    // Should get broadcasts from the update loop
    uint32_t final_can_sent = real_canbus.get_statistics().messages_sent;
    assert(final_can_sent > initial_can_sent);
    
    std::cout << "PASSED" << std::endl;
    return true;
}

// Main test runner
int main() {
    std::cout << "=== External Message Broadcasting Tests ===" << std::endl;
    
    // Initialize message bus
    g_message_bus.init();
    
    // Run tests
    bool all_passed = true;
    
    all_passed &= test_initialization();
    all_passed &= test_message_registration();
    all_passed &= test_message_unregistration();
    all_passed &= test_enable_disable_broadcasts();
    all_passed &= test_message_broadcasting();
    all_passed &= test_convenience_functions();
    all_passed &= test_statistics();
    all_passed &= test_configuration_access();
    all_passed &= test_frequency_based_broadcasting();
    
    std::cout << std::endl;
    std::cout << "=== Test Results ===" << std::endl;
    std::cout << "All tests " << (all_passed ? "PASSED" : "FAILED") << std::endl;
    
    if (all_passed) {
        std::cout << "✅ External Message Broadcasting module is working correctly!" << std::endl;
    } else {
        std::cout << "❌ Some tests failed!" << std::endl;
    }
    
    return all_passed ? 0 : 1;
} 