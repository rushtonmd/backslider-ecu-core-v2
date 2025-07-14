// tests/message_bus/test_message_bus.cpp
// Test suite for the message bus system

#include <iostream>
#include <cassert>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

// Include message bus for testing
#include "../../msg_definitions.h"
#include "../../msg_bus.h"

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

// Test message reception
static CANMessage received_message;
static bool message_received = false;

void test_message_handler(const CANMessage* msg) {
    received_message = *msg;
    message_received = true;
}

// Test basic message bus creation and initialization
TEST(message_bus_creation) {
    MessageBus bus;
    bus.init();
    
    // Check initial state
    assert(bus.getMessagesProcessed() == 0);
    assert(bus.getQueueOverflows() == 0);
    assert(bus.getSubscriberCount() == 0);
    assert(!bus.isQueueFull());
}

// Test subscription mechanism
TEST(message_subscription) {
    MessageBus bus;
    bus.init();
    
    // Subscribe to a message
    bool result = bus.subscribe(MSG_ENGINE_RPM, test_message_handler);
    assert(result == true);
    assert(bus.getSubscriberCount() == 1);
    
    // Try to subscribe to another message
    result = bus.subscribe(MSG_COOLANT_TEMP, test_message_handler);
    assert(result == true);
    assert(bus.getSubscriberCount() == 2);
}

// Test message publishing and delivery
TEST(message_publish_and_delivery) {
    MessageBus bus;
    bus.init();
    message_received = false;
    
    // Subscribe to RPM messages
    bus.subscribe(MSG_ENGINE_RPM, test_message_handler);
    
    // Publish an RPM message
    float test_rpm = 3000.0f;
    bool result = bus.publishFloat(MSG_ENGINE_RPM, test_rpm);
    assert(result == true);
    
    // Process messages
    bus.process();
    
    // Check that message was delivered
    assert(message_received == true);
    assert(received_message.id == MSG_ENGINE_RPM);
    assert(received_message.len == sizeof(float));
    
    // Check that the data is correct
    float received_rpm = MSG_UNPACK_FLOAT(&received_message);
    assert(received_rpm == test_rpm);
    
    // Check statistics
    assert(bus.getMessagesProcessed() == 1);
}

// Test different data types
TEST(different_data_types) {
    MessageBus bus;
    bus.init();
    message_received = false;
    
    // Test uint8 data
    bus.subscribe(MSG_ENGINE_STATUS, test_message_handler);
    uint8_t status = ENGINE_STATUS_RUNNING;
    
    bus.publishUint8(MSG_ENGINE_STATUS, status);
    bus.process();
    
    assert(message_received == true);
    assert(MSG_UNPACK_UINT8(&received_message) == status);
    
    // Reset for next test
    message_received = false;
    
    // Test uint16 data
    bus.subscribe(MSG_IDLE_TARGET_RPM, test_message_handler);
    uint16_t target_rpm = 800;
    
    bus.publishUint16(MSG_IDLE_TARGET_RPM, target_rpm);
    bus.process();
    
    assert(message_received == true);
    assert(MSG_UNPACK_UINT16(&received_message) == target_rpm);
}

// Test queue functionality
TEST(queue_management) {
    MessageBus bus;
    bus.init();
    
    // Initial queue should be empty
    assert(bus.getQueueSize() == 0);
    assert(!bus.isQueueFull());
    
    // Add some messages without processing
    for (int i = 0; i < 10; i++) {
        bus.publishFloat(MSG_ENGINE_RPM, 1000.0f + i);
    }
    
    // Queue should have messages now
    assert(bus.getQueueSize() == 10);
    assert(!bus.isQueueFull());
    
    // Process all messages
    bus.process();
    
    // Queue should be empty again
    assert(bus.getQueueSize() == 0);
    assert(bus.getMessagesProcessed() == 10);
}

// Test message filtering (subscribers only get their messages)
TEST(message_filtering) {
    MessageBus bus;
    bus.init();
    
    static int rpm_messages = 0;
    static int temp_messages = 0;
    
    // Handler that counts RPM messages
    auto rpm_handler = [](const CANMessage* msg) {
        rpm_messages++;
    };
    
    // Handler that counts temperature messages
    auto temp_handler = [](const CANMessage* msg) {
        temp_messages++;
    };
    
    // Subscribe to different messages
    bus.subscribe(MSG_ENGINE_RPM, rpm_handler);
    bus.subscribe(MSG_COOLANT_TEMP, temp_handler);
    
    // Publish various messages
    bus.publishFloat(MSG_ENGINE_RPM, 3000.0f);
    bus.publishFloat(MSG_ENGINE_RPM, 3500.0f);
    bus.publishFloat(MSG_COOLANT_TEMP, 85.0f);
    bus.publishFloat(MSG_BATTERY_VOLTAGE, 12.6f);  // No subscriber
    
    // Process all messages
    bus.process();
    
    // Check that handlers only received their messages
    assert(rpm_messages == 2);
    assert(temp_messages == 1);
    assert(bus.getMessagesProcessed() == 4);  // All messages processed
}

// Test statistics and diagnostics
TEST(statistics_and_diagnostics) {
    MessageBus bus;
    bus.init();
    
    // Initial statistics
    assert(bus.getMessagesProcessed() == 0);
    assert(bus.getQueueOverflows() == 0);
    
    // Process some messages
    bus.publishFloat(MSG_ENGINE_RPM, 3000.0f);
    bus.publishFloat(MSG_COOLANT_TEMP, 85.0f);
    bus.process();
    
    assert(bus.getMessagesProcessed() == 2);
    
    // Reset statistics
    bus.resetStatistics();
    assert(bus.getMessagesProcessed() == 0);
}

// Main test runner
int main() {
    std::cout << "=== Message Bus Tests ===" << std::endl;
    
    // Run all tests
    run_test_message_bus_creation();
    run_test_message_subscription();
    run_test_message_publish_and_delivery();
    run_test_different_data_types();
    run_test_queue_management();
    run_test_message_filtering();
    run_test_statistics_and_diagnostics();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Message Bus Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL MESSAGE BUS TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME MESSAGE BUS TESTS FAILED!" << std::endl;
        return 1;
    }
}