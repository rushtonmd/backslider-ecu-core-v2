// test_parameter_registry.cpp
// Tests for parameter registry system

#include "../mock_arduino.h"
#include "../../parameter_registry.h"
#include "../../msg_bus.h"
#include "../../parameter_helpers.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

// Test framework globals
extern MessageBus g_message_bus;
static std::vector<CANMessage> captured_messages;
static int test_count = 0;
static int passed_tests = 0;

// Test message capture
static void capture_message(const CANMessage* msg) {
    captured_messages.push_back(*msg);
}

// Test helper functions
static void clear_captured_messages() {
    captured_messages.clear();
}

static CANMessage* find_message_by_id(uint32_t msg_id) {
    for (auto& msg : captured_messages) {
        if (msg.id == msg_id) {
            return &msg;
        }
    }
    return nullptr;
}

static parameter_msg_t* get_parameter_from_message(const CANMessage* msg) {
    if (msg && msg->len == sizeof(parameter_msg_t)) {
        return (parameter_msg_t*)msg->buf;
    }
    return nullptr;
}

static void create_parameter_request(CANMessage* msg, uint32_t param_id, uint8_t operation, 
                                   float value = 0.0f, uint8_t source_channel = 0, uint8_t request_id = 0) {
    parameter_msg_t param = {
        .operation = operation,
        .value = value,
        .source_channel = source_channel,
        .request_id = request_id,
        .reserved = {0}
    };
    
    msg->id = param_id;
    msg->len = sizeof(parameter_msg_t);
    memcpy(msg->buf, &param, sizeof(parameter_msg_t));
}

static void run_test(const char* test_name, bool (*test_func)()) {
    test_count++;
    std::cout << "Running " << test_name << "... ";
    
    // Clear state
    clear_captured_messages();
    
    if (test_func()) {
        std::cout << "PASSED\n";
        passed_tests++;
    } else {
        std::cout << "FAILED\n";
    }
}

// Test functions
static bool test_parameter_registration() {
    // Test parameter registration
    bool success = ParameterRegistry::register_parameter(0x1000, 
        []() -> float { return 42.0f; }, 
        nullptr, "Test Parameter");
    
    if (!success) {
        return false;
    }
    
    // Test finding the registered parameter
    ParameterHandler* handler = ParameterRegistry::find_handler(0x1000);
    if (!handler) {
        return false;
    }
    
    if (handler->param_id != 0x1000) {
        return false;
    }
    
    if (strcmp(handler->description, "Test Parameter") != 0) {
        return false;
    }
    
    // Test that the handler returns the expected value
    float value = handler->read_handler();
    if (value != 42.0f) {
        return false;
    }
    
    return true;
}

static bool test_parameter_request_handling() {
    // Register a test parameter
    ParameterRegistry::register_parameter(0x2000, 
        []() -> float { return 123.45f; }, 
        nullptr, "Test Parameter 2");
    
    // Create a read request
    CANMessage request;
    create_parameter_request(&request, 0x2000, PARAM_OP_READ_REQUEST, 0.0f, CHANNEL_SERIAL_USB, 1);
    
    // Clear any previous messages
    clear_captured_messages();
    
    // Call parameter registry directly
    ParameterRegistry::handle_parameter_request(&request);
    
    // Process the message bus to deliver responses
    g_message_bus.process();
    
    // Check that a response was generated
    if (captured_messages.size() != 1) {
        return false;
    }
    
    CANMessage* response = &captured_messages[0];
    if (response->id != 0x2000) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_from_message(response);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_READ_RESPONSE) {
        return false;
    }
    
    if (param->value != 123.45f) {
        return false;
    }
    
    if (param->source_channel != CHANNEL_SERIAL_USB) {
        return false;
    }
    
    if (param->request_id != 1) {
        return false;
    }
    
    return true;
}

static bool test_parameter_error_handling() {
    // Try to read a non-existent parameter
    CANMessage request;
    create_parameter_request(&request, 0x9999, PARAM_OP_READ_REQUEST, 0.0f, CHANNEL_CAN_BUS, 2);
    
    // Clear any previous messages
    clear_captured_messages();
    
    // Call parameter registry directly
    ParameterRegistry::handle_parameter_request(&request);
    
    // Process the message bus to deliver responses
    g_message_bus.process();
    
    // Check that an error was generated
    if (captured_messages.size() != 1) {
        return false;
    }
    
    CANMessage* response = &captured_messages[0];
    if (response->id != 0x9999) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_from_message(response);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_ERROR) {
        return false;
    }
    
    if (param->source_channel != CHANNEL_CAN_BUS) {
        return false;
    }
    
    if (param->request_id != 2) {
        return false;
    }
    
    return true;
}

// Global variables for test state
static bool test_write_called = false;
static float test_written_value = 0.0f;

// Test write handler function
static bool test_write_handler(float value) {
    test_write_called = true;
    test_written_value = value;
    return true;
}

static bool test_write_parameter_handling() {
    // Reset test state
    test_write_called = false;
    test_written_value = 0.0f;
    
    // Register a writable parameter
    ParameterRegistry::register_parameter(0x3000, 
        []() -> float { return 0.0f; }, 
        test_write_handler, "Writable Parameter");
    
    // Create a write request
    CANMessage request;
    create_parameter_request(&request, 0x3000, PARAM_OP_WRITE_REQUEST, 99.99f, CHANNEL_SERIAL_1, 3);
    
    // Call parameter registry directly
    ParameterRegistry::handle_parameter_request(&request);
    
    // Process the message bus to deliver responses
    g_message_bus.process();
    
    // Check that write was called
    if (!test_write_called) {
        return false;
    }
    
    if (test_written_value != 99.99f) {
        return false;
    }
    
    // Check that an acknowledgment was generated
    if (captured_messages.size() != 1) {
        return false;
    }
    
    CANMessage* response = &captured_messages[0];
    if (response->id != 0x3000) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_from_message(response);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_WRITE_ACK) {
        return false;
    }
    
    if (param->value != 99.99f) {
        return false;
    }
    
    if (param->source_channel != CHANNEL_SERIAL_1) {
        return false;
    }
    
    if (param->request_id != 3) {
        return false;
    }
    
    return true;
}

static bool test_readonly_parameter_write_error() {
    // Register a read-only parameter
    ParameterRegistry::register_parameter(0x4000, 
        []() -> float { return 0.0f; }, 
        nullptr, "Read-Only Parameter");
    
    // Create a write request
    CANMessage request;
    create_parameter_request(&request, 0x4000, PARAM_OP_WRITE_REQUEST, 50.0f, CHANNEL_SERIAL_2, 4);
    
    // Call parameter registry directly
    ParameterRegistry::handle_parameter_request(&request);
    
    // Process the message bus to deliver responses
    g_message_bus.process();
    
    // Check that an error was generated
    if (captured_messages.size() != 1) {
        return false;
    }
    
    CANMessage* response = &captured_messages[0];
    if (response->id != 0x4000) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_from_message(response);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_ERROR) {
        return false;
    }
    
    if (param->source_channel != CHANNEL_SERIAL_2) {
        return false;
    }
    
    if (param->request_id != 4) {
        return false;
    }
    
    return true;
}

// Main test function
int main() {
    std::cout << "=== Parameter Registry Tests ===\n";
    
    // Set up message bus
    g_message_bus.init();
    
    // Subscribe to capture responses
    g_message_bus.subscribe(0x1000, capture_message);
    g_message_bus.subscribe(0x2000, capture_message);
    g_message_bus.subscribe(0x3000, capture_message);
    g_message_bus.subscribe(0x4000, capture_message);
    g_message_bus.subscribe(0x9999, capture_message);
    
    // Run tests
    run_test("Parameter Registration", test_parameter_registration);
    run_test("Parameter Request Handling", test_parameter_request_handling);
    run_test("Parameter Error Handling", test_parameter_error_handling);
    run_test("Write Parameter Handling", test_write_parameter_handling);
    run_test("Read-Only Parameter Write Error", test_readonly_parameter_write_error);
    
    // Print results
    std::cout << "\n=== Test Results ===\n";
    std::cout << "Passed: " << passed_tests << "/" << test_count << "\n";
    
    return (passed_tests == test_count) ? 0 : 1;
} 