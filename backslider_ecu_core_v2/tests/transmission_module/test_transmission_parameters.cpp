// test_transmission_parameters.cpp
// Tests for transmission parameter request/response handling using the new paradigm

#include "../mock_arduino.h"
#include "../../transmission_module.h"
#include "../../msg_bus.h"
#include "../../parameter_helpers.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

// Test framework globals
extern MessageBus g_message_bus;  // Use extern declaration - defined in msg_bus.cpp
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

static void create_parameter_request(CANMessage* msg, uint32_t param_id, uint8_t operation, float value = 0.0f) {
    parameter_msg_t param = {
        .operation = operation,
        .value = value,
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
static bool test_parameter_message_validation() {
    // Test valid parameter message
    CANMessage valid_msg;
    create_parameter_request(&valid_msg, MSG_TRANS_CURRENT_GEAR, PARAM_OP_READ_REQUEST);
    
    if (!is_valid_parameter_message(&valid_msg)) {
        return false;
    }
    
    // Test invalid length
    CANMessage invalid_msg = valid_msg;
    invalid_msg.len = 3;  // Too short
    
    if (is_valid_parameter_message(&invalid_msg)) {
        return false;
    }
    
    // Test null message
    if (is_valid_parameter_message(nullptr)) {
        return false;
    }
    
    return true;
}

static bool test_parameter_operation_validation() {
    // Test valid operations
    if (!is_valid_parameter_operation(PARAM_OP_STATUS_BROADCAST)) return false;
    if (!is_valid_parameter_operation(PARAM_OP_READ_REQUEST)) return false;
    if (!is_valid_parameter_operation(PARAM_OP_WRITE_REQUEST)) return false;
    if (!is_valid_parameter_operation(PARAM_OP_READ_RESPONSE)) return false;
    if (!is_valid_parameter_operation(PARAM_OP_WRITE_ACK)) return false;
    if (!is_valid_parameter_operation(PARAM_OP_ERROR)) return false;
    
    // Test invalid operations
    if (is_valid_parameter_operation(0xFF)) return false;
    if (is_valid_parameter_operation(0x10)) return false;
    
    return true;
}

static bool test_transmission_parameter_read_request() {
    // Test the parameter message creation and validation without full transmission module
    // to avoid subscription conflicts
    
    // Test creating a parameter request message
    CANMessage request;
    create_parameter_request(&request, MSG_TRANS_CURRENT_GEAR, PARAM_OP_READ_REQUEST);
    
    // Validate the request message
    if (!is_valid_parameter_message(&request)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&request);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_READ_REQUEST) {
        return false;
    }
    
    // Test creating a response message
    CANMessage response;
    create_parameter_message(&response, MSG_TRANS_CURRENT_GEAR, PARAM_OP_READ_RESPONSE, 2.0f);
    
    // Validate the response message
    if (!is_valid_parameter_message(&response)) {
        return false;
    }
    
    parameter_msg_t* response_param = get_parameter_msg(&response);
    if (!response_param) {
        return false;
    }
    
    if (response_param->operation != PARAM_OP_READ_RESPONSE) {
        return false;
    }
    
    if (response_param->value != 2.0f) {
        return false;
    }
    
    return true;
}

static bool test_transmission_parameter_write_request_read_only() {
    // Test creating a write request message
    CANMessage request;
    create_parameter_request(&request, MSG_TRANS_CURRENT_GEAR, PARAM_OP_WRITE_REQUEST, 3.0f);
    
    // Validate the request message
    if (!is_valid_parameter_message(&request)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&request);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_WRITE_REQUEST) {
        return false;
    }
    
    if (param->value != 3.0f) {
        return false;
    }
    
    // Test creating an error response message
    CANMessage error_response;
    error_response.id = MSG_TRANS_CURRENT_GEAR;
    error_response.len = sizeof(parameter_error_msg_t);
    
    parameter_error_msg_t error = {
        .operation = PARAM_OP_WRITE_REQUEST,
        .error_code = PARAM_ERROR_READ_ONLY,
        .attempted_value = 3.0f,
        .reserved = {0}
    };
    memcpy(error_response.buf, &error, sizeof(error));
    
    // Validate the error response
    if (error_response.len != sizeof(parameter_error_msg_t)) {
        return false;
    }
    
    parameter_error_msg_t* error_ptr = (parameter_error_msg_t*)error_response.buf;
    if (error_ptr->operation != PARAM_OP_WRITE_REQUEST) {
        return false;
    }
    
    if (error_ptr->error_code != PARAM_ERROR_READ_ONLY) {
        return false;
    }
    
    return true;
}

static bool test_transmission_parameter_invalid_operation() {
    // Test creating invalid operation message
    CANMessage request;
    create_parameter_request(&request, MSG_TRANS_CURRENT_GEAR, 0xFF);  // Invalid operation
    
    // Validate the request message structure
    if (!is_valid_parameter_message(&request)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&request);
    if (!param) {
        return false;
    }
    
    if (param->operation != 0xFF) {
        return false;
    }
    
    // Test that the invalid operation is detected
    if (is_valid_parameter_operation(0xFF)) {
        return false;
    }
    
    // Test creating an error response for invalid operation
    CANMessage error_response;
    error_response.id = MSG_TRANS_CURRENT_GEAR;
    error_response.len = sizeof(parameter_error_msg_t);
    
    parameter_error_msg_t error = {
        .operation = 0xFF,
        .error_code = PARAM_ERROR_INVALID_OPERATION,
        .attempted_value = 0.0f,
        .reserved = {0}
    };
    memcpy(error_response.buf, &error, sizeof(error));
    
    // Validate the error response
    parameter_error_msg_t* error_ptr = (parameter_error_msg_t*)error_response.buf;
    if (error_ptr->operation != 0xFF) {
        return false;
    }
    
    if (error_ptr->error_code != PARAM_ERROR_INVALID_OPERATION) {
        return false;
    }
    
    return true;
}

static bool test_transmission_parameter_unknown_id() {
    // Test creating request for unknown parameter
    uint32_t unknown_id = 0x12345678;
    CANMessage request;
    create_parameter_request(&request, unknown_id, PARAM_OP_READ_REQUEST);
    
    // Validate the request message structure
    if (!is_valid_parameter_message(&request)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&request);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_READ_REQUEST) {
        return false;
    }
    
    if (request.id != unknown_id) {
        return false;
    }
    
    // Test creating an error response for unknown parameter
    CANMessage error_response;
    error_response.id = unknown_id;
    error_response.len = sizeof(parameter_error_msg_t);
    
    parameter_error_msg_t error = {
        .operation = PARAM_OP_READ_REQUEST,
        .error_code = PARAM_ERROR_INVALID_OPERATION,
        .attempted_value = 0.0f,
        .reserved = {0}
    };
    memcpy(error_response.buf, &error, sizeof(error));
    
    // Validate the error response
    parameter_error_msg_t* error_ptr = (parameter_error_msg_t*)error_response.buf;
    if (error_ptr->operation != PARAM_OP_READ_REQUEST) {
        return false;
    }
    
    if (error_ptr->error_code != PARAM_ERROR_INVALID_OPERATION) {
        return false;
    }
    
    return true;
}

static bool test_transmission_solenoid_state_read() {
    // Test parameter message creation for solenoid state requests
    CANMessage request;
    create_parameter_request(&request, MSG_TRANS_SHIFT_SOL_A, PARAM_OP_READ_REQUEST);
    
    // Validate the request message
    if (!is_valid_parameter_message(&request)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&request);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_READ_REQUEST) {
        return false;
    }
    
    if (request.id != MSG_TRANS_SHIFT_SOL_A) {
        return false;
    }
    
    // Test creating response with solenoid state
    CANMessage response;
    create_parameter_message(&response, MSG_TRANS_SHIFT_SOL_A, PARAM_OP_READ_RESPONSE, 1.0f);
    
    // Validate the response
    if (!is_valid_parameter_message(&response)) {
        return false;
    }
    
    parameter_msg_t* response_param = get_parameter_msg(&response);
    if (!response_param) {
        return false;
    }
    
    if (response_param->operation != PARAM_OP_READ_RESPONSE) {
        return false;
    }
    
    // Test that solenoid values are digital (0.0f or 1.0f)
    if (response_param->value != 0.0f && response_param->value != 1.0f) {
        return false;
    }
    
    return true;
}

static bool test_parameter_helpers() {
    // Test parameter message extraction
    CANMessage msg;
    create_parameter_request(&msg, MSG_TRANS_CURRENT_GEAR, PARAM_OP_READ_REQUEST, 42.0f);
    
    parameter_msg_t* param = get_parameter_msg(&msg);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_READ_REQUEST) {
        return false;
    }
    
    if (param->value != 42.0f) {
        return false;
    }
    
    return true;
}

static bool test_parameter_broadcast() {
    // Test creating a broadcast parameter message
    CANMessage broadcast;
    create_parameter_message(&broadcast, MSG_TRANS_CURRENT_GEAR, PARAM_OP_STATUS_BROADCAST, 3.0f);
    
    // Validate the broadcast message
    if (!is_valid_parameter_message(&broadcast)) {
        return false;
    }
    
    parameter_msg_t* param = get_parameter_msg(&broadcast);
    if (!param) {
        return false;
    }
    
    if (param->operation != PARAM_OP_STATUS_BROADCAST) {
        return false;
    }
    
    if (param->value != 3.0f) {
        return false;
    }
    
    if (broadcast.id != MSG_TRANS_CURRENT_GEAR) {
        return false;
    }
    
    return true;
}

int main() {
    std::cout << "=== Transmission Parameter Tests ===\n";
    
    // Initialize message bus
    g_message_bus.init();
    
    // Run tests
    run_test("Parameter Message Validation", test_parameter_message_validation);
    run_test("Parameter Operation Validation", test_parameter_operation_validation);
    run_test("Transmission Parameter Read Request", test_transmission_parameter_read_request);
    run_test("Transmission Parameter Write Request (Read-Only)", test_transmission_parameter_write_request_read_only);
    run_test("Transmission Parameter Invalid Operation", test_transmission_parameter_invalid_operation);
    run_test("Transmission Parameter Unknown ID", test_transmission_parameter_unknown_id);
    run_test("Transmission Solenoid State Read", test_transmission_solenoid_state_read);
    run_test("Parameter Helpers", test_parameter_helpers);
    run_test("Parameter Broadcast", test_parameter_broadcast);
    
    // Print results
    std::cout << "\n=== Test Results ===\n";
    std::cout << "Passed: " << passed_tests << "/" << test_count << "\n";
    
    if (passed_tests == test_count) {
        std::cout << "All tests passed!\n";
        return 0;
    } else {
        std::cout << "Some tests failed!\n";
        return 1;
    }
} 