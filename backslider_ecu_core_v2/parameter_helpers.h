// parameter_helpers.h
// Helper functions for parameter message handling using the new paradigm

#ifndef PARAMETER_HELPERS_H
#define PARAMETER_HELPERS_H

#include "msg_definitions.h"
#include "msg_bus.h"

// External reference to global message bus
extern MessageBus g_message_bus;

// =============================================================================
// PARAMETER MESSAGE HELPER FUNCTIONS
// =============================================================================

// Helper function to send parameter response (legacy - no routing)
inline void send_parameter_response(uint32_t param_id, uint8_t operation, float value) {
    parameter_msg_t response = {
        .operation = operation,
        .value = value,
        .source_channel = 0,  // No routing
        .request_id = 0,      // No routing
        .reserved = {0}
    };
    g_message_bus.publish(param_id, &response, sizeof(response));
}

// Helper function to send parameter error (legacy - no routing)
inline void send_parameter_error(uint32_t param_id, uint8_t failed_operation, 
                                uint8_t error_code, float attempted_value) {
    parameter_error_msg_t error = {
        .operation = failed_operation,
        .error_code = error_code,
        .attempted_value = attempted_value,
        .reserved = {0}
    };
    g_message_bus.publish(param_id, &error, sizeof(error));
}

// Helper function to extract parameter message from CAN message
inline parameter_msg_t* get_parameter_msg(const CANMessage* msg) {
    return (parameter_msg_t*)msg->buf;
}

// Helper function to validate parameter operation
inline bool is_valid_parameter_operation(uint8_t operation) {
    return (operation >= PARAM_OP_STATUS_BROADCAST && operation <= PARAM_OP_ERROR);
}

// Helper function to validate parameter message length
inline bool is_valid_parameter_message(const CANMessage* msg) {
    return (msg != nullptr && msg->len == sizeof(parameter_msg_t));
}

// Helper function to create parameter message for publishing (legacy - no routing)
inline void create_parameter_message(CANMessage* msg, uint32_t param_id, 
                                    uint8_t operation, float value) {
    if (msg == nullptr) return;
    
    parameter_msg_t param = {
        .operation = operation,
        .value = value,
        .source_channel = 0,  // No routing
        .request_id = 0,      // No routing
        .reserved = {0}
    };
    
    msg->id = param_id;
    msg->len = sizeof(parameter_msg_t);
    memcpy(msg->buf, &param, sizeof(parameter_msg_t));
}

// Helper function to broadcast parameter status (for periodic updates)
inline void broadcast_parameter_status(uint32_t param_id, float value) {
    send_parameter_response(param_id, PARAM_OP_STATUS_BROADCAST, value);
}

// =============================================================================
// ROUTING-AWARE HELPER FUNCTIONS
// =============================================================================

// Helper function to create parameter message with routing info
inline void create_parameter_message_routed(CANMessage* msg, uint32_t param_id, 
                                           uint8_t operation, float value,
                                           uint8_t source_channel, uint8_t request_id) {
    if (msg == nullptr) return;
    
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

// Helper function to add routing metadata to existing parameter message
inline void add_routing_metadata(CANMessage* msg, uint8_t source_channel, uint8_t request_id) {
    if (msg == nullptr || msg->len != sizeof(parameter_msg_t)) return;
    
    parameter_msg_t* param = (parameter_msg_t*)msg->buf;
    param->source_channel = source_channel;
    param->request_id = request_id;
}

// Helper function to strip routing metadata from parameter message
inline void strip_routing_metadata(CANMessage* msg) {
    if (msg == nullptr || msg->len != sizeof(parameter_msg_t)) return;
    
    parameter_msg_t* param = (parameter_msg_t*)msg->buf;
    param->source_channel = 0;
    param->request_id = 0;
}

#endif // PARAMETER_HELPERS_H 