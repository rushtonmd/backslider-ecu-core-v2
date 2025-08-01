// parameter_registry.h
// Central parameter registry for request-response channel routing
// Provides unified parameter management and routing

#ifndef PARAMETER_REGISTRY_H
#define PARAMETER_REGISTRY_H

#include "msg_definitions.h"
#include "msg_bus.h"
#include "external_serial.h"
#include "external_canbus.h"

// External reference to global message bus
extern MessageBus g_message_bus;

// =============================================================================
// PARAMETER HANDLER TYPES
// =============================================================================

// Function pointer types for parameter handlers
typedef float (*parameter_read_handler_t)(void);
typedef bool (*parameter_write_handler_t)(float value);

// Parameter handler structure
struct ParameterHandler {
    uint32_t param_id;
    parameter_read_handler_t read_handler;
    parameter_write_handler_t write_handler;
    const char* description;
};

// =============================================================================
// PARAMETER REGISTRY CLASS
// =============================================================================

class ParameterRegistry {
public:
    // Configuration constants
    static const uint8_t MAX_PARAMETERS = 64;
    
    // Registration methods
    static bool register_parameter(uint32_t param_id, 
                                 parameter_read_handler_t read_handler,
                                 parameter_write_handler_t write_handler, 
                                 const char* description);
    
    // Lookup methods
    static ParameterHandler* find_handler(uint32_t param_id);
    
    // Request handling
    static void handle_parameter_request(const CANMessage* msg);
    
    // Statistics
    static uint8_t get_registered_count() { return parameter_count; }
    static void reset_statistics();
    
private:
    // Registry storage
    static ParameterHandler registered_parameters[MAX_PARAMETERS];
    static uint8_t parameter_count;
    
    // Statistics
    static uint32_t requests_processed;
    static uint32_t read_requests;
    static uint32_t write_requests;
    static uint32_t errors_generated;
    
    // Helper methods
    static void send_parameter_response(uint32_t param_id, uint8_t operation, 
                                      float value, uint8_t source_channel, 
                                      uint8_t request_id);
    static void send_parameter_error(uint32_t param_id, uint8_t failed_operation,
                                   uint8_t error_code, float attempted_value,
                                   uint8_t source_channel, uint8_t request_id);
};

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Helper function to send parameter response with routing info
inline void send_parameter_response_routed(uint32_t param_id, uint8_t operation, 
                                          float value, uint8_t source_channel, 
                                          uint8_t request_id) {
    parameter_msg_t response = {
        .operation = operation,
        .value = value,
        .source_channel = source_channel,
        .request_id = request_id,
        .reserved = {0}
    };
    
    // Publish to message bus
    g_message_bus.publish(param_id, &response, sizeof(response));
    
    // Also send directly to external interfaces for immediate response
    extern ExternalSerial g_external_serial;
    extern ExternalCanBus g_external_canbus;
    
    // Create CAN message for external interfaces
    CANMessage can_response;
    can_response.id = param_id;
    can_response.len = sizeof(parameter_msg_t);
    memcpy(can_response.buf, &response, sizeof(parameter_msg_t));
    
    // Send to external serial if initialized
    if (g_external_serial.is_initialized()) {
        g_external_serial.on_selective_message(&can_response);
    }
    
    // Send to external CAN bus if initialized
    if (g_external_canbus.is_initialized()) {
        g_external_canbus.on_message_bus_message(&can_response);
    }
}

// Helper function to send parameter error with routing info
inline void send_parameter_error_routed(uint32_t param_id, uint8_t failed_operation,
                                       uint8_t error_code, float attempted_value,
                                       uint8_t source_channel, uint8_t request_id) {
    parameter_msg_t error = {
        .operation = PARAM_OP_ERROR,
        .value = attempted_value,
        .source_channel = source_channel,
        .request_id = request_id,
        .reserved = {0}
    };
    g_message_bus.publish(param_id, &error, sizeof(error));
}

#endif // PARAMETER_REGISTRY_H 