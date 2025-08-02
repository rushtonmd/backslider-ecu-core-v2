// parameter_registry.cpp
// Central parameter registry implementation for request-response channel routing

#include "parameter_registry.h"
#include "parameter_helpers.h"

// =============================================================================
// STATIC MEMBER INITIALIZATION
// =============================================================================

ParameterHandler ParameterRegistry::registered_parameters[MAX_PARAMETERS];
uint8_t ParameterRegistry::parameter_count = 0;
uint32_t ParameterRegistry::requests_processed = 0;
uint32_t ParameterRegistry::read_requests = 0;
uint32_t ParameterRegistry::write_requests = 0;
uint32_t ParameterRegistry::errors_generated = 0;

// =============================================================================
// REGISTRATION METHODS
// =============================================================================

bool ParameterRegistry::register_parameter(uint32_t param_id, 
                                         parameter_read_handler_t read_handler,
                                         parameter_write_handler_t write_handler, 
                                         const char* description) {
    if (parameter_count >= MAX_PARAMETERS) {
        return false; // Registry full
    }
    
    // Check if parameter already registered
    for (uint8_t i = 0; i < parameter_count; i++) {
        if (registered_parameters[i].param_id == param_id) {
            // Update existing entry
            registered_parameters[i].read_handler = read_handler;
            registered_parameters[i].write_handler = write_handler;
            registered_parameters[i].description = description;
            return true;
        }
    }
    
    // Add new entry
    registered_parameters[parameter_count].param_id = param_id;
    registered_parameters[parameter_count].read_handler = read_handler;
    registered_parameters[parameter_count].write_handler = write_handler;
    registered_parameters[parameter_count].description = description;
    parameter_count++;
    
    return true;
}

// =============================================================================
// LOOKUP METHODS
// =============================================================================

ParameterHandler* ParameterRegistry::find_handler(uint32_t param_id) {
    for (uint8_t i = 0; i < parameter_count; i++) {
        if (registered_parameters[i].param_id == param_id) {
            return &registered_parameters[i];
        }
    }
    return nullptr;
}

// =============================================================================
// REQUEST HANDLING
// =============================================================================

void ParameterRegistry::handle_parameter_request(const CANMessage* msg) {
    if (msg == nullptr) return;
    
    // Only process messages that are actually parameter messages
    if (!is_valid_parameter_message(msg)) {
        // Not a parameter message, silently ignore it
        return;
    }
    
    #ifdef ARDUINO
    Serial.print("ParameterRegistry: handle_parameter_request called for CAN ID 0x");
    Serial.println(msg->id, HEX);
    
    // Special debug for vehicle speed requests
    if (msg->id == 0x10300002) {  // MSG_VEHICLE_SPEED
        Serial.println("DEBUG: Vehicle speed parameter request received!");
    }
    #endif
    
    parameter_msg_t* param = get_parameter_msg(msg);
    
    #ifdef ARDUINO
    Serial.print("ParameterRegistry: Parameter message - operation=");
    Serial.print(param->operation);
    Serial.print(", value=");
    Serial.print(param->value);
    Serial.print(", channel=");
    Serial.print(param->source_channel);
    Serial.print(", request_id=");
    Serial.println(param->request_id);
    #endif
    
    // Only process read/write requests, not responses
    if (param->operation != PARAM_OP_READ_REQUEST && param->operation != PARAM_OP_WRITE_REQUEST) {
        #ifdef ARDUINO
        Serial.println("ParameterRegistry: Not a read/write request, ignoring");
        #endif
        return;
    }
    
    requests_processed++;
    
    // Find handler for this parameter
    ParameterHandler* handler = find_handler(msg->id);
    
    #ifdef ARDUINO
    if (handler) {
        Serial.print("ParameterRegistry: Found handler for parameter 0x");
        Serial.print(msg->id, HEX);
        Serial.print(" - ");
        Serial.println(handler->description);
    } else {
        Serial.print("ParameterRegistry: No handler found for parameter 0x");
        Serial.println(msg->id, HEX);
    }
    #endif
    
    if (!handler) {
        send_parameter_error_routed(msg->id, param->operation, 
                                  PARAM_ERROR_INVALID_OPERATION, param->value,
                                  param->source_channel, param->request_id);
        errors_generated++;
        return;
    }
    
    switch (param->operation) {
        case PARAM_OP_READ_REQUEST:
            read_requests++;
            #ifdef ARDUINO
            Serial.println("ParameterRegistry: Processing READ_REQUEST");
            #endif
            if (handler->read_handler) {
                float value = handler->read_handler();
                #ifdef ARDUINO
                Serial.print("ParameterRegistry: Read handler returned value: ");
                Serial.println(value);
                #endif
                send_parameter_response_routed(msg->id, PARAM_OP_READ_RESPONSE, 
                                             value, param->source_channel, 
                                             param->request_id);
                #ifdef ARDUINO
                Serial.println("ParameterRegistry: READ_RESPONSE sent");
                #endif
            } else {
                #ifdef ARDUINO
                Serial.println("ParameterRegistry: No read handler available");
                #endif
                send_parameter_error_routed(msg->id, param->operation, 
                                          PARAM_ERROR_READ_ONLY, param->value,
                                          param->source_channel, param->request_id);
                errors_generated++;
            }
            break;
            
        case PARAM_OP_WRITE_REQUEST:
            write_requests++;
            if (handler->write_handler) {
                bool success = handler->write_handler(param->value);
                if (success) {
                    send_parameter_response_routed(msg->id, PARAM_OP_WRITE_ACK, 
                                                 param->value, param->source_channel, 
                                                 param->request_id);
                } else {
                    send_parameter_error_routed(msg->id, param->operation, 
                                              PARAM_ERROR_WRITE_FAILED, param->value,
                                              param->source_channel, param->request_id);
                    errors_generated++;
                }
            } else {
                send_parameter_error_routed(msg->id, param->operation, 
                                          PARAM_ERROR_READ_ONLY, param->value,
                                          param->source_channel, param->request_id);
                errors_generated++;
            }
            break;
            
        default:
            // Unknown operation
            send_parameter_error_routed(msg->id, param->operation, 
                                      PARAM_ERROR_INVALID_OPERATION, param->value,
                                      param->source_channel, param->request_id);
            errors_generated++;
            break;
    }
}

// =============================================================================
// STATISTICS
// =============================================================================

void ParameterRegistry::reset_statistics() {
    requests_processed = 0;
    read_requests = 0;
    write_requests = 0;
    errors_generated = 0;
} 