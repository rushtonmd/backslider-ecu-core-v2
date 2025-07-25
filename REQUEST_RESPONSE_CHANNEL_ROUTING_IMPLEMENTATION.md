# Request-Response Channel Routing Implementation

## Overview

This document describes the implementation of the request-response channel routing system for the Backslider ECU. This system solves the problem of external tools receiving responses to requests they didn't make, by tracking which external interface sent which request and routing responses back to the correct channel.

## Problem Solved

### Before Implementation
- External tools send READ/WRITE requests via serial or CAN
- Responses get broadcast to all external interfaces (wrong channel)
- No tracking of which external interface sent which request
- Global broadcast floods external tools with irrelevant messages

### After Implementation
- ✅ Request-response channel tracking
- ✅ No message flooding
- ✅ Central parameter management
- ✅ Module self-registration
- ✅ Automatic timeout handling
- ✅ Works within CAN message constraints

## Architecture

### 1. Enhanced Parameter Message Structure

The `parameter_msg_t` structure was updated to include routing information:

```cpp
typedef struct {
    uint8_t operation;          // Operation type (PARAM_OP_*)
    float value;                // Parameter value (4 bytes)
    uint8_t source_channel;     // Which external interface sent request (1 byte)
    uint8_t request_id;         // Unique request ID for that channel (1 byte)
    uint8_t reserved[1];        // Future use (1 byte)
} __attribute__((packed)) parameter_msg_t;
```

### 2. Channel Identifiers

Channel IDs are defined for routing:

```cpp
#define CHANNEL_SERIAL_USB    0x01
#define CHANNEL_SERIAL_1      0x02
#define CHANNEL_SERIAL_2      0x03
#define CHANNEL_CAN_BUS       0x04
```

### 3. Parameter Registry System

A central parameter registry manages all parameter handlers:

```cpp
class ParameterRegistry {
public:
    static bool register_parameter(uint32_t param_id, 
                                 parameter_read_handler_t read_handler,
                                 parameter_write_handler_t write_handler, 
                                 const char* description);
    static ParameterHandler* find_handler(uint32_t param_id);
    static void handle_parameter_request(const CANMessage* msg);
};
```

### 4. Request Tracker System

Each external interface has a request tracker that monitors pending requests:

```cpp
class RequestTracker {
public:
    void add_request(uint8_t channel, uint32_t param_id);
    void remove_request(uint8_t request_id, uint8_t channel);
    void cleanup_timeouts(uint32_t timeout_ms = DEFAULT_TIMEOUT_MS);
    uint8_t get_next_request_id();
    bool is_pending_request(uint8_t request_id, uint8_t channel) const;
};
```

## Implementation Details

### Parameter Registration

Modules register their parameters during initialization:

```cpp
// In transmission_module.cpp
ParameterRegistry::register_parameter(MSG_TRANS_CURRENT_GEAR, 
    []() -> float { return (float)trans_state.current_gear; }, 
    nullptr, "Current Gear");
```

### Request Processing

When an external interface receives a parameter request:

1. **Add routing metadata**: Set `source_channel` and `request_id`
2. **Track the request**: Add to request tracker
3. **Forward to message bus**: Publish to internal message bus

```cpp
// In external_serial.cpp - SerialBridge::process_complete_message()
if (current_message.len == sizeof(parameter_msg_t)) {
    parameter_msg_t* param = (parameter_msg_t*)current_message.buf;
    
    if (param->operation == PARAM_OP_READ_REQUEST || 
        param->operation == PARAM_OP_WRITE_REQUEST) {
        
        // Add routing metadata
        param->source_channel = channel_id;
        param->request_id = request_tracker.get_next_request_id();
        
        // Track this request
        request_tracker.add_request(channel_id, current_message.id);
    }
}
```

### Response Routing

When a parameter response is generated:

1. **Filter by channel**: Only process responses for this channel
2. **Strip routing info**: Remove routing metadata before sending to external tool
3. **Remove from tracker**: Clean up the tracked request

```cpp
// In external_serial.cpp - ExternalSerial::on_message_bus_message()
if (param->operation == PARAM_OP_READ_RESPONSE || 
    param->operation == PARAM_OP_WRITE_ACK) {
    
    if (param->source_channel == CHANNEL_SERIAL_USB && usb_bridge.is_enabled()) {
        // Strip routing info before sending to external tool
        CANMessage external_response = *msg;
        strip_routing_metadata(&external_response);
        usb_bridge.send_message(external_response);
        
        // Remove from request tracker
        usb_bridge.remove_pending_request(param->request_id, param->source_channel);
    }
}
```

### Central Parameter Handling

The parameter registry handles all parameter requests centrally:

```cpp
// In parameter_registry.cpp
void ParameterRegistry::handle_parameter_request(const CANMessage* msg) {
    parameter_msg_t* param = get_parameter_msg(msg);
    ParameterHandler* handler = find_handler(msg->id);
    
    if (!handler) {
        send_parameter_error_routed(msg->id, param->operation, 
                                  PARAM_ERROR_INVALID_OPERATION, param->value,
                                  param->source_channel, param->request_id);
        return;
    }
    
    switch (param->operation) {
        case PARAM_OP_READ_REQUEST:
            if (handler->read_handler) {
                float value = handler->read_handler();
                send_parameter_response_routed(msg->id, PARAM_OP_READ_RESPONSE, 
                                             value, param->source_channel, 
                                             param->request_id);
            }
            break;
        // ... handle other operations
    }
}
```

## File Structure

### New Files Created

- `parameter_registry.h` - Parameter registry header
- `parameter_registry.cpp` - Parameter registry implementation
- `request_tracker.h` - Request tracker header
- `request_tracker.cpp` - Request tracker implementation
- `tests/parameter_registry/test_parameter_registry.cpp` - Parameter registry tests
- `tests/parameter_registry/test_request_tracker.cpp` - Request tracker tests

### Modified Files

- `msg_definitions.h` - Updated parameter message structure and added channel IDs
- `parameter_helpers.h` - Added routing-aware helper functions
- `external_serial.h/cpp` - Added request tracking and response filtering
- `external_canbus.h/cpp` - Added request tracking and response filtering
- `transmission_module.cpp` - Updated to use parameter registry
- `main_application.cpp` - Added parameter registry setup

## Usage Examples

### Registering Parameters

```cpp
// Register a read-only parameter
ParameterRegistry::register_parameter(MSG_ENGINE_RPM, 
    get_engine_rpm, nullptr, "Engine RPM");

// Register a writable parameter
ParameterRegistry::register_parameter(MSG_FUEL_PRESSURE, 
    get_fuel_pressure, set_fuel_pressure, "Fuel Pressure");
```

### Sending Parameter Requests

External tools send parameter requests as before, but now the system automatically:
- Adds routing metadata
- Tracks the request
- Routes the response back to the correct channel

### Receiving Parameter Responses

External tools receive parameter responses as before, but now:
- Only receive responses to their own requests
- Don't receive responses to requests from other tools
- Responses are automatically cleaned up after delivery

## Benefits

1. **No Message Flooding**: External tools only receive responses to their own requests
2. **Central Management**: All parameters are managed in one place
3. **Module Self-Registration**: Modules register their own parameters during initialization
4. **Automatic Timeout Handling**: Stale requests are automatically cleaned up
5. **Backward Compatibility**: Existing external tools continue to work without changes
6. **Scalable**: Easy to add new parameters and external interfaces

## Testing

The implementation includes comprehensive tests:

- Parameter registration and lookup
- Request/response handling with routing
- Error handling
- Request tracking
- Channel isolation
- Timeout handling

Run tests with:
```bash
cd backslider_ecu_core_v2/tests
make test_parameter_registry
make test_request_tracker
```

## Future Enhancements

1. **Priority Queuing**: Add priority levels for parameter requests
2. **Bulk Operations**: Support for reading/writing multiple parameters at once
3. **Parameter Validation**: Add range checking and validation
4. **Statistics**: Add detailed statistics for monitoring
5. **Configuration**: Make timeout values and limits configurable

## Conclusion

The request-response channel routing system successfully solves the message flooding problem while maintaining backward compatibility and providing a clean, scalable architecture for parameter management. The implementation is well-tested and ready for production use. 