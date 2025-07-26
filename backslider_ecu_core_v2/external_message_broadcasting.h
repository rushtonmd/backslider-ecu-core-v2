#ifndef EXTERNAL_MESSAGE_BROADCASTING_H
#define EXTERNAL_MESSAGE_BROADCASTING_H

#include "msg_bus.h"
#include "msg_definitions.h"

// =============================================================================
// EXTERNAL MESSAGE BROADCASTING MODULE
// =============================================================================
//
// This module handles selective broadcasting of specific internal messages
// to external interfaces (CAN bus and serial). It provides a centralized,
// configurable way to specify which messages should be broadcast externally.
//
// DESIGN PRINCIPLES:
// - Only broadcast messages that are safe and useful for external consumption
// - Consistent behavior across all external interfaces
// - Easy to configure and maintain
// - Minimal performance impact
//
// USAGE:
// 1. Register messages that should be broadcast externally
// 2. Module automatically subscribes to those messages
// 3. When messages are published, they're forwarded to external interfaces
// 4. External interfaces receive only the messages they should see
//
// =============================================================================

// Forward declarations
class ExternalCanBus;
class ExternalSerial;

// Message broadcasting configuration
typedef struct {
    uint32_t msg_id;           // Message ID to broadcast
    const char* description;   // Human-readable description
    bool enabled;              // Whether this message should be broadcast
    uint32_t broadcast_frequency_hz;  // How often to broadcast (0 = only when value changes)
    uint32_t last_broadcast_ms;       // Last time this message was broadcast
    float cached_value;               // Cached value for frequency-based broadcasting
    bool has_cached_value;            // Whether we have a valid cached value
    uint32_t last_update_ms;          // Last time the value was updated from message bus
} broadcast_message_config_t;

// External interface types
typedef enum {
    BROADCAST_TARGET_CAN_BUS = 0x01,
    BROADCAST_TARGET_SERIAL  = 0x02,
    BROADCAST_TARGET_ALL     = 0x03
} broadcast_target_t;

class ExternalMessageBroadcasting {
public:
    // Initialize the broadcasting module
    static void init();
    
    // Register a message for external broadcasting
    static bool register_broadcast_message(uint32_t msg_id, 
                                         const char* description,
                                         uint32_t frequency_hz = 0,
                                         broadcast_target_t targets = BROADCAST_TARGET_ALL);
    
    // Unregister a message from external broadcasting
    static bool unregister_broadcast_message(uint32_t msg_id);
    
    // Enable/disable broadcasting for a specific message
    static bool enable_broadcast_message(uint32_t msg_id, bool enable);
    
    // Set broadcast frequency for a specific message
    static bool set_broadcast_frequency(uint32_t msg_id, uint32_t frequency_hz);
    
    // Enable/disable broadcasting for all messages
    static void enable_all_broadcasts(bool enable);
    
    // Set external interface pointers (called by main application)
    static void set_external_interfaces(ExternalCanBus* canbus, ExternalSerial* serial);
    
    // Get statistics
    static uint32_t get_messages_broadcast();
    static uint32_t get_can_bus_broadcasts();
    static uint32_t get_serial_broadcasts();
    
    // Reset statistics
    static void reset_statistics();
    
    // Get list of registered broadcast messages
    static const broadcast_message_config_t* get_broadcast_configs(uint8_t* count);
    
    // Check if a message is registered for broadcasting
    static bool is_message_registered(uint32_t msg_id);
    
    // Update function (called from main loop)
    static void update();
    
    // Force broadcast of cached values (for frequency-based messages)
    static void force_broadcast_cached_values();

private:
    // Maximum number of broadcast messages
    static const uint8_t MAX_BROADCAST_MESSAGES = 50;
    
    // Message handler for subscribed messages
    static void on_message_received(const CANMessage* msg);
    
    // Broadcast a message to external interfaces
    static void broadcast_message(const CANMessage* msg);
    
    // Internal state
    static broadcast_message_config_t broadcast_configs[MAX_BROADCAST_MESSAGES];
    static uint8_t registered_message_count;
    static bool broadcasting_enabled;
    static ExternalCanBus* external_canbus;
    static ExternalSerial* external_serial;
    
    // Statistics
    static uint32_t total_messages_broadcast;
    static uint32_t can_bus_broadcasts;
    static uint32_t serial_broadcasts;
    
    // Find message config by ID
    static int8_t find_message_config(uint32_t msg_id);
};

// =============================================================================
// PREDEFINED BROADCAST MESSAGES
// =============================================================================
//
// These are the messages that are typically safe and useful to broadcast
// externally. They can be enabled/disabled individually or as groups.
//

// Engine-related messages (safe for external consumption)
#define BROADCAST_MSG_ENGINE_RPM              MSG_ENGINE_RPM
#define BROADCAST_MSG_COOLANT_TEMP            MSG_COOLANT_TEMP
#define BROADCAST_MSG_OIL_PRESSURE            MSG_OIL_PRESSURE
#define BROADCAST_MSG_AIR_INTAKE_TEMP         MSG_AIR_INTAKE_TEMP
#define BROADCAST_MSG_BATTERY_VOLTAGE         MSG_BATTERY_VOLTAGE

// Vehicle state messages
#define BROADCAST_MSG_VEHICLE_SPEED           MSG_VEHICLE_SPEED
#define BROADCAST_MSG_THROTTLE_POSITION       MSG_THROTTLE_POSITION
#define BROADCAST_MSG_BRAKE_PEDAL             MSG_BRAKE_PEDAL

// Transmission messages (safe subset)
#define BROADCAST_MSG_TRANS_CURRENT_GEAR      MSG_TRANS_CURRENT_GEAR
#define BROADCAST_MSG_TRANS_DRIVE_GEAR        MSG_TRANS_DRIVE_GEAR
#define BROADCAST_MSG_TRANS_STATE_VALID       MSG_TRANS_STATE_VALID
#define BROADCAST_MSG_TRANS_OVERRUN_STATE     MSG_TRANS_OVERRUN_STATE

// System status messages
#define BROADCAST_MSG_SYSTEM_STATUS           MSG_SYSTEM_STATUS
#define BROADCAST_MSG_ERROR_CODES             MSG_ERROR_CODES

// =============================================================================
// CONVENIENCE FUNCTIONS
// =============================================================================

// Register common broadcast messages
void register_common_broadcast_messages();

// Register engine-related broadcast messages
void register_engine_broadcast_messages();

// Register transmission broadcast messages  
void register_transmission_broadcast_messages();

// Register vehicle state broadcast messages
void register_vehicle_state_broadcast_messages();

// Register high-frequency critical messages (5Hz+)
void register_critical_broadcast_messages();

#endif // EXTERNAL_MESSAGE_BROADCASTING_H 