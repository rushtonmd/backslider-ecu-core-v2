#include "external_message_broadcasting.h"
#include "external_canbus.h"
#include "external_serial.h"

// =============================================================================
// STATIC MEMBER VARIABLES
// =============================================================================

broadcast_message_config_t ExternalMessageBroadcasting::broadcast_configs[MAX_BROADCAST_MESSAGES];
uint8_t ExternalMessageBroadcasting::registered_message_count = 0;
bool ExternalMessageBroadcasting::broadcasting_enabled = true;
ExternalCanBus* ExternalMessageBroadcasting::external_canbus = nullptr;
ExternalSerial* ExternalMessageBroadcasting::external_serial = nullptr;

uint32_t ExternalMessageBroadcasting::total_messages_broadcast = 0;
uint32_t ExternalMessageBroadcasting::can_bus_broadcasts = 0;
uint32_t ExternalMessageBroadcasting::serial_broadcasts = 0;

// =============================================================================
// PUBLIC METHODS
// =============================================================================

void ExternalMessageBroadcasting::init() {
    // Initialize broadcast configuration array
    for (uint8_t i = 0; i < MAX_BROADCAST_MESSAGES; i++) {
        broadcast_configs[i].msg_id = 0;
        broadcast_configs[i].description = nullptr;
        broadcast_configs[i].enabled = false;
    }
    
    registered_message_count = 0;
    broadcasting_enabled = true;
    external_canbus = nullptr;
    external_serial = nullptr;
    
    total_messages_broadcast = 0;
    can_bus_broadcasts = 0;
    serial_broadcasts = 0;
    
    #ifdef ARDUINO
    Serial.println("ExternalMessageBroadcasting: Initialized");
    #endif
}

bool ExternalMessageBroadcasting::register_broadcast_message(uint32_t msg_id, 
                                                           const char* description,
                                                           uint32_t frequency_hz,
                                                           broadcast_target_t targets) {
    #ifdef ARDUINO
    Serial.print("DEBUG: register_broadcast_message called for 0x");
    Serial.print(msg_id, HEX);
    Serial.print(" (");
    Serial.print(description);
    Serial.print(") frequency_hz=");
    Serial.print(frequency_hz);
    Serial.print(", current count=");
    Serial.print(registered_message_count);
    Serial.print("/");
    Serial.print(MAX_BROADCAST_MESSAGES);
    Serial.println();
    #endif
    
    if (registered_message_count >= MAX_BROADCAST_MESSAGES) {
        #ifdef ARDUINO
        Serial.println("ExternalMessageBroadcasting: Maximum broadcast messages reached");
        #endif
        return false;
    }
    
    // Check if message is already registered
    if (find_message_config(msg_id) >= 0) {
        #ifdef ARDUINO
        Serial.print("ExternalMessageBroadcasting: Message 0x");
        Serial.print(msg_id, HEX);
        Serial.println(" already registered");
        #endif
        return false;
    }
    
    // Add new broadcast configuration
    broadcast_configs[registered_message_count].msg_id = msg_id;
    broadcast_configs[registered_message_count].description = description;
    broadcast_configs[registered_message_count].enabled = true;
    broadcast_configs[registered_message_count].broadcast_frequency_hz = frequency_hz;
    broadcast_configs[registered_message_count].last_broadcast_ms = 0;
    broadcast_configs[registered_message_count].cached_value = 0.0f;
    broadcast_configs[registered_message_count].has_cached_value = false;
    broadcast_configs[registered_message_count].last_update_ms = 0;
    
    #ifdef ARDUINO
    Serial.print("DEBUG: About to subscribe to message 0x");
    Serial.print(msg_id, HEX);
    Serial.println(" on message bus");
    #endif
    
    // Subscribe to this message on the message bus
    bool subscribe_result = g_message_bus.subscribe(msg_id, on_message_received);
    
    #ifdef ARDUINO
    Serial.print("DEBUG: Subscribe result for message 0x");
    Serial.print(msg_id, HEX);
    Serial.print(": ");
    Serial.println(subscribe_result ? "SUCCESS" : "FAILED");
    #endif
    
    #ifdef ARDUINO
    Serial.print("DEBUG: Successfully subscribed to message 0x");
    Serial.print(msg_id, HEX);
    Serial.println(" on message bus");
    #endif
    
    registered_message_count++;
    
    #ifdef ARDUINO
    Serial.print("ExternalMessageBroadcasting: Registered message 0x");
    Serial.print(msg_id, HEX);
    Serial.print(" (");
    Serial.print(description);
    Serial.print(") at index ");
    Serial.print(registered_message_count - 1);
    Serial.print(", frequency_hz=");
    Serial.print(frequency_hz);
    Serial.print(" - SUCCESS");
    Serial.println();
    #endif
    
    return true;
}

bool ExternalMessageBroadcasting::unregister_broadcast_message(uint32_t msg_id) {
    int8_t index = find_message_config(msg_id);
    if (index < 0) {
        return false;
    }
    
    // Note: MessageBus doesn't have unsubscribe method yet
    // g_message_bus.unsubscribe(msg_id, on_message_received);
    
    // Remove from configuration array (shift remaining entries)
    for (uint8_t i = index; i < registered_message_count - 1; i++) {
        broadcast_configs[i] = broadcast_configs[i + 1];
    }
    
    // Clear last entry
    broadcast_configs[registered_message_count - 1].msg_id = 0;
    broadcast_configs[registered_message_count - 1].description = nullptr;
    broadcast_configs[registered_message_count - 1].enabled = false;
    
    registered_message_count--;
    
    #ifdef ARDUINO
    Serial.print("ExternalMessageBroadcasting: Unregistered message 0x");
    Serial.println(msg_id, HEX);
    #endif
    
    return true;
}

bool ExternalMessageBroadcasting::enable_broadcast_message(uint32_t msg_id, bool enable) {
    int8_t index = find_message_config(msg_id);
    if (index < 0) {
        return false;
    }
    
    broadcast_configs[index].enabled = enable;
    
    #ifdef ARDUINO
    Serial.print("ExternalMessageBroadcasting: ");
    Serial.print(enable ? "Enabled" : "Disabled");
    Serial.print(" broadcast for message 0x");
    Serial.println(msg_id, HEX);
    #endif
    
    return true;
}

bool ExternalMessageBroadcasting::set_broadcast_frequency(uint32_t msg_id, uint32_t frequency_hz) {
    int8_t index = find_message_config(msg_id);
    if (index < 0) {
        return false;
    }
    
    broadcast_configs[index].broadcast_frequency_hz = frequency_hz;
    
    #ifdef ARDUINO
    Serial.print("ExternalMessageBroadcasting: Set frequency for message 0x");
    Serial.print(msg_id, HEX);
    Serial.print(" to ");
    Serial.print(frequency_hz);
    Serial.println(" Hz");
    #endif
    
    return true;
}

void ExternalMessageBroadcasting::enable_all_broadcasts(bool enable) {
    broadcasting_enabled = enable;
    
    #ifdef ARDUINO
    Serial.print("ExternalMessageBroadcasting: ");
    Serial.print(enable ? "Enabled" : "Disabled");
    Serial.println(" all broadcasts");
    #endif
}

void ExternalMessageBroadcasting::set_external_interfaces(ExternalCanBus* canbus, ExternalSerial* serial) {
    external_canbus = canbus;
    external_serial = serial;
    
    #ifdef ARDUINO
    Serial.println("ExternalMessageBroadcasting: External interfaces set");
    #endif
}

uint32_t ExternalMessageBroadcasting::get_messages_broadcast() {
    return total_messages_broadcast;
}

uint32_t ExternalMessageBroadcasting::get_can_bus_broadcasts() {
    return can_bus_broadcasts;
}

uint32_t ExternalMessageBroadcasting::get_serial_broadcasts() {
    return serial_broadcasts;
}

void ExternalMessageBroadcasting::reset_statistics() {
    total_messages_broadcast = 0;
    can_bus_broadcasts = 0;
    serial_broadcasts = 0;
}

const broadcast_message_config_t* ExternalMessageBroadcasting::get_broadcast_configs(uint8_t* count) {
    if (count) {
        *count = registered_message_count;
    }
    return broadcast_configs;
}

bool ExternalMessageBroadcasting::is_message_registered(uint32_t msg_id) {
    return find_message_config(msg_id) >= 0;
}

void ExternalMessageBroadcasting::update() {
    if (!broadcasting_enabled) {
        return;
    }
    
    uint32_t now_ms = millis();
    
    // Check all registered messages for frequency-based broadcasting
    for (uint8_t i = 0; i < registered_message_count; i++) {
        if (!broadcast_configs[i].enabled || 
            broadcast_configs[i].broadcast_frequency_hz == 0 ||
            !broadcast_configs[i].has_cached_value) {
            continue;
        }
        
        // Check if it's time to broadcast based on frequency
        uint32_t interval_ms = 1000 / broadcast_configs[i].broadcast_frequency_hz;
        if (now_ms - broadcast_configs[i].last_broadcast_ms >= interval_ms) {
            // Create a message from cached value and broadcast it
            CANMessage cached_msg;
            cached_msg.id = broadcast_configs[i].msg_id;
            cached_msg.len = 4;
            memcpy(cached_msg.buf, &broadcast_configs[i].cached_value, 4);
            #ifdef ARDUINO
            cached_msg.timestamp = (uint16_t)(micros() & 0xFFFF);  // Set timestamp for cached message
            cached_msg.flags.extended = 1;
            cached_msg.flags.remote = 0;
            #else
            cached_msg.timestamp = 0;
            cached_msg.flags.extended = 1;
            cached_msg.flags.remote = 0;
            #endif
            
            broadcast_message(&cached_msg);
            broadcast_configs[i].last_broadcast_ms = now_ms;
        }
    }
}

// =============================================================================
// PRIVATE METHODS
// =============================================================================

void ExternalMessageBroadcasting::on_message_received(const CANMessage* msg) {
    if (!broadcasting_enabled || !msg) {
        #ifdef ARDUINO
        Serial.println("DEBUG: on_message_received early return - broadcasting disabled or null msg");
        #endif
        return;
    }
    
    // Check if this message is registered for broadcasting
    int8_t index = find_message_config(msg->id);
    

    

    // Temporarily disable find debug output to reduce flooding
    // #ifdef ARDUINO
    // static uint32_t last_find_debug = 0;
    // uint32_t find_now = millis();
    // if (find_now - last_find_debug >= 3000) {
    //     Serial.print("DEBUG: find_message_config for 0x");
    //     Serial.print(msg->id, HEX);
    //     Serial.print(" returned index: ");
    //     Serial.print(index);
    //     Serial.print(", enabled: ");
    //     Serial.print(index >= 0 ? (broadcast_configs[index].enabled ? "true" : "false") : "N/A");
    //     Serial.print(", registered_count: ");
    //     Serial.print(registered_message_count);
    //     Serial.println();
    //     
    //     // Show what's actually in the broadcast configs
    //     Serial.println("DEBUG: Current broadcast configs:");
    //     for (uint8_t i = 0; i < registered_message_count; i++) {
    //         Serial.print("  [");
    //             Serial.print(i);
    //             Serial.print("] 0x");
    //             Serial.print(broadcast_configs[i].msg_id, HEX);
    //             Serial.print(" (");
    //             Serial.print(broadcast_configs[i].description ? broadcast_configs[i].description : "null");
    //             Serial.print(") enabled=");
    //             Serial.println(broadcast_configs[i].enabled ? "true" : "false");
    //         }
    //         last_find_debug = find_now;
    //     }
    //     #endif
    if (index < 0 || !broadcast_configs[index].enabled) {
        return;
    }
    
    // Extract current value
    float current_value = 0.0f;
    if (msg->len >= 4) {
        memcpy(&current_value, msg->buf, 4);
    }
    
    // Store the original cached value for comparison
    float original_cached_value = broadcast_configs[index].cached_value;
    bool had_cached_value = broadcast_configs[index].has_cached_value;
    
    // Update cache with new value
    broadcast_configs[index].cached_value = current_value;
    broadcast_configs[index].has_cached_value = true;
    broadcast_configs[index].last_update_ms = millis();
    
    // For frequency-based broadcasting, just cache the value and let update() handle timing
    // For change-based broadcasting, broadcast immediately when value changes
    bool should_broadcast = false;
    
    if (broadcast_configs[index].broadcast_frequency_hz > 0) {
        // Frequency-based broadcasting - just cache the value, don't broadcast yet
        // The update() method will handle broadcasting based on frequency
        should_broadcast = false;
    } else {
        // Change-based broadcasting (only when value changes)
        // For change-based broadcasting, we should broadcast on the first value received
        // and then only when the value actually changes
        if (!had_cached_value || current_value != original_cached_value) {
            should_broadcast = true;
        }
    }
    
    if (should_broadcast) {
        broadcast_message(msg);
    }
}

void ExternalMessageBroadcasting::broadcast_message(const CANMessage* msg) {
    if (!msg) {
        return;
    }

    total_messages_broadcast++;
    
    // TEMPORARILY DISABLED: Comment out broadcasting to clean up debug output
    /*
    // NEW: Use selective message handling instead of legacy global broadcast
    // Only broadcast messages that are explicitly registered for broadcasting
    extern ExternalSerial g_external_serial;
    if (g_external_serial.is_initialized()) {
        g_external_serial.on_selective_message(msg);
    }
    */
    
    // Broadcast to CAN bus if available
    if (external_canbus && external_canbus->is_initialized()) {
        // Convert CANMessage to CAN_message_t and send
        CAN_message_t can_msg;
        can_msg.id = msg->id;
        can_msg.len = msg->len;
        memcpy(can_msg.buf, msg->buf, msg->len);
        
        if (external_canbus->send_custom_message(can_msg.id, can_msg.buf, can_msg.len)) {
            can_bus_broadcasts++;
        }
    }
    
    // Forward message to external serial for broadcasting
    // REMOVED: g_external_serial.on_message_bus_message(msg); - this was the legacy blast-everything approach
    
    // Send to external CAN networks if available
    int8_t config_index = find_message_config(msg->id);
    const broadcast_message_config_t* config = (config_index >= 0) ? &broadcast_configs[config_index] : nullptr;
    if (config) {
        // Debug output for message broadcasting (disabled to reduce serial clutter)
        /*
        #ifdef ARDUINO
        float value = MSG_UNPACK_FLOAT(msg);
        Serial.print("DEBUG: Broadcasting message 0x");
        Serial.print(config->message_id, HEX);
        Serial.print(" = ");
        Serial.println(value, 2);
        #endif
        */
        
        // Use cached timestamp for all external outputs for consistency
        CANMessage cached_msg = *msg;
        
        // Forward to external CAN bus if available (disabled for now)
        /*
        if (external_canbus) {
            external_canbus->send_can_message(cached_msg);
            can_bus_broadcasts++;
        }
        */
    }
    
    // Debug output removed to clean up serial monitor
    /*
    #ifdef ARDUINO
    // Debug output every 5 seconds
    static uint32_t last_debug_time = 0;
    uint32_t now = millis();
    if (now - last_debug_time >= 5000) {
        float value = 0.0f;
        if (msg->len >= 4) {
            memcpy(&value, msg->buf, 4);
        }
        Serial.print("DEBUG: Broadcasting message 0x");
        Serial.print(msg->id, HEX);
        Serial.print(" = ");
        Serial.println(value);
        last_debug_time = now;
    }
    #endif
    */
}

void ExternalMessageBroadcasting::force_broadcast_cached_values() {
    if (!broadcasting_enabled) {
        return;
    }
    
    // Force broadcast all cached values immediately
    for (uint8_t i = 0; i < registered_message_count; i++) {
        if (!broadcast_configs[i].enabled || !broadcast_configs[i].has_cached_value) {
            continue;
        }
        
        // Create a message from cached value and broadcast it
        CANMessage cached_msg;
        cached_msg.id = broadcast_configs[i].msg_id;
        cached_msg.len = 4;
        memcpy(cached_msg.buf, &broadcast_configs[i].cached_value, 4);
        
        broadcast_message(&cached_msg);
        broadcast_configs[i].last_broadcast_ms = millis();
    }
}

int8_t ExternalMessageBroadcasting::find_message_config(uint32_t msg_id) {
    // Debug output for message searching (disabled to reduce serial clutter)
    /*
    #ifdef ARDUINO
    static uint32_t last_search_debug = 0;
    uint32_t search_now = millis();
    if (search_now - last_search_debug >= 10000) {  // Increased to 10 seconds
        Serial.print("DEBUG: Searching for message 0x");
        Serial.print(msg_id, HEX);
        Serial.print(" in ");
        Serial.print(registered_message_count);
        Serial.println(" registered messages:");
        for (uint8_t i = 0; i < registered_message_count; i++) {
            Serial.print("  [");
            Serial.print(i);
            Serial.print("] 0x");
            Serial.print(broadcast_configs[i].msg_id, HEX);
            Serial.print(" (");
            Serial.print(broadcast_configs[i].description ? broadcast_configs[i].description : "null");
            Serial.print(") enabled=");
            Serial.println(broadcast_configs[i].enabled ? "true" : "false");
        }
        last_search_debug = search_now;
    }
    #endif
    */
    
    for (uint8_t i = 0; i < registered_message_count; i++) {
        if (broadcast_configs[i].msg_id == msg_id) {
            return i;
        }
    }
    return -1;
}

// =============================================================================
// CONVENIENCE FUNCTIONS
// =============================================================================

void register_common_broadcast_messages() {
    // Register engine-related messages (1Hz - moderate frequency)
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_ENGINE_RPM, "Engine RPM", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_COOLANT_TEMP, "Coolant Temperature", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_OIL_PRESSURE, "Oil Pressure", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_AIR_INTAKE_TEMP, "Air Intake Temperature", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_BATTERY_VOLTAGE, "Battery Voltage", 1);
    
    // Register vehicle state messages (moderate frequency for debugging)
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_VEHICLE_SPEED, "Vehicle Speed", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_THROTTLE_POSITION, "Throttle Position", 2);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_BRAKE_PEDAL, "Brake Pedal", 2);
    
    // Register transmission messages (change-based - only when gear changes)
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_CURRENT_GEAR, "Transmission Current Gear", 0);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_DRIVE_GEAR, "Transmission Drive Gear", 1);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_STATE_VALID, "Transmission State Valid", 0);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_OVERRUN_STATE, "Transmission Overrun State", 0);
}

void register_engine_broadcast_messages() {
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_ENGINE_RPM, "Engine RPM");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_COOLANT_TEMP, "Coolant Temperature");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_OIL_PRESSURE, "Oil Pressure");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_AIR_INTAKE_TEMP, "Air Intake Temperature");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_BATTERY_VOLTAGE, "Battery Voltage");
}

void register_transmission_broadcast_messages() {
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_CURRENT_GEAR, "Transmission Current Gear");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_STATE_VALID, "Transmission State Valid");
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_OVERRUN_STATE, "Transmission Overrun State");
}

void register_vehicle_state_broadcast_messages() {
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_VEHICLE_SPEED, "Vehicle Speed", 2);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_THROTTLE_POSITION, "Throttle Position", 2);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_BRAKE_PEDAL, "Brake Pedal", 2);
}

void register_critical_broadcast_messages() {
    // Critical messages that need high-frequency broadcasting (5Hz+)
    // These are essential for safety and real-time control
    
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_TRANS_CURRENT_GEAR, "Transmission Current Gear", 5);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_ENGINE_RPM, "Engine RPM", 5);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_VEHICLE_SPEED, "Vehicle Speed", 5);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_THROTTLE_POSITION, "Throttle Position", 5);
    ExternalMessageBroadcasting::register_broadcast_message(
        BROADCAST_MSG_BRAKE_PEDAL, "Brake Pedal", 5);
} 