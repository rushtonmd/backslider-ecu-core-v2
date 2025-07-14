// custom_message_handler.cpp
// Implementation of custom message protocol handler

#include "custom_message_handler.h"

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

CustomMessageHandler::CustomMessageHandler(ExternalCanBusCache* cache) :
    cache(cache),
    initialized(false),
    last_update_time(0)
{
    // Initialize statistics
    reset_statistics();
}

CustomMessageHandler::~CustomMessageHandler() {
    shutdown();
}

bool CustomMessageHandler::init() {
    if (initialized) {
        debug_print("CustomMessageHandler: Already initialized");
        return true;
    }
    
    if (cache == nullptr) {
        debug_print("CustomMessageHandler: Cache reference is null");
        return false;
    }
    
    // Clear all containers
    message_handlers.clear();
    value_providers.clear();
    message_configs.clear();
    last_transmission_time.clear();
    
    // Reset statistics
    reset_statistics();
    
    initialized = true;
    last_update_time = millis();
    
    debug_print("CustomMessageHandler: Initialization complete");
    return true;
}

void CustomMessageHandler::shutdown() {
    if (!initialized) {
        return;
    }
    
    // Clear all containers
    message_handlers.clear();
    value_providers.clear();
    message_configs.clear();
    last_transmission_time.clear();
    
    initialized = false;
    debug_print("CustomMessageHandler: Shutdown complete");
}

void CustomMessageHandler::update() {
    if (!initialized) {
        return;
    }
    
    // Process scheduled transmissions
    process_scheduled_transmissions();
    
    last_update_time = millis();
}

// ============================================================================
// MESSAGE HANDLER REGISTRATION
// ============================================================================

bool CustomMessageHandler::register_handler(uint32_t can_id, custom_message_handler_t handler) {
    if (!initialized || handler == nullptr) {
        return false;
    }
    
    message_handlers[can_id] = handler;
    
    char debug_msg[60];
    snprintf(debug_msg, sizeof(debug_msg), 
            "CustomMessageHandler: Registered handler for CAN ID 0x%03X", can_id);
    debug_print(debug_msg);
    
    return true;
}

bool CustomMessageHandler::unregister_handler(uint32_t can_id) {
    if (!initialized) {
        return false;
    }
    
    auto it = message_handlers.find(can_id);
    if (it != message_handlers.end()) {
        message_handlers.erase(it);
        
        char debug_msg[60];
        snprintf(debug_msg, sizeof(debug_msg), 
                "CustomMessageHandler: Unregistered handler for CAN ID 0x%03X", can_id);
        debug_print(debug_msg);
        
        return true;
    }
    
    return false;
}

bool CustomMessageHandler::register_value_provider(uint32_t external_key, custom_value_provider_t provider) {
    if (!initialized || provider == nullptr) {
        return false;
    }
    
    value_providers[external_key] = provider;
    
    char debug_msg[60];
    snprintf(debug_msg, sizeof(debug_msg), 
            "CustomMessageHandler: Registered value provider for key 0x%08X", external_key);
    debug_print(debug_msg);
    
    return true;
}

bool CustomMessageHandler::unregister_value_provider(uint32_t external_key) {
    if (!initialized) {
        return false;
    }
    
    auto it = value_providers.find(external_key);
    if (it != value_providers.end()) {
        value_providers.erase(it);
        return true;
    }
    
    return false;
}

// ============================================================================
// MESSAGE CONFIGURATION
// ============================================================================

bool CustomMessageHandler::configure_message(const custom_message_config_t& config) {
    if (!initialized) {
        return false;
    }
    
    message_configs[config.can_id] = config;
    
    // Initialize transmission tracking for transmit messages
    if (config.is_transmit && config.transmit_interval_ms > 0) {
        last_transmission_time[config.can_id] = 0;
    }
    
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), 
            "CustomMessageHandler: Configured CAN ID 0x%03X (%s) - %s", 
            config.can_id, 
            config.is_transmit ? "TX" : "RX",
            config.description);
    debug_print(debug_msg);
    
    return true;
}

bool CustomMessageHandler::configure_message(uint32_t can_id, uint32_t external_key, 
                                            uint32_t interval_ms, bool is_transmit, const char* description) {
    custom_message_config_t config = {
        .can_id = can_id,
        .external_key = external_key,
        .transmit_interval_ms = interval_ms,
        .timeout_ms = 1000,  // Default 1 second timeout
        .is_transmit = is_transmit,
        .cache_enabled = true,
        .description = description
    };
    
    return configure_message(config);
}

bool CustomMessageHandler::remove_message_config(uint32_t can_id) {
    if (!initialized) {
        return false;
    }
    
    auto config_it = message_configs.find(can_id);
    if (config_it != message_configs.end()) {
        message_configs.erase(config_it);
    }
    
    auto time_it = last_transmission_time.find(can_id);
    if (time_it != last_transmission_time.end()) {
        last_transmission_time.erase(time_it);
    }
    
    return true;
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

bool CustomMessageHandler::process_message(const CAN_message_t& msg) {
    if (!initialized) {
        return false;
    }
    
    stats.messages_received++;
    debug_print_custom_message(msg);
    
    // Process incoming message
    process_incoming_message(msg);
    
    // Look for registered handler
    auto handler_it = message_handlers.find(msg.id);
    if (handler_it != message_handlers.end()) {
        handler_it->second(msg.id, msg.buf, msg.len);
        stats.handler_calls++;
    }
    
    stats.messages_processed++;
    return true;
}

void CustomMessageHandler::process_incoming_message(const CAN_message_t& msg) {
    // Find message configuration
    custom_message_config_t* config = find_message_config(msg.id);
    if (config == nullptr || config->is_transmit) {
        return;  // No config or this is a transmit-only message
    }
    
    // Update cache if enabled
    if (config->cache_enabled) {
        if (update_cache_from_message(config->external_key, msg.buf, msg.len)) {
            stats.cache_updates++;
        }
    }
}

void CustomMessageHandler::process_scheduled_transmissions() {
    uint32_t current_time = millis();
    
    for (auto& config_pair : message_configs) {
        const custom_message_config_t& config = config_pair.second;
        
        // Skip non-transmit messages
        if (!config.is_transmit || config.transmit_interval_ms == 0) {
            continue;
        }
        
        // Check if transmission is due
        if (is_transmission_due(config.can_id)) {
            // Get value for transmission
            float value;
            if (get_value_for_transmission(config.external_key, &value)) {
                // Send the message
                if (send_float_message(config.can_id, value)) {
                    last_transmission_time[config.can_id] = current_time;
                } else {
                    stats.transmission_timeouts++;
                }
            }
        }
    }
}

bool CustomMessageHandler::send_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (!initialized || data == nullptr || length > 8) {
        return false;
    }
    
    // For this implementation, we'll just track statistics
    // In a real implementation, this would send via the parent ExternalCanBus
    stats.messages_sent++;
    
    char debug_msg[80];
    snprintf(debug_msg, sizeof(debug_msg), 
            "CustomMessageHandler: Sent message CAN ID 0x%03X, length %d", can_id, length);
    debug_print(debug_msg);
    
    return true;
}

bool CustomMessageHandler::send_float_message(uint32_t can_id, float value) {
    return send_message(can_id, (const uint8_t*)&value, sizeof(float));
}

bool CustomMessageHandler::send_uint32_message(uint32_t can_id, uint32_t value) {
    return send_message(can_id, (const uint8_t*)&value, sizeof(uint32_t));
}

// ============================================================================
// PREDEFINED MESSAGE PROTOCOLS
// ============================================================================

bool CustomMessageHandler::configure_dashboard_messages() {
    bool success = true;
    
    // Configure dashboard protocol messages
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DASHBOARD_CAN_ID_RPM, CUSTOM_DASHBOARD_RPM, 
        DEFAULT_DASHBOARD_INTERVAL_MS, "Dashboard RPM"));
    
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DASHBOARD_CAN_ID_SPEED, CUSTOM_DASHBOARD_SPEED, 
        DEFAULT_DASHBOARD_INTERVAL_MS, "Dashboard Speed"));
    
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DASHBOARD_CAN_ID_TEMPERATURE, CUSTOM_DASHBOARD_TEMP, 
        DEFAULT_DASHBOARD_INTERVAL_MS, "Dashboard Temperature"));
    
    if (success) {
        debug_print("CustomMessageHandler: Dashboard messages configured");
    }
    
    return success;
}

bool CustomMessageHandler::send_dashboard_rpm(float rpm) {
    return send_float_message(DASHBOARD_CAN_ID_RPM, rpm);
}

bool CustomMessageHandler::send_dashboard_speed(float speed) {
    return send_float_message(DASHBOARD_CAN_ID_SPEED, speed);
}

bool CustomMessageHandler::send_dashboard_temperature(float temp) {
    return send_float_message(DASHBOARD_CAN_ID_TEMPERATURE, temp);
}

bool CustomMessageHandler::configure_datalogger_messages() {
    bool success = true;
    
    // Configure datalogger protocol messages
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DATALOGGER_CAN_ID_ENGINE_DATA, CUSTOM_DATALOGGER_RPM, 
        DEFAULT_DATALOGGER_INTERVAL_MS, "Datalogger Engine Data"));
    
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DATALOGGER_CAN_ID_SENSOR_DATA, CUSTOM_DATALOGGER_TPS, 
        DEFAULT_DATALOGGER_INTERVAL_MS, "Datalogger Sensor Data"));
    
    if (success) {
        debug_print("CustomMessageHandler: Datalogger messages configured");
    }
    
    return success;
}

bool CustomMessageHandler::send_datalogger_data(float rpm, float tps, float map, float temp) {
    // Pack multiple values into a single message
    struct {
        float rpm;
        float tps;
    } engine_data = {rpm, tps};
    
    return send_message(DATALOGGER_CAN_ID_ENGINE_DATA, 
                       (const uint8_t*)&engine_data, sizeof(engine_data));
}

bool CustomMessageHandler::configure_display_messages() {
    bool success = true;
    
    // Configure display protocol messages
    success &= configure_message(CUSTOM_MESSAGE_CONFIG_TRANSMIT(
        DISPLAY_CAN_ID_BOOST, CUSTOM_DISPLAY_BOOST, 
        DEFAULT_DISPLAY_INTERVAL_MS, "Boost Display"));
    
    if (success) {
        debug_print("CustomMessageHandler: Display messages configured");
    }
    
    return success;
}

bool CustomMessageHandler::send_boost_display(float boost_psi) {
    return send_float_message(DISPLAY_CAN_ID_BOOST, boost_psi);
}

// ============================================================================
// CACHE INTEGRATION
// ============================================================================

bool CustomMessageHandler::update_cache_from_message(uint32_t external_key, const uint8_t* data, uint8_t length) {
    if (cache == nullptr || data == nullptr) {
        return false;
    }
    
    // For now, assume all custom messages contain float values
    if (length == sizeof(float)) {
        float value;
        memcpy(&value, data, sizeof(float));
        
        // We would need a way to inject this into the cache
        // For testing, we can simulate this
        #ifndef ARDUINO
        cache->simulate_internal_message(external_key, value);
        #endif
        
        return true;
    }
    
    return false;
}

bool CustomMessageHandler::get_value_for_transmission(uint32_t external_key, float* value) {
    if (cache == nullptr || value == nullptr) {
        return false;
    }
    
    // Try custom value provider first
    auto provider_it = value_providers.find(external_key);
    if (provider_it != value_providers.end()) {
        return provider_it->second(external_key, value);
    }
    
    // Fall back to cache
    return cache->get_value(external_key, value);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float CustomMessageHandler::extract_float_from_message(const CAN_message_t& msg) {
    if (msg.len == sizeof(float)) {
        float value;
        memcpy(&value, msg.buf, sizeof(float));
        return value;
    }
    return 0.0f;
}

uint32_t CustomMessageHandler::extract_uint32_from_message(const CAN_message_t& msg) {
    if (msg.len == sizeof(uint32_t)) {
        uint32_t value;
        memcpy(&value, msg.buf, sizeof(uint32_t));
        return value;
    }
    return 0;
}

custom_message_config_t* CustomMessageHandler::find_message_config(uint32_t can_id) {
    auto it = message_configs.find(can_id);
    if (it != message_configs.end()) {
        return &it->second;
    }
    return nullptr;
}

bool CustomMessageHandler::is_transmission_due(uint32_t can_id) {
    custom_message_config_t* config = find_message_config(can_id);
    if (config == nullptr || config->transmit_interval_ms == 0) {
        return false;
    }
    
    uint32_t current_time = millis();
    auto time_it = last_transmission_time.find(can_id);
    
    if (time_it == last_transmission_time.end()) {
        return true;  // First transmission
    }
    
    return (current_time - time_it->second) >= config->transmit_interval_ms;
}

// ============================================================================
// DIAGNOSTICS AND STATISTICS
// ============================================================================

void CustomMessageHandler::reset_statistics() {
    stats = {};  // Zero-initialize all fields
}

uint32_t CustomMessageHandler::get_handler_count() const {
    return message_handlers.size();
}

uint32_t CustomMessageHandler::get_configured_message_count() const {
    return message_configs.size();
}

// ============================================================================
// ERROR HANDLING AND DEBUGGING
// ============================================================================

void CustomMessageHandler::handle_error(const char* error_msg) {
    stats.format_errors++;
    debug_print(error_msg);
}

void CustomMessageHandler::increment_error_count() {
    stats.format_errors++;
}

void CustomMessageHandler::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void CustomMessageHandler::debug_print_custom_message(const CAN_message_t& msg) {
    #ifdef ARDUINO
    Serial.print("Custom Message: ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" LEN=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (uint8_t i = 0; i < msg.len; i++) {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    #else
    printf("Custom Message: ID=0x%03X LEN=%d DATA=", msg.id, msg.len);
    for (uint8_t i = 0; i < msg.len; i++) {
        printf("%02X ", msg.buf[i]);
    }
    printf("\n");
    #endif
}

// ============================================================================
// TESTING INTERFACE
// ============================================================================

#ifndef ARDUINO
bool CustomMessageHandler::simulate_custom_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (!initialized || data == nullptr || length > 8) {
        return false;
    }
    
    CAN_message_t msg = {};
    msg.id = can_id;
    msg.len = length;
    memcpy(msg.buf, data, length);
    msg.timestamp = micros();
    
    return process_message(msg);
}

bool CustomMessageHandler::simulate_dashboard_request() {
    // Simulate dashboard requesting RPM data
    float rpm_data = 3500.0f;
    return simulate_custom_message(DASHBOARD_CAN_ID_RPM, 
                                  (const uint8_t*)&rpm_data, sizeof(float));
}

bool CustomMessageHandler::simulate_datalogger_request() {
    // Simulate datalogger sending engine data
    struct {
        float rpm;
        float tps;
    } engine_data = {4200.0f, 85.0f};
    
    return simulate_custom_message(DATALOGGER_CAN_ID_ENGINE_DATA, 
                                  (const uint8_t*)&engine_data, sizeof(engine_data));
}

bool CustomMessageHandler::get_message_config_for_testing(uint32_t can_id, custom_message_config_t* config) {
    if (config == nullptr) {
        return false;
    }
    
    custom_message_config_t* found_config = find_message_config(can_id);
    if (found_config != nullptr) {
        *config = *found_config;
        return true;
    }
    
    return false;
}
#endif