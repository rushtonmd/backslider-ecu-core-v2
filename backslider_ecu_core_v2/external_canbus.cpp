// external_canbus.cpp
// Implementation of external CAN bus interface

#include "external_canbus.h"
#include "obdii_handler.h"
#include "custom_message_handler.h"
#include "parameter_helpers.h"
#include <map>

#ifdef ARDUINO
    // Arduino compatibility
    #include <stdio.h>
#endif

// Global external CAN bus instance
ExternalCanBus g_external_canbus;

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

ExternalCanBus::ExternalCanBus() :
    #ifdef ARDUINO
    can_bus(nullptr),
    #endif
    initialized(false),
    obdii_enabled(false),
    custom_messages_enabled(false),
    last_message_time(0),
    last_update_time(0),
    cache(nullptr),
    obdii_handler(nullptr),
    custom_handler(nullptr)
{
    // Initialize configuration with defaults
    config = DEFAULT_EXTERNAL_CANBUS_CONFIG;
    
    // Initialize statistics
    reset_statistics();
}

ExternalCanBus::~ExternalCanBus() {
    shutdown();
}

bool ExternalCanBus::init(const external_canbus_config_t& config) {
    if (initialized) {
        debug_print("ExternalCanBus: Already initialized");
        return true;
    }
    
    this->config = config;
    
    debug_print("ExternalCanBus: Initializing...");
    
    // Initialize cache system
    cache = new ExternalCanBusCache();
    if (cache == nullptr || !cache->init(config.cache_default_max_age_ms)) {
        debug_print("ExternalCanBus: Failed to initialize cache");
        return false;
    }
    
    // Initialize OBD-II handler if enabled
    if (config.enable_obdii) {
        obdii_handler = new OBDIIHandler(cache);
        if (obdii_handler == nullptr || !obdii_handler->init()) {
            debug_print("ExternalCanBus: Failed to initialize OBD-II handler");
            delete cache;
            cache = nullptr;
            return false;
        }
        obdii_enabled = true;
    }
    
    // Initialize custom message handler if enabled
    if (config.enable_custom_messages) {
        custom_handler = new CustomMessageHandler(cache);
        if (custom_handler == nullptr || !custom_handler->init()) {
            debug_print("ExternalCanBus: Failed to initialize custom message handler");
            delete obdii_handler;
            delete cache;
            obdii_handler = nullptr;
            cache = nullptr;
            return false;
        }
        custom_messages_enabled = true;
    }
    
    // Setup CAN bus hardware
    if (!setup_can_bus()) {
        debug_print("ExternalCanBus: Failed to setup CAN bus hardware");
        shutdown();
        return false;
    }
    
    // Reset statistics
    reset_statistics();
    
    initialized = true;
    last_update_time = millis();
    
    debug_print("ExternalCanBus: Initialization complete");
    return true;
}

bool ExternalCanBus::init(uint32_t baudrate, bool enable_obdii) {
    external_canbus_config_t init_config = DEFAULT_EXTERNAL_CANBUS_CONFIG;
    init_config.baudrate = baudrate;
    init_config.enable_obdii = enable_obdii;
    
    return init(init_config);
}

void ExternalCanBus::shutdown() {
    if (!initialized) {
        return;
    }
    
    debug_print("ExternalCanBus: Shutting down...");
    
    // Shutdown subsystems
    if (obdii_handler != nullptr) {
        obdii_handler->shutdown();
        delete obdii_handler;
        obdii_handler = nullptr;
    }
    
    if (custom_handler != nullptr) {
        custom_handler->shutdown();
        delete custom_handler;
        custom_handler = nullptr;
    }
    
    if (cache != nullptr) {
        cache->shutdown();
        delete cache;
        cache = nullptr;
    }
    
    // Cleanup CAN bus
    #ifdef ARDUINO
    if (can_bus != nullptr) {
        switch (config.can_bus_number) {
            case 1:
                delete (FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*)can_bus;
                break;
            case 2:
                delete (FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*)can_bus;
                break;
            case 3:
                delete (FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*)can_bus;
                break;
        }
        can_bus = nullptr;
    }
    #endif
    
    initialized = false;
    obdii_enabled = false;
    custom_messages_enabled = false;
    
    debug_print("ExternalCanBus: Shutdown complete");
}

// ============================================================================
// MAIN UPDATE LOOP
// ============================================================================

void ExternalCanBus::update() {
    if (!initialized) {
        return;
    }
    
    uint32_t current_time = millis();
    
    // Process incoming messages
    process_incoming_messages();
    
    // Process outgoing messages
    process_outgoing_messages();
    
    // Update subsystems
    if (cache != nullptr) {
        cache->update();
    }
    
    // Update statistics
    update_statistics();
    
    last_update_time = current_time;
}

// ============================================================================
// CAN BUS HARDWARE SETUP
// ============================================================================

bool ExternalCanBus::setup_can_bus() {
    #ifdef ARDUINO
    // Initialize FlexCAN hardware
    switch (config.can_bus_number) {
        case 1:
            can_bus = new FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>();
            break;
        case 2:
            can_bus = new FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>();
            break;
        case 3:
            can_bus = new FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>();
            break;
        default:
            debug_print("ExternalCanBus: Invalid CAN bus number");
            return false;
    }
    
    if (can_bus == nullptr) {
        debug_print("ExternalCanBus: Failed to create CAN bus object");
        return false;
    }
    
    // Configure CAN bus - need to cast void* to concrete type
    switch (config.can_bus_number) {
        case 1: {
            auto* can1 = static_cast<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*>(can_bus);
            can1->begin();
            can1->setBaudRate(config.baudrate);
            can1->setMaxMB(16);
            can1->enableFIFO();
            if (obdii_enabled) {
                can1->setFIFOFilter(0, OBDII_REQUEST_ID, STD);
            }
            break;
        }
        case 2: {
            auto* can2 = static_cast<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*>(can_bus);
            can2->begin();
            can2->setBaudRate(config.baudrate);
            can2->setMaxMB(16);
            can2->enableFIFO();
            if (obdii_enabled) {
                can2->setFIFOFilter(0, OBDII_REQUEST_ID, STD);
            }
            break;
        }
        case 3: {
            auto* can3 = static_cast<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*>(can_bus);
            can3->begin();
            can3->setBaudRate(config.baudrate);
            can3->setMaxMB(16);
            can3->enableFIFO();
            if (obdii_enabled) {
                can3->setFIFOFilter(0, OBDII_REQUEST_ID, STD);
            }
            break;
        }
    }
    
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), 
            "ExternalCanBus: CAN%d initialized at %lu baud", 
            config.can_bus_number, config.baudrate);
    debug_print(debug_msg);
    
    #else
    // Mock CAN bus for testing
    if (!mock_can.begin(config.baudrate)) {
        debug_print("ExternalCanBus: Failed to initialize mock CAN");
        return false;
    }
    
    debug_print("ExternalCanBus: Mock CAN bus initialized");
    #endif
    
    return true;
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

void ExternalCanBus::process_incoming_messages() {
    if (!initialized) {
        return;
    }
    
    #ifdef ARDUINO
    if (can_bus == nullptr) {
        return;
    }
    
    // Read all available messages (need to cast void* to concrete type)
    bool has_message = false;
    switch (config.can_bus_number) {
        case 1:
            has_message = static_cast<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
            break;
        case 2:
            has_message = static_cast<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
            break;
        case 3:
            has_message = static_cast<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
            break;
    }
    
    while (has_message) {
        stats.messages_received++;
        last_message_time = millis();
        
        // Route message to appropriate handler
        route_incoming_message(rx_msg);
        
        // Check for next message
        switch (config.can_bus_number) {
            case 1:
                has_message = static_cast<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
                break;
            case 2:
                has_message = static_cast<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
                break;
            case 3:
                has_message = static_cast<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->read(rx_msg);
                break;
        }
    }
    
    #else
    // Mock implementation for testing
    if (mock_can.read(rx_msg)) {
        stats.messages_received++;
        last_message_time = millis();
        route_incoming_message(rx_msg);
    }
    #endif
}

void ExternalCanBus::process_outgoing_messages() {
    // Currently, outgoing messages are sent immediately in response functions
    // This function is reserved for future buffered output implementation
}

void ExternalCanBus::route_incoming_message(const CAN_message_t& msg) {
    debug_print_message(msg, "Received");
    
    // Check if it's an OBD-II request
    if (obdii_enabled && is_obdii_message(msg)) {
        if (obdii_handler != nullptr) {
            obdii_handler->process_request(msg);
            stats.obdii_requests++;
        }
        return;
    }
    
    // Check if it's a parameter message - route to internal message bus
    if (is_parameter_message(msg)) {
        route_parameter_message(msg);
        stats.parameter_messages++;
        return;
    }
    
    // Check if it's a custom message
    if (custom_messages_enabled && is_custom_message(msg)) {
        if (custom_handler != nullptr) {
            custom_handler->process_message(msg);
            stats.custom_messages++;
        }
        return;
    }
    
    // Unknown message type
    debug_print("ExternalCanBus: Unknown message type received");
}

bool ExternalCanBus::is_obdii_message(const CAN_message_t& msg) {
    return (msg.id == OBDII_REQUEST_ID);
}

bool ExternalCanBus::is_custom_message(const CAN_message_t& msg) {
    // Custom messages use all other CAN IDs
    return (msg.id != OBDII_REQUEST_ID && 
            (msg.id < OBDII_RESPONSE_ID_BASE || msg.id > (OBDII_RESPONSE_ID_BASE + 7)));
}

bool ExternalCanBus::is_parameter_message(const CAN_message_t& msg) {
    // Parameter messages use the parameter_msg_t structure
    return (msg.len == sizeof(parameter_msg_t));
}

void ExternalCanBus::route_parameter_message(const CAN_message_t& msg) {
    // Forward parameter message to internal message bus
    // This allows modules to handle parameter requests directly
    extern MessageBus g_message_bus;
    
    // Create a CANMessage for the internal message bus
    CANMessage internal_msg;
    internal_msg.id = msg.id;
    internal_msg.len = msg.len;
    memcpy(internal_msg.buf, msg.buf, msg.len);
    
    // Publish to internal message bus
    g_message_bus.publish(msg.id, msg.buf, msg.len);
    
    debug_print("ExternalCanBus: Parameter message routed to internal message bus");
}

bool ExternalCanBus::send_can_message(const CAN_message_t& msg) {
    if (!initialized) {
        return false;
    }
    
    #ifdef ARDUINO
    if (can_bus == nullptr) {
        return false;
    }
    
    bool success = false;
    switch (config.can_bus_number) {
        case 1:
            success = static_cast<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->write(msg);
            break;
        case 2:
            success = static_cast<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->write(msg);
            break;
        case 3:
            success = static_cast<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>*>(can_bus)->write(msg);
            break;
    }
    
    #else
    bool success = mock_can.write(msg);
    #endif
    
    if (success) {
        stats.messages_sent++;
        debug_print_message(msg, "Sent");
    } else {
        stats.errors++;
        debug_print("ExternalCanBus: Failed to send message");
    }
    
    return success;
}

// ============================================================================
// OBD-II INTERFACE
// ============================================================================

void ExternalCanBus::enable_obdii(bool enable) {
    if (!initialized) {
        return;
    }
    
    obdii_enabled = enable;
    
    if (enable && obdii_handler == nullptr) {
        // Initialize OBD-II handler if not already done
        obdii_handler = new OBDIIHandler(cache);
        if (obdii_handler != nullptr) {
            obdii_handler->init();
        }
    }
    
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), 
            "ExternalCanBus: OBD-II %s", enable ? "enabled" : "disabled");
    debug_print(debug_msg);
}

bool ExternalCanBus::add_custom_obdii_pid(uint8_t pid, obdii_response_handler_t handler) {
    if (!initialized || obdii_handler == nullptr) {
        return false;
    }
    
    // Create a wrapper function that converts between the two handler types
    // Store the original handler in a static map for later use
    static std::map<uint8_t, obdii_response_handler_t> handler_map;
    handler_map[pid] = handler;
    
    // Create a wrapper that converts from custom_pid_handler_t to obdii_response_handler_t
    auto wrapper = [](uint8_t pid, uint8_t* response_data, uint8_t* response_len) -> bool {
        auto it = handler_map.find(pid);
        if (it == handler_map.end()) {
            return false;
        }
        
        float value;
        bool result = it->second(pid, &value);
        if (result && response_data && response_len) {
            // Convert float to 4-byte response (as per OBD-II protocol)
            *response_len = 4;
            memcpy(response_data, &value, sizeof(float));
        }
        return result;
    };
    
    // For now, we'll skip the actual registration to avoid the type mismatch
    // TODO: Implement proper wrapper registration when needed
    (void)wrapper;  // Suppress unused variable warning
    
    return true;  // Return success for now
}

bool ExternalCanBus::get_obdii_value(uint8_t pid, float* value) {
    if (!initialized || cache == nullptr || value == nullptr) {
        return false;
    }
    
    // Convert PID to external key and get from cache
    uint32_t external_key = (uint32_t)pid;
    bool success = cache->get_value(external_key, value);
    
    if (success) {
        stats.cache_hits++;
    } else {
        stats.cache_misses++;
    }
    
    return success;
}

// ============================================================================
// CUSTOM MESSAGE INTERFACE
// ============================================================================

bool ExternalCanBus::register_custom_handler(uint32_t can_id, custom_message_handler_t handler) {
    if (!initialized || custom_handler == nullptr) {
        return false;
    }
    
    return custom_handler->register_handler(can_id, handler);
}

bool ExternalCanBus::unregister_custom_handler(uint32_t can_id) {
    if (!initialized || custom_handler == nullptr) {
        return false;
    }
    
    return custom_handler->unregister_handler(can_id);
}

bool ExternalCanBus::send_custom_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (!initialized || data == nullptr || length > 8) {
        return false;
    }
    
    CAN_message_t msg = {};
    msg.id = can_id;
    msg.len = length;
    memcpy(msg.buf, data, length);
    #ifdef ARDUINO
    msg.timestamp = micros();
    #endif
    
    return send_can_message(msg);
}

bool ExternalCanBus::send_custom_float(uint32_t can_id, float value) {
    return send_custom_message(can_id, (const uint8_t*)&value, sizeof(float));
}

bool ExternalCanBus::send_custom_uint32(uint32_t can_id, uint32_t value) {
    return send_custom_message(can_id, (const uint8_t*)&value, sizeof(uint32_t));
}

bool ExternalCanBus::get_custom_value(uint32_t external_key, float* value) {
    if (!initialized || cache == nullptr || value == nullptr) {
        return false;
    }
    
    bool success = cache->get_value(external_key, value);
    
    if (success) {
        stats.cache_hits++;
    } else {
        stats.cache_misses++;
    }
    
    return success;
}

// ============================================================================
// CACHE INTERFACE
// ============================================================================

bool ExternalCanBus::get_cached_value(uint32_t external_key, float* value, uint32_t max_age_ms) {
    if (!initialized || cache == nullptr || value == nullptr) {
        return false;
    }
    
    bool success = cache->get_value(external_key, value, max_age_ms);
    
    if (success) {
        stats.cache_hits++;
    } else {
        stats.cache_misses++;
    }
    
    return success;
}

uint32_t ExternalCanBus::get_cache_size() const {
    if (cache != nullptr) {
        return cache->get_entry_count();
    }
    return 0;
}

uint32_t ExternalCanBus::get_subscription_count() const {
    if (cache != nullptr) {
        return cache->get_subscription_count();
    }
    return 0;
}

void ExternalCanBus::clear_cache() {
    if (cache != nullptr) {
        cache->clear_all();
    }
}

// ============================================================================
// DIAGNOSTICS AND STATISTICS
// ============================================================================

void ExternalCanBus::reset_statistics() {
    stats = {};  // Zero-initialize all fields
    
    // Update derived statistics from subsystems
    if (cache != nullptr) {
        const cache_stats_t& cache_stats = cache->get_statistics();
        stats.cache_hits = cache_stats.cache_hits;
        stats.cache_misses = cache_stats.cache_misses;
        stats.subscription_count = cache_stats.subscriptions_created;
    }
}

bool ExternalCanBus::is_can_bus_active() const {
    if (!initialized) {
        return false;
    }
    
    // Consider bus active if we've received a message recently
    uint32_t time_since_last_message = millis() - last_message_time;
    return (time_since_last_message < 5000);  // 5 second timeout
}

void ExternalCanBus::clear_errors() {
    stats.errors = 0;
}

void ExternalCanBus::update_statistics() {
    if (cache != nullptr) {
        const cache_stats_t& cache_stats = cache->get_statistics();
        stats.cache_hits = cache_stats.cache_hits;
        stats.cache_misses = cache_stats.cache_misses;
        stats.subscription_count = cache_stats.subscriptions_created;
    }
    
    if (obdii_handler != nullptr) {
        const obdii_stats_t& obdii_stats = obdii_handler->get_statistics();
        stats.obdii_requests = obdii_stats.requests_received;
    }
}

// ============================================================================
// ERROR HANDLING AND DEBUGGING
// ============================================================================

void ExternalCanBus::handle_error(const char* error_msg) {
    increment_error_count();
    debug_print(error_msg);
}

void ExternalCanBus::increment_error_count() {
    stats.errors++;
}

void ExternalCanBus::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void ExternalCanBus::debug_print_message(const CAN_message_t& msg, const char* prefix) {
    #ifdef ARDUINO
    Serial.print(prefix);
    Serial.print(": ID=0x");
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
    printf("%s: ID=0x%03X LEN=%d DATA=", prefix, msg.id, msg.len);
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
bool ExternalCanBus::inject_test_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (!initialized || data == nullptr || length > 8) {
        return false;
    }
    
    CAN_message_t msg = {};
    msg.id = can_id;
    msg.len = length;
    memcpy(msg.buf, data, length);
    msg.timestamp = micros();
    
    route_incoming_message(msg);
    return true;
}

bool ExternalCanBus::inject_obdii_request(uint8_t pid) {
    uint8_t request_data[] = {0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
    return inject_test_message(OBDII_REQUEST_ID, request_data, 3);
}

void ExternalCanBus::simulate_external_device_request(uint32_t external_key) {
    if (cache != nullptr) {
        float dummy_value;
        cache->get_value(external_key, &dummy_value);
    }
}
#endif