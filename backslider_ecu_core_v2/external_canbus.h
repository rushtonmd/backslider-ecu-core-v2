// external_canbus.h
// External CAN bus interface for communicating with external devices
// Provides OBD-II support and custom message handling with lazy-loading cache

#ifndef EXTERNAL_CANBUS_H
#define EXTERNAL_CANBUS_H

#include "msg_definitions.h"
#include "external_canbus_cache.h"

#ifdef ARDUINO
    #include <Arduino.h>
    #include <FlexCAN_T4.h>
#else
    #include "tests/mock_arduino.h"
#endif

// Forward declarations
class OBDIIHandler;
class CustomMessageHandler;

// External CAN bus statistics
struct external_canbus_stats_t {
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t obdii_requests;
    uint32_t custom_messages;
    uint32_t parameter_messages;
    uint32_t cache_hits;
    uint32_t cache_misses;
    uint32_t subscription_count;
    uint32_t errors;
};

// External CAN bus configuration
struct external_canbus_config_t {
    uint32_t baudrate;
    bool enable_obdii;
    bool enable_custom_messages;
    uint8_t can_bus_number;  // 1 for CAN1, 2 for CAN2, etc.
    uint32_t cache_default_max_age_ms;
};

class ExternalCanBus {
public:
    // Constructor
    ExternalCanBus();
    ~ExternalCanBus();
    
    // =========================================================================
    // INITIALIZATION AND CONFIGURATION
    // =========================================================================
    
    // Initialize the external CAN bus
    bool init(const external_canbus_config_t& config);
    bool init(uint32_t baudrate = 500000, bool enable_obdii = true);
    
    // Update function - call from main loop
    void update();
    
    // Shutdown and cleanup
    void shutdown();
    
    // =========================================================================
    // OBD-II INTERFACE
    // =========================================================================
    
    // Enable/disable OBD-II functionality
    void enable_obdii(bool enable);
    bool is_obdii_enabled() const { return obdii_enabled; }
    
    // Add custom OBD-II PID support
    typedef bool (*obdii_response_handler_t)(uint8_t pid, float* value);
    bool add_custom_obdii_pid(uint8_t pid, obdii_response_handler_t handler);
    
    // Get OBD-II response for a PID (uses cache)
    bool get_obdii_value(uint8_t pid, float* value);
    
    // =========================================================================
    // CUSTOM MESSAGE INTERFACE
    // =========================================================================
    
    // Custom message handler function type
    typedef void (*custom_message_handler_t)(uint32_t can_id, const uint8_t* data, uint8_t length);
    
    // Register custom message handler
    bool register_custom_handler(uint32_t can_id, custom_message_handler_t handler);
    bool unregister_custom_handler(uint32_t can_id);
    
    // Send custom message
    bool send_custom_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool send_custom_float(uint32_t can_id, float value);
    bool send_custom_uint32(uint32_t can_id, uint32_t value);
    
    // Get value from custom message cache
    bool get_custom_value(uint32_t external_key, float* value);
    
    // =========================================================================
    // CACHE INTERFACE
    // =========================================================================
    
    // Direct cache access
    bool get_cached_value(uint32_t external_key, float* value, uint32_t max_age_ms = 0);
    
    // Cache statistics
    uint32_t get_cache_size() const;
    uint32_t get_subscription_count() const;
    void clear_cache();
    
    // =========================================================================
    // DIAGNOSTICS AND STATISTICS
    // =========================================================================
    
    // Get statistics
    const external_canbus_stats_t& get_statistics() const { return stats; }
    void reset_statistics();
    
    // Status checks
    bool is_initialized() const { return initialized; }
    bool is_can_bus_active() const;
    uint32_t get_last_message_time() const { return last_message_time; }
    
    // Error handling
    uint32_t get_error_count() const { return stats.errors; }
    void clear_errors();
    
    // =========================================================================
    // TESTING INTERFACE
    // =========================================================================
    
    #if !defined(ARDUINO) || defined(TESTING)
    // Mock functions for testing
    bool inject_test_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool inject_obdii_request(uint8_t pid);
    void simulate_external_device_request(uint32_t external_key);
    MockFlexCAN* get_mock_can() { return &mock_can; }
    #endif
    
private:
    // =========================================================================
    // HARDWARE INTERFACE
    // =========================================================================
    
    #ifdef ARDUINO
    void* can_bus;  // Generic pointer to avoid template complexity in header
    CAN_message_t rx_msg;
    CAN_message_t tx_msg;
    #else
    MockFlexCAN mock_can;
    CAN_message_t rx_msg;
    CAN_message_t tx_msg;
    #endif
    
    // =========================================================================
    // CONFIGURATION AND STATE
    // =========================================================================
    
    external_canbus_config_t config;
    external_canbus_stats_t stats;
    
    bool initialized;
    bool obdii_enabled;
    bool custom_messages_enabled;
    uint32_t last_message_time;
    uint32_t last_update_time;
    
    // =========================================================================
    // SUBSYSTEM MODULES
    // =========================================================================
    
    ExternalCanBusCache* cache;
    OBDIIHandler* obdii_handler;
    CustomMessageHandler* custom_handler;
    
    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================
    
    // CAN bus operations
    bool setup_can_bus();
    void process_incoming_messages();
    void process_outgoing_messages();
    bool send_can_message(const CAN_message_t& msg);
    
    // Message routing
    void route_incoming_message(const CAN_message_t& msg);
    bool is_obdii_message(const CAN_message_t& msg);
    bool is_custom_message(const CAN_message_t& msg);
    bool is_parameter_message(const CAN_message_t& msg);
    void route_parameter_message(const CAN_message_t& msg);
    
    // Error handling
    void handle_error(const char* error_msg);
    void increment_error_count();
    
    // Diagnostics
    void update_statistics();
    void debug_print(const char* message);
    void debug_print_message(const CAN_message_t& msg, const char* prefix);
};

// Global external CAN bus instance
extern ExternalCanBus g_external_canbus;

// Convenience macros
#define SEND_CUSTOM_FLOAT(id, val) g_external_canbus.send_custom_float(id, val)
#define SEND_CUSTOM_UINT32(id, val) g_external_canbus.send_custom_uint32(id, val)
#define GET_CACHED_VALUE(key, val) g_external_canbus.get_cached_value(key, val)
#define GET_OBDII_VALUE(pid, val) g_external_canbus.get_obdii_value(pid, val)

// Default configuration
static const external_canbus_config_t DEFAULT_EXTERNAL_CANBUS_CONFIG = {
    .baudrate = 500000,
    .enable_obdii = true,
    .enable_custom_messages = true,
    .can_bus_number = 1,
    .cache_default_max_age_ms = 1000
};

#endif