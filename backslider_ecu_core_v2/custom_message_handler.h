// custom_message_handler.h
// Custom message protocol handler for external CAN bus
// Supports user-defined message formats and protocols

#ifndef CUSTOM_MESSAGE_HANDLER_H
#define CUSTOM_MESSAGE_HANDLER_H

#include "msg_definitions.h"
#include "external_canbus_cache.h"

#ifdef ARDUINO
    #include <Arduino.h>
    #include <map>
    #include <vector>
#else
    #include "tests/mock_arduino.h"
    #include <map>
    #include <vector>
#endif

// Custom message handler function types
typedef void (*custom_message_handler_t)(uint32_t can_id, const uint8_t* data, uint8_t length);
typedef bool (*custom_value_provider_t)(uint32_t external_key, float* value);

// Custom message configuration
struct custom_message_config_t {
    uint32_t can_id;                    // CAN ID for this message
    uint32_t external_key;              // External key for cache mapping
    uint32_t transmit_interval_ms;      // How often to send (0 = on-demand only)
    uint32_t timeout_ms;                // Timeout for received messages
    bool is_transmit;                   // True for outgoing, false for incoming
    bool cache_enabled;                 // Whether to use cache for this message
    const char* description;            // Description for debugging
};

// Custom message statistics
struct custom_message_stats_t {
    uint32_t messages_processed;        // Total messages processed
    uint32_t messages_sent;             // Messages sent
    uint32_t messages_received;         // Messages received
    uint32_t handler_calls;             // Custom handler invocations
    uint32_t cache_updates;             // Cache updates from custom messages
    uint32_t transmission_timeouts;     // Failed transmissions
    uint32_t reception_timeouts;        // Missing expected messages
    uint32_t format_errors;             // Malformed messages
};

class CustomMessageHandler {
public:
    // Constructor
    CustomMessageHandler(ExternalCanBusCache* cache);
    ~CustomMessageHandler();
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    
    // Initialize custom message handler
    bool init();
    
    // Shutdown handler
    void shutdown();
    
    // Update function - call periodically for scheduled transmissions
    void update();
    
    // =========================================================================
    // MESSAGE HANDLER REGISTRATION
    // =========================================================================
    
    // Register message handler for incoming messages
    bool register_handler(uint32_t can_id, custom_message_handler_t handler);
    bool unregister_handler(uint32_t can_id);
    
    // Register value provider for outgoing messages
    bool register_value_provider(uint32_t external_key, custom_value_provider_t provider);
    bool unregister_value_provider(uint32_t external_key);
    
    // =========================================================================
    // MESSAGE CONFIGURATION
    // =========================================================================
    
    // Configure custom message
    bool configure_message(const custom_message_config_t& config);
    bool configure_message(uint32_t can_id, uint32_t external_key, 
                          uint32_t interval_ms, bool is_transmit, const char* description);
    
    // Remove message configuration
    bool remove_message_config(uint32_t can_id);
    
    // =========================================================================
    // MESSAGE PROCESSING
    // =========================================================================
    
    // Process incoming custom message
    bool process_message(const CAN_message_t& msg);
    
    // Send custom message immediately
    bool send_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool send_float_message(uint32_t can_id, float value);
    bool send_uint32_message(uint32_t can_id, uint32_t value);
    
    // =========================================================================
    // PREDEFINED MESSAGE PROTOCOLS
    // =========================================================================
    
    // Dashboard protocol messages
    bool configure_dashboard_messages();
    bool send_dashboard_rpm(float rpm);
    bool send_dashboard_speed(float speed);
    bool send_dashboard_temperature(float temp);
    
    // Datalogger protocol messages
    bool configure_datalogger_messages();
    bool send_datalogger_data(float rpm, float tps, float map, float temp);
    
    // Display protocol messages
    bool configure_display_messages();
    bool send_boost_display(float boost_psi);
    
    // =========================================================================
    // CACHE INTEGRATION
    // =========================================================================
    
    // Update cache from custom message
    bool update_cache_from_message(uint32_t external_key, const uint8_t* data, uint8_t length);
    
    // Get value for transmission from cache
    bool get_value_for_transmission(uint32_t external_key, float* value);
    
    // =========================================================================
    // DIAGNOSTICS AND STATISTICS
    // =========================================================================
    
    // Get statistics
    const custom_message_stats_t& get_statistics() const { return stats; }
    void reset_statistics();
    
    // Status
    bool is_initialized() const { return initialized; }
    uint32_t get_handler_count() const;
    uint32_t get_configured_message_count() const;
    
    // =========================================================================
    // TESTING INTERFACE
    // =========================================================================
    
    #ifndef ARDUINO
    // Simulate custom message for testing
    bool simulate_custom_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool simulate_dashboard_request();
    bool simulate_datalogger_request();
    
    // Get internal state for testing
    bool get_message_config_for_testing(uint32_t can_id, custom_message_config_t* config);
    #endif
    
private:
    // =========================================================================
    // INTERNAL DATA
    // =========================================================================
    
    ExternalCanBusCache* cache;         // Reference to cache system
    bool initialized;
    uint32_t last_update_time;
    
    // Statistics
    custom_message_stats_t stats;
    
    // Message handlers
    std::map<uint32_t, custom_message_handler_t> message_handlers;
    
    // Value providers for outgoing messages
    std::map<uint32_t, custom_value_provider_t> value_providers;
    
    // Message configurations
    std::map<uint32_t, custom_message_config_t> message_configs;
    
    // Transmission tracking
    std::map<uint32_t, uint32_t> last_transmission_time;
    
    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================
    
    // Message processing
    void process_incoming_message(const CAN_message_t& msg);
    void process_scheduled_transmissions();
    
    // Data conversion
    float extract_float_from_message(const CAN_message_t& msg);
    uint32_t extract_uint32_from_message(const CAN_message_t& msg);
    
    // Configuration helpers
    custom_message_config_t* find_message_config(uint32_t can_id);
    bool is_transmission_due(uint32_t can_id);
    
    // Error handling
    void handle_error(const char* error_msg);
    void increment_error_count();
    
    // Debugging
    void debug_print(const char* message);
    void debug_print_custom_message(const CAN_message_t& msg);
};

// ============================================================================
// PREDEFINED CUSTOM MESSAGE PROTOCOLS
// ============================================================================

// Dashboard protocol CAN IDs
#define DASHBOARD_CAN_ID_RPM            0x100
#define DASHBOARD_CAN_ID_SPEED          0x101
#define DASHBOARD_CAN_ID_TEMPERATURE    0x102
#define DASHBOARD_CAN_ID_FUEL_LEVEL     0x103
#define DASHBOARD_CAN_ID_OIL_PRESSURE   0x104

// Datalogger protocol CAN IDs
#define DATALOGGER_CAN_ID_ENGINE_DATA   0x200
#define DATALOGGER_CAN_ID_SENSOR_DATA   0x201
#define DATALOGGER_CAN_ID_STATUS        0x202

// Display protocol CAN IDs
#define DISPLAY_CAN_ID_BOOST            0x300
#define DISPLAY_CAN_ID_EGT              0x301
#define DISPLAY_CAN_ID_AFR              0x302

// Generic protocol CAN IDs
#define GENERIC_CAN_ID_FLOAT_DATA       0x400
#define GENERIC_CAN_ID_INT_DATA         0x401
#define GENERIC_CAN_ID_STATUS_DATA      0x402

// Default transmission intervals
#define DEFAULT_DASHBOARD_INTERVAL_MS   100     // 10 Hz for smooth gauges
#define DEFAULT_DATALOGGER_INTERVAL_MS  50      // 20 Hz for high-res logging
#define DEFAULT_DISPLAY_INTERVAL_MS     200     // 5 Hz for displays

// Utility macros
#define CUSTOM_MESSAGE_CONFIG_TRANSMIT(id, key, interval, desc) \
    { .can_id = (id), .external_key = (key), .transmit_interval_ms = (interval), \
      .timeout_ms = 0, .is_transmit = true, .cache_enabled = true, .description = (desc) }

#define CUSTOM_MESSAGE_CONFIG_RECEIVE(id, key, timeout, desc) \
    { .can_id = (id), .external_key = (key), .transmit_interval_ms = 0, \
      .timeout_ms = (timeout), .is_transmit = false, .cache_enabled = true, .description = (desc) }

#endif