// external_canbus_cache.h
// Lazy-loading cache system for external CAN bus
// Automatically subscribes to internal messages when first requested

#ifndef EXTERNAL_CANBUS_CACHE_H
#define EXTERNAL_CANBUS_CACHE_H

#include "msg_definitions.h"
#include "msg_bus.h"

#ifdef ARDUINO
    #include <Arduino.h>
    #include <map>
    #include <vector>
#else
    #include "tests/mock_arduino.h"
    #include <map>
    #include <vector>
#endif

// Cache entry states
enum cache_entry_state_t {
    CACHE_STATE_EMPTY = 0,          // No data yet
    CACHE_STATE_SUBSCRIBED = 1,     // Subscribed but no data received
    CACHE_STATE_VALID = 2,          // Valid data available
    CACHE_STATE_STALE = 3,          // Data is too old
    CACHE_STATE_ERROR = 4           // Error in subscription or data
};

// Cache entry structure
struct cache_entry_t {
    // Data
    float value;                    // Cached value
    uint32_t last_update_time;      // When this value was last updated
    
    // Configuration
    uint32_t internal_msg_id;       // Internal message ID to subscribe to
    uint32_t max_age_ms;            // Maximum age before data is stale
    
    // State
    cache_entry_state_t state;      // Current state of this cache entry
    bool is_subscribed;             // Have we subscribed to the internal message?
    uint32_t subscription_time;     // When we subscribed
    uint32_t request_count;         // How many times this has been requested
    
    // Metadata
    const char* description;        // Human-readable description for debugging
};

// Cache mapping structure - defines how external keys map to internal messages
struct cache_mapping_t {
    uint32_t external_key;          // External key (OBD PID, custom ID, etc.)
    uint32_t internal_msg_id;       // Internal message ID to subscribe to
    uint32_t default_max_age_ms;    // Default freshness timeout
    const char* description;        // Description for debugging
};

// Cache statistics
struct cache_stats_t {
    uint32_t total_requests;        // Total cache requests
    uint32_t cache_hits;            // Requests with valid data
    uint32_t cache_misses;          // Requests with no/stale data
    uint32_t subscriptions_created; // Number of dynamic subscriptions
    uint32_t messages_received;     // Messages received from internal bus
    uint32_t entries_created;       // Total cache entries created
    uint32_t stale_entries;         // Entries with stale data
    uint32_t subscription_errors;   // Failed subscription attempts
};

class ExternalCanBusCache {
public:
    // Constructor
    ExternalCanBusCache();
    ~ExternalCanBusCache();
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    
    // Initialize the cache system
    bool init(uint32_t default_max_age_ms = 1000);
    
    // Shutdown and cleanup
    void shutdown();
    
    // =========================================================================
    // CACHE OPERATIONS
    // =========================================================================
    
    // Get cached value (core function - implements lazy loading)
    bool get_value(uint32_t external_key, float* value, uint32_t max_age_ms = 0);
    
    // Check if value exists and is fresh
    bool has_fresh_value(uint32_t external_key, uint32_t max_age_ms = 0);
    
    // Force refresh a cached value (triggers immediate subscription if needed)
    bool refresh_value(uint32_t external_key);
    
    // Invalidate a cached entry (mark as stale)
    void invalidate_entry(uint32_t external_key);
    
    // Clear all cached data
    void clear_all();
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Add cache mapping (defines external_key -> internal_msg_id relationship)
    bool add_mapping(const cache_mapping_t& mapping);
    bool add_mapping(uint32_t external_key, uint32_t internal_msg_id, 
                     uint32_t max_age_ms, const char* description);
    
    // Remove cache mapping
    bool remove_mapping(uint32_t external_key);
    
    // Load predefined mappings
    bool load_obdii_mappings();
    bool load_custom_mappings();
    
    // =========================================================================
    // STATISTICS AND DIAGNOSTICS
    // =========================================================================
    
    // Get cache statistics
    const cache_stats_t& get_statistics() const { return stats; }
    void reset_statistics();
    
    // Cache status
    uint32_t get_entry_count() const;
    uint32_t get_subscription_count() const;
    uint32_t get_fresh_entry_count() const;
    uint32_t get_stale_entry_count() const;
    
    // Get cache entry info (for debugging)
    bool get_entry_info(uint32_t external_key, cache_entry_t* entry_info);
    
    // =========================================================================
    // INTERNAL MESSAGE BUS INTEGRATION
    // =========================================================================
    
    // Update function - call periodically to check for stale entries
    void update();
    
    // Message handler for internal message bus (static callback)
    static void message_handler(const CANMessage* msg);
    
    // =========================================================================
    // TESTING INTERFACE
    // =========================================================================
    
    #if !defined(ARDUINO) || defined(TESTING)
    // Simulate receiving an internal message
    void simulate_internal_message(uint32_t msg_id, float value);
    
    // Get cache entry for testing
    const cache_entry_t* get_cache_entry_for_testing(uint32_t external_key);
    
    // Force subscription for testing
    bool force_subscription_for_testing(uint32_t external_key);
    
    // Debug version of get_value with detailed output
    bool debug_get_value(uint32_t external_key, float* value, uint32_t max_age_ms = 0);
    #endif
    
private:
    // =========================================================================
    // INTERNAL DATA
    // =========================================================================
    
    // Cache storage - external_key -> cache_entry
    std::map<uint32_t, cache_entry_t> cache_entries;
    
    // Mapping storage - external_key -> mapping_info
    std::map<uint32_t, cache_mapping_t> cache_mappings;
    
    // Reverse lookup - internal_msg_id -> list of external_keys
    std::map<uint32_t, std::vector<uint32_t>> subscription_map;
    
    // Configuration
    uint32_t default_max_age_ms;
    bool initialized;
    
    // Statistics
    cache_stats_t stats;
    
    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================
    
    // Lazy loading implementation
    cache_entry_t* get_or_create_entry(uint32_t external_key);
    bool subscribe_to_internal_message(uint32_t external_key, cache_entry_t& entry);
    
    // Cache maintenance
    void check_stale_entries();
    void cleanup_unused_entries();
    
    // Message handling
    void handle_internal_message(const CANMessage* msg);
    void update_cache_entry(uint32_t external_key, float value);
    
    // Utility functions
    cache_mapping_t* find_mapping(uint32_t external_key);
    bool is_entry_fresh(const cache_entry_t& entry, uint32_t max_age_ms) const;
    uint32_t get_entry_age_ms(const cache_entry_t& entry) const;
    
    // Error handling
    void handle_subscription_error(uint32_t external_key);
    void increment_error_count();
    
    // Debugging
    void debug_print(const char* message);
    void debug_print_entry(uint32_t external_key, const cache_entry_t& entry);
    
    // Static instance pointer for callback
    static ExternalCanBusCache* instance;
};

// ============================================================================
// PREDEFINED CACHE MAPPINGS
// ============================================================================

// OBD-II PID mappings (PID -> internal message)
extern const cache_mapping_t OBDII_CACHE_MAPPINGS[];
extern const uint32_t OBDII_CACHE_MAPPINGS_COUNT;

// Custom message mappings (custom ID -> internal message)
extern const cache_mapping_t CUSTOM_CACHE_MAPPINGS[];
extern const uint32_t CUSTOM_CACHE_MAPPINGS_COUNT;

// ============================================================================
// COMMON EXTERNAL KEYS
// ============================================================================

// OBD-II PIDs (Mode 01)
#define OBDII_PID_ENGINE_RPM            0x0C
#define OBDII_PID_VEHICLE_SPEED         0x0D
#define OBDII_PID_COOLANT_TEMP          0x05
#define OBDII_PID_THROTTLE_POSITION     0x11
#define OBDII_PID_INTAKE_AIR_TEMP       0x0F
#define OBDII_PID_MANIFOLD_PRESSURE     0x0B
#define OBDII_PID_ENGINE_LOAD          0x04

// Common custom message IDs
#define CUSTOM_DASHBOARD_RPM            0x1000
#define CUSTOM_DASHBOARD_SPEED          0x1001
#define CUSTOM_DASHBOARD_TEMP           0x1002
#define CUSTOM_DATALOGGER_RPM           0x2000
#define CUSTOM_DATALOGGER_TPS           0x2001
#define CUSTOM_DISPLAY_BOOST            0x3000

// Utility macros
#define CACHE_ENTRY_IS_FRESH(entry, max_age) \
    ((entry).state == CACHE_STATE_VALID && \
     (millis() - (entry).last_update_time) < (max_age))

#define CACHE_ENTRY_IS_STALE(entry, max_age) \
    ((entry).state == CACHE_STATE_VALID && \
     (millis() - (entry).last_update_time) >= (max_age))

#endif