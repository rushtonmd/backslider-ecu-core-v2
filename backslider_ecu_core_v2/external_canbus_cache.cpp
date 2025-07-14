// external_canbus_cache.cpp
// Implementation of lazy-loading cache system for external CAN bus

#include "external_canbus_cache.h"

// Static instance for callback
ExternalCanBusCache* ExternalCanBusCache::instance = nullptr;

// ============================================================================
// PREDEFINED CACHE MAPPINGS
// ============================================================================

// OBD-II PID to internal message mappings
const cache_mapping_t OBDII_CACHE_MAPPINGS[] = {
    {OBDII_PID_ENGINE_RPM,        MSG_ENGINE_RPM,         100,  "OBD Engine RPM"},
    {OBDII_PID_VEHICLE_SPEED,     MSG_VEHICLE_SPEED,      200,  "OBD Vehicle Speed"},
    {OBDII_PID_COOLANT_TEMP,      MSG_COOLANT_TEMP,       1000, "OBD Coolant Temperature"},
    {OBDII_PID_THROTTLE_POSITION, MSG_THROTTLE_POSITION,  100,  "OBD Throttle Position"},
    {OBDII_PID_INTAKE_AIR_TEMP,   MSG_AIR_INTAKE_TEMP,    1000, "OBD Intake Air Temperature"},
    {OBDII_PID_MANIFOLD_PRESSURE, MSG_MANIFOLD_PRESSURE,  100,  "OBD Manifold Pressure"},
    // Add more OBD-II PIDs as needed
};

const uint32_t OBDII_CACHE_MAPPINGS_COUNT = sizeof(OBDII_CACHE_MAPPINGS) / sizeof(cache_mapping_t);

// Custom message to internal message mappings
const cache_mapping_t CUSTOM_CACHE_MAPPINGS[] = {
    {CUSTOM_DASHBOARD_RPM,    MSG_ENGINE_RPM,         50,   "Dashboard Tachometer"},
    {CUSTOM_DASHBOARD_SPEED,  MSG_VEHICLE_SPEED,      200,  "Dashboard Speedometer"},
    {CUSTOM_DASHBOARD_TEMP,   MSG_COOLANT_TEMP,       500,  "Dashboard Temperature"},
    {CUSTOM_DATALOGGER_RPM,   MSG_ENGINE_RPM,         20,   "Datalogger RPM"},
    {CUSTOM_DATALOGGER_TPS,   MSG_THROTTLE_POSITION,  50,   "Datalogger TPS"},
    {CUSTOM_DISPLAY_BOOST,    MSG_MANIFOLD_PRESSURE,  100,  "Boost Display"},
    // Add more custom mappings as needed
};

const uint32_t CUSTOM_CACHE_MAPPINGS_COUNT = sizeof(CUSTOM_CACHE_MAPPINGS) / sizeof(cache_mapping_t);

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

ExternalCanBusCache::ExternalCanBusCache() :
    default_max_age_ms(1000),
    initialized(false)
{
    // Initialize statistics
    reset_statistics();
    
    // Set static instance for callback
    instance = this;
}

ExternalCanBusCache::~ExternalCanBusCache() {
    shutdown();
}

bool ExternalCanBusCache::init(uint32_t default_max_age_ms) {
    if (initialized) {
        debug_print("Cache: Already initialized");
        return true;
    }
    
    this->default_max_age_ms = default_max_age_ms;
    
    // Clear all data structures
    cache_entries.clear();
    cache_mappings.clear();
    subscription_map.clear();
    
    // Reset statistics
    reset_statistics();
    
    initialized = true;  // Set initialized BEFORE loading mappings
    
    // Load predefined mappings (now that we're initialized)
    bool obdii_result = load_obdii_mappings();
    bool custom_result = load_custom_mappings();
    
    if (!obdii_result) {
        debug_print("Cache: Warning - Failed to load OBD-II mappings");
    }
    
    if (!custom_result) {
        debug_print("Cache: Warning - Failed to load custom mappings");
    }
    
    debug_print("Cache: Initialization complete");
    
    return true;  // Return true even if mapping loading fails - core functionality works
}

void ExternalCanBusCache::shutdown() {
    if (!initialized) {
        return;
    }
    
    // Clear all data
    cache_entries.clear();
    cache_mappings.clear();
    subscription_map.clear();
    
    initialized = false;
    instance = nullptr;
    
    debug_print("Cache: Shutdown complete");
}

// ============================================================================
// CORE CACHE OPERATIONS (LAZY LOADING IMPLEMENTATION)
// ============================================================================

bool ExternalCanBusCache::get_value(uint32_t external_key, float* value, uint32_t max_age_ms) {
    if (!initialized || value == nullptr) {
        return false;
    }
    
    stats.total_requests++;
    
    // Use default max age if not specified
    if (max_age_ms == 0) {
        max_age_ms = default_max_age_ms;
    }
    
    // Get or create cache entry (implements lazy loading)
    cache_entry_t* entry = get_or_create_entry(external_key);
    if (entry == nullptr) {
        stats.cache_misses++;
        return false;
    }
    
    // Increment request count
    entry->request_count++;
    
    // Check if we have valid data (not just fresh)
    if (entry->state == CACHE_STATE_VALID) {
        // Check freshness
        uint32_t age_ms = get_entry_age_ms(*entry);
        if (age_ms < max_age_ms) {
            *value = entry->value;
            stats.cache_hits++;
            return true;
        } else {
            // Data is stale
            entry->state = CACHE_STATE_STALE;
            stats.stale_entries++;
        }
    }
    
    // Data is missing, stale, or invalid
    stats.cache_misses++;
    return false;
}

bool ExternalCanBusCache::has_fresh_value(uint32_t external_key, uint32_t max_age_ms) {
    if (!initialized) {
        return false;
    }
    
    auto it = cache_entries.find(external_key);
    if (it == cache_entries.end()) {
        return false;
    }
    
    if (max_age_ms == 0) {
        max_age_ms = default_max_age_ms;
    }
    
    return is_entry_fresh(it->second, max_age_ms);
}

bool ExternalCanBusCache::refresh_value(uint32_t external_key) {
    if (!initialized) {
        return false;
    }
    
    // Invalidate current entry to force refresh
    invalidate_entry(external_key);
    
    // Try to get value (will trigger subscription if needed)
    float dummy_value;
    return get_value(external_key, &dummy_value, 0);
}

void ExternalCanBusCache::invalidate_entry(uint32_t external_key) {
    auto it = cache_entries.find(external_key);
    if (it != cache_entries.end()) {
        it->second.state = CACHE_STATE_EMPTY;
        it->second.last_update_time = 0;
    }
}

void ExternalCanBusCache::clear_all() {
    cache_entries.clear();
    reset_statistics();
    debug_print("Cache: All entries cleared");
}

// ============================================================================
// LAZY LOADING IMPLEMENTATION
// ============================================================================

cache_entry_t* ExternalCanBusCache::get_or_create_entry(uint32_t external_key) {
    // Check if entry already exists
    auto it = cache_entries.find(external_key);
    if (it != cache_entries.end()) {
        return &it->second;
    }
    
    // Find mapping for this external key
    cache_mapping_t* mapping = find_mapping(external_key);
    if (mapping == nullptr) {
        debug_print("Cache: No mapping found for external key");
        return nullptr;
    }
    
    // Create new cache entry
    cache_entry_t new_entry = {};
    new_entry.value = 0.0f;
    new_entry.last_update_time = 0;
    new_entry.internal_msg_id = mapping->internal_msg_id;
    new_entry.max_age_ms = mapping->default_max_age_ms;
    new_entry.state = CACHE_STATE_EMPTY;
    new_entry.is_subscribed = false;
    new_entry.subscription_time = 0;
    new_entry.request_count = 0;
    new_entry.description = mapping->description;
    
    // Insert into cache
    auto result = cache_entries.insert(std::make_pair(external_key, new_entry));
    if (!result.second) {
        debug_print("Cache: Failed to create cache entry");
        return nullptr;
    }
    
    stats.entries_created++;
    
    // Subscribe to internal message
    if (!subscribe_to_internal_message(external_key, result.first->second)) {
        // Subscription failed, but keep the entry for retry
        handle_subscription_error(external_key);
    }
    
    debug_print("Cache: Created new cache entry");
    return &result.first->second;
}

bool ExternalCanBusCache::subscribe_to_internal_message(uint32_t external_key, cache_entry_t& entry) {
    if (entry.is_subscribed) {
        return true;  // Already subscribed
    }
    
    // Subscribe to internal message bus
    bool success = g_message_bus.subscribe(entry.internal_msg_id, message_handler);
    if (success) {
        entry.is_subscribed = true;
        entry.subscription_time = millis();
        entry.state = CACHE_STATE_SUBSCRIBED;
        
        // Add to subscription map for reverse lookup
        subscription_map[entry.internal_msg_id].push_back(external_key);
        
        stats.subscriptions_created++;
        
        char debug_msg[100];
        snprintf(debug_msg, sizeof(debug_msg), 
                "Cache: Subscribed to internal message 0x%03X for external key 0x%08X", 
                entry.internal_msg_id, external_key);
        debug_print(debug_msg);
        
        return true;
    } else {
        entry.state = CACHE_STATE_ERROR;
        stats.subscription_errors++;
        debug_print("Cache: Failed to subscribe to internal message");
        return false;
    }
}

// ============================================================================
// MESSAGE HANDLING FROM INTERNAL BUS
// ============================================================================

void ExternalCanBusCache::message_handler(const CANMessage* msg) {
    if (instance != nullptr) {
        instance->handle_internal_message(msg);
    }
}

void ExternalCanBusCache::handle_internal_message(const CANMessage* msg) {
    if (msg == nullptr) {
        return;
    }
    
    stats.messages_received++;
    
    // Find all external keys that map to this internal message ID
    auto it = subscription_map.find(msg->id);
    if (it == subscription_map.end()) {
        return;  // No external keys interested in this message
    }
    
    // Extract float value from message
    float value = 0.0f;
    if (msg->len == sizeof(float)) {
        memcpy(&value, msg->buf, sizeof(float));
    } else {
        // Handle other data types if needed
        debug_print("Cache: Unsupported message data type");
        return;
    }
    
    // Update all cache entries that map to this internal message
    for (uint32_t external_key : it->second) {
        update_cache_entry(external_key, value);
    }
}

void ExternalCanBusCache::update_cache_entry(uint32_t external_key, float value) {
    auto it = cache_entries.find(external_key);
    if (it == cache_entries.end()) {
        return;
    }
    
    cache_entry_t& entry = it->second;
    entry.value = value;
    entry.last_update_time = millis();
    entry.state = CACHE_STATE_VALID;
    
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), 
            "Cache: Updated external key 0x%08X with value %.2f", 
            external_key, value);
    debug_print(debug_msg);
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================

bool ExternalCanBusCache::add_mapping(const cache_mapping_t& mapping) {
    // Allow adding mappings even during initialization
    cache_mappings[mapping.external_key] = mapping;
    
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), 
            "Cache: Added mapping 0x%08X -> 0x%03X (%s)", 
            mapping.external_key, mapping.internal_msg_id, mapping.description);
    debug_print(debug_msg);
    
    return true;
}

bool ExternalCanBusCache::add_mapping(uint32_t external_key, uint32_t internal_msg_id, 
                                      uint32_t max_age_ms, const char* description) {
    cache_mapping_t mapping = {
        .external_key = external_key,
        .internal_msg_id = internal_msg_id,
        .default_max_age_ms = max_age_ms,
        .description = description
    };
    
    return add_mapping(mapping);
}

bool ExternalCanBusCache::remove_mapping(uint32_t external_key) {
    if (!initialized) {
        return false;
    }
    
    // Remove from mappings
    auto mapping_it = cache_mappings.find(external_key);
    if (mapping_it != cache_mappings.end()) {
        cache_mappings.erase(mapping_it);
    }
    
    // Remove cache entry if it exists
    auto entry_it = cache_entries.find(external_key);
    if (entry_it != cache_entries.end()) {
        cache_entries.erase(entry_it);
    }
    
    return true;
}

bool ExternalCanBusCache::load_obdii_mappings() {
    // Allow loading even during initialization
    for (uint32_t i = 0; i < OBDII_CACHE_MAPPINGS_COUNT; i++) {
        if (!add_mapping(OBDII_CACHE_MAPPINGS[i])) {
            return false;
        }
    }
    
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), 
            "Cache: Loaded %d OBD-II mappings", OBDII_CACHE_MAPPINGS_COUNT);
    debug_print(debug_msg);
    
    return true;
}

bool ExternalCanBusCache::load_custom_mappings() {
    // Allow loading even during initialization
    for (uint32_t i = 0; i < CUSTOM_CACHE_MAPPINGS_COUNT; i++) {
        if (!add_mapping(CUSTOM_CACHE_MAPPINGS[i])) {
            return false;
        }
    }
    
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), 
            "Cache: Loaded %d custom mappings", CUSTOM_CACHE_MAPPINGS_COUNT);
    debug_print(debug_msg);
    
    return true;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

cache_mapping_t* ExternalCanBusCache::find_mapping(uint32_t external_key) {
    auto it = cache_mappings.find(external_key);
    if (it != cache_mappings.end()) {
        return &it->second;
    }
    return nullptr;
}

bool ExternalCanBusCache::is_entry_fresh(const cache_entry_t& entry, uint32_t max_age_ms) const {
    return (entry.state == CACHE_STATE_VALID && 
            get_entry_age_ms(entry) < max_age_ms);
}

uint32_t ExternalCanBusCache::get_entry_age_ms(const cache_entry_t& entry) const {
    if (entry.last_update_time == 0) {
        return UINT32_MAX;  // Never updated
    }
    uint32_t current_time = millis();
    if (current_time >= entry.last_update_time) {
        return current_time - entry.last_update_time;
    } else {
        // Handle time wraparound
        return 0;
    }
}

// ============================================================================
// STATISTICS AND DIAGNOSTICS
// ============================================================================

void ExternalCanBusCache::reset_statistics() {
    stats = {};  // Zero-initialize all fields
}

uint32_t ExternalCanBusCache::get_entry_count() const {
    return cache_entries.size();
}

uint32_t ExternalCanBusCache::get_subscription_count() const {
    return stats.subscriptions_created;
}

uint32_t ExternalCanBusCache::get_fresh_entry_count() const {
    uint32_t count = 0;
    for (const auto& pair : cache_entries) {
        if (is_entry_fresh(pair.second, default_max_age_ms)) {
            count++;
        }
    }
    return count;
}

uint32_t ExternalCanBusCache::get_stale_entry_count() const {
    uint32_t count = 0;
    for (const auto& pair : cache_entries) {
        if (pair.second.state == CACHE_STATE_STALE) {
            count++;
        }
    }
    return count;
}

void ExternalCanBusCache::update() {
    if (!initialized) {
        return;
    }
    
    // Check for stale entries
    check_stale_entries();
    
    // Cleanup unused entries (could be done periodically)
    // cleanup_unused_entries();
}

void ExternalCanBusCache::check_stale_entries() {
    for (auto& pair : cache_entries) {
        cache_entry_t& entry = pair.second;
        if (entry.state == CACHE_STATE_VALID && 
            !is_entry_fresh(entry, entry.max_age_ms)) {
            entry.state = CACHE_STATE_STALE;
        }
    }
}

// ============================================================================
// ERROR HANDLING AND DEBUGGING
// ============================================================================

void ExternalCanBusCache::handle_subscription_error(uint32_t external_key) {
    debug_print("Cache: Subscription error handled");
}

void ExternalCanBusCache::increment_error_count() {
    stats.subscription_errors++;
}

void ExternalCanBusCache::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void ExternalCanBusCache::debug_print_entry(uint32_t external_key, const cache_entry_t& entry) {
    #ifdef ARDUINO
    Serial.print("Cache Entry 0x");
    Serial.print(external_key, HEX);
    Serial.print(": value=");
    Serial.print(entry.value);
    Serial.print(" state=");
    Serial.print(entry.state);
    Serial.print(" age=");
    Serial.print(get_entry_age_ms(entry));
    Serial.println("ms");
    #else
    printf("Cache Entry 0x%08X: value=%.2f state=%d age=%dms\n", 
           external_key, entry.value, entry.state, get_entry_age_ms(entry));
    #endif
}

// ============================================================================
// TESTING INTERFACE
// ============================================================================

#ifndef ARDUINO
void ExternalCanBusCache::simulate_internal_message(uint32_t msg_id, float value) {
    CANMessage msg = {};
    msg.id = msg_id;
    msg.len = sizeof(float);
    memcpy(msg.buf, &value, sizeof(float));
    msg.timestamp = micros();
    
    handle_internal_message(&msg);
}

const cache_entry_t* ExternalCanBusCache::get_cache_entry_for_testing(uint32_t external_key) {
    auto it = cache_entries.find(external_key);
    if (it != cache_entries.end()) {
        return &it->second;
    }
    return nullptr;
}

bool ExternalCanBusCache::force_subscription_for_testing(uint32_t external_key) {
    cache_entry_t* entry = get_or_create_entry(external_key);
    return entry != nullptr && entry->is_subscribed;
}

// Debug version of get_value for testing
bool ExternalCanBusCache::debug_get_value(uint32_t external_key, float* value, uint32_t max_age_ms) {
    printf("DEBUG get_value: external_key=0x%08X, max_age_ms=%d\n", external_key, max_age_ms);
    
    if (!initialized || value == nullptr) {
        printf("DEBUG: Not initialized or null value pointer\n");
        return false;
    }
    
    if (max_age_ms == 0) {
        max_age_ms = default_max_age_ms;
    }
    printf("DEBUG: Using max_age_ms=%d\n", max_age_ms);
    
    cache_entry_t* entry = get_or_create_entry(external_key);
    if (entry == nullptr) {
        printf("DEBUG: No cache entry found/created\n");
        return false;
    }
    
    printf("DEBUG: Cache entry state=%d, value=%f, last_update_time=%d\n", 
           entry->state, entry->value, entry->last_update_time);
    
    if (entry->state == CACHE_STATE_VALID) {
        uint32_t age_ms = get_entry_age_ms(*entry);
        printf("DEBUG: Entry age=%d ms, max_age=%d ms\n", age_ms, max_age_ms);
        
        if (age_ms < max_age_ms) {
            *value = entry->value;
            printf("DEBUG: Returning value=%f\n", *value);
            return true;
        } else {
            printf("DEBUG: Data is stale (age=%d >= max_age=%d)\n", age_ms, max_age_ms);
        }
    } else {
        printf("DEBUG: Entry state is not VALID (state=%d)\n", entry->state);
    }
    
    return false;
}
#endif