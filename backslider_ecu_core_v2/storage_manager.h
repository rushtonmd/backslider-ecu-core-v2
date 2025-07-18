// storage_manager.h
// Message-driven storage manager - Extended CAN ID Architecture

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <stdint.h>
#include "msg_definitions.h"
#include "storage_backend.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// =============================================================================
// StorageManager Class
// =============================================================================

class StorageManager {
public:
    static const int CACHE_SIZE = 20;  // Number of cache entries
    
    // Cache entry structure
    struct CacheEntry {
        uint32_t storage_key;       // Extended CAN ID used as storage key
        float value;                // Cached value
        uint32_t timestamp;         // Last access time
        bool dirty;                 // Needs write to backend
        uint32_t access_count;      // Access frequency counter
    };
    
    // Constructor
    StorageManager(StorageBackend* storage_backend);
    
    // Initialization
    bool init();
    
    // Main update loop (call from main loop)
    void update();
    
    // Message handlers
    void handle_save_float_message(const CANMessage* msg);
    void handle_load_float_message(const CANMessage* msg);
    void handle_commit_cache_message(const CANMessage* msg);
    void handle_stats_request_message(const CANMessage* msg);
    
    // Direct access methods (for backwards compatibility)
    bool save_float(const char* key, float value);
    bool load_float(const char* key, float* value, float default_value = 0.0f);
    bool save_data(const char* key, const void* data, size_t size);
    bool load_data(const char* key, void* data, size_t size);
    
    // Extended CAN ID methods
    bool save_float(uint32_t storage_key, float value);
    bool load_float(uint32_t storage_key, float* value, float default_value = 0.0f);
    bool save_data(uint32_t storage_key, const void* data, size_t size);
    bool load_data(uint32_t storage_key, void* data, size_t size);
    
    // Debug and maintenance methods
    void print_cache_info();
    void print_storage_info();
    bool verify_integrity();
    void force_commit_cache();
    
    // Statistics
    uint32_t get_cache_hits() const { return cache_hits; }
    uint32_t get_cache_misses() const { return cache_misses; }
    uint32_t get_disk_writes() const { return disk_writes; }
    uint32_t get_disk_reads() const { return disk_reads; }
    
private:
    // Backend storage
    StorageBackend* backend;
    
    // Cache management
    CacheEntry cache[CACHE_SIZE];
    int cache_index;  // Current cache index for round-robin
    
    // Statistics
    uint32_t cache_hits;
    uint32_t cache_misses;
    uint32_t disk_writes;
    uint32_t disk_reads;
    
    // Cache management methods
    bool save_to_cache(uint32_t storage_key, float value);
    bool load_from_cache(uint32_t storage_key, float* value);
    void add_to_cache(uint32_t storage_key, float value);
    int find_cache_entry(uint32_t storage_key);
    int find_oldest_cache_entry();
    void commit_dirty_entries();
    
    // Response helpers
    void send_save_response(uint32_t storage_key, bool success);
    void send_load_response(uint32_t storage_key, float value, bool success);
    void send_error_response(uint32_t storage_key, uint8_t error_code);
    void send_stats_response();
    
    // String to CAN ID conversion
    uint32_t convert_string_to_extended_can_id(const char* key);
};

// =============================================================================
// Global Storage Manager Instance
// =============================================================================

extern StorageManager* g_storage_manager_instance;

// =============================================================================
// Message Handler Functions
// =============================================================================

void storage_save_float_handler(const CANMessage* msg);
void storage_load_float_handler(const CANMessage* msg);
void storage_commit_cache_handler(const CANMessage* msg);
void storage_stats_handler(const CANMessage* msg);

// =============================================================================
// Convenience Macros
// =============================================================================

// Save a float value using extended CAN ID
#define STORAGE_SAVE_FLOAT(can_id, value) do { \
    CANMessage msg; \
    MSG_PACK_STORAGE_SAVE_FLOAT(&msg, can_id, value); \
    g_message_bus.publish(MSG_STORAGE_SAVE, &msg, sizeof(msg)); \
} while(0)

// Load a float value using extended CAN ID
#define STORAGE_LOAD_FLOAT(can_id, default_val) do { \
    CANMessage msg; \
    MSG_PACK_STORAGE_LOAD_FLOAT(&msg, can_id, default_val); \
    g_message_bus.publish(MSG_STORAGE_LOAD, &msg, sizeof(msg)); \
} while(0)

// Common storage operations for map cells
#define STORAGE_SAVE_FUEL_MAP_CELL(row, col, value) \
    STORAGE_SAVE_FLOAT(MSG_FUEL_MAP_CELL(row, col), value)

#define STORAGE_LOAD_FUEL_MAP_CELL(row, col, default_val) \
    STORAGE_LOAD_FLOAT(MSG_FUEL_MAP_CELL(row, col), default_val)

#define STORAGE_SAVE_IGNITION_MAP_CELL(row, col, value) \
    STORAGE_SAVE_FLOAT(MSG_IGNITION_MAP_CELL(row, col), value)

#define STORAGE_LOAD_IGNITION_MAP_CELL(row, col, default_val) \
    STORAGE_LOAD_FLOAT(MSG_IGNITION_MAP_CELL(row, col), default_val)

#define STORAGE_SAVE_BOOST_MAP_CELL(row, col, value) \
    STORAGE_SAVE_FLOAT(MSG_BOOST_MAP_CELL(row, col), value)

#define STORAGE_LOAD_BOOST_MAP_CELL(row, col, default_val) \
    STORAGE_LOAD_FLOAT(MSG_BOOST_MAP_CELL(row, col), default_val)

// Configuration parameter storage
#define STORAGE_SAVE_CONFIG(config_id, value) \
    STORAGE_SAVE_FLOAT(config_id, value)

#define STORAGE_LOAD_CONFIG(config_id, default_val) \
    STORAGE_LOAD_FLOAT(config_id, default_val)

#endif 