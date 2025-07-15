// storage_manager.h
// Message-driven storage manager for ECU key-value storage

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <stdint.h>
#include "storage_backend.h"
#include "msg_definitions.h"

// Forward declarations for static handlers
extern class StorageManager* g_storage_manager_instance;
void storage_save_float_handler(const CANMessage* msg);
void storage_load_float_handler(const CANMessage* msg);
void storage_commit_cache_handler(const CANMessage* msg);
void storage_stats_handler(const CANMessage* msg);

class StorageManager {
    friend void storage_save_float_handler(const CANMessage* msg);
    friend void storage_load_float_handler(const CANMessage* msg);
    friend void storage_commit_cache_handler(const CANMessage* msg);
    friend void storage_stats_handler(const CANMessage* msg);
    
private:
    StorageBackend* backend;
    
    // Cache for frequently accessed values
    struct CacheEntry {
        uint16_t key_hash;
        float value;
        uint32_t timestamp;
        bool dirty;
        uint8_t access_count;
    };
    
    static const int CACHE_SIZE = 20;
    CacheEntry cache[CACHE_SIZE];
    int cache_index;
    
    // Statistics
    uint32_t cache_hits;
    uint32_t cache_misses;
    uint32_t disk_writes;
    uint32_t disk_reads;
    
    // Message handlers
    void handle_save_float_message(const CANMessage* msg);
    void handle_load_float_message(const CANMessage* msg);
    void handle_commit_cache_message(const CANMessage* msg);
    void handle_stats_request_message(const CANMessage* msg);
    
    // Cache management
    bool save_to_cache(uint16_t key_hash, float value);
    bool load_from_cache(uint16_t key_hash, float* value);
    void add_to_cache(uint16_t key_hash, float value);
    int find_cache_entry(uint16_t key_hash);
    int find_oldest_cache_entry();
    void commit_dirty_entries();
    
    // Response helpers
    void send_save_response(uint16_t key_hash, bool success, uint8_t sender_id);
    void send_load_response(uint16_t key_hash, float value, bool success, uint8_t sender_id, uint8_t request_id);
    void send_error_response(uint16_t key_hash, uint8_t error_code, uint8_t sender_id);
    void send_stats_response();
    
public:
    StorageManager(StorageBackend* storage_backend);
    
    // Main interface
    bool init();
    void update();
    
    // Direct access (for testing or special cases)
    bool save_float(const char* key, float value);
    bool load_float(const char* key, float* value, float default_value = 0.0f);
    
    // Statistics
    uint32_t get_cache_hits() const { return cache_hits; }
    uint32_t get_cache_misses() const { return cache_misses; }
    uint32_t get_disk_writes() const { return disk_writes; }
    uint32_t get_disk_reads() const { return disk_reads; }
    
    // Debug/maintenance
    void print_cache_info();
    void print_storage_info();
    bool verify_integrity();
    void force_commit_cache();
};

#endif 