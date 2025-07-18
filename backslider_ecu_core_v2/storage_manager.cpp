// storage_manager.cpp
// Implementation of message-driven storage manager

#ifndef ARDUINO
#include "../tests/mock_arduino.h"
extern MockSerial Serial;
#endif

#include "storage_manager.h"
#include "spi_flash_storage_backend.h"
#include "msg_bus.h"
#include "msg_definitions.h"  // For CRC16 function

#ifdef ARDUINO
#include <Arduino.h>
#else
// Mock millis for testing
extern uint32_t mock_millis_time;
#define millis() mock_millis_time
#endif

// =============================================================================
// StorageManager Implementation
// =============================================================================

StorageManager::StorageManager(StorageBackend* storage_backend) 
    : backend(storage_backend), cache_index(0), cache_hits(0), cache_misses(0), 
      disk_writes(0), disk_reads(0) {
    
    // Initialize cache
    for (int i = 0; i < CACHE_SIZE; i++) {
        cache[i].key_hash = 0;
        cache[i].value = 0.0f;
        cache[i].timestamp = 0;
        cache[i].dirty = false;
        cache[i].access_count = 0;
    }
}

bool StorageManager::init() {
    if (!backend || !backend->begin()) {
        return false;
    }
    
    // Subscribe to storage messages
    // Note: Message handlers will be called statically, so we need global access
    g_storage_manager_instance = this;
    g_message_bus.subscribe(MSG_STORAGE_SAVE_FLOAT, storage_save_float_handler);
    g_message_bus.subscribe(MSG_STORAGE_LOAD_FLOAT, storage_load_float_handler);
    g_message_bus.subscribe(MSG_STORAGE_COMMIT_CACHE, storage_commit_cache_handler);
    g_message_bus.subscribe(MSG_STORAGE_STATS, storage_stats_handler);
    
    return true;
}

void StorageManager::update() {
    static uint32_t last_commit = 0;
    
    // Periodic commit of dirty cache entries (every 10 seconds)
    if (millis() - last_commit >= 10000) {
        commit_dirty_entries();
        last_commit = millis();
    }
}

// =============================================================================
// Message Handlers
// =============================================================================

void StorageManager::handle_save_float_message(const CANMessage* msg) {
    if (msg->len != sizeof(storage_save_float_msg_t)) {
        return;  // Invalid message size
    }
    
    storage_save_float_msg_t* save_msg = MSG_UNPACK_STORAGE_SAVE_FLOAT(msg);
    
    bool success = false;
    
    // Save to cache
    if (save_to_cache(save_msg->key_hash, save_msg->value)) {
        success = true;
        
        // Immediate write to backend if high priority
        if (save_msg->priority == 1) {
            if (backend->writeData(save_msg->key_hash, &save_msg->value, sizeof(float))) {
                disk_writes++;
            } else {
                success = false;
            }
        }
    }
    
    // Send response
    send_save_response(save_msg->key_hash, success, save_msg->sender_id);
}

void StorageManager::handle_load_float_message(const CANMessage* msg) {
    if (msg->len != sizeof(storage_load_float_msg_t)) {
        return;  // Invalid message size
    }
    
    storage_load_float_msg_t* load_msg = MSG_UNPACK_STORAGE_LOAD_FLOAT(msg);
    
    float value = load_msg->default_value;
    bool success = false;
    
    // Check cache first
    if (load_from_cache(load_msg->key_hash, &value)) {
        success = true;
        cache_hits++;
    } else {
        // Load from backend
        if (backend->readData(load_msg->key_hash, &value, sizeof(float))) {
            success = true;
            cache_misses++;
            disk_reads++;
            
            // Add to cache for future access
            add_to_cache(load_msg->key_hash, value);
        } else {
            cache_misses++;
            // Keep default value if key not found
        }
    }
    
    // Send response
    send_load_response(load_msg->key_hash, value, success, load_msg->sender_id, load_msg->request_id);
}

void StorageManager::handle_commit_cache_message(const CANMessage* msg) {
    commit_dirty_entries();
}

void StorageManager::handle_stats_request_message(const CANMessage* msg) {
    send_stats_response();
}

// =============================================================================
// Cache Management
// =============================================================================

bool StorageManager::save_to_cache(uint16_t key_hash, float value) {
    if (key_hash == 0) return false;
    
    // Check if key already exists in cache
    int existing_index = find_cache_entry(key_hash);
    if (existing_index >= 0) {
        // Update existing entry
        cache[existing_index].value = value;
        cache[existing_index].timestamp = millis();
        cache[existing_index].dirty = true;
        cache[existing_index].access_count++;
        return true;
    }
    
    // Add new entry
    add_to_cache(key_hash, value);
    cache[cache_index].dirty = true;
    
    return true;
}

bool StorageManager::load_from_cache(uint16_t key_hash, float* value) {
    if (key_hash == 0 || !value) return false;
    
    int index = find_cache_entry(key_hash);
    if (index >= 0) {
        *value = cache[index].value;
        cache[index].timestamp = millis();
        cache[index].access_count++;
        return true;
    }
    
    return false;
}

void StorageManager::add_to_cache(uint16_t key_hash, float value) {
    if (key_hash == 0) return;
    
    // Find oldest entry to replace
    int oldest_index = find_oldest_cache_entry();
    
    // Clear the entry
    cache[oldest_index].key_hash = 0;
    cache[oldest_index].value = 0.0f;
    cache[oldest_index].timestamp = millis();
    cache[oldest_index].dirty = false;
    cache[oldest_index].access_count = 1;
    
    // Add new data
    cache[oldest_index].key_hash = key_hash;
    cache[oldest_index].value = value;
    
    cache_index = oldest_index;
}

int StorageManager::find_cache_entry(uint16_t key_hash) {
    if (key_hash == 0) return -1;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (cache[i].key_hash != 0 && cache[i].key_hash == key_hash) {
            return i;
        }
    }
    
    return -1;
}

int StorageManager::find_oldest_cache_entry() {
    int oldest = 0;
    uint32_t oldest_time = cache[0].timestamp;
    
    for (int i = 1; i < CACHE_SIZE; i++) {
        if (cache[i].key_hash == 0) {
            // Empty slot - use this
            return i;
        }
        
        if (cache[i].timestamp < oldest_time) {
            oldest = i;
            oldest_time = cache[i].timestamp;
        }
    }
    
    return oldest;
}

void StorageManager::commit_dirty_entries() {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (cache[i].dirty && cache[i].key_hash != 0) {
            if (backend->writeData(cache[i].key_hash, &cache[i].value, sizeof(float))) {
                cache[i].dirty = false;
                disk_writes++;
            }
        }
    }
}

// =============================================================================
// Response Helpers
// =============================================================================

void StorageManager::send_save_response(uint16_t key_hash, bool success, uint8_t sender_id) {
    storage_save_response_msg_t response;
    response.key_hash = key_hash;
    response.success = success ? 1 : 0;
    response.sender_id = sender_id;
    
    g_message_bus.publish(MSG_STORAGE_SAVE_RESPONSE, &response, sizeof(response));
}

void StorageManager::send_load_response(uint16_t key_hash, float value, bool success, uint8_t sender_id, uint8_t request_id) {
    storage_load_response_msg_t response;
    response.key_hash = key_hash;
    response.value = value;
    response.success = success ? 1 : 0;
    response.request_id = request_id;
    
    g_message_bus.publish(MSG_STORAGE_LOAD_RESPONSE, &response, sizeof(response));
}

void StorageManager::send_error_response(uint16_t key_hash, uint8_t error_code, uint8_t sender_id) {
    storage_error_msg_t response;
    response.key_hash = key_hash;
    response.error_code = error_code;
    response.sender_id = sender_id;
    
    g_message_bus.publish(MSG_STORAGE_ERROR, &response, sizeof(response));
}

void StorageManager::send_stats_response() {
    storage_stats_msg_t stats;
    stats.cache_hits = cache_hits;
    stats.cache_misses = cache_misses;
    stats.disk_writes = disk_writes;
    stats.disk_reads = disk_reads;
    
    // Count current cache entries
    uint16_t cache_count = 0;
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (cache[i].key_hash != 0) {
            cache_count++;
        }
    }
    stats.cache_size = cache_count;
    stats.free_space_kb = backend->getFreeSpace() / 1024;
    
    g_message_bus.publish(MSG_STORAGE_STATS, &stats, sizeof(stats));
}

// =============================================================================
// Direct Access Methods
// =============================================================================

bool StorageManager::save_float(const char* key, float value) {
    if (!key) return false;
    
    uint16_t key_hash = crc16(key);
    
    // Save to cache
    if (!save_to_cache(key_hash, value)) {
        return false;
    }
    
    // Write to backend immediately
    if (backend->writeData(key_hash, &value, sizeof(float))) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_float(const char* key, float* value, float default_value) {
    if (!key || !value) return false;
    
    uint16_t key_hash = crc16(key);
    
    *value = default_value;
    
    // Check cache first
    if (load_from_cache(key_hash, value)) {
        cache_hits++;
        return true;
    }
    
    // Load from backend
    if (backend->readData(key_hash, value, sizeof(float))) {
        cache_misses++;
        disk_reads++;
        
        // Add to cache for future access
        add_to_cache(key_hash, *value);
        return true;
    }
    
    cache_misses++;
    return false;
}

bool StorageManager::save_data(const char* key, const void* data, size_t size) {
    if (!key || !data || size == 0) return false;
    
    uint16_t key_hash = crc16(key);
    
    // Write to backend immediately
    if (backend->writeData(key_hash, data, size)) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_data(const char* key, void* data, size_t size) {
    if (!key || !data || size == 0) return false;
    
    uint16_t key_hash = crc16(key);
    
    // Load from backend
    if (backend->readData(key_hash, data, size)) {
        disk_reads++;
        return true;
    }
    
    return false;
}

// =============================================================================
// Debug/Maintenance Methods
// =============================================================================

void StorageManager::print_cache_info() {
    Serial.println("=== Storage Manager Cache Info ===");
    Serial.print("Cache Hits: "); Serial.println(cache_hits);
    Serial.print("Cache Misses: "); Serial.println(cache_misses);
    Serial.print("Disk Writes: "); Serial.println(disk_writes);
    Serial.print("Disk Reads: "); Serial.println(disk_reads);
    
    Serial.println("\nCached Keys:");
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (cache[i].key_hash != 0) {
            Serial.print("  Hash: 0x"); Serial.print(cache[i].key_hash, HEX);
            Serial.print(" = "); Serial.print(cache[i].value);
            Serial.print(" (access: "); Serial.print(cache[i].access_count);
            Serial.print(", dirty: "); Serial.print(cache[i].dirty ? "Y" : "N");
            Serial.println(")");
        }
    }
    Serial.println("=================================");
}

void StorageManager::print_storage_info() {
    if (backend) {
        Serial.println("=== Storage Backend Info ===");
        // Cast to SPIFlashStorageBackend for access to specific methods
        SPIFlashStorageBackend* spi_backend = static_cast<SPIFlashStorageBackend*>(backend);
        if (spi_backend) {
            spi_backend->printStorageInfo();
        }
    }
}

bool StorageManager::verify_integrity() {
    if (!backend) return false;
    // Cast to SPIFlashStorageBackend for access to specific methods
    SPIFlashStorageBackend* spi_backend = static_cast<SPIFlashStorageBackend*>(backend);
    if (spi_backend) {
        return spi_backend->verifyIntegrity();
    }
    return true;  // Default to true if backend doesn't support integrity check
}

void StorageManager::force_commit_cache() {
    commit_dirty_entries();
}

// =============================================================================
// Global Storage Manager Instance and Static Handlers
// =============================================================================

StorageManager* g_storage_manager_instance = nullptr;

void storage_save_float_handler(const CANMessage* msg) {
    if (g_storage_manager_instance) {
        g_storage_manager_instance->handle_save_float_message(msg);
    }
}

void storage_load_float_handler(const CANMessage* msg) {
    if (g_storage_manager_instance) {
        g_storage_manager_instance->handle_load_float_message(msg);
    }
}

void storage_commit_cache_handler(const CANMessage* msg) {
    if (g_storage_manager_instance) {
        g_storage_manager_instance->handle_commit_cache_message(msg);
    }
}

void storage_stats_handler(const CANMessage* msg) {
    if (g_storage_manager_instance) {
        g_storage_manager_instance->handle_stats_request_message(msg);
    }
} 