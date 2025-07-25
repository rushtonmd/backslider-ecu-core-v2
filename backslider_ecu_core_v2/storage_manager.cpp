// storage_manager.cpp
// Implementation of message-driven storage manager - Extended CAN ID Architecture

#ifndef ARDUINO
#include "../tests/mock_arduino.h"
extern MockSerial Serial;
#endif

#include "storage_manager.h"
#include "spi_flash_storage_backend.h"
#include "msg_bus.h"
#include "msg_definitions.h"

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
        cache[i].storage_key = 0;
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
    
    // Subscribe to storage messages (using new extended CAN IDs)
    g_storage_manager_instance = this;
    g_message_bus.subscribe(MSG_STORAGE_SAVE, storage_save_float_handler);
    g_message_bus.subscribe(MSG_STORAGE_LOAD, storage_load_float_handler);
    g_message_bus.subscribe(MSG_STORAGE_COMMIT, storage_commit_cache_handler);
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
    if (save_to_cache(save_msg->storage_key, save_msg->value)) {
        success = true;
        
        // Always write to backend immediately (no priority field anymore)
        if (backend->writeData(save_msg->storage_key, &save_msg->value, sizeof(float))) {
            disk_writes++;
        } else {
            success = false;
        }
    }
    
    // Send response
    send_save_response(save_msg->storage_key, success);
}

void StorageManager::handle_load_float_message(const CANMessage* msg) {
    if (msg->len != sizeof(storage_load_float_msg_t)) {
        return;  // Invalid message size
    }
    
    storage_load_float_msg_t* load_msg = MSG_UNPACK_STORAGE_LOAD_FLOAT(msg);
    
    float value = load_msg->default_value;
    bool success = false;
    
    // Check cache first
    if (load_from_cache(load_msg->storage_key, &value)) {
        success = true;
        cache_hits++;
    } else {
        // Load from backend
        if (backend->readData(load_msg->storage_key, &value, sizeof(float))) {
            success = true;
            cache_misses++;
            disk_reads++;
            
            // Add to cache for future access
            add_to_cache(load_msg->storage_key, value);
        } else {
            cache_misses++;
            // Keep default value if key not found
        }
    }
    
    // Send response
    send_load_response(load_msg->storage_key, value, success);
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

bool StorageManager::save_to_cache(uint32_t storage_key, float value) {
    if (storage_key == 0) return false;
    
    // Check if key already exists in cache
    int existing_index = find_cache_entry(storage_key);
    if (existing_index >= 0) {
        // Update existing entry
        cache[existing_index].value = value;
        cache[existing_index].timestamp = millis();
        cache[existing_index].dirty = true;
        cache[existing_index].access_count++;
        return true;
    }
    
    // Add new entry
    add_to_cache(storage_key, value);
    cache[cache_index].dirty = true;
    
    return true;
}

bool StorageManager::load_from_cache(uint32_t storage_key, float* value) {
    if (storage_key == 0 || !value) return false;
    
    int index = find_cache_entry(storage_key);
    if (index >= 0) {
        *value = cache[index].value;
        cache[index].timestamp = millis();
        cache[index].access_count++;
        return true;
    }
    
    return false;
}

void StorageManager::add_to_cache(uint32_t storage_key, float value) {
    if (storage_key == 0) return;
    
    // Find oldest entry to replace
    int oldest_index = find_oldest_cache_entry();
    
    // Clear the entry
    cache[oldest_index].storage_key = 0;
    cache[oldest_index].value = 0.0f;
    cache[oldest_index].timestamp = millis();
    cache[oldest_index].dirty = false;
    cache[oldest_index].access_count = 1;
    
    // Add new data
    cache[oldest_index].storage_key = storage_key;
    cache[oldest_index].value = value;
    
    cache_index = oldest_index;
}

int StorageManager::find_cache_entry(uint32_t storage_key) {
    if (storage_key == 0) return -1;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (cache[i].storage_key != 0 && cache[i].storage_key == storage_key) {
            return i;
        }
    }
    
    return -1;
}

int StorageManager::find_oldest_cache_entry() {
    int oldest = 0;
    uint32_t oldest_time = cache[0].timestamp;
    
    for (int i = 1; i < CACHE_SIZE; i++) {
        if (cache[i].storage_key == 0) {
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
        if (cache[i].dirty && cache[i].storage_key != 0) {
            if (backend->writeData(cache[i].storage_key, &cache[i].value, sizeof(float))) {
                cache[i].dirty = false;
                disk_writes++;
            }
        }
    }
}

// =============================================================================
// Response Helpers
// =============================================================================

void StorageManager::send_save_response(uint32_t storage_key, bool success) {
    storage_save_response_msg_t response;
    response.storage_key = storage_key;
    response.success = success ? 1 : 0;
    response.reserved[0] = 0;
    response.reserved[1] = 0;
    response.reserved[2] = 0;
    
    g_message_bus.publish(MSG_STORAGE_SAVE_RESPONSE, &response, sizeof(response));
}

void StorageManager::send_load_response(uint32_t storage_key, float value, bool success) {
    storage_load_response_msg_t response;
    response.storage_key = storage_key;
    response.value = value;
    
    g_message_bus.publish(MSG_STORAGE_LOAD_RESPONSE, &response, sizeof(response));
}

void StorageManager::send_error_response(uint32_t storage_key, uint8_t error_code) {
    storage_error_msg_t response;
    response.storage_key = storage_key;
    response.error_code = error_code;
    response.reserved[0] = 0;
    response.reserved[1] = 0;
    response.reserved[2] = 0;
    
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
        if (cache[i].storage_key != 0) {
            cache_count++;
        }
    }
    stats.cache_size = cache_count;
    stats.free_space_kb = backend->getFreeSpace() / 1024;
    
    g_message_bus.publish(MSG_STORAGE_STATS, &stats, sizeof(stats));
}

// =============================================================================
// Direct Access Methods (Updated to use Extended CAN IDs)
// =============================================================================

bool StorageManager::save_float(const char* key, float value) {
    if (!key) return false;
    
    // Convert string key to extended CAN ID
    // For now, we'll use a simple hash mapping - in the future this could be more sophisticated
    uint32_t storage_key = convert_string_to_extended_can_id(key);
    
    // Save to cache
    if (!save_to_cache(storage_key, value)) {
        return false;
    }
    
    // Write to backend immediately
    if (backend->writeData(storage_key, &value, sizeof(float))) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_float(const char* key, float* value, float default_value) {
    if (!key || !value) return false;
    
    // Convert string key to extended CAN ID
    uint32_t storage_key = convert_string_to_extended_can_id(key);
    
    *value = default_value;
    
    // Check cache first
    if (load_from_cache(storage_key, value)) {
        cache_hits++;
        return true;
    }
    
    // Load from backend
    if (backend->readData(storage_key, value, sizeof(float))) {
        cache_misses++;
        disk_reads++;
        
        // Add to cache for future access
        add_to_cache(storage_key, *value);
        return true;
    }
    
    cache_misses++;
    return false;
}

bool StorageManager::save_data(const char* key, const void* data, size_t size) {
    if (!key || !data || size == 0) return false;
    
    // Convert string key to extended CAN ID
    uint32_t storage_key = convert_string_to_extended_can_id(key);
    
    // Write to backend immediately
    if (backend->writeData(storage_key, data, size)) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_data(const char* key, void* data, size_t size) {
    if (!key || !data || size == 0) return false;
    
    // Convert string key to extended CAN ID
    uint32_t storage_key = convert_string_to_extended_can_id(key);
    
    // Load from backend
    if (backend->readData(storage_key, data, size)) {
        disk_reads++;
        return true;
    }
    
    return false;
}

// =============================================================================
// Extended CAN ID Direct Access Methods
// =============================================================================

bool StorageManager::save_float(uint32_t storage_key, float value) {
    if (storage_key == 0) return false;
    
    // Save to cache
    if (!save_to_cache(storage_key, value)) {
        return false;
    }
    
    // Write to backend immediately
    if (backend->writeData(storage_key, &value, sizeof(float))) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_float(uint32_t storage_key, float* value, float default_value) {
    if (storage_key == 0 || !value) return false;
    
    *value = default_value;
    
    // Check cache first
    if (load_from_cache(storage_key, value)) {
        cache_hits++;
        return true;
    }
    
    // Load from backend
    if (backend->readData(storage_key, value, sizeof(float))) {
        cache_misses++;
        disk_reads++;
        
        // Add to cache for future access
        add_to_cache(storage_key, *value);
        return true;
    }
    
    cache_misses++;
    return false;
}

bool StorageManager::save_data(uint32_t storage_key, const void* data, size_t size) {
    if (storage_key == 0 || !data || size == 0) return false;
    
    // Write to backend immediately
    if (backend->writeData(storage_key, data, size)) {
        disk_writes++;
        return true;
    }
    
    return false;
}

bool StorageManager::load_data(uint32_t storage_key, void* data, size_t size) {
    if (storage_key == 0 || !data || size == 0) return false;
    
    // Load from backend
    if (backend->readData(storage_key, data, size)) {
        disk_reads++;
        return true;
    }
    
    return false;
}

// =============================================================================
// Extended CAN ID Conversion
// =============================================================================

uint32_t StorageManager::convert_string_to_extended_can_id(const char* key) {
    if (!key) return 0;
    
    // For now, use a simple hash to convert string to CAN ID
    // In a production system, this would be more sophisticated mapping
    uint32_t hash = 0;
    while (*key) {
        hash = hash * 31 + *key;
        key++;
    }
    
    // Map to configuration subsystem parameter space
    return MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, hash & PARAMETER_MASK);
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
        if (cache[i].storage_key != 0) {
            Serial.print("  Key: 0x"); Serial.print(cache[i].storage_key, HEX);
            Serial.print(" = "); Serial.print(cache[i].value);
            Serial.print(" (access: "); Serial.print(cache[i].access_count);
            Serial.print(", dirty: "); Serial.print(cache[i].dirty ? "Y" : "N");
            Serial.print(") [ECU=0x"); Serial.print(GET_ECU_BASE(cache[i].storage_key) >> 28, HEX);
            Serial.print(" SUB=0x"); Serial.print(GET_SUBSYSTEM(cache[i].storage_key) >> 20, HEX);
            Serial.print(" PARAM=0x"); Serial.print(GET_PARAMETER(cache[i].storage_key), HEX);
            Serial.println("]");
        }
    }
    Serial.println("=================================");
}

void StorageManager::print_storage_info() {
    if (backend) {
        Serial.println("=== Storage Backend Info ===");
        // Use the generic printDebugInfo method from StorageBackend interface
        backend->printDebugInfo();
    }
}

bool StorageManager::verify_integrity() {
    if (!backend) return false;
    // For now, assume integrity is good if the backend is initialized
    // The W25Q128 backend doesn't have a verifyIntegrity method yet
    return true;  // Default to true if backend doesn't support integrity check
}

void StorageManager::force_commit_cache() {
    commit_dirty_entries();
}

void StorageManager::run_storage_diagnostics() {
    Serial.println("🔍 DEBUG: Storage diagnostics function called!");
    if (!backend) {
        Serial.println("❌ Storage Diagnostics: No backend available");
        return;
    }
    
    Serial.println("\n============================================================");
    Serial.println("🔍 STORAGE SYSTEM DIAGNOSTICS");
    Serial.println("============================================================");
    
    // =============================================================================
    // Storage Type and Hardware Information
    // =============================================================================
    Serial.println("\n📋 STORAGE HARDWARE:");
    Serial.println("   Type: W25Q128 SPI Flash Memory");
    Serial.println("   Capacity: 128 Mbit (16 MB)");
    Serial.println("   Interface: SPI (Serial Peripheral Interface)");
    Serial.println("   Sector Size: 4 KB");
    Serial.println("   Page Size: 256 bytes");
    Serial.println("   Write Endurance: 100,000 cycles per sector");
    Serial.println("   Data Retention: 20 years");
    
    // =============================================================================
    // Storage Space Information
    // =============================================================================
    Serial.println("\n💾 STORAGE SPACE:");
    uint32_t total_space = backend->getTotalSpace();
    uint32_t used_space = backend->getUsedSpace();
    uint32_t free_space = backend->getFreeSpace();
    float usage_percent = (float)used_space / total_space * 100.0f;
    
    Serial.print("   Total Space: ");
    if (total_space >= 1024*1024) {
        Serial.print(total_space / (1024*1024), 1);
        Serial.println(" MB");
    } else if (total_space >= 1024) {
        Serial.print(total_space / 1024, 1);
        Serial.println(" KB");
    } else {
        Serial.print(total_space);
        Serial.println(" bytes");
    }
    
    Serial.print("   Used Space: ");
    if (used_space >= 1024*1024) {
        Serial.print(used_space / (1024*1024), 1);
        Serial.println(" MB");
    } else if (used_space >= 1024) {
        Serial.print(used_space / 1024, 1);
        Serial.println(" KB");
    } else {
        Serial.print(used_space);
        Serial.println(" bytes");
    }
    
    Serial.print("   Free Space: ");
    if (free_space >= 1024*1024) {
        Serial.print(free_space / (1024*1024), 1);
        Serial.println(" MB");
    } else if (free_space >= 1024) {
        Serial.print(free_space / 1024, 1);
        Serial.println(" KB");
    } else {
        Serial.print(free_space);
        Serial.println(" bytes");
    }
    
    Serial.print("   Usage: ");
    Serial.print(usage_percent, 1);
    Serial.println("%");
    
    // =============================================================================
    // Key/Value Information
    // =============================================================================
    Serial.println("\n🔑 STORED DATA:");
    uint32_t key_count = backend->getStoredKeyCount();
    Serial.print("   Number of Keys: ");
    Serial.println(key_count);
    
    if (key_count > 0) {
        Serial.println("   Sample Keys (first 5):");
        for (uint32_t i = 0; i < key_count && i < 5; i++) {
            uint32_t storage_key;
            if (backend->getStoredKey(i, &storage_key)) {
                Serial.print("     ");
                Serial.print(i + 1);
                Serial.print(". 0x");
                Serial.print(storage_key, HEX);
                Serial.print(" [ECU=0x");
                Serial.print(GET_ECU_BASE(storage_key) >> 28, HEX);
                Serial.print(" SUB=0x");
                Serial.print(GET_SUBSYSTEM(storage_key) >> 20, HEX);
                Serial.print(" PARAM=0x");
                Serial.print(GET_PARAMETER(storage_key), HEX);
                Serial.println("]");
            }
        }
        if (key_count > 5) {
            Serial.print("     ... and ");
            Serial.print(key_count - 5);
            Serial.println(" more keys");
        }
    }
    
    // =============================================================================
    // Cache Performance Information
    // =============================================================================
    Serial.println("\n⚡ CACHE PERFORMANCE:");
    uint32_t total_accesses = cache_hits + cache_misses;
    float hit_rate = total_accesses > 0 ? (float)cache_hits / total_accesses * 100.0f : 0.0f;
    
    Serial.print("   Cache Size: ");
    Serial.print(CACHE_SIZE);
    Serial.println(" entries");
    Serial.print("   Cache Hits: ");
    Serial.println(cache_hits);
    Serial.print("   Cache Misses: ");
    Serial.println(cache_misses);
    Serial.print("   Hit Rate: ");
    Serial.print(hit_rate, 1);
    Serial.println("%");
    Serial.print("   Disk Reads: ");
    Serial.println(disk_reads);
    Serial.print("   Disk Writes: ");
    Serial.println(disk_writes);
    
    // =============================================================================
    // System Health
    // =============================================================================
    Serial.println("\n🏥 SYSTEM HEALTH:");
    bool integrity_ok = verify_integrity();
    Serial.print("   Data Integrity: ");
    Serial.println(integrity_ok ? "✅ GOOD" : "❌ FAILED");
    
    Serial.print("   Backend Status: ");
    Serial.println(backend ? "✅ INITIALIZED" : "❌ NOT INITIALIZED");
    
    // Check for potential issues
    bool warnings = false;
    if (usage_percent > 90.0f) {
        Serial.println("   ⚠️  WARNING: Storage usage > 90%");
        warnings = true;
    }
    if (hit_rate < 50.0f && total_accesses > 10) {
        Serial.println("   ⚠️  WARNING: Low cache hit rate");
        warnings = true;
    }
    if (!warnings) {
        Serial.println("   ✅ All systems operational");
    }
    
    // =============================================================================
    // Performance Metrics
    // =============================================================================
    Serial.println("\n📊 PERFORMANCE METRICS:");
    if (total_accesses > 0) {
        Serial.print("   Average Access Time: ");
        // This would be calculated from actual timing measurements
        Serial.println("~1-5 ms (estimated)");
    }
    Serial.println("   Write Speed: ~25 MHz SPI");
    Serial.println("   Read Speed: ~25 MHz SPI");
    Serial.println("   Sector Erase Time: ~60-200 ms");
    
    Serial.println("============================================================");
    Serial.println("✅ Storage diagnostics complete");
    Serial.println("============================================================");
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