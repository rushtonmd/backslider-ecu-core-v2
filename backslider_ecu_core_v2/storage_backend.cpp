// storage_backend.cpp
// Implementation of storage backend interfaces

#ifndef ARDUINO
#include "../tests/mock_arduino.h"
extern MockSerial Serial;
#endif

#include "storage_backend.h"
#include "msg_definitions.h"  // For CRC16 function

#ifdef ARDUINO
#include <Arduino.h>
#include <EEPROM.h>
#else
// Mock EEPROM for testing
class MockEEPROM {
private:
    uint8_t data[1080];
public:
    MockEEPROM() { memset(data, 0xFF, sizeof(data)); }
    uint8_t read(int address) { return data[address]; }
    void write(int address, uint8_t value) { data[address] = value; }
    
    // Template functions to match Teensy EEPROM API
    template<typename T>
    void put(int address, const T& value) {
        memcpy(&data[address], &value, sizeof(T));
    }
    
    template<typename T>
    T& get(int address, T& value) {
        memcpy(&value, &data[address], sizeof(T));
        return value;
    }
} MockEEPROM;
#define EEPROM MockEEPROM
#endif

// =============================================================================
// EEPROM Storage Backend Implementation
// =============================================================================

EEPROMStorageBackend::EEPROMStorageBackend() : write_count(0), read_count(0) {
}

bool EEPROMStorageBackend::begin() {
    // Check if EEPROM is formatted (look for valid entries)
    bool has_valid_data = false;
    
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 1 && entry.checksum == calculateChecksum(&entry)) {
            has_valid_data = true;
            break;
        }
    }
    
    // If no valid data found, format the storage
    if (!has_valid_data) {
        formatStorage();
    }
    
    return true;
}

bool EEPROMStorageBackend::writeData(uint16_t key_hash, const void* data, size_t size) {
    if (!data || size == 0 || size > 255) {
        return false;
    }
    
    // Check if key already exists
    KeyEntry existing_entry;
    int existing_index;
    bool key_exists = findKeyEntry(key_hash, &existing_entry, &existing_index);
    
    if (key_exists) {
        // Update existing entry
        if (existing_entry.data_size == size) {
            // Same size - just update the data
            const uint8_t* byte_data = static_cast<const uint8_t*>(data);
            for (size_t i = 0; i < size; i++) {
                EEPROM.write(existing_entry.data_offset + i, byte_data[i]);
            }
        } else {
            // Different size - mark old entry as deleted and create new one
            existing_entry.valid = 0;
            EEPROM.put(EEPROM_START + existing_index * sizeof(KeyEntry), existing_entry);
            
            // Fall through to create new entry
            key_exists = false;
        }
    }
    
    if (!key_exists) {
        // Create new entry
        int free_index;
        if (!findFreeEntry(&free_index)) {
            return false;  // No free slots
        }
        
        uint16_t data_offset = findFreeDataSpace(size);
        if (data_offset == 0) {
            return false;  // No free space
        }
        
        // Write data to EEPROM
        const uint8_t* byte_data = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < size; i++) {
            EEPROM.write(data_offset + i, byte_data[i]);
        }
        
        // Create key entry
        KeyEntry new_entry;
        memset(&new_entry, 0, sizeof(new_entry));
        new_entry.key_hash = key_hash;
        new_entry.data_offset = data_offset;
        new_entry.data_size = size;
        new_entry.valid = 1;
        new_entry.checksum = calculateChecksum(&new_entry);
        
        // Write key entry to EEPROM
        EEPROM.put(EEPROM_START + free_index * sizeof(KeyEntry), new_entry);
    }
    
    write_count++;
    return true;
}

bool EEPROMStorageBackend::readData(uint16_t key_hash, void* data, size_t size) {
    if (!data || size == 0) {
        return false;
    }
    
    KeyEntry entry;
    int index;
    if (!findKeyEntry(key_hash, &entry, &index)) {
        return false;  // Key not found
    }
    
    if (entry.data_size != size) {
        return false;  // Size mismatch
    }
    
    // Read data from EEPROM
    uint8_t* byte_data = static_cast<uint8_t*>(data);
    for (size_t i = 0; i < size; i++) {
        byte_data[i] = EEPROM.read(entry.data_offset + i);
    }
    
    read_count++;
    return true;
}

bool EEPROMStorageBackend::deleteKey(uint16_t key_hash) {
    KeyEntry entry;
    int index;
    if (!findKeyEntry(key_hash, &entry, &index)) {
        return false;  // Key not found
    }
    
    // Mark entry as deleted
    entry.valid = 0;
    EEPROM.put(EEPROM_START + index * sizeof(KeyEntry), entry);
    
    return true;
}

bool EEPROMStorageBackend::keyExists(uint16_t key_hash) {
    KeyEntry entry;
    int index;
    return findKeyEntry(key_hash, &entry, &index);
}

size_t EEPROMStorageBackend::getFreeSpace() {
    size_t used_space = DATA_START;
    
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 1 && entry.checksum == calculateChecksum(&entry)) {
            used_space += entry.data_size;
        }
    }
    
    return EEPROM_SIZE - used_space;
}

size_t EEPROMStorageBackend::getTotalSpace() {
    return EEPROM_SIZE - DATA_START;
}

void EEPROMStorageBackend::formatStorage() {
    // Clear all key entries
    KeyEntry empty_entry;
    memset(&empty_entry, 0, sizeof(empty_entry));
    
    for (int i = 0; i < MAX_KEYS; i++) {
        EEPROM.put(EEPROM_START + i * sizeof(KeyEntry), empty_entry);
    }
    
    // Clear data area
    for (int i = DATA_START; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0xFF);
    }
}

void EEPROMStorageBackend::printStorageInfo() {
    Serial.println("=== EEPROM Storage Info ===");
    Serial.print("Total Space: "); Serial.print(getTotalSpace()); Serial.println(" bytes");
    Serial.print("Free Space: "); Serial.print(getFreeSpace()); Serial.println(" bytes");
    Serial.print("Write Count: "); Serial.println(write_count);
    Serial.print("Read Count: "); Serial.println(read_count);
    
    Serial.println("\nStored Keys:");
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 1 && entry.checksum == calculateChecksum(&entry)) {
            Serial.print("  Hash: 0x"); Serial.print(entry.key_hash, HEX);
            Serial.print(" ("); Serial.print(entry.data_size); Serial.println(" bytes)");
        }
    }
    Serial.println("========================");
}

bool EEPROMStorageBackend::verifyIntegrity() {
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 1) {
            if (entry.checksum != calculateChecksum(&entry)) {
                return false;  // Checksum mismatch
            }
        }
    }
    return true;
}

// =============================================================================
// Private Helper Functions
// =============================================================================

bool EEPROMStorageBackend::findKeyEntry(uint16_t key_hash, KeyEntry* entry, int* index) {
    for (int i = 0; i < MAX_KEYS; i++) {
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), *entry);
        
        if (entry->valid == 1 && 
            entry->checksum == calculateChecksum(entry) &&
            entry->key_hash == key_hash) {
            *index = i;
            return true;
        }
    }
    return false;
}

bool EEPROMStorageBackend::findFreeEntry(int* index) {
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 0) {
            *index = i;
            return true;
        }
    }
    return false;
}

uint16_t EEPROMStorageBackend::findFreeDataSpace(uint16_t size) {
    // Simple allocation - find first free space
    // This is not optimal but works for initial implementation
    
    bool used[EEPROM_SIZE - DATA_START];
    memset(used, false, sizeof(used));
    
    // Mark used spaces
    for (int i = 0; i < MAX_KEYS; i++) {
        KeyEntry entry;
        EEPROM.get(EEPROM_START + i * sizeof(KeyEntry), entry);
        
        if (entry.valid == 1 && entry.checksum == calculateChecksum(&entry)) {
            for (int j = entry.data_offset; j < entry.data_offset + entry.data_size; j++) {
                if (j >= DATA_START && j < EEPROM_SIZE) {
                    used[j - DATA_START] = true;
                }
            }
        }
    }
    
    // Find first free block of required size
    for (int i = 0; i <= (EEPROM_SIZE - DATA_START) - size; i++) {
        bool block_free = true;
        for (int j = 0; j < size; j++) {
            if (used[i + j]) {
                block_free = false;
                break;
            }
        }
        if (block_free) {
            return DATA_START + i;
        }
    }
    
    return 0;  // No free space found
}

uint8_t EEPROMStorageBackend::calculateChecksum(const KeyEntry* entry) {
    uint8_t checksum = 0;
    const uint8_t* data = (const uint8_t*)entry;
    
    // Calculate checksum for all fields except the checksum field itself
    for (size_t i = 0; i < sizeof(KeyEntry) - 1; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

void EEPROMStorageBackend::defragmentStorage() {
    // TODO: Implement defragmentation if needed
    // For now, this is a placeholder
} 