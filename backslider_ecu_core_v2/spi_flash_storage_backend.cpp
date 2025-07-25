// spi_flash_storage_backend.cpp
// SPI Flash storage backend implementation - Extended CAN ID Architecture

#include "spi_flash_storage_backend.h"
#include <cstring>
#include <sstream>
#include <iostream>

#ifdef TESTING
    #include "../tests/mock_arduino.h"
    extern MockSerial Serial;
#else
    #if defined(ARDUINO) && !defined(TESTING)
        #include <Arduino.h>
    #endif
#endif

// =============================================================================
// Constructor/Destructor
// =============================================================================

SPIFlashStorageBackend::SPIFlashStorageBackend() 
    : total_reads(0), total_writes(0), cache_hits(0), cache_misses(0) {
}

SPIFlashStorageBackend::~SPIFlashStorageBackend() {
}

// =============================================================================
// Core Interface Implementation
// =============================================================================

bool SPIFlashStorageBackend::begin() {
    // Initialize mock file system
    mock_files.clear();
    
    Serial.println("SPI Flash Storage Backend initialized (Extended CAN ID)");
    Serial.print("Total capacity: ");
    Serial.print(FLASH_SIZE / (1024 * 1024));
    Serial.println(" MB");
    
    return true;
}

bool SPIFlashStorageBackend::end() {
    sync();
    return true;
}

bool SPIFlashStorageBackend::readData(uint32_t storage_key, void* data, size_t dataSize) {
    if (!data || dataSize == 0 || storage_key == 0) {
        return false;
    }
    
    std::string filepath = getFilePath(storage_key);
    MockFile* file = openFile(filepath, "r");
    
    if (!file || !file->exists()) {
        return false;
    }
    
    size_t bytes_read = file->read(data, dataSize);
    file->close();
    
    total_reads++;
    
    if (bytes_read == dataSize) {
        return true;
    }
    
    return false;
}

bool SPIFlashStorageBackend::writeData(uint32_t storage_key, const void* data, size_t dataSize) {
    if (!data || dataSize == 0 || storage_key == 0) {
        return false;
    }
    
    std::string filepath = getFilePath(storage_key);
    
    // Ensure directory exists
    if (!ensureDirectoryExists(filepath)) {
        return false;
    }
    
    MockFile* file = openFile(filepath, "w");
    if (!file) {
        return false;
    }
    
    size_t bytes_written = file->write(data, dataSize);
    file->close();
    
    total_writes++;
    
    return bytes_written == dataSize;
}

bool SPIFlashStorageBackend::deleteData(uint32_t storage_key) {
    if (storage_key == 0) {
        return false;
    }
    
    std::string filepath = getFilePath(storage_key);
    auto it = mock_files.find(filepath);
    
    if (it != mock_files.end()) {
        mock_files.erase(it);
        return true;
    }
    
    return false;
}

bool SPIFlashStorageBackend::hasData(uint32_t storage_key) {
    if (storage_key == 0) {
        return false;
    }
    
    std::string filepath = getFilePath(storage_key);
    auto it = mock_files.find(filepath);
    
    return (it != mock_files.end() && it->second.exists());
}

// =============================================================================
// Storage Management
// =============================================================================

uint32_t SPIFlashStorageBackend::getTotalSpace() {
    return FLASH_SIZE;
}

uint32_t SPIFlashStorageBackend::getFreeSpace() {
    uint32_t used = getUsedSpace();
    return (used < FLASH_SIZE) ? (FLASH_SIZE - used) : 0;
}

uint32_t SPIFlashStorageBackend::getUsedSpace() {
    uint32_t total_used = 0;
    
    for (const auto& pair : mock_files) {
        total_used += pair.second.size();
    }
    
    return total_used;
}

void SPIFlashStorageBackend::sync() {
    // In a real implementation, this would sync to flash
    // For mock implementation, data is already "synced"
}

void SPIFlashStorageBackend::flush() {
    // In a real implementation, this would flush write cache
    // For mock implementation, no cache to flush
}

// =============================================================================
// Iteration Support
// =============================================================================

uint32_t SPIFlashStorageBackend::getStoredKeyCount() {
    return mock_files.size();
}

bool SPIFlashStorageBackend::getStoredKey(uint32_t index, uint32_t* storage_key) {
    if (!storage_key || index >= mock_files.size()) {
        return false;
    }
    
    auto it = mock_files.begin();
    std::advance(it, index);
    
    // Extract CAN ID from filename
    // Format: keys/ECU_BASE/SUBSYSTEM/PARAMETER.bin
    std::string filepath = it->first;
    
    // Parse the path: keys/1/03/00001.bin -> 0x10300001
    size_t pos = filepath.find("keys/");
    if (pos == std::string::npos) return false;
    
    std::string path_part = filepath.substr(pos + 5);  // Skip "keys/"
    
    // Extract ECU base (1 hex digit)
    size_t slash1 = path_part.find('/');
    if (slash1 == std::string::npos) return false;
    
    std::string ecu_str = path_part.substr(0, slash1);
    
    // Extract subsystem (2 hex digits)
    size_t slash2 = path_part.find('/', slash1 + 1);
    if (slash2 == std::string::npos) return false;
    
    std::string subsystem_str = path_part.substr(slash1 + 1, slash2 - slash1 - 1);
    
    // Extract parameter (5 hex digits, remove .bin extension)
    std::string param_str = path_part.substr(slash2 + 1);
    size_t dot = param_str.find('.');
    if (dot != std::string::npos) {
        param_str = param_str.substr(0, dot);
    }
    
    // Convert to CAN ID
    uint32_t ecu_base = strtoul(ecu_str.c_str(), nullptr, 16);
    uint32_t subsystem = strtoul(subsystem_str.c_str(), nullptr, 16);
    uint32_t parameter = strtoul(param_str.c_str(), nullptr, 16);
    
    *storage_key = (ecu_base << 28) | (subsystem << 20) | (parameter & 0xFFFFF);
    
    return true;
}

// =============================================================================
// Debug/Information
// =============================================================================

void SPIFlashStorageBackend::printDebugInfo() {
    Serial.println("=== SPI Flash Storage Backend Debug Info ===");
    Serial.print("Total reads: "); Serial.println(total_reads);
    Serial.print("Total writes: "); Serial.println(total_writes);
    Serial.print("Cache hits: "); Serial.println(cache_hits);
    Serial.print("Cache misses: "); Serial.println(cache_misses);
    Serial.print("Stored keys: "); Serial.println(getStoredKeyCount());
    Serial.print("Used space: "); Serial.print(getUsedSpace()); Serial.println(" bytes");
    Serial.print("Free space: "); Serial.print(getFreeSpace()); Serial.println(" bytes");
    
    Serial.println("\nStored Keys:");
    for (uint32_t i = 0; i < getStoredKeyCount(); i++) {
        uint32_t storage_key;
        if (getStoredKey(i, &storage_key)) {
            Serial.print("  ["); Serial.print(i); Serial.print("] 0x");
            Serial.print(storage_key, HEX);
            Serial.print(" -> ");
            printExtendedCanId(storage_key);
            Serial.println();
        }
    }
    Serial.println("============================================");
}

void SPIFlashStorageBackend::printStorageInfo() {
    Serial.println("=== SPI Flash Storage Info ===");
    Serial.print("Capacity: "); Serial.print(FLASH_SIZE / (1024 * 1024)); Serial.println(" MB");
    Serial.print("Used: "); Serial.print(getUsedSpace()); Serial.println(" bytes");
    Serial.print("Free: "); Serial.print(getFreeSpace()); Serial.println(" bytes");
    Serial.print("Files: "); Serial.println(getStoredKeyCount());
    Serial.println("==============================");
}

bool SPIFlashStorageBackend::verifyIntegrity() {
    // In a real implementation, this would verify checksums
    // For mock implementation, assume all data is valid
    return true;
}

void SPIFlashStorageBackend::formatStorage() {
    mock_files.clear();
    total_reads = 0;
    total_writes = 0;
    cache_hits = 0;
    cache_misses = 0;
    
    Serial.println("SPI Flash storage formatted");
}

// =============================================================================
// Private Helper Methods
// =============================================================================

std::string SPIFlashStorageBackend::getFilePath(uint32_t storage_key) {
    char filename[64];
    storage_key_to_filename(storage_key, filename, sizeof(filename));
    return std::string(filename);
}

bool SPIFlashStorageBackend::ensureDirectoryExists(const std::string& filepath) {
    // Extract directory path from filepath
    size_t last_slash = filepath.find_last_of('/');
    if (last_slash == std::string::npos) {
        return true;  // No directory needed
    }
    
    std::string dir_path = filepath.substr(0, last_slash);
    
    // For mock implementation, directories are implicit
    // Real implementation would create directories on flash filesystem
    
    return true;
}

SPIFlashStorageBackend::MockFile* SPIFlashStorageBackend::openFile(const std::string& path, const char* mode) {
    auto it = mock_files.find(path);
    
    if (it == mock_files.end()) {
        // Create new file
        mock_files[path] = MockFile();
    }
    
    MockFile* file = &mock_files[path];
    file->open(path.c_str(), mode);
    
    return file;
}

void SPIFlashStorageBackend::printExtendedCanId(uint32_t storage_key) {
    uint8_t ecu_base = (storage_key >> 28) & 0x0F;
    uint8_t subsystem = (storage_key >> 20) & 0xFF;
    uint32_t parameter = storage_key & 0xFFFFF;
    
    Serial.print("ECU="); Serial.print(ecu_base, HEX);
    Serial.print(" SUB="); Serial.print(subsystem, HEX);
    Serial.print(" PARAM="); Serial.print(parameter, HEX);
    
    // Print human-readable subsystem name
    switch (subsystem) {
        case 0x01: Serial.print(" (FUEL)"); break;
        case 0x02: Serial.print(" (IGNITION)"); break;
        case 0x03: Serial.print(" (SENSORS)"); break;
        case 0x04: Serial.print(" (CONFIG)"); break;
        case 0x05: Serial.print(" (TRANSMISSION)"); break;
        case 0x06: Serial.print(" (COOLING)"); break;
        case 0x07: Serial.print(" (EXHAUST)"); break;
        case 0x08: Serial.print(" (BOOST)"); break;
        case 0x09: Serial.print(" (STORAGE)"); break;
        case 0x0A: Serial.print(" (SYSTEM)"); break;
        case 0x0B: Serial.print(" (DEBUG)"); break;
        case 0x0C: Serial.print(" (EXTERNAL)"); break;
        default: Serial.print(" (UNKNOWN)"); break;
    }
} 