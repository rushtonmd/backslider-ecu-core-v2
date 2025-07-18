// spi_flash_storage_backend.cpp
// Implementation of SPI Flash storage backend using Adafruit W25Q128 + FAT32

#include "spi_flash_storage_backend.h"

#ifndef ARDUINO
#include "../tests/mock_arduino.h"
extern MockSerial Serial;
#endif

#if defined(ARDUINO) && !defined(TESTING)
#include <Arduino.h>
// Flash transport configuration for Adafruit W25Q128
// This creates the flashTransport object needed by Adafruit_SPIFlash
#include "flash_config.h"
#else
// Static storage for mock files
#include <map>
#include <vector>
std::map<std::string, std::vector<uint8_t>> MockFatVolume::mock_files;
#endif

#ifndef ARDUINO
// Mock fatfs object for testing
static MockFatVolume fatfs;
#endif

// =============================================================================
// Constructor & Destructor
// =============================================================================

SPIFlashStorageBackend::SPIFlashStorageBackend() 
    : write_count(0), read_count(0), format_count(0) {
    #if defined(ARDUINO) && !defined(TESTING)
    // Initialize flash object with transport from flash_config.h
    flash = new Adafruit_SPIFlash(&flashTransport);
    #else
    // Initialize mock flash object for testing
    flash = new MockSPIFlash();
    #endif
}

SPIFlashStorageBackend::~SPIFlashStorageBackend() {
    #if defined(ARDUINO) && !defined(TESTING)
    delete flash;
    #endif
}

// =============================================================================
// StorageBackend Interface Implementation
// =============================================================================

bool SPIFlashStorageBackend::begin() {
    #if defined(ARDUINO) && !defined(TESTING)
    Serial.println("Initializing SPI Flash storage...");
    
    // Initialize the flash chip
    if (!flash->begin()) {
        Serial.println("ERROR: Failed to initialize SPI flash chip");
        return false;
    }
    
    Serial.print("Flash chip size: ");
    Serial.print(flash->size() / 1024 / 1024);
    Serial.println(" MB");
    
    // Initialize FAT32 filesystem
    if (!fatfs.begin(flash)) {
        Serial.println("WARNING: No filesystem found. Formatting...");
        if (!formatFilesystem()) {
            Serial.println("ERROR: Failed to format filesystem");
            return false;
        }
        
        if (!fatfs.begin(flash)) {
            Serial.println("ERROR: Failed to mount filesystem after format");
            return false;
        }
    }
    
    // Create directory structure
    ensureDirectoryExists("/keys");
    ensureDirectoryExists("/config");
    ensureDirectoryExists("/maps");
    ensureDirectoryExists("/logs");
    
    Serial.println("SPI Flash storage initialized successfully");
    return true;
    #else
    // Mock implementation for testing
    return true;
    #endif
}

bool SPIFlashStorageBackend::writeData(uint16_t key_hash, const void* data, size_t size) {
    if (!data || size == 0) {
        return false;
    }
    
    char filename[32];
    keyHashToFilename(key_hash, filename, sizeof(filename));
    
    #ifndef ARDUINO
    std::cout << "Writing to file: " << filename << ", size: " << size << std::endl;
    #endif
    
    #if defined(ARDUINO) && !defined(TESTING)
    // Remove existing file if it exists
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    // Create new file
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    // Write data
    size_t written = file.write(data, size);
    file.close();
    
    write_count++;
    return written == size;
    #else
    // Mock implementation for testing
    // Remove existing file if it exists (to match Arduino behavior)
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    size_t written = file.write(data, size);
    file.close();
    
    write_count++;
    return written == size;
    #endif
}

bool SPIFlashStorageBackend::readData(uint16_t key_hash, void* data, size_t size) {
    if (!data || size == 0) {
        return false;
    }
    
    char filename[32];
    keyHashToFilename(key_hash, filename, sizeof(filename));
    
    #ifndef ARDUINO
    std::cout << "Reading from file: " << filename << ", size: " << size << std::endl;
    #endif
    
    #if defined(ARDUINO) && !defined(TESTING)
    // Open file for reading
    File32 file = fatfs.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    // Check file size matches expected size
    if (file.size() != size) {
        file.close();
        return false;
    }
    
    // Read data
    size_t read_bytes = file.read(data, size);
    file.close();
    
    read_count++;
    return read_bytes == size;
    #else
    // Mock implementation for testing
    File32 file = fatfs.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    // Check file size matches expected size
    if (file.size() != size) {
        file.close();
        return false;
    }
    
    // Read data
    size_t read_bytes = file.read(data, size);
    file.close();
    
    read_count++;
    
    std::cout << "Mock read completed, bytes read: " << read_bytes << std::endl;
    float* float_data = static_cast<float*>(data);
    std::cout << "Read float value: " << *float_data << std::endl;
    
    return read_bytes == size;
    #endif
}

bool SPIFlashStorageBackend::deleteKey(uint16_t key_hash) {
    char filename[32];
    keyHashToFilename(key_hash, filename, sizeof(filename));
    
    #if defined(ARDUINO) && !defined(TESTING)
    if (!fatfs.exists(filename)) {
        return false;
    }
    
    return fatfs.remove(filename);
    #else
    // Mock implementation for testing
    return fatfs.remove(filename);
    #endif
}

bool SPIFlashStorageBackend::keyExists(uint16_t key_hash) {
    char filename[32];
    keyHashToFilename(key_hash, filename, sizeof(filename));
    
    #if defined(ARDUINO) && !defined(TESTING)
    return fatfs.exists(filename);
    #else
    // Mock implementation for testing
    return fatfs.exists(filename);
    #endif
}

size_t SPIFlashStorageBackend::getFreeSpace() {
    #if defined(ARDUINO) && !defined(TESTING)
    // FAT32 free space calculation
    // This is approximate - exact calculation requires cluster counting
    return flash->size() / 2;  // Rough estimate
    #else
    return 16 * 1024 * 1024;  // 16MB for testing
    #endif
}

size_t SPIFlashStorageBackend::getTotalSpace() {
    #if defined(ARDUINO) && !defined(TESTING)
    return flash->size();
    #else
    return 16 * 1024 * 1024;  // 16MB for testing
    #endif
}

// =============================================================================
// Helper Methods
// =============================================================================

void SPIFlashStorageBackend::keyHashToFilename(uint16_t key_hash, char* filename, size_t max_len) {
    // Organize files by hash prefix for better directory structure
    // e.g., hash 0x1234 -> "/keys/12/1234.bin"
    uint8_t dir_prefix = (key_hash >> 8) & 0xFF;
    snprintf(filename, max_len, "/keys/%02X/%04X.bin", dir_prefix, key_hash);
}

bool SPIFlashStorageBackend::ensureDirectoryExists(const char* path) {
    #if defined(ARDUINO) && !defined(TESTING)
    // FAT32 doesn't require explicit directory creation for simple paths
    // The filesystem will create directories automatically when files are written
    return true;
    #else
    return true;
    #endif
}

bool SPIFlashStorageBackend::formatFilesystem() {
    format_count++;
    
    // Note: SdFat library doesn't provide a format() method
    // Format would typically be done externally or during initialization
    // For now, we'll assume the filesystem is already formatted
    return true;
}

// =============================================================================
// Enhanced Storage Methods
// =============================================================================

bool SPIFlashStorageBackend::formatStorage() {
    return formatFilesystem();
}

void SPIFlashStorageBackend::printStorageInfo() {
    #if defined(ARDUINO) && !defined(TESTING)
    Serial.println("=== SPI Flash Storage Info ===");
    Serial.print("Total space: ");
    Serial.print(getTotalSpace() / 1024);
    Serial.println(" KB");
    
    Serial.print("Free space: ");
    Serial.print(getFreeSpace() / 1024);
    Serial.println(" KB");
    
    Serial.print("Write operations: ");
    Serial.println(write_count);
    
    Serial.print("Read operations: ");
    Serial.println(read_count);
    
    Serial.print("Format operations: ");
    Serial.println(format_count);
    #endif
}

bool SPIFlashStorageBackend::verifyIntegrity() {
    // Basic integrity check - verify flash chip is responding
    #if defined(ARDUINO) && !defined(TESTING)
    return flash->begin();
    #else
    return true;
    #endif
}

// =============================================================================
// Configuration-Specific Helper Methods
// =============================================================================

bool SPIFlashStorageBackend::saveJSON(const char* key, const char* json_data) {
    if (!key || !json_data) {
        return false;
    }
    
    #if defined(ARDUINO) && !defined(TESTING)
    char filename[64];
    snprintf(filename, sizeof(filename), "/config/%s.json", key);
    
    // Remove existing file
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    // Create new file
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    // Write JSON data
    size_t written = file.write(json_data, strlen(json_data));
    file.close();
    
    write_count++;
    return written == strlen(json_data);
    #else
    // Mock implementation using mock file system
    char filename[64];
    snprintf(filename, sizeof(filename), "/config/%s.json", key);
    
    // Remove existing file if it exists (to match Arduino behavior)
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    // Create new file
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    // Write JSON data
    size_t written = file.write(json_data, strlen(json_data));
    file.close();
    
    write_count++;
    return written == strlen(json_data);
    #endif
}

bool SPIFlashStorageBackend::loadJSON(const char* key, char* json_data, size_t max_size) {
    if (!key || !json_data || max_size == 0) {
        return false;
    }
    
    #if defined(ARDUINO) && !defined(TESTING)
    char filename[64];
    snprintf(filename, sizeof(filename), "/config/%s.json", key);
    
    // Open file for reading
    File32 file = fatfs.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    // Check file size
    size_t file_size = file.size();
    if (file_size >= max_size) {
        file.close();
        return false;
    }
    
    // Read JSON data
    size_t read_bytes = file.read(json_data, file_size);
    file.close();
    
    if (read_bytes == file_size) {
        json_data[read_bytes] = '\0';  // Null terminate
        read_count++;
        return true;
    }
    
    return false;
    #else
    // Mock implementation using mock file system
    char filename[64];
    snprintf(filename, sizeof(filename), "/config/%s.json", key);
    
    // Open file for reading
    File32 file = fatfs.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    // Check file size
    size_t file_size = file.size();
    if (file_size >= max_size) {
        file.close();
        return false;
    }
    
    // Read JSON data
    size_t read_bytes = file.read(json_data, file_size);
    file.close();
    
    if (read_bytes == file_size) {
        json_data[read_bytes] = '\0';  // Null terminate
        read_count++;
        return true;
    }
    
    return false;
    #endif
}

bool SPIFlashStorageBackend::saveFloatArray(const char* key, const float* values, size_t count) {
    if (!key || !values || count == 0) {
        return false;
    }
    
    #if defined(ARDUINO) && !defined(TESTING)
    char filename[64];
    snprintf(filename, sizeof(filename), "/maps/%s.map", key);
    
    // Remove existing file
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    // Create new file
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    // Write array size first
    file.write(&count, sizeof(size_t));
    
    // Write float array
    size_t written = file.write(values, count * sizeof(float));
    file.close();
    
    write_count++;
    return written == count * sizeof(float);
    #else
    // Mock implementation using mock file system
    char filename[64];
    snprintf(filename, sizeof(filename), "/maps/%s.map", key);
    
    // Remove existing file if it exists (to match Arduino behavior)
    if (fatfs.exists(filename)) {
        fatfs.remove(filename);
    }
    
    // Create new file
    File32 file = fatfs.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    
    // Write array size first
    size_t size_written = file.write(&count, sizeof(size_t));
    
    // Write float array
    size_t written = file.write(values, count * sizeof(float));
    file.close();
    
    std::cout << "Saved float array: count=" << count << ", size_written=" << size_written 
              << ", array_written=" << written << ", total_expected=" << (sizeof(size_t) + count * sizeof(float)) << std::endl;
    
    write_count++;
    return written == count * sizeof(float);
    #endif
}

bool SPIFlashStorageBackend::loadFloatArray(const char* key, float* values, size_t count) {
    if (!key || !values || count == 0) {
        return false;
    }
    
    char filename[64];
    snprintf(filename, sizeof(filename), "/maps/%s.map", key);
    
    File32 file = fatfs.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    // Read the stored count first
    size_t stored_count;
    size_t size_bytes_read = file.read(&stored_count, sizeof(stored_count));
    
    if (size_bytes_read != sizeof(stored_count)) {
        file.close();
        return false;
    }
    
    if (stored_count != count) {
        file.close();
        return false;
    }
    
    // Read the array data
    size_t data_bytes_read = file.read(values, count * sizeof(float));
    file.close();
    
    read_count++;
    return data_bytes_read == count * sizeof(float);
} 