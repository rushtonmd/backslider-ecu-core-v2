// storage_backend.h
// Abstract base class for storage backends - Extended CAN ID Architecture

#ifndef STORAGE_BACKEND_H
#define STORAGE_BACKEND_H

#include <stdint.h>
#include <string.h>
#include <cstdio>

// =============================================================================
// Abstract Storage Backend Interface
// =============================================================================

class StorageBackend {
public:
    virtual ~StorageBackend() = default;
    
    // Core interface
    virtual bool begin() = 0;
    virtual bool end() = 0;
    
    // Data access using extended CAN ID as key
    virtual bool readData(uint32_t storage_key, void* data, size_t dataSize) = 0;
    virtual bool writeData(uint32_t storage_key, const void* data, size_t dataSize) = 0;
    virtual bool deleteData(uint32_t storage_key) = 0;
    virtual bool hasData(uint32_t storage_key) = 0;
    
    // Storage management
    virtual uint32_t getTotalSpace() = 0;
    virtual uint32_t getFreeSpace() = 0;
    virtual uint32_t getUsedSpace() = 0;
    
    // Maintenance
    virtual void sync() = 0;
    virtual void flush() = 0;
    
    // Optional iteration support
    virtual uint32_t getStoredKeyCount() = 0;
    virtual bool getStoredKey(uint32_t index, uint32_t* storage_key) = 0;
    
    // Debug/information
    virtual void printDebugInfo() = 0;
    
protected:
    // Helper method to convert extended CAN ID to filename
    static void storage_key_to_filename(uint32_t storage_key, char* filename, size_t filename_size);
};

// =============================================================================
// Inline Helper Implementation
// =============================================================================

inline void StorageBackend::storage_key_to_filename(uint32_t storage_key, char* filename, size_t filename_size) {
    // Convert extended CAN ID to hierarchical filename
    // Format: keys/ECU_BASE/SUBSYSTEM/PARAMETER.bin
    // Example: 0x10300001 -> keys/1/03/00001.bin
    
    uint8_t ecu_base = (storage_key >> 28) & 0x0F;
    uint8_t subsystem = (storage_key >> 20) & 0xFF;
    uint32_t parameter = storage_key & 0xFFFFF;
    
    snprintf(filename, filename_size, "keys/%01X/%02X/%05X.bin", ecu_base, subsystem, parameter);
}

#endif 