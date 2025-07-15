// storage_backend.h
// Abstract storage backend interface for ECU key-value storage

#ifndef STORAGE_BACKEND_H
#define STORAGE_BACKEND_H

#include <stdint.h>
#include <string.h>

// Abstract storage backend interface
class StorageBackend {
public:
    virtual ~StorageBackend() {}
    
    // Core operations
    virtual bool begin() = 0;
    virtual bool writeData(uint16_t key_hash, const void* data, size_t size) = 0;
    virtual bool readData(uint16_t key_hash, void* data, size_t size) = 0;
    virtual bool deleteKey(uint16_t key_hash) = 0;
    virtual bool keyExists(uint16_t key_hash) = 0;
    
    // Storage information
    virtual size_t getFreeSpace() = 0;
    virtual size_t getTotalSpace() = 0;
    virtual uint32_t getWriteCount() = 0;
    virtual uint32_t getReadCount() = 0;
};

// Simple EEPROM-based storage backend
class EEPROMStorageBackend : public StorageBackend {
private:
    struct KeyEntry {
        uint16_t key_hash;      // CRC16 hash of the key string
        uint16_t data_offset;   // Offset in EEPROM where data starts
        uint16_t data_size;     // Size of data in bytes
        uint8_t valid;          // 1=valid entry, 0=deleted
        uint8_t checksum;       // Simple checksum for entry integrity
    } __attribute__((packed));
    
    static const int MAX_KEYS = 20;              // Maximum number of keys
    static const int EEPROM_START = 0;           // Start of EEPROM usage
    static const int HEADER_SIZE = sizeof(KeyEntry) * MAX_KEYS;
    static const int DATA_START = HEADER_SIZE;   // Where data storage begins
    static const int EEPROM_SIZE = 1080;         // Teensy 4.x EEPROM size
    
    uint32_t write_count;
    uint32_t read_count;
    
    // Helper functions
    bool findKeyEntry(uint16_t key_hash, KeyEntry* entry, int* index);
    bool findFreeEntry(int* index);
    uint16_t findFreeDataSpace(uint16_t size);
    uint8_t calculateChecksum(const KeyEntry* entry);
    void defragmentStorage();
    
public:
    EEPROMStorageBackend();
    
    // StorageBackend interface
    bool begin() override;
    bool writeData(uint16_t key_hash, const void* data, size_t size) override;
    bool readData(uint16_t key_hash, void* data, size_t size) override;
    bool deleteKey(uint16_t key_hash) override;
    bool keyExists(uint16_t key_hash) override;
    
    // Storage information
    size_t getFreeSpace() override;
    size_t getTotalSpace() override;
    uint32_t getWriteCount() override { return write_count; }
    uint32_t getReadCount() override { return read_count; }
    
    // EEPROM-specific functions
    void formatStorage();
    void printStorageInfo();
    bool verifyIntegrity();
};

#endif 