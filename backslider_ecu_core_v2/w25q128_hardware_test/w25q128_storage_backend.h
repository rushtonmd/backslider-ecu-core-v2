// w25q128_storage_backend.h
// Optimized W25Q128 SPI Flash storage backend for high-performance key/value storage
// Uses Extended CAN ID as storage key for optimal performance

#ifndef W25Q128_STORAGE_BACKEND_H
#define W25Q128_STORAGE_BACKEND_H

#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "storage_backend.h"
#include "ecu_config.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// =============================================================================
// W25Q128 Flash Specifications
// =============================================================================
#define W25Q128_FLASH_SIZE           (16 * 1024 * 1024)  // 16MB (128Mbit)
#define W25Q128_SECTOR_SIZE          4096                 // 4KB sectors
#define W25Q128_PAGE_SIZE            256                  // 256 bytes per page
#define W25Q128_SECTORS_PER_BLOCK    16                   // 64KB blocks
#define W25Q128_BLOCK_SIZE           (W25Q128_SECTOR_SIZE * W25Q128_SECTORS_PER_BLOCK)

// =============================================================================
// W25Q128 Commands
// =============================================================================
#define W25Q128_CMD_WRITE_ENABLE     0x06
#define W25Q128_CMD_WRITE_DISABLE    0x04
#define W25Q128_CMD_READ_STATUS1     0x05
#define W25Q128_CMD_READ_STATUS2     0x35
#define W25Q128_CMD_WRITE_STATUS     0x01
#define W25Q128_CMD_PAGE_PROGRAM     0x02
#define W25Q128_CMD_QUAD_PAGE_PROGRAM 0x32
#define W25Q128_CMD_BLOCK_ERASE_64K  0xD8
#define W25Q128_CMD_BLOCK_ERASE_32K  0x52
#define W25Q128_CMD_SECTOR_ERASE_4K  0x20
#define W25Q128_CMD_CHIP_ERASE       0xC7
#define W25Q128_CMD_ERASE_SUSPEND    0x75
#define W25Q128_CMD_ERASE_RESUME     0x7A
#define W25Q128_CMD_POWER_DOWN       0xB9
#define W25Q128_CMD_HIGH_PERFORMANCE 0xA3
#define W25Q128_CMD_CONTINUOUS_READ  0xFF
#define W25Q128_CMD_READ_DATA        0x03
#define W25Q128_CMD_FAST_READ        0x0B
#define W25Q128_CMD_FAST_READ_DUAL   0x3B
#define W25Q128_CMD_FAST_READ_QUAD   0x6B
#define W25Q128_CMD_READ_ID          0x90
#define W25Q128_CMD_READ_JEDEC_ID    0x9F

// =============================================================================
// Storage Entry Structure (Optimized for W25Q128)
// =============================================================================
struct __attribute__((packed)) StorageEntry {
    uint32_t magic;           // Magic number for validation (0x57463132 = "WF12")
    uint32_t storage_key;     // Extended CAN ID as key
    uint32_t data_size;       // Size of data in bytes
    uint32_t checksum;        // CRC32 checksum of data
    uint32_t timestamp;       // Timestamp when written
    uint8_t data[];           // Variable length data
};

// =============================================================================
// W25Q128 Storage Backend Class
// =============================================================================
class W25Q128StorageBackend : public StorageBackend {
public:
    // Constructor with ECU configuration
    W25Q128StorageBackend(const ECUConfiguration& config);
    virtual ~W25Q128StorageBackend();
    
    // StorageBackend interface implementation
    bool begin() override;
    bool end() override;
    
    // High-performance data access using extended CAN ID as key
    bool readData(uint32_t storage_key, void* data, size_t dataSize) override;
    bool writeData(uint32_t storage_key, const void* data, size_t dataSize) override;
    bool deleteData(uint32_t storage_key) override;
    bool hasData(uint32_t storage_key) override;
    
    // Storage management
    uint32_t getTotalSpace() override;
    uint32_t getFreeSpace() override;
    uint32_t getUsedSpace() override;
    
    // Maintenance
    void sync() override;
    void flush() override;
    
    // Optional iteration support
    uint32_t getStoredKeyCount() override;
    bool getStoredKey(uint32_t index, uint32_t* storage_key) override;
    
    // Debug/information
    void printDebugInfo() override;
    
    // W25Q128 specific methods
    bool initializeFlash();
    bool verifyFlash();
    void formatFlash();
    uint32_t getFlashID();
    void printFlashInfo();
    bool isFlashReady();
    
    // Performance optimization methods
    void enableWriteCache(bool enable);
    void setCacheSize(uint32_t size);
    uint32_t getCacheHitRate();
    void clearCache();
    
    // Error handling
    uint32_t getErrorCount();
    void clearErrors();
    const char* getLastError();

private:
    // Configuration
    const ECUConfiguration& ecu_config;
    uint8_t cs_pin;
    uint32_t spi_frequency;
    
    // Flash state
    bool flash_initialized;
    uint32_t flash_id;
    uint32_t total_sectors;
    uint32_t used_sectors;
    
    // Performance cache
    struct CacheEntry {
        uint32_t storage_key;
        std::vector<uint8_t> data;
        uint32_t timestamp;
        bool dirty;
    };
    
    std::unordered_map<uint32_t, CacheEntry> write_cache;
    bool cache_enabled;
    uint32_t cache_size_limit;
    uint32_t cache_hits;
    uint32_t cache_misses;
    
    // Error tracking
    uint32_t error_count;
    char last_error[128];
    
    // SPI communication methods
    void selectChip();
    void deselectChip();
    uint8_t spiTransfer(uint8_t data);
    void spiTransfer(const uint8_t* data, uint8_t* result, size_t length);
    void spiWrite(const uint8_t* data, size_t length);
    
    // Flash command methods
    void writeEnable();
    void writeDisable();
    bool waitForWriteComplete();
    uint8_t readStatus();
    void writeStatus(uint8_t status);
    
    // Flash operation methods
    bool readPage(uint32_t page_address, uint8_t* buffer, size_t length);
    bool writePage(uint32_t page_address, const uint8_t* buffer, size_t length);
    bool eraseSector(uint32_t sector_address);
    bool eraseBlock(uint32_t block_address);
    
    // Storage management methods
    uint32_t findStorageEntry(uint32_t storage_key);
    bool writeStorageEntry(uint32_t storage_key, const void* data, size_t dataSize);
    bool readStorageEntry(uint32_t storage_key, void* data, size_t dataSize);
    bool deleteStorageEntry(uint32_t storage_key);
    
    // Utility methods
    uint32_t calculateChecksum(const void* data, size_t length);
    uint32_t findFreeSector();
    void updateSectorMap();
    void rebuildIndex();
    
    // Sector allocation tracking
    std::vector<bool> sector_allocated;
    std::unordered_map<uint32_t, uint32_t> key_to_sector;
    
    // Constants
    static const uint32_t STORAGE_MAGIC = 0x57463132;  // "WF12"
    static const uint32_t MAX_CACHE_SIZE = 1024 * 1024; // 1MB cache limit
    static const uint32_t MAX_DATA_SIZE = W25Q128_PAGE_SIZE - sizeof(StorageEntry);
};

#endif // W25Q128_STORAGE_BACKEND_H 