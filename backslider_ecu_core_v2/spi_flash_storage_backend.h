// spi_flash_storage_backend.h
// SPI Flash storage backend using Extended CAN ID Architecture

#ifndef SPI_FLASH_STORAGE_BACKEND_H
#define SPI_FLASH_STORAGE_BACKEND_H

#include "storage_backend.h"
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

// =============================================================================
// SPI Flash Storage Backend
// =============================================================================

class SPIFlashStorageBackend : public StorageBackend {
public:
    SPIFlashStorageBackend();
    virtual ~SPIFlashStorageBackend();
    
    // StorageBackend interface implementation
    bool begin() override;
    bool end() override;
    
    // Data access using extended CAN ID as key
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
    
    // SPI Flash specific methods
    void printStorageInfo();
    bool verifyIntegrity();
    void formatStorage();
    
private:
    // Constants
    static const uint32_t FLASH_SIZE = 16 * 1024 * 1024;  // 16MB flash
    static const uint32_t SECTOR_SIZE = 4096;              // 4KB sectors
    static const uint32_t CACHE_SIZE = 256 * 1024;         // 256KB cache
    
    // Mock file system for testing
    class MockFile {
    public:
        std::vector<uint8_t> data;
        size_t position;
        bool is_open;
        
        MockFile() : position(0), is_open(false) {}
        
        bool open(const char* path, const char* mode) {
            is_open = true;
            position = 0;
            if (strchr(mode, 'w') || strchr(mode, 'a')) {
                // Write mode - clear data if 'w', keep if 'a'
                if (strchr(mode, 'w')) {
                    data.clear();
                }
            }
            return true;
        }
        
        void close() {
            is_open = false;
        }
        
        size_t read(void* buffer, size_t size) {
            if (!is_open || position >= data.size()) return 0;
            
            size_t bytes_to_read = std::min(size, data.size() - position);
            memcpy(buffer, data.data() + position, bytes_to_read);
            position += bytes_to_read;
            return bytes_to_read;
        }
        
        size_t write(const void* buffer, size_t size) {
            if (!is_open) return 0;
            
            // Extend data if needed
            if (position + size > data.size()) {
                data.resize(position + size);
            }
            
            const uint8_t* src = static_cast<const uint8_t*>(buffer);
            std::copy(src, src + size, data.begin() + position);
            position += size;
            return size;
        }
        
        size_t size() const {
            return data.size();
        }
        
        bool exists() const {
            return !data.empty();
        }
    };
    
    // Mock file system
    std::unordered_map<std::string, MockFile> mock_files;
    
    // Statistics
    uint32_t total_reads;
    uint32_t total_writes;
    uint32_t cache_hits;
    uint32_t cache_misses;
    
    // Helper methods
    std::string getFilePath(uint32_t storage_key);
    bool ensureDirectoryExists(const std::string& path);
    MockFile* openFile(const std::string& path, const char* mode);
    
    // Debug helpers
    void printExtendedCanId(uint32_t storage_key);
};

#endif 