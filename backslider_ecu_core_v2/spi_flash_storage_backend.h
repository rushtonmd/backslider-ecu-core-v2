// spi_flash_storage_backend.h
// SPI Flash storage backend using Adafruit W25Q128 + FAT32

#ifndef SPI_FLASH_STORAGE_BACKEND_H
#define SPI_FLASH_STORAGE_BACKEND_H

#include "storage_backend.h"

#if defined(ARDUINO) && !defined(TESTING)
    #include <SPI.h>
    #include "SdFat_Adafruit_Fork.h"
    #include "Adafruit_SPIFlash.h"
#endif

#ifdef TESTING
    // Mock implementations for testing
    #include <map>
    #include <vector>
    #include <iostream>
    
    class MockFatVolume {
    private:
        static std::map<std::string, std::vector<uint8_t>> mock_files;
        
    public:
        bool begin(void* flash) { return true; }
        
        class MockFile {
        private:
            std::string filename;
            size_t read_pos;
            bool valid;
            
        public:
            MockFile(const std::string& name = "", int file_mode = 0) 
                : filename(name), read_pos(0), valid(!name.empty()) {}
                
            bool open(const char* filename, int mode = 0) { return true; }
            
            size_t write(const void* data, size_t size) {
                if (!valid) return 0;
                
                const uint8_t* byte_data = static_cast<const uint8_t*>(data);
                
                // Append data to file (correct behavior for sequential writes)
                std::vector<uint8_t>& file_data = mock_files[filename];
                size_t old_size = file_data.size();
                file_data.insert(file_data.end(), byte_data, byte_data + size);
                
                // Debug output removed for clean test runs
                
                return size;
            }
            
            size_t read(void* data, size_t size) {
                if (!valid || mock_files.find(filename) == mock_files.end()) {
                    return 0;
                }
                
                std::vector<uint8_t>& file_data = mock_files[filename];
                size_t bytes_to_read = std::min(size, file_data.size() - read_pos);
                
                #ifndef ARDUINO
                std::cout << "MockFile::read: filename=" << filename << ", size=" << size 
                          << ", read_pos=" << read_pos << ", file_size=" << file_data.size() 
                          << ", bytes_to_read=" << bytes_to_read << std::endl;
                #endif
                
                if (bytes_to_read > 0) {
                    memcpy(data, &file_data[read_pos], bytes_to_read);
                    read_pos += bytes_to_read;
                }
                
                return bytes_to_read;
            }
            
            size_t size() {
                if (!valid || mock_files.find(filename) == mock_files.end()) {
                    return 0;
                }
                return mock_files[filename].size();
            }
            
            void close() { read_pos = 0; }
            bool available() { return false; }
            operator bool() { return valid; }
        };
        
        MockFile open(const char* filename, int mode = 0) { 
            return MockFile(std::string(filename), mode); 
        }
        
        bool exists(const char* filename) { 
            return mock_files.find(std::string(filename)) != mock_files.end(); 
        }
        
        bool remove(const char* filename) { 
            mock_files.erase(std::string(filename)); 
            return true; 
        }
        
        bool format(void* flash) { 
            mock_files.clear(); 
            return true; 
        }
    };
    
    class MockSPIFlash {
    public:
        bool begin() { return true; }
        size_t size() { return 16 * 1024 * 1024; }  // 16MB for testing
    };
    
    #define FatVolume MockFatVolume
    #define File32 MockFatVolume::MockFile
    #define Adafruit_SPIFlash MockSPIFlash
    #define FILE_WRITE 1
    #define FILE_READ 0
#endif

class SPIFlashStorageBackend : public StorageBackend {
private:
    #if defined(ARDUINO) && !defined(TESTING)
    // Flash hardware and filesystem
    Adafruit_SPIFlash* flash;
    FatVolume fatfs;
    
    // Configuration
    static const int CHIP_SELECT_PIN = 10;  // CS pin for SPI flash
    static const uint32_t SPI_SPEED = 25000000;  // 25MHz SPI speed
    #else
    // Mock objects for testing
    MockSPIFlash* flash;
    MockFatVolume fatfs;
    #endif
    
    // Statistics
    uint32_t write_count;
    uint32_t read_count;
    uint32_t format_count;
    
    // Helper methods
    void keyHashToFilename(uint16_t key_hash, char* filename, size_t max_len);
    bool ensureDirectoryExists(const char* path);
    bool formatFilesystem();
    
public:
    SPIFlashStorageBackend();
    virtual ~SPIFlashStorageBackend();
    
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
    
    // SPI Flash specific methods
    bool formatStorage();
    void printStorageInfo();
    bool verifyIntegrity();
    
    // Directory organization
    bool createDirectory(const char* path);
    bool listDirectory(const char* path, void (*callback)(const char* filename, size_t size));
    
    // Bulk operations
    bool saveStructuredData(const char* key, const void* data, size_t size, const char* extension = ".bin");
    bool loadStructuredData(const char* key, void* data, size_t size, const char* extension = ".bin");
    
    // Configuration-specific helpers
    bool saveJSON(const char* key, const char* json_data);
    bool loadJSON(const char* key, char* json_data, size_t max_size);
    bool saveFloatArray(const char* key, const float* values, size_t count);
    bool loadFloatArray(const char* key, float* values, size_t count);
};

#endif // SPI_FLASH_STORAGE_BACKEND_H 