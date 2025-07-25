// test_w25q128_simple.cpp
// Simple test to isolate W25Q128 storage backend issues

#include <iostream>
#include <cassert>
#include <cstring>
#include "../mock_arduino.h"
#include "../../w25q128_storage_backend.h"
#include "../../ecu_config.h"

// Mock ECU configuration for testing
ECUConfiguration createTestConfig() {
    ECUConfiguration config = {};
    
    // Set ECU type
    config.ecu_type = ECU_TRANSMISSION;
    strcpy(config.ecu_name, "Test ECU");
    strcpy(config.firmware_version, "1.0.0");
    config.serial_number = 12345;
    
    // Set SPI configuration for W25Q128
    config.spi.mosi_pin = 11;
    config.spi.miso_pin = 12;
    config.spi.sck_pin = 13;
    config.spi.qspi_flash.cs_pin = 10;
    config.spi.qspi_flash.frequency = 10000000; // 10MHz
    config.spi.qspi_flash.mode = 0;
    config.spi.qspi_flash.bit_order = 0; // MSBFIRST equivalent
    config.spi.qspi_flash.enabled = true;
    
    return config;
}

// Simple test data
struct SimpleData {
    uint32_t id;
    float value;
    uint32_t timestamp;
};

int main() {
    std::cout << "=== Simple W25Q128 Test ===" << std::endl;
    
    try {
        ECUConfiguration config = createTestConfig();
        W25Q128StorageBackend backend(config);
        
        std::cout << "Initializing backend..." << std::endl;
        bool init_result = backend.begin();
        assert(init_result && "Backend initialization failed");
        std::cout << "✓ Backend initialized" << std::endl;
        
        // Test basic write
        std::cout << "Testing write..." << std::endl;
        SimpleData test_data = {0x10300001, 42.5f, 1234567890};
        bool write_result = backend.writeData(test_data.id, &test_data, sizeof(test_data));
        assert(write_result && "Write failed");
        std::cout << "✓ Write successful" << std::endl;
        
        // Test basic read
        std::cout << "Testing read..." << std::endl;
        SimpleData read_data;
        bool read_result = backend.readData(test_data.id, &read_data, sizeof(read_data));
        assert(read_result && "Read failed");
        std::cout << "✓ Read successful" << std::endl;
        
        // Verify data
        assert(memcmp(&test_data, &read_data, sizeof(test_data)) == 0);
        std::cout << "✓ Data integrity verified" << std::endl;
        
        // Test hasData
        std::cout << "Testing hasData..." << std::endl;
        assert(backend.hasData(test_data.id));
        std::cout << "✓ hasData works" << std::endl;
        
        // Test storage stats
        std::cout << "Testing storage stats..." << std::endl;
        uint32_t count = backend.getStoredKeyCount();
        assert(count == 1);
        std::cout << "✓ Storage count: " << count << std::endl;
        
        std::cout << "\n🎉 Simple test completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }
} 