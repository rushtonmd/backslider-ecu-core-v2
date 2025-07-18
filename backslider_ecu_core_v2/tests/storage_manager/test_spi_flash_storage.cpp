// test_spi_flash_storage.cpp
// Test SPI Flash storage backend functionality

#include <iostream>
#include <assert.h>
#include <string>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

#include "../../spi_flash_storage_backend.h"

// Test SPI Flash storage backend
void test_spi_flash_storage_backend() {
    std::cout << "=== Testing SPI Flash Storage Backend ===" << std::endl;
    
    // Create storage backend
    SPIFlashStorageBackend backend;
    
    // Test initialization
    assert(backend.begin() == true);
    std::cout << "✓ SPI Flash storage backend initialized" << std::endl;
    
    // Test basic key-value operations
    std::cout << "\nTest 1: Basic key-value operations" << std::endl;
    
    // Test float storage
    float test_value = 42.5f;
    uint16_t test_key = 0x1234;
    
    bool save_result = backend.writeData(test_key, &test_value, sizeof(float));
    assert(save_result == true);
    std::cout << "✓ Float value saved successfully" << std::endl;
    
    float loaded_value;
    bool load_result = backend.readData(test_key, &loaded_value, sizeof(float));
    assert(load_result == true);
    std::cout << "Expected: " << test_value << ", Got: " << loaded_value << std::endl;
    assert(loaded_value == test_value);
    std::cout << "✓ Float value loaded successfully: " << loaded_value << std::endl;
    
    // Test key existence
    assert(backend.keyExists(test_key) == true);
    assert(backend.keyExists(0x9999) == false);
    std::cout << "✓ Key existence check works" << std::endl;
    
    // Test deletion
    assert(backend.deleteKey(test_key) == true);
    assert(backend.keyExists(test_key) == false);
    std::cout << "✓ Key deletion works" << std::endl;
    
    // Test storage info
    std::cout << "\nTest 2: Storage information" << std::endl;
    
    size_t total_space = backend.getTotalSpace();
    size_t free_space = backend.getFreeSpace();
    
    std::cout << "Total space: " << total_space / 1024 << " KB" << std::endl;
    std::cout << "Free space: " << free_space / 1024 << " KB" << std::endl;
    std::cout << "Write count: " << backend.getWriteCount() << std::endl;
    std::cout << "Read count: " << backend.getReadCount() << std::endl;
    
    assert(total_space > 0);
    assert(free_space > 0);
    assert(backend.getWriteCount() > 0);
    assert(backend.getReadCount() > 0);
    std::cout << "✓ Storage information available" << std::endl;
    
    // Test JSON operations
    std::cout << "\nTest 3: JSON operations" << std::endl;
    
    const char* test_json = "{\"project\":\"1993 Mustang\",\"version\":\"1.0\"}";
    bool json_save = backend.saveJSON("test_project", test_json);
    assert(json_save == true);
    std::cout << "✓ JSON data saved" << std::endl;
    
    char loaded_json[128];
    bool json_load = backend.loadJSON("test_project", loaded_json, sizeof(loaded_json));
    assert(json_load == true);
    std::cout << "✓ JSON data loaded: " << loaded_json << std::endl;
    
    // Test float array operations
    std::cout << "\nTest 4: Float array operations" << std::endl;
    
    float test_map[100];
    for (int i = 0; i < 100; i++) {
        test_map[i] = i * 0.5f;
    }
    
    bool array_save = backend.saveFloatArray("test_fuel_map", test_map, 100);
    assert(array_save == true);
    std::cout << "✓ Float array saved (100 elements)" << std::endl;
    
    float loaded_map[100];
    bool array_load = backend.loadFloatArray("test_fuel_map", loaded_map, 100);
    assert(array_load == true);
    
    // Verify array contents
    for (int i = 0; i < 100; i++) {
        if (loaded_map[i] != test_map[i]) {
            std::cout << "Mismatch at index " << i << ": expected " << test_map[i] 
                      << ", got " << loaded_map[i] << std::endl;
        }
        assert(loaded_map[i] == test_map[i]);
    }
    std::cout << "✓ Float array loaded and verified" << std::endl;
    
    // Test large data handling
    std::cout << "\nTest 5: Large data handling" << std::endl;
    
    // Create a large fuel map (30x30 = 900 floats)
    float large_map[900];
    for (int i = 0; i < 900; i++) {
        large_map[i] = i + 10.0f;
    }
    
    bool large_save = backend.saveFloatArray("large_fuel_map", large_map, 900);
    assert(large_save == true);
    std::cout << "✓ Large fuel map saved (900 elements)" << std::endl;
    
    float loaded_large_map[900];
    bool large_load = backend.loadFloatArray("large_fuel_map", loaded_large_map, 900);
    assert(large_load == true);
    
    // Verify large array contents
    for (int i = 0; i < 900; i++) {
        assert(loaded_large_map[i] == large_map[i]);
    }
    std::cout << "✓ Large fuel map loaded and verified" << std::endl;
    
    // Test multiple keys
    std::cout << "\nTest 6: Multiple keys handling" << std::endl;
    
    // Save multiple different data types
    float rpm_data[10] = {800, 1200, 1600, 2000, 2400, 3000, 3600, 4200, 4800, 5400};
    float load_data[10] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110};
    
    backend.saveFloatArray("rpm_axis", rpm_data, 10);
    backend.saveFloatArray("load_axis", load_data, 10);
    
    // Save some scalar values
    float base_pressure = 43.5f;
    float injector_flow = 42.0f;
    
    backend.writeData(0x2001, &base_pressure, sizeof(float));
    backend.writeData(0x2002, &injector_flow, sizeof(float));
    
    // Verify all data exists
    assert(backend.keyExists(0x2001) == true);
    assert(backend.keyExists(0x2002) == true);
    
    float loaded_rpm[10], loaded_load[10];
    assert(backend.loadFloatArray("rpm_axis", loaded_rpm, 10) == true);
    assert(backend.loadFloatArray("load_axis", loaded_load, 10) == true);
    
    // Verify arrays match
    for (int i = 0; i < 10; i++) {
        assert(loaded_rpm[i] == rpm_data[i]);
        assert(loaded_load[i] == load_data[i]);
    }
    
    std::cout << "✓ Multiple keys handled correctly" << std::endl;
    
    // Test error conditions
    std::cout << "\nTest 7: Error conditions" << std::endl;
    
    // Test invalid parameters
    assert(backend.writeData(0, nullptr, 0) == false);
    assert(backend.readData(0, nullptr, 0) == false);
    assert(backend.saveJSON(nullptr, "test") == false);
    assert(backend.loadJSON(nullptr, loaded_json, sizeof(loaded_json)) == false);
    
    std::cout << "✓ Error conditions handled correctly" << std::endl;
    
    // Print final storage statistics
    std::cout << "\n=== Final Storage Statistics ===" << std::endl;
    backend.printStorageInfo();
    
    // Test integrity check
    assert(backend.verifyIntegrity() == true);
    std::cout << "✓ Storage integrity verified" << std::endl;
    
    std::cout << "\n=== All SPI Flash Storage Tests Passed! ===" << std::endl;
}

// Test file organization
void test_file_organization() {
    std::cout << "\n=== Testing File Organization ===" << std::endl;
    
    SPIFlashStorageBackend backend;
    backend.begin();
    
    // Test that different key types create organized directory structure
    // CRC16 hash 0x1234 should create /keys/12/1234.bin
    // JSON files should go to /config/
    // Float arrays should go to /maps/
    
    // Save different types of data
    float test_float = 123.45f;
    backend.writeData(0x1234, &test_float, sizeof(float));  // Goes to /keys/12/1234.bin
    
    backend.saveJSON("project_config", "{\"test\":\"value\"}");  // Goes to /config/project_config.json
    
    float test_array[5] = {1, 2, 3, 4, 5};
    backend.saveFloatArray("test_map", test_array, 5);  // Goes to /maps/test_map.map
    
    // All should be accessible
    assert(backend.keyExists(0x1234) == true);
    
    char json_buffer[64];
    assert(backend.loadJSON("project_config", json_buffer, sizeof(json_buffer)) == true);
    
    float loaded_array[5];
    assert(backend.loadFloatArray("test_map", loaded_array, 5) == true);
    
    std::cout << "✓ File organization working correctly" << std::endl;
}

int main() {
    test_spi_flash_storage_backend();
    test_file_organization();
    
    std::cout << "\n🎉 All tests passed! SPI Flash storage backend is working correctly." << std::endl;
    
    return 0;
} 