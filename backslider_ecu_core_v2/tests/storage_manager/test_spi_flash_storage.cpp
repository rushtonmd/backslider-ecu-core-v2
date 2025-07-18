// test_spi_flash_storage.cpp
// Unit tests for SPI Flash Storage Backend - Extended CAN ID Architecture

#include <iostream>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include "../mock_arduino.h"
#include "../../spi_flash_storage_backend.h"
#include "../../msg_definitions.h"
extern uint32_t mock_millis_time;

void test_basic_storage_operations() {
    std::cout << "Testing basic storage operations..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test writing and reading float data using extended CAN ID
    uint32_t test_key = MSG_CONFIG_FUEL_BASE_PRESSURE;  // Use a real extended CAN ID
    float test_value = 43.5f;
    
    // Write data
    assert(backend.writeData(test_key, &test_value, sizeof(float)) == true);
    
    // Verify data exists
    assert(backend.hasData(test_key) == true);
    assert(backend.hasData(0x99999999) == false);
    
    // Delete data
    assert(backend.deleteData(test_key) == true);
    assert(backend.hasData(test_key) == false);
    
    std::cout << "✓ Basic storage operations passed" << std::endl;
}

void test_storage_statistics() {
    std::cout << "Testing storage statistics..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test basic statistics
    std::cout << "Total space: " << backend.getTotalSpace() << " bytes" << std::endl;
    std::cout << "Free space: " << backend.getFreeSpace() << " bytes" << std::endl;
    std::cout << "Used space: " << backend.getUsedSpace() << " bytes" << std::endl;
    
    // Write some data to change statistics
    uint32_t test_key = MSG_CONFIG_ENGINE_DISPLACEMENT;
    float test_value = 2.0f;
    assert(backend.writeData(test_key, &test_value, sizeof(float)) == true);
    
    // Check that used space increased
    assert(backend.getUsedSpace() > 0);
    
    std::cout << "✓ Storage statistics passed" << std::endl;
}

void test_configuration_storage() {
    std::cout << "Testing configuration parameter storage..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test storing various configuration parameters
    struct {
        uint32_t can_id;
        float value;
        const char* name;
    } test_configs[] = {
        {MSG_CONFIG_FUEL_BASE_PRESSURE, 43.5f, "Fuel Base Pressure"},
        {MSG_CONFIG_FUEL_INJECTOR_FLOW, 550.0f, "Injector Flow Rate"},
        {MSG_CONFIG_IGNITION_BASE_TIMING, 10.0f, "Base Ignition Timing"},
        {MSG_CONFIG_IGNITION_DWELL_TIME, 3.0f, "Ignition Dwell Time"},
        {MSG_CONFIG_ENGINE_DISPLACEMENT, 2.0f, "Engine Displacement"}
    };
    
    // Write all configuration parameters
    for (size_t i = 0; i < sizeof(test_configs) / sizeof(test_configs[0]); i++) {
        bool success = backend.writeData(test_configs[i].can_id, &test_configs[i].value, sizeof(float));
        assert(success == true);
        std::cout << "  Saved " << test_configs[i].name << ": " << test_configs[i].value << std::endl;
    }
    
    // Read back and verify all configuration parameters
    for (size_t i = 0; i < sizeof(test_configs) / sizeof(test_configs[0]); i++) {
        float read_value;
        bool success = backend.readData(test_configs[i].can_id, &read_value, sizeof(float));
        assert(success == true);
        assert(read_value == test_configs[i].value);
        std::cout << "  Verified " << test_configs[i].name << ": " << read_value << std::endl;
    }
    
    std::cout << "✓ Configuration storage passed" << std::endl;
}

void test_map_cell_storage() {
    std::cout << "Testing map cell storage..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test storing fuel map cells
    const int MAP_SIZE = 10;
    float fuel_map[MAP_SIZE][MAP_SIZE];
    
    // Fill with test data
    for (int row = 0; row < MAP_SIZE; row++) {
        for (int col = 0; col < MAP_SIZE; col++) {
            fuel_map[row][col] = 14.7f + (row * 0.1f) + (col * 0.01f);
        }
    }
    
    // Store each cell with its extended CAN ID
    for (int row = 0; row < MAP_SIZE; row++) {
        for (int col = 0; col < MAP_SIZE; col++) {
            uint32_t cell_id = MSG_FUEL_MAP_CELL(row, col);
            bool success = backend.writeData(cell_id, &fuel_map[row][col], sizeof(float));
            assert(success == true);
        }
    }
    
    // Read back and verify
    float read_map[MAP_SIZE][MAP_SIZE];
    for (int row = 0; row < MAP_SIZE; row++) {
        for (int col = 0; col < MAP_SIZE; col++) {
            uint32_t cell_id = MSG_FUEL_MAP_CELL(row, col);
            bool success = backend.readData(cell_id, &read_map[row][col], sizeof(float));
            assert(success == true);
            assert(read_map[row][col] == fuel_map[row][col]);
        }
    }
    
    std::cout << "✓ Map cell storage passed" << std::endl;
}

void test_large_data_storage() {
    std::cout << "Testing large data storage..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test storing large data block
    const size_t LARGE_SIZE = 1024;  // 1KB test data
    uint8_t large_data[LARGE_SIZE];
    uint8_t read_data[LARGE_SIZE];
    
    // Fill with test pattern
    for (size_t i = 0; i < LARGE_SIZE; i++) {
        large_data[i] = (uint8_t)(i % 256);
    }
    
    // Use a configuration parameter ID for large data
    uint32_t large_data_id = MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, 0x12345);
    
    // Store large data
    bool write_success = backend.writeData(large_data_id, large_data, LARGE_SIZE);
    assert(write_success == true);
    
    // Read back and verify
    bool read_success = backend.readData(large_data_id, read_data, LARGE_SIZE);
    assert(read_success == true);
    
    // Verify data integrity
    for (size_t i = 0; i < LARGE_SIZE; i++) {
        assert(read_data[i] == large_data[i]);
    }
    
    std::cout << "✓ Large data storage passed" << std::endl;
}

void test_key_iteration() {
    std::cout << "Testing key iteration..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Store some test data
    uint32_t test_keys[] = {
        MSG_CONFIG_FUEL_BASE_PRESSURE,
        MSG_CONFIG_IGNITION_BASE_TIMING,
        MSG_CONFIG_ENGINE_DISPLACEMENT,
        MSG_FUEL_MAP_CELL(5, 5),
        MSG_IGNITION_MAP_CELL(3, 7)
    };
    
    float test_values[] = {43.5f, 10.0f, 2.0f, 14.7f, 25.0f};
    
    // Store all test data
    for (size_t i = 0; i < sizeof(test_keys) / sizeof(test_keys[0]); i++) {
        bool success = backend.writeData(test_keys[i], &test_values[i], sizeof(float));
        assert(success == true);
    }
    
    // Test key iteration
    uint32_t key_count = backend.getStoredKeyCount();
    std::cout << "  Found " << key_count << " stored keys" << std::endl;
    
    assert(key_count >= 5);  // Should have at least our test keys
    
    // Iterate through all keys
    for (uint32_t i = 0; i < key_count; i++) {
        uint32_t storage_key;
        bool success = backend.getStoredKey(i, &storage_key);
        assert(success == true);
        
        // Verify we can read the data
        float value;
        bool read_success = backend.readData(storage_key, &value, sizeof(float));
        assert(read_success == true);
        
        std::cout << "  Key " << i << ": 0x" << std::hex << storage_key << std::dec 
                  << " = " << value << std::endl;
    }
    
    std::cout << "✓ Key iteration passed" << std::endl;
}

void test_extended_can_id_breakdown() {
    std::cout << "Testing extended CAN ID breakdown..." << std::endl;
    
    SPIFlashStorageBackend backend;
    assert(backend.begin() == true);
    
    // Test various extended CAN IDs
    uint32_t test_ids[] = {
        MSG_CONFIG_FUEL_BASE_PRESSURE,      // Config system
        MSG_FUEL_MAP_CELL(10, 15),          // Fuel map cell
        MSG_IGNITION_MAP_CELL(5, 8),        // Ignition map cell
        MSG_ENGINE_RPM,                     // Sensor reading
        MSG_IGNITION_TIMING,                // Control output
        MSG_HEARTBEAT                       // System message
    };
    
    for (size_t i = 0; i < sizeof(test_ids) / sizeof(test_ids[0]); i++) {
        uint32_t id = test_ids[i];
        float test_value = 100.0f + i;
        
        // Store data
        bool success = backend.writeData(id, &test_value, sizeof(float));
        assert(success == true);
        
        // Print breakdown
        uint8_t ecu_base = (id >> 28) & 0x0F;
        uint8_t subsystem = (id >> 20) & 0xFF;
        uint32_t parameter = id & 0xFFFFF;
        
        std::cout << "  ID 0x" << std::hex << id << std::dec
                  << " -> ECU=" << (int)ecu_base 
                  << " SUB=" << (int)subsystem
                  << " PARAM=" << parameter << std::endl;
    }
    
    std::cout << "✓ Extended CAN ID breakdown passed" << std::endl;
}

int main() {
    std::cout << "Starting SPI Flash Storage Backend Tests (Extended CAN ID)..." << std::endl;
    
    try {
        test_basic_storage_operations();
        test_storage_statistics();
        test_configuration_storage();
        test_map_cell_storage();
        test_large_data_storage();
        test_key_iteration();
        test_extended_can_id_breakdown();
        
        std::cout << "\n🎉 All SPI Flash Storage Backend tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
} 