// storage_manager_test.cpp
// Test storage manager with extended CAN ID architecture

#include <iostream>
#include <cassert>
#include <cstring>
#include <cstdint>
#include <string>

#include "../mock_arduino.h"
#include "../../storage_manager.h"
#include "../../spi_flash_storage_backend.h"
#include "../../msg_bus.h"
#include "../../msg_definitions.h"

// Mock global time for testing
extern uint32_t mock_millis_time;

// Mock Serial for desktop testing
// MockSerial Serial is defined in mock_arduino.cpp

// Test storage manager
StorageManager* test_storage_manager = nullptr;

// Test message response handlers
bool save_response_received = false;
bool load_response_received = false;
float load_response_value = 0.0f;
uint32_t load_response_key = 0;

void test_save_response_handler(const CANMessage* msg) {
    auto response = MSG_UNPACK_STORAGE_SAVE_RESPONSE(msg);
    save_response_received = true;
    std::cout << "Save response received for key 0x" << std::hex << response->storage_key << std::dec 
              << " success=" << (int)response->success << std::endl;
}

void test_load_response_handler(const CANMessage* msg) {
    auto response = MSG_UNPACK_STORAGE_LOAD_RESPONSE(msg);
    load_response_received = true;
    load_response_value = response->value;
    load_response_key = response->storage_key;
    std::cout << "Load response received for key 0x" << std::hex << response->storage_key << std::dec 
              << " value=" << response->value << std::endl;
}

void test_basic_storage_operations() {
    std::cout << "\n=== Test 1: Basic Storage Operations ===\n" << std::endl;
    
    // Test 1: Direct save/load
    std::cout << "Test 1: Direct save/load operation" << std::endl;
    
    // Use extended CAN ID directly as storage key
    uint32_t fuel_map_key = MSG_FUEL_MAP_CELL(5, 10);
    float test_value = 42.5f;
    
    bool save_result = test_storage_manager->save_float(fuel_map_key, test_value);
    assert(save_result == true);
    std::cout << "✓ Direct save operation successful" << std::endl;
    
    float loaded_value;
    bool load_result = test_storage_manager->load_float(fuel_map_key, &loaded_value);
    assert(load_result == true);
    assert(loaded_value == test_value);
    std::cout << "✓ Direct load operation successful, value: " << loaded_value << std::endl;
    
    // Test 2: Message-driven save
    std::cout << "\nTest 2: Message-driven save operation" << std::endl;
    
    // Create save message using new extended CAN ID structure
    CANMessage save_msg;
    uint32_t config_key = MSG_CONFIG_FUEL_BASE_PRESSURE;
    float save_value = 14.7f;
    
    MSG_PACK_STORAGE_SAVE_FLOAT(&save_msg, config_key, save_value);
    
    // Send message via message bus
    g_message_bus.publish(MSG_STORAGE_SAVE, save_msg.buf, save_msg.len);
    
    // Process message bus to handle the save
    g_message_bus.process();
    
    // Update storage manager
    test_storage_manager->update();
    
    std::cout << "✓ Message-driven save completed" << std::endl;
    
    // Test 3: Message-driven load
    std::cout << "\nTest 3: Message-driven load operation" << std::endl;
    
    // Create load message using new extended CAN ID structure
    CANMessage load_msg;
    float default_value = 0.0f;
    
    MSG_PACK_STORAGE_LOAD_FLOAT(&load_msg, config_key, default_value);
    
    // Send message via message bus
    g_message_bus.publish(MSG_STORAGE_LOAD, load_msg.buf, load_msg.len);
    
    // Process message bus to handle the load
    g_message_bus.process();
    
    // Update storage manager
    test_storage_manager->update();
    
    std::cout << "✓ Message-driven load completed" << std::endl;
    
    // Test 4: Cache functionality
    std::cout << "\nTest 4: Cache functionality" << std::endl;
    
    // Load the same value multiple times to test caching
    float cached_value1, cached_value2;
    
    bool cache_result1 = test_storage_manager->load_float(fuel_map_key, &cached_value1);
    bool cache_result2 = test_storage_manager->load_float(fuel_map_key, &cached_value2);
    
    assert(cache_result1 == true);
    assert(cache_result2 == true);
    assert(cached_value1 == cached_value2);
    assert(cached_value1 == test_value);
    
    std::cout << "✓ Cache functionality working correctly" << std::endl;
    
    // Test 5: Extended CAN ID breakdown
    std::cout << "\nTest 5: Extended CAN ID breakdown" << std::endl;
    
    // Test various extended CAN IDs
    uint32_t test_keys[] = {
        MSG_ENGINE_RPM,
        MSG_FUEL_MAP_CELL(10, 15),
        MSG_IGNITION_MAP_CELL(5, 8),
        MSG_CONFIG_FUEL_BASE_PRESSURE,
        MSG_TRANS_FLUID_TEMP
    };
    
    for (size_t i = 0; i < sizeof(test_keys) / sizeof(test_keys[0]); i++) {
        uint32_t key = test_keys[i];
        float value = 100.0f + i;
        
        // Save and load
        bool save_ok = test_storage_manager->save_float(key, value);
        assert(save_ok == true);
        
        float loaded;
        bool load_ok = test_storage_manager->load_float(key, &loaded);
        assert(load_ok == true);
        assert(loaded == value);
        
        // Print breakdown
        uint8_t ecu_base = GET_ECU_BASE(key) >> 28;
        uint8_t subsystem = GET_SUBSYSTEM(key) >> 20;
        uint32_t parameter = GET_PARAMETER(key);
        
        std::cout << "  Key 0x" << std::hex << key << std::dec
                  << " -> ECU=" << (int)ecu_base 
                  << " SUB=" << (int)subsystem
                  << " PARAM=" << parameter
                  << " Value=" << value << std::endl;
    }
    
    std::cout << "✓ Extended CAN ID breakdown test passed" << std::endl;
}

void test_message_responses() {
    std::cout << "\n=== Test 2: Message Response Handling ===\n" << std::endl;
    
    // Subscribe to response messages
    g_message_bus.subscribe(MSG_STORAGE_SAVE_RESPONSE, test_save_response_handler);
    g_message_bus.subscribe(MSG_STORAGE_LOAD_RESPONSE, test_load_response_handler);
    
    // Reset response flags
    save_response_received = false;
    load_response_received = false;
    
    // Send save message
    CANMessage save_msg;
    uint32_t test_key = MSG_CONFIG_IGNITION_BASE_TIMING;
    float test_value = 15.0f;
    
    MSG_PACK_STORAGE_SAVE_FLOAT(&save_msg, test_key, test_value);
    g_message_bus.publish(MSG_STORAGE_SAVE, save_msg.buf, save_msg.len);
    
    // Process messages
    g_message_bus.process();
    test_storage_manager->update();
    g_message_bus.process();  // Process any response messages
    
    // Check save response
    assert(save_response_received == true);
    std::cout << "✓ Save response received" << std::endl;
    
    // Send load message
    CANMessage load_msg;
    float default_val = 0.0f;
    
    MSG_PACK_STORAGE_LOAD_FLOAT(&load_msg, test_key, default_val);
    g_message_bus.publish(MSG_STORAGE_LOAD, load_msg.buf, load_msg.len);
    
    // Process messages
    g_message_bus.process();
    test_storage_manager->update();
    g_message_bus.process();  // Process any response messages
    
    // Check load response
    assert(load_response_received == true);
    assert(load_response_key == test_key);
    assert(load_response_value == test_value);
    std::cout << "✓ Load response received with correct value" << std::endl;
}

void test_cache_performance() {
    std::cout << "\n=== Test 3: Cache Performance ===\n" << std::endl;
    
    // Fill cache with multiple values
    std::cout << "Filling cache with test data..." << std::endl;
    
    for (int i = 0; i < 15; i++) {
        uint32_t key = MSG_FUEL_MAP_CELL(i, i + 1);
        float value = 1000.0f + i;
        
        bool result = test_storage_manager->save_float(key, value);
        assert(result == true);
    }
    
    std::cout << "✓ Cache filled successfully" << std::endl;
    
    // Test cache hit performance
    std::cout << "Testing cache hit performance..." << std::endl;
    
    for (int i = 0; i < 15; i++) {
        uint32_t key = MSG_FUEL_MAP_CELL(i, i + 1);
        float expected_value = 1000.0f + i;
        float loaded_value;
        
        bool result = test_storage_manager->load_float(key, &loaded_value);
        assert(result == true);
        assert(loaded_value == expected_value);
    }
    
    std::cout << "✓ Cache performance test passed" << std::endl;
    
    // Print cache statistics
    test_storage_manager->print_cache_info();
}

void test_map_cell_macros() {
    std::cout << "\n=== Test 4: Map Cell Macros ===\n" << std::endl;
    
    // Test fuel map cell macro
    uint32_t fuel_cell_key = MSG_FUEL_MAP_CELL(10, 20);
    float fuel_value = 12.5f;
    
    bool save_result = test_storage_manager->save_float(fuel_cell_key, fuel_value);
    assert(save_result == true);
    
    float loaded_fuel_value;
    bool load_result = test_storage_manager->load_float(fuel_cell_key, &loaded_fuel_value);
    assert(load_result == true);
    assert(loaded_fuel_value == fuel_value);
    
    std::cout << "✓ Fuel map cell macro test passed" << std::endl;
    
    // Test ignition map cell macro
    uint32_t ignition_cell_key = MSG_IGNITION_MAP_CELL(5, 15);
    float ignition_value = 25.0f;
    
    save_result = test_storage_manager->save_float(ignition_cell_key, ignition_value);
    assert(save_result == true);
    
    float loaded_ignition_value;
    load_result = test_storage_manager->load_float(ignition_cell_key, &loaded_ignition_value);
    assert(load_result == true);
    assert(loaded_ignition_value == ignition_value);
    
    std::cout << "✓ Ignition map cell macro test passed" << std::endl;
    
    // Test boost map cell macro
    uint32_t boost_cell_key = MSG_BOOST_MAP_CELL(3, 7);
    float boost_value = 18.0f;
    
    save_result = test_storage_manager->save_float(boost_cell_key, boost_value);
    assert(save_result == true);
    
    float loaded_boost_value;
    load_result = test_storage_manager->load_float(boost_cell_key, &loaded_boost_value);
    assert(load_result == true);
    assert(loaded_boost_value == boost_value);
    
    std::cout << "✓ Boost map cell macro test passed" << std::endl;
}

int main() {
    std::cout << "=== Storage Manager Test Suite (Extended CAN ID Architecture) ===" << std::endl;
    
    // Initialize message bus
    g_message_bus.init();
    
    // Initialize storage backend
    SPIFlashStorageBackend storage_backend;
    
    // Initialize storage manager
    test_storage_manager = new StorageManager(&storage_backend);
    
    if (!test_storage_manager->init()) {
        std::cerr << "Failed to initialize storage manager" << std::endl;
        return 1;
    }
    
    std::cout << "✓ Storage manager initialized successfully" << std::endl;
    
    // Run tests
    try {
        test_basic_storage_operations();
        test_message_responses();
        test_cache_performance();
        test_map_cell_macros();
        
        std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        delete test_storage_manager;
        return 1;
    }
    
    // Cleanup
    delete test_storage_manager;
    
    return 0;
} 