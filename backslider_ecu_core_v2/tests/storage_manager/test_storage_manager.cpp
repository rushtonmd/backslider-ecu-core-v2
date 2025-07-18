// test_storage_manager.cpp
// Test storage manager functionality with message bus integration

#include <iostream>
#include <assert.h>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

#include "../../storage_manager.h"
#include "../../spi_flash_storage_backend.h"
#include "../../msg_bus.h"

// Mock millis for testing - use the one from mock_arduino.cpp
extern uint32_t mock_millis_time;

// Test storage manager functionality
void test_storage_manager() {
    std::cout << "=== Testing Storage Manager ===" << std::endl;
    
    // Create storage backend and manager
    SPIFlashStorageBackend backend;
    StorageManager storage_manager(&backend);
    
    // Initialize systems
    g_message_bus.init();
    assert(storage_manager.init() == true);
    
    std::cout << "✓ Storage manager initialized successfully" << std::endl;
    
    // Test 1: Direct save/load
    std::cout << "\nTest 1: Direct save/load operations" << std::endl;
    
    bool save_result = storage_manager.save_float("test_key", 42.5f);
    assert(save_result == true);
    std::cout << "✓ Direct save operation successful" << std::endl;
    
    float loaded_value;
    bool load_result = storage_manager.load_float("test_key", &loaded_value);
    assert(load_result == true);
    assert(loaded_value == 42.5f);
    std::cout << "✓ Direct load operation successful, value: " << loaded_value << std::endl;
    
    // Test 2: Message-driven save
    std::cout << "\nTest 2: Message-driven save operation" << std::endl;
    
    // Create save message
    storage_save_float_msg_t save_data;
    save_data.key_hash = crc16("fuel_map.5.10");
    save_data.value = 14.7f;
    save_data.priority = 0;
    save_data.sender_id = 1;
    
    // Send message via message bus
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    // Process message bus to handle the save
    g_message_bus.process();
    
    // Update storage manager
    storage_manager.update();
    
    std::cout << "✓ Message-driven save completed" << std::endl;
    
    // Test 3: Message-driven load
    std::cout << "\nTest 3: Message-driven load operation" << std::endl;
    
    // Create load message
    storage_load_float_msg_t load_data;
    load_data.key_hash = crc16("fuel_map.5.10");
    load_data.default_value = 0.0f;
    load_data.sender_id = 1;
    load_data.request_id = 42;
    
    // Send message via message bus
    g_message_bus.publish(MSG_STORAGE_LOAD_FLOAT, &load_data, sizeof(load_data));
    
    // Process message bus to handle the load
    g_message_bus.process();
    
    // Update storage manager
    storage_manager.update();
    
    std::cout << "✓ Message-driven load completed" << std::endl;
    
    // Test 4: Cache functionality
    std::cout << "\nTest 4: Cache functionality" << std::endl;
    
    // Save multiple values to test cache
    for (int i = 0; i < 5; i++) {
        std::string key = "cache_test_" + std::to_string(i);
        float value = i * 10.5f;
        
        bool result = storage_manager.save_float(key.c_str(), value);
        assert(result == true);
    }
    
    // Load values back (should hit cache)
    for (int i = 0; i < 5; i++) {
        std::string key = "cache_test_" + std::to_string(i);
        float expected_value = i * 10.5f;
        float actual_value;
        
        bool result = storage_manager.load_float(key.c_str(), &actual_value);
        assert(result == true);
        assert(actual_value == expected_value);
    }
    
    std::cout << "✓ Cache functionality working correctly" << std::endl;
    
    // Test 5: Statistics
    std::cout << "\nTest 5: Statistics tracking" << std::endl;
    
    assert(storage_manager.get_cache_hits() > 0);
    assert(storage_manager.get_disk_writes() > 0);
    
    std::cout << "Cache hits: " << storage_manager.get_cache_hits() << std::endl;
    std::cout << "Cache misses: " << storage_manager.get_cache_misses() << std::endl;
    std::cout << "Disk writes: " << storage_manager.get_disk_writes() << std::endl;
    std::cout << "Disk reads: " << storage_manager.get_disk_reads() << std::endl;
    
    std::cout << "✓ Statistics tracking working correctly" << std::endl;
    
    // Test 6: Error handling
    std::cout << "\nTest 6: Error handling" << std::endl;
    
    float non_existent_value;
    bool error_result = storage_manager.load_float("non_existent_key", &non_existent_value, 99.9f);
    assert(error_result == false);
    assert(non_existent_value == 99.9f);  // Should return default value
    
    std::cout << "✓ Error handling working correctly" << std::endl;
    
    std::cout << "\n=== All Storage Manager Tests Passed! ===" << std::endl;
}

// Test ECU use case - transmission settings
void test_transmission_settings() {
    std::cout << "\n=== Testing Transmission Settings Use Case ===" << std::endl;
    
    // Create storage backend and manager
    SPIFlashStorageBackend backend;
    StorageManager storage_manager(&backend);
    
    // Initialize systems
    g_message_bus.init();
    assert(storage_manager.init() == true);
    
    // Simulate transmission module saving settings
    std::cout << "Saving transmission settings..." << std::endl;
    
    // Save various transmission parameters
    struct TransmissionSettings {
        float shift_rpm_1_2;
        float shift_rpm_2_3;
        float shift_rpm_3_4;
        float line_pressure;
        float lockup_speed;
    } settings = {
        .shift_rpm_1_2 = 2500.0f,
        .shift_rpm_2_3 = 3000.0f,
        .shift_rpm_3_4 = 3500.0f,
        .line_pressure = 80.0f,
        .lockup_speed = 45.0f
    };
    
    // Save via message bus (as transmission module would)
    storage_save_float_msg_t save_data;
    save_data.priority = 1;
    save_data.sender_id = 2;
    
    save_data.key_hash = crc16("trans.shift_rpm_1_2");
    save_data.value = settings.shift_rpm_1_2;
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    save_data.key_hash = crc16("trans.shift_rpm_2_3");
    save_data.value = settings.shift_rpm_2_3;
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    save_data.key_hash = crc16("trans.shift_rpm_3_4");
    save_data.value = settings.shift_rpm_3_4;
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    save_data.key_hash = crc16("trans.line_pressure");
    save_data.value = settings.line_pressure;
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    save_data.key_hash = crc16("trans.lockup_speed");
    save_data.value = settings.lockup_speed;
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_data, sizeof(save_data));
    
    // Process all messages
    g_message_bus.process();
    storage_manager.update();
    
    std::cout << "✓ Transmission settings saved" << std::endl;
    
    // Simulate ECU reboot - load settings back
    std::cout << "Loading transmission settings after reboot..." << std::endl;
    
    TransmissionSettings loaded_settings;
    
    assert(storage_manager.load_float("trans.shift_rpm_1_2", &loaded_settings.shift_rpm_1_2) == true);
    assert(storage_manager.load_float("trans.shift_rpm_2_3", &loaded_settings.shift_rpm_2_3) == true);
    assert(storage_manager.load_float("trans.shift_rpm_3_4", &loaded_settings.shift_rpm_3_4) == true);
    assert(storage_manager.load_float("trans.line_pressure", &loaded_settings.line_pressure) == true);
    assert(storage_manager.load_float("trans.lockup_speed", &loaded_settings.lockup_speed) == true);
    
    // Verify all values match
    assert(loaded_settings.shift_rpm_1_2 == settings.shift_rpm_1_2);
    assert(loaded_settings.shift_rpm_2_3 == settings.shift_rpm_2_3);
    assert(loaded_settings.shift_rpm_3_4 == settings.shift_rpm_3_4);
    assert(loaded_settings.line_pressure == settings.line_pressure);
    assert(loaded_settings.lockup_speed == settings.lockup_speed);
    
    std::cout << "✓ All transmission settings loaded correctly" << std::endl;
    std::cout << "  1->2 shift: " << loaded_settings.shift_rpm_1_2 << " RPM" << std::endl;
    std::cout << "  2->3 shift: " << loaded_settings.shift_rpm_2_3 << " RPM" << std::endl;
    std::cout << "  3->4 shift: " << loaded_settings.shift_rpm_3_4 << " RPM" << std::endl;
    std::cout << "  Line pressure: " << loaded_settings.line_pressure << " PSI" << std::endl;
    std::cout << "  Lockup speed: " << loaded_settings.lockup_speed << " MPH" << std::endl;
    
    std::cout << "\n=== Transmission Settings Test Passed! ===" << std::endl;
}

int main() {
    std::cout << "Starting Storage Manager Tests..." << std::endl;
    
    // Debug: Check message sizes
    std::cout << "Message sizes:" << std::endl;
    std::cout << "  storage_save_float_msg_t: " << sizeof(storage_save_float_msg_t) << " bytes" << std::endl;
    std::cout << "  storage_load_float_msg_t: " << sizeof(storage_load_float_msg_t) << " bytes" << std::endl;
    std::cout << "  storage_load_response_msg_t: " << sizeof(storage_load_response_msg_t) << " bytes" << std::endl;
    std::cout << "  storage_save_response_msg_t: " << sizeof(storage_save_response_msg_t) << " bytes" << std::endl;
    
    test_storage_manager();
    test_transmission_settings();
    
    std::cout << "\n🎉 All tests passed! Storage system is working correctly." << std::endl;
    return 0;
} 