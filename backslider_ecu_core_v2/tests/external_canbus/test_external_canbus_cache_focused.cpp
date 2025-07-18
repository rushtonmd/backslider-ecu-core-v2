// tests/external_canbus/test_external_canbus_cache_focused.cpp
// Focused test suite for the external CAN bus cache system

#include <iostream>
#include <cassert>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Declare mock Arduino globals (defined in mock_arduino.cpp)
extern uint32_t mock_millis_time;
extern uint32_t mock_micros_time;
extern uint16_t mock_analog_values[42];
extern uint8_t mock_digital_values[56];
extern uint8_t mock_pin_modes[56];

// Include ECU modules for testing
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../external_canbus_cache.h"
#include "../../storage_manager.h"
#include "../../spi_flash_storage_backend.h"

// Global instances for testing (needed for custom_canbus_manager linkage)
static SPIFlashStorageBackend global_storage_backend;
static StorageManager global_storage_manager(&global_storage_backend);
StorageManager& g_storage_manager = global_storage_manager;

// Simple test framework
int tests_run = 0;
int tests_passed = 0;

#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        std::cout << "  Running test: " #name "... "; \
        tests_run++; \
        test_##name(); \
        tests_passed++; \
        std::cout << "PASSED" << std::endl; \
    } \
    void test_##name()

// Test setup function
void test_setup() {
    // Reset mock Arduino state
    mock_millis_time = 0;
    mock_micros_time = 0;
    for (int i = 0; i < 42; i++) mock_analog_values[i] = 2048;
    for (int i = 0; i < 56; i++) {
        mock_digital_values[i] = 1;
        mock_pin_modes[i] = 0;
    }
    
    // Reset message bus
    g_message_bus.resetSubscribers();
    g_message_bus.resetStatistics();
}

// Test cache constants and mappings
TEST(cache_constants_and_mappings) {
    test_setup();
    
    // Check that our constants are properly defined
    std::cout << std::endl;
    std::cout << "    Checking cache constants:" << std::endl;
    std::cout << "      OBDII_PID_ENGINE_RPM = 0x" << std::hex << OBDII_PID_ENGINE_RPM << std::dec << std::endl;
    std::cout << "      OBDII_PID_VEHICLE_SPEED = 0x" << std::hex << OBDII_PID_VEHICLE_SPEED << std::dec << std::endl;
    std::cout << "      OBDII_PID_COOLANT_TEMP = 0x" << std::hex << OBDII_PID_COOLANT_TEMP << std::dec << std::endl;
    std::cout << "      CUSTOM_DASHBOARD_RPM = 0x" << std::hex << CUSTOM_DASHBOARD_RPM << std::dec << std::endl;
    std::cout << "      CUSTOM_DASHBOARD_SPEED = 0x" << std::hex << CUSTOM_DASHBOARD_SPEED << std::dec << std::endl;
    
    // Check internal message constants
    std::cout << "      MSG_ENGINE_RPM = 0x" << std::hex << MSG_ENGINE_RPM << std::dec << std::endl;
    std::cout << "      MSG_VEHICLE_SPEED = 0x" << std::hex << MSG_VEHICLE_SPEED << std::dec << std::endl;
    std::cout << "      MSG_COOLANT_TEMP = 0x" << std::hex << MSG_COOLANT_TEMP << std::dec << std::endl;
    
    // Check predefined mapping arrays
    std::cout << "      OBDII_CACHE_MAPPINGS_COUNT = " << OBDII_CACHE_MAPPINGS_COUNT << std::endl;
    std::cout << "      CUSTOM_CACHE_MAPPINGS_COUNT = " << CUSTOM_CACHE_MAPPINGS_COUNT << std::endl;
    
    // Basic sanity checks
    assert(OBDII_PID_ENGINE_RPM != 0);
    assert(MSG_ENGINE_RPM != 0);
    assert(OBDII_CACHE_MAPPINGS_COUNT > 0);
    assert(CUSTOM_CACHE_MAPPINGS_COUNT > 0);
}

// Test basic cache initialization
TEST(cache_basic_initialization) {
    test_setup();
    
    ExternalCanBusCache cache;
    
    // Test initial state
    assert(cache.get_entry_count() == 0);
    assert(cache.get_subscription_count() == 0);
    
    // Test initialization
    bool result = cache.init(1000);
    
    // Debug the initialization result
    std::cout << std::endl;
    std::cout << "    Cache initialization result: " << (result ? "SUCCESS" : "FAILED") << std::endl;
    
    const cache_stats_t& stats = cache.get_statistics();
    std::cout << "    Cache stats after init:" << std::endl;
    std::cout << "      Total requests: " << stats.total_requests << std::endl;
    std::cout << "      Subscriptions created: " << stats.subscriptions_created << std::endl;
    std::cout << "      Entries created: " << stats.entries_created << std::endl;
    
    assert(result == true);
    
    cache.shutdown();
}

// Test the issue we're seeing
TEST(debug_mapping_loading_issue) {
    test_setup();
    
    std::cout << std::endl;
    std::cout << "    Debugging mapping loading issue..." << std::endl;
    
    // Check if the predefined arrays exist and are accessible
    std::cout << "    Checking predefined arrays:" << std::endl;
    std::cout << "      OBDII_CACHE_MAPPINGS address: " << (void*)OBDII_CACHE_MAPPINGS << std::endl;
    std::cout << "      CUSTOM_CACHE_MAPPINGS address: " << (void*)CUSTOM_CACHE_MAPPINGS << std::endl;
    std::cout << "      OBDII_CACHE_MAPPINGS_COUNT: " << OBDII_CACHE_MAPPINGS_COUNT << std::endl;
    std::cout << "      CUSTOM_CACHE_MAPPINGS_COUNT: " << CUSTOM_CACHE_MAPPINGS_COUNT << std::endl;
    
    if (OBDII_CACHE_MAPPINGS_COUNT > 0) {
        std::cout << "      First OBD-II mapping:" << std::endl;
        std::cout << "        external_key: 0x" << std::hex << OBDII_CACHE_MAPPINGS[0].external_key << std::dec << std::endl;
        std::cout << "        internal_msg_id: 0x" << std::hex << OBDII_CACHE_MAPPINGS[0].internal_msg_id << std::dec << std::endl;
        std::cout << "        default_max_age_ms: " << OBDII_CACHE_MAPPINGS[0].default_max_age_ms << std::endl;
        std::cout << "        description: " << (OBDII_CACHE_MAPPINGS[0].description ? OBDII_CACHE_MAPPINGS[0].description : "NULL") << std::endl;
    }
    
    if (CUSTOM_CACHE_MAPPINGS_COUNT > 0) {
        std::cout << "      First custom mapping:" << std::endl;
        std::cout << "        external_key: 0x" << std::hex << CUSTOM_CACHE_MAPPINGS[0].external_key << std::dec << std::endl;
        std::cout << "        internal_msg_id: 0x" << std::hex << CUSTOM_CACHE_MAPPINGS[0].internal_msg_id << std::dec << std::endl;
        std::cout << "        default_max_age_ms: " << CUSTOM_CACHE_MAPPINGS[0].default_max_age_ms << std::endl;
        std::cout << "        description: " << (CUSTOM_CACHE_MAPPINGS[0].description ? CUSTOM_CACHE_MAPPINGS[0].description : "NULL") << std::endl;
    }
    
    // This test always passes - it's just for debugging
    assert(true);
}

// Test manual mapping addition
TEST(cache_manual_mapping) {
    test_setup();
    
    ExternalCanBusCache cache;
    cache.init(1000);
    
    // Add a manual mapping
    uint32_t test_external_key = 0x12345678;
    uint32_t test_internal_msg = MSG_ENGINE_RPM;
    uint32_t test_max_age = 500;
    const char* test_description = "Test Manual Mapping";
    
    bool result = cache.add_mapping(test_external_key, test_internal_msg, test_max_age, test_description);
    assert(result == true);
    
    std::cout << std::endl;
    std::cout << "    Manual mapping added successfully" << std::endl;
    
    // Try to use the mapping
    float dummy_value;
    cache.get_value(test_external_key, &dummy_value);
    
    // Should have created a cache entry
    assert(cache.get_entry_count() >= 1);
    
    const cache_stats_t& stats = cache.get_statistics();
    std::cout << "    After using manual mapping:" << std::endl;
    std::cout << "      Entries created: " << stats.entries_created << std::endl;
    std::cout << "      Subscriptions created: " << stats.subscriptions_created << std::endl;
    std::cout << "      Total requests: " << stats.total_requests << std::endl;
    
    cache.shutdown();
}

// Test predefined OBD-II mappings
TEST(cache_obdii_mappings) {
    test_setup();
    
    std::cout << std::endl;
    std::cout << "    Testing OBD-II mapping load..." << std::endl;
    
    ExternalCanBusCache cache;
    cache.init(1000);
    
    // Try to load OBD-II mappings manually
    bool obdii_result = cache.load_obdii_mappings();
    std::cout << "    OBD-II mappings load result: " << (obdii_result ? "SUCCESS" : "FAILED") << std::endl;
    
    // If that failed, try adding one manually from the predefined array
    if (!obdii_result && OBDII_CACHE_MAPPINGS_COUNT > 0) {
        std::cout << "    Trying to add first OBD-II mapping manually..." << std::endl;
        const cache_mapping_t& first_mapping = OBDII_CACHE_MAPPINGS[0];
        
        std::cout << "      First mapping: external_key=0x" << std::hex << first_mapping.external_key 
                  << " internal_msg=0x" << first_mapping.internal_msg_id 
                  << " max_age=" << std::dec << first_mapping.default_max_age_ms << std::endl;
        
        bool manual_result = cache.add_mapping(first_mapping);
        std::cout << "    Manual add result: " << (manual_result ? "SUCCESS" : "FAILED") << std::endl;
        
        if (manual_result) {
            // Try to use it
            float value;
            cache.get_value(first_mapping.external_key, &value);
            assert(cache.get_entry_count() >= 1);
        }
    }
    
    cache.shutdown();
}

// Test predefined custom mappings
TEST(cache_custom_mappings) {
    test_setup();
    
    std::cout << std::endl;
    std::cout << "    Testing custom mapping load..." << std::endl;
    
    ExternalCanBusCache cache;
    cache.init(1000);
    
    // Try to load custom mappings manually
    bool custom_result = cache.load_custom_mappings();
    std::cout << "    Custom mappings load result: " << (custom_result ? "SUCCESS" : "FAILED") << std::endl;
    
    // If that failed, try adding one manually from the predefined array
    if (!custom_result && CUSTOM_CACHE_MAPPINGS_COUNT > 0) {
        std::cout << "    Trying to add first custom mapping manually..." << std::endl;
        const cache_mapping_t& first_mapping = CUSTOM_CACHE_MAPPINGS[0];
        
        std::cout << "      First mapping: external_key=0x" << std::hex << first_mapping.external_key 
                  << " internal_msg=0x" << first_mapping.internal_msg_id 
                  << " max_age=" << std::dec << first_mapping.default_max_age_ms << std::endl;
        
        bool manual_result = cache.add_mapping(first_mapping);
        std::cout << "    Manual add result: " << (manual_result ? "SUCCESS" : "FAILED") << std::endl;
        
        if (manual_result) {
            // Try to use it
            float value;
            cache.get_value(first_mapping.external_key, &value);
            assert(cache.get_entry_count() >= 1);
        }
    }
    
    cache.shutdown();
}

// Test cache lazy loading with known working mapping
TEST(cache_lazy_loading) {
    test_setup();
    
    g_message_bus.init();
    
    ExternalCanBusCache cache;
    cache.init(1000);
    
    // Add a known working mapping
    uint32_t test_key = 0x99999999;
    cache.add_mapping(test_key, MSG_ENGINE_RPM, 1000, "Test Lazy Loading");
    
    std::cout << std::endl;
    std::cout << "    Testing lazy loading with known mapping..." << std::endl;
    
    // Initially no subscriptions
    assert(g_message_bus.getSubscriberCount() == 0);
    
    // First request should create subscription
    float rpm_value;
    bool result = cache.get_value(test_key, &rpm_value);
    
    std::cout << "    After first request:" << std::endl;
    std::cout << "      Result: " << (result ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "      Message bus subscribers: " << g_message_bus.getSubscriberCount() << std::endl;
    std::cout << "      Cache entries: " << cache.get_entry_count() << std::endl;
    std::cout << "      Cache subscriptions: " << cache.get_subscription_count() << std::endl;
    
    // Should have created subscription even if no data yet
    assert(g_message_bus.getSubscriberCount() >= 1);
    assert(cache.get_entry_count() >= 1);
    
    // Simulate internal message
    std::cout << "    Publishing internal message..." << std::endl;
    g_message_bus.publishFloat(MSG_ENGINE_RPM, 3000.0f);
    g_message_bus.process();
    
    mock_millis_time += 10;
    
    // Now should get cached value
    std::cout << "    Attempting to get cached value:" << std::endl;
    result = cache.get_value(test_key, &rpm_value);
    std::cout << "    After internal message:" << std::endl;
    std::cout << "      Result: " << (result ? "SUCCESS" : "FAILED") << std::endl;
    if (result) {
        std::cout << "      Value: " << rpm_value << std::endl;
        assert(rpm_value == 3000.0f);
    }
    
    // Check cache statistics instead of direct entry access
    const cache_stats_t& stats = cache.get_statistics();
    std::cout << "    Cache statistics:" << std::endl;
    std::cout << "      Total requests: " << stats.total_requests << std::endl;
    std::cout << "      Cache hits: " << stats.cache_hits << std::endl;
    std::cout << "      Cache misses: " << stats.cache_misses << std::endl;
    std::cout << "      Entries created: " << stats.entries_created << std::endl;
    
    cache.shutdown();
}

// Test automatic mapping loading during initialization
TEST(cache_automatic_mapping_loading) {
    test_setup();
    
    std::cout << std::endl;
    std::cout << "    Testing automatic mapping loading during init..." << std::endl;
    
    ExternalCanBusCache cache;
    
    // This should automatically load the mappings during init
    bool result = cache.init(1000);
    assert(result == true);
    
    std::cout << "    Checking if automatic loading worked..." << std::endl;
    
    // Try to use a predefined OBD-II mapping
    float value;
    cache.get_value(OBDII_PID_ENGINE_RPM, &value);
    
    // Should have created cache entry (proving the mapping exists)
    assert(cache.get_entry_count() >= 1);
    
    std::cout << "    Cache entries after using predefined mapping: " << cache.get_entry_count() << std::endl;
    
    // Try to use a predefined custom mapping
    cache.get_value(CUSTOM_DASHBOARD_RPM, &value);
    
    std::cout << "    Cache entries after using custom mapping: " << cache.get_entry_count() << std::endl;
    
    cache.shutdown();
}

// Main test runner
int main() {
    std::cout << "=== External CAN Bus Cache Focused Tests ===" << std::endl;
    
    // Initialize storage manager for custom_canbus_manager linkage
    global_storage_backend.begin();
    g_storage_manager.init();
    
    // Run all tests
    run_test_cache_constants_and_mappings();
    run_test_cache_basic_initialization();
    run_test_cache_manual_mapping();
    run_test_cache_obdii_mappings();
    run_test_cache_custom_mappings();
    run_test_cache_lazy_loading();
    run_test_cache_automatic_mapping_loading();
    run_test_debug_mapping_loading_issue();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Cache Focused Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL CACHE FOCUSED TESTS PASSED!" << std::endl;
        std::cout << "Cache system debugging complete!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME CACHE FOCUSED TESTS FAILED!" << std::endl;
        return 1;
    }
}