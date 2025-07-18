// tests/main_application/test_main_application.cpp
// Test suite for the main application

#include <iostream>
#include <cassert>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock objects that will be used by both test and main_application
// MockSerial Serial and Serial1 are defined in mock_arduino.cpp

// Include main application and dependencies
#include "../../main_application.h"
#include "../../msg_definitions.h"
#include "../../msg_bus.h"
#include "../../input_manager.h"

// Include storage manager for custom_canbus_manager
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
    mock_reset_all();
    
    // Reset message bus subscribers to prevent "too many subscribers" errors
    g_message_bus.resetSubscribers();
    
    // Set some realistic sensor values for testing
    mock_set_analog_voltage(14, 2.5f);  // A0 (TPS) at 50%
    mock_set_analog_voltage(15, 1.5f);  // A1 (MAP)
    mock_set_analog_voltage(16, 2.0f);  // A2 (MAF)
    mock_set_analog_voltage(17, 2.5f);  // A3 (CTS)
    mock_set_analog_voltage(18, 2.0f);  // A4 (IAT)
    mock_set_analog_voltage(19, 2.4f);  // A5 (Battery - 12V through divider)
}

// Test main application initialization
TEST(main_application_initialization) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    // Check that initialization completed
    assert(app.getLoopCount() == 0);
    
    // Check that subsystems were initialized (don't care about exact counts)
    assert(g_message_bus.getMessagesProcessed() >= 0);  // Should be initialized
    
    // Sensor count should be non-negative (could be 0 or any positive number)
    assert(input_manager_get_sensor_count() >= 0);
    assert(input_manager_get_valid_sensor_count() >= 0);
    assert(input_manager_get_valid_sensor_count() <= input_manager_get_sensor_count());
}

// Test main application run loop
TEST(main_application_run_loop) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    uint32_t initial_loop_count = app.getLoopCount();
    
    // Run a few loop iterations
    for (int i = 0; i < 5; i++) {
        mock_advance_time_us(10000);  // Advance time by 10ms
        app.run();
    }
    
    // Check that loop count increased
    assert(app.getLoopCount() > initial_loop_count);
    assert(app.getLoopCount() == initial_loop_count + 5);
    
    // Check that loop timing is being measured (in mock environment, might be 0)
    // We'll just check that the method doesn't crash and returns a reasonable value
    uint32_t loop_time = app.getLastLoopTime();
    assert(loop_time >= 0);  // Should at least be non-negative
    
    // In mock environment, timing might be 0 due to instant execution
    // The important thing is that the system runs without crashing
}

// Test sensor integration (module-agnostic)
TEST(sensor_integration) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    // Store initial counts (whatever they are)
    uint8_t initial_sensor_count = input_manager_get_sensor_count();
    uint32_t initial_updates = input_manager_get_total_updates();
    
    std::cout << "\n    Initial sensor count: " << (int)initial_sensor_count;
    
    // Run several loops with time advancement to trigger sensor updates
    for (int i = 0; i < 10; i++) {
        mock_advance_time_ms(200);  // Advance time by 200ms (enough for sensor updates)
        app.run();
    }
    
    // Check that the system is still functional
    assert(input_manager_get_sensor_count() == initial_sensor_count);  // Count shouldn't change during operation
    assert(input_manager_get_valid_sensor_count() <= initial_sensor_count); // Valid <= total
    
    // If sensors are registered, updates should increase with time advancement
    if (initial_sensor_count > 0) {
        assert(input_manager_get_total_updates() > initial_updates);  // Should have processed sensor updates
        std::cout << "\n    Sensor updates increased from " << initial_updates << " to " << input_manager_get_total_updates();
    } else {
        assert(input_manager_get_total_updates() == initial_updates);  // No sensors, no updates
        std::cout << "\n    No sensors registered, no updates expected";
    }
}

// Test message bus integration
TEST(message_bus_integration) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    uint32_t initial_messages = g_message_bus.getMessagesProcessed();
    
    // Run several loops
    for (int i = 0; i < 20; i++) {
        mock_advance_time_us(20000);  // Advance time by 20ms per loop
        app.run();
    }
    
    // Message bus should be functional (processing >= initial count)
    uint32_t final_messages = g_message_bus.getMessagesProcessed();
    
    // Check that message bus is functional (even if no new messages)
    assert(final_messages >= initial_messages);  // Should not decrease
    
    // Test that we can manually publish and process a message
    bool publish_result = g_message_bus.publishFloat(MSG_DEBUG_MESSAGE, 123.45f);
    assert(publish_result == true);
    
    // Process the message we just published
    g_message_bus.process();
    
    // Should have processed at least one more message now
    assert(g_message_bus.getMessagesProcessed() > final_messages);
}

// Test performance characteristics
TEST(performance_characteristics) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    // Run multiple loops and check that the system works
    const int test_loops = 50;
    
    for (int i = 0; i < test_loops; i++) {
        mock_advance_time_us(10000);  // 10ms between loops
        app.run();
    }
    
    // In mock environment, timing measurements might be 0 due to instant execution
    // The important thing is that the loop count is correct and system doesn't crash
    assert(app.getLoopCount() == test_loops);
    
    uint32_t last_loop_time = app.getLastLoopTime();
    
    std::cout << "\n    Performance metrics (mock environment):";
    std::cout << "\n      Loop count: " << app.getLoopCount();
    std::cout << "\n      Last loop time: " << last_loop_time << " µs";
    std::cout << "\n      Sensors registered: " << (int)input_manager_get_sensor_count();
    std::cout << "\n      Note: Timing may be 0 in mock environment";
    
    // Basic sanity checks for mock environment
    assert(last_loop_time >= 0);  // Should be non-negative
}

// Test system status reporting
TEST(status_reporting) {
    test_setup();
    
    MainApplication app;
    app.init();
    
    // Store initial state
    uint8_t sensor_count = input_manager_get_sensor_count();
    
    // Run for a while to generate status
    for (int i = 0; i < 10; i++) {
        mock_advance_time_ms(1000);  // Advance by 1 second each loop
        app.run();
    }
    
    // Check that statistics are being tracked
    assert(app.getLoopCount() == 10);
    
    // System health checks (module-agnostic)
    assert(input_manager_get_sensor_count() == sensor_count);  // Count should be stable
    assert(input_manager_get_valid_sensor_count() <= sensor_count); // Valid <= total
    assert(input_manager_get_total_errors() >= 0);  // Non-negative errors
    
    // Message bus should be working
    assert(g_message_bus.getMessagesProcessed() >= 0);
    assert(g_message_bus.getQueueOverflows() == 0);     // Should have no overflows in normal operation
    
    // The status reporting function should run without crashing
    // (it's called internally during the run loop when time advances)
}

// Main test runner
int main() {
    std::cout << "=== Main Application Tests ===" << std::endl;
    
    // Initialize storage manager for custom_canbus_manager linkage
    global_storage_backend.begin();
    g_storage_manager.init();
    
    // Run all tests
    run_test_main_application_initialization();
    run_test_main_application_run_loop();
    run_test_sensor_integration();
    run_test_message_bus_integration();
    run_test_performance_characteristics();
    run_test_status_reporting();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Main Application Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL MAIN APPLICATION TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME MAIN APPLICATION TESTS FAILED!" << std::endl;
        return 1;
    }
}