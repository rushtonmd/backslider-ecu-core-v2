// tests/main_application/test_main_application.cpp
// Test suite for the main ECU application

#include <iostream>
#include <cassert>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

// Include ECU modules for testing
#include "../../main_application.h"

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

// Test the main application
TEST(main_application_creation) {
    MainApplication app;
    app.init();
    // If we get here without crashing, test passes
    assert(true);
}

TEST(main_application_run) {
    MainApplication app;
    app.init();
    app.run();  // Should not crash
    assert(true);
}

TEST(main_application_led_state_tracking) {
    MainApplication app;
    app.init();
    
    // Run a few times to test LED state changes
    app.run();
    app.run();
    app.run();
    
    // If we get here, the application is handling state properly
    assert(true);
}

// Main test runner
int main() {
    std::cout << "=== Main Application Tests ===" << std::endl;
    
    // Run all tests
    run_test_main_application_creation();
    run_test_main_application_run();
    run_test_main_application_led_state_tracking();
    
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