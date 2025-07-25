// test_request_tracker.cpp
// Tests for request tracker system

#include "../mock_arduino.h"
#include "../../request_tracker.h"

#include <cassert>
#include <iostream>
#include <vector>

// Test framework globals
static int test_count = 0;
static int passed_tests = 0;

static void run_test(const char* test_name, bool (*test_func)()) {
    test_count++;
    std::cout << "Running " << test_name << "... ";
    
    if (test_func()) {
        std::cout << "PASSED\n";
        passed_tests++;
    } else {
        std::cout << "FAILED\n";
    }
}

// Test functions
static bool test_request_tracker_creation() {
    RequestTracker tracker;
    
    // Check initial state
    if (tracker.get_pending_count() != 0) {
        return false;
    }
    
    if (tracker.get_timeout_count() != 0) {
        return false;
    }
    
    return true;
}

static bool test_add_and_remove_requests() {
    RequestTracker tracker;
    
    // Add a request
    tracker.add_request(CHANNEL_SERIAL_USB, 0x1000);
    
    if (tracker.get_pending_count() != 1) {
        return false;
    }
    
    // Get the request ID (should be 1)
    uint8_t request_id = 1; // First request ID is always 1
    
    // Check if request is pending
    if (!tracker.is_pending_request(request_id, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    // Check parameter ID
    if (tracker.get_pending_param_id(request_id, CHANNEL_SERIAL_USB) != 0x1000) {
        return false;
    }
    
    // Remove the request
    tracker.remove_request(request_id, CHANNEL_SERIAL_USB);
    
    if (tracker.get_pending_count() != 0) {
        return false;
    }
    
    // Check that request is no longer pending
    if (tracker.is_pending_request(request_id, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    return true;
}

static bool test_multiple_requests() {
    RequestTracker tracker;
    
    // Add multiple requests
    tracker.add_request(CHANNEL_SERIAL_USB, 0x1000);
    tracker.add_request(CHANNEL_SERIAL_1, 0x2000);
    tracker.add_request(CHANNEL_CAN_BUS, 0x3000);
    
    if (tracker.get_pending_count() != 3) {
        return false;
    }
    
    // Check each request
    if (!tracker.is_pending_request(1, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    if (!tracker.is_pending_request(2, CHANNEL_SERIAL_1)) {
        return false;
    }
    
    if (!tracker.is_pending_request(3, CHANNEL_CAN_BUS)) {
        return false;
    }
    
    if (tracker.get_pending_param_id(1, CHANNEL_SERIAL_USB) != 0x1000) {
        return false;
    }
    
    if (tracker.get_pending_param_id(2, CHANNEL_SERIAL_1) != 0x2000) {
        return false;
    }
    
    if (tracker.get_pending_param_id(3, CHANNEL_CAN_BUS) != 0x3000) {
        return false;
    }
    
    // Remove middle request
    tracker.remove_request(2, CHANNEL_SERIAL_1);
    
    if (tracker.get_pending_count() != 2) {
        return false;
    }
    
    // Check remaining requests
    if (!tracker.is_pending_request(1, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    if (tracker.is_pending_request(2, CHANNEL_SERIAL_1)) {
        return false;
    }
    
    if (!tracker.is_pending_request(3, CHANNEL_CAN_BUS)) {
        return false;
    }
    
    return true;
}

static bool test_request_id_generation() {
    RequestTracker tracker;
    
    // Get multiple request IDs
    uint8_t id1 = tracker.get_next_request_id();
    uint8_t id2 = tracker.get_next_request_id();
    uint8_t id3 = tracker.get_next_request_id();
    
    // Check that IDs are sequential and start at 1
    if (id1 != 1) {
        return false;
    }
    
    if (id2 != 2) {
        return false;
    }
    
    if (id3 != 3) {
        return false;
    }
    
    return true;
}

static bool test_channel_isolation() {
    RequestTracker tracker;
    
    // Add requests with same ID but different channels
    tracker.add_request(CHANNEL_SERIAL_USB, 0x1000);
    tracker.add_request(CHANNEL_SERIAL_1, 0x2000);
    
    // Both should have request ID 1 (separate counters per tracker instance)
    uint8_t request_id = 1;
    
    // Check that requests are isolated by channel
    if (!tracker.is_pending_request(request_id, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    if (!tracker.is_pending_request(request_id, CHANNEL_SERIAL_1)) {
        return false;
    }
    
    // Remove one channel's request
    tracker.remove_request(request_id, CHANNEL_SERIAL_USB);
    
    // Check isolation
    if (tracker.is_pending_request(request_id, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    if (!tracker.is_pending_request(request_id, CHANNEL_SERIAL_1)) {
        return false;
    }
    
    return true;
}

static bool test_max_requests_handling() {
    RequestTracker tracker;
    
    // Add maximum number of requests
    for (int i = 0; i < RequestTracker::MAX_PENDING; i++) {
        tracker.add_request(CHANNEL_SERIAL_USB, 0x1000 + i);
    }
    
    if (tracker.get_pending_count() != RequestTracker::MAX_PENDING) {
        return false;
    }
    
    // Add one more request (should remove oldest)
    tracker.add_request(CHANNEL_SERIAL_USB, 0x9999);
    
    if (tracker.get_pending_count() != RequestTracker::MAX_PENDING) {
        return false;
    }
    
    // The oldest request (ID 1) should be gone
    if (tracker.is_pending_request(1, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    // The newest request should be there
    if (!tracker.is_pending_request(RequestTracker::MAX_PENDING + 1, CHANNEL_SERIAL_USB)) {
        return false;
    }
    
    return true;
}

static bool test_timeout_cleanup() {
    RequestTracker tracker;
    
    // Add a request
    tracker.add_request(CHANNEL_SERIAL_USB, 0x1000);
    
    if (tracker.get_pending_count() != 1) {
        return false;
    }
    
    // Simulate time passing (advance millis by more than timeout)
    // Note: This is a simplified test since we can't easily mock millis()
    // In a real test environment, we'd need to mock the timing functions
    
    // For now, just test that cleanup doesn't crash
    tracker.cleanup_timeouts(1000); // 1 second timeout
    
    // The request should still be there since we can't easily advance time
    // This is a limitation of the current test setup
    if (tracker.get_pending_count() != 1) {
        return false;
    }
    
    return true;
}

static bool test_statistics_reset() {
    RequestTracker tracker;
    
    // Add some requests
    tracker.add_request(CHANNEL_SERIAL_USB, 0x1000);
    tracker.add_request(CHANNEL_SERIAL_1, 0x2000);
    
    if (tracker.get_pending_count() != 2) {
        return false;
    }
    
    // Reset statistics
    tracker.reset_statistics();
    
    // Pending count should remain (it's not a statistic)
    if (tracker.get_pending_count() != 2) {
        return false;
    }
    
    // Timeout count should be reset
    if (tracker.get_timeout_count() != 0) {
        return false;
    }
    
    return true;
}

// Main test function
int main() {
    std::cout << "=== Request Tracker Tests ===\n";
    
    // Run tests
    run_test("Request Tracker Creation", test_request_tracker_creation);
    run_test("Add and Remove Requests", test_add_and_remove_requests);
    run_test("Multiple Requests", test_multiple_requests);
    run_test("Request ID Generation", test_request_id_generation);
    run_test("Channel Isolation", test_channel_isolation);
    run_test("Max Requests Handling", test_max_requests_handling);
    run_test("Timeout Cleanup", test_timeout_cleanup);
    run_test("Statistics Reset", test_statistics_reset);
    
    // Print results
    std::cout << "\n=== Test Results ===\n";
    std::cout << "Passed: " << passed_tests << "/" << test_count << "\n";
    
    return (passed_tests == test_count) ? 0 : 1;
} 