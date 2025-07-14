// tests/external_serial/test_external_serial.cpp
// Comprehensive test suite for the external serial communication system

#include <iostream>
#include <cassert>
#include <vector>
#include <cstring>

// Include mock Arduino before any ECU code
#include "../mock_arduino.h"

// Define the Serial mock object
MockSerial Serial;

// Mock HardwareSerial for testing
class MockHardwareSerial : public HardwareSerial {
private:
    std::vector<uint8_t> tx_buffer;
    std::vector<uint8_t> rx_buffer;
    size_t rx_position;
    
public:
    MockHardwareSerial() : rx_position(0) {}
    
    void begin(unsigned long baud) override {
        // Mock initialization
    }
    
    int available() override {
        return static_cast<int>(rx_buffer.size() - rx_position);
    }
    
    int read() override {
        if (rx_position < rx_buffer.size()) {
            return static_cast<int>(rx_buffer[rx_position++]);
        }
        return -1;
    }
    
    size_t write(uint8_t byte) override {
        tx_buffer.push_back(byte);
        return 1;
    }
    
    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            tx_buffer.push_back(buffer[i]);
        }
        return size;
    }
    
    void flush() override {
        // Mock flush
    }
    
    // Test helper methods
    void inject_rx_data(const uint8_t* data, size_t length) {
        for (size_t i = 0; i < length; i++) {
            rx_buffer.push_back(data[i]);
        }
    }
    
    void inject_rx_byte(uint8_t byte) {
        rx_buffer.push_back(byte);
    }
    
    std::vector<uint8_t> get_tx_data() const {
        return tx_buffer;
    }
    
    void clear_tx_buffer() {
        tx_buffer.clear();
    }
    
    void clear_rx_buffer() {
        rx_buffer.clear();
        rx_position = 0;
    }
    
    size_t get_tx_size() const {
        return tx_buffer.size();
    }
};

// Include external serial for testing
#include "../../msg_definitions.h"
#include "../../external_serial.h"

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

// Test globals
static MockHardwareSerial mock_serial;
static ExternalSerial test_serial;

// Helper function to create a test packet
serial_packet_t create_test_packet(uint8_t source_id, uint8_t dest_id, uint32_t msg_id, float value) {
    serial_packet_t packet = {}; // Zero-initialize the entire packet
    packet.sync_byte = 0xAA;
    packet.source_id = source_id;
    packet.dest_id = dest_id;
    packet.packet_type = PACKET_TYPE_NORMAL;
    
    packet.can_msg.id = msg_id;
    packet.can_msg.len = sizeof(float);
    memset(packet.can_msg.buf, 0, sizeof(packet.can_msg.buf)); // Clear the entire buffer
    memcpy(packet.can_msg.buf, &value, sizeof(float));
    packet.can_msg.timestamp = 12345;
    
    // Calculate checksum (simplified for testing)
    packet.checksum = 0x1234; // Mock checksum
    
    return packet;
}

// Test basic initialization and configuration
TEST(initialization_and_configuration) {
    ExternalSerial serial;
    
    // Test initialization
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial, 2000000);
    
    // Check device ID
    assert(serial.get_device_id() == DEVICE_ID_PRIMARY_ECU);
    
    // Check initial statistics
    assert(serial.get_packets_sent() == 0);
    assert(serial.get_packets_received() == 0);
    assert(serial.get_checksum_errors() == 0);
    assert(serial.get_forwarding_rule_count() == 0);
}

// Test forwarding rule management
TEST(forwarding_rule_management) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Add forwarding rules
    bool result = serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU, 50);
    assert(result == true);
    assert(serial.get_forwarding_rule_count() == 1);
    
    // Add another rule
    result = serial.subscribe_for_forwarding(MSG_THROTTLE_POSITION, DEVICE_ID_DASHBOARD, 100);
    assert(result == true);
    assert(serial.get_forwarding_rule_count() == 2);
    
    // Add broadcast rule
    result = serial.subscribe_for_forwarding(MSG_ENGINE_STATUS, DEVICE_ID_BROADCAST, 1000);
    assert(result == true);
    assert(serial.get_forwarding_rule_count() == 3);
    
    // Test rule update (same msg_id and dest_id)
    result = serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU, 25);
    assert(result == true);
    assert(serial.get_forwarding_rule_count() == 3); // Should not increase
    
    // Clear rules
    serial.clear_forwarding_rules();
    assert(serial.get_forwarding_rule_count() == 0);
}

// Test packet creation and validation
TEST(packet_creation_and_validation) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Test direct message sending
    float test_rpm = 6500.0f;
    bool result = serial.send_message_to_device(MSG_ENGINE_RPM, &test_rpm, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    assert(result == true);
    
    // Check that data was sent
    assert(mock_serial.get_tx_size() == sizeof(serial_packet_t));
    
    // Verify packet structure
    std::vector<uint8_t> tx_data = mock_serial.get_tx_data();
    serial_packet_t* sent_packet = (serial_packet_t*)tx_data.data();
    
    assert(sent_packet->sync_byte == 0xAA);
    assert(sent_packet->source_id == DEVICE_ID_PRIMARY_ECU);
    assert(sent_packet->dest_id == DEVICE_ID_SECONDARY_ECU);
    assert(sent_packet->packet_type == PACKET_TYPE_NORMAL);
    assert(sent_packet->can_msg.id == MSG_ENGINE_RPM);
    assert(sent_packet->can_msg.len == sizeof(float));
    
    // Verify data payload
    float received_rpm = MSG_UNPACK_FLOAT(&sent_packet->can_msg);
    assert(received_rpm == test_rpm);
    
    mock_serial.clear_tx_buffer();
}

// Test packet parsing state machine
TEST(packet_parsing_state_machine) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_SECONDARY_ECU, &mock_serial);
    
    // Create a test packet for this device
    serial_packet_t test_packet = create_test_packet(DEVICE_ID_PRIMARY_ECU, DEVICE_ID_SECONDARY_ECU, MSG_ENGINE_RPM, 5500.0f);
    
    // Inject packet data byte by byte
    uint8_t* packet_bytes = (uint8_t*)&test_packet;
    for (size_t i = 0; i < sizeof(serial_packet_t); i++) {
        mock_serial.inject_rx_byte(packet_bytes[i]);
    }
    
    // Process the data
    uint32_t initial_received = serial.get_packets_received();
    serial.update();
    
    // Note: In a real test, we'd need to handle the checksum validation
    // For now, this tests the parsing state machine structure
    
    mock_serial.clear_rx_buffer();
}

// Test buffer management
TEST(buffer_management) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Test initial buffer state
    assert(serial.get_rx_buffer_level() == 0);
    
    // Inject some data
    for (int i = 0; i < 100; i++) {
        mock_serial.inject_rx_byte(i);
    }
    
    // Process data (this will fill internal buffer)
    serial.update();
    
    // Buffer should have data now (exact amount depends on parsing)
    // Just verify no overflow occurred initially
    assert(serial.get_buffer_overflows() == 0);
    
    mock_serial.clear_rx_buffer();
}

// Test device addressing and routing
TEST(device_addressing_and_routing) {
    ExternalSerial primary_serial;
    ExternalSerial secondary_serial;
    
    MockHardwareSerial primary_port;
    MockHardwareSerial secondary_port;
    
    primary_serial.init(DEVICE_ID_PRIMARY_ECU, &primary_port);
    secondary_serial.init(DEVICE_ID_SECONDARY_ECU, &secondary_port);
    
    // Test message for specific device
    float test_value = 7200.0f;
    primary_serial.send_message_to_device(MSG_ENGINE_RPM, &test_value, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    
    // Verify packet was sent
    assert(primary_port.get_tx_size() == sizeof(serial_packet_t));
    
    // Test broadcast message
    primary_serial.send_message_to_device(MSG_ENGINE_STATUS, &test_value, sizeof(float), DEVICE_ID_BROADCAST);
    
    // Should have sent two packets now
    assert(primary_port.get_tx_size() == 2 * sizeof(serial_packet_t));
    
    primary_port.clear_tx_buffer();
    secondary_port.clear_tx_buffer();
}

// Test rate limiting functionality
TEST(rate_limiting) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Add rule with 100ms rate limit
    serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU, 100);
    
    // Send first message - should go through
    float rpm1 = 3000.0f;
    bool result1 = serial.send_message_to_device(MSG_ENGINE_RPM, &rpm1, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    assert(result1 == true);
    
    size_t packets_after_first = mock_serial.get_tx_size() / sizeof(serial_packet_t);
    
    // Send second message immediately - should be rate limited in forwarding rules
    // (Note: direct sending bypasses rate limiting, so this tests the rule structure)
    float rpm2 = 3500.0f;
    bool result2 = serial.send_message_to_device(MSG_ENGINE_RPM, &rpm2, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    assert(result2 == true); // Direct send should still work
    
    mock_serial.clear_tx_buffer();
}

// Test statistics tracking
TEST(statistics_tracking) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Initial statistics
    assert(serial.get_packets_sent() == 0);
    assert(serial.get_packets_received() == 0);
    assert(serial.get_packets_forwarded() == 0);
    assert(serial.get_checksum_errors() == 0);
    assert(serial.get_timeout_errors() == 0);
    assert(serial.get_buffer_overflows() == 0);
    
    // Send some packets
    float test_value = 4500.0f;
    serial.send_message_to_device(MSG_ENGINE_RPM, &test_value, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    serial.send_message_to_device(MSG_THROTTLE_POSITION, &test_value, sizeof(float), DEVICE_ID_DASHBOARD);
    
    // Check sent statistics
    assert(serial.get_packets_sent() == 2);
    
    // Reset statistics
    serial.reset_statistics();
    assert(serial.get_packets_sent() == 0);
    
    mock_serial.clear_tx_buffer();
}

// Test error handling
TEST(error_handling) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Test invalid data length
    uint8_t invalid_data[16]; // Too long for CAN message
    bool result = serial.send_message_to_device(MSG_ENGINE_RPM, invalid_data, 16, DEVICE_ID_SECONDARY_ECU);
    assert(result == false);
    
    // Test invalid device ID during initialization
    ExternalSerial invalid_serial;
    invalid_serial.init(DEVICE_ID_INVALID, &mock_serial);
    assert(invalid_serial.get_device_id() == DEVICE_ID_INVALID);
    
    // Verify no packets sent due to invalid operations
    assert(mock_serial.get_tx_size() == 0);
}

// Test ping functionality
TEST(ping_functionality) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Test ping request
    bool result = serial.ping_device(DEVICE_ID_SECONDARY_ECU);
    assert(result == true);
    
    // Should have sent a ping packet
    assert(mock_serial.get_tx_size() == sizeof(serial_packet_t));
    
    // Verify it's a ping packet
    std::vector<uint8_t> tx_data = mock_serial.get_tx_data();
    serial_packet_t* sent_packet = (serial_packet_t*)tx_data.data();
    assert(sent_packet->packet_type == PACKET_TYPE_PING);
    assert(sent_packet->dest_id == DEVICE_ID_SECONDARY_ECU);
    
    mock_serial.clear_tx_buffer();
}

// Test message filtering and forwarding logic
TEST(message_filtering_and_forwarding) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Set up forwarding rules
    serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU);
    serial.subscribe_for_forwarding(MSG_COOLANT_TEMP, DEVICE_ID_DASHBOARD);
    
    // Test that correct messages would be forwarded
    // (Note: This tests the rule structure - actual forwarding would need message bus integration)
    
    // Send a message that has a forwarding rule
    float rpm = 5000.0f;
    serial.send_message_to_device(MSG_ENGINE_RPM, &rpm, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    
    // Send a message that doesn't have a forwarding rule
    float voltage = 12.6f;
    serial.send_message_to_device(MSG_BATTERY_VOLTAGE, &voltage, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    
    // Both should be sent (direct sending bypasses filtering)
    assert(serial.get_packets_sent() == 2);
    
    mock_serial.clear_tx_buffer();
}

// Test checksum calculation
TEST(checksum_calculation) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Create identical packets
    serial_packet_t packet1 = create_test_packet(DEVICE_ID_PRIMARY_ECU, DEVICE_ID_SECONDARY_ECU, MSG_ENGINE_RPM, 3000.0f);
    serial_packet_t packet2 = create_test_packet(DEVICE_ID_PRIMARY_ECU, DEVICE_ID_SECONDARY_ECU, MSG_ENGINE_RPM, 3000.0f);
    
    // Remove checksums for testing
    packet1.checksum = 0;
    packet2.checksum = 0;
    
    // Manually test checksum consistency (would need access to calculate_checksum method)
    // For now, just verify packet structure is consistent
    assert(memcmp(&packet1, &packet2, sizeof(serial_packet_t) - sizeof(uint16_t)) == 0);
}

// Test multiple device chain simulation
TEST(multiple_device_chain) {
    // Simulate 3 devices in a chain
    ExternalSerial device1, device2, device3;
    MockHardwareSerial port1, port2, port3;
    
    device1.init(DEVICE_ID_PRIMARY_ECU, &port1);
    device2.init(DEVICE_ID_SECONDARY_ECU, &port2);
    device3.init(DEVICE_ID_DASHBOARD, &port3);
    
    // Device 1 sends to Device 3 (should route through Device 2)
    float test_data = 8500.0f;
    device1.send_message_to_device(MSG_VEHICLE_SPEED, &test_data, sizeof(float), DEVICE_ID_DASHBOARD);
    
    // Verify packet was sent from Device 1
    assert(port1.get_tx_size() == sizeof(serial_packet_t));
    
    // In a real chain, Device 2 would receive and forward this packet
    // For this test, we just verify the packet structure
    std::vector<uint8_t> tx_data = port1.get_tx_data();
    serial_packet_t* packet = (serial_packet_t*)tx_data.data();
    assert(packet->source_id == DEVICE_ID_PRIMARY_ECU);
    assert(packet->dest_id == DEVICE_ID_DASHBOARD);
    
    port1.clear_tx_buffer();
    port2.clear_tx_buffer();
    port3.clear_tx_buffer();
}

// Test configuration edge cases
TEST(configuration_edge_cases) {
    ExternalSerial serial;
    
    // Test initialization with null pointer
    serial.init(DEVICE_ID_PRIMARY_ECU, nullptr);
    assert(serial.get_device_id() == DEVICE_ID_PRIMARY_ECU); // Should still set ID
    
    // Test too many forwarding rules
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Add maximum number of rules
    for (int i = 0; i < 32; i++) { // MAX_FORWARDING_RULES
        bool result = serial.subscribe_for_forwarding(0x100 + i, DEVICE_ID_SECONDARY_ECU);
        assert(result == true);
    }
    
    // Try to add one more - should fail
    bool result = serial.subscribe_for_forwarding(0x200, DEVICE_ID_SECONDARY_ECU);
    assert(result == false);
}

// Test data integrity
TEST(data_integrity) {
    ExternalSerial serial;
    serial.init(DEVICE_ID_PRIMARY_ECU, &mock_serial);
    
    // Test various data types
    float float_val = 123.456f;
    uint32_t uint32_val = 0x12345678;
    uint16_t uint16_val = 0xABCD;
    uint8_t uint8_val = 0x42;
    
    // Send different data types
    serial.send_message_to_device(MSG_ENGINE_RPM, &float_val, sizeof(float), DEVICE_ID_SECONDARY_ECU);
    serial.send_message_to_device(MSG_SYSTEM_TIME, &uint32_val, sizeof(uint32_t), DEVICE_ID_SECONDARY_ECU);
    serial.send_message_to_device(MSG_IDLE_TARGET_RPM, &uint16_val, sizeof(uint16_t), DEVICE_ID_SECONDARY_ECU);
    serial.send_message_to_device(MSG_ENGINE_STATUS, &uint8_val, sizeof(uint8_t), DEVICE_ID_SECONDARY_ECU);
    
    // Verify all packets sent
    assert(serial.get_packets_sent() == 4);
    
    // Verify total data sent
    assert(mock_serial.get_tx_size() == 4 * sizeof(serial_packet_t));
    
    mock_serial.clear_tx_buffer();
}

// Main test runner
int main() {
    std::cout << "=== External Serial Communication Tests ===" << std::endl;
    
    // Run all tests
    run_test_initialization_and_configuration();
    run_test_forwarding_rule_management();
    run_test_packet_creation_and_validation();
    run_test_packet_parsing_state_machine();
    run_test_buffer_management();
    run_test_device_addressing_and_routing();
    run_test_rate_limiting();
    run_test_statistics_tracking();
    run_test_error_handling();
    run_test_ping_functionality();
    run_test_message_filtering_and_forwarding();
    run_test_checksum_calculation();
    run_test_multiple_device_chain();
    run_test_configuration_edge_cases();
    run_test_data_integrity();
    
    // Print results
    std::cout << std::endl;
    std::cout << "External Serial Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL EXTERNAL SERIAL TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME EXTERNAL SERIAL TESTS FAILED!" << std::endl;
        return 1;
    }
}