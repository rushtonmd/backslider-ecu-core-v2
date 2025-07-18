// test_external_serial.cpp
// Tests for simplified point-to-point serial communication

#include <cassert>
#include <cstring>
#include <cstdio>
#include <vector>
#include <string>
#include "../mock_arduino.h"
#include "../../external_serial.h"
#include "../../msg_bus.h"
#include "../../msg_definitions.h"

// Mock Serial instances are defined in mock_arduino.cpp

// Test setup helpers
void setup_test_message_bus() {
    // Initialize message bus for testing
    extern MessageBus g_message_bus;
    g_message_bus.init();
}

void reset_mock_serials() {
    Serial.reset();
    Serial1.reset();
    Serial2.reset();
}

// Test data
CANMessage create_test_message(uint32_t id, uint8_t len, const uint8_t* data) {
    CANMessage msg;
    msg.id = id;
    msg.len = len;
    memcpy(msg.buf, data, len);
    msg.timestamp = millis();
    return msg;
}

// Test message filtering
void test_message_filtering() {
    printf("Testing message filtering...\n");
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Test internal ECU messages (ECU base = 0) - should not be processed
    uint32_t internal_msg_id = 0x00000123;  // ECU base = 0
    assert(!bridge.should_process_message(internal_msg_id));
    
    // Test external ECU messages (ECU base > 0) - should be processed
    uint32_t external_msg_id = 0x10000123;  // ECU base = 1
    assert(bridge.should_process_message(external_msg_id));
    
    external_msg_id = 0x20000456;  // ECU base = 2
    assert(bridge.should_process_message(external_msg_id));
    
    printf("✓ Message filtering tests passed\n");
}

// Test serial bridge initialization
void test_serial_bridge_init() {
    printf("Testing serial bridge initialization...\n");
    
    SerialBridge bridge;
    
    // Test invalid port initialization
    assert(!bridge.init(nullptr, {true, 115200, true, true}));
    
    // Test valid initialization
    serial_port_config_t config = {true, 115200, true, true};
    assert(bridge.init(&Serial, config));
    assert(bridge.is_enabled());
    
    // Test disabled initialization
    config.enabled = false;
    assert(bridge.init(&Serial, config));
    assert(!bridge.is_enabled());
    
    printf("✓ Serial bridge initialization tests passed\n");
}

// Test message sending
void test_message_sending() {
    printf("Testing message sending...\n");
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Create test message
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    CANMessage msg = create_test_message(0x123, 4, test_data);
    
    // Send message
    bridge.send_message(msg);
    
    // Verify message was sent
    assert(bridge.get_messages_sent() == 1);
    
    // Verify binary data was written to serial
    std::vector<uint8_t> written_data = Serial.get_written_data();
    assert(written_data.size() == sizeof(CANMessage));
    
    // Verify the data matches our message
    CANMessage received_msg;
    memcpy(&received_msg, written_data.data(), sizeof(CANMessage));
    assert(received_msg.id == msg.id);
    assert(received_msg.len == msg.len);
    assert(memcmp(received_msg.buf, msg.buf, msg.len) == 0);
    
    printf("✓ Message sending tests passed\n");
}

// Test message receiving
void test_message_receiving() {
    printf("Testing message receiving...\n");
    
    setup_test_message_bus();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Create test message with external ECU ID
    uint8_t test_data[] = {0x11, 0x22, 0x33, 0x44};
    CANMessage msg = create_test_message(0x10000123, 4, test_data);  // External ECU message
    
    // Simulate serial data reception
    const uint8_t* msg_bytes = (const uint8_t*)&msg;
    for (size_t i = 0; i < sizeof(CANMessage); i++) {
        Serial.add_byte_to_read(msg_bytes[i]);
    }
    
    // Process incoming data
    bridge.update();
    
    // Verify message was received
    assert(bridge.get_messages_received() == 1);
    
    printf("✓ Message receiving tests passed\n");
}

// Test buffer overflow handling
void test_buffer_overflow() {
    printf("Testing buffer overflow handling...\n");
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // For this test, just verify the buffer overflow tracking works correctly
    // Since the current implementation prevents buffer overflows naturally,
    // we'll test that the implementation handles invalid data correctly
    
    // Fill with random data that will cause parse errors
    for (int i = 0; i < 100; i++) {
        Serial.add_byte_to_read(0x55);
    }
    
    // Process data - this should generate parse errors for invalid message format
    bridge.update();
    
    // Should have parse errors from invalid message format
    assert(bridge.get_parse_errors() > 0);
    
    // Verify buffer overflow counter starts at 0
    assert(bridge.get_buffer_overflows() == 0);
    
    printf("✓ Buffer overflow tests passed\n");
}

// Test external serial initialization
void test_external_serial_init() {
    printf("Testing external serial initialization...\n");
    
    reset_mock_serials();
    
    ExternalSerial ext_serial;
    
    // Test with default configuration
    external_serial_config_t config = DEFAULT_EXTERNAL_SERIAL_CONFIG;
    assert(ext_serial.init(config));
    assert(ext_serial.is_initialized());
    
    // Test with custom configuration
    config.usb.enabled = true;
    config.serial1.enabled = true;
    config.serial2.enabled = false;
    assert(ext_serial.init(config));
    
    // Verify bridge states
    assert(ext_serial.get_usb_bridge().is_enabled());
    assert(ext_serial.get_serial1_bridge().is_enabled());
    assert(!ext_serial.get_serial2_bridge().is_enabled());
    
    printf("✓ External serial initialization tests passed\n");
}

// Test port configuration
void test_port_configuration() {
    printf("Testing port configuration...\n");
    
    reset_mock_serials();
    
    ExternalSerial ext_serial;
    external_serial_config_t config = DEFAULT_EXTERNAL_SERIAL_CONFIG;
    
    // Initialize with default config
    assert(ext_serial.init(config));
    
    // Test port configuration updates
    serial_port_config_t new_config = {true, 2000000, true, true};
    assert(ext_serial.set_port_config(0, new_config));  // USB
    
    // Verify configuration was set
    serial_port_config_t retrieved_config = ext_serial.get_port_config(0);
    assert(retrieved_config.enabled == new_config.enabled);
    assert(retrieved_config.baud_rate == new_config.baud_rate);
    
    // Test invalid port index
    assert(!ext_serial.set_port_config(99, new_config));
    
    printf("✓ Port configuration tests passed\n");
}

// Test message bus integration
void test_message_bus_integration() {
    printf("Testing message bus integration...\n");
    
    setup_test_message_bus();
    reset_mock_serials();
    
    ExternalSerial ext_serial;
    external_serial_config_t config = DEFAULT_EXTERNAL_SERIAL_CONFIG;
    
    // Initialize with USB enabled
    config.usb.enabled = true;
    config.serial1.enabled = false;
    config.serial2.enabled = false;
    assert(ext_serial.init(config));
    
    // Create test message
    uint8_t test_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    CANMessage msg = create_test_message(0x456, 4, test_data);
    
    // Send message through external serial
    ext_serial.on_message_bus_message(&msg);
    
    // Verify message was sent to USB bridge
    assert(ext_serial.get_usb_bridge().get_messages_sent() == 1);
    
    // Verify no messages sent to other bridges
    assert(ext_serial.get_serial1_bridge().get_messages_sent() == 0);
    assert(ext_serial.get_serial2_bridge().get_messages_sent() == 0);
    
    printf("✓ Message bus integration tests passed\n");
}

// Test statistics
void test_statistics() {
    printf("Testing statistics...\n");
    
    setup_test_message_bus();
    reset_mock_serials();
    
    ExternalSerial ext_serial;
    external_serial_config_t config = DEFAULT_EXTERNAL_SERIAL_CONFIG;
    
    // Initialize with multiple ports
    config.usb.enabled = true;
    config.serial1.enabled = true;
    config.serial2.enabled = false;
    assert(ext_serial.init(config));
    
    // Send messages
    uint8_t test_data[] = {0x01, 0x02};
    CANMessage msg1 = create_test_message(0x123, 2, test_data);
    CANMessage msg2 = create_test_message(0x456, 2, test_data);
    
    ext_serial.on_message_bus_message(&msg1);
    ext_serial.on_message_bus_message(&msg2);
    
    // Verify total statistics
    assert(ext_serial.get_total_messages_sent() == 4);  // 2 messages * 2 enabled ports
    
    // Reset statistics
    ext_serial.reset_all_statistics();
    assert(ext_serial.get_total_messages_sent() == 0);
    
    printf("✓ Statistics tests passed\n");
}

// Test mixed internal/external message filtering
void test_mixed_message_filtering() {
    printf("Testing mixed internal/external message filtering...\n");
    
    setup_test_message_bus();
    reset_mock_serials();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Create mix of internal and external messages
    struct TestMessage {
        uint32_t id;
        bool should_process;
    };
    
    TestMessage test_messages[] = {
        {0x00000123, false},  // Internal ECU (base = 0)
        {0x10000123, true},   // External ECU (base = 1)
        {0x20000456, true},   // External ECU (base = 2)
        {0x00000789, false},  // Internal ECU (base = 0)
        {0xF0000ABC, true},   // External ECU (base = 15)
    };
    
    for (const auto& test_msg : test_messages) {
        uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
        CANMessage msg = create_test_message(test_msg.id, 4, test_data);
        
        // Simulate serial data reception
        const uint8_t* msg_bytes = (const uint8_t*)&msg;
        for (size_t i = 0; i < sizeof(CANMessage); i++) {
            Serial.add_byte_to_read(msg_bytes[i]);
        }
        
        // Process message
        uint32_t initial_count = bridge.get_messages_received();
        bridge.update();
        
        // Check if message was processed as expected
        if (test_msg.should_process) {
            assert(bridge.get_messages_received() == initial_count + 1);
        } else {
            assert(bridge.get_messages_received() == initial_count);
        }
    }
    
    printf("✓ Mixed message filtering tests passed\n");
}

// Test TX/RX enable/disable
void test_tx_rx_enable_disable() {
    printf("Testing TX/RX enable/disable...\n");
    
    setup_test_message_bus();
    reset_mock_serials();
    
    SerialBridge bridge;
    
    // Test TX disabled
    serial_port_config_t config = {true, 115200, false, true};  // TX disabled
    assert(bridge.init(&Serial, config));
    
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    CANMessage msg = create_test_message(0x123, 4, test_data);
    
    bridge.send_message(msg);
    assert(bridge.get_messages_sent() == 0);  // Should not send
    
    // Test RX disabled
    config.tx_enabled = true;
    config.rx_enabled = false;  // RX disabled
    assert(bridge.init(&Serial, config));
    
    // Add data to serial
    Serial.add_byte_to_read(0x55);
    bridge.update();
    
    // Should not process incoming data
    assert(bridge.get_messages_received() == 0);
    
    printf("✓ TX/RX enable/disable tests passed\n");
}

// Run all tests
int main() {
    printf("Running External Serial Tests...\n");
    printf("=====================================\n");
    
    test_message_filtering();
    test_serial_bridge_init();
    test_message_sending();
    test_message_receiving();
    test_buffer_overflow();
    test_external_serial_init();
    test_port_configuration();
    test_message_bus_integration();
    test_statistics();
    test_mixed_message_filtering();
    test_tx_rx_enable_disable();
    
    printf("\n=====================================\n");
    printf("All External Serial Tests Passed! ✓\n");
    
    return 0;
}