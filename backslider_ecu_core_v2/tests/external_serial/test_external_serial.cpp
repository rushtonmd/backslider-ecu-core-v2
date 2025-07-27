// test_external_serial.cpp
// Tests for 0xFF 0xFF prefix-based binary serial communication

#include <cassert>
#include <cstring>
#include <cstdio>
#include <vector>
#include <string>
#include "../mock_arduino.h"
#include "../../external_serial.h"
#include "../../msg_bus.h"
#include "../../msg_definitions.h"
#include "../../parameter_registry.h"

// Mock Serial instances are defined in mock_arduino.cpp

// Test setup helpers
void setup_test_message_bus() {
    // Initialize message bus for testing
    extern MessageBus g_message_bus;
    g_message_bus.init();
}

void setup_test_parameter_registry() {
    // Initialize parameter registry for testing
    // Note: ParameterRegistry doesn't have an init() method, it's initialized automatically
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

// Create parameter message (8-byte payload)
CANMessage create_parameter_message(uint32_t id, uint8_t operation, float value, uint8_t source_channel, uint8_t request_id) {
    CANMessage msg;
    msg.id = id;
    msg.len = 8;
    
    // Pack parameter data: [operation][value][source_channel][request_id][reserved]
    uint8_t param_data[8];
    param_data[0] = operation;
    memcpy(&param_data[1], &value, 4);  // 4-byte float
    param_data[5] = source_channel;
    param_data[6] = request_id;
    param_data[7] = 0;  // reserved
    
    memcpy(msg.buf, param_data, 8);
    msg.timestamp = millis();
    return msg;
}

// Test binary prefix handling
void test_binary_prefix_handling() {
    printf("Testing binary prefix handling...\n");
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Test that outgoing messages include 0xFF 0xFF prefix
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    CANMessage msg = create_test_message(0x123, 4, test_data);
    
    // Send message
    bridge.send_message(msg);
    
    // Verify prefix was sent
    std::vector<uint8_t> written_data = bridge.get_written_data_for_testing();
    assert(written_data.size() >= 2);
    assert(written_data[0] == 0xFF);
    assert(written_data[1] == 0xFF);
    
    // Verify CAN message follows prefix
    assert(written_data.size() == 2 + sizeof(CANMessage));
    
    printf("✓ Binary prefix handling tests passed\n");
}

// Test incoming prefix parsing
void test_incoming_prefix_parsing() {
    printf("Testing incoming prefix parsing...\n");
    
    setup_test_message_bus();
    setup_test_parameter_registry();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Create parameter message
    CANMessage msg = create_parameter_message(0x10500001, 0x01, 23.5f, 1, 1);
    
    // Simulate serial data with prefix
    Serial.add_byte_to_read(0xFF);  // Prefix byte 1
    Serial.add_byte_to_read(0xFF);  // Prefix byte 2
    
    // Add CAN message bytes
    const uint8_t* msg_bytes = (const uint8_t*)&msg;
    for (size_t i = 0; i < sizeof(CANMessage); i++) {
        Serial.add_byte_to_read(msg_bytes[i]);
    }
    
    // Process incoming data
    bridge.update();
    
    // Verify message was received
    assert(bridge.get_messages_received() == 1);
    
    printf("✓ Incoming prefix parsing tests passed\n");
}

// Test mixed text/binary stream handling
void test_mixed_stream_handling() {
    printf("Testing mixed text/binary stream handling...\n");
    
    setup_test_message_bus();
    setup_test_parameter_registry();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Add some text data first
    const char* text_data = "DEBUG: Some debug message\n";
    for (size_t i = 0; i < strlen(text_data); i++) {
        Serial.add_byte_to_read(text_data[i]);
    }
    
    // Add binary message with prefix
    CANMessage msg = create_parameter_message(0x10500001, 0x01, 23.5f, 1, 1);
    Serial.add_byte_to_read(0xFF);
    Serial.add_byte_to_read(0xFF);
    const uint8_t* msg_bytes = (const uint8_t*)&msg;
    for (size_t i = 0; i < sizeof(CANMessage); i++) {
        Serial.add_byte_to_read(msg_bytes[i]);
    }
    
    // Add more text
    const char* more_text = "DEBUG: Another message\n";
    for (size_t i = 0; i < strlen(more_text); i++) {
        Serial.add_byte_to_read(more_text[i]);
    }
    
    // Process data
    bridge.update();
    
    // Should have received the binary message despite text interference
    assert(bridge.get_messages_received() == 1);
    
    printf("✓ Mixed stream handling tests passed\n");
}

// Test parameter message processing
void test_parameter_message_processing() {
    printf("Testing parameter message processing...\n");
    
    setup_test_message_bus();
    setup_test_parameter_registry();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Register a test parameter
    bool registered = ParameterRegistry::register_parameter(
        0x10500001,
        []() -> float { return 23.5f; },
        nullptr,
        "Test Parameter"
    );
    assert(registered);
    
    // Create parameter request
    CANMessage request = create_parameter_message(0x10500001, 0x01, 0.0f, 1, 1);
    
    // Simulate incoming request with prefix
    Serial.add_byte_to_read(0xFF);
    Serial.add_byte_to_read(0xFF);
    const uint8_t* msg_bytes = (const uint8_t*)&request;
    for (size_t i = 0; i < sizeof(CANMessage); i++) {
        Serial.add_byte_to_read(msg_bytes[i]);
    }
    
    // Process request
    bridge.update();
    
    // Should have received the request
    assert(bridge.get_messages_received() == 1);
    
    printf("✓ Parameter message processing tests passed\n");
}

// Test prefix filtering
void test_prefix_filtering() {
    printf("Testing prefix filtering...\n");
    
    setup_test_message_bus();
    setup_test_parameter_registry();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Add data without prefix - should be ignored
    for (int i = 0; i < 50; i++) {
        Serial.add_byte_to_read(0x55);
    }
    
    // Process data
    bridge.update();
    
    // Should not have received any messages
    assert(bridge.get_messages_received() == 0);
    
    // Add data with partial prefix - should be ignored
    Serial.add_byte_to_read(0xFF);
    Serial.add_byte_to_read(0xFE);  // Wrong second byte
    
    bridge.update();
    assert(bridge.get_messages_received() == 0);
    
    printf("✓ Prefix filtering tests passed\n");
}

// Test buffer management
void test_buffer_management() {
    printf("Testing buffer management...\n");
    
    setup_test_message_bus();
    setup_test_parameter_registry();
    reset_mock_serials();
    
    SerialBridge bridge;
    serial_port_config_t config = {true, 115200, true, true};
    
    // Initialize bridge
    assert(bridge.init(&Serial, config));
    
    // Test basic message processing
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    CANMessage msg = create_test_message(0x10000123, 4, test_data);  // External ECU message
    Serial.add_byte_to_read(0xFF);
    Serial.add_byte_to_read(0xFF);
    const uint8_t* msg_bytes = (const uint8_t*)&msg;
    for (size_t i = 0; i < sizeof(CANMessage); i++) {
        Serial.add_byte_to_read(msg_bytes[i]);
    }
    
    bridge.update();
    
    // Should have received the complete message
    assert(bridge.get_messages_received() == 1);
    
    printf("✓ Buffer management tests passed\n");
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

// Test message sending with prefix
void test_message_sending_with_prefix() {
    printf("Testing message sending with prefix...\n");
    
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
    
    // Verify binary data was written to serial with prefix
    std::vector<uint8_t> written_data = bridge.get_written_data_for_testing();
    assert(written_data.size() == 2 + sizeof(CANMessage));  // prefix + CAN message
    
    // Verify prefix
    assert(written_data[0] == 0xFF);
    assert(written_data[1] == 0xFF);
    
    // Verify the CAN message data
    CANMessage received_msg;
    memcpy(&received_msg, &written_data[2], sizeof(CANMessage));
    assert(received_msg.id == msg.id);
    assert(received_msg.len == msg.len);
    assert(memcmp(received_msg.buf, msg.buf, msg.len) == 0);
    
    printf("✓ Message sending with prefix tests passed\n");
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

// Test message bus integration with prefix
void test_message_bus_integration_with_prefix() {
    printf("Testing message bus integration with prefix...\n");
    
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
    
    printf("✓ Message bus integration with prefix tests passed\n");
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
    Serial.add_byte_to_read(0xFF);
    Serial.add_byte_to_read(0xFF);
    bridge.update();
    
    // Should not process incoming data
    assert(bridge.get_messages_received() == 0);
    
    printf("✓ TX/RX enable/disable tests passed\n");
}

// Run all tests
int main() {
    printf("Running External Serial Tests (0xFF 0xFF Prefix)...\n");
    printf("==================================================\n");
    
    test_binary_prefix_handling();
    test_incoming_prefix_parsing();
    test_mixed_stream_handling();
    test_parameter_message_processing();
    test_prefix_filtering();
    test_buffer_management();
    test_serial_bridge_init();
    test_message_sending_with_prefix();
    test_external_serial_init();
    test_port_configuration();
    test_message_bus_integration_with_prefix();
    test_statistics();
    test_tx_rx_enable_disable();
    
    printf("\n==================================================\n");
    printf("All External Serial Tests Passed! ✓\n");
    printf("✅ 0xFF 0xFF prefix implementation verified\n");
    printf("✅ Parameter message handling verified\n");
    printf("✅ Mixed text/binary stream processing verified\n");
    
    return 0;
}