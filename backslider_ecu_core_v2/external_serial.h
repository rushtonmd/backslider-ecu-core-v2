// external_serial.h
// Simplified point-to-point serial communication for Backslider ECU
//
// ============================================================================
// SIMPLIFIED SERIAL BRIDGE ARCHITECTURE
// ============================================================================
//
// This module provides simple serial-to-CAN bridges for direct communication
// between the ECU and external devices. Each serial port acts as an independent
// bridge that passes CAN messages bidirectionally.
//
// TOPOLOGY:
//
//   External Tool  ←→  USB Serial    ←→  ECU Message Bus
//   Dashboard      ←→  Serial1       ←→  ECU Message Bus  
//   Datalogger     ←→  Serial2       ←→  ECU Message Bus
//
// MESSAGE FORMAT:
//
//   Binary CAN message structure sent directly over serial:
//   ┌─────────────┬─────────────┬─────────────┬─────────────┐
//   │ CAN ID      │ Length      │ Data        │ Timestamp   │
//   │ (4 bytes)   │ (1 byte)    │ (0-8 bytes) │ (4 bytes)   │
//   └─────────────┴─────────────┴─────────────┴─────────────┘
//
// MESSAGE FLOW:
//
// 1. INBOUND (Serial → Message Bus):
//    - Receive binary CAN message from serial port
//    - Validate message format and extended CAN ID
//    - Filter for external ECU messages (ECU base > 0)
//    - Publish to internal message bus
//
// 2. OUTBOUND (Message Bus → Serial):
//    - Subscribe to internal message bus
//    - Send all messages over configured serial ports
//    - Direct binary CAN message transmission
//
// FILTERING:
//   - Only process messages with extended CAN ID ECU base > 0
//   - This includes messages from other ECUs and external tools
//   - Prevents processing of internal-only messages
//
// USAGE EXAMPLES:
//
// // Initialize all serial bridges
// external_serial_config_t config = {
//     .usb = {.enabled = true, .baud_rate = 2000000},
//     .serial1 = {.enabled = true, .baud_rate = 1000000},
//     .serial2 = {.enabled = false, .baud_rate = 115200}
// };
// g_external_serial.init(config);
//
// // Process in main loop
// g_external_serial.update();
//
// INTEGRATION WITH MESSAGE BUS:
//
// • Direct binary CAN message pass-through
// • Same CANMessage format as internal bus
// • Automatic bidirectional message routing
// • Perfect integration with extended CAN ID architecture
//
// ============================================================================

#ifndef EXTERNAL_SERIAL_H
#define EXTERNAL_SERIAL_H

#include "msg_definitions.h"
#include "request_tracker.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "tests/mock_arduino.h"
    extern class MockSerial Serial;
    extern class MockSerial Serial1;
    extern class MockSerial Serial2;
#endif

// Serial port configuration
struct serial_port_config_t {
    bool enabled;               // Port is active
    uint32_t baud_rate;         // Baud rate (default: 2000000)
    bool tx_enabled;            // Allow outbound messages (default: true)
    bool rx_enabled;            // Allow inbound messages (default: true)
};

// External serial configuration
struct external_serial_config_t {
    serial_port_config_t usb;       // USB Serial port
    serial_port_config_t serial1;   // Hardware Serial1
    serial_port_config_t serial2;   // Hardware Serial2
};

// Default configuration
static const external_serial_config_t DEFAULT_EXTERNAL_SERIAL_CONFIG = {
    .usb = {.enabled = true, .baud_rate = 2000000, .tx_enabled = true, .rx_enabled = true},
    .serial1 = {.enabled = false, .baud_rate = 1000000, .tx_enabled = true, .rx_enabled = true},
    .serial2 = {.enabled = false, .baud_rate = 115200, .tx_enabled = true, .rx_enabled = true}
};

// Serial bridge for a single port
class SerialBridge {
public:
    SerialBridge();
    
    // Configuration
    bool init(void* port, const serial_port_config_t& config);
    void shutdown();
    
    // Main update loop
    void update();
    
    // Message handling
    void send_message(const CANMessage& msg);
    
    // Status and statistics
    bool is_enabled() const { return enabled; }
    uint32_t get_messages_sent() const { return messages_sent; }
    uint32_t get_messages_received() const { return messages_received; }
    uint32_t get_parse_errors() const { return parse_errors; }
    uint32_t get_buffer_overflows() const { return buffer_overflows; }
    
    // Reset functions
    void reset_statistics();
    
    // Test helper methods (public for testing)
    bool should_process_message(uint32_t can_id);
    
    // Channel configuration
    void set_channel_id(uint8_t id) { channel_id = id; }
    uint8_t get_channel_id() const { return channel_id; }
    
    // Request tracking access
    void remove_pending_request(uint8_t request_id, uint8_t channel) {
        request_tracker.remove_request(request_id, channel);
    }
    
    #ifdef TESTING
    // Test-specific method to get written data
    std::vector<uint8_t> get_written_data_for_testing();
    #endif

private:
    // Configuration
    void* serial_port;
    serial_port_config_t config;
    bool enabled;
    
    // Receive buffer and parsing
    static const uint16_t RX_BUFFER_SIZE = 512;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    
    // Message parsing state
    CANMessage current_message;
    uint16_t bytes_received;
    bool parsing_message;
    
    // Prefix buffer for 0xFF 0xFF detection
    uint8_t prefix_buffer[2];
    
    // Statistics
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t parse_errors;
    uint32_t buffer_overflows;
    
    // Request tracking for parameter routing
    RequestTracker request_tracker;
    uint8_t channel_id;
    
    // Buffer management
    void add_byte_to_buffer(uint8_t byte);
    bool get_byte_from_buffer(uint8_t* byte);
    void clear_receive_buffer();
    uint16_t get_buffer_available_space() const;
    
    // Message processing
    void process_incoming_bytes();
    void process_complete_message();
    
    // Serial I/O
    void send_message_bytes(const CANMessage& msg);
    
    // Error handling
    void handle_parse_error();
    
    // Debug
    void debug_print(const char* message);
};

// Main external serial communication module
class ExternalSerial {
public:
    ExternalSerial();
    
    // Initialization and main loop
    bool init(const external_serial_config_t& config);
    void update();
    void shutdown();
    
    // Configuration management
    bool set_port_config(int port_index, const serial_port_config_t& config);
    serial_port_config_t get_port_config(int port_index) const;
    
    // Message bus integration
    void setup_message_bus_integration();
    
    // NEW: Selective message handling (replaces legacy global broadcast)
    void on_selective_message(const CANMessage* msg);
    void route_parameter_response(const CANMessage* msg, parameter_msg_t* param);
    void broadcast_to_enabled_ports(const CANMessage& msg);
    
    // Legacy method (deprecated)
    void on_message_bus_message(const CANMessage* msg);
    
    // Status and statistics
    bool is_initialized() const { return initialized; }
    uint32_t get_total_messages_sent() const;
    uint32_t get_total_messages_received() const;
    uint32_t get_total_parse_errors() const;
    uint32_t get_total_buffer_overflows() const;
    
    // Port access
    SerialBridge& get_usb_bridge() { return usb_bridge; }
    SerialBridge& get_serial1_bridge() { return serial1_bridge; }
    SerialBridge& get_serial2_bridge() { return serial2_bridge; }
    
    // Reset functions
    void reset_all_statistics();

private:
    // Configuration and state
    external_serial_config_t config;
    bool initialized;
    
    // Serial bridges
    SerialBridge usb_bridge;
    SerialBridge serial1_bridge;
    SerialBridge serial2_bridge;
    
    // Message bus integration
    static void on_internal_message_published(const CANMessage* msg);
};

// Global instance
extern ExternalSerial g_external_serial;

// Utility macros
#define EXTERNAL_SERIAL_INIT(config) g_external_serial.init(config)
#define EXTERNAL_SERIAL_UPDATE() g_external_serial.update()

#endif // EXTERNAL_SERIAL_H