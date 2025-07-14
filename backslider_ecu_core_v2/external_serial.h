// external_serial.h
// Daisy-chain serial communication module for Backslider ECU
//
// ============================================================================
// EXTERNAL SERIAL ARCHITECTURE OVERVIEW
// ============================================================================
//
// This module provides daisy-chained serial communication between multiple
// ECU units while maintaining the same CANMessage format used by the internal
// message bus. Multiple devices can be connected in a chain, and messages
// are automatically routed to their destinations.
//
// TOPOLOGY:
//
//   Primary ECU ←→ Secondary ECU ←→ Logging ECU ←→ Dashboard ECU
//     (ID: 1)       (ID: 2)         (ID: 3)        (ID: 4)
//       ↑             ↑               ↑              ↑
//   Internal        Internal       Internal       Internal
//   Message Bus     Message Bus    Message Bus    Message Bus
//
// PACKET FORMAT:
//
//   ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐
//   │ Sync Byte   │ Source ID   │ Dest ID     │ Packet Type │ CAN Message │ Checksum    │
//   │ (0xAA)      │ (1-254)     │ (1-254/0xFF)│ (1 byte)    │ (variable)  │ (CRC16)     │
//   └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘
//
// MESSAGE FLOW:
//
// 1. OUTBOUND (Internal → Serial):
//    - Module subscribes to internal message bus messages
//    - Applies forwarding rules to determine destination device
//    - Wraps CANMessage in SerialPacket with addressing
//    - Sends packet over serial with error detection
//
// 2. INBOUND (Serial → Internal):
//    - Receives serial data into circular buffer
//    - Parses packets and validates checksums
//    - If packet is for this device: forwards to internal message bus
//    - If packet is for another device: forwards it along the chain
//
// 3. DAISY-CHAIN FORWARDING:
//    - Each device checks destination ID of incoming packets
//    - Packets for other devices are automatically forwarded
//    - Broadcast packets (dest ID 0xFF) are processed by all devices
//    - Source ID prevents infinite loops
//
// DEVICE IDs:
//   - 0x01-0xFE: Individual device addresses
//   - 0xFF: Broadcast to all devices
//   - 0x00: Reserved/invalid
//
// PACKET TYPES:
//   - 0x01: Normal message
//   - 0x02: Acknowledgment
//   - 0x03: Negative acknowledgment
//   - 0x04: Ping request
//   - 0x05: Ping response
//
// USAGE EXAMPLES:
//
// // Initialize as Primary ECU
// external_serial.init(DEVICE_ID_PRIMARY_ECU, &Serial2, 2000000);
//
// // Forward engine RPM to Secondary ECU
// external_serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU);
//
// // Broadcast system status to all devices
// external_serial.subscribe_for_forwarding(MSG_ENGINE_STATUS, DEVICE_ID_BROADCAST);
//
// // Process in main loop
// external_serial.update();
//
// INTEGRATION WITH MESSAGE BUS:
//
// • Uses same CANMessage format as internal bus
// • Same MSG_ IDs for consistency
// • Transparent integration - application code unchanged
// • Automatic bidirectional message routing
// • Rate limiting prevents bus saturation
//
// ============================================================================

#ifndef EXTERNAL_SERIAL_H
#define EXTERNAL_SERIAL_H

#include "msg_definitions.h"
#include <Arduino.h>


// Device ID definitions
#define DEVICE_ID_INVALID           0x00
#define DEVICE_ID_PRIMARY_ECU       0x01    // Engine timing ECU
#define DEVICE_ID_SECONDARY_ECU     0x02    // Control/logging ECU
#define DEVICE_ID_LOGGING_ECU       0x03    // Dedicated data logger
#define DEVICE_ID_DASHBOARD         0x04    // Dashboard/display unit
#define DEVICE_ID_TUNING_TABLET     0x05    // Tuning software/tablet
#define DEVICE_ID_DIAGNOSTIC_TOOL   0x06    // Diagnostic equipment
#define DEVICE_ID_BROADCAST         0xFF    // Send to all devices

// Packet type definitions
typedef enum {
    PACKET_TYPE_NORMAL      = 0x01,    // Standard message
    PACKET_TYPE_ACK         = 0x02,    // Acknowledgment
    PACKET_TYPE_NACK        = 0x03,    // Negative acknowledgment
    PACKET_TYPE_PING        = 0x04,    // Ping request
    PACKET_TYPE_PONG        = 0x05,    // Ping response
    PACKET_TYPE_CONFIG      = 0x06,    // Configuration message
    PACKET_TYPE_ERROR       = 0x07     // Error notification
} packet_type_t;

// Serial packet structure (wraps CANMessage for addressing)
typedef struct __attribute__((packed)) {
    uint8_t     sync_byte;      // Always 0xAA (packet start marker)
    uint8_t     source_id;      // Device that sent this packet (1-254)
    uint8_t     dest_id;        // Target device (1-254, 0xFF=broadcast)
    uint8_t     packet_type;    // Type of packet (see packet_type_t)
    CANMessage  can_msg;        // Standard CAN message payload
    uint16_t    checksum;       // CRC16 for error detection
} serial_packet_t;

// Forwarding rule configuration
typedef struct {
    uint32_t msg_id;            // Which message ID to forward
    uint8_t  dest_device_id;    // Where to send it
    uint32_t last_sent_ms;      // Last transmission time
    uint32_t rate_limit_ms;     // Minimum time between sends (0=no limit)
    bool     enabled;           // Rule is active
} forwarding_rule_t;

// External Serial Communication Module
class ExternalSerial {
private:
    // Configuration constants
    static const uint16_t RX_BUFFER_SIZE = 1024;   // Receive buffer size
    static const uint16_t MAX_FORWARDING_RULES = 32; // Max forwarding rules
    static const uint32_t PACKET_TIMEOUT_MS = 100; // Packet reception timeout
    static const uint8_t  SYNC_BYTE = 0xAA;        // Packet sync marker
    
    // Device configuration
    uint8_t my_device_id;
    HardwareSerial* serial_port;
    uint32_t baud_rate;
    bool initialized;
    
    // Message forwarding rules
    forwarding_rule_t forwarding_rules[MAX_FORWARDING_RULES];
    uint8_t forwarding_rule_count;
    
    // Receive buffer and parsing state
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    
    // Packet parsing state machine
    typedef enum {
        PARSE_STATE_SYNC,
        PARSE_STATE_SOURCE,
        PARSE_STATE_DEST,
        PARSE_STATE_TYPE,
        PARSE_STATE_CAN_MSG,
        PARSE_STATE_CHECKSUM_HIGH,
        PARSE_STATE_CHECKSUM_LOW
    } parse_state_t;
    
    parse_state_t parse_state;
    serial_packet_t current_packet;
    uint16_t can_msg_bytes_received;
    uint32_t last_byte_time;
    
    // Statistics
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_forwarded;
    uint32_t checksum_errors;
    uint32_t timeout_errors;
    uint32_t buffer_overflows;
    
public:
    // Constructor
    ExternalSerial();
    
    // Initialization and main loop
    void init(uint8_t device_id, HardwareSerial* port, uint32_t baud = 2000000);
    void update();
    
    // Forwarding configuration
    bool subscribe_for_forwarding(uint32_t msg_id, uint8_t dest_device_id, uint32_t rate_limit_ms = 0);
    bool remove_forwarding_rule(uint32_t msg_id, uint8_t dest_device_id);
    void clear_forwarding_rules();
    
    // Direct message sending
    bool send_message_to_device(uint32_t msg_id, const void* data, uint8_t length, uint8_t dest_device_id);
    bool send_packet(const serial_packet_t& packet);
    
    // Utility functions
    bool ping_device(uint8_t device_id);
    bool is_device_online(uint8_t device_id);
    
    // Statistics and diagnostics
    uint32_t get_packets_sent() const { return packets_sent; }
    uint32_t get_packets_received() const { return packets_received; }
    uint32_t get_packets_forwarded() const { return packets_forwarded; }
    uint32_t get_checksum_errors() const { return checksum_errors; }
    uint32_t get_timeout_errors() const { return timeout_errors; }
    uint32_t get_buffer_overflows() const { return buffer_overflows; }
    uint8_t  get_device_id() const { return my_device_id; }
    uint16_t get_rx_buffer_level() const;
    uint8_t  get_forwarding_rule_count() const { return forwarding_rule_count; }
    
    // Reset functions
    void reset_statistics();
    void reset_parser();
    
private:
    // Internal message bus integration
    void setup_message_bus_integration();
    static void on_internal_message_published(const CANMessage* msg);
    
    // Packet processing
    void process_received_bytes();
    void process_complete_packet();
    bool validate_packet(const serial_packet_t& packet);
    void handle_packet_for_this_device(const serial_packet_t& packet);
    void forward_packet_to_next_device(const serial_packet_t& packet);
    
    // Forwarding logic
    bool should_forward_message(uint32_t msg_id, uint8_t* dest_device_id);
    bool is_rate_limited(uint32_t msg_id, uint8_t dest_device_id);
    void update_rate_limit_timestamp(uint32_t msg_id, uint8_t dest_device_id);
    
    // Buffer management
    void add_byte_to_buffer(uint8_t byte);
    bool get_byte_from_buffer(uint8_t* byte);
    uint16_t get_buffer_available_space() const;
    void reset_receive_buffer();
    
    // Packet creation and validation
    void create_packet(serial_packet_t* packet, uint8_t dest_id, packet_type_t type, const CANMessage& can_msg);
    uint16_t calculate_checksum(const serial_packet_t* packet);
    bool verify_checksum(const serial_packet_t* packet);
    
    // Serial I/O
    void send_packet_bytes(const serial_packet_t& packet);
    void send_bytes(const uint8_t* data, uint16_t length);
    
    // Error handling
    void handle_parse_error(const char* error_msg);
    void handle_checksum_error();
    void handle_timeout_error();
    
    // Debugging
    void debug_print_packet(const serial_packet_t* packet, const char* prefix);
    void debug_print(const char* message);
};

// Global instance
extern ExternalSerial g_external_serial;

// Utility macros
#define FORWARD_TO_DEVICE(msg_id, device_id) g_external_serial.subscribe_for_forwarding(msg_id, device_id)
#define FORWARD_BROADCAST(msg_id) g_external_serial.subscribe_for_forwarding(msg_id, DEVICE_ID_BROADCAST)
#define SEND_TO_DEVICE(msg_id, data, length, device_id) g_external_serial.send_message_to_device(msg_id, data, length, device_id)

#endif // EXTERNAL_SERIAL_H