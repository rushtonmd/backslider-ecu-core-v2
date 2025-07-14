// external_serial.cpp
// Daisy-chain serial communication implementation for Arduino/Teensy 4.1

#include "external_serial.h"
#include "msg_bus.h"

// Global instance
ExternalSerial g_external_serial;

// Static pointer for message bus callback (since callbacks can't be member functions)
static ExternalSerial* g_serial_instance = nullptr;

// CRC16 lookup table for fast checksum calculation
static const uint16_t crc16_table[256] PROGMEM = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

ExternalSerial::ExternalSerial() {
    my_device_id = DEVICE_ID_INVALID;
    serial_port = nullptr;
    baud_rate = 2000000;
    initialized = false;
    forwarding_rule_count = 0;
    rx_head = 0;
    rx_tail = 0;
    parse_state = PARSE_STATE_SYNC;
    can_msg_bytes_received = 0;
    last_byte_time = 0;
    packets_sent = 0;
    packets_received = 0;
    packets_forwarded = 0;
    checksum_errors = 0;
    timeout_errors = 0;
    buffer_overflows = 0;
    
    // Clear forwarding rules
    for (uint8_t i = 0; i < MAX_FORWARDING_RULES; i++) {
        forwarding_rules[i].msg_id = 0;
        forwarding_rules[i].dest_device_id = DEVICE_ID_INVALID;
        forwarding_rules[i].last_sent_ms = 0;
        forwarding_rules[i].rate_limit_ms = 0;
        forwarding_rules[i].enabled = false;
    }
    
    // Clear receive buffer
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    
    // Clear current packet
    current_packet = {};
}

void ExternalSerial::init(uint8_t device_id, HardwareSerial* port, uint32_t baud) {
    // Always set the device ID, even if other parameters are invalid
    my_device_id = device_id;
    baud_rate = baud;
    
    if (device_id == DEVICE_ID_INVALID || port == nullptr) {
        debug_print("ExternalSerial: Invalid device ID or serial port");
        serial_port = nullptr;
        return;
    }
    
    serial_port = port;
    
    // Initialize serial port
    serial_port->begin(baud_rate);
    
    // Set up message bus integration
    setup_message_bus_integration();
    
    // Reset state
    reset_parser();
    reset_statistics();
    
    initialized = true;
    
    char debug_msg[64];
    snprintf(debug_msg, sizeof(debug_msg), "ExternalSerial: Initialized device ID %d at %u baud", 
             device_id, (unsigned int)baud_rate);
    debug_print(debug_msg);
}

void ExternalSerial::update() {
    if (!initialized) return;
    
    // Process incoming serial data
    process_received_bytes();
    
    // Check for packet timeout
    if (parse_state != PARSE_STATE_SYNC && millis() - last_byte_time > PACKET_TIMEOUT_MS) {
        handle_timeout_error();
        reset_parser();
    }
}

void ExternalSerial::setup_message_bus_integration() {
    // Set global instance pointer for callback
    g_serial_instance = this;
    
    // Subscribe to ALL internal messages - we'll filter in the callback
    // Note: This is a simplified approach. In a real implementation, you might
    // want to subscribe to specific messages or use a different mechanism
    // For now, we'll implement a polling approach in the update() method
}

bool ExternalSerial::subscribe_for_forwarding(uint32_t msg_id, uint8_t dest_device_id, uint32_t rate_limit_ms) {
    if (forwarding_rule_count >= MAX_FORWARDING_RULES) {
        debug_print("ExternalSerial: Too many forwarding rules");
        return false;
    }
    
    // Check if rule already exists
    for (uint8_t i = 0; i < forwarding_rule_count; i++) {
        if (forwarding_rules[i].msg_id == msg_id && 
            forwarding_rules[i].dest_device_id == dest_device_id) {
            // Update existing rule
            forwarding_rules[i].rate_limit_ms = rate_limit_ms;
            forwarding_rules[i].enabled = true;
            return true;
        }
    }
    
    // Add new rule
    forwarding_rules[forwarding_rule_count].msg_id = msg_id;
    forwarding_rules[forwarding_rule_count].dest_device_id = dest_device_id;
    forwarding_rules[forwarding_rule_count].last_sent_ms = 0;
    forwarding_rules[forwarding_rule_count].rate_limit_ms = rate_limit_ms;
    forwarding_rules[forwarding_rule_count].enabled = true;
    forwarding_rule_count++;
    
    char debug_msg[80];
    snprintf(debug_msg, sizeof(debug_msg), "ExternalSerial: Added forwarding rule 0x%03X -> Device %d", 
             (unsigned int)msg_id, dest_device_id);
    debug_print(debug_msg);
    
    return true;
}

bool ExternalSerial::remove_forwarding_rule(uint32_t msg_id, uint8_t dest_device_id) {
    for (uint8_t i = 0; i < forwarding_rule_count; i++) {
        if (forwarding_rules[i].msg_id == msg_id && 
            forwarding_rules[i].dest_device_id == dest_device_id) {
            
            // Shift remaining rules down
            for (uint8_t j = i; j < forwarding_rule_count - 1; j++) {
                forwarding_rules[j] = forwarding_rules[j + 1];
            }
            forwarding_rule_count--;
            return true;
        }
    }
    return false;
}

void ExternalSerial::clear_forwarding_rules() {
    forwarding_rule_count = 0;
    for (uint8_t i = 0; i < MAX_FORWARDING_RULES; i++) {
        forwarding_rules[i].enabled = false;
    }
}

bool ExternalSerial::send_message_to_device(uint32_t msg_id, const void* data, uint8_t length, uint8_t dest_device_id) {
    if (!initialized || length > 8) return false;
    
    // Create CANMessage
    CANMessage can_msg;
    can_msg.id = msg_id;
    can_msg.len = length;
    memcpy(can_msg.buf, data, length);
    can_msg.timestamp = millis();
    
    // Create and send packet
    serial_packet_t packet;
    create_packet(&packet, dest_device_id, PACKET_TYPE_NORMAL, can_msg);
    
    return send_packet(packet);
}

bool ExternalSerial::send_packet(const serial_packet_t& packet) {
    if (!initialized) return false;
    
    send_packet_bytes(packet);
    packets_sent++;
    
    debug_print_packet(&packet, "TX");
    return true;
}

bool ExternalSerial::ping_device(uint8_t device_id) {
    if (!initialized) return false;
    
    CANMessage ping_msg;
    ping_msg.id = MSG_HEARTBEAT;
    ping_msg.len = 4;
    uint32_t timestamp = millis();
    memcpy(ping_msg.buf, &timestamp, sizeof(timestamp));
    
    serial_packet_t ping_packet;
    create_packet(&ping_packet, device_id, PACKET_TYPE_PING, ping_msg);
    
    return send_packet(ping_packet);
}

bool ExternalSerial::is_device_online(uint8_t device_id) {
    // For now, just return true - would need ping/pong tracking
    return true;
}

void ExternalSerial::process_received_bytes() {
    if (!initialized) return;
    
    // Read available bytes into buffer
    while (serial_port->available() && get_buffer_available_space() > 0) {
        uint8_t byte = serial_port->read();
        add_byte_to_buffer(byte);
        last_byte_time = millis();
    }
    
    // Process buffered data
    uint8_t byte;
    while (get_byte_from_buffer(&byte)) {
        switch (parse_state) {
            case PARSE_STATE_SYNC:
                if (byte == SYNC_BYTE) {
                    current_packet.sync_byte = byte;
                    parse_state = PARSE_STATE_SOURCE;
                }
                break;
                
            case PARSE_STATE_SOURCE:
                current_packet.source_id = byte;
                parse_state = PARSE_STATE_DEST;
                break;
                
            case PARSE_STATE_DEST:
                current_packet.dest_id = byte;
                parse_state = PARSE_STATE_TYPE;
                break;
                
            case PARSE_STATE_TYPE:
                current_packet.packet_type = byte;
                parse_state = PARSE_STATE_CAN_MSG;
                can_msg_bytes_received = 0;
                break;
                
            case PARSE_STATE_CAN_MSG:
                ((uint8_t*)&current_packet.can_msg)[can_msg_bytes_received] = byte;
                can_msg_bytes_received++;
                
                if (can_msg_bytes_received >= sizeof(CANMessage)) {
                    parse_state = PARSE_STATE_CHECKSUM_HIGH;
                }
                break;
                
            case PARSE_STATE_CHECKSUM_HIGH:
                current_packet.checksum = (byte << 8);
                parse_state = PARSE_STATE_CHECKSUM_LOW;
                break;
                
            case PARSE_STATE_CHECKSUM_LOW:
                current_packet.checksum |= byte;
                
                // Packet complete - process it
                process_complete_packet();
                reset_parser();
                break;
        }
    }
}

void ExternalSerial::process_complete_packet() {
    // Validate packet
    if (!validate_packet(current_packet)) {
        handle_checksum_error();
        return;
    }
    
    packets_received++;
    debug_print_packet(&current_packet, "RX");
    
    // Check if packet is for this device
    if (current_packet.dest_id == my_device_id || current_packet.dest_id == DEVICE_ID_BROADCAST) {
        handle_packet_for_this_device(current_packet);
    } else {
        // Forward to next device in chain
        forward_packet_to_next_device(current_packet);
    }
}

bool ExternalSerial::validate_packet(const serial_packet_t& packet) {
    return verify_checksum(&packet);
}

void ExternalSerial::handle_packet_for_this_device(const serial_packet_t& packet) {
    switch (packet.packet_type) {
        case PACKET_TYPE_NORMAL:
            // Forward CAN message to internal message bus
            // Note: In a real implementation, you would publish this to the internal message bus
            // For now, we'll just debug print it
            debug_print("Received message for internal message bus");
            break;
            
        case PACKET_TYPE_PING:
            // Respond to ping
            {
                CANMessage pong_msg;
                pong_msg.id = MSG_HEARTBEAT;
                pong_msg.len = 4;
                uint32_t timestamp = millis();
                memcpy(pong_msg.buf, &timestamp, sizeof(timestamp));
                
                serial_packet_t pong_packet;
                create_packet(&pong_packet, packet.source_id, PACKET_TYPE_PONG, pong_msg);
                send_packet(pong_packet);
            }
            break;
            
        case PACKET_TYPE_PONG:
            debug_print("Received pong response");
            break;
            
        default:
            debug_print("Received unknown packet type");
            break;
    }
}

void ExternalSerial::forward_packet_to_next_device(const serial_packet_t& packet) {
    // Prevent forwarding our own packets (loop prevention)
    if (packet.source_id == my_device_id) {
        return;
    }
    
    // Forward the packet unchanged
    send_packet_bytes(packet);
    packets_forwarded++;
    
    debug_print_packet(&packet, "FWD");
}

bool ExternalSerial::should_forward_message(uint32_t msg_id, uint8_t* dest_device_id) {
    for (uint8_t i = 0; i < forwarding_rule_count; i++) {
        if (forwarding_rules[i].enabled && forwarding_rules[i].msg_id == msg_id) {
            if (!is_rate_limited(msg_id, forwarding_rules[i].dest_device_id)) {
                *dest_device_id = forwarding_rules[i].dest_device_id;
                return true;
            }
        }
    }
    return false;
}

bool ExternalSerial::is_rate_limited(uint32_t msg_id, uint8_t dest_device_id) {
    uint32_t current_time = millis();
    
    for (uint8_t i = 0; i < forwarding_rule_count; i++) {
        if (forwarding_rules[i].msg_id == msg_id && 
            forwarding_rules[i].dest_device_id == dest_device_id) {
            
            if (forwarding_rules[i].rate_limit_ms == 0) {
                return false; // No rate limit
            }
            
            return (current_time - forwarding_rules[i].last_sent_ms) < forwarding_rules[i].rate_limit_ms;
        }
    }
    return false;
}

void ExternalSerial::update_rate_limit_timestamp(uint32_t msg_id, uint8_t dest_device_id) {
    uint32_t current_time = millis();
    
    for (uint8_t i = 0; i < forwarding_rule_count; i++) {
        if (forwarding_rules[i].msg_id == msg_id && 
            forwarding_rules[i].dest_device_id == dest_device_id) {
            forwarding_rules[i].last_sent_ms = current_time;
            break;
        }
    }
}

void ExternalSerial::add_byte_to_buffer(uint8_t byte) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    
    if (next_head == rx_tail) {
        // Buffer overflow
        buffer_overflows++;
        return;
    }
    
    rx_buffer[rx_head] = byte;
    rx_head = next_head;
}

bool ExternalSerial::get_byte_from_buffer(uint8_t* byte) {
    if (rx_head == rx_tail) {
        return false; // Buffer empty
    }
    
    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return true;
}

uint16_t ExternalSerial::get_buffer_available_space() const {
    if (rx_head >= rx_tail) {
        return RX_BUFFER_SIZE - (rx_head - rx_tail) - 1;
    } else {
        return rx_tail - rx_head - 1;
    }
}

void ExternalSerial::reset_receive_buffer() {
    rx_head = 0;
    rx_tail = 0;
}

void ExternalSerial::create_packet(serial_packet_t* packet, uint8_t dest_id, packet_type_t type, const CANMessage& can_msg) {
    packet->sync_byte = SYNC_BYTE;
    packet->source_id = my_device_id;
    packet->dest_id = dest_id;
    packet->packet_type = type;
    packet->can_msg = can_msg;
    packet->checksum = calculate_checksum(packet);
}

uint16_t ExternalSerial::calculate_checksum(const serial_packet_t* packet) {
    uint16_t crc = 0xFFFF;
    const uint8_t* data = (const uint8_t*)packet;
    
    // Calculate CRC16 over everything except the checksum field itself
    uint16_t length = sizeof(serial_packet_t) - sizeof(uint16_t);
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tbl_idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = ((crc << 8) ^ pgm_read_word(&crc16_table[tbl_idx])) & 0xFFFF;
    }
    
    return crc;
}

bool ExternalSerial::verify_checksum(const serial_packet_t* packet) {
    uint16_t calculated = calculate_checksum(packet);
    return calculated == packet->checksum;
}

void ExternalSerial::send_packet_bytes(const serial_packet_t& packet) {
    const uint8_t* data = (const uint8_t*)&packet;
    send_bytes(data, sizeof(serial_packet_t));
}

void ExternalSerial::send_bytes(const uint8_t* data, uint16_t length) {
    if (serial_port) {
        serial_port->write(data, length);
        serial_port->flush(); // Ensure data is sent immediately
    }
}

uint16_t ExternalSerial::get_rx_buffer_level() const {
    if (rx_head >= rx_tail) {
        return rx_head - rx_tail;
    } else {
        return RX_BUFFER_SIZE - rx_tail + rx_head;
    }
}

void ExternalSerial::reset_statistics() {
    packets_sent = 0;
    packets_received = 0;
    packets_forwarded = 0;
    checksum_errors = 0;
    timeout_errors = 0;
    buffer_overflows = 0;
}

void ExternalSerial::reset_parser() {
    parse_state = PARSE_STATE_SYNC;
    can_msg_bytes_received = 0;
    current_packet = {};
}

void ExternalSerial::handle_parse_error(const char* error_msg) {
    debug_print(error_msg);
    reset_parser();
}

void ExternalSerial::handle_checksum_error() {
    checksum_errors++;
    debug_print("ExternalSerial: Checksum error");
}

void ExternalSerial::handle_timeout_error() {
    timeout_errors++;
    debug_print("ExternalSerial: Packet timeout");
}

void ExternalSerial::debug_print_packet(const serial_packet_t* packet, const char* prefix) {
    char debug_msg[128];
    snprintf(debug_msg, sizeof(debug_msg), 
             "%s: SRC=%d DST=%d TYPE=%d ID=0x%03X LEN=%d CRC=0x%04X", 
             prefix, packet->source_id, packet->dest_id, packet->packet_type,
             (unsigned int)packet->can_msg.id, packet->can_msg.len, packet->checksum);
    debug_print(debug_msg);
}

void ExternalSerial::debug_print(const char* message) {
    Serial.println(message);
}

// Static callback function for message bus integration
void ExternalSerial::on_internal_message_published(const CANMessage* msg) {
    if (g_serial_instance && g_serial_instance->initialized) {
        // Check if this message should be forwarded
        uint8_t dest_device_id;
        if (g_serial_instance->should_forward_message(msg->id, &dest_device_id)) {
            g_serial_instance->send_message_to_device(msg->id, msg->buf, msg->len, dest_device_id);
            g_serial_instance->update_rate_limit_timestamp(msg->id, dest_device_id);
        }
    }
}