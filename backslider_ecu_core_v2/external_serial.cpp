// external_serial.cpp
// Simplified point-to-point serial communication implementation

#include "external_serial.h"
#include "msg_bus.h"

// Global instance
ExternalSerial g_external_serial;

// Static pointer for message bus callback (since callbacks can't be member functions)
static ExternalSerial* g_serial_instance = nullptr;

// =============================================================================
// SERIAL BRIDGE IMPLEMENTATION
// =============================================================================

SerialBridge::SerialBridge() : 
    serial_port(nullptr),
    enabled(false),
    rx_head(0),
    rx_tail(0),
    bytes_received(0),
    parsing_message(false),
    messages_sent(0),
    messages_received(0),
    parse_errors(0),
    buffer_overflows(0)
{
    config = {false, 115200, true, true};
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    current_message = {};
}

bool SerialBridge::init(HardwareSerial* port, const serial_port_config_t& port_config) {
    if (port == nullptr) {
        return false;
    }
    
    serial_port = port;
    config = port_config;
    enabled = config.enabled;
    
    if (enabled) {
        serial_port->begin(config.baud_rate);
        
        // Reset state
        rx_head = 0;
        rx_tail = 0;
        bytes_received = 0;
        parsing_message = false;
        reset_statistics();
        
        #ifdef ARDUINO
        Serial.print("SerialBridge: Initialized port at ");
        Serial.print(config.baud_rate);
        Serial.println(" baud");
        #endif
    }
    
    return true;
}

void SerialBridge::shutdown() {
    enabled = false;
    serial_port = nullptr;
}

void SerialBridge::update() {
    if (!enabled || !config.rx_enabled) {
        return;
    }
    
    process_incoming_bytes();
}

void SerialBridge::send_message(const CANMessage& msg) {
    if (!enabled || !config.tx_enabled || serial_port == nullptr) {
        return;
    }
    
    send_message_bytes(msg);
    messages_sent++;
}

void SerialBridge::reset_statistics() {
    messages_sent = 0;
    messages_received = 0;
    parse_errors = 0;
    buffer_overflows = 0;
}

void SerialBridge::process_incoming_bytes() {
    if (!serial_port) return;
    
    // Read available bytes into buffer
    while (serial_port->available() && get_buffer_available_space() > 0) {
        uint8_t byte = serial_port->read();
        add_byte_to_buffer(byte);
    }
    
    // Process buffered data
    uint8_t byte;
    while (get_byte_from_buffer(&byte)) {
        // Add byte to current message
        if (bytes_received < sizeof(CANMessage)) {
            ((uint8_t*)&current_message)[bytes_received] = byte;
            bytes_received++;
            
            // Check if we have a complete message
            if (bytes_received >= sizeof(CANMessage)) {
                process_complete_message();
                // Reset for next message
                bytes_received = 0;
                parsing_message = false;
            }
        } else {
            // Overflow - start over
            handle_parse_error();
            bytes_received = 0;
            parsing_message = false;
        }
    }
}

void SerialBridge::process_complete_message() {
    // Validate message
    if (current_message.len > 8) {
        handle_parse_error();
        return;
    }
    
    // Filter messages - only process external ECU messages
    if (!should_process_message(current_message.id)) {
        return;
    }
    
    // Publish to internal message bus
    extern MessageBus g_message_bus;
    g_message_bus.publish(current_message.id, current_message.buf, current_message.len);
    
    messages_received++;
    
    #ifdef ARDUINO
    Serial.print("SerialBridge: Received CAN ID 0x");
    Serial.print(current_message.id, HEX);
    Serial.print(" len=");
    Serial.println(current_message.len);
    #endif
}

bool SerialBridge::should_process_message(uint32_t can_id) {
    // Only process messages with ECU base > 0 (external ECU messages)
    // This filters for messages from other ECUs and external tools
    return (can_id & 0xF0000000) > 0;
}

void SerialBridge::send_message_bytes(const CANMessage& msg) {
    // Send raw binary CAN message
    const uint8_t* data = (const uint8_t*)&msg;
    serial_port->write(data, sizeof(CANMessage));
    serial_port->flush(); // Ensure immediate transmission
}

void SerialBridge::add_byte_to_buffer(uint8_t byte) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    
    if (next_head == rx_tail) {
        // Buffer overflow
        buffer_overflows++;
        return;
    }
    
    rx_buffer[rx_head] = byte;
    rx_head = next_head;
}

bool SerialBridge::get_byte_from_buffer(uint8_t* byte) {
    if (rx_head == rx_tail) {
        return false; // Buffer empty
    }
    
    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return true;
}

uint16_t SerialBridge::get_buffer_available_space() const {
    if (rx_head >= rx_tail) {
        return RX_BUFFER_SIZE - (rx_head - rx_tail) - 1;
    } else {
        return rx_tail - rx_head - 1;
    }
}

void SerialBridge::handle_parse_error() {
    parse_errors++;
    debug_print("SerialBridge: Parse error");
}

void SerialBridge::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

// =============================================================================
// EXTERNAL SERIAL IMPLEMENTATION
// =============================================================================

ExternalSerial::ExternalSerial() : initialized(false) {
    config = DEFAULT_EXTERNAL_SERIAL_CONFIG;
}

bool ExternalSerial::init(const external_serial_config_t& new_config) {
    config = new_config;
    
    // Initialize serial bridges
    bool usb_ok = true;
    bool serial1_ok = true;
    bool serial2_ok = true;
    
    #ifdef ARDUINO
    // On Arduino/Teensy platforms, Serial1 and Serial2 are predefined
    usb_ok = usb_bridge.init(&Serial, config.usb);
    #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
    serial1_ok = serial1_bridge.init(&Serial1, config.serial1);
    serial2_ok = serial2_bridge.init(&Serial2, config.serial2);
    #else
    // For Arduino boards without Serial1/Serial2, skip them
    serial1_ok = !config.serial1.enabled;
    serial2_ok = !config.serial2.enabled;
    #endif
    #else
    // For testing, use mock serials
    extern MockSerial Serial, Serial1, Serial2;
    usb_ok = usb_bridge.init(&Serial, config.usb);
    serial1_ok = serial1_bridge.init(&Serial1, config.serial1);
    serial2_ok = serial2_bridge.init(&Serial2, config.serial2);
    #endif
    
    // Set up message bus integration
    setup_message_bus_integration();
    
    initialized = true;
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial: Initialized successfully");
    Serial.print("USB: "); Serial.println(config.usb.enabled ? "enabled" : "disabled");
    Serial.print("Serial1: "); Serial.println(config.serial1.enabled ? "enabled" : "disabled");
    Serial.print("Serial2: "); Serial.println(config.serial2.enabled ? "enabled" : "disabled");
    #endif
    
    return usb_ok && serial1_ok && serial2_ok;
}

void ExternalSerial::update() {
    if (!initialized) return;
    
    // Update all bridges
    usb_bridge.update();
    serial1_bridge.update();
    serial2_bridge.update();
}

void ExternalSerial::shutdown() {
    usb_bridge.shutdown();
    serial1_bridge.shutdown();
    serial2_bridge.shutdown();
    initialized = false;
}

bool ExternalSerial::set_port_config(int port_index, const serial_port_config_t& new_config) {
    switch (port_index) {
        case 0: // USB
            config.usb = new_config;
            return usb_bridge.init(&Serial, config.usb);
        case 1: // Serial1
            config.serial1 = new_config;
            #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
            return serial1_bridge.init(&Serial1, config.serial1);
            #else
            return !config.serial1.enabled; // Skip if not supported
            #endif
        case 2: // Serial2
            config.serial2 = new_config;
            #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
            return serial2_bridge.init(&Serial2, config.serial2);
            #else
            return !config.serial2.enabled; // Skip if not supported
            #endif
        default:
            return false;
    }
}

serial_port_config_t ExternalSerial::get_port_config(int port_index) const {
    switch (port_index) {
        case 0: return config.usb;
        case 1: return config.serial1;
        case 2: return config.serial2;
        default: return {false, 115200, false, false};
    }
}

void ExternalSerial::on_message_bus_message(const CANMessage* msg) {
    if (!initialized || msg == nullptr) return;
    
    // Send message to all enabled serial ports
    if (usb_bridge.is_enabled()) {
        usb_bridge.send_message(*msg);
    }
    
    if (serial1_bridge.is_enabled()) {
        serial1_bridge.send_message(*msg);
    }
    
    if (serial2_bridge.is_enabled()) {
        serial2_bridge.send_message(*msg);
    }
}

uint32_t ExternalSerial::get_total_messages_sent() const {
    return usb_bridge.get_messages_sent() + 
           serial1_bridge.get_messages_sent() + 
           serial2_bridge.get_messages_sent();
}

uint32_t ExternalSerial::get_total_messages_received() const {
    return usb_bridge.get_messages_received() + 
           serial1_bridge.get_messages_received() + 
           serial2_bridge.get_messages_received();
}

uint32_t ExternalSerial::get_total_parse_errors() const {
    return usb_bridge.get_parse_errors() + 
           serial1_bridge.get_parse_errors() + 
           serial2_bridge.get_parse_errors();
}

uint32_t ExternalSerial::get_total_buffer_overflows() const {
    return usb_bridge.get_buffer_overflows() + 
           serial1_bridge.get_buffer_overflows() + 
           serial2_bridge.get_buffer_overflows();
}

void ExternalSerial::reset_all_statistics() {
    usb_bridge.reset_statistics();
    serial1_bridge.reset_statistics();
    serial2_bridge.reset_statistics();
}

void ExternalSerial::setup_message_bus_integration() {
    // Set global instance pointer for callback
    g_serial_instance = this;
    
    // Subscribe to all message bus messages
    // Note: In a complete implementation, we would subscribe to the message bus
    // For now, the message bus would need to call our callback function
}

// Static callback function for message bus integration
void ExternalSerial::on_internal_message_published(const CANMessage* msg) {
    if (g_serial_instance && g_serial_instance->initialized) {
        g_serial_instance->on_message_bus_message(msg);
    }
}