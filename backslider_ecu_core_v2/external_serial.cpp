// external_serial.cpp
// Simplified point-to-point serial communication implementation

#include "external_serial.h"
#include "msg_bus.h"
#include "parameter_helpers.h"
#ifdef TESTING
// In testing mode, Stream class is defined in mock_arduino.h
#else
#include <Stream.h>
#endif

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
    buffer_overflows(0),
    channel_id(0)
{
    config = {false, 115200, true, true};
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    current_message = {};
}

bool SerialBridge::init(void* port, const serial_port_config_t& port_config) {
    #ifdef ARDUINO
    Serial.println("SerialBridge::init() - Starting...");
    #endif
    
    if (port == nullptr) {
        #ifdef ARDUINO
        Serial.println("SerialBridge::init() - Port is null, returning false");
        #endif
        return false;
    }
    
    #ifdef ARDUINO
    Serial.println("SerialBridge::init() - Port is valid");
    #endif
    
    serial_port = port;
    config = port_config;
    enabled = config.enabled;
    
    #ifdef ARDUINO
    Serial.print("SerialBridge::init() - Enabled: ");
    Serial.println(enabled ? "true" : "false");
    #endif
    
    if (enabled) {
        #ifdef ARDUINO
        Serial.println("SerialBridge::init() - Port is enabled, about to call begin()...");
        #endif
        
        // Only call begin() if it's a HardwareSerial (not USB Serial)
        // USB Serial is already initialized by Arduino framework
        // Note: We can't use dynamic_cast due to -fno-rtti, so we'll assume
        // that if the port is not Serial (USB), it's a HardwareSerial
        #ifdef ARDUINO
        if (serial_port != &Serial) {
            Serial.println("SerialBridge::init() - Calling HardwareSerial::begin()...");
            static_cast<HardwareSerial*>(serial_port)->begin(config.baud_rate);
            Serial.println("SerialBridge::init() - HardwareSerial::begin() completed");
        } else {
            Serial.println("SerialBridge::init() - Skipping begin() for USB Serial");
        }
        #endif
        
        #ifdef ARDUINO
        Serial.println("SerialBridge::init() - Resetting state...");
        #endif
        
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
    
    #ifdef ARDUINO
    Serial.println("SerialBridge::init() - Completed successfully");
    #endif
    
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
    
    #ifdef ARDUINO
    // Debug: Show that update is being called
    static uint32_t last_update_debug = 0;
    uint32_t now = millis();
    if (now - last_update_debug >= 10000) {  // Every 10 seconds
        Serial.print("SerialBridge: Update called, enabled=");
        Serial.print(enabled ? "true" : "false");
        Serial.print(", rx_enabled=");
        Serial.println(config.rx_enabled ? "true" : "false");
        last_update_debug = now;
    }
    #endif

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
    
    #ifdef ARDUINO
    // Debug: Show bytes available
    static uint32_t last_bytes_debug = 0;
    uint32_t now = millis();
    Stream* stream = static_cast<Stream*>(serial_port);
    int available = stream->available();
    
    // Only show debug when there are actual bytes available
    if (available > 0) {
        Serial.print("!!! SerialBridge: process_incoming_bytes called with ");
        Serial.print(available);
        Serial.print(" bytes available - instance at ");
        Serial.print((uintptr_t)this, HEX);
        Serial.print(", serial_port pointer: ");
        Serial.print((uintptr_t)serial_port, HEX);
        Serial.print(", enabled: ");
        Serial.print(enabled ? "true" : "false");
        Serial.print(", config.baud_rate: ");
        Serial.println(config.baud_rate);
        Serial.flush();
    }
    
    if (available > 0 && (now - last_bytes_debug >= 1000)) {  // Every 1 second when data available
        Serial.print("SerialBridge: ");
        Serial.print(available);
        Serial.println(" bytes available for reading");
        last_bytes_debug = now;
    }
    #endif
    
    // Read available bytes into buffer
    // In test environment, this will use MockSerial methods
    // In Arduino environment, this will use real Serial methods
    #ifdef TESTING
    MockSerial* mock_serial = static_cast<MockSerial*>(serial_port);
    while (mock_serial->available() && get_buffer_available_space() > 0) {
        uint8_t byte = mock_serial->read();
        add_byte_to_buffer(byte);
    }
    #else
    // Cast to Stream* for Arduino environment
    while (stream->available() && get_buffer_available_space() > 0) {
        uint8_t byte = stream->read();
        add_byte_to_buffer(byte);
        
        #ifdef ARDUINO
        static uint32_t last_buffer_debug = 0;
        uint32_t buffer_now = millis();
        if (buffer_now - last_buffer_debug >= 1000) {  // Every 1 second
            Serial.print("SerialBridge: Added byte to buffer - buffer space: ");
            Serial.println(get_buffer_available_space());
            last_buffer_debug = buffer_now;
        }
        #endif
        
        #ifdef ARDUINO
        // Debug: Show bytes being received
        static uint32_t last_byte_debug = 0;
        uint32_t byte_now = millis();
        if (byte_now - last_byte_debug >= 2000) {  // Every 2 seconds
            Serial.print("SerialBridge: Received byte 0x");
            Serial.print(byte, HEX);
            Serial.print(", buffer bytes: ");
            Serial.println(bytes_received);
            last_byte_debug = byte_now;
        }
        #endif
    }
    #endif
    
    // Process buffered data
    uint8_t byte;
    
    #ifdef ARDUINO
    uint16_t buffer_data = (rx_head - rx_tail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
    // Only show debug when there are actual bytes in buffer
    if (buffer_data > 0) {
        Serial.print("SerialBridge: About to process buffered data - buffer has ");
        Serial.print(buffer_data);
        Serial.println(" bytes");
    }
    #endif
    
    while (get_byte_from_buffer(&byte)) {
        #ifdef ARDUINO
        Serial.println("SerialBridge: Processing byte from buffer");
        #endif
        #ifdef ARDUINO
        static uint32_t last_byte_debug = 0;
        uint32_t byte_now = millis();
        if (byte_now - last_byte_debug >= 1000) {  // Every 1 second
            Serial.print("SerialBridge: Processing byte from buffer - bytes_received=");
            Serial.println(bytes_received);
            last_byte_debug = byte_now;
        }
        #endif
        // Handle 0xFF 0xFF prefix for incoming messages
        if (bytes_received == 0) {
            // Starting a new message - look for prefix
            if (byte == 0xFF) {
                // First 0xFF found, wait for second
                bytes_received = 1;
                prefix_buffer[0] = byte;
                continue;
            } else {
                // Not a prefix byte, skip this byte (could be text debug output)
                continue;
            }
        } else if (bytes_received == 1) {
            // Second byte of potential prefix
            if (byte == 0xFF) {
                // Prefix found! Clear message buffer and start collecting CAN message
                memset(&current_message, 0, sizeof(CANMessage));
                bytes_received = 2; // Start collecting CAN message data
                #ifdef ARDUINO
                Serial.println("SerialBridge: Found 0xFF 0xFF prefix, starting CAN message collection");
                #endif
                continue;
            } else {
                // Not a prefix, reset and try again
                bytes_received = 0;
                continue;
            }
        }
        
        // Collect CAN message data (bytes_received >= 2)
        if (bytes_received < sizeof(CANMessage) + 2) { // +2 for prefix
            ((uint8_t*)&current_message)[bytes_received - 2] = byte; // -2 to account for prefix
            bytes_received++;
            
            #ifdef ARDUINO
            if (bytes_received == sizeof(CANMessage)) {
                Serial.println("SerialBridge: Reached sizeof(CANMessage) bytes");
            }
            #endif
            
            // Check if we have a complete message (prefix + CAN message)
            #ifdef ARDUINO
            if (bytes_received >= sizeof(CANMessage) + 2) {
                Serial.print("SerialBridge: Complete message check - bytes_received=");
                Serial.print(bytes_received);
                Serial.print(", sizeof(CANMessage)=");
                Serial.println(sizeof(CANMessage));
            }
            #endif
            
            if (bytes_received >= sizeof(CANMessage) + 2) {
                #ifdef ARDUINO
                Serial.println("!!! COMPLETE MESSAGE RECEIVED - ENTERING PROCESSING");
                Serial.print("SerialBridge: Complete message received, size=");
                Serial.print(sizeof(CANMessage));
                Serial.print(", CAN ID=0x");
                Serial.println(current_message.id, HEX);
                
                // Debug: Show structure field offsets and sizes
                Serial.print("SerialBridge: Structure analysis - ");
                Serial.print("sizeof(CANMessage)=");
                Serial.print(sizeof(CANMessage));
                Serial.print(", id_offset=");
                Serial.print((size_t)&current_message.id - (size_t)&current_message);
                Serial.print(", timestamp_offset=");
                Serial.print((size_t)&current_message.timestamp - (size_t)&current_message);
                Serial.print(", len_offset=");
                Serial.print((size_t)&current_message.len - (size_t)&current_message);
                Serial.print(", buf_offset=");
                Serial.println((size_t)&current_message.buf - (size_t)&current_message);
                
                // Debug: Show raw bytes received
                Serial.print("SerialBridge: Raw bytes: ");
                uint8_t* raw_bytes = (uint8_t*)&current_message;
                for (int i = 0; i < sizeof(CANMessage) && i < 12; i++) {  // Show first 12 bytes
                    Serial.print("0x");
                    if (raw_bytes[i] < 16) Serial.print("0");
                    Serial.print(raw_bytes[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Debug: Show individual fields
                Serial.print("SerialBridge: Fields - ID=0x");
                Serial.print(current_message.id, HEX);
                Serial.print(", timestamp=");
                Serial.print(current_message.timestamp);
                Serial.print(", len=");
                Serial.print(current_message.len);
                Serial.print(", flags.extended=");
                Serial.println(current_message.flags.extended ? "true" : "false");
                #endif
                
                #ifdef ARDUINO
                Serial.println("!!! CALLING process_complete_message()");
                #endif
                process_complete_message();
                // Reset for next message
                bytes_received = 0;
                parsing_message = false;
                // Clear any remaining buffer data to prevent mixing with next message
                clear_receive_buffer();
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
    
    // Check if this is a parameter message and add routing metadata
    #ifdef ARDUINO
    Serial.print("SerialBridge: Checking message length: ");
    Serial.print(current_message.len);
    Serial.print(" vs sizeof(parameter_msg_t): ");
    Serial.println(sizeof(parameter_msg_t));
    #endif
    
    if (current_message.len == sizeof(parameter_msg_t)) {
        parameter_msg_t* param = (parameter_msg_t*)current_message.buf;
        
        #ifdef ARDUINO
        // Debug: Show raw parameter message bytes
        Serial.print("SerialBridge: Raw parameter message bytes: ");
        for (int i = 0; i < sizeof(parameter_msg_t); i++) {
            Serial.print("0x");
            Serial.print(current_message.buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Debug: Show how operation is extracted
        Serial.print("SerialBridge: Extracting operation from offset 0: 0x");
        Serial.print(current_message.buf[0], HEX);
        Serial.print(" (");
        Serial.print(current_message.buf[0]);
        Serial.println(" decimal)");
        
        Serial.print("SerialBridge: Parameter message detected - operation=");
        Serial.print(param->operation);
        Serial.print(" (0x");
        Serial.print(param->operation, HEX);
        Serial.print("), value=");
        Serial.print(param->value);
        Serial.print(", channel=");
        Serial.print(param->source_channel);
        Serial.print(", request_id=");
        Serial.println(param->request_id);
        #endif
        
        // Only add routing for read/write requests (not responses)
        if (param->operation == PARAM_OP_READ_REQUEST || 
            param->operation == PARAM_OP_WRITE_REQUEST) {
            
            // Add routing metadata
            param->source_channel = channel_id;
            param->request_id = request_tracker.get_next_request_id();
            
            // Track this request
            request_tracker.add_request(channel_id, current_message.id);
            
            #ifdef ARDUINO
            Serial.print("SerialBridge: Added routing metadata - channel=");
            Serial.print(channel_id);
            Serial.print(", request_id=");
            Serial.println(param->request_id);
            #endif
        }
    }
    
    // Publish to internal message bus
    extern MessageBus g_message_bus;
    g_message_bus.publish(current_message.id, current_message.buf, current_message.len);
    
    #ifdef ARDUINO
    Serial.print("SerialBridge: Publishing message to bus - len=");
    Serial.print(current_message.len);
    Serial.print(", sizeof(parameter_msg_t)=");
    Serial.println(sizeof(parameter_msg_t));
    
    if (current_message.len == sizeof(parameter_msg_t)) {
        Serial.print("SerialBridge: Published parameter message to message bus - CAN ID=0x");
        Serial.println(current_message.id, HEX);
    }
    #endif
    
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
    bool should_process = (can_id & 0xF0000000) > 0;
    
    #ifdef ARDUINO
    Serial.print("SerialBridge: should_process_message(0x");
    Serial.print(can_id, HEX);
    Serial.print(") = ");
    Serial.println(should_process ? "true" : "false");
    #endif
    
    return should_process;
}

void SerialBridge::send_message_bytes(const CANMessage& msg) {
    // Debug: Check timestamp at send_message_bytes (disabled to reduce serial clutter)
    /*
    #ifdef ARDUINO
    static uint32_t last_send_debug = 0;
    uint32_t send_now = millis();
    if (send_now - last_send_debug >= 5000) {
        Serial.print("DEBUG: send_message_bytes - timestamp: ");
        Serial.println(msg.timestamp);
        last_send_debug = send_now;
    }
    #endif
    */
    
    // Send binary CAN message format with 0xFF 0xFF prefix
    // This ensures reliable binary message framing
    
    #ifdef ARDUINO
    Serial.print("SerialBridge: Sending binary response - CAN ID 0x");
    Serial.print(msg.id, HEX);
    Serial.print(", len=");
    Serial.println(msg.len);
    #endif
    
    // Send the binary message with prefix
    #ifdef TESTING
    MockSerial* mock_serial = static_cast<MockSerial*>(serial_port);
    // Send 0xFF 0xFF prefix
    mock_serial->write(0xFF);
    mock_serial->write(0xFF);
    // Send the CAN message
    mock_serial->write((const uint8_t*)&msg, sizeof(CANMessage));
    mock_serial->flush(); // Ensure immediate transmission
    #else
    // Cast to Stream* for Arduino environment
    Stream* stream = static_cast<Stream*>(serial_port);
    // Send 0xFF 0xFF prefix
    stream->write(0xFF);
    stream->write(0xFF);
    // Send the CAN message
    stream->write((const uint8_t*)&msg, sizeof(CANMessage));
    stream->flush(); // Ensure immediate transmission
    #endif
}

#ifdef TESTING
std::vector<uint8_t> SerialBridge::get_written_data_for_testing() {
    // Cast to MockSerial to access the test data
    MockSerial* mock_serial = static_cast<MockSerial*>(serial_port);
    if (mock_serial) {
        return mock_serial->get_written_data();
    }
    return std::vector<uint8_t>();
}
#endif

void SerialBridge::add_byte_to_buffer(uint8_t byte) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    if (next_head != rx_tail) { // Not full
        rx_buffer[rx_head] = byte;
        rx_head = next_head;
        #ifdef ARDUINO
        Serial.print("SerialBridge: add_byte_to_buffer - Added byte 0x");
        Serial.print(byte, HEX);
        Serial.print(" at rx_head=");
        Serial.println(rx_head);
        #endif
    } else {
        buffer_overflows++;
        #ifdef ARDUINO
        Serial.println("SerialBridge: add_byte_to_buffer - Buffer overflow!");
        #endif
    }
}

bool SerialBridge::get_byte_from_buffer(uint8_t* byte) {
    if (rx_head == rx_tail) {
        return false; // Empty
    }
    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    #ifdef ARDUINO
    Serial.print("SerialBridge: get_byte_from_buffer - Got byte 0x");
    Serial.print(*byte, HEX);
    Serial.print(" from rx_tail=");
    Serial.println(rx_tail);
    #endif
    return true;
}

void SerialBridge::clear_receive_buffer() {
    rx_head = rx_tail; // Reset buffer pointers to clear all data
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
    #ifdef ARDUINO
    Serial.println("ExternalSerial::init() - Starting...");
    #endif
    
    config = new_config;
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial::init() - Config copied");
    #endif
    
    // Initialize serial bridges
    bool usb_ok = true;
    bool serial1_ok = true;
    bool serial2_ok = true;
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial::init() - About to initialize USB bridge...");
    // On Arduino/Teensy platforms, Serial1 and Serial2 are predefined
    usb_ok = usb_bridge.init(&Serial, config.usb);
    Serial.println("ExternalSerial::init() - USB bridge initialized");
    
    #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
    Serial.println("ExternalSerial::init() - About to initialize Serial1 bridge...");
    serial1_ok = serial1_bridge.init(&Serial1, config.serial1);
    Serial.println("ExternalSerial::init() - Serial1 bridge initialized");
    
    Serial.println("ExternalSerial::init() - About to initialize Serial2 bridge...");
    serial2_ok = serial2_bridge.init(&Serial2, config.serial2);
    Serial.println("ExternalSerial::init() - Serial2 bridge initialized");
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
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial::init() - About to setup message bus integration...");
    #endif
    
    // Set up channel IDs for request tracking
    usb_bridge.set_channel_id(CHANNEL_SERIAL_USB);
    serial1_bridge.set_channel_id(CHANNEL_SERIAL_1);
    serial2_bridge.set_channel_id(CHANNEL_SERIAL_2);
    
    // Set up message bus integration
    setup_message_bus_integration();
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial::init() - Message bus integration setup complete");
    #endif
    
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
    #ifdef ARDUINO
    static uint32_t last_update_debug = 0;
    uint32_t update_now = millis();
    if (update_now - last_update_debug >= 5000) {  // Every 5 seconds
        Serial.println("!!! ExternalSerial::update() called");
        Serial.flush();
        last_update_debug = update_now;
    }
    #endif
    
    // Update USB bridge
    if (usb_bridge.is_enabled()) {
        usb_bridge.update();
    }
    
    // Update Serial1 bridge
    if (serial1_bridge.is_enabled()) {
        serial1_bridge.update();
    }
    
    // Update Serial2 bridge
    if (serial2_bridge.is_enabled()) {
        serial2_bridge.update();
    }
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
    
    // Check if this is a parameter response message
    if (msg->len == sizeof(parameter_msg_t)) {
        parameter_msg_t* param = (parameter_msg_t*)msg->buf;
        
        // Only filter parameter responses (not broadcasts or errors)
        if (param->operation == PARAM_OP_READ_RESPONSE || 
            param->operation == PARAM_OP_WRITE_ACK) {
            
            // Route response only to the requesting channel
            if (param->source_channel == CHANNEL_SERIAL_USB && usb_bridge.is_enabled()) {
                // Strip routing info before sending to external tool
                CANMessage external_response = *msg;
                strip_routing_metadata(&external_response);
                usb_bridge.send_message(external_response);
                
                // Remove from request tracker
                usb_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            else if (param->source_channel == CHANNEL_SERIAL_1 && serial1_bridge.is_enabled()) {
                // Strip routing info before sending to external tool
                CANMessage external_response = *msg;
                strip_routing_metadata(&external_response);
                serial1_bridge.send_message(external_response);
                
                // Remove from request tracker
                serial1_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            else if (param->source_channel == CHANNEL_SERIAL_2 && serial2_bridge.is_enabled()) {
                // Strip routing info before sending to external tool
                CANMessage external_response = *msg;
                strip_routing_metadata(&external_response);
                serial2_bridge.send_message(external_response);
                
                // Remove from request tracker
                serial2_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            
            return; // Don't broadcast parameter responses
        }
    }
    
    // For non-parameter messages or parameter broadcasts, send to all enabled ports
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
    
    // REMOVED: Legacy "blast everything" global broadcast handler
    // OLD: g_message_bus.setGlobalBroadcastHandler(on_internal_message_published);
    
    // NEW: Only handle parameter responses and selective broadcasts
    // The ExternalMessageBroadcasting system will handle selective message broadcasting
    // We only need to process:
    // 1. Parameter responses (for request/response routing)
    // 2. Specific broadcast messages (controlled by ExternalMessageBroadcasting)
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial: Set up selective message handling (no global broadcast flooding)");
    #endif
}

// NEW: Selective message handler - only called by ExternalMessageBroadcasting system
void ExternalSerial::on_selective_message(const CANMessage* msg) {
    if (!initialized || msg == nullptr) return;
    
    // Handle parameter responses with proper routing
    if (msg->len == sizeof(parameter_msg_t)) {
        parameter_msg_t* param = (parameter_msg_t*)msg->buf;
        
        // Route parameter responses to requesting channel only
        if (param->operation == PARAM_OP_READ_RESPONSE || 
            param->operation == PARAM_OP_WRITE_ACK ||
            param->operation == PARAM_OP_ERROR) {
            
            route_parameter_response(msg, param);
            return; // Don't broadcast parameter responses
        }
    }
    
    // Handle selective broadcasts (called by ExternalMessageBroadcasting)
    broadcast_to_enabled_ports(*msg);
}

// NEW: Route parameter responses to the correct requesting channel
void ExternalSerial::route_parameter_response(const CANMessage* msg, parameter_msg_t* param) {
    // Strip routing metadata before sending to external tool
    CANMessage external_response = *msg;
    strip_routing_metadata(&external_response);
    
    // Send response only to the channel that made the request
    switch (param->source_channel) {
        case CHANNEL_SERIAL_USB:
            if (usb_bridge.is_enabled()) {
                usb_bridge.send_message(external_response);
                usb_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            break;
            
        case CHANNEL_SERIAL_1:
            if (serial1_bridge.is_enabled()) {
                serial1_bridge.send_message(external_response);
                serial1_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            break;
            
        case CHANNEL_SERIAL_2:
            if (serial2_bridge.is_enabled()) {
                serial2_bridge.send_message(external_response);
                serial2_bridge.remove_pending_request(param->request_id, param->source_channel);
            }
            break;
            
        default:
            #ifdef ARDUINO
            Serial.print("ExternalSerial: Unknown source channel for parameter response: ");
            Serial.println(param->source_channel);
            #endif
            break;
    }
}

// NEW: Broadcast message to enabled ports (selective broadcasting only)
void ExternalSerial::broadcast_to_enabled_ports(const CANMessage& msg) {
    if (usb_bridge.is_enabled()) {
        usb_bridge.send_message(msg);
    }
    
    if (serial1_bridge.is_enabled()) {
        serial1_bridge.send_message(msg);
    }
    
    if (serial2_bridge.is_enabled()) {
        serial2_bridge.send_message(msg);
    }
}