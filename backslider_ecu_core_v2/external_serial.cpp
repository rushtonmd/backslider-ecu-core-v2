// external_serial.cpp
// Simplified point-to-point serial communication implementation

#include "external_serial.h"
#include "msg_bus.h"
#include "parameter_helpers.h"

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
    Stream* stream = static_cast<Stream*>(serial_port);
    while (stream->available() && get_buffer_available_space() > 0) {
        uint8_t byte = stream->read();
        add_byte_to_buffer(byte);
    }
    #endif
    
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
    
    // Check if this is a parameter message and add routing metadata
    if (current_message.len == sizeof(parameter_msg_t)) {
        parameter_msg_t* param = (parameter_msg_t*)current_message.buf;
        
        // Only add routing for read/write requests (not responses)
        if (param->operation == PARAM_OP_READ_REQUEST || 
            param->operation == PARAM_OP_WRITE_REQUEST) {
            
            // Add routing metadata
            param->source_channel = channel_id;
            param->request_id = request_tracker.get_next_request_id();
            
            // Track this request
            request_tracker.add_request(channel_id, current_message.id);
        }
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
    // In test environment, this will use MockSerial methods
    // In Arduino environment, this will use real Serial methods
    const uint8_t* data = (const uint8_t*)&msg;
    #ifdef TESTING
    MockSerial* mock_serial = static_cast<MockSerial*>(serial_port);
    mock_serial->write(data, sizeof(CANMessage));
    mock_serial->flush(); // Ensure immediate transmission
    #else
    // Cast to Stream* for Arduino environment
    Stream* stream = static_cast<Stream*>(serial_port);
    stream->write(data, sizeof(CANMessage));
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
    
    // Set up global broadcast handler to forward ALL messages to serial
    // This allows external tools to receive all message bus traffic
    g_message_bus.setGlobalBroadcastHandler(on_internal_message_published);
    
    #ifdef ARDUINO
    Serial.println("ExternalSerial: Set up global broadcast handler for ALL message bus messages");
    #endif
}

// Static callback function for message bus integration
void ExternalSerial::on_internal_message_published(const CANMessage* msg) {
    if (g_serial_instance && g_serial_instance->initialized) {
        g_serial_instance->on_message_bus_message(msg);
    }
}