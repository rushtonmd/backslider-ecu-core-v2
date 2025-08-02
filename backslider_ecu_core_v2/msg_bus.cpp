// msg_bus.cpp
// Unified message bus implementation using FlexCAN format - Extended CAN ID Support

#include "msg_bus.h"

// Global message bus instance
MessageBus g_message_bus;

// Global broadcast handler (for external serial forwarding)
MessageHandler MessageBus::global_broadcast_handler = nullptr;

MessageBus::MessageBus() :
    subscriber_count(0),
    queue_head(0),
    queue_tail(0),
    messages_processed(0),
    queue_overflows(0),
    messages_published(0),
    messages_per_second(0),
    last_stats_reset_ms(0)
{
    // Initialize subscriber array
    for (uint8_t i = 0; i < MAX_SUBSCRIBERS; i++) {
        subscribers[i].msg_id = 0;
        subscribers[i].handler = nullptr;
    }
}

void MessageBus::init() {
    debug_print("MessageBus: Initializing with extended CAN ID support...");
    
    // Reset queue
    queue_head = 0;
    queue_tail = 0;
    
    // Reset statistics
    resetStatistics();
    
    debug_print("MessageBus: Initialization complete");
}

bool MessageBus::subscribe(uint32_t msg_id, MessageHandler handler) {
    if (subscriber_count >= MAX_SUBSCRIBERS || handler == nullptr) {
        debug_print("MessageBus: Subscribe failed - too many subscribers or null handler");
        return false;
    }
    
    subscribers[subscriber_count].msg_id = msg_id;
    subscribers[subscriber_count].handler = handler;
    subscriber_count++;
    
    char debug_msg[80];
    if (is_extended_can_id(msg_id)) {
        snprintf(debug_msg, sizeof(debug_msg), "MessageBus: Subscribed to Extended ID 0x%08X", msg_id);
    } else {
        snprintf(debug_msg, sizeof(debug_msg), "MessageBus: Subscribed to Standard ID 0x%03X", msg_id);
    }
    debug_print(debug_msg);
    
    return true;
}

bool MessageBus::publish(uint32_t msg_id, const void* data, uint8_t length) {
    if (length > 8) {
        debug_print("MessageBus: Publish failed - data too long");
        return false;
    }
    
    // Create CAN message
    CANMessage msg;
    
    // Set up message based on ID type
    if (is_extended_can_id(msg_id)) {
        create_extended_can_message(&msg, msg_id, data, length);
    } else {
        create_standard_can_message(&msg, msg_id, data, length);
    }
    
    #ifdef ARDUINO
    // Debug: Check if this is a parameter message
    if (length == 8) {  // parameter_msg_t is 8 bytes
        Serial.print("MessageBus: Publishing parameter message to queue - CAN ID 0x");
        Serial.println(msg_id, HEX);
    }
    #endif
    
    // Add to internal queue
    if (!enqueue_internal_message(msg)) {
        queue_overflows++;
        debug_print("MessageBus: Internal queue overflow");
        return false;
    }
    
    // Track message count
    messages_published++;
    
    return true;
}

bool MessageBus::publishFloat(uint32_t msg_id, float value) {
    // DEBUG: Show throttle position messages specifically
    #ifdef ARDUINO
    if (msg_id == MSG_THROTTLE_POSITION) {
        Serial.print("MessageBus: Published MSG_THROTTLE_POSITION = ");
        Serial.print(value);
        Serial.println("%");
    }
    
    // DEBUG: Show vehicle speed messages specifically
    if (msg_id == MSG_VEHICLE_SPEED) {
        Serial.print("MessageBus: Published MSG_VEHICLE_SPEED = ");
        Serial.print(value);
        Serial.print(" KPH (data bytes: ");
        uint8_t* bytes = (uint8_t*)&value;
        for (int i = 0; i < 4; i++) {
            Serial.print("0x");
            Serial.print(bytes[i], HEX);
            Serial.print(" ");
        }
        Serial.println(")");
    }
    #else
    if (msg_id == MSG_THROTTLE_POSITION) {
        printf("MessageBus: Published MSG_THROTTLE_POSITION = %.2f%%\n", value);
    }
    #endif
    
    return publish(msg_id, &value, sizeof(float));
}

bool MessageBus::publishUint32(uint32_t msg_id, uint32_t value) {
    return publish(msg_id, &value, sizeof(uint32_t));
}

bool MessageBus::publishUint16(uint32_t msg_id, uint16_t value) {
    return publish(msg_id, &value, sizeof(uint16_t));
}

bool MessageBus::publishUint8(uint32_t msg_id, uint8_t value) {
    return publish(msg_id, &value, sizeof(uint8_t));
}

void MessageBus::process() {
    // Process internal message queue
    process_internal_queue();
    
    // Update messages per second statistics
    uint32_t now_ms = millis();
    if (now_ms - last_stats_reset_ms >= 1000) {  // Every second
        messages_per_second = messages_published;
        messages_published = 0;  // Reset counter
        last_stats_reset_ms = now_ms;
    }
}

// Private methods

bool MessageBus::enqueue_internal_message(const CANMessage& msg) {
    uint16_t next_head = next_queue_index(queue_head);
    
    if (next_head == queue_tail) {
        // Queue is full
        return false;
    }
    
    internal_queue[queue_head] = msg;
    queue_head = next_head;
    
    return true;
}

void MessageBus::process_internal_queue() {
    while (queue_tail != queue_head) {
        const CANMessage& msg = internal_queue[queue_tail];
        
        // Deliver to all subscribers
        deliver_to_subscribers(msg);
        
        // Move to next message
        queue_tail = next_queue_index(queue_tail);
        messages_processed++;
    }
}

void MessageBus::deliver_to_subscribers(const CANMessage& msg) {
    // Temporarily disabled to avoid serial corruption
    /*
    #ifdef ARDUINO
    static uint32_t last_delivery_debug = 0;
    uint32_t now = millis();
    if (now - last_delivery_debug >= 5000) {
            // Serial.print("DEBUG: MessageBus delivery - timestamp: ");
    // Serial.println(msg.timestamp);
        last_delivery_debug = now;
    }
    #endif
    */
    
    // Call global broadcast handler first (for external serial forwarding)
    if (global_broadcast_handler != nullptr) {
        #ifdef ARDUINO
        // Debug: Check if this is a parameter message
        if (msg.len == 8) {  // parameter_msg_t is 8 bytes
            Serial.print("MessageBus: Calling global broadcast handler for parameter message - CAN ID 0x");
            Serial.println(msg.id, HEX);
        }
        #endif
        global_broadcast_handler(&msg);
    }
    
    // Deliver to specific subscribers
    for (uint8_t i = 0; i < subscriber_count; i++) {
        if (subscribers[i].msg_id == msg.id && subscribers[i].handler != nullptr) {
            subscribers[i].handler(&msg);
        }
    }
}

uint16_t MessageBus::next_queue_index(uint16_t index) const {
    return (index + 1) % INTERNAL_QUEUE_SIZE;
}

uint16_t MessageBus::getQueueSize() const {
    if (queue_head >= queue_tail) {
        return queue_head - queue_tail;
    } else {
        return INTERNAL_QUEUE_SIZE - queue_tail + queue_head;
    }
}

bool MessageBus::isQueueFull() const {
    return next_queue_index(queue_head) == queue_tail;
}

void MessageBus::resetStatistics() {
    messages_processed = 0;
    queue_overflows = 0;
    messages_published = 0;
    messages_per_second = 0;
    last_stats_reset_ms = millis();
}

void MessageBus::resetSubscribers() {
    subscriber_count = 0;
    // Clear all subscriber entries
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        subscribers[i].msg_id = 0;
        subscribers[i].handler = nullptr;
    }
}

void MessageBus::setGlobalBroadcastHandler(MessageHandler handler) {
    global_broadcast_handler = handler;
}

void MessageBus::clearGlobalBroadcastHandler() {
    global_broadcast_handler = nullptr;
}

void MessageBus::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void MessageBus::debug_print_message(const CANMessage& msg, const char* prefix) {
    #ifdef ARDUINO
    Serial.print(prefix);
    Serial.print(": ");
    
    // Print ID based on type
    #ifdef ARDUINO
    if (msg.flags.extended) {
        Serial.print("Extended ID=0x");
        Serial.print(msg.id, HEX);
    } else {
        Serial.print("Standard ID=0x");
        Serial.print(msg.id, HEX);
    }
    #else
    if (is_extended_can_id(msg.id)) {
        Serial.print("Extended ID=0x");
        Serial.print(msg.id, HEX);
    } else {
        Serial.print("Standard ID=0x");
        Serial.print(msg.id, HEX);
    }
    #endif
    
    Serial.print(" LEN=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.buf[i] < 0x10) Serial.print("0");
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    
    // Print extended ID breakdown for debugging
    #ifdef ARDUINO
    if (msg.flags.extended) {
    #else
    if (is_extended_can_id(msg.id)) {
    #endif
        Serial.print(" [ECU=0x");
        Serial.print(GET_ECU_BASE(msg.id) >> 28, HEX);
        Serial.print(" SUB=0x");
        Serial.print(GET_SUBSYSTEM(msg.id) >> 20, HEX);
        Serial.print(" PARAM=0x");
        Serial.print(GET_PARAMETER(msg.id), HEX);
        Serial.print("]");
    }
    
    Serial.println();
    #else
    // Desktop/test environment
    if (is_extended_can_id(msg.id)) {
        printf("%s: Extended ID=0x%08X LEN=%d DATA=", prefix, msg.id, msg.len);
    } else {
        printf("%s: Standard ID=0x%03X LEN=%d DATA=", prefix, msg.id, msg.len);
    }
    
    for (uint8_t i = 0; i < msg.len; i++) {
        printf("%02X ", msg.buf[i]);
    }
    
    // Print extended ID breakdown for debugging
    if (is_extended_can_id(msg.id)) {
        printf(" [ECU=0x%X SUB=0x%02X PARAM=0x%05X]", 
               GET_ECU_BASE(msg.id) >> 28,
               GET_SUBSYSTEM(msg.id) >> 20,
               GET_PARAMETER(msg.id));
    }
    
    printf("\n");
    #endif
}