// msg_bus.cpp
// Unified message bus implementation using FlexCAN format

#include "msg_bus.h"

// Global message bus instance
MessageBus g_message_bus;

MessageBus::MessageBus() :
    subscriber_count(0),
    queue_head(0),
    queue_tail(0),
    messages_processed(0),
    queue_overflows(0)
{
    // Initialize subscriber array
    for (uint8_t i = 0; i < MAX_SUBSCRIBERS; i++) {
        subscribers[i].msg_id = 0;
        subscribers[i].handler = nullptr;
    }
}

void MessageBus::init() {
    debug_print("MessageBus: Initializing...");
    
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
    
    char debug_msg[64];
    snprintf(debug_msg, sizeof(debug_msg), "MessageBus: Subscribed to ID 0x%03X", msg_id);
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
    msg.id = msg_id;
    msg.len = length;
    memcpy(msg.buf, data, length);
    #ifdef ARDUINO
    msg.timestamp = micros();
    #else
    msg.timestamp = 0;  // Mock timestamp for desktop
    #endif
    
    // Add to internal queue
    if (!enqueue_internal_message(msg)) {
        queue_overflows++;
        debug_print("MessageBus: Internal queue overflow");
        return false;
    }
    
    return true;
}

bool MessageBus::publishFloat(uint32_t msg_id, float value) {
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
}

void MessageBus::resetSubscribers() {
    subscriber_count = 0;
    // Clear all subscriber entries
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        subscribers[i].msg_id = 0;
        subscribers[i].handler = nullptr;
    }
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
    Serial.print(": ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" LEN=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (uint8_t i = 0; i < msg.len; i++) {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    #else
    printf("%s: ID=0x%03X LEN=%d DATA=", prefix, msg.id, msg.len);
    for (uint8_t i = 0; i < msg.len; i++) {
        printf("%02X ", msg.buf[i]);
    }
    printf("\n");
    #endif
}