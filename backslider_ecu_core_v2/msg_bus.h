// msg_bus.h
// Unified message bus using FlexCAN format for Backslider ECU
//
// ============================================================================
// MESSAGE BUS ARCHITECTURE OVERVIEW
// ============================================================================
//
// This message bus provides a unified communication system that treats internal
// and external (CAN) messaging identically. All messages use the standard CAN
// format (11-bit ID, 0-8 bytes payload) whether they stay local or go over
// physical CAN hardware.
//
// ARCHITECTURE:
//
//   ECU Modules (Fuel, Ignition, Sensors, etc.)
//        ↓ publish()     ↑ subscribe()
//   ┌─────────────────────────────────────────┐
//   │        Unified Message Bus              │
//   │  ┌─────────────┐    ┌─────────────────┐ │
//   │  │ Internal    │◄──►│ Physical CAN    │ │
//   │  │ Queue       │    │ Interface       │ │
//   │  │ (RAM)       │    │ (FlexCAN_T4)    │ │
//   │  └─────────────┘    └─────────────────┘ │
//   └─────────────────────────────────────────┘
//        ↓                        ↓
//   Local Subscribers        Physical CAN Bus
//                           (Other ECUs, Tools)
//
// MESSAGE FLOW:
//
// 1. PUBLISHING A MESSAGE:
//    - Module calls: PUBLISH_FLOAT(MSG_ENGINE_RPM, 3000.0f)
//    - Message added to internal queue for local delivery
//    - Message sent to physical CAN bus (if enabled)
//    - All happens in single publish() call
//
// 2. PROCESSING MESSAGES:
//    - bus.process() called from main loop
//    - Reads incoming CAN messages → adds to internal queue
//    - Processes internal queue → delivers to subscribers
//    - Subscribers receive messages regardless of source
//
// 3. SUBSCRIBER BEHAVIOR:
//    - Modules subscribe: bus.subscribe(MSG_ENGINE_RPM, handler)
//    - Handler called for EVERY message with that ID
//    - No distinction between local vs CAN-sourced messages
//    - Same handler processes both sources transparently
//
// KEY CONCEPTS:
//
// • UNIFIED FORMAT: All messages use CAN_message_t structure
//   - msg.id: 11-bit CAN identifier (0x000-0x7FF)
//   - msg.len: Data length (0-8 bytes, like CAN DLC)
//   - msg.buf[8]: Payload data (like CAN data field)
//
// • MESSAGE IDs: Organized by priority (automotive standard)
//   - 0x000-0x0FF: High priority (critical real-time)
//   - 0x100-0x1FF: Medium priority (control commands)
//   - 0x200-0x2FF: Low priority (status/diagnostics)
//   - 0x300-0x3FF: System messages
//
// • TRANSPARENT COMMUNICATION: Modules don't know message source
//   - Local module publishing RPM looks identical to
//   - Remote ECU sending RPM over physical CAN
//   - Both trigger same subscriber handlers
//
// • PHYSICAL CAN INTEGRATION: When enabled via init(true)
//   - Uses FlexCAN_T4 library for hardware interface
//   - Automatic bidirectional bridging with internal queue
//   - Standard automotive CAN bus protocols supported
//   - Compatible with CAN analysis tools (CANoe, etc.)
//
// USAGE EXAMPLES:
//
// // Initialize bus (internal only)
// g_message_bus.init(false);
//
// // Initialize with physical CAN at 500k baud
// g_message_bus.init(true, 500000);
//
// // Subscribe to engine RPM messages
// g_message_bus.subscribe(MSG_ENGINE_RPM, handle_rpm);
//
// // Publish RPM (goes to internal + CAN if enabled)
// PUBLISH_FLOAT(MSG_ENGINE_RPM, 3000.0f);
//
// // Publish internal-only message
// g_message_bus.publishFloat(MSG_DEBUG_MESSAGE, value, false);
//
// // Process all pending messages (call from main loop)
// g_message_bus.process();
//
// DEPLOYMENT SCENARIOS:
//
// • SINGLE ECU: All modules on one Teensy, internal queue only
// • DISTRIBUTED: Multiple ECUs connected via CAN bus
// • DEVELOPMENT: Desktop testing with mocked CAN interface
// • PRODUCTION: Real CAN bus with automotive diagnostic tools
//
// The same module code works in ALL scenarios without modification!
//
// ============================================================================



#ifndef MSG_BUS_H
#define MSG_BUS_H

#include "msg_definitions.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <iostream>
    #include <cstdio>
    // Mock timing functions for desktop (only if not already defined by mock_arduino.h)
    #ifndef TESTING
        inline uint32_t millis() { return 0; }
        inline uint32_t micros() { return 0; }
    #endif
#endif

class MessageBus {
public:
    // Configuration constants
    static const uint8_t MAX_SUBSCRIBERS = 32;  // Increased for comprehensive ECU module support
    static const uint16_t INTERNAL_QUEUE_SIZE = 128;
    
    // Constructor
    MessageBus();
    
    // Initialize the message bus
    void init();
    
    // Subscribe to a message ID
    bool subscribe(uint32_t msg_id, MessageHandler handler);
    
    // Publish a message to internal queue
    bool publish(uint32_t msg_id, const void* data, uint8_t length);
    
    // Process all pending messages (call from main loop)
    void process();
    
    // Helper methods for common data types
    bool publishFloat(uint32_t msg_id, float value);
    bool publishUint32(uint32_t msg_id, uint32_t value);
    bool publishUint16(uint32_t msg_id, uint16_t value);
    bool publishUint8(uint32_t msg_id, uint8_t value);
    
    // Statistics and diagnostics
    uint32_t getMessagesProcessed() const { return messages_processed; }
    uint32_t getQueueOverflows() const { return queue_overflows; }

    uint16_t getSubscriberCount() const { return subscriber_count; }
    
    // Queue status
    uint16_t getQueueSize() const;
    bool isQueueFull() const;
    
    // Reset statistics
    void resetStatistics();

    // Reset subscribers (for testing)
    void resetSubscribers();
    
    // Global broadcast callback (called for every message)
    static MessageHandler global_broadcast_handler;
    static void setGlobalBroadcastHandler(MessageHandler handler);
    static void clearGlobalBroadcastHandler();

private:
    // Subscriber management
    struct Subscriber {
        uint32_t msg_id;
        MessageHandler handler;
    };
    Subscriber subscribers[MAX_SUBSCRIBERS];
    uint8_t subscriber_count;
    
    // Internal message queue (circular buffer using FlexCAN format)
    CANMessage internal_queue[INTERNAL_QUEUE_SIZE];
    volatile uint16_t queue_head;
    volatile uint16_t queue_tail;
    
    // Statistics
    uint32_t messages_processed;
    uint32_t queue_overflows;
    
    // Internal methods
    bool enqueue_internal_message(const CANMessage& msg);
    void process_internal_queue();
    void deliver_to_subscribers(const CANMessage& msg);
    uint16_t next_queue_index(uint16_t index) const;
    
    // Debugging
    void debug_print(const char* message);
    void debug_print_message(const CANMessage& msg, const char* prefix);
};

// Global message bus instance
extern MessageBus g_message_bus;

// Convenience macros for publishing
#define PUBLISH_FLOAT(id, val) g_message_bus.publishFloat(id, val)
#define PUBLISH_UINT32(id, val) g_message_bus.publishUint32(id, val)
#define PUBLISH_UINT16(id, val) g_message_bus.publishUint16(id, val)
#define PUBLISH_UINT8(id, val) g_message_bus.publishUint8(id, val)

// Convenience macros for subscribing
#define SUBSCRIBE(id, handler) g_message_bus.subscribe(id, handler)

#endif