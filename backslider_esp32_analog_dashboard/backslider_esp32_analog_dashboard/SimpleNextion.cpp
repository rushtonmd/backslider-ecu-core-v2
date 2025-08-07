/*
 * SimpleNextion.cpp - Lightweight Nextion Display Library Implementation with Command Queue
 * 
 * Simple write-only communication with Nextion displays
 * Includes built-in command throttling to prevent overwhelming the display
 */

#include "SimpleNextion.h"

SimpleNextion::SimpleNextion(HardwareSerial* serialPort) {
    serial = serialPort;
    initialized = false;
    last_command_time = 0;
}

bool SimpleNextion::begin(long baudRate, int rxPin, int txPin) {
    // Initialize serial communication
    if (rxPin >= 0 && txPin >= 0) {
        // Use custom pins
        serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
    } else {
        // Use default pins
        serial->begin(baudRate);
    }
    
    delay(100); // Give time for serial to initialize
    
    // Send initial commands immediately (bypassing queue for setup)
    sendCommandImmediate("bkcmd=0"); // Disable return data for write-only mode
    delay(100);
    
    // Try to wake the display in case it's sleeping
    sendCommandImmediate("sleep=0");
    delay(100);
    
    initialized = true;
    last_command_time = millis();
    
    Serial.printf("Nextion: Initialized on baud %ld", baudRate);
    if (rxPin >= 0 && txPin >= 0) {
        Serial.printf(" (RX=%d, TX=%d)", rxPin, txPin);
    }
    Serial.println();
    
    return true;
}

void SimpleNextion::update() {
    if (!initialized || command_queue.empty()) return;
    
    unsigned long current_time = millis();
    if (current_time - last_command_time >= COMMAND_INTERVAL) {
        // Get the next command
        NextionCommand cmd = command_queue.front();
        command_queue.pop();
        
        // Send the command immediately
        sendCommandImmediate(cmd.command.c_str());
        last_command_time = current_time;
        
        // Debug: Warn if queue is getting large
        if (command_queue.size() > 20) {
            Serial.printf("⚠️ Nextion queue large: %d commands\n", command_queue.size());
        }
    }
}

void SimpleNextion::setThrottleInterval(unsigned long interval_ms) {
    // Allow changing the throttle interval if needed
    // Note: This changes a const, so we need to cast away const-ness
    const_cast<unsigned long&>(COMMAND_INTERVAL) = interval_ms;
}

void SimpleNextion::endCommand() {
    serial->write(0xFF);
    serial->write(0xFF);
    serial->write(0xFF);
}

void SimpleNextion::sendCommandImmediate(const char* command) {
    if (!initialized) return;
    
    serial->print(command);
    endCommand();
    
    // Small delay to prevent overwhelming the display
    delayMicroseconds(100);
}

void SimpleNextion::sendCommand(const char* command) {
    if (!initialized) return;
    
    NextionCommand cmd;
    cmd.command = String(command);
    cmd.timestamp = millis();
    
    command_queue.push(cmd);
}

void SimpleNextion::sendCommand(const String& command) {
    sendCommand(command.c_str());
}

void SimpleNextion::sendImmediate(const char* command) {
    // Emergency bypass of queue - use sparingly
    sendCommandImmediate(command);
    last_command_time = millis();
}

// Basic display control
void SimpleNextion::setBrightness(int brightness) {
    if (brightness < 0) brightness = 0;
    if (brightness > 100) brightness = 100;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "dim=%d", brightness);
    sendCommand(cmd);
}

void SimpleNextion::sleep() {
    sendCommand("sleep=1");
}

void SimpleNextion::wake() {
    sendCommand("sleep=0");
}

void SimpleNextion::reset() {
    sendCommand("rest");
    // Note: reset will clear the queue, so we might want to handle this specially
}

// Text object functions
void SimpleNextion::setText(const char* objectName, const char* text) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", objectName, text);
    sendCommand(cmd);
}

void SimpleNextion::setText(const char* objectName, String text) {
    setText(objectName, text.c_str());
}

void SimpleNextion::setTextFloat(const char* objectName, float value, int decimals) {
    char valueStr[32];
    dtostrf(value, 0, decimals, valueStr);
    setText(objectName, valueStr);
}

void SimpleNextion::setTextInt(const char* objectName, int value) {
    char valueStr[16];
    itoa(value, valueStr, 10);
    setText(objectName, valueStr);
}

// Number/Value object functions
void SimpleNextion::setValue(const char* objectName, int value) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.val=%d", objectName, value);
    sendCommand(cmd);
}

void SimpleNextion::setValueFloat(const char* objectName, float value) {
    setValue(objectName, (int)round(value));
}

// Color functions
void SimpleNextion::setBackgroundColor(const char* objectName, int color) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.bco=%d", objectName, color);
    sendCommand(cmd);
}

void SimpleNextion::setForegroundColor(const char* objectName, int color) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.pco=%d", objectName, color);
    sendCommand(cmd);
}

// Progress bar functions
void SimpleNextion::setProgress(const char* objectName, int percentage) {
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    setValue(objectName, percentage);
}

// Gauge/Meter functions
void SimpleNextion::setGaugeValue(const char* objectName, int value) {
    setValue(objectName, value);
}

// Picture functions
void SimpleNextion::setPicture(const char* objectName, int pictureId) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.pic=%d", objectName, pictureId);
    sendCommand(cmd);
}

// Page functions
void SimpleNextion::setPage(int pageId) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "page %d", pageId);
    sendCommand(cmd);
}

void SimpleNextion::setPage(const char* pageName) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "page %s", pageName);
    sendCommand(cmd);
}

// Visibility functions
void SimpleNextion::setVisible(const char* objectName, bool visible) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "vis %s,%d", objectName, visible ? 1 : 0);
    sendCommand(cmd);
}

// Utility functions
void SimpleNextion::flush() {
    if (!initialized) return;
    
    // Process all queued commands immediately (with throttling)
    while (!command_queue.empty()) {
        update();
        delay(COMMAND_INTERVAL); // Respect throttling even during flush
    }
    
    // Then flush the serial buffer
    serial->flush();
}