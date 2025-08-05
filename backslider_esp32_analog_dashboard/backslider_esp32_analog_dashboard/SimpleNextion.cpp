/*
 * SimpleNextion.cpp - Lightweight Nextion Display Library Implementation
 * 
 * Simple write-only communication with Nextion displays
 */

#include "SimpleNextion.h"

SimpleNextion::SimpleNextion(HardwareSerial* serialPort) {
    serial = serialPort;
    initialized = false;
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
    
    // Send a simple command to test communication
    sendCommand("bkcmd=0"); // Disable return data for write-only mode
    delay(100);
    
    // Try to wake the display in case it's sleeping
    wake();
    delay(100);
    
    initialized = true;
    
    Serial.printf("Nextion: Initialized on baud %ld", baudRate);
    if (rxPin >= 0 && txPin >= 0) {
        Serial.printf(" (RX=%d, TX=%d)", rxPin, txPin);
    }
    Serial.println();
    
    return true;
}

void SimpleNextion::endCommand() {
    serial->write(0xFF);
    serial->write(0xFF);
    serial->write(0xFF);
}

void SimpleNextion::sendCommand(const char* command) {
    if (!initialized) return;
    
    serial->print(command);
    endCommand();
    
    // Small delay to prevent overwhelming the display
    delayMicroseconds(100);
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
    delay(500); // Give time for reset
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
    if (initialized) {
        serial->flush();
    }
}