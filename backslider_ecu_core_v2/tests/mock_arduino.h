// tests/mock_arduino.h
// Mock Arduino functions for desktop testing

#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <cstdio>   // for snprintf

// Mock Arduino types
#define HIGH 1
#define LOW 0
#define LED_BUILTIN 13

// Pin mode constants
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Mock Arduino functions
inline uint32_t millis() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

inline uint32_t micros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
    // Mock - just print for testing
    std::cout << "digitalWrite(pin=" << (int)pin << ", value=" << (int)value << ")" << std::endl;
}

inline void pinMode(uint8_t pin, uint8_t mode) {
    // Mock - do nothing for testing
}

// Mock Serial class
class MockSerial {
public:
    void begin(uint32_t baud) {}
    void println(const char* str) { std::cout << str << std::endl; }
    void println(int val) { std::cout << val << std::endl; }
    void println(uint32_t val) { std::cout << val << std::endl; }
    void println(float val) { std::cout << val << std::endl; }
    void println() { std::cout << std::endl; }
    void print(const char* str) { std::cout << str; }
    void print(int val) { std::cout << val; }
    void print(uint32_t val) { std::cout << val; }
    void print(float val) { std::cout << val; }
    void print(int val, int format) { 
        if (format == 16) {
            std::cout << std::hex << val << std::dec;
        } else {
            std::cout << val;
        }
    }
    void print(uint32_t val, int format) { 
        if (format == 16) {
            std::cout << std::hex << val << std::dec;
        } else {
            std::cout << val;
        }
    }
};

// Format constants for Serial.print()
#define HEX 16
#define DEC 10

extern MockSerial Serial;

#endif