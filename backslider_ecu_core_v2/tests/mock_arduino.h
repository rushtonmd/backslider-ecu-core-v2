// mock_arduino.h
// Enhanced mock Arduino environment for testing ECU code on desktop
//
// This provides all the Arduino/Teensy functions and constants needed
// for testing without actual hardware.

#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <cstdint>
#include <iostream>
#include <string>

// =============================================================================
// ARDUINO CONSTANTS
// =============================================================================

// Pin modes
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Digital states
#define LOW 0
#define HIGH 1

// =============================================================================
// ANALOG PIN DEFINITIONS (Teensy 4.1 compatible)
// =============================================================================

#define A0  14
#define A1  15
#define A2  16
#define A3  17
#define A4  18
#define A5  19
#define A6  20
#define A7  21
#define A8  22
#define A9  23
#define A10 24
#define A11 25
#define A12 26
#define A13 27
#define A14 38
#define A15 39
#define A16 40
#define A17 41

// =============================================================================
// MOCK TIMING FUNCTIONS
// =============================================================================

// Global time variables for mock timing
extern uint32_t mock_millis_time;
extern uint32_t mock_micros_time;

inline uint32_t millis() {
    return mock_millis_time;
}

inline uint32_t micros() {
    return mock_micros_time;
}

// Helper functions to control mock time in tests
inline void mock_set_millis(uint32_t time) {
    mock_millis_time = time;
}

inline void mock_set_micros(uint32_t time) {
    mock_micros_time = time;
}

inline void mock_advance_time_ms(uint32_t ms) {
    mock_millis_time += ms;
    mock_micros_time += (ms * 1000);
}

inline void mock_advance_time_us(uint32_t us) {
    mock_micros_time += us;
    mock_millis_time = mock_micros_time / 1000;
}

// =============================================================================
// MOCK ANALOG/DIGITAL I/O
// =============================================================================

// Mock ADC values for testing
extern uint16_t mock_analog_values[42];  // Enough for all pins

inline uint16_t analogRead(int pin) {
    if (pin >= 0 && pin < 42) {
        return mock_analog_values[pin];
    }
    return 2048;  // Default 12-bit mid-range value
}

inline int digitalRead(int pin) {
    // Simple mock - return HIGH for even pins, LOW for odd
    return (pin % 2) ? LOW : HIGH;
}

inline void pinMode(int pin, int mode) {
    // Mock function - does nothing in simulation
    (void)pin;
    (void)mode;
}

inline void digitalWrite(int pin, int value) {
    // Mock function - does nothing in simulation
    (void)pin;
    (void)value;
}

// =============================================================================
// MOCK SERIAL CLASS
// =============================================================================

class MockSerial {
public:
    void begin(unsigned long baud) {
        (void)baud;
        std::cout << "Mock Serial initialized at " << baud << " baud" << std::endl;
    }
    
    void print(const char* str) {
        std::cout << str;
    }
    
    void print(const std::string& str) {
        std::cout << str;
    }
    
    void print(int value) {
        std::cout << value;
    }
    
    void print(unsigned int value) {
        std::cout << value;
    }
    
    void print(long value) {
        std::cout << value;
    }
    
    void print(unsigned long value) {
        std::cout << value;
    }
    
    void print(float value) {
        std::cout << value;
    }
    
    void print(double value) {
        std::cout << value;
    }
    
    void print(int value, int format) {
        if (format == 16) {
            std::cout << std::hex << value << std::dec;
        } else {
            std::cout << value;
        }
    }
    
    void println() {
        std::cout << std::endl;
    }
    
    void println(const char* str) {
        std::cout << str << std::endl;
    }
    
    void println(const std::string& str) {
        std::cout << str << std::endl;
    }
    
    void println(int value) {
        std::cout << value << std::endl;
    }
    
    void println(unsigned int value) {
        std::cout << value << std::endl;
    }
    
    void println(long value) {
        std::cout << value << std::endl;
    }
    
    void println(unsigned long value) {
        std::cout << value << std::endl;
    }
    
    void println(float value) {
        std::cout << value << std::endl;
    }
    
    void println(double value) {
        std::cout << value << std::endl;
    }
};

// =============================================================================
// MOCK ADC FUNCTIONS
// =============================================================================

inline void analogReadResolution(int bits) {
    // Mock function - does nothing in simulation
    (void)bits;
}

inline void analogReadAveraging(int samples) {
    // Mock function - does nothing in simulation
    (void)samples;
}

// =============================================================================
// HELPER FUNCTIONS FOR TESTING
// =============================================================================

// Set mock analog reading for a specific pin
inline void mock_set_analog_reading(int pin, uint16_t value) {
    if (pin >= 0 && pin < 42) {
        mock_analog_values[pin] = value;
    }
}

// Set mock analog voltage for a specific pin (converts to 12-bit counts)
inline void mock_set_analog_voltage(int pin, float voltage) {
    uint16_t counts = (uint16_t)((voltage / 3.3f) * 4095.0f);
    mock_set_analog_reading(pin, counts);
}

// Reset all mock values to defaults
inline void mock_reset_all() {
    mock_millis_time = 0;
    mock_micros_time = 0;
    
    // Set all analog pins to mid-range (1.65V)
    for (int i = 0; i < 42; i++) {
        mock_analog_values[i] = 2048;
    }
}

// =============================================================================
// FORMAT CONSTANTS
// =============================================================================

#define HEX 16
#define DEC 10
#define BIN 2

#endif