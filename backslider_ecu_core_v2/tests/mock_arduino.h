// mock_arduino.h
// Enhanced mock Arduino environment for testing ECU code on desktop
//
// This provides all the Arduino/Teensy functions and constants needed
// for testing without actual hardware.

#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

// Define TESTING macro to prevent conflicts with msg_bus.h (only if not already defined)
#ifndef TESTING
#define TESTING
#endif

// Define platform macros for FlexCAN compatibility
#ifndef __IMXRT1062__
#define __IMXRT1062__
#endif

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

// Built-in LED pin
#define LED_BUILTIN 13

// Interrupt modes
#define RISING 1
#define FALLING 2
#define CHANGE 3

// Print format constants
#define HEX 16
#define DEC 10

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
// CAN MESSAGE SUPPORT FOR EXTERNAL CANBUS
// =============================================================================

// Mock CAN message structure (exact match to FlexCAN_T4 library)
struct CAN_message_t {
    uint32_t id = 0;          // can identifier
    uint16_t timestamp = 0;   // FlexCAN time when message arrived
    uint8_t idhit = 0;        // filter that id came from
    struct {
        bool extended = 0;    // identifier is extended (29-bit)
        bool remote = 0;      // remote transmission request packet type
        bool overrun = 0;     // message overrun
        bool reserved = 0;
    } flags;
    uint8_t len = 8;          // length of data
    uint8_t buf[8] = { 0 };   // data
    int8_t mb = 0;            // used to identify mailbox reception
    uint8_t bus = 0;          // used to identify where the message came from when events() is used.
    bool seq = 0;             // sequential frames
    
    CAN_message_t() = default;
    CAN_message_t(uint32_t id, uint8_t len, const uint8_t* data) 
        : id(id), len(len) {
        if (data && len <= 8) {
            for (uint8_t i = 0; i < len; i++) {
                buf[i] = data[i];
            }
        }
    }
};

// Backward compatibility typedef
typedef CAN_message_t MockCANMessage;

// =============================================================================
// FLEXCAN CONSTANTS (For linter compatibility - matches FlexCAN_T4 library)
// =============================================================================

#ifndef ARDUINO
// CAN device table enum (exact match to FlexCAN_T4 library)
typedef enum CAN_DEV_TABLE {
#if defined(__IMXRT1062__)
  CAN0 = (uint32_t)0x0,
  CAN1 = (uint32_t)0x401D0000,
  CAN2 = (uint32_t)0x401D4000,
  CAN3 = (uint32_t)0x401D8000
#endif
} CAN_DEV_TABLE;

// FlexCAN queue table enums (exact match to FlexCAN_T4 library)
typedef enum FLEXCAN_RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} FLEXCAN_RXQUEUE_TABLE;

typedef enum FLEXCAN_TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} FLEXCAN_TXQUEUE_TABLE;

// CAN frame type constants
typedef enum FLEXCAN_IDE {
  NONE = 0,
  EXT = 1,
  RTR = 2,
  STD = 3,
  INACTIVE
} FLEXCAN_IDE;

// Mock FlexCAN_T4 template class (for linter compatibility)
template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize = RX_SIZE_16, FLEXCAN_TXQUEUE_TABLE _txSize = TX_SIZE_16>
class FlexCAN_T4 {
public:
    void begin() { }
    void setBaudRate(uint32_t baudrate) { (void)baudrate; }
    bool write(const CAN_message_t& msg) { (void)msg; return true; }
    bool read(CAN_message_t& msg) { (void)msg; return false; }
    void setMaxMB(uint8_t mb) { (void)mb; }
    void enableFIFO(bool enable = true) { (void)enable; }
    void setFIFOFilter(uint8_t filter, uint32_t id, uint32_t mask) { (void)filter; (void)id; (void)mask; }
};

// Mock FlexCAN interface
class MockFlexCAN {
public:
    bool begin(uint32_t baudrate = 500000) { (void)baudrate; return true; }
    void setBaudRate(uint32_t baudrate) { (void)baudrate; }
    bool write(const CAN_message_t& msg) { (void)msg; return true; }
    bool read(CAN_message_t& msg) { (void)msg; return false; }
    void setMaxMB(uint8_t mb) { (void)mb; }
    void enableFIFO(bool enable = true) { (void)enable; }
    void setFIFOFilter(uint8_t filter, uint32_t id, uint32_t mask) { (void)filter; (void)id; (void)mask; }
};
#endif // ARDUINO

// =============================================================================
// MISC CONSTANTS
// =============================================================================

// Mock TIMING FUNCTIONS
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

// Mock digital values for testing
extern uint8_t mock_digital_values[56];  // Teensy 4.1 has pins 0-55

// Mock pin modes for testing
extern uint8_t mock_pin_modes[56];

inline uint16_t analogRead(int pin) {
    if (pin >= 0 && pin < 42) {
        return mock_analog_values[pin];
    }
    return 2048;  // Default 12-bit mid-range value
}

inline int digitalRead(int pin) {
    if (pin >= 0 && pin < 56) {
        return mock_digital_values[pin];
    }
    return HIGH;  // Default to HIGH
}

inline void pinMode(int pin, int mode) {
    if (pin >= 0 && pin < 56) {
        mock_pin_modes[pin] = mode;
        
        // If setting INPUT_PULLUP, default the pin to HIGH
        if (mode == INPUT_PULLUP) {
            mock_digital_values[pin] = HIGH;
        }
    }
}

inline void digitalWrite(int pin, int value) {
    if (pin >= 0 && pin < 56) {
        mock_digital_values[pin] = value ? HIGH : LOW;
    }
}

inline void analogWrite(int pin, int value) {
    // Mock PWM output - just store the value
    if (pin >= 0 && pin < 56) {
        mock_digital_values[pin] = value;
    }
}

inline void analogWriteFrequency(int pin, uint32_t frequency) {
    // Mock PWM frequency setting
    (void)pin;
    (void)frequency;
}

inline void analogWriteResolution(int resolution) {
    // Mock PWM resolution setting
    (void)resolution;
}

inline void delayMicroseconds(unsigned int us) {
    // Mock delay function - do nothing in testing
    (void)us;
}

inline void attachInterrupt(uint8_t interrupt_num, void (*isr)(), int mode) {
    // Mock interrupt attachment
    (void)interrupt_num;
    (void)isr;
    (void)mode;
}

inline uint8_t digitalPinToInterrupt(uint8_t pin) {
    // Mock pin to interrupt mapping - just return pin number
    return pin;
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
    
    void println(uint32_t value, int format) {
        if (format == HEX) {
            std::cout << std::hex << value << std::dec << std::endl;
        } else {
            std::cout << value << std::endl;
        }
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

// Set mock digital value for a specific pin
inline void mock_set_digital_value(int pin, int value) {
    if (pin >= 0 && pin < 56) {
        mock_digital_values[pin] = value ? HIGH : LOW;
    }
}

// Get mock pin mode for a specific pin
inline int mock_get_pin_mode(int pin) {
    if (pin >= 0 && pin < 56) {
        return mock_pin_modes[pin];
    }
    return INPUT;
}

// Reset all mock values to defaults
inline void mock_reset_all() {
    mock_millis_time = 0;
    mock_micros_time = 0;
    
    // Set all analog pins to mid-range (1.65V)
    for (int i = 0; i < 42; i++) {
        mock_analog_values[i] = 2048;
    }
    
    // Set all digital pins to HIGH (inactive for pullup inputs)
    for (int i = 0; i < 56; i++) {
        mock_digital_values[i] = HIGH;
        mock_pin_modes[i] = INPUT;
    }
}

// =============================================================================
// HARDWARE SERIAL BASE CLASS
// =============================================================================

// Abstract base class for serial communication
class HardwareSerial {
public:
    virtual ~HardwareSerial() = default;
    
    // Core serial interface methods
    virtual void begin(unsigned long baud) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t byte) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;
    virtual void flush() = 0;
};

// =============================================================================
// FORMAT CONSTANTS
// =============================================================================

#define HEX 16
#define DEC 10
#define BIN 2

#endif