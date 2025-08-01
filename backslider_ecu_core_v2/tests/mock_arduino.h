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
// MOCK SERIAL CLASS
// =============================================================================

class MockSerial : public HardwareSerial {
private:
    std::vector<uint8_t> rx_buffer;
    std::vector<uint8_t> tx_buffer;
    size_t rx_index = 0;
    
public:
    // HardwareSerial interface methods
    void begin(unsigned long baud) override {
        (void)baud;
        // Reset buffers on begin
        rx_buffer.clear();
        tx_buffer.clear();
        rx_index = 0;
    }
    
    int available() override {
        return static_cast<int>(rx_buffer.size() - rx_index);
    }
    
    int read() override {
        if (rx_index < rx_buffer.size()) {
            return static_cast<int>(rx_buffer[rx_index++]);
        }
        return -1;
    }
    
    size_t write(uint8_t byte) override {
        tx_buffer.push_back(byte);
        return 1;
    }
    
    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            tx_buffer.push_back(buffer[i]);
        }
        return size;
    }
    
    void flush() override {
        // Nothing to flush in mock implementation
    }
    
    // Test helper methods
    void add_byte_to_read(uint8_t byte) {
        rx_buffer.push_back(byte);
    }
    
    void add_data_to_read(const uint8_t* data, size_t size) {
        for (size_t i = 0; i < size; i++) {
            rx_buffer.push_back(data[i]);
        }
    }
    
    std::vector<uint8_t> get_written_data() const {
        return tx_buffer;
    }
    
    void clear_written_data() {
        tx_buffer.clear();
    }
    
    void clear_read_data() {
        rx_buffer.clear();
        rx_index = 0;
    }
    
    void reset() {
        rx_buffer.clear();
        tx_buffer.clear();
        rx_index = 0;
    }
    
    // Legacy print/println methods for compatibility
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



// =============================================================================
// MOCK WIRE (I2C) IMPLEMENTATION
// =============================================================================

class MockWire {
public:
    void begin() {
        // Mock I2C initialization (default pins)
    }
    
    void begin(uint8_t sda_pin, uint8_t scl_pin) {
        // Mock I2C initialization
        (void)sda_pin;
        (void)scl_pin;
    }
    
    void setClock(uint32_t frequency) {
        // Mock I2C clock setting
        (void)frequency;
    }
    
    void beginTransmission(uint8_t address) {
        // Mock I2C transmission start
        (void)address;
    }
    
    uint8_t endTransmission() {
        // Mock I2C transmission end
        return 0;  // Success
    }
    
    uint8_t requestFrom(uint8_t address, uint8_t quantity) {
        // Mock I2C request
        (void)address;
        (void)quantity;
        return 0;  // No data available
    }
    
    size_t write(uint8_t data) {
        // Mock I2C write
        (void)data;
        return 1;  // Success
    }
    
    int available() {
        // Mock I2C available
        return 0;  // No data
    }
    
    int read() {
        // Mock I2C read
        return 0;  // No data
    }
};

extern MockWire Wire;

// =============================================================================
// MOCK I2C DEVICE FUNCTIONS
// =============================================================================

// Global mock state for I2C devices
extern int16_t mock_ads1015_readings[4];
extern bool mock_mcp23017_pins[16];

// Mock ADS1015 ADC functions
inline int16_t mock_ads1015_read_channel(uint8_t channel) {
    // Return a mock ADC reading (0-32767 for 16-bit)
    if (channel < 4) {
        return mock_ads1015_readings[channel];
    }
    return 0;
}

inline void mock_set_ads1015_reading(uint8_t channel, int16_t value) {
    if (channel < 4) {
        mock_ads1015_readings[channel] = value;
    }
}

// Mock MCP23017 GPIO functions
inline bool mock_mcp23017_read_pin(uint8_t pin) {
    // Return a mock GPIO reading (HIGH by default for pullup inputs)
    if (pin < 16) {
        return mock_mcp23017_pins[pin];
    }
    return false;
}

inline void mock_set_mcp23017_pin(uint8_t pin, bool value) {
    if (pin < 16) {
        mock_mcp23017_pins[pin] = value;
    }
}

inline void mock_mcp23017_write_pin(uint8_t pin, bool value) {
    mock_set_mcp23017_pin(pin, value);
}

inline void mock_mcp23017_configure_pin(uint8_t pin, uint8_t mode) {
    // Mock pin configuration - just store the mode
    (void)pin;
    (void)mode;
}

// =============================================================================
// FORMAT CONSTANTS
// =============================================================================

#define HEX 16
#define DEC 10
#define BIN 2

#endif