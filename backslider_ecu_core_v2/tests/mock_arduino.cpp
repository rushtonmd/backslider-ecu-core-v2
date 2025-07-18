// mock_arduino.cpp
// Implementation of mock Arduino environment for testing

#include "mock_arduino.h"
#include <iostream>
#include <cstring>
#include <vector>

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Mock timing variables
uint32_t mock_millis_time = 0;
uint32_t mock_micros_time = 0;

// Mock analog values (12-bit ADC)
uint16_t mock_analog_values[42] = {0};

// Mock digital values
uint8_t mock_digital_values[56] = {0};

// Mock pin modes
uint8_t mock_pin_modes[56] = {0};

// Global mock state for I2C devices
int16_t mock_ads1015_readings[4] = {16384, 16384, 16384, 16384}; // Mid-range by default
bool mock_mcp23017_pins[16] = {true, true, true, true, true, true, true, true, 
                               true, true, true, true, true, true, true, true}; // HIGH by default

// Mock Serial instances
MockSerial Serial;
MockSerial Serial1;
MockSerial Serial2;

// Mock Wire instance
MockWire Wire;

// =============================================================================
// INITIALIZATION
// =============================================================================

// Initialize all mock values to defaults
void mock_initialize() {
    mock_millis_time = 0;
    mock_micros_time = 0;
    
    // Set all analog pins to mid-range (1.65V for 3.3V reference)
    for (int i = 0; i < 42; i++) {
        mock_analog_values[i] = 2048;  // 12-bit mid-range
    }
    
    // Set all digital pins to HIGH (inactive for pullup inputs)
    for (int i = 0; i < 56; i++) {
        mock_digital_values[i] = HIGH;
        mock_pin_modes[i] = INPUT;
    }
    
    // Reset I2C device mock state
    for (int i = 0; i < 4; i++) {
        mock_ads1015_readings[i] = 16384; // Mid-range ADC readings
    }
    for (int i = 0; i < 16; i++) {
        mock_mcp23017_pins[i] = true; // HIGH for pullup inputs
    }
}

// =============================================================================
// MOCK FUNCTIONS
// =============================================================================

// Functions are defined inline in the header file to avoid conflicts

// =============================================================================
// AUTOMATIC INITIALIZATION
// =============================================================================

// Automatically initialize mock values when library is loaded
namespace {
    struct MockInitializer {
        MockInitializer() {
            mock_initialize();
        }
    };
    static MockInitializer g_mock_initializer;
}