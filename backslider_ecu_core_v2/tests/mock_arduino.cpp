// mock_arduino.cpp
// Implementation of mock Arduino environment for testing

#include "mock_arduino.h"

// =============================================================================
// GLOBAL MOCK VARIABLES
// =============================================================================

// Timing variables
uint32_t mock_millis_time = 0;
uint32_t mock_micros_time = 0;

// Analog pin mock values (12-bit ADC, 0-4095)
uint16_t mock_analog_values[42] = {
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,  // 0-9
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,  // 10-19
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,  // 20-29
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,  // 30-39
    2048, 2048                                                   // 40-41
};

// Digital pin mock values (0 = LOW, 1 = HIGH)
uint8_t mock_digital_values[56] = {
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  // 0-9
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  // 10-19
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  // 20-29
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  // 30-39
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  // 40-49
    HIGH, HIGH, HIGH, HIGH, HIGH, HIGH                          // 50-55
};

// Pin mode mock values (INPUT, OUTPUT, INPUT_PULLUP)
uint8_t mock_pin_modes[56] = {
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,  // 0-9
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,  // 10-19
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,  // 20-29
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,  // 30-39
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,  // 40-49
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT                              // 50-55
};

// =============================================================================
// MOCK WIRE (I2C) IMPLEMENTATION
// =============================================================================

MockWire Wire;

// Note: Serial objects are defined in individual test files