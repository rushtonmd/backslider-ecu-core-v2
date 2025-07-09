// mock_arduino.cpp
// Implementation of mock Arduino global variables

#include "mock_arduino.h"

// Global time variables
uint32_t mock_millis_time = 0;
uint32_t mock_micros_time = 0;

// Mock ADC values array (initialized to mid-range)
uint16_t mock_analog_values[42] = {
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
    2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
    2048, 2048
};