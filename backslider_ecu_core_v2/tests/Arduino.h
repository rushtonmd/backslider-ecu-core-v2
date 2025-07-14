// Arduino.h - Mock Arduino environment for testing
// This file is included when external_serial.h tries to include <Arduino.h> during testing

#ifndef ARDUINO_H_MOCK
#define ARDUINO_H_MOCK

// Include our full mock Arduino environment
#include "mock_arduino.h"

// External Serial object (defined by test files)
extern MockSerial Serial;

// Additional Arduino-specific items that might be needed
#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_word
#define pgm_read_word(addr) (*(addr))
#endif

#endif // ARDUINO_H_MOCK 