// transmission_module.h
// Transmission control module for manual paddle shifting
//
// This module handles:
// - Transmission fluid temperature monitoring
// - Paddle shifter inputs (upshift/downshift)
// - Gear position detection (P, R, N, D, 2, 1)
// - State management and safety logic

#ifndef TRANSMISSION_MODULE_H
#define TRANSMISSION_MODULE_H

#include <stdint.h>
#include "input_manager_types.h"
#include "msg_definitions.h"
#include "pin_assignments.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
// Mock functions for desktop testing
#ifndef MOCK_ARDUINO_H
inline uint32_t millis() { return 0; }
#endif
#endif

// =============================================================================
// TRANSMISSION CONFIGURATION
// =============================================================================

// Timing configuration
#define PADDLE_DEBOUNCE_MS      200        // Minimum time between shifts (configurable)

// Thermistor configuration for transmission fluid temperature
#define TRANS_TEMP_REF1_C       25.0f      // Reference temperature 1 (°C)
#define TRANS_TEMP_REF1_OHMS    3500.0f    // Resistance at ref temp 1 (ohms)
#define TRANS_TEMP_REF2_C       110.0f     // Reference temperature 2 (°C)
#define TRANS_TEMP_REF2_OHMS    250.0f     // Resistance at ref temp 2 (ohms)
#define TRANS_TEMP_PULLUP_OHMS  2200       // 2.2K pullup resistor
#define TRANS_TEMP_TABLE_SIZE   25         // Number of lookup table points
#define TRANS_TEMP_MIN_C        -30.0f     // Minimum temperature range
#define TRANS_TEMP_MAX_C        140.0f     // Maximum temperature range


// =============================================================================
// TRANSMISSION TYPES
// =============================================================================

typedef enum {
    GEAR_UNKNOWN = 0,
    GEAR_PARK,
    GEAR_REVERSE,
    GEAR_NEUTRAL,
    GEAR_DRIVE,
    GEAR_SECOND,
    GEAR_FIRST
} gear_position_t;

typedef enum {
    SHIFT_NONE = 0,
    SHIFT_UP,
    SHIFT_DOWN
} shift_request_t;

typedef struct {
    gear_position_t current_gear;
    float fluid_temperature;
    shift_request_t shift_request;
    uint32_t last_paddle_time_ms;
    bool valid_gear_position;
    bool upshift_requested;
    bool downshift_requested;
    
    // Individual switch states for diagnostics
    bool park_switch;
    bool reverse_switch;
    bool neutral_switch;
    bool drive_switch;
    bool second_switch;
    bool first_switch;
} transmission_state_t;

// =============================================================================
// PUBLIC API
// =============================================================================

/**
 * Initialize the transmission module
 * - Generates thermistor lookup tables
 * - Registers sensors with input manager
 * - Subscribes to transmission messages
 * @return Number of sensors successfully registered
 */
uint8_t transmission_module_init(void);

/**
 * Update transmission logic (call from main loop)
 * - Processes shift requests
 * - Updates combined state messages
 * - Handles safety logic
 */
void transmission_module_update(void);

/**
 * Get current transmission state (read-only)
 * @return Pointer to current transmission state
 */
const transmission_state_t* transmission_get_state(void);

/**
 * Clear current shift request (call after processing)
 */
void transmission_clear_shift_request(void);

/**
 * Check if transmission fluid is overheating
 * @param threshold_c Temperature threshold in Celsius
 * @return true if fluid temperature exceeds threshold
 */
bool transmission_is_overheating(float threshold_c);

/**
 * Get gear position as string (for debugging)
 * @param gear Gear position enum
 * @return String representation of gear
 */
const char* transmission_gear_to_string(gear_position_t gear);

// =============================================================================
// CONFIGURATION FUNCTIONS
// =============================================================================

/**
 * Set paddle debounce time
 * @param debounce_ms New debounce time in milliseconds
 */
void transmission_set_paddle_debounce(uint16_t debounce_ms);

/**
 * Get current paddle debounce time
 * @return Current debounce time in milliseconds
 */
uint16_t transmission_get_paddle_debounce(void);

// =============================================================================
// DIAGNOSTICS AND STATUS
// =============================================================================

/**
 * Get number of valid shifts processed
 * @return Total shift count
 */
uint32_t transmission_get_shift_count(void);

/**
 * Get number of invalid gear position detections
 * @return Invalid gear position count
 */
uint32_t transmission_get_invalid_gear_count(void);

/**
 * Reset transmission statistics
 */
void transmission_reset_statistics(void);

#endif // TRANSMISSION_MODULE_H