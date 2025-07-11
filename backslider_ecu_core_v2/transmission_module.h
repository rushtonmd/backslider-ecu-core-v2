// transmission_module.h
// Transmission control module for manual paddle shifting with overrun clutch control
//
// This module handles:
// - Transmission fluid temperature monitoring
// - Paddle shifter inputs (upshift/downshift)
// - Gear position detection (P, R, N, D, 2, 1)
// - Overrun clutch control for race car applications
// - State management and safety logic
//
// 5-Solenoid Transmission Control System:
// - Shift Solenoid A (Pin 40): Digital ON/OFF
// - Shift Solenoid B (Pin 41): Digital ON/OFF  
// - Overrun Solenoid (Pin 42): Digital ON/OFF (Race car logic: aggressive engagement for control)
// - Line Pressure Solenoid (Pin 43): PWM 0-100% (0% Park/Neutral, 100% all moving gears)
// - Lockup Solenoid (Pin 44): Digital ON/OFF (automatic - ON in 4th gear only)

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

// Race car overrun clutch tuning parameters
#define OVERRUN_THROTTLE_DISENGAGE_THRESHOLD    75.0f   // Disengage above 75% throttle
#define OVERRUN_THROTTLE_ENGAGE_THRESHOLD       15.0f   // Engage below 15% throttle  
#define OVERRUN_MINIMUM_SPEED_MPH               15.0f   // No engagement below 15 mph
#define OVERRUN_BRAKING_SPEED_THRESHOLD         30.0f   // Always engage when braking above 30 mph
#define OVERRUN_MODERATE_THROTTLE_THRESHOLD     60.0f   // Keep engaged below 60% in lower gears
#define OVERRUN_MODERATE_THROTTLE_THRESHOLD     60.0f   // Keep engaged below 60% in lower gears

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

typedef enum {
    OVERRUN_ENGAGED,        // Solenoid OFF (0V), engine braking active, maximum control
    OVERRUN_DISENGAGED      // Solenoid ON (12V), freewheeling/fluid coupling, smooth power delivery
} overrun_clutch_state_t;

typedef struct {
    gear_position_t current_gear;
    float fluid_temperature;
    shift_request_t shift_request;
    uint32_t last_paddle_time_ms;
    bool valid_gear_position;
    bool upshift_requested;
    bool downshift_requested;
    overrun_clutch_state_t overrun_state;  // Current overrun clutch state
    
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
 * - Initializes overrun clutch control
 * @return Number of sensors successfully registered
 */
uint8_t transmission_module_init(void);

/**
 * Update transmission logic (call from main loop)
 * - Processes shift requests
 * - Updates overrun clutch control based on driving conditions
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

/**
 * Get overrun clutch state as string (for debugging)
 * @param state Overrun clutch state enum
 * @return String representation of overrun state
 */
const char* transmission_overrun_to_string(overrun_clutch_state_t state);

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
// OVERRUN CLUTCH CONTROL
// =============================================================================

/**
 * Manually override overrun clutch state (for testing/diagnostics)
 * WARNING: Manual override bypasses all safety logic and racing algorithms
 * @param state Desired overrun clutch state
 * @param override_enable true to enable manual override, false for automatic control
 */
void transmission_set_overrun_override(overrun_clutch_state_t state, bool override_enable);

/**
 * Check if overrun clutch is in manual override mode
 * @return true if manual override is active
 */
bool transmission_is_overrun_override_active(void);

/**
 * Set race car overrun tuning parameters
 * Allows fine-tuning of overrun behavior for different tracks and driving styles
 * @param throttle_disengage_pct Throttle % above which to disengage (default 75%)
 * @param throttle_engage_pct Throttle % below which to engage (default 15%)
 * @param min_speed_mph Minimum vehicle speed for engagement (default 15 mph)
 * @param braking_speed_mph Speed above which to force engagement when braking (default 30 mph)
 */
void transmission_set_overrun_tuning(float throttle_disengage_pct, float throttle_engage_pct, 
                                     float min_speed_mph, float braking_speed_mph);

/**
 * Get current overrun tuning parameters
 * @param throttle_disengage_pct Output: throttle disengage threshold
 * @param throttle_engage_pct Output: throttle engage threshold  
 * @param min_speed_mph Output: minimum speed for engagement
 * @param braking_speed_mph Output: braking speed threshold
 */
void transmission_get_overrun_tuning(float* throttle_disengage_pct, float* throttle_engage_pct,
                                     float* min_speed_mph, float* braking_speed_mph);

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
 * Get number of overrun clutch state changes
 * @return Total overrun state change count
 */
uint32_t transmission_get_overrun_change_count(void);

/**
 * Reset transmission statistics
 */
void transmission_reset_statistics(void);

// =============================================================================
// TRANSMISSION OUTPUT CONTROL
// =============================================================================

/**
 * Manually set lockup solenoid state (for testing/diagnostics)
 * Note: In normal operation, lockup is automatically controlled:
 * - OFF in gears 1, 2, 3 and Park/Reverse/Neutral
 * - ON in gear 4 only
 * @param engage true to engage lockup, false to disengage
 */
void transmission_set_lockup(bool engage);

/**
 * Manually set transmission line pressure (for testing/diagnostics)
 * Note: In normal operation, line pressure is automatically controlled:
 * - 0% (OFF) in Park and Neutral
 * - 100% (ON) in all moving gears (Reverse, Drive, manual gears)
 * @param pressure_percent Pressure as percentage (0.0-1.0)
 */
void transmission_set_line_pressure(float pressure_percent);

/**
 * Manually set all solenoids for specific gear (for testing/diagnostics)
 * Controls Shift Solenoid A, Shift Solenoid B, and Lockup Solenoid
 * @param gear Gear number (0=Park/Neutral, 1=Gear1, 2=Gear2, 3=Gear3, 4=Gear4)
 * Gear patterns: 1=A:ON,B:ON,L:OFF  2=A:OFF,B:ON,L:OFF  3=A:OFF,B:OFF,L:OFF  4=A:ON,B:OFF,L:ON
 */
void transmission_set_solenoid_pattern(uint8_t gear);

/**
 * Enable/disable automatic shifting (manual mode)
 * @param enable true for automatic, false for manual paddle control only
 */
void transmission_set_auto_shift(bool enable);

/**
 * Force all transmission outputs to safe state
 * Used for emergency shutdown or initialization
 * Sets overrun clutch to disengaged for safe operation
 */
void transmission_outputs_safe_state(void);

// =============================================================================
// EXTERNAL DATA INTERFACE (for other modules)
// =============================================================================

/**
 * These functions allow the transmission module to get driving condition data
 * from other ECU modules via the message bus. The transmission module subscribes
 * to the appropriate messages and caches the data with timeout protection.
 */

/**
 * Get current throttle position from engine module via MSG_THROTTLE_POSITION
 * Uses cached data from message bus with 1-second timeout protection
 * @return Throttle position as percentage (0.0-100.0), safe default if data stale
 */
float transmission_get_throttle_position_percent(void);

/**
 * Get current vehicle speed from engine/wheel speed sensors via message bus
 * Uses cached data with timeout protection  
 * @return Vehicle speed in MPH, safe default if data stale
 */
float transmission_get_vehicle_speed_mph(void);

/**
 * Get brake pedal status from input sensors via message bus
 * Uses cached data with timeout protection
 * @return true if brake pedal is pressed, false if released or data stale
 */
bool transmission_get_brake_pedal_active(void);

/**
 * Check if vehicle is decelerating based on throttle position
 * @return true if vehicle is decelerating (light throttle)
 */
bool transmission_is_decelerating(void);

#endif // TRANSMISSION_MODULE_H