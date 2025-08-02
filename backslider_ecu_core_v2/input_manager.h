// input_manager.h
// Main input manager interface
//
// This provides the public API for the input manager system.
// Modules use these functions to register sensors and get status.

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "input_manager_types.h"
#include "msg_definitions.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    // Mock functions for desktop testing (only if not already defined)
    #ifndef MOCK_ARDUINO_H
        inline uint16_t analogRead(int pin) { return 2048; }
        inline uint32_t micros() { return 0; }
        inline void pinMode(int pin, int mode) {}
        #define INPUT 0
        #define INPUT_PULLUP 1
    #endif
#endif

// =============================================================================
// PUBLIC API
// =============================================================================

// Initialize the input manager system
void input_manager_init(void);

// Register sensors from a module
// Returns number of sensors successfully registered
uint8_t input_manager_register_sensors(const sensor_definition_t* sensors, uint8_t count);

// Main update function - call from main loop
void input_manager_update(void);

// =============================================================================
// STATUS AND DIAGNOSTICS
// =============================================================================

// Get system status
uint8_t input_manager_get_sensor_count(void);
uint8_t input_manager_get_valid_sensor_count(void);
uint32_t input_manager_get_total_updates(void);
uint32_t input_manager_get_total_errors(void);

// Get individual sensor status (by index)
uint8_t input_manager_get_sensor_status(uint8_t sensor_index, sensor_runtime_t* status);

// Find sensor by message ID
int8_t input_manager_find_sensor_by_msg_id(uint32_t msg_id);

// =============================================================================
// HIGH-PERFORMANCE FREQUENCY COUNTER DIAGNOSTICS
// =============================================================================

// Get interrupt frequency counter performance statistics
void input_manager_get_interrupt_freq_stats(uint32_t* total_interrupts, uint32_t* max_isr_time_us, uint32_t* overflow_count);

// Get count of active interrupt frequency counters
uint8_t input_manager_get_interrupt_freq_counter_count(void);

// Get current frequency for a sensor (by message ID)
uint32_t input_manager_get_current_frequency(uint32_t msg_id);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Convert ADC counts to voltage (inline for performance)
static inline float adc_counts_to_voltage(uint16_t counts) {
    return (counts * ADC_VOLTAGE_REF) / ADC_RESOLUTION;
}

// Validate voltage reading
static inline uint8_t is_voltage_valid(float voltage) {
    return (voltage >= SENSOR_VOLTAGE_MIN && voltage <= SENSOR_VOLTAGE_MAX) ? 1 : 0;
}

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

// Helper macro to define a linear sensor
#define DEFINE_LINEAR_SENSOR(pin_name, msg_id_name, min_v, max_v, min_val, max_val, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_ANALOG_LINEAR, \
        .config.linear = { \
            .min_voltage = min_v, \
            .max_voltage = max_v, \
            .min_value = min_val, \
            .max_value = max_val, \
            .pullup_ohms = 0 \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 32, \
        .name = sensor_name \
    }

// Helper macro to define a thermistor sensor
#define DEFINE_THERMISTOR_SENSOR(pin_name, msg_id_name, pullup_value, v_table, t_table, size, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_THERMISTOR, \
        .config.thermistor = { \
            .pullup_ohms = pullup_value, \
            .voltage_table = v_table, \
            .temp_table = t_table, \
            .table_size = size \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 128, \
        .name = sensor_name \
    }

// =============================================================================
// HIGH-PERFORMANCE FREQUENCY COUNTER MACROS
// =============================================================================

// Interrupt edge definitions
#define FREQ_EDGE_RISING   0
#define FREQ_EDGE_FALLING  1  
#define FREQ_EDGE_CHANGE   2

// Helper macro to define a high-performance interrupt-based frequency sensor
#define DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, edge_type, max_rate, msg_rate, ppu, scale, timeout_val, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_FREQUENCY_COUNTER, \
        .config.frequency = { \
            .pulses_per_unit = ppu, \
            .scaling_factor = scale, \
            .timeout_us = timeout_val, \
            .message_update_rate_hz = msg_rate, \
            .use_interrupts = 1, \
            .trigger_edge = edge_type \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = 0, \
        .filter_strength = 16, \
        .name = sensor_name \
    }

// Helper macro to define a polling-based frequency sensor (legacy)
#define DEFINE_POLLING_FREQUENCY_SENSOR(pin_name, msg_id_name, ppu, scale, timeout_val, interval_us, sensor_name) \
    { \
        .pin = pin_name, \
        .type = SENSOR_FREQUENCY_COUNTER, \
        .config.frequency = { \
            .pulses_per_unit = ppu, \
            .scaling_factor = scale, \
            .timeout_us = timeout_val, \
            .message_update_rate_hz = 10, \
            .use_interrupts = 0, \
            .trigger_edge = FREQ_EDGE_RISING \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 32, \
        .name = sensor_name \
    }

// =============================================================================
// COMMON ECU FREQUENCY SENSOR CONFIGURATIONS
// =============================================================================
//
// PARAMETER EXPLANATIONS FOR ALL CONFIGURATIONS:
// ==============================================
//
// Each macro uses: DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin, msg_id, edge, max_rate, msg_rate, ppu, scale, timeout, name)
//
// pin_name:     Physical pin number for sensor input (user-provided)
// msg_id_name:  CAN message ID for publishing sensor data (user-provided)  
// edge:         Interrupt trigger (FREQ_EDGE_RISING/FALLING/CHANGE)
// max_rate:     Maximum expected frequency in Hz (for documentation)
// msg_rate:     Message publication rate in Hz (affects CAN bus load)
// ppu:          Pulses per unit (pulses per revolution, mile, etc.)
// scale:        Scaling factor applied to final result  
// timeout:      Timeout in microseconds (sensor disconnection detection)
// name:         Human-readable sensor name for debugging
//
// ⚠️  IMPORTANT - CAN MESSAGE UNITS:
// =================================
// The CAN message does NOT contain raw frequency (Hz). Instead, it contains
// the CALIBRATED result with application-specific units:
//
// 📡 CAN Message Content: calibrated_value (float)
// 🔄 Calibration Formula: ((frequency_hz * 60) / pulses_per_unit) * scaling_factor
//
// OUTPUT UNITS BY SENSOR TYPE:
// - Engine RPM sensor     → RPM (revolutions per minute)
// - Transmission sensors  → RPM (revolutions per minute)
// - Vehicle speed sensor  → MPH or KPH (depending on scaling_factor)
// - Wheel speed sensors   → RPM or speed units (depending on scaling_factor)
//
// TIME ASPECT:
// - Raw measurement: Hz (pulses per second) - contains time inherently
// - Published result: Application units (RPM, MPH, etc.) - time converted to rate
// - Message timing: Separate from sensor value (controlled by msg_rate parameter)

// =============================================================================
// ENGINE RPM SENSOR CONFIGURATION
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Engine crankshaft position sensor
// - Typically uses a 60-2 tooth wheel (58 actual teeth, 2 missing for sync)
// - Mounted on crankshaft, reads engine rotational speed
// - Critical for ignition timing, fuel injection, and engine management
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified (typically interrupt-capable pin like 2, 3)
// msg_id_name:  User-specified (typically MSG_ENGINE_RPM)
// FREQ_EDGE_RISING:  Trigger on rising edge only (clean, consistent timing)
// 7000:         Max frequency 7kHz (redline RPM: 7000 RPM = 7000/60 = 116 RPS × 58 teeth = 6733 Hz)
// 10:           10Hz message rate (every 100ms - fast enough for engine control)
// 58:           58 pulses per revolution (60-2 tooth wheel standard)
// 1.0f:         No additional scaling (frequency directly represents pulses/sec)
// 500000:       500ms timeout (engine stall detection - longer than normal cranking)
// "Engine RPM": Descriptive name for diagnostics
//
// CALIBRATION RESULT: Raw Hz → Engine RPM (published in CAN message)
// Formula: RPM = (frequency_hz * 60) / 58 * 1.0f
// Example: 2900 Hz → (2900 * 60) / 58 * 1.0f = 3000.0 RPM
// 📡 CAN Message Value: 3000.0 (float, units = RPM)

#define ENGINE_RPM_SENSOR(pin_name, msg_id_name) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 7000, 10, 58, 1.0f, 500000, "Engine RPM")

// =============================================================================
// TRANSMISSION INPUT SPEED SENSOR CONFIGURATION  
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Transmission input shaft speed sensor
// - Monitors speed of transmission input shaft (after torque converter)
// - Used for gear ratio calculation, shift timing, and slip detection
// - Typically magnetic pickup reading transmission input shaft teeth
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified transmission input speed pin
// msg_id_name:  User-specified (typically MSG_TRANS_INPUT_SPEED)
// FREQ_EDGE_RISING:  Rising edge trigger (consistent with engine RPM)
// 5500:         Max frequency 5.5kHz (8000 RPM max: 8000/60 = 133 RPS × 40 teeth = 5333 Hz)
// 5:            5Hz message rate (every 200ms - adequate for transmission control)
// 40:           40 pulses per revolution (common transmission gear tooth count)
// 1.0f:         No additional scaling
// 200000:       200ms timeout (quick detection of shaft stoppage during shifts)
// "Trans Input": Descriptive name
//
// CALIBRATION RESULT: Raw Hz → Transmission Input RPM (published in CAN message)
// Formula: RPM = (frequency_hz * 60) / 40 * 1.0f
// Example: 2000 Hz → (2000 * 60) / 40 * 1.0f = 3000.0 RPM
// 📡 CAN Message Value: 3000.0 (float, units = RPM)

#define TRANS_INPUT_SPEED_SENSOR(pin_name, msg_id_name) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 5500, 5, 40, 1.0f, 200000, "Trans Input")

// =============================================================================
// TRANSMISSION OUTPUT SPEED SENSOR CONFIGURATION
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Transmission output shaft speed sensor
// - Monitors speed of transmission output shaft (to differential/wheels)
// - Used for vehicle speed calculation, gear ratio determination, ABS systems
// - Critical for transmission shift points and electronic stability control
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified transmission output speed pin
// msg_id_name:  User-specified (typically MSG_TRANS_OUTPUT_SPEED)
// FREQ_EDGE_RISING:  Rising edge trigger
// 5500:         Max frequency 5.5kHz (same as input - can spin at engine speed in 1st gear)
// 5:            5Hz message rate (every 200ms - matches input shaft timing)
// 40:           40 pulses per revolution (same sensor type as input shaft)
// 1.0f:         No additional scaling
// 200000:       200ms timeout (detect vehicle stopping quickly)
// "Trans Output": Descriptive name
//
// CALIBRATION RESULT: Raw Hz → Transmission Output RPM (published in CAN message)
// Formula: RPM = (frequency_hz * 60) / 40 * 1.0f
// Example: 800 Hz → (800 * 60) / 40 * 1.0f = 1200.0 RPM
// 📡 CAN Message Value: 1200.0 (float, units = RPM)

#define TRANS_OUTPUT_SPEED_SENSOR(pin_name, msg_id_name) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 5500, 5, 40, 1.0f, 200000, "Trans Output")

// =============================================================================
// VEHICLE SPEED SENSOR CONFIGURATION
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Vehicle speed sensor (VSS)
// - Monitors overall vehicle ground speed (usually from transmission output or differential)
// - Used for speedometer, cruise control, ABS, traction control
// - Fewer pulses per revolution than shaft sensors (designed for speed, not precision timing)
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified vehicle speed sensor pin
// msg_id_name:  User-specified (typically MSG_VEHICLE_SPEED)  
// FREQ_EDGE_RISING:  Rising edge trigger
// 500:          Max frequency 500Hz (200 MPH: assuming 4 pulses/rev, ~125 Hz per 100 MPH)
// 2:            2Hz message rate (every 500ms - speed doesn't change rapidly)
// 4:            4 pulses per revolution (common VSS tooth count - lower resolution)
// 0.01f:        0.01 scaling factor (converts to meaningful speed units like MPH/KPH)
// 2000000:      2 second timeout (vehicle stopped detection - longer than trans sensors)
// "Vehicle Speed": Descriptive name
//
// CALIBRATION RESULT: Raw Hz → Vehicle Speed (published in CAN message)
// Formula: Speed = (frequency_hz * 60) / 4 * 0.01f
// Example: 100 Hz → (100 * 60) / 4 * 0.01f = 1.5 speed units
// 📡 CAN Message Value: 1.5 (float, units = speed units - MPH/KPH based on scaling)
// Note: 0.01f scaling factor needs adjustment for real vehicle calibration

#define VEHICLE_SPEED_SENSOR(pin_name, msg_id_name) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 500, 2, 4, 0.01f, 2000000, "Vehicle Speed")

// =============================================================================
// WHEEL SPEED SENSOR CONFIGURATION (ABS)
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Individual wheel speed sensor for ABS/ESP systems
// - Monitors rotational speed of individual wheels
// - Used for anti-lock braking, traction control, electronic stability programs
// - High pulse count for precise wheel slip detection
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified wheel speed sensor pin (typically one per wheel)
// msg_id_name:  User-specified (typically MSG_WHEEL_SPEED_FL, FR, RL, RR)
// FREQ_EDGE_RISING:  Rising edge trigger
// 2000:         Max frequency 2kHz (high-speed driving with small diameter wheels)
// 2:            2Hz message rate (every 500ms - adequate for ABS systems)
// 48:           48 pulses per revolution (high resolution for precise slip detection)
// 0.05f:        0.05 scaling factor (converts to wheel RPM or surface speed)
// 1000000:      1 second timeout (wheel lock detection for ABS intervention)
// "Wheel Speed": Descriptive name
//
// CALIBRATION RESULT: Raw Hz → Wheel Speed (published in CAN message)
// Formula: Wheel_Units = (frequency_hz * 60) / 48 * 0.05f
// Example: 800 Hz → (800 * 60) / 48 * 0.05f = 50.0 speed units
// 📡 CAN Message Value: 50.0 (float, units = speed units - wheel RPM or surface speed)
// Note: 0.05f scaling factor converts to desired units (RPM, MPH, etc.)

#define WHEEL_SPEED_SENSOR(pin_name, msg_id_name) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 2000, 2, 48, 0.05f, 1000000, "Wheel Speed")

// =============================================================================
// GENERIC HIGH-FREQUENCY SENSOR CONFIGURATION
// =============================================================================
//
// AUTOMOTIVE CONTEXT: Configurable sensor for custom applications
// - Turbine speed sensors (turbocharger, supercharger)
// - Cooling fan speed sensors
// - Fuel pump speed monitoring
// - Custom rotation monitoring applications
//
// PARAMETER BREAKDOWN:
// pin_name:     User-specified pin
// msg_id_name:  User-specified message ID
// FREQ_EDGE_RISING:  Rising edge trigger (can be changed if needed)
// 10000:        Max frequency 10kHz (very high-speed applications)
// msg_rate:     User-configurable message rate (1Hz to 100Hz)
// ppu:          User-configurable pulses per unit (depends on sensor)
// scale:        User-configurable scaling factor
// timeout_val:  User-configurable timeout (depends on application)
// "Generic Freq": Generic name (user should provide specific name)
//
// USAGE EXAMPLES:
// - Turbo speed: GENERIC_FREQ_SENSOR(pin5, MSG_TURBO_SPEED, 120, 0.1f, 20, 100000)
// - Fan speed:   GENERIC_FREQ_SENSOR(pin7, MSG_FAN_SPEED, 8, 1.0f, 1, 5000000)
//
// ⭐ REAL-WORLD SPEED SENSOR EXAMPLE:
// ==================================
// Speed sensor: 2000 pulses per mile, want output in MPH
//
// SETUP: GENERIC_FREQ_SENSOR(pin4, MSG_VEHICLE_SPEED, 2000, 60.0f, 2, 2000000)
//
// CALCULATION WALKTHROUGH:
// -----------------------
// Scenario: Vehicle traveling at 60 MPH
// 
// Step 1: Physical measurement
// - 60 miles/hour = 1 mile/minute = 1/60 mile/second
// - At 2000 pulses/mile: (1/60) × 2000 = 33.33 pulses/second = 33.33 Hz
//
// Step 2: Calibration formula
// calibrated_value = ((frequency_hz * 60) / ppu) * scaling_factor
//                  = ((33.33 * 60) / 2000) * 60.0f
//                  = (2000 / 2000) * 60.0f  
//                  = 1.0 * 60.0f = 60.0
//
// Step 3: CAN message result
// 📡 CAN Message Value: 60.0 (float, units = MPH)
//
// 🔍 WHY scaling_factor = 60 WORKS:
// The base formula assumes "per minute" units (hence the *60).
// For speed in "per hour" units, we need *3600 (seconds/hour).
// Since formula already has *60, we use scaling_factor = 60 to get: *60*60 = *3600
//
// ✅ RESULT: Direct MPH output from pulses-per-mile sensor!
//
// 🌍 OTHER SPEED SENSOR EXAMPLES:
// ===============================
//
// Metric speed sensor (pulses per kilometer → KPH):
// - 1000 pulses/km, want KPH: ppu=1000, scaling_factor=60.0f
// - Result: Direct KPH output
//
// High-resolution speed sensor (pulses per foot → MPH):
// - 500 pulses/foot, want MPH: ppu=500, scaling_factor=60*5280=316800.0f
// - Calculation: 316800 = 60 × 5280 feet/mile
// - Result: Direct MPH output
//
// Distance sensor with different time base (pulses per meter → m/s):
// - 100 pulses/meter, want m/s: ppu=100, scaling_factor=1/60≈0.0167f  
// - Calculation: 0.0167 = 1/60 (convert "per minute" back to "per second")
// - Result: Direct m/s output
//
// 💡 GENERAL RULE:
// For distance-based sensors: scaling_factor = (target_time_unit / 60) × (distance_conversion)
// - MPH from pulses/mile: 60 × 1 = 60
// - KPH from pulses/km:   60 × 1 = 60  
// - MPH from pulses/ft:   60 × 5280 = 316800
// - m/s from pulses/m:    (1/60) × 1 = 0.0167

#define GENERIC_FREQ_SENSOR(pin_name, msg_id_name, ppu, scale, msg_rate, timeout_val) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 10000, msg_rate, ppu, scale, timeout_val, "Generic Freq")

// =============================================================================
// CONVENIENCE MACROS FOR DISTANCE-BASED SPEED SENSORS
// =============================================================================

// Speed sensor with pulses per mile → MPH output
#define SPEED_SENSOR_PULSES_PER_MILE(pin_name, msg_id_name, ppu_val) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 1000, 2, ppu_val, 60.0f, 2000000, "Speed MPH")

// Speed sensor with pulses per kilometer → KPH output  
#define SPEED_SENSOR_PULSES_PER_KM(pin_name, msg_id_name, ppu_val) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 1000, 2, ppu_val, 60.0f, 2000000, "Speed KPH")

// Speed sensor with pulses per foot → MPH output (high resolution)
#define SPEED_SENSOR_PULSES_PER_FOOT(pin_name, msg_id_name, ppu_val) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 5000, 2, ppu_val, 316800.0f, 1000000, "Speed MPH")

// Speed sensor with pulses per meter → m/s output
#define SPEED_SENSOR_PULSES_PER_METER(pin_name, msg_id_name, ppu_val) \
    DEFINE_INTERRUPT_FREQUENCY_SENSOR(pin_name, msg_id_name, FREQ_EDGE_RISING, 2000, 5, ppu_val, 0.0167f, 1000000, "Speed m/s")

// USAGE EXAMPLES:
// SPEED_SENSOR_PULSES_PER_MILE(pin3, MSG_VEHICLE_SPEED, 2000)   // 2000 pulses/mile → MPH
// SPEED_SENSOR_PULSES_PER_KM(pin4, MSG_VEHICLE_SPEED, 1000)     // 1000 pulses/km → KPH  
// SPEED_SENSOR_PULSES_PER_FOOT(pin5, MSG_VEHICLE_SPEED, 500)    // 500 pulses/foot → MPH
// SPEED_SENSOR_PULSES_PER_METER(pin6, MSG_VEHICLE_SPEED, 100)   // 100 pulses/meter → m/s

// =============================================================================
// I2C DEVICE HELPER FUNCTIONS
// =============================================================================

// Function to read from ADS1015 ADC
int16_t read_ads1015_channel(uint8_t channel);

// Function to read from MCP23017 GPIO expander
bool read_mcp23017_pin(uint8_t pin);

// Function to write to MCP23017 GPIO expander
void write_mcp23017_pin(uint8_t pin, bool value);

// Function to configure MCP23017 pin mode
void configure_mcp23017_pin(uint8_t pin, uint8_t mode);

// Function to print I2C device status
void print_i2c_status();

// =============================================================================
// I2C SENSOR CONFIGURATION MACROS
// =============================================================================

// Helper macro to define an I2C ADC sensor (ADS1015)
#define DEFINE_I2C_ADC_SENSOR(channel_num, msg_id_name, min_v, max_v, min_val, max_val, gain_setting, interval_us, sensor_name) \
    { \
        .pin = 0xFF, /* Not used for I2C sensors */ \
        .type = SENSOR_I2C_ADC, \
        .config.i2c_adc = { \
            .channel = channel_num, \
            .min_voltage = min_v, \
            .max_voltage = max_v, \
            .min_value = min_val, \
            .max_value = max_val, \
            .gain_setting = gain_setting \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 32, \
        .name = sensor_name \
    }

// Helper macro to define an I2C GPIO sensor (MCP23017)
#define DEFINE_I2C_GPIO_SENSOR(pin_num, msg_id_name, use_pullup, invert_logic, interval_us, sensor_name) \
    { \
        .pin = 0xFF, /* Not used for I2C sensors */ \
        .type = SENSOR_I2C_GPIO, \
        .config.i2c_gpio = { \
            .pin = pin_num, \
            .use_pullup = use_pullup, \
            .invert_logic = invert_logic \
        }, \
        .msg_id = msg_id_name, \
        .update_interval_us = interval_us, \
        .filter_strength = 16, \
        .name = sensor_name \
    }

// =============================================================================
// COMMON I2C SENSOR CONFIGURATIONS
// =============================================================================

// I2C ADC sensor for pressure sensors (0-5V range)
#define I2C_PRESSURE_SENSOR(channel_num, msg_id_name) \
    DEFINE_I2C_ADC_SENSOR(channel_num, msg_id_name, 0.0f, 5.0f, 0.0f, 100.0f, 0, 100000, "I2C Pressure")

// I2C ADC sensor for temperature sensors (0-5V range)
#define I2C_TEMPERATURE_SENSOR(channel_num, msg_id_name) \
    DEFINE_I2C_ADC_SENSOR(channel_num, msg_id_name, 0.0f, 5.0f, -40.0f, 150.0f, 0, 100000, "I2C Temperature")

// I2C ADC sensor for throttle position (0-5V range)
#define I2C_THROTTLE_SENSOR(channel_num, msg_id_name) \
    DEFINE_I2C_ADC_SENSOR(channel_num, msg_id_name, 0.0f, 5.0f, 0.0f, 100.0f, 0, 50000, "I2C Throttle")

// I2C GPIO sensor for digital switches
#define I2C_DIGITAL_SENSOR(pin_num, msg_id_name) \
    DEFINE_I2C_GPIO_SENSOR(pin_num, msg_id_name, 1, 0, 100000, "I2C Digital")

// I2C GPIO sensor for digital switches (inverted logic)
#define I2C_DIGITAL_SENSOR_INVERTED(pin_num, msg_id_name) \
    DEFINE_I2C_GPIO_SENSOR(pin_num, msg_id_name, 1, 1, 100000, "I2C Digital Inverted")

#endif