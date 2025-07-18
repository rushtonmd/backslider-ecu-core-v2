// input_manager_types.h
// Core data structures for the input manager system
//
// This file contains only the type definitions and structures.
// Keep it small and focused on data layout.

#ifndef INPUT_MANAGER_TYPES_H
#define INPUT_MANAGER_TYPES_H

#include <stdint.h>

// =============================================================================
// SENSOR TYPES
// =============================================================================

typedef enum {
    SENSOR_ANALOG_LINEAR = 0,    // Linear voltage-to-value mapping
    SENSOR_THERMISTOR,           // Non-linear thermistor with lookup table
    SENSOR_DIGITAL_PULLUP,       // Digital input with pullup
    SENSOR_FREQUENCY_COUNTER,    // Frequency-based sensor (speed, RPM)
    SENSOR_I2C_ADC,              // I2C ADC (ADS1015) input
    SENSOR_I2C_GPIO,             // I2C GPIO (MCP23017) input
    SENSOR_TYPE_COUNT            // Keep this last
} sensor_type_t;

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================

// Configuration for linear analog sensors (TPS, MAP, pressures, etc.)
typedef struct {
    float min_voltage;      // Voltage at minimum value
    float max_voltage;      // Voltage at maximum value  
    float min_value;        // Minimum output value
    float max_value;        // Maximum output value
    uint16_t pullup_ohms;   // Pullup resistor value (0 = no pullup)
} linear_config_t;

// Configuration for thermistor sensors (CTS, IAT, etc.)
typedef struct {
    uint16_t pullup_ohms;        // Pullup resistor value
    const float* voltage_table;  // Voltage points for lookup
    const float* temp_table;     // Corresponding temperature points
    uint8_t table_size;          // Number of points in tables
} thermistor_config_t;

// Configuration for digital sensors
typedef struct {
    uint8_t use_pullup;     // 1 = enable internal pullup
    uint8_t invert_logic;   // 1 = invert reading (active low)
} digital_config_t;

// Configuration for frequency-based sensors
typedef struct {
    uint16_t pulses_per_unit;   // Pulses per revolution/unit
    float scaling_factor;       // Additional scaling factor
    uint32_t timeout_us;        // Timeout for zero reading
    uint32_t message_update_rate_hz; // Rate to publish messages (much slower than interrupts)
    uint8_t use_interrupts;     // 1 = use high-speed interrupts, 0 = use polling
    uint8_t trigger_edge;       // 0=RISING, 1=FALLING, 2=CHANGE (interrupt mode only)
} frequency_config_t;

// Configuration for I2C ADC sensors (ADS1015)
typedef struct {
    uint8_t channel;            // ADC channel (0-3)
    float min_voltage;          // Voltage at minimum value
    float max_voltage;          // Voltage at maximum value
    float min_value;            // Minimum output value
    float max_value;            // Maximum output value
    uint8_t gain_setting;       // ADC gain setting (0-5, see ADS1015 datasheet)
} i2c_adc_config_t;

// Configuration for I2C GPIO sensors (MCP23017)
typedef struct {
    uint8_t pin;                // GPIO pin number (0-15)
    uint8_t use_pullup;         // 1 = enable internal pullup
    uint8_t invert_logic;       // 1 = invert reading (active low)
} i2c_gpio_config_t;

// Union for sensor-specific configuration
typedef union {
    linear_config_t linear;
    thermistor_config_t thermistor;
    digital_config_t digital;
    frequency_config_t frequency;
    i2c_adc_config_t i2c_adc;
    i2c_gpio_config_t i2c_gpio;
} sensor_config_u;

// =============================================================================
// SENSOR DEFINITION
// =============================================================================

typedef struct {
    // Hardware configuration
    uint8_t pin;                    // Arduino/Teensy pin number
    sensor_type_t type;             // Sensor type
    sensor_config_u config;         // Type-specific configuration
    
    // Message bus integration
    uint32_t msg_id;                // Message ID for publishing results
    
    // Timing configuration  
    uint32_t update_interval_us;    // Update interval in microseconds
    
    // Filtering
    uint8_t filter_strength;        // 0-255, higher = more filtering
    
    // Metadata
    const char* name;               // Human-readable name for debugging
} sensor_definition_t;

// =============================================================================
// SENSOR RUNTIME DATA
// =============================================================================

typedef struct {
    // Current readings
    float calibrated_value;         // Final calibrated value
    float raw_voltage;              // Raw voltage reading
    uint16_t raw_counts;            // Raw ADC counts
    
    // Timing
    uint32_t last_update_us;        // Last update timestamp (micros)
    uint32_t update_count;          // Total number of updates
    
    // Status
    uint8_t is_valid;               // 1 = valid reading, 0 = invalid
    uint8_t error_count;            // Consecutive error count
    
    // Filtering state
    uint8_t first_reading;          // 1 = first reading (no filtering yet)
} sensor_runtime_t;

// =============================================================================
// INPUT MANAGER CONFIGURATION
// =============================================================================

#define MAX_SENSORS 32              // Maximum number of sensors
#define ADC_RESOLUTION 4095.0f      // 12-bit ADC
#define ADC_VOLTAGE_REF 3.3f        // 3.3V reference on Teensy 4.1

// Error thresholds
#define SENSOR_VOLTAGE_MIN 0.1f     // Below this = likely short circuit
#define SENSOR_VOLTAGE_MAX 4.9f     // Above this = likely open circuit
#define MAX_CONSECUTIVE_ERRORS 5    // After this many errors, mark sensor as failed

#endif