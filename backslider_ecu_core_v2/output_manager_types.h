// output_manager_types.h
// Type definitions for ECU output management system

#ifndef OUTPUT_MANAGER_TYPES_H
#define OUTPUT_MANAGER_TYPES_H

#include <stdint.h>

// =============================================================================
// OUTPUT TYPE DEFINITIONS
// =============================================================================

typedef enum {
    OUTPUT_PWM = 0,           // PWM output (0.0-1.0 duty cycle)
    OUTPUT_DIGITAL,           // Digital output (0=low, 1=high)
    OUTPUT_ANALOG,            // Analog output (0.0-5.0V or similar)
    OUTPUT_SPI,               // SPI-controlled output (shift registers, etc.)
    OUTPUT_VIRTUAL,           // Virtual output (logging, CAN, internal logic)
    OUTPUT_TYPE_COUNT         // Number of output types
} output_type_t;

// =============================================================================
// OUTPUT CONFIGURATION STRUCTURES
// =============================================================================

// PWM output configuration
typedef struct {
    uint16_t frequency_hz;    // PWM frequency (e.g., 1000 Hz for solenoids)
    uint8_t resolution_bits;  // PWM resolution (8, 10, 12, or 16 bits)
    float min_duty_cycle;     // Minimum allowed duty cycle (0.0-1.0)
    float max_duty_cycle;     // Maximum allowed duty cycle (0.0-1.0)
    float default_duty_cycle; // Default/safe duty cycle
    uint8_t invert_output;    // 1=invert PWM signal, 0=normal
} pwm_config_t;

// Digital output configuration
typedef struct {
    uint8_t active_high;      // 1=active high, 0=active low
    uint8_t default_state;    // Default/safe state (0 or 1)
    uint8_t open_drain;       // 1=open drain, 0=push-pull
} digital_output_config_t;

// Analog output configuration (for DACs or PWM-filtered outputs)
typedef struct {
    float min_voltage;        // Minimum output voltage
    float max_voltage;        // Maximum output voltage
    float default_voltage;    // Default/safe voltage
    uint8_t resolution_bits;  // DAC resolution (8, 10, 12, or 16 bits)
    uint8_t use_pwm_filter;   // 1=use PWM+filter, 0=true DAC
} analog_config_t;

// SPI output configuration (for shift registers, SPI relay boards, etc.)
typedef struct {
    uint8_t spi_device_id;    // Which SPI device (0-3)
    uint8_t bit_position;     // Bit position within SPI device (0-15, 0-31, etc.)
    uint8_t active_high;      // 1=active high, 0=active low
    uint8_t default_state;    // Default/safe state (0 or 1)
    uint32_t spi_speed_hz;    // SPI clock speed
} spi_config_t;

// Virtual output configuration (for logging, CAN transmission, internal logic)
typedef struct {
    float min_value;          // Minimum value for range checking
    float max_value;          // Maximum value for range checking
    float default_value;      // Default/safe value
    uint8_t log_to_serial;    // 1=log value changes to serial, 0=silent
    uint8_t send_to_can;      // 1=send value on CAN bus, 0=internal only
} virtual_config_t;

// Union for output type-specific configuration
typedef union {
    pwm_config_t pwm;
            digital_output_config_t digital;
    analog_config_t analog;
    spi_config_t spi;
    virtual_config_t virtual_out;
} output_config_t;

// =============================================================================
// OUTPUT DEFINITION STRUCTURE
// =============================================================================

typedef struct {
    uint8_t pin;                    // Hardware pin number
    output_type_t type;             // Output type (PWM, digital, analog)
    output_config_t config;         // Type-specific configuration
    uint32_t msg_id;                // Message ID to subscribe to for control
    float current_value;            // Current output value
    uint32_t last_update_time_ms;   // Last time output was updated
    uint16_t update_rate_limit_ms;  // Minimum time between updates (safety)
    uint8_t fault_detected;         // 1=fault detected, 0=normal
    const char* name;               // Human-readable description
} output_definition_t;

// =============================================================================
// OUTPUT STATUS AND STATISTICS
// =============================================================================

typedef struct {
    uint8_t total_outputs;          // Total number of configured outputs
    uint8_t pwm_outputs;            // Number of PWM outputs
    uint8_t digital_outputs;        // Number of digital outputs
    uint8_t analog_outputs;         // Number of analog outputs
    uint8_t spi_outputs;            // Number of SPI outputs
    uint8_t virtual_outputs;        // Number of virtual outputs
    uint32_t total_updates;         // Total output updates performed
    uint32_t rate_limited_updates;  // Updates blocked by rate limiting
    uint32_t range_violations;      // Values clamped due to range limits
    uint32_t fault_count;           // Total faults detected
    uint32_t last_update_time_ms;   // Time of last output update
} output_manager_stats_t;

// =============================================================================
// SAFETY AND FAULT DETECTION
// =============================================================================

typedef enum {
    OUTPUT_FAULT_NONE = 0,
    OUTPUT_FAULT_OVERCURRENT,
    OUTPUT_FAULT_SHORT_TO_GROUND,
    OUTPUT_FAULT_SHORT_TO_POWER,
    OUTPUT_FAULT_OPEN_CIRCUIT,
    OUTPUT_FAULT_OVERTEMPERATURE,
    OUTPUT_FAULT_RANGE_VIOLATION,
    OUTPUT_FAULT_RATE_LIMIT_EXCEEDED,
    OUTPUT_FAULT_COUNT
} output_fault_t;

// =============================================================================
// INTERRUPT-DRIVEN IGNITION CONTROL
// =============================================================================

// Ignition timing configuration (set via message bus)
typedef struct {
    float base_timing_advance;      // Base timing advance in degrees BTDC
    float timing_per_rpm;           // Additional timing per 1000 RPM
    float timing_per_load;          // Additional timing per 10% load
    uint32_t coil_charge_time_us;   // Coil charge time in microseconds
    uint32_t max_dwell_time_us;     // Maximum dwell time (safety)
    uint8_t cylinder_count;         // Number of cylinders
    uint8_t firing_order[8];        // Firing order (cylinder indices)
} ignition_config_t;

// Interrupt-driven ignition state
typedef struct {
    volatile uint8_t next_cylinder;     // Next cylinder to fire
    volatile uint32_t last_trigger_us;  // Last crank trigger time
    volatile uint32_t trigger_interval_us; // Time between triggers
    volatile uint8_t ignition_enabled;  // Master ignition enable
    uint8_t coil_pins[8];              // Ignition coil pin assignments
    uint8_t crank_trigger_pin;         // Crank angle sensor pin
} ignition_state_t;

// =============================================================================
// ORIGINAL FAULT STRUCTURES
// =============================================================================

typedef struct {
    output_fault_t fault_type;
    uint8_t output_index;           // Which output has the fault
    uint32_t fault_time_ms;         // When the fault occurred
    float fault_value;              // Value that caused the fault
    const char* description;        // Human-readable fault description
} output_fault_record_t;



#endif // OUTPUT_MANAGER_TYPES_H 