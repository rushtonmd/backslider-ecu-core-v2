// transmission_module.cpp
// Implementation of transmission control module with race car overrun clutch control
// Uses pure message bus architecture for all data exchange
//
// 5-Solenoid Transmission Control System:
// - Shift Solenoid A (Pin 40): Digital ON/OFF
// - Shift Solenoid B (Pin 41): Digital ON/OFF
// - Overrun Solenoid (Pin 42): Digital ON/OFF (Race car logic implemented)
// - Line Pressure Solenoid (Pin 43): PWM 0-100% (0% Park/Neutral, 100% all moving gears)
// - Lockup Solenoid (Pin 44): Digital ON/OFF (automatic - ON in 4th gear only)
//
// Gear Patterns (A/B/Lockup/Pressure):
// Park/Neutral: OFF/OFF/OFF/0%
// Reverse: OFF/OFF/OFF/100%
// Gear 1: ON/ON/OFF/100%
// Gear 2: OFF/ON/OFF/100%
// Gear 3: OFF/OFF/OFF/100%
// Gear 4: ON/OFF/ON/100%  (Lockup engages for fuel efficiency)

#ifndef ARDUINO
// For desktop testing, include mock Arduino before anything else
#include "../tests/mock_arduino.h"
#endif

#include "transmission_module.h"
#include "input_manager.h"
#include "msg_bus.h"
#include "parameter_helpers.h"
#include "thermistor_table_generator.h"

// Forward declarations to avoid header conflicts
typedef struct output_definition_t output_definition_t;
extern uint8_t output_manager_register_outputs(output_definition_t* outputs, uint8_t count);

#ifdef ARDUINO
#include <Arduino.h>
#endif

// =============================================================================
// TRANSMISSION HARDWARE DEFINITIONS
// =============================================================================

// Sensor timing constants
#define TRANS_TEMP_UPDATE_INTERVAL_US    100000  // 100ms for thermistor (slow, filtered)
#define PADDLE_UPDATE_INTERVAL_US        20000   // 20ms for paddle shifters (fast response)
#define GEAR_SWITCH_UPDATE_INTERVAL_US   50000   // 50ms for gear switches (moderate)

// Filter strength constants
#define TRANS_TEMP_FILTER_STRENGTH       128     // Heavy filtering for temperature
#define PADDLE_FILTER_STRENGTH           0       // No filtering for paddle shifters
#define GEAR_SWITCH_FILTER_STRENGTH      0       // No filtering for gear switches

// PWM frequency constants
#define TRANS_PRESSURE_PWM_FREQ     1000    // 1kHz for line pressure solenoid
#define TRANS_SOLENOID_PWM_FREQ     100     // 100Hz for digital solenoids

// Timing constants
#define TRANS_OUTPUT_UPDATE_RATE_MS 10      // 10ms update rate for all outputs

// Default duty cycle constants
#define TRANS_PRESSURE_DEFAULT_DUTY 0.0f    // 0% pressure (safe for Park/Neutral)
#define TRANS_SOLENOID_DEFAULT_DUTY 0.0f    // OFF (safe for all digital solenoids)

// Number of sensors and outputs
#define TRANSMISSION_SENSOR_COUNT 9   // 1 thermistor + 2 paddles + 6 gear switches
#define TRANSMISSION_OUTPUT_COUNT 5   // 5 solenoids (A, B, Overrun, Pressure, Lockup)

// =============================================================================
// PRIVATE DATA
// =============================================================================

// Static lookup tables for transmission fluid temperature sensor
static float trans_temp_voltage_table[TRANS_TEMP_TABLE_SIZE];
static float trans_temp_temp_table[TRANS_TEMP_TABLE_SIZE];

// Transmission state
static transmission_state_t trans_state;

// Current automatic gear when in Drive position (1-4)
static uint8_t current_auto_gear = 1;  // Start in gear 1

// Configuration
static uint16_t paddle_debounce_ms = PADDLE_DEBOUNCE_MS;

// Race car overrun clutch tuning parameters (adjustable for different tracks/drivers)
static float overrun_throttle_disengage_threshold = OVERRUN_THROTTLE_DISENGAGE_THRESHOLD;
static float overrun_throttle_engage_threshold = OVERRUN_THROTTLE_ENGAGE_THRESHOLD;
static float overrun_minimum_speed_mph = OVERRUN_MINIMUM_SPEED_MPH;
static float overrun_braking_speed_threshold = OVERRUN_BRAKING_SPEED_THRESHOLD;
static float overrun_moderate_throttle_threshold = OVERRUN_MODERATE_THROTTLE_THRESHOLD;

// External data caching for overrun control (from message bus)
#define EXTERNAL_DATA_TIMEOUT_MS 500  // 500ms timeout for external data validity
static float cached_throttle_position = 20.0f;  // Safe default
static float cached_vehicle_speed = 35.0f;      // Safe default
static bool cached_brake_active = false;        // Safe default
static uint32_t last_throttle_update_ms = 0;
static uint32_t last_speed_update_ms = 0;
static uint32_t last_brake_update_ms = 0;

// Overrun clutch override control (for testing/diagnostics)
static bool overrun_manual_override_active = false;
static overrun_clutch_state_t overrun_manual_override_state = OVERRUN_DISENGAGED;

// Statistics
static uint32_t shift_count = 0;
static uint32_t invalid_gear_count = 0;
static uint32_t overrun_change_count = 0;

// =============================================================================
// HARDWARE ARRAYS
// =============================================================================

// Transmission hardware definition arrays (initialized in init functions below)
static sensor_definition_t TRANSMISSION_SENSORS[TRANSMISSION_SENSOR_COUNT];
// TODO: Re-enable when output manager header conflicts are resolved
// static output_definition_t TRANSMISSION_OUTPUTS[TRANSMISSION_OUTPUT_COUNT];

// =============================================================================
// PRIVATE FUNCTION DECLARATIONS
// =============================================================================

static void init_transmission_temp_tables(void);

// =============================================================================
// HARDWARE INITIALIZATION FUNCTIONS
// =============================================================================

static void init_transmission_sensor_array(sensor_definition_t* sensors, 
                                          float* temp_voltage_table, 
                                          float* temp_temp_table) {
    // Transmission fluid temperature sensor (thermistor)
    sensors[0].pin = PIN_TRANS_FLUID_TEMP;
    sensors[0].type = SENSOR_THERMISTOR;
    sensors[0].config.thermistor.pullup_ohms = TRANS_TEMP_PULLUP_OHMS;
    sensors[0].config.thermistor.voltage_table = temp_voltage_table;
    sensors[0].config.thermistor.temp_table = temp_temp_table;
    sensors[0].config.thermistor.table_size = TRANS_TEMP_TABLE_SIZE;
    sensors[0].msg_id = MSG_TRANS_FLUID_TEMP;
    sensors[0].update_interval_us = TRANS_TEMP_UPDATE_INTERVAL_US;
    sensors[0].filter_strength = TRANS_TEMP_FILTER_STRENGTH;
    sensors[0].name = "Trans Fluid Temp";
    
    // Paddle upshift
    sensors[1].pin = PIN_PADDLE_UPSHIFT;
    sensors[1].type = SENSOR_DIGITAL_PULLUP;
    sensors[1].config.digital.use_pullup = 1;
    sensors[1].config.digital.invert_logic = 1;
    sensors[1].msg_id = MSG_PADDLE_UPSHIFT;
    sensors[1].update_interval_us = PADDLE_UPDATE_INTERVAL_US;
    sensors[1].filter_strength = PADDLE_FILTER_STRENGTH;
    sensors[1].name = "Paddle Upshift";
    
    // Paddle downshift
    sensors[2].pin = PIN_PADDLE_DOWNSHIFT;
    sensors[2].type = SENSOR_DIGITAL_PULLUP;
    sensors[2].config.digital.use_pullup = 1;
    sensors[2].config.digital.invert_logic = 1;
    sensors[2].msg_id = MSG_PADDLE_DOWNSHIFT;
    sensors[2].update_interval_us = PADDLE_UPDATE_INTERVAL_US;
    sensors[2].filter_strength = PADDLE_FILTER_STRENGTH;
    sensors[2].name = "Paddle Downshift";
    
    // Park switch
    sensors[3].pin = PIN_TRANS_PARK;
    sensors[3].type = SENSOR_DIGITAL_PULLUP;
    sensors[3].config.digital.use_pullup = 1;
    sensors[3].config.digital.invert_logic = 1;
    sensors[3].msg_id = MSG_TRANS_PARK_SWITCH;
    sensors[3].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[3].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[3].name = "Trans Park Switch";
    
    // Reverse switch
    sensors[4].pin = PIN_TRANS_REVERSE;
    sensors[4].type = SENSOR_DIGITAL_PULLUP;
    sensors[4].config.digital.use_pullup = 1;
    sensors[4].config.digital.invert_logic = 1;
    sensors[4].msg_id = MSG_TRANS_REVERSE_SWITCH;
    sensors[4].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[4].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[4].name = "Trans Reverse Switch";
    
    // Neutral switch
    sensors[5].pin = PIN_TRANS_NEUTRAL;
    sensors[5].type = SENSOR_DIGITAL_PULLUP;
    sensors[5].config.digital.use_pullup = 1;
    sensors[5].config.digital.invert_logic = 1;
    sensors[5].msg_id = MSG_TRANS_NEUTRAL_SWITCH;
    sensors[5].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[5].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[5].name = "Trans Neutral Switch";
    
    // Drive switch
    sensors[6].pin = PIN_TRANS_DRIVE;
    sensors[6].type = SENSOR_DIGITAL_PULLUP;
    sensors[6].config.digital.use_pullup = 1;
    sensors[6].config.digital.invert_logic = 1;
    sensors[6].msg_id = MSG_TRANS_DRIVE_SWITCH;
    sensors[6].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[6].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[6].name = "Trans Drive Switch";
    
    // Second switch
    sensors[7].pin = PIN_TRANS_SECOND;
    sensors[7].type = SENSOR_DIGITAL_PULLUP;
    sensors[7].config.digital.use_pullup = 1;
    sensors[7].config.digital.invert_logic = 1;
    sensors[7].msg_id = MSG_TRANS_SECOND_SWITCH;
    sensors[7].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[7].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[7].name = "Trans Second Switch";
    
    // First switch
    sensors[8].pin = PIN_TRANS_FIRST;
    sensors[8].type = SENSOR_DIGITAL_PULLUP;
    sensors[8].config.digital.use_pullup = 1;
    sensors[8].config.digital.invert_logic = 1;
    sensors[8].msg_id = MSG_TRANS_FIRST_SWITCH;
    sensors[8].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[8].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[8].name = "Trans First Switch";
}

// TODO: Temporarily commented out due to header conflicts between 
// input_manager_types.h and output_manager_types.h (both define digital_config_t differently)
/*
static void init_transmission_output_array(output_definition_t* outputs) {
    // Shift Solenoid A (Digital ON/OFF)
    outputs[0].pin = PIN_TRANS_SHIFT_SOL_A;
    outputs[0].type = OUTPUT_PWM;
    outputs[0].config.pwm.frequency_hz = TRANS_SOLENOID_PWM_FREQ;
    outputs[0].config.pwm.min_duty_cycle = 0.0f;
    outputs[0].config.pwm.max_duty_cycle = 1.0f;
    outputs[0].config.pwm.default_duty_cycle = TRANS_SOLENOID_DEFAULT_DUTY;
    outputs[0].config.pwm.invert_output = 0;
    outputs[0].msg_id = MSG_TRANS_SHIFT_SOL_A;
    outputs[0].current_value = TRANS_SOLENOID_DEFAULT_DUTY;
    outputs[0].last_update_time_ms = 0;
    outputs[0].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[0].fault_detected = 0;
    outputs[0].name = "Trans Shift Sol A";
    
    // ... (rest of the output definitions)
}
*/
static void subscribe_to_transmission_messages(void);
static void handle_trans_fluid_temp(const CANMessage* msg);
static void handle_paddle_upshift(const CANMessage* msg);
static void handle_paddle_downshift(const CANMessage* msg);
static void handle_gear_position_switches(const CANMessage* msg);
static void handle_throttle_position(const CANMessage* msg);
static void handle_vehicle_speed(const CANMessage* msg);
static void handle_brake_pedal(const CANMessage* msg);
static void handle_transmission_parameter_request(const CANMessage* msg);
static void update_gear_position(void);
static void process_shift_requests(void);
static void publish_transmission_state(void);
static bool is_shift_safe(void);
static bool execute_upshift(void);
static bool execute_downshift(void);
static void set_shift_solenoid_pattern(uint8_t gear);
static void set_line_pressure_for_gear(gear_position_t gear);
static void set_line_pressure(float pressure_percent);

// Solenoid state getters for parameter requests
static float get_shift_solenoid_a_state(void);
static float get_shift_solenoid_b_state(void);
static float get_lockup_solenoid_state(void);
static float get_pressure_solenoid_state(void);
static float get_overrun_solenoid_state(void);

// Overrun clutch control functions
static overrun_clutch_state_t calculate_overrun_clutch_state(void);
static void set_overrun_clutch(overrun_clutch_state_t state);
static void update_overrun_clutch_control(void);

// Helper functions for external data
static float get_throttle_position_with_timeout(void);
static float get_vehicle_speed_with_timeout(void);
static bool get_brake_pedal_with_timeout(void);
static bool is_decelerating_with_timeout(void);

// =============================================================================
// PUBLIC FUNCTIONS
// =============================================================================

uint8_t transmission_module_init(void) {
    #ifdef ARDUINO
    Serial.println("Initializing transmission module...");
    #endif
    
    // Initialize thermistor lookup tables
    init_transmission_temp_tables();
    
    // Initialize sensor arrays using the hardware definitions
    init_transmission_sensor_array(TRANSMISSION_SENSORS, trans_temp_voltage_table, trans_temp_temp_table);
    // TODO: Initialize output arrays when header conflicts are resolved
    // init_transmission_output_array(TRANSMISSION_OUTPUTS);
    
    // Register all transmission sensors with the input manager
    uint8_t registered_sensors = input_manager_register_sensors(TRANSMISSION_SENSORS, TRANSMISSION_SENSOR_COUNT);
    
    // TODO: Register outputs with output manager when header conflicts are resolved
    // uint8_t registered_outputs = output_manager_register_outputs(TRANSMISSION_OUTPUTS, TRANSMISSION_OUTPUT_COUNT);
    uint8_t registered_outputs = TRANSMISSION_OUTPUT_COUNT;  // For now, assume all outputs registered
    (void)registered_outputs;  // Suppress unused variable warning
    
    // Subscribe to transmission messages
    subscribe_to_transmission_messages();
    
    // Initialize state
    trans_state.current_gear = GEAR_UNKNOWN;
    trans_state.fluid_temperature = 0.0f;
    trans_state.shift_request = SHIFT_NONE;
    trans_state.last_paddle_time_ms = 0;
    trans_state.valid_gear_position = false;
    trans_state.upshift_requested = false;
    trans_state.downshift_requested = false;
    trans_state.overrun_state = OVERRUN_DISENGAGED;  // Start in safe state
    trans_state.park_switch = false;
    trans_state.reverse_switch = false;
    trans_state.neutral_switch = false;
    trans_state.drive_switch = false;
    trans_state.second_switch = false;
    trans_state.first_switch = false;
    
    // Reset statistics
    shift_count = 0;
    invalid_gear_count = 0;
    overrun_change_count = 0;
    
    // Initialize cached external data timestamps
    last_throttle_update_ms = 0;
    last_speed_update_ms = 0;
    last_brake_update_ms = 0;
    
    // Set transmission outputs to safe state initially
    transmission_outputs_safe_state();
    
    #ifdef ARDUINO
    Serial.print("Transmission module initialized with ");
    Serial.print(registered_sensors);
    Serial.print(" sensors and ");
    Serial.print(registered_outputs);
    Serial.println(" outputs");
    Serial.print("Paddle debounce time: ");
    Serial.print(paddle_debounce_ms);
    Serial.println("ms");
    Serial.println("Race car overrun clutch control enabled");
    #endif
    
    return registered_sensors;
}

void transmission_module_update(void) {
    // Update gear position based on switch states
    update_gear_position();
    
    // Process any pending shift requests
    process_shift_requests();
    
    // Update overrun clutch control based on driving conditions (race car logic)
    update_overrun_clutch_control();
    
    // Publish current transmission state
    publish_transmission_state();
}

const transmission_state_t* transmission_get_state(void) {
    return &trans_state;
}

void transmission_clear_shift_request(void) {
    trans_state.upshift_requested = false;
    trans_state.downshift_requested = false;
    trans_state.shift_request = SHIFT_NONE;
}

bool transmission_is_overheating(float threshold_c) {
    return trans_state.fluid_temperature > threshold_c;
}

const char* transmission_gear_to_string(gear_position_t gear) {
    switch (gear) {
        case GEAR_PARK: return "P";
        case GEAR_REVERSE: return "R";
        case GEAR_NEUTRAL: return "N";
        case GEAR_DRIVE: return "D";
        case GEAR_SECOND: return "2";
        case GEAR_FIRST: return "1";
        case GEAR_UNKNOWN: 
        default: return "?";
    }
}

const char* transmission_overrun_to_string(overrun_clutch_state_t state) {
    switch (state) {
        case OVERRUN_ENGAGED: return "ENGAGED";
        case OVERRUN_DISENGAGED: return "DISENGAGED";
        default: return "UNKNOWN";
    }
}

void transmission_set_paddle_debounce(uint16_t debounce_ms) {
    paddle_debounce_ms = debounce_ms;
}

uint16_t transmission_get_paddle_debounce(void) {
    return paddle_debounce_ms;
}

uint32_t transmission_get_shift_count(void) {
    return shift_count;
}

uint32_t transmission_get_invalid_gear_count(void) {
    return invalid_gear_count;
}

uint32_t transmission_get_overrun_change_count(void) {
    return overrun_change_count;
}

void transmission_reset_statistics(void) {
    shift_count = 0;
    invalid_gear_count = 0;
    overrun_change_count = 0;
}

void transmission_set_overrun_override(overrun_clutch_state_t state, bool override_enable) {
    overrun_manual_override_active = override_enable;
    overrun_manual_override_state = state;
    
    if (override_enable) {
        // Apply the manual override immediately
        set_overrun_clutch(state);
        
        #ifdef ARDUINO
        Serial.print("Overrun clutch manual override ENABLED: ");
        Serial.println(transmission_overrun_to_string(state));
        #endif
    } else {
        #ifdef ARDUINO
        Serial.println("Overrun clutch manual override DISABLED - returning to automatic control");
        #endif
    }
}

bool transmission_is_overrun_override_active(void) {
    return overrun_manual_override_active;
}

void transmission_set_overrun_tuning(float throttle_disengage_pct, float throttle_engage_pct, 
                                     float min_speed_mph, float braking_speed_mph) {
    // Clamp values to reasonable ranges for safety
    overrun_throttle_disengage_threshold = (throttle_disengage_pct < 10.0f) ? 10.0f : 
                                          (throttle_disengage_pct > 100.0f) ? 100.0f : throttle_disengage_pct;
    overrun_throttle_engage_threshold = (throttle_engage_pct < 0.0f) ? 0.0f : 
                                       (throttle_engage_pct > 50.0f) ? 50.0f : throttle_engage_pct;
    overrun_minimum_speed_mph = (min_speed_mph < 0.0f) ? 0.0f : 
                               (min_speed_mph > 30.0f) ? 30.0f : min_speed_mph;
    overrun_braking_speed_threshold = (braking_speed_mph < 10.0f) ? 10.0f : 
                                     (braking_speed_mph > 100.0f) ? 100.0f : braking_speed_mph;
    
    #ifdef ARDUINO
    Serial.println("Overrun clutch tuning parameters updated:");
    Serial.print("  Throttle disengage: ");
    Serial.print(overrun_throttle_disengage_threshold);
    Serial.println("%");
    Serial.print("  Throttle engage: ");
    Serial.print(overrun_throttle_engage_threshold);
    Serial.println("%");
    Serial.print("  Minimum speed: ");
    Serial.print(overrun_minimum_speed_mph);
    Serial.println(" mph");
    Serial.print("  Braking speed threshold: ");
    Serial.print(overrun_braking_speed_threshold);
    Serial.println(" mph");
    #endif
}

void transmission_get_overrun_tuning(float* throttle_disengage_pct, float* throttle_engage_pct,
                                     float* min_speed_mph, float* braking_speed_mph) {
    if (throttle_disengage_pct) *throttle_disengage_pct = overrun_throttle_disengage_threshold;
    if (throttle_engage_pct) *throttle_engage_pct = overrun_throttle_engage_threshold;
    if (min_speed_mph) *min_speed_mph = overrun_minimum_speed_mph;
    if (braking_speed_mph) *braking_speed_mph = overrun_braking_speed_threshold;
}

void transmission_set_lockup(bool engage) {
    // Publish lockup control message - output manager will handle the rest
    g_message_bus.publishFloat(MSG_TRANS_LOCKUP_SOL, engage ? 1.0f : 0.0f);
    #ifdef ARDUINO
    Serial.print("Lockup ");
    Serial.println(engage ? "engaged" : "disengaged");
    #endif
}

void transmission_set_line_pressure(float pressure_percent) {
    set_line_pressure(pressure_percent);
}

void transmission_set_solenoid_pattern(uint8_t gear) {
    set_shift_solenoid_pattern(gear);
}

void transmission_set_auto_shift(bool enable) {
    // Implementation depends on how automatic shifting is controlled
    // For now, just log the setting
    #ifdef ARDUINO
    Serial.print("Automatic shifting ");
    Serial.println(enable ? "enabled" : "disabled");
    #endif
}

void transmission_outputs_safe_state(void) {
    // Set all outputs to safe state
    set_shift_solenoid_pattern(0);        // Both shift solenoids OFF (Park/Neutral)
    set_line_pressure_for_gear(GEAR_PARK); // No pressure (0%)
    transmission_set_lockup(false);       // Lockup disengaged
    set_overrun_clutch(OVERRUN_DISENGAGED); // Overrun clutch disengaged for safe operation
    
    #ifdef ARDUINO
    Serial.println("Transmission outputs set to safe state");
    #endif
}

// =============================================================================
// PRIVATE FUNCTIONS
// =============================================================================

static void init_transmission_temp_tables(void) {
    // Generate thermistor lookup table using the table generator
    float beta = generate_thermistor_table(
        TRANS_TEMP_REF1_C, TRANS_TEMP_REF1_OHMS,    // First reference point
        TRANS_TEMP_REF2_C, TRANS_TEMP_REF2_OHMS,    // Second reference point
        TRANS_TEMP_PULLUP_OHMS,                     // Pullup resistor
        TRANS_TEMP_MIN_C, TRANS_TEMP_MAX_C,         // Temperature range
        TRANS_TEMP_TABLE_SIZE,                      // Table size
        trans_temp_voltage_table,                   // Output voltage table
        trans_temp_temp_table                       // Output temperature table
    );
    
    #ifdef ARDUINO
    Serial.print("Transmission temp sensor initialized, Beta = ");
    Serial.println(beta);
    Serial.print("Temperature range: ");
    Serial.print(TRANS_TEMP_MIN_C);
    Serial.print("°C to ");
    Serial.print(TRANS_TEMP_MAX_C);
    Serial.println("°C");
    #else
    (void)beta;  // Suppress unused variable warning in tests
    #endif
}

static void subscribe_to_transmission_messages(void) {
    // Subscribe to sensor messages
    g_message_bus.subscribe(MSG_TRANS_FLUID_TEMP, handle_trans_fluid_temp);
    g_message_bus.subscribe(MSG_PADDLE_UPSHIFT, handle_paddle_upshift);
    g_message_bus.subscribe(MSG_PADDLE_DOWNSHIFT, handle_paddle_downshift);
    
    // Subscribe to all gear position switches with the same handler
    g_message_bus.subscribe(MSG_TRANS_PARK_SWITCH, handle_gear_position_switches);
    g_message_bus.subscribe(MSG_TRANS_REVERSE_SWITCH, handle_gear_position_switches);
    g_message_bus.subscribe(MSG_TRANS_NEUTRAL_SWITCH, handle_gear_position_switches);
    g_message_bus.subscribe(MSG_TRANS_DRIVE_SWITCH, handle_gear_position_switches);
    g_message_bus.subscribe(MSG_TRANS_SECOND_SWITCH, handle_gear_position_switches);
    g_message_bus.subscribe(MSG_TRANS_FIRST_SWITCH, handle_gear_position_switches);
    
    // Subscribe to external data for overrun clutch control
    g_message_bus.subscribe(MSG_THROTTLE_POSITION, handle_throttle_position);
    g_message_bus.subscribe(MSG_VEHICLE_SPEED, handle_vehicle_speed);
    g_message_bus.subscribe(MSG_BRAKE_PEDAL, handle_brake_pedal);
    
    // Subscribe to parameter requests for transmission status
    g_message_bus.subscribe(MSG_TRANS_CURRENT_GEAR, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_SHIFT_REQUEST, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_OVERRUN_STATE, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_STATE_VALID, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_A, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_B, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_LOCKUP_SOL, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_PRESSURE_SOL, handle_transmission_parameter_request);
    g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, handle_transmission_parameter_request);
}

static void handle_trans_fluid_temp(const CANMessage* msg) {
    trans_state.fluid_temperature = MSG_UNPACK_FLOAT(msg);
}

static void handle_throttle_position(const CANMessage* msg) {
    cached_throttle_position = MSG_UNPACK_FLOAT(msg);
    last_throttle_update_ms = millis();
}

static void handle_vehicle_speed(const CANMessage* msg) {
    cached_vehicle_speed = MSG_UNPACK_FLOAT(msg);
    last_speed_update_ms = millis();
}

static void handle_brake_pedal(const CANMessage* msg) {
    cached_brake_active = (MSG_UNPACK_FLOAT(msg) > 0.5f);  // Convert to boolean
    last_brake_update_ms = millis();
}

static void handle_paddle_upshift(const CANMessage* msg) {
    if (MSG_UNPACK_FLOAT(msg) > 0.5f) {  // Paddle pressed
        uint32_t now_ms = millis();
        if (now_ms - trans_state.last_paddle_time_ms >= paddle_debounce_ms) {
            trans_state.upshift_requested = true;
            trans_state.shift_request = SHIFT_UP;
            trans_state.last_paddle_time_ms = now_ms;
            shift_count++;
            
            #ifdef ARDUINO
            Serial.println("Upshift paddle pressed");
            #endif
        }
    }
}

static void handle_paddle_downshift(const CANMessage* msg) {
    if (MSG_UNPACK_FLOAT(msg) > 0.5f) {  // Paddle pressed
        uint32_t now_ms = millis();
        if (now_ms - trans_state.last_paddle_time_ms >= paddle_debounce_ms) {
            trans_state.downshift_requested = true;
            trans_state.shift_request = SHIFT_DOWN;
            trans_state.last_paddle_time_ms = now_ms;
            shift_count++;
            
            #ifdef ARDUINO
            Serial.println("Downshift paddle pressed");
            #endif
        }
    }
}

static void handle_gear_position_switches(const CANMessage* msg) {
    // Update individual switch states based on message ID
    bool switch_active = (MSG_UNPACK_FLOAT(msg) > 0.5f);
    
    switch (msg->id) {
        case MSG_TRANS_PARK_SWITCH:     trans_state.park_switch = switch_active; break;
        case MSG_TRANS_REVERSE_SWITCH:  trans_state.reverse_switch = switch_active; break;
        case MSG_TRANS_NEUTRAL_SWITCH:  trans_state.neutral_switch = switch_active; break;
        case MSG_TRANS_DRIVE_SWITCH:    trans_state.drive_switch = switch_active; break;
        case MSG_TRANS_SECOND_SWITCH:   trans_state.second_switch = switch_active; break;
        case MSG_TRANS_FIRST_SWITCH:    trans_state.first_switch = switch_active; break;
    }
}

// =============================================================================
// PARAMETER REQUEST HANDLER
// =============================================================================

static void handle_transmission_parameter_request(const CANMessage* msg) {
    // Validate parameter message
    if (!is_valid_parameter_message(msg)) {
        return;
    }
    
    parameter_msg_t* param = get_parameter_msg(msg);
    
    // Only handle read requests for transmission parameters (read-only)
    if (param->operation == PARAM_OP_READ_REQUEST) {
        float current_value = 0.0f;
        
        // Get current value based on parameter ID
        switch (msg->id) {
            case MSG_TRANS_CURRENT_GEAR:
                current_value = (float)trans_state.current_gear;
                break;
            case MSG_TRANS_SHIFT_REQUEST:
                current_value = (float)trans_state.shift_request;
                break;
            case MSG_TRANS_OVERRUN_STATE:
                current_value = (float)trans_state.overrun_state;
                break;
            case MSG_TRANS_STATE_VALID:
                current_value = trans_state.valid_gear_position ? 1.0f : 0.0f;
                break;
            case MSG_TRANS_SHIFT_SOL_A:
                current_value = (get_shift_solenoid_a_state() > 0.5f) ? 1.0f : 0.0f;
                break;
            case MSG_TRANS_SHIFT_SOL_B:
                current_value = (get_shift_solenoid_b_state() > 0.5f) ? 1.0f : 0.0f;
                break;
            case MSG_TRANS_LOCKUP_SOL:
                current_value = (get_lockup_solenoid_state() > 0.5f) ? 1.0f : 0.0f;
                break;
            case MSG_TRANS_PRESSURE_SOL:
                current_value = get_pressure_solenoid_state();
                break;
            case MSG_TRANS_OVERRUN_SOL:
                current_value = (get_overrun_solenoid_state() > 0.5f) ? 1.0f : 0.0f;
                break;
            default:
                // Unknown parameter - send error
                send_parameter_error(msg->id, PARAM_OP_READ_REQUEST, 
                                   PARAM_ERROR_INVALID_OPERATION, 0.0f);
                return;
        }
        
        // Send response with current value
        send_parameter_response(msg->id, PARAM_OP_READ_RESPONSE, current_value);
        
    } else if (param->operation == PARAM_OP_WRITE_REQUEST) {
        // Transmission parameters are read-only
        send_parameter_error(msg->id, PARAM_OP_WRITE_REQUEST, 
                           PARAM_ERROR_READ_ONLY, param->value);
    } else {
        // Invalid operation
        send_parameter_error(msg->id, param->operation, 
                           PARAM_ERROR_INVALID_OPERATION, param->value);
    }
}

// =============================================================================
// EXTERNAL DATA HELPER FUNCTIONS (MESSAGE BUS WITH TIMEOUT)
// =============================================================================

static float get_throttle_position_with_timeout(void) {
    uint32_t now_ms = millis();
    
    // Check if data is fresh (within timeout)
    if (now_ms - last_throttle_update_ms < EXTERNAL_DATA_TIMEOUT_MS) {
        return cached_throttle_position;
    } else {
        // Return safe default if data is stale
        return 20.0f;  // Safe light throttle default
    }
}

static float get_vehicle_speed_with_timeout(void) {
    uint32_t now_ms = millis();
    
    // Check if data is fresh (within timeout)
    if (now_ms - last_speed_update_ms < EXTERNAL_DATA_TIMEOUT_MS) {
        return cached_vehicle_speed;
    } else {
        // Return safe default if data is stale
        return 35.0f;  // Safe moderate speed default
    }
}

static bool get_brake_pedal_with_timeout(void) {
    uint32_t now_ms = millis();
    
    // Check if data is fresh (within timeout)
    if (now_ms - last_brake_update_ms < EXTERNAL_DATA_TIMEOUT_MS) {
        return cached_brake_active;
    } else {
        // Return safe default if data is stale
        return false;  // Safe default (no braking)
    }
}

static bool is_decelerating_with_timeout(void) {
    // Calculate deceleration based on throttle position
    float throttle = get_throttle_position_with_timeout();
    return (throttle < 10.0f);  // Consider very light throttle as potential deceleration
}

static void update_gear_position(void) {
    // Count active switches
    uint8_t active_count = trans_state.park_switch + trans_state.reverse_switch + 
                          trans_state.neutral_switch + trans_state.drive_switch + 
                          trans_state.second_switch + trans_state.first_switch;
    
    gear_position_t previous_gear = trans_state.current_gear;
    
    if (active_count == 1) {
        // Valid state - exactly one switch active
        trans_state.valid_gear_position = true;
        
        if (trans_state.park_switch) trans_state.current_gear = GEAR_PARK;
        else if (trans_state.reverse_switch) trans_state.current_gear = GEAR_REVERSE;
        else if (trans_state.neutral_switch) trans_state.current_gear = GEAR_NEUTRAL;
        else if (trans_state.drive_switch) trans_state.current_gear = GEAR_DRIVE;
        else if (trans_state.second_switch) trans_state.current_gear = GEAR_SECOND;
        else if (trans_state.first_switch) trans_state.current_gear = GEAR_FIRST;
        
        // If gear position changed, update solenoid pattern and line pressure
        if (trans_state.current_gear != previous_gear) {
            if (trans_state.current_gear == GEAR_DRIVE) {
                // Entering Drive - set to current automatic gear
                set_shift_solenoid_pattern(current_auto_gear);
            } else {
                // Not in Drive - turn off shift solenoids (Park/Reverse/Neutral/Manual)
                set_shift_solenoid_pattern(0);  // Both solenoids OFF
            }
            
            // Set line pressure based on gear position
            set_line_pressure_for_gear(trans_state.current_gear);
            
            #ifdef ARDUINO
            Serial.print("Gear position changed to: ");
            Serial.println(transmission_gear_to_string(trans_state.current_gear));
            #endif
        }
    } else {
        // Invalid state - multiple switches active or no switches active
        trans_state.valid_gear_position = false;
        trans_state.current_gear = GEAR_NEUTRAL;  // Default to neutral for safety
        invalid_gear_count++;
        
        // Set safe state - solenoids off, neutral pressure (0%)
        set_shift_solenoid_pattern(0);
        set_line_pressure_for_gear(GEAR_NEUTRAL);
        
        #ifdef ARDUINO
        Serial.print("Invalid gear position - ");
        Serial.print(active_count);
        Serial.println(" switches active, defaulting to neutral");
        #endif
    }
}

static void process_shift_requests(void) {
    if (trans_state.shift_request != SHIFT_NONE) {
        // 1. Check if shifting is safe
        if (!is_shift_safe()) {
            #ifdef ARDUINO
            Serial.println("Shift request denied - conditions not safe");
            #endif
            transmission_clear_shift_request();
            return;
        }
        
        // 2. Execute the shift
        bool shift_successful = false;
        if (trans_state.shift_request == SHIFT_UP) {
            shift_successful = execute_upshift();
        } else if (trans_state.shift_request == SHIFT_DOWN) {
            shift_successful = execute_downshift();
        }
        
        if (shift_successful) {
            #ifdef ARDUINO
            Serial.print("Executed ");
            Serial.print(trans_state.shift_request == SHIFT_UP ? "upshift" : "downshift");
            Serial.print(" to automatic gear ");
            Serial.println(current_auto_gear);
            #endif
        } else {
            #ifdef ARDUINO
            Serial.println("Shift execution failed");
            #endif
        }
        
        // 3. Clear the shift request
        transmission_clear_shift_request();
    }
}

static void publish_transmission_state(void) {
    // Publish combined transmission state messages
    g_message_bus.publishFloat(MSG_TRANS_CURRENT_GEAR, (float)trans_state.current_gear);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_REQUEST, (float)trans_state.shift_request);
    g_message_bus.publishFloat(MSG_TRANS_STATE_VALID, trans_state.valid_gear_position ? 1.0f : 0.0f);
    g_message_bus.publishFloat(MSG_TRANS_OVERRUN_STATE, (float)trans_state.overrun_state);
}

static bool is_shift_safe(void) {
    // Check if shifting is safe based on current conditions
    
    // 1. Must have valid gear position
    if (!trans_state.valid_gear_position) {
        return false;
    }
    
    // 2. Can ONLY shift when shift lever is in Drive position
    if (trans_state.current_gear != GEAR_DRIVE) {
        return false;
    }
    
    // 3. Cannot shift if transmission is overheating
    if (transmission_is_overheating(120.0f)) {  // 120°C threshold
        return false;
    }
    
    return true;
}

static bool execute_upshift(void) {
    // Execute upshift - only works when in Drive position
    // Shift from current automatic gear to next higher gear (1→2→3→4)
    
    if (current_auto_gear >= 4) {
        // Already in highest gear
        return false;
    }
    
    // Shift to next gear
    current_auto_gear++;
    set_shift_solenoid_pattern(current_auto_gear);
    // Line pressure remains at 100% for all moving gears
    
    #ifdef ARDUINO
    Serial.print("Upshift executed: Drive gear ");
    Serial.println(current_auto_gear);
    #endif
    
    return true;
}

static bool execute_downshift(void) {
    // Execute downshift - only works when in Drive position
    // Shift from current automatic gear to next lower gear (4→3→2→1)
    
    if (current_auto_gear <= 1) {
        // Already in lowest gear
        return false;
    }
    
    // Shift to lower gear
    current_auto_gear--;
    set_shift_solenoid_pattern(current_auto_gear);
    // Line pressure remains at 100% for all moving gears
    
    #ifdef ARDUINO
    Serial.print("Downshift executed: Drive gear ");
    Serial.println(current_auto_gear);
    #endif
    
    return true;
}

static void set_shift_solenoid_pattern(uint8_t gear) {
    // Set solenoid pattern for specific gear using message bus
    // Based on actual transmission logic:
    // Park/Reverse/Neutral: A=OFF, B=OFF, Lockup=OFF
    // Gear 1: A=ON, B=ON, Lockup=OFF
    // Gear 2: A=OFF, B=ON, Lockup=OFF  
    // Gear 3: A=OFF, B=OFF, Lockup=OFF
    // Gear 4: A=ON, B=OFF, Lockup=ON
    
    bool sol_a_state = false;
    bool sol_b_state = false;
    bool lockup_state = false;
    
    switch (gear) {
        case 1:  // Gear 1: A=ON, B=ON, Lockup=OFF
            sol_a_state = true;
            sol_b_state = true;
            lockup_state = false;
            break;
        case 2:  // Gear 2: A=OFF, B=ON, Lockup=OFF
            sol_a_state = false;
            sol_b_state = true;
            lockup_state = false;
            break;
        case 3:  // Gear 3: A=OFF, B=OFF, Lockup=OFF
            sol_a_state = false;
            sol_b_state = false;
            lockup_state = false;
            break;
        case 4:  // Gear 4: A=ON, B=OFF, Lockup=ON
            sol_a_state = true;
            sol_b_state = false;
            lockup_state = true;
            break;
        default: // Park/Reverse/Neutral: A=OFF, B=OFF, Lockup=OFF
            sol_a_state = false;
            sol_b_state = false;
            lockup_state = false;
            break;
    }
    
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_A, sol_a_state ? 1.0f : 0.0f);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_B, sol_b_state ? 1.0f : 0.0f);
    g_message_bus.publishFloat(MSG_TRANS_LOCKUP_SOL, lockup_state ? 1.0f : 0.0f);
                            
    #ifdef ARDUINO
    Serial.print("Solenoids - Gear ");
    Serial.print(gear);
    Serial.print(": A=");
    Serial.print(sol_a_state ? "ON" : "OFF");
    Serial.print(", B=");
    Serial.print(sol_b_state ? "ON" : "OFF");
    Serial.print(", Lockup=");
    Serial.println(lockup_state ? "ON" : "OFF");
    #endif
}

static void set_line_pressure_for_gear(gear_position_t gear) {
    // Set line pressure based on gear position:
    // - OFF (0%) in Park and Neutral (no hydraulic pressure needed)
    // - ON (100%) in all moving gears (Reverse, Drive, manual gears)
    
    float pressure_percent;
    
    if (gear == GEAR_PARK || gear == GEAR_NEUTRAL) {
        pressure_percent = 0.0f;  // No pressure needed
    } else {
        pressure_percent = 1.0f;  // Full pressure for all moving gears
    }
    
    // Publish pressure control message
    g_message_bus.publishFloat(MSG_TRANS_PRESSURE_SOL, pressure_percent);
                            
    #ifdef ARDUINO
    Serial.print("Line pressure set for ");
    Serial.print(transmission_gear_to_string(gear));
    Serial.print(": ");
    Serial.print(pressure_percent * 100.0f);
    Serial.println("%");
    #endif
}

static void set_line_pressure(float pressure_percent) {
    // Manual line pressure control (for testing/diagnostics)
    // Clamp to safe range
    if (pressure_percent < 0.0f) pressure_percent = 0.0f;
    if (pressure_percent > 1.0f) pressure_percent = 1.0f;
    
    // Publish pressure control message
    g_message_bus.publishFloat(MSG_TRANS_PRESSURE_SOL, pressure_percent);
                            
    #ifdef ARDUINO
    Serial.print("Line pressure manually set to: ");
    Serial.print(pressure_percent * 100.0f);
    Serial.println("%");
    #endif
}

// =============================================================================
// SOLENOID STATE GETTERS
// =============================================================================

static float get_shift_solenoid_a_state(void) {
    // Return current shift solenoid A state based on gear
    if (trans_state.current_gear == GEAR_DRIVE) {
        return (current_auto_gear == 1 || current_auto_gear == 4) ? 1.0f : 0.0f;
    }
    return 0.0f;  // OFF for all other gears
}

static float get_shift_solenoid_b_state(void) {
    // Return current shift solenoid B state based on gear
    if (trans_state.current_gear == GEAR_DRIVE) {
        return (current_auto_gear == 1 || current_auto_gear == 2) ? 1.0f : 0.0f;
    }
    return 0.0f;  // OFF for all other gears
}

static float get_lockup_solenoid_state(void) {
    // Return current lockup solenoid state based on gear
    if (trans_state.current_gear == GEAR_DRIVE) {
        return (current_auto_gear == 4) ? 1.0f : 0.0f;  // Only ON in 4th gear
    }
    return 0.0f;  // OFF for all other gears
}

static float get_pressure_solenoid_state(void) {
    // Return current pressure solenoid state based on gear
    if (trans_state.current_gear == GEAR_PARK || trans_state.current_gear == GEAR_NEUTRAL) {
        return 0.0f;  // No pressure needed
    }
    return 1.0f;  // Full pressure for all moving gears
}

static float get_overrun_solenoid_state(void) {
    // Return current overrun solenoid state
    return (trans_state.overrun_state == OVERRUN_ENGAGED) ? 1.0f : 0.0f;
}

// =============================================================================
// OVERRUN CLUTCH CONTROL FUNCTIONS
// =============================================================================

static overrun_clutch_state_t calculate_overrun_clutch_state(void) {
    // If manual override is active, use the override state
    if (overrun_manual_override_active) {
        return overrun_manual_override_state;
    }
    
    // Get current transmission state
    const transmission_state_t* trans_state = transmission_get_state();
    
    // Always disengage during active shifting to prevent binding
    // This is critical for smooth, fast shifts under power
    if (trans_state->shift_request != SHIFT_NONE) {
        return OVERRUN_DISENGAGED;
    }
    
    // Follow manual specification: 4th gear keeps clutch disengaged
    // This might be for high-speed stability or specific transmission design
    if (trans_state->current_gear == GEAR_DRIVE && current_auto_gear == 4) {
        return OVERRUN_DISENGAGED;
    }
    
    // Get driving conditions from message bus (with timeout handling)
    float throttle_position = get_throttle_position_with_timeout();  // 0-100%
    float vehicle_speed = get_vehicle_speed_with_timeout();          // MPH
    bool is_braking = get_brake_pedal_with_timeout();                // True if brake pressed
    bool is_decelerating = is_decelerating_with_timeout();           // True if decelerating
    
    // RACE CAR SPECIFIC LOGIC - Aggressive engagement for maximum control
    
    // During braking zones - ALWAYS engage for maximum engine braking control
    // This helps with corner entry balance and gives driver more tools
    if (is_braking && vehicle_speed > overrun_braking_speed_threshold) {
        return OVERRUN_ENGAGED;
    }
    
    // Light throttle with decent speed - engage for precise control
    // Race drivers need immediate response to throttle lift
    if (throttle_position < overrun_throttle_engage_threshold && vehicle_speed > overrun_minimum_speed_mph) {
        return OVERRUN_ENGAGED;
    }
    
    // Moderate throttle in lower gears - keep engaged for responsiveness
    // Unlike street cars, we want aggressive response even under moderate load
    if (throttle_position < overrun_moderate_throttle_threshold && (current_auto_gear <= 2)) {
        return OVERRUN_ENGAGED;
    }
    
    // Deceleration scenarios - engage for driver control
    if (is_decelerating && vehicle_speed > overrun_minimum_speed_mph) {
        return OVERRUN_ENGAGED;
    }
    
    // High throttle (power application) - disengage to avoid drivetrain shock
    // This prevents harsh transitions that could upset the car during acceleration
    if (throttle_position > overrun_throttle_disengage_threshold) {
        return OVERRUN_DISENGAGED;
    }
    
    // Very low speeds - disengage for smooth pit lane/paddock driving
    if (vehicle_speed < overrun_minimum_speed_mph) {
        return OVERRUN_DISENGAGED;
    }
    
    // Park, Reverse, Neutral - always disengage for safety
    if (trans_state->current_gear == GEAR_PARK || 
        trans_state->current_gear == GEAR_REVERSE || 
        trans_state->current_gear == GEAR_NEUTRAL) {
        return OVERRUN_DISENGAGED;
    }
    
    // Default for race car: ENGAGED for maximum control and responsiveness
    // This is opposite of many street car strategies that prioritize comfort
    return OVERRUN_ENGAGED;
}

static void set_overrun_clutch(overrun_clutch_state_t state) {
    // Remember: Solenoid ON (12V) = Clutch OFF, Solenoid OFF (0V) = Clutch ON
    // This is the inverted logic specified in the manual
    bool solenoid_power = (state == OVERRUN_DISENGAGED);
    
    // Send control message to output manager
    g_message_bus.publishFloat(MSG_TRANS_OVERRUN_SOL, solenoid_power ? 1.0f : 0.0f);
    
    #ifdef ARDUINO
    Serial.print("Overrun clutch ");
    Serial.print(transmission_overrun_to_string(state));
    Serial.print(" (solenoid ");
    Serial.print(solenoid_power ? "ON-12V" : "OFF-0V");
    Serial.println(")");
    #endif
}

static void update_overrun_clutch_control(void) {
    // Calculate desired overrun clutch state based on current conditions
    overrun_clutch_state_t desired_state = calculate_overrun_clutch_state();
    
    // Only change state if it's different from current state
    if (desired_state != trans_state.overrun_state) {
        overrun_clutch_state_t previous_state = trans_state.overrun_state;
        trans_state.overrun_state = desired_state;
        overrun_change_count++;
        
        // Apply the new state to the hardware
        set_overrun_clutch(desired_state);
        
        #ifdef ARDUINO
        Serial.print("Overrun clutch state changed: ");
        Serial.print(transmission_overrun_to_string(previous_state));
        Serial.print(" → ");
        Serial.println(transmission_overrun_to_string(desired_state));
        #else
        // Suppress unused variable warning in non-Arduino builds
        (void)previous_state;
        #endif
    }
}