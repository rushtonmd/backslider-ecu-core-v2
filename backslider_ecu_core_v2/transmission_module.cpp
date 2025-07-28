// transmission_module.cpp
// Implementation of transmission control module with race car overrun clutch control
// Uses pure message bus architecture for all data exchange
//
// 5-Solenoid Transmission Control System:
// - Shift Solenoid A (Pin 21): Digital ON/OFF
// - Shift Solenoid B (Pin 22): Digital ON/OFF
// - Overrun Solenoid (Pin 23): PWM (Race car logic implemented)
// - Line Pressure Solenoid (Pin 19): PWM 0-100% (0% Park/Neutral, 100% all moving gears)
// - Lockup Solenoid (Pin 18): PWM (automatic - ON in 4th gear only)
//
// Gear Patterns (A/B/Lockup/Pressure):
// Park/Neutral: OFF/OFF/OFF/0%
// Reverse: OFF/OFF/OFF/100%
// Gear 1: ON/ON/OFF/100%
// Gear 2: OFF/ON/OFF/100%
// Gear 3: OFF/OFF/OFF/100%
// Gear 4: ON/OFF/ON/100%  (Lockup engages for fuel efficiency)

#include "transmission_module.h"
#include "input_manager.h"
#include "msg_bus.h"
#include "parameter_helpers.h"
#include "parameter_registry.h"
#include "thermistor_table_generator.h"
#include "custom_canbus_manager.h"
#include "external_message_broadcasting.h"
#include "output_manager.h"
#include <Arduino.h>

// Forward declarations to avoid header conflicts

// External global instances
extern CustomCanBusManager g_custom_canbus_manager;

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
#define TRANS_PRESSURE_PWM_FREQ     250     // 250Hz for line pressure solenoid (typical auto solenoid range)
#define TRANS_SOLENOID_PWM_FREQ     200     // 200Hz for digital solenoids (typical auto solenoid range)

// Timing constants
#define TRANS_OUTPUT_UPDATE_RATE_MS 10      // 10ms update rate for all outputs

// Default duty cycle constants
#define TRANS_PRESSURE_DEFAULT_DUTY 0.0f    // 0% pressure (safe for Park/Neutral)
#define TRANS_SOLENOID_DEFAULT_DUTY 0.0f    // OFF (safe for all digital solenoids)

// Number of sensors and outputs
#define TRANSMISSION_SENSOR_COUNT 10  // 1 thermistor + 2 paddles + 6 gear switches + 1 vehicle speed
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
static output_definition_t TRANSMISSION_OUTPUTS[TRANSMISSION_OUTPUT_COUNT];

// =============================================================================
// PRIVATE FUNCTION DECLARATIONS
// =============================================================================

static void init_transmission_temp_tables(void);
static void init_transmission_output_array(output_definition_t* outputs);

// =============================================================================
// HARDWARE INITIALIZATION FUNCTIONS
// =============================================================================

static void init_transmission_sensor_array(sensor_definition_t* sensors, 
                                          float* temp_voltage_table, 
                                          float* temp_temp_table) {
    // I2C GPIO Pin Mapping for MCP23017 (Gear Selector Switches):
    // Pin 0: Park switch
    // Pin 1: Reverse switch  
    // Pin 2: Neutral switch
    // Pin 3: Drive switch
    // Pin 4: Second switch
    // Pin 5: First switch
    // Pins 6-15: Available for future use
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
    
    // Park switch (Digital pin 3)
    sensors[3].pin = PIN_TRANS_PARK;
    sensors[3].type = SENSOR_DIGITAL_PULLUP;
    sensors[3].config.digital.invert_logic = 1;
    sensors[3].msg_id = MSG_TRANS_PARK_SWITCH;
    sensors[3].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[3].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[3].name = "Trans Park Switch";
    
    // Reverse switch (Digital pin 4)
    sensors[4].pin = PIN_TRANS_REVERSE;
    sensors[4].type = SENSOR_DIGITAL_PULLUP;
    sensors[4].config.digital.invert_logic = 1;
    sensors[4].msg_id = MSG_TRANS_REVERSE_SWITCH;
    sensors[4].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[4].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[4].name = "Trans Reverse Switch";
    
    // Neutral switch (Digital pin 5)
    sensors[5].pin = PIN_TRANS_NEUTRAL;
    sensors[5].type = SENSOR_DIGITAL_PULLUP;
    sensors[5].config.digital.invert_logic = 1;
    sensors[5].msg_id = MSG_TRANS_NEUTRAL_SWITCH;
    sensors[5].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[5].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[5].name = "Trans Neutral Switch";
    
    // Drive switch (Digital pin 6)
    sensors[6].pin = PIN_TRANS_DRIVE;
    sensors[6].type = SENSOR_DIGITAL_PULLUP;
    sensors[6].config.digital.invert_logic = 1;
    sensors[6].msg_id = MSG_TRANS_DRIVE_SWITCH;
    sensors[6].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[6].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[6].name = "Trans Drive Switch";
    
    // Second switch (Digital pin 24)
    sensors[7].pin = PIN_TRANS_SECOND;
    sensors[7].type = SENSOR_DIGITAL_PULLUP;
    sensors[7].config.digital.invert_logic = 1;
    sensors[7].msg_id = MSG_TRANS_SECOND_SWITCH;
    sensors[7].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[7].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[7].name = "Trans Second Switch";
    
    // First switch (Digital pin 25)
    sensors[8].pin = PIN_TRANS_FIRST;
    sensors[8].type = SENSOR_DIGITAL_PULLUP;
    sensors[8].config.digital.invert_logic = 1;
    sensors[8].msg_id = MSG_TRANS_FIRST_SWITCH;
    sensors[8].update_interval_us = GEAR_SWITCH_UPDATE_INTERVAL_US;
    sensors[8].filter_strength = GEAR_SWITCH_FILTER_STRENGTH;
    sensors[8].name = "Trans First Switch";

    // Vehicle speed sensor (Hall effect frequency sensor on pin 6)
    sensors[9].pin = PIN_VEHICLE_SPEED;
    sensors[9].type = SENSOR_FREQUENCY_COUNTER;
    sensors[9].config.frequency.pulses_per_unit = 4;              // 4 pulses per revolution (typical VSS)
    sensors[9].config.frequency.scaling_factor = 0.01f;          // Scale to MPH/KPH
    sensors[9].config.frequency.timeout_us = 2000000;            // 2 second timeout (vehicle stopped)
    sensors[9].config.frequency.message_update_rate_hz = 1;      // 1Hz message rate for debugging
    sensors[9].config.frequency.use_interrupts = 1;             // Use high-speed interrupts
    sensors[9].config.frequency.trigger_edge = 0;               // Rising edge (FREQ_EDGE_RISING = 0)
    sensors[9].msg_id = MSG_VEHICLE_SPEED;
    sensors[9].update_interval_us = 1000000;                     // 1Hz = 1000ms update
    sensors[9].filter_strength = 0;                             // No filtering for speed sensor
    sensors[9].name = "Vehicle Speed";
}

static void init_transmission_output_array(output_definition_t* outputs) {
    // Shift Solenoid A (Digital ON/OFF) - Pin 21
    outputs[0].pin = PIN_TRANS_SHIFT_SOL_A;
    outputs[0].type = OUTPUT_DIGITAL;
    outputs[0].config.digital.active_high = 1;      // Active high
    outputs[0].config.digital.default_state = 0;    // Default OFF (safe)
    outputs[0].config.digital.open_drain = 0;       // Push-pull output
    outputs[0].msg_id = MSG_TRANS_SHIFT_SOL_A;
    outputs[0].current_value = 0.0f;                // Start OFF
    outputs[0].last_update_time_ms = 0;
    outputs[0].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[0].fault_detected = 0;
    outputs[0].name = "Trans Shift Sol A";
    
    // Shift Solenoid B (Digital ON/OFF) - Pin 22
    outputs[1].pin = PIN_TRANS_SHIFT_SOL_B;
    outputs[1].type = OUTPUT_DIGITAL;
    outputs[1].config.digital.active_high = 1;      // Active high
    outputs[1].config.digital.default_state = 0;    // Default OFF (safe)
    outputs[1].config.digital.open_drain = 0;       // Push-pull output
    outputs[1].msg_id = MSG_TRANS_SHIFT_SOL_B;
    outputs[1].current_value = 0.0f;                // Start OFF
    outputs[1].last_update_time_ms = 0;
    outputs[1].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[1].fault_detected = 0;
    outputs[1].name = "Trans Shift Sol B";
    
    // Overrun Solenoid (PWM) - Pin 23
    outputs[2].pin = PIN_TRANS_OVERRUN_SOL;
    outputs[2].type = OUTPUT_PWM;
    outputs[2].config.pwm.frequency_hz = TRANS_SOLENOID_PWM_FREQ;
    outputs[2].config.pwm.resolution_bits = 8;      // 8-bit resolution
    outputs[2].config.pwm.min_duty_cycle = 0.0f;    // 0% minimum
    outputs[2].config.pwm.max_duty_cycle = 1.0f;    // 100% maximum
    outputs[2].config.pwm.default_duty_cycle = 0.0f; // Default OFF (safe)
    outputs[2].config.pwm.invert_output = 0;        // Normal polarity
    outputs[2].msg_id = MSG_TRANS_OVERRUN_SOL;
    outputs[2].current_value = 0.0f;                // Start OFF
    outputs[2].last_update_time_ms = 0;
    outputs[2].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[2].fault_detected = 0;
    outputs[2].name = "Trans Overrun Sol";
    
    // Pressure Solenoid (PWM 0-100%) - Pin 19
    outputs[3].pin = PIN_TRANS_PRESSURE_SOL;
    outputs[3].type = OUTPUT_PWM;
    outputs[3].config.pwm.frequency_hz = TRANS_PRESSURE_PWM_FREQ;
    outputs[3].config.pwm.resolution_bits = 10;     // 10-bit resolution for fine control
    outputs[3].config.pwm.min_duty_cycle = 0.0f;    // 0% minimum
    outputs[3].config.pwm.max_duty_cycle = 1.0f;    // 100% maximum
    outputs[3].config.pwm.default_duty_cycle = 0.0f; // Default OFF (safe)
    outputs[3].config.pwm.invert_output = 0;        // Normal polarity
    outputs[3].msg_id = MSG_TRANS_PRESSURE_SOL;
    outputs[3].current_value = 0.0f;                // Start OFF
    outputs[3].last_update_time_ms = 0;
    outputs[3].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[3].fault_detected = 0;
    outputs[3].name = "Trans Pressure Sol";
    
    // Lockup Solenoid (PWM) - Pin 18
    outputs[4].pin = PIN_TRANS_LOCKUP_SOL;
    outputs[4].type = OUTPUT_PWM;
    outputs[4].config.pwm.frequency_hz = TRANS_SOLENOID_PWM_FREQ;
    outputs[4].config.pwm.resolution_bits = 8;      // 8-bit resolution
    outputs[4].config.pwm.min_duty_cycle = 0.0f;    // 0% minimum
    outputs[4].config.pwm.max_duty_cycle = 1.0f;    // 100% maximum
    outputs[4].config.pwm.default_duty_cycle = 0.0f; // Default OFF (safe)
    outputs[4].config.pwm.invert_output = 0;        // Normal polarity
    outputs[4].msg_id = MSG_TRANS_LOCKUP_SOL;
    outputs[4].current_value = 0.0f;                // Start OFF
    outputs[4].last_update_time_ms = 0;
    outputs[4].update_rate_limit_ms = TRANS_OUTPUT_UPDATE_RATE_MS;
    outputs[4].fault_detected = 0;
    outputs[4].name = "Trans Lockup Sol";
}
static void subscribe_to_transmission_messages(void);
static void handle_trans_fluid_temp(const CANMessage* msg);
static void handle_paddle_upshift(const CANMessage* msg);
static void handle_paddle_downshift(const CANMessage* msg);
static void handle_gear_position_switches(const CANMessage* msg);
static void handle_throttle_position(const CANMessage* msg);
static void handle_vehicle_speed(const CANMessage* msg);
static void handle_brake_pedal(const CANMessage* msg);
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

// External CAN bus configuration
static void configure_external_canbus_mappings(void);

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
    
    // Initialize output arrays
    init_transmission_output_array(TRANSMISSION_OUTPUTS);
    
    // Register all transmission sensors with the input manager
    uint8_t trans_sensors_registered = input_manager_register_sensors(TRANSMISSION_SENSORS, TRANSMISSION_SENSOR_COUNT);
    
    #ifdef ARDUINO
    Serial.print("Transmission: Registered ");
    Serial.print(trans_sensors_registered);
    Serial.print(" sensors out of ");
    Serial.print(TRANSMISSION_SENSOR_COUNT);
    Serial.println(" requested");
    
    // Debug: Show details of vehicle speed sensor (sensor #9)
    if (trans_sensors_registered >= 10) {
            // Serial.print("DEBUG: Vehicle speed sensor registered - pin ");
    // Serial.print(TRANSMISSION_SENSORS[9].pin);
    // Serial.print(", type ");
    // Serial.print(TRANSMISSION_SENSORS[9].type);
    // Serial.print(", msg_id 0x");
    // Serial.println(TRANSMISSION_SENSORS[9].msg_id, HEX);
    } else {
        Serial.println("WARNING: Vehicle speed sensor may not have been registered!");
    }
    #endif
    
    // Configure MCP23017 GPIO pins for transmission gear switches
    #ifdef ARDUINO
    // SKIPPING MCP23017 configuration for debugging (MCP23017 not initialized)
    // Configure pins 0-5 as inputs with pullup for gear selector switches
    // for (uint8_t pin = 0; pin <= 5; pin++) {
    //     configure_mcp23017_pin(pin, INPUT_PULLUP);
    // }
    // Serial.println("Transmission: Configured MCP23017 pins 0-5 as gear selector inputs");
    Serial.println("Transmission: SKIPPING MCP23017 configuration (not initialized)");
    #endif
    
    // Register outputs with output manager
    uint8_t registered_outputs = output_manager_register_outputs(TRANSMISSION_OUTPUTS, TRANSMISSION_OUTPUT_COUNT);
    
    #ifdef ARDUINO
    Serial.print("Transmission: Registered ");
    Serial.print(registered_outputs);
    Serial.print(" outputs out of ");
    Serial.print(TRANSMISSION_OUTPUT_COUNT);
    Serial.println(" requested");
    #endif
    
    (void)registered_outputs;  // Suppress unused variable warning
    
    // Configure external CAN bus mappings for transmission data
    configure_external_canbus_mappings();
    
    // Subscribe to transmission messages
    subscribe_to_transmission_messages();
    
    // Register transmission parameters with the parameter registry
    ParameterRegistry::register_parameter(MSG_TRANS_CURRENT_GEAR, 
                                        []() -> float { return (float)trans_state.current_gear; }, 
                                        nullptr, "Current Gear");
    ParameterRegistry::register_parameter(MSG_TRANS_DRIVE_GEAR,
                                        []() -> float { return (float)current_auto_gear; }, 
                                        nullptr, "Drive Gear");
    ParameterRegistry::register_parameter(MSG_VEHICLE_SPEED,
                                        []() -> float { 
                                            // Return actual vehicle speed from cached value
                                            return get_vehicle_speed_with_timeout();
                                        }, 
                                        nullptr, "Vehicle Speed");
    ParameterRegistry::register_parameter(MSG_TRANS_OVERRUN_STATE, 
                                        []() -> float { return (float)trans_state.overrun_state; }, 
                                        nullptr, "Overrun State");
    ParameterRegistry::register_parameter(MSG_TRANS_STATE_VALID, 
                                        []() -> float { return trans_state.valid_gear_position ? 1.0f : 0.0f; }, 
                                        nullptr, "State Valid");
    ParameterRegistry::register_parameter(MSG_TRANS_SHIFT_SOL_A, 
                                        get_shift_solenoid_a_state, nullptr, "Shift Solenoid A");
    ParameterRegistry::register_parameter(MSG_TRANS_SHIFT_SOL_B, 
                                        get_shift_solenoid_b_state, nullptr, "Shift Solenoid B");
    ParameterRegistry::register_parameter(MSG_TRANS_LOCKUP_SOL, 
                                        get_lockup_solenoid_state, nullptr, "Lockup Solenoid");
    ParameterRegistry::register_parameter(MSG_TRANS_PRESSURE_SOL, 
                                        get_pressure_solenoid_state, nullptr, "Pressure Solenoid");
    ParameterRegistry::register_parameter(MSG_TRANS_OVERRUN_SOL, 
                                        get_overrun_solenoid_state, nullptr, "Overrun Solenoid");
    
    // Register transmission fluid temperature for parameter access
    #ifdef ARDUINO
    Serial.print("Transmission: Registering MSG_TRANS_FLUID_TEMP (0x");
    Serial.print(MSG_TRANS_FLUID_TEMP, HEX);
    Serial.println(") with parameter registry...");
    #endif
    
    bool fluid_temp_registered = ParameterRegistry::register_parameter(MSG_TRANS_FLUID_TEMP,
                                        []() -> float { return trans_state.fluid_temperature; },
                                        nullptr, "Fluid Temperature");
    
    #ifdef ARDUINO
    Serial.print("Transmission: Fluid temperature parameter registration result: ");
    Serial.println(fluid_temp_registered ? "SUCCESS" : "FAILED");
    #endif
    
    // Register transmission messages for external broadcasting
    #ifdef ARDUINO
    Serial.println("Transmission: Registering messages for external broadcasting...");
    Serial.println("Transmission: Registering MSG_TRANS_CURRENT_GEAR for broadcasting...");
    #endif
    
    // Note: Vehicle speed is already registered in register_common_broadcast_messages()
    // Don't register it again here to avoid conflicts
    
    // Register transmission gear for moderate-frequency broadcasting (1Hz)
    // Register transmission current gear for broadcasting (1Hz)
    // ExternalMessageBroadcasting::register_broadcast_message(
    //     MSG_TRANS_CURRENT_GEAR, "Transmission Current Gear", 1);
    
    // Register transmission fluid temperature for low-frequency broadcasting (1Hz)
    // ExternalMessageBroadcasting::register_broadcast_message(
    //     MSG_TRANS_FLUID_TEMP, "Transmission Fluid Temperature", 1);
    // Debug output disabled to avoid serial corruption
    /*
    #ifdef ARDUINO
    Serial.println("Transmission: MSG_TRANS_FLUID_TEMP registered successfully");
    #endif
    */
    
    // Register transmission drive gear for moderate-frequency broadcasting (1Hz)
    // ExternalMessageBroadcasting::register_broadcast_message(
    //     MSG_TRANS_DRIVE_GEAR, "Transmission Drive Gear", 1);
    
    // Register vehicle speed for broadcasting (1Hz) - this was missing!
    // ExternalMessageBroadcasting::register_broadcast_message(
    //     MSG_VEHICLE_SPEED, "Vehicle Speed", 1);
    
    #ifdef ARDUINO
    Serial.print("Transmission: Registered ");
    Serial.print(trans_sensors_registered);
    Serial.println(" sensors with input manager");
    Serial.print("Transmission: Fluid temp sensor on pin A");
    Serial.println(PIN_TRANS_FLUID_TEMP - A0);
    #endif
    
    #ifdef ARDUINO
    Serial.println("Transmission: External broadcasting registration complete");
    #endif
    
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
    Serial.print(trans_sensors_registered);
    Serial.print(" sensors and ");
    Serial.print(registered_outputs);
    Serial.println(" outputs");
    Serial.print("Paddle debounce time: ");
    Serial.print(paddle_debounce_ms);
    Serial.println("ms");
    Serial.println("Race car overrun clutch control enabled");
    #endif
    
    return trans_sensors_registered;
}

// =============================================================================
// EXTERNAL CAN BUS CONFIGURATION
// =============================================================================

static void configure_external_canbus_mappings(void) {
    // Configure Haltech throttle position mapping
    // Maps Haltech CAN ID 0x360 to internal MSG_THROTTLE_POSITION
    can_mapping_t throttle_mapping = create_can_mapping(
        0x360,                      // External CAN ID (Haltech throttle position)
        MSG_THROTTLE_POSITION,      // Internal message ID
        0,                          // Start at byte 0
        2,                          // 2 bytes long
        false,                      // Little endian
        0.1f,                       // Scale factor (raw * 0.1 = percentage)
        0.0f,                       // Min value
        100.0f                      // Max value
    );
    
    // Add the mapping to the custom CAN bus manager
    if (g_custom_canbus_manager.add_mapping(throttle_mapping)) {
        #ifdef ARDUINO
        Serial.println("Transmission: Added Haltech throttle position mapping (0x360 -> MSG_THROTTLE_POSITION)");
        #endif
    } else {
        #ifdef ARDUINO
        Serial.println("Transmission: Failed to add Haltech throttle position mapping");
        #endif
    }
}

void transmission_module_update(void) {
    #ifdef ARDUINO
    static uint32_t last_debug_time = 0;
    uint32_t now = millis();
    if (now - last_debug_time >= 1000) {  // Every second
        Serial.print("Transmission module update - Current gear: ");
        Serial.print(transmission_gear_to_string(trans_state.current_gear));
        Serial.print(", Valid: ");
        Serial.print(trans_state.valid_gear_position ? "YES" : "NO");
        Serial.print(", Switches: P=");
        Serial.print(trans_state.park_switch ? "1" : "0");
        Serial.print(" R=");
        Serial.print(trans_state.reverse_switch ? "1" : "0");
        Serial.print(" N=");
        Serial.print(trans_state.neutral_switch ? "1" : "0");
        Serial.print(" D=");
        Serial.print(trans_state.drive_switch ? "1" : "0");
        Serial.print(" 2=");
        Serial.print(trans_state.second_switch ? "1" : "0");
        Serial.print(" 1=");
        Serial.println(trans_state.first_switch ? "1" : "0");
        last_debug_time = now;
    }
    #endif
    
    // Update gear position based on switch states
    update_gear_position();
    
    // Process any pending shift requests
    process_shift_requests();
    
    // Update overrun clutch control based on driving conditions (race car logic)
    update_overrun_clutch_control();
    
    // Publish current transmission state to message bus (reduced frequency)
    static uint32_t last_publish_time = 0;
    if (now - last_publish_time >= 50) {  // Publish every 50ms (20Hz) instead of every update
        publish_transmission_state();
        last_publish_time = now;
    }
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
    
    // Debug: Show voltage range of lookup table
    Serial.print("Voltage range: ");
    Serial.print(trans_temp_voltage_table[0]);
    Serial.print("V to ");
    Serial.print(trans_temp_voltage_table[TRANS_TEMP_TABLE_SIZE-1]);
    Serial.println("V");
    
    // Debug: Show a few table entries
    Serial.println("Lookup table entries:");
    for (uint8_t i = 0; i < TRANS_TEMP_TABLE_SIZE; i += 4) {  // Show every 4th entry
        Serial.print("  [");
        Serial.print(i);
        Serial.print("] ");
        Serial.print(trans_temp_voltage_table[i]);
        Serial.print("V = ");
        Serial.print(trans_temp_temp_table[i]);
        Serial.println("°C");
    }
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
    
    // Subscribe to parameter requests for transmission status (handled by parameter registry)
    // Note: Individual parameter subscriptions are no longer needed as the parameter registry
    // handles all parameter requests centrally
}

static void handle_trans_fluid_temp(const CANMessage* msg) {
    trans_state.fluid_temperature = MSG_UNPACK_FLOAT(msg);
    // Temporarily disabled to avoid serial corruption
    /*
    #ifdef ARDUINO
    static uint32_t last_temp_debug = 0;
    uint32_t now = millis();
    if (now - last_temp_debug >= 5000) {  // Every 5 seconds
            // Serial.print("DEBUG: Received transmission fluid temp: ");
    // Serial.print(trans_state.fluid_temperature);
    // Serial.println("°C");
        last_temp_debug = now;
    }
    #endif
    */
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
            // Serial.println("Upshift paddle pressed");
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
            // Serial.println("Downshift paddle pressed");
            #endif
        }
    }
}

static void handle_gear_position_switches(const CANMessage* msg) {
    // Update individual switch states based on message ID
    bool switch_active = (MSG_UNPACK_FLOAT(msg) > 0.5f);
    
    #ifndef ARDUINO
    // Debug output for testing
    printf("Debug: Received gear switch message 0x%08X = %.2f (active=%d)\n", 
           msg->id, MSG_UNPACK_FLOAT(msg), switch_active);
    #endif
    
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

// Parameter request handling is now done by the parameter registry
// This function is no longer needed as parameters are registered centrally

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
                    // Serial.print("Gear position changed to: ");
        // Serial.println(transmission_gear_to_string(trans_state.current_gear));
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
        // Serial.print("Invalid gear position - ");
        // Serial.print(active_count);
        // Serial.println(" switches active, defaulting to neutral");
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
                    // Serial.print("Executed ");
        // Serial.print(trans_state.shift_request == SHIFT_UP ? "upshift" : "downshift");
        // Serial.print(" to automatic gear ");
        // Serial.println(current_auto_gear);
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
    g_message_bus.publishFloat(MSG_TRANS_DRIVE_GEAR, (float)current_auto_gear);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_REQUEST, (float)trans_state.shift_request);
    g_message_bus.publishFloat(MSG_TRANS_STATE_VALID, trans_state.valid_gear_position ? 1.0f : 0.0f);
    g_message_bus.publishFloat(MSG_TRANS_OVERRUN_STATE, (float)trans_state.overrun_state);
    
    // Publish vehicle speed - get from sensor or default to 0.0 when stopped
    float vehicle_speed = 0.0f;  // Default for stopped vehicle
    int8_t speed_sensor_index = input_manager_find_sensor_by_msg_id(MSG_VEHICLE_SPEED);
    if (speed_sensor_index >= 0) {
        sensor_runtime_t status;
        if (input_manager_get_sensor_status(speed_sensor_index, &status)) {
            vehicle_speed = status.calibrated_value;  // Use actual speed if available
        }
    }
    g_message_bus.publishFloat(MSG_VEHICLE_SPEED, vehicle_speed);
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
            // Serial.print("Upshift executed: Drive gear ");
        // Serial.println(current_auto_gear);
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
            // Serial.print("Downshift executed: Drive gear ");
        // Serial.println(current_auto_gear);
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
    
    // Only publish if values have changed (use static variables to track last states)
    static bool last_sol_a_state = false;
    static bool last_sol_b_state = false;
    static bool last_lockup_state = false;
    
    if (sol_a_state != last_sol_a_state) {
        g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_A, sol_a_state ? 1.0f : 0.0f);
        last_sol_a_state = sol_a_state;
    }
    
    if (sol_b_state != last_sol_b_state) {
        g_message_bus.publishFloat(MSG_TRANS_SHIFT_SOL_B, sol_b_state ? 1.0f : 0.0f);
        last_sol_b_state = sol_b_state;
    }
    
    if (lockup_state != last_lockup_state) {
        g_message_bus.publishFloat(MSG_TRANS_LOCKUP_SOL, lockup_state ? 1.0f : 0.0f);
        last_lockup_state = lockup_state;
    }
                            
    #ifdef ARDUINO
            // Serial.print("Solenoids - Gear ");
        // Serial.print(gear);
        // Serial.print(": A=");
        // Serial.print(sol_a_state ? "ON" : "OFF");
        // Serial.print(", B=");
        // Serial.print(sol_b_state ? "ON" : "OFF");
        // Serial.print(", Lockup=");
        // Serial.println(lockup_state ? "ON" : "OFF");
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
    
    // Only publish if pressure value has changed
    static float last_pressure_percent = -1.0f;  // Start with invalid value to force first update
    if (fabs(pressure_percent - last_pressure_percent) > 0.001f) {
        // Publish pressure control message
        g_message_bus.publishFloat(MSG_TRANS_PRESSURE_SOL, pressure_percent);
        last_pressure_percent = pressure_percent;
                            
        #ifdef ARDUINO
        Serial.print("GEAR PRESSURE PUBLISHED: ");
        Serial.print(transmission_gear_to_string(gear));
        Serial.print(" = ");
        Serial.print(pressure_percent * 100.0f);
        Serial.println("%");
        #endif
    }
}

static void set_line_pressure(float pressure_percent) {
    // Manual line pressure control (for testing/diagnostics)
    // Clamp to safe range
    if (pressure_percent < 0.0f) pressure_percent = 0.0f;
    if (pressure_percent > 1.0f) pressure_percent = 1.0f;
    
    // Publish pressure control message
    g_message_bus.publishFloat(MSG_TRANS_PRESSURE_SOL, pressure_percent);
                            
    #ifdef ARDUINO
            // Serial.print("Line pressure manually set to: ");
        // Serial.print(pressure_percent * 100.0f);
        // Serial.println("%");
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
            // Serial.print("Overrun clutch ");
        // Serial.print(transmission_overrun_to_string(state));
        // Serial.print(" (solenoid ");
        // Serial.print(solenoid_power ? "ON-12V" : "OFF-0V");
        // Serial.println(")");
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
        // Serial.print("Overrun clutch state changed: ");
        // Serial.print(transmission_overrun_to_string(previous_state));
        // Serial.print(" → ");
        // Serial.println(transmission_overrun_to_string(desired_state));
        #else
        // Suppress unused variable warning in non-Arduino builds
        (void)previous_state;
        #endif
    }
}