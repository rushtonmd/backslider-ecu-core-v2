// transmission_module.cpp
// Implementation of transmission control module

#ifndef ARDUINO
// For desktop testing, include mock Arduino before anything else
#include "../tests/mock_arduino.h"
#endif

#include "transmission_module.h"
#include "input_manager.h"
#include "msg_bus.h"
#include "thermistor_table_generator.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// =============================================================================
// PRIVATE DATA
// =============================================================================

// Static lookup tables for transmission fluid temperature sensor
static float trans_temp_voltage_table[TRANS_TEMP_TABLE_SIZE];
static float trans_temp_temp_table[TRANS_TEMP_TABLE_SIZE];

// Transmission state
static transmission_state_t trans_state;

// Configuration
static uint16_t paddle_debounce_ms = PADDLE_DEBOUNCE_MS;

// Statistics
static uint32_t shift_count = 0;
static uint32_t invalid_gear_count = 0;

// =============================================================================
// SENSOR DEFINITIONS
// =============================================================================

// Initialize sensor array using explicit C++ syntax
static sensor_definition_t TRANSMISSION_SENSORS[9];

static void init_transmission_sensor_definitions() {
    // Transmission fluid temperature sensor (thermistor)
    TRANSMISSION_SENSORS[0].pin = PIN_TRANS_FLUID_TEMP;
    TRANSMISSION_SENSORS[0].type = SENSOR_THERMISTOR;
    TRANSMISSION_SENSORS[0].config.thermistor.pullup_ohms = TRANS_TEMP_PULLUP_OHMS;
    TRANSMISSION_SENSORS[0].config.thermistor.voltage_table = trans_temp_voltage_table;
    TRANSMISSION_SENSORS[0].config.thermistor.temp_table = trans_temp_temp_table;
    TRANSMISSION_SENSORS[0].config.thermistor.table_size = TRANS_TEMP_TABLE_SIZE;
    TRANSMISSION_SENSORS[0].msg_id = MSG_TRANS_FLUID_TEMP;
    TRANSMISSION_SENSORS[0].update_interval_us = 100000;
    TRANSMISSION_SENSORS[0].filter_strength = 128;
    TRANSMISSION_SENSORS[0].name = "Trans Fluid Temp";
    
    // Paddle upshift
    TRANSMISSION_SENSORS[1].pin = PIN_PADDLE_UPSHIFT;
    TRANSMISSION_SENSORS[1].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[1].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[1].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[1].msg_id = MSG_PADDLE_UPSHIFT;
    TRANSMISSION_SENSORS[1].update_interval_us = 20000;
    TRANSMISSION_SENSORS[1].filter_strength = 0;
    TRANSMISSION_SENSORS[1].name = "Paddle Upshift";
    
    // Paddle downshift
    TRANSMISSION_SENSORS[2].pin = PIN_PADDLE_DOWNSHIFT;
    TRANSMISSION_SENSORS[2].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[2].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[2].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[2].msg_id = MSG_PADDLE_DOWNSHIFT;
    TRANSMISSION_SENSORS[2].update_interval_us = 20000;
    TRANSMISSION_SENSORS[2].filter_strength = 0;
    TRANSMISSION_SENSORS[2].name = "Paddle Downshift";
    
    // Park switch
    TRANSMISSION_SENSORS[3].pin = PIN_TRANS_PARK;
    TRANSMISSION_SENSORS[3].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[3].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[3].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[3].msg_id = MSG_TRANS_PARK_SWITCH;
    TRANSMISSION_SENSORS[3].update_interval_us = 50000;
    TRANSMISSION_SENSORS[3].filter_strength = 0;
    TRANSMISSION_SENSORS[3].name = "Trans Park Switch";
    
    // Reverse switch
    TRANSMISSION_SENSORS[4].pin = PIN_TRANS_REVERSE;
    TRANSMISSION_SENSORS[4].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[4].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[4].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[4].msg_id = MSG_TRANS_REVERSE_SWITCH;
    TRANSMISSION_SENSORS[4].update_interval_us = 50000;
    TRANSMISSION_SENSORS[4].filter_strength = 0;
    TRANSMISSION_SENSORS[4].name = "Trans Reverse Switch";
    
    // Neutral switch
    TRANSMISSION_SENSORS[5].pin = PIN_TRANS_NEUTRAL;
    TRANSMISSION_SENSORS[5].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[5].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[5].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[5].msg_id = MSG_TRANS_NEUTRAL_SWITCH;
    TRANSMISSION_SENSORS[5].update_interval_us = 50000;
    TRANSMISSION_SENSORS[5].filter_strength = 0;
    TRANSMISSION_SENSORS[5].name = "Trans Neutral Switch";
    
    // Drive switch
    TRANSMISSION_SENSORS[6].pin = PIN_TRANS_DRIVE;
    TRANSMISSION_SENSORS[6].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[6].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[6].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[6].msg_id = MSG_TRANS_DRIVE_SWITCH;
    TRANSMISSION_SENSORS[6].update_interval_us = 50000;
    TRANSMISSION_SENSORS[6].filter_strength = 0;
    TRANSMISSION_SENSORS[6].name = "Trans Drive Switch";
    
    // Second switch
    TRANSMISSION_SENSORS[7].pin = PIN_TRANS_SECOND;
    TRANSMISSION_SENSORS[7].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[7].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[7].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[7].msg_id = MSG_TRANS_SECOND_SWITCH;
    TRANSMISSION_SENSORS[7].update_interval_us = 50000;
    TRANSMISSION_SENSORS[7].filter_strength = 0;
    TRANSMISSION_SENSORS[7].name = "Trans Second Switch";
    
    // First switch
    TRANSMISSION_SENSORS[8].pin = PIN_TRANS_FIRST;
    TRANSMISSION_SENSORS[8].type = SENSOR_DIGITAL_PULLUP;
    TRANSMISSION_SENSORS[8].config.digital.use_pullup = 1;
    TRANSMISSION_SENSORS[8].config.digital.invert_logic = 1;
    TRANSMISSION_SENSORS[8].msg_id = MSG_TRANS_FIRST_SWITCH;
    TRANSMISSION_SENSORS[8].update_interval_us = 50000;
    TRANSMISSION_SENSORS[8].filter_strength = 0;
    TRANSMISSION_SENSORS[8].name = "Trans First Switch";
}

// Calculate sensor count at compile time
static const uint8_t TRANSMISSION_SENSOR_COUNT = 9;  // 1 thermistor + 2 paddles + 6 gear switches

// =============================================================================
// PRIVATE FUNCTION DECLARATIONS
// =============================================================================

static void init_transmission_temp_tables(void);
static void subscribe_to_transmission_messages(void);
static void handle_trans_fluid_temp(const CANMessage* msg);
static void handle_paddle_upshift(const CANMessage* msg);
static void handle_paddle_downshift(const CANMessage* msg);
static void handle_gear_position_switches(const CANMessage* msg);
static void update_gear_position(void);
static void process_shift_requests(void);
static void publish_transmission_state(void);

// =============================================================================
// PUBLIC FUNCTIONS
// =============================================================================

uint8_t transmission_module_init(void) {
    #ifdef ARDUINO
    Serial.println("Initializing transmission module...");
    #endif
    
    // Initialize thermistor lookup tables
    init_transmission_temp_tables();
    
    // Initialize sensor definitions
    init_transmission_sensor_definitions();
    
    // Register all transmission sensors with the input manager
    uint8_t registered = input_manager_register_sensors(TRANSMISSION_SENSORS, TRANSMISSION_SENSOR_COUNT);
    
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
    trans_state.park_switch = false;
    trans_state.reverse_switch = false;
    trans_state.neutral_switch = false;
    trans_state.drive_switch = false;
    trans_state.second_switch = false;
    trans_state.first_switch = false;
    
    // Reset statistics
    shift_count = 0;
    invalid_gear_count = 0;
    
    #ifdef ARDUINO
    Serial.print("Transmission module initialized with ");
    Serial.print(registered);
    Serial.println(" sensors");
    Serial.print("Paddle debounce time: ");
    Serial.print(paddle_debounce_ms);
    Serial.println("ms");
    #endif
    
    return registered;
}

void transmission_module_update(void) {
    // Update gear position based on switch states
    update_gear_position();
    
    // Process any pending shift requests
    process_shift_requests();
    
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

void transmission_reset_statistics(void) {
    shift_count = 0;
    invalid_gear_count = 0;
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
}

static void handle_trans_fluid_temp(const CANMessage* msg) {
    trans_state.fluid_temperature = MSG_UNPACK_FLOAT(msg);
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

static void update_gear_position(void) {
    // Count active switches
    uint8_t active_count = trans_state.park_switch + trans_state.reverse_switch + 
                          trans_state.neutral_switch + trans_state.drive_switch + 
                          trans_state.second_switch + trans_state.first_switch;
    
    if (active_count == 1) {
        // Valid state - exactly one switch active
        trans_state.valid_gear_position = true;
        
        if (trans_state.park_switch) trans_state.current_gear = GEAR_PARK;
        else if (trans_state.reverse_switch) trans_state.current_gear = GEAR_REVERSE;
        else if (trans_state.neutral_switch) trans_state.current_gear = GEAR_NEUTRAL;
        else if (trans_state.drive_switch) trans_state.current_gear = GEAR_DRIVE;
        else if (trans_state.second_switch) trans_state.current_gear = GEAR_SECOND;
        else if (trans_state.first_switch) trans_state.current_gear = GEAR_FIRST;
    } else {
        // Invalid state - multiple switches active or no switches active
        trans_state.valid_gear_position = false;
        trans_state.current_gear = GEAR_NEUTRAL;  // Default to neutral for safety
        invalid_gear_count++;
        
        #ifdef ARDUINO
        Serial.print("Invalid gear position - ");
        Serial.print(active_count);
        Serial.println(" switches active, defaulting to neutral");
        #endif
    }
}

static void process_shift_requests(void) {
    // This is where you would add logic to actually control solenoids
    // For now, just acknowledge the shift request
    
    if (trans_state.shift_request != SHIFT_NONE) {
        // In a real implementation, you would:
        // 1. Check if shifting is safe (gear position, RPM, etc.)
        // 2. Control transmission solenoids
        // 3. Monitor shift completion
        
        // For now, just acknowledge the shift request
        #ifdef ARDUINO
        Serial.print("Processing ");
        Serial.print(trans_state.shift_request == SHIFT_UP ? "upshift" : "downshift");
        Serial.print(" request in gear ");
        Serial.println(transmission_gear_to_string(trans_state.current_gear));
        #endif
    }
}

static void publish_transmission_state(void) {
    // Publish combined transmission state messages
    g_message_bus.publishFloat(MSG_TRANS_CURRENT_GEAR, (float)trans_state.current_gear, false);
    g_message_bus.publishFloat(MSG_TRANS_SHIFT_REQUEST, (float)trans_state.shift_request, false);
    g_message_bus.publishFloat(MSG_TRANS_STATE_VALID, trans_state.valid_gear_position ? 1.0f : 0.0f, false);
}