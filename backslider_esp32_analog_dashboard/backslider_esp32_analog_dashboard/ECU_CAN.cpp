/*
 * ECU_CAN.cpp - CAN Bus ECU Communication Implementation
 * 
 * Handles all CAN bus communication with ECU parameters
 */

#include "ECU_CAN.h"

// ECU Parameters Array
static ECUParameter ecu_parameters[] = {
    {PARAM_FLUID_TEMP,    "Fluid Temperature", "°C",  0.0, 0, false},
    {PARAM_CURRENT_GEAR,  "Current Gear",      "",    0.0, 0, false},
    {PARAM_DRIVE_GEAR,    "Drive Gear",        "",    0.0, 0, false},
    {PARAM_VEHICLE_SPEED, "Vehicle Speed",     "kph", 0.0, 0, false},
    {PARAM_SHIFT_SOL_A,   "Shift Solenoid A",  "",    0.0, 0, false},
    {PARAM_SHIFT_SOL_B,   "Shift Solenoid B",  "",    0.0, 0, false},
    {PARAM_OVERRUN_SOL,   "Overrun Solenoid",  "",    0.0, 0, false},
    {PARAM_PRESSURE_SOL,  "Pressure Solenoid", "%",   0.0, 0, false},
    {PARAM_LOCKUP_SOL,    "Lockup Solenoid",   "",    0.0, 0, false}
};

static const int NUM_PARAMETERS = sizeof(ecu_parameters) / sizeof(ECUParameter);

// Private Variables
static bool can_initialized = false;
static unsigned long last_request_time = 0;
static const unsigned long REQUEST_INTERVAL = 2000; // 2 seconds
static int current_parameter_index = 0;
static CANStats stats = {0, 0, 0, 0};

// Private Function Declarations
static void sendNextParameterRequest();
static void handleCANMessages();
static bool processMessage(const twai_message_t& message);
static ECUParameter* findParameterByID(uint32_t can_id);

// Public Functions Implementation

bool CAN_Initialize() {
    // Simple TWAI configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    Serial.printf("CAN: Initializing TX=GPIO%d, RX=GPIO%d, 500kbps\n", CAN_TX_PIN, CAN_RX_PIN);
    
    // Install driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("CAN: Failed to install TWAI driver");
        return false;
    }
    
    // Start driver
    if (twai_start() != ESP_OK) {
        Serial.println("CAN: Failed to start TWAI driver");
        twai_driver_uninstall();
        return false;
    }
    
    can_initialized = true;
    stats.start_time = millis();
    Serial.println("CAN: ✅ Initialized successfully");
    
    return true;
}

void CAN_Update() {
    if (!can_initialized) return;
    
    // Handle incoming messages
    handleCANMessages();
    
    // Send requests periodically
    if (millis() - last_request_time >= REQUEST_INTERVAL) {
        sendNextParameterRequest();
        last_request_time = millis();
    }
}

void CAN_PrintStatus() {
    if (!can_initialized) {
        Serial.println("CAN: Not initialized");
        return;
    }
    
    Serial.println("\n📊 === CAN Status ===");
    Serial.printf("Requests: %lu, Responses: %lu, Success: %lu\n", 
                  stats.total_requests, stats.total_responses, stats.successful_responses);
    
    if (stats.total_requests > 0) {
        float success_rate = (float)stats.successful_responses / stats.total_requests * 100.0;
        Serial.printf("Success rate: %.1f%%\n", success_rate);
    }
    
    Serial.println("Parameters:");
    for (int i = 0; i < NUM_PARAMETERS; i++) {
        ECUParameter& param = ecu_parameters[i];
        if (param.has_data) {
            unsigned long age = (millis() - param.last_update) / 1000;
            Serial.printf("  %s: %.2f %s (%lu sec ago)\n", 
                          param.name, param.last_value, param.unit, age);
        } else {
            Serial.printf("  %s: No data\n", param.name);
        }
    }
    Serial.println("===================\n");
}

// Parameter Access Functions
float CAN_GetParameterValue(uint32_t can_id) {
    ECUParameter* param = findParameterByID(can_id);
    return (param && param->has_data) ? param->last_value : 0.0;
}

bool CAN_IsParameterFresh(uint32_t can_id, unsigned long max_age_seconds) {
    ECUParameter* param = findParameterByID(can_id);
    if (!param || !param->has_data) return false;
    
    unsigned long age = (millis() - param->last_update) / 1000;
    return age <= max_age_seconds;
}

const char* CAN_GetParameterName(uint32_t can_id) {
    ECUParameter* param = findParameterByID(can_id);
    return param ? param->name : "Unknown";
}

const char* CAN_GetParameterUnit(uint32_t can_id) {
    ECUParameter* param = findParameterByID(can_id);
    return param ? param->unit : "";
}

unsigned long CAN_GetParameterAge(uint32_t can_id) {
    ECUParameter* param = findParameterByID(can_id);
    if (!param || !param->has_data) return ULONG_MAX;
    
    return (millis() - param->last_update) / 1000;
}

// Convenience Functions
float CAN_GetFluidTemperature() { return CAN_GetParameterValue(PARAM_FLUID_TEMP); }
float CAN_GetCurrentGear() { return CAN_GetParameterValue(PARAM_CURRENT_GEAR); }
float CAN_GetDriveGear() { return CAN_GetParameterValue(PARAM_DRIVE_GEAR); }
float CAN_GetVehicleSpeed() { return CAN_GetParameterValue(PARAM_VEHICLE_SPEED); }
float CAN_GetShiftSolenoidA() { return CAN_GetParameterValue(PARAM_SHIFT_SOL_A); }
float CAN_GetShiftSolenoidB() { return CAN_GetParameterValue(PARAM_SHIFT_SOL_B); }
float CAN_GetOverrunSolenoid() { return CAN_GetParameterValue(PARAM_OVERRUN_SOL); }
float CAN_GetPressureSolenoid() { return CAN_GetParameterValue(PARAM_PRESSURE_SOL); }
float CAN_GetLockupSolenoid() { return CAN_GetParameterValue(PARAM_LOCKUP_SOL); }

// Status Functions
bool CAN_IsInitialized() { return can_initialized; }
CANStats CAN_GetStats() { return stats; }

// Private Functions Implementation

static void sendNextParameterRequest() {
    if (!can_initialized) return;
    
    // Get current parameter
    ECUParameter& param = ecu_parameters[current_parameter_index];
    
    // Create simple message data
    uint8_t data[8] = {READ_REQUEST, 0x00, 0x00, 0x00, 0x00, 1, 0, 0};
    
    // Create CAN message
    twai_message_t message;
    message.identifier = param.can_id;
    message.extd = 1;          // Extended frame
    message.rtr = 0;           // Data frame
    message.data_length_code = 8;
    memcpy(message.data, data, 8);
    
    // Send message
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
    
    if (result == ESP_OK) {
        stats.total_requests++;
        Serial.printf("CAN: 📤 Requested %s\n", param.name);
    } else {
        Serial.printf("CAN: ❌ Send failed for %s: %s\n", param.name, esp_err_to_name(result));
        
        // Check for bus-off and recover
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
            Serial.println("CAN: Bus-off detected, attempting recovery...");
            twai_initiate_recovery();
        }
    }
    
    // Move to next parameter
    current_parameter_index = (current_parameter_index + 1) % NUM_PARAMETERS;
}

static void handleCANMessages() {
    twai_message_t message;
    
    // Check for messages
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(0));
    
    if (result == ESP_OK) {
        stats.total_responses++;
        
        // Process message
        if (processMessage(message)) {
            stats.successful_responses++;
        }
    }
}

static bool processMessage(const twai_message_t& message) {
    // Find matching parameter by exact ID first
    ECUParameter* param = findParameterByID(message.identifier);
    
    // If not found and it's a standard frame, try matching truncated extended IDs
    if (!param && !message.extd) {
        for (int i = 0; i < NUM_PARAMETERS; i++) {
            uint32_t extended_id = ecu_parameters[i].can_id;
            uint32_t truncated = extended_id & 0x7FF; // Lower 11 bits
            
            if (truncated == message.identifier) {
                param = &ecu_parameters[i];
                Serial.printf("CAN: ✅ Matched truncated ID 0x%03X -> %s\n", 
                              message.identifier, param->name);
                break;
            }
        }
    }
    
    if (!param) {
        // Uncomment for debugging unknown IDs
        // Serial.printf("CAN: Unknown ID: 0x%08X\n", message.identifier);
        return false;
    }
    
    if (message.data_length_code != 8) {
        Serial.printf("CAN: Invalid DLC for %s: %d\n", param->name, message.data_length_code);
        return false;
    }
    
    uint8_t operation = message.data[0];
    
    if (operation == READ_REQUEST) {
        // Echo of our own request - ignore silently
        return false;
    } else if (operation == READ_RESPONSE) {
        // Extract float value
        float value;
        memcpy(&value, &message.data[1], 4);
        
        // Update parameter
        param->last_value = value;
        param->last_update = millis();
        param->has_data = true;
        
        Serial.printf("CAN: ✅ %s: %.2f %s\n", param->name, value, param->unit);
        return true;
    }
    
    return false;
}

static ECUParameter* findParameterByID(uint32_t can_id) {
    for (int i = 0; i < NUM_PARAMETERS; i++) {
        if (ecu_parameters[i].can_id == can_id) {
            return &ecu_parameters[i];
        }
    }
    return nullptr;
}