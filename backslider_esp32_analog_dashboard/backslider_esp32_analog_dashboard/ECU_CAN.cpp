/*
 * ECU_CAN.cpp - CAN Bus ECU Communication Implementation
 * 
 * Handles all CAN bus communication with ECU parameters using multi-rate requests
 */

#include "ECU_CAN.h"

// Haltech Data Structure
struct HaltechData {
    bool temp_data_valid = false;
    bool pressure_data_valid = false;
    float coolant_temp = 0.0;
    float oil_pressure = 0.0;
    unsigned long last_temp_update = 0;
    unsigned long last_pressure_update = 0;
};

// Global instance of Haltech data
static HaltechData haltech_data;

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

// CAN System Variables
static bool can_initialized = false;
static CANStats stats = {0, 0, 0, 0};

// Multi-rate timing variables
static unsigned long last_speed_request = 0;
static unsigned long last_temp_request = 0;
static unsigned long last_gear_request = 0;

// Update intervals (in milliseconds)
static const unsigned long SPEED_REQUEST_INTERVAL = 100;    // 10Hz
static const unsigned long TEMP_REQUEST_INTERVAL = 1000;    // 1Hz  
static const unsigned long GEAR_REQUEST_INTERVAL = 200;     // 5Hz

// Parameter categories
static const uint32_t SPEED_PARAMS[] = {
    PARAM_VEHICLE_SPEED
};

static const uint32_t TEMP_PARAMS[] = {
    PARAM_FLUID_TEMP
    // Add other temperature parameters here if needed
};

static const uint32_t GEAR_PARAMS[] = {
    PARAM_CURRENT_GEAR,
    PARAM_DRIVE_GEAR,
    PARAM_SHIFT_SOL_A,
    PARAM_SHIFT_SOL_B,
    PARAM_OVERRUN_SOL,
    PARAM_PRESSURE_SOL,
    PARAM_LOCKUP_SOL
};

static const int NUM_SPEED_PARAMS = sizeof(SPEED_PARAMS) / sizeof(uint32_t);
static const int NUM_TEMP_PARAMS = sizeof(TEMP_PARAMS) / sizeof(uint32_t);
static const int NUM_GEAR_PARAMS = sizeof(GEAR_PARAMS) / sizeof(uint32_t);

// Current indices for each category
static int current_speed_index = 0;
static int current_temp_index = 0;
static int current_gear_index = 0;

// Private Function Declarations
static void handleCANMessages();
static bool processMessage(const twai_message_t& message);
static bool processCustomECUMessage(const twai_message_t& message);
static bool processHaltechMessage(const twai_message_t& message);
static ECUParameter* findParameterByID(uint32_t can_id);
static void sendParameterRequest(uint32_t param_id, const char* category);

// Public Functions Implementation

bool CAN_Initialize() {
    // Simple TWAI configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Serial.printf("CAN: Initializing TX=GPIO%d, RX=GPIO%d, 500kbps\n", CAN_TX_PIN, CAN_RX_PIN);
    
    // Install driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        // Serial.println("CAN: Failed to install TWAI driver");
        return false;
    }
    
    // Start driver
    if (twai_start() != ESP_OK) {
        // Serial.println("CAN: Failed to start TWAI driver");
        twai_driver_uninstall();
        return false;
    }
    
    can_initialized = true;
    stats.start_time = millis();
    // Serial.println("CAN: ✅ Initialized successfully");
    
    return true;
}

void CAN_Update() {
    if (!can_initialized) return;
    
    // Handle incoming messages (check frequently)
    handleCANMessages();
    
    unsigned long current_time = millis();
    
    // Speed parameters - 10Hz (every 100ms)
    if (current_time - last_speed_request >= SPEED_REQUEST_INTERVAL) {
        sendParameterRequest(SPEED_PARAMS[current_speed_index], "SPEED");
        current_speed_index = (current_speed_index + 1) % NUM_SPEED_PARAMS;
        last_speed_request = current_time;
    }
    
    // Temperature parameters - 1Hz (every 1000ms)  
    if (current_time - last_temp_request >= TEMP_REQUEST_INTERVAL) {
        sendParameterRequest(TEMP_PARAMS[current_temp_index], "TEMP");
        current_temp_index = (current_temp_index + 1) % NUM_TEMP_PARAMS;
        last_temp_request = current_time;
    }
    
    // Gear/Solenoid parameters - 5Hz (every 200ms)
    if (current_time - last_gear_request >= GEAR_REQUEST_INTERVAL) {
        sendParameterRequest(GEAR_PARAMS[current_gear_index], "GEAR");
        current_gear_index = (current_gear_index + 1) % NUM_GEAR_PARAMS;
        last_gear_request = current_time;
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
    
    // Print Haltech data
    Serial.println("Haltech Data:");
    if (haltech_data.temp_data_valid) {
        unsigned long temp_age = (millis() - haltech_data.last_temp_update) / 1000;
        Serial.printf("  Coolant Temp: %.1f°C (%lu sec ago)\n", 
                      haltech_data.coolant_temp, temp_age);
    } else {
        Serial.println("  Coolant Temp: No data");
    }
    
    if (haltech_data.pressure_data_valid) {
        unsigned long pressure_age = (millis() - haltech_data.last_pressure_update) / 1000;
        Serial.printf("  Oil Pressure: %.1f kPa (%lu sec ago)\n", 
                      haltech_data.oil_pressure, pressure_age);
    } else {
        Serial.println("  Oil Pressure: No data");
    }
    
    Serial.println("===================\n");
}

void CAN_PrintDetailedDebug() {
    Serial.println("\n🔍 === DETAILED CAN DEBUG ===");
    Serial.printf("Current millis(): %lu\n", millis());
    Serial.printf("CAN initialized: %s\n", can_initialized ? "YES" : "NO");
    
    Serial.printf("Multi-rate timings:\n");
    Serial.printf("  Speed (10Hz): last=%lu, next in %lu ms\n", 
                  last_speed_request, SPEED_REQUEST_INTERVAL - (millis() - last_speed_request));
    Serial.printf("  Temp (1Hz): last=%lu, next in %lu ms\n", 
                  last_temp_request, TEMP_REQUEST_INTERVAL - (millis() - last_temp_request));
    Serial.printf("  Gear (5Hz): last=%lu, next in %lu ms\n", 
                  last_gear_request, GEAR_REQUEST_INTERVAL - (millis() - last_gear_request));
    
    Serial.printf("Parameter indices: Speed=%d/%d, Temp=%d/%d, Gear=%d/%d\n",
                  current_speed_index, NUM_SPEED_PARAMS,
                  current_temp_index, NUM_TEMP_PARAMS,
                  current_gear_index, NUM_GEAR_PARAMS);
    
    Serial.println("\nParameter Details:");
    for (int i = 0; i < NUM_PARAMETERS; i++) {
        ECUParameter& param = ecu_parameters[i];
        if (param.has_data) {
            unsigned long age_ms = millis() - param.last_update;
            unsigned long age_sec = age_ms / 1000;
            Serial.printf("  %s: %.2f %s\n", param.name, param.last_value, param.unit);
            Serial.printf("    Last update: %lu (%lu ms ago = %lu sec)\n", 
                          param.last_update, age_ms, age_sec);
        } else {
            Serial.printf("  %s: NO DATA\n", param.name);
        }
    }
    
    Serial.println("================================\n");
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

// Haltech Parameter Functions
float CAN_GetCoolantTemperature() { 
    return haltech_data.temp_data_valid ? haltech_data.coolant_temp : 0.0; 
}

float CAN_GetOilPressure() { 
    return haltech_data.pressure_data_valid ? haltech_data.oil_pressure : 0.0; 
}

// Status Functions
bool CAN_IsInitialized() { return can_initialized; }
CANStats CAN_GetStats() { return stats; }

// Check if Haltech data is fresh
bool CAN_IsHaltechTempFresh(unsigned long max_age_seconds) {
    if (!haltech_data.temp_data_valid) return false;
    unsigned long age = (millis() - haltech_data.last_temp_update) / 1000;
    return age <= max_age_seconds;
}

bool CAN_IsHaltechPressureFresh(unsigned long max_age_seconds) {
    if (!haltech_data.pressure_data_valid) return false;
    unsigned long age = (millis() - haltech_data.last_pressure_update) / 1000;
    return age <= max_age_seconds;
}

// Private Functions Implementation

static void sendParameterRequest(uint32_t param_id, const char* category) {
    if (!can_initialized) return;
    
    // Create simple message data
    uint8_t data[8] = {READ_REQUEST, 0x00, 0x00, 0x00, 0x00, 1, 0, 0};
    
    // Create CAN message
    twai_message_t message;
    message.identifier = param_id;
    message.extd = 1;          // Extended frame
    message.rtr = 0;           // Data frame
    message.data_length_code = 8;
    memcpy(message.data, data, 8);
    
    // Send message
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(50)); // Short timeout
    
    if (result == ESP_OK) {
        stats.total_requests++;
        const char* param_name = CAN_GetParameterName(param_id);
        //Serial.printf("CAN: 📤 %s: %s\n", category, param_name);
    } else {
        //Serial.printf("CAN: ❌ %s send failed: %s\n", category, esp_err_to_name(result));
        
        // Check for bus-off and recover
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
            //Serial.println("CAN: Bus-off detected, attempting recovery...");
            twai_initiate_recovery();
        }
    }
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
    // Route to appropriate processor based on frame type
    if (message.extd) {
        // Extended frames - Custom ECU
        return processCustomECUMessage(message);
    } else {
        // Standard frames - Check if it's Haltech
        if (message.identifier == HALTECH_ENGINE_DATA_1 || 
            message.identifier == HALTECH_ENGINE_DATA_2 ||
            message.identifier == HALTECH_TEMPERATURE_DATA) {
            return processHaltechMessage(message);
        } else {
            // Other standard frames - try custom ECU with truncated ID matching
            return processCustomECUMessage(message);
        }
    }
}

static bool processCustomECUMessage(const twai_message_t& message) {
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
        // Serial.printf("CAN: Unknown Custom ECU ID: 0x%08X\n", message.identifier);
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
        // Extract float value (little endian)
        float value;
        memcpy(&value, &message.data[1], 4);
        
        // ALWAYS update timestamp and validity when we receive a response
        // This ensures data is marked as "fresh" even if the value didn't change
        param->last_update = millis();  // ✅ ALWAYS update timestamp
        param->has_data = true;         // ✅ ALWAYS mark as having data
        
        // Check if value actually changed before logging
        bool value_changed = (param->last_value != value);
        param->last_value = value;      // ✅ ALWAYS update value
        
        // Only log if value changed to reduce spam
        if (value_changed) {
            //Serial.printf("CAN: ✅ %s: %.2f %s (VALUE CHANGED)\n", param->name, value, param->unit);
        }
        // Uncomment this line to see all responses (even unchanged values)
        // else { Serial.printf("CAN: 🔄 %s: %.2f %s (unchanged)\n", param->name, value, param->unit); }
        
        return true;
    }
    
    return false;
}

static bool processHaltechMessage(const twai_message_t& message) {
    if (message.data_length_code != 8) {
        Serial.printf("CAN: Invalid Haltech DLC: %d\n", message.data_length_code);
        return false;
    }
    
    bool updated = false;
    
    switch (message.identifier) {
        case HALTECH_TEMPERATURE_DATA: // 0x3E0
            {
                // Coolant Temperature: bytes 0-1, big-endian, Kelvin
                uint16_t raw_coolant = (message.data[0] << 8) | message.data[1];
                float coolant_kelvin = raw_coolant / 10.0;
                float new_coolant_temp = coolant_kelvin - 273.15; // Convert to Celsius
                
                // Check if value changed
                bool temp_changed = (haltech_data.coolant_temp != new_coolant_temp);
                
                // ALWAYS update timestamp and validity
                haltech_data.coolant_temp = new_coolant_temp;
                haltech_data.last_temp_update = millis();  // ✅ ALWAYS update
                haltech_data.temp_data_valid = true;       // ✅ ALWAYS mark valid
                
                // Only log if value changed - COMMENTED OUT FOR PERFORMANCE
                /*
                if (temp_changed) {
                    Serial.printf("CAN: ✅ Haltech Coolant Temp: %.1f°C\n", haltech_data.coolant_temp);
                }
                */
                updated = true;
            }
            break;
            
        case HALTECH_ENGINE_DATA_2: // 0x361
            {
                // Oil Pressure: bytes 2-3, big-endian, absolute kPa
                uint16_t raw_oil = (message.data[2] << 8) | message.data[3];
                float oil_absolute = raw_oil / 10.0;
                float new_oil_pressure = oil_absolute - 101.3; // Convert to gauge pressure
                
                // Check if value changed
                bool pressure_changed = (haltech_data.oil_pressure != new_oil_pressure);
                
                // ALWAYS update timestamp and validity
                haltech_data.oil_pressure = new_oil_pressure;
                haltech_data.last_pressure_update = millis();  // ✅ ALWAYS update
                haltech_data.pressure_data_valid = true;       // ✅ ALWAYS mark valid
                
                // Only log if value changed - COMMENTED OUT FOR PERFORMANCE
                /*
                if (pressure_changed) {
                    Serial.printf("CAN: ✅ Haltech Oil Pressure: %.1f kPa\n", haltech_data.oil_pressure);
                }
                */
                updated = true;
            }
            break;
            
        case HALTECH_ENGINE_DATA_1: // 0x360
            // We don't need anything from this message currently
            // (contains RPM, MAP, TPS, Coolant Pressure)
            break;
            
        default:
            return false;
    }
    
    return updated;
}

static ECUParameter* findParameterByID(uint32_t can_id) {
    for (int i = 0; i < NUM_PARAMETERS; i++) {
        if (ecu_parameters[i].can_id == can_id) {
            return &ecu_parameters[i];
        }
    }
    return nullptr;
}