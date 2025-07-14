// obdii_handler.cpp
// Implementation of OBD-II protocol handler

#include "obdii_handler.h"

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

OBDIIHandler::OBDIIHandler(ExternalCanBusCache* cache) :
    cache(cache),
    initialized(false),
    last_request_time(0),
    supported_modes(DEFAULT_SUPPORTED_MODES),
    supported_pids_01_20(DEFAULT_SUPPORTED_PIDS_01_20),
    supported_pids_21_40(DEFAULT_SUPPORTED_PIDS_21_40),
    supported_pids_41_60(DEFAULT_SUPPORTED_PIDS_41_60)
{
    // Initialize statistics
    reset_statistics();
    
    // Initialize custom PID handlers array
    for (int i = 0; i < 256; i++) {
        custom_pid_handlers[i] = nullptr;
    }
    
    // Initialize last request
    memset(&last_request, 0, sizeof(last_request));
}

OBDIIHandler::~OBDIIHandler() {
    shutdown();
}

bool OBDIIHandler::init() {
    if (initialized) {
        debug_print("OBDIIHandler: Already initialized");
        return true;
    }
    
    if (cache == nullptr) {
        debug_print("OBDIIHandler: Cache reference is null");
        return false;
    }
    
    // Reset statistics
    reset_statistics();
    
    initialized = true;
    debug_print("OBDIIHandler: Initialization complete");
    
    return true;
}

void OBDIIHandler::shutdown() {
    if (!initialized) {
        return;
    }
    
    // Clear custom handlers
    for (int i = 0; i < 256; i++) {
        custom_pid_handlers[i] = nullptr;
    }
    
    initialized = false;
    debug_print("OBDIIHandler: Shutdown complete");
}

// ============================================================================
// REQUEST PROCESSING
// ============================================================================

bool OBDIIHandler::process_request(const CAN_message_t& msg) {
    if (!initialized) {
        return false;
    }
    
    if (!is_obdii_request(msg)) {
        return false;
    }
    
    stats.requests_received++;
    last_request_time = millis();
    
    // Parse the request
    obdii_request_t request;
    if (!parse_request_message(msg, request)) {
        stats.malformed_requests++;
        debug_print("OBDIIHandler: Failed to parse request");
        return false;
    }
    
    // Store for debugging
    last_request = request;
    debug_print_request(request);
    
    // Validate request
    if (!validate_request(request)) {
        stats.malformed_requests++;
        debug_print("OBDIIHandler: Invalid request");
        return false;
    }
    
    // Generate and send response
    CAN_message_t response_msg;
    if (generate_response(request, response_msg)) {
        stats.responses_sent++;
        return true;
    } else {
        stats.negative_responses++;
        return false;
    }
}

bool OBDIIHandler::is_obdii_request(const CAN_message_t& msg) {
    return (msg.id == OBDII_REQUEST_ID && msg.len >= 2);
}

bool OBDIIHandler::parse_request_message(const CAN_message_t& msg, obdii_request_t& request) {
    if (msg.len < 2) {
        return false;
    }
    
    // OBD-II request format: [length] [mode] [pid] [additional_data...]
    uint8_t length = msg.buf[0];
    if (length < 1 || length > 7) {
        return false;
    }
    
    request.mode = msg.buf[1];
    request.pid = (msg.len >= 3) ? msg.buf[2] : 0;
    request.data_len = (length > 2) ? (length - 2) : 0;
    request.timestamp = millis();
    request.source_id = msg.id;
    
    // Copy additional data if present
    if (request.data_len > 0 && request.data_len <= 5) {
        memcpy(request.data, &msg.buf[3], request.data_len);
    }
    
    return true;
}

bool OBDIIHandler::validate_request(const obdii_request_t& request) {
    // Check if mode is supported
    if (!is_mode_supported(request.mode)) {
        return false;
    }
    
    // Mode-specific validation
    switch (request.mode) {
        case OBDII_MODE_CURRENT_DATA:
            // Mode 01 should have a PID
            return true;  // PID validation happens in handler
            
        case OBDII_MODE_DIAGNOSTIC_CODES:
        case OBDII_MODE_CLEAR_CODES:
            // These modes don't require a PID
            return true;
            
        case OBDII_MODE_VEHICLE_INFO:
            // Mode 09 should have a PID
            return true;
            
        default:
            return false;
    }
}

// ============================================================================
// RESPONSE GENERATION
// ============================================================================

bool OBDIIHandler::generate_response(const obdii_request_t& request, CAN_message_t& response_msg) {
    obdii_response_t response = {};
    response.mode = request.mode + OBDII_POSITIVE_RESPONSE;
    response.pid = request.pid;
    response.is_valid = false;
    
    // Route to appropriate handler based on mode
    switch (request.mode) {
        case OBDII_MODE_CURRENT_DATA:
            stats.mode01_requests++;
            response.is_valid = handle_mode01_request(request, response);
            break;
            
        case OBDII_MODE_DIAGNOSTIC_CODES:
            response.is_valid = handle_mode03_request(request, response);
            break;
            
        case OBDII_MODE_VEHICLE_INFO:
            response.is_valid = handle_mode09_request(request, response);
            break;
            
        default:
            // Unsupported mode
            return send_negative_response(request, OBDII_NRC_SERVICE_NOT_SUPPORTED, response_msg);
    }
    
    // Convert response to CAN message
    if (response.is_valid) {
        return response_to_can_message(response, response_msg);
    } else {
        return send_negative_response(request, OBDII_NRC_SUBFUNC_NOT_SUPPORTED, response_msg);
    }
}

bool OBDIIHandler::send_negative_response(const obdii_request_t& request, uint8_t nrc, CAN_message_t& response_msg) {
    response_msg.id = OBDII_ECU_RESPONSE_ID;
    response_msg.len = 3;
    response_msg.buf[0] = 0x03;  // Length
    response_msg.buf[1] = OBDII_NEGATIVE_RESPONSE;
    response_msg.buf[2] = request.mode;
    response_msg.buf[3] = nrc;
    
    #ifdef ARDUINO
    response_msg.timestamp = micros();
    #endif
    
    stats.negative_responses++;
    debug_print("OBDIIHandler: Sent negative response");
    
    return true;
}

// ============================================================================
// MODE 01 (CURRENT DATA) HANDLERS
// ============================================================================

bool OBDIIHandler::handle_mode01_request(const obdii_request_t& request, obdii_response_t& response) {
    // Check for supported PIDs request
    if (request.pid == OBDII_PID_SUPPORTED_01_20 || 
        request.pid == OBDII_PID_SUPPORTED_21_40 || 
        request.pid == OBDII_PID_SUPPORTED_41_60) {
        stats.supported_pid_requests++;
        return generate_supported_pids_response(request.pid, response);
    }
    
    // Check if PID is supported
    if (!is_pid_supported(request.pid)) {
        stats.unsupported_requests++;
        return false;
    }
    
    // Route to specific PID handler
    switch (request.pid) {
        case OBDII_PID_ENGINE_RPM:
            return handle_pid_engine_rpm(response);
            
        case OBDII_PID_VEHICLE_SPEED:
            return handle_pid_vehicle_speed(response);
            
        case OBDII_PID_COOLANT_TEMP:
            return handle_pid_coolant_temp(response);
            
        case OBDII_PID_THROTTLE_POSITION:
            return handle_pid_throttle_position(response);
            
        case OBDII_PID_INTAKE_AIR_TEMP:
            return handle_pid_intake_air_temp(response);
            
        case OBDII_PID_MANIFOLD_PRESSURE:
            return handle_pid_manifold_pressure(response);
            
        case OBDII_PID_ENGINE_LOAD:
            return handle_pid_engine_load(response);
            
        default:
            // Check for custom PID handler
            if (custom_pid_handlers[request.pid] != nullptr) {
                return custom_pid_handlers[request.pid](request.pid, response.data, &response.data_len);
            }
            
            stats.unsupported_requests++;
            return false;
    }
}

bool OBDIIHandler::generate_supported_pids_response(uint8_t pid_range, obdii_response_t& response) {
    uint32_t supported_pids = 0;
    
    switch (pid_range) {
        case OBDII_PID_SUPPORTED_01_20:
            supported_pids = supported_pids_01_20;
            break;
            
        case OBDII_PID_SUPPORTED_21_40:
            supported_pids = supported_pids_21_40;
            break;
            
        case OBDII_PID_SUPPORTED_41_60:
            supported_pids = supported_pids_41_60;
            break;
            
        default:
            return false;
    }
    
    // Pack 32-bit value into 4 bytes (big-endian)
    response.data[0] = (supported_pids >> 24) & 0xFF;
    response.data[1] = (supported_pids >> 16) & 0xFF;
    response.data[2] = (supported_pids >> 8) & 0xFF;
    response.data[3] = supported_pids & 0xFF;
    response.data_len = 4;
    
    return true;
}

// ============================================================================
// STANDARD PID HANDLERS
// ============================================================================

bool OBDIIHandler::handle_pid_engine_rpm(obdii_response_t& response) {
    float rpm_value;
    if (cache->get_value(OBDII_PID_ENGINE_RPM, &rpm_value)) {
        uint16_t obdii_rpm = float_to_obdii_rpm(rpm_value);
        response.data[0] = (obdii_rpm >> 8) & 0xFF;
        response.data[1] = obdii_rpm & 0xFF;
        response.data_len = 2;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_vehicle_speed(obdii_response_t& response) {
    float speed_value;
    if (cache->get_value(OBDII_PID_VEHICLE_SPEED, &speed_value)) {
        response.data[0] = float_to_obdii_speed(speed_value);
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_coolant_temp(obdii_response_t& response) {
    float temp_value;
    if (cache->get_value(OBDII_PID_COOLANT_TEMP, &temp_value)) {
        response.data[0] = float_to_obdii_temp(temp_value);
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_throttle_position(obdii_response_t& response) {
    float tps_value;
    if (cache->get_value(OBDII_PID_THROTTLE_POSITION, &tps_value)) {
        response.data[0] = float_to_obdii_percent(tps_value);
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_intake_air_temp(obdii_response_t& response) {
    float temp_value;
    if (cache->get_value(OBDII_PID_INTAKE_AIR_TEMP, &temp_value)) {
        response.data[0] = float_to_obdii_temp(temp_value);
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_manifold_pressure(obdii_response_t& response) {
    float map_value;
    if (cache->get_value(OBDII_PID_MANIFOLD_PRESSURE, &map_value)) {
        // Convert kPa to OBD-II format (1 byte, 0-255 kPa)
        uint8_t obdii_map = (uint8_t)(map_value);
        if (obdii_map > 255) obdii_map = 255;
        response.data[0] = obdii_map;
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

bool OBDIIHandler::handle_pid_engine_load(obdii_response_t& response) {
    // Calculate engine load based on throttle position and RPM
    float tps_value, rpm_value;
    if (cache->get_value(OBDII_PID_THROTTLE_POSITION, &tps_value) &&
        cache->get_value(OBDII_PID_ENGINE_RPM, &rpm_value)) {
        
        // Simple engine load calculation: weighted average of TPS and RPM%
        float rpm_percent = (rpm_value / 7000.0f) * 100.0f;  // Assume 7000 RPM redline
        if (rpm_percent > 100.0f) rpm_percent = 100.0f;
        
        float engine_load = (tps_value * 0.7f + rpm_percent * 0.3f);
        response.data[0] = float_to_obdii_percent(engine_load);
        response.data_len = 1;
        stats.cache_hits++;
        return true;
    }
    
    stats.cache_misses++;
    return false;
}

// ============================================================================
// MODE 03 AND MODE 09 HANDLERS (MINIMAL IMPLEMENTATION)
// ============================================================================

bool OBDIIHandler::handle_mode03_request(const obdii_request_t& request, obdii_response_t& response) {
    // Mode 03: Show stored diagnostic codes
    // For now, return no codes stored
    response.data[0] = 0x00;  // Number of codes (high byte)
    response.data[1] = 0x00;  // Number of codes (low byte)
    response.data_len = 2;
    
    return true;
}

bool OBDIIHandler::handle_mode09_request(const obdii_request_t& request, obdii_response_t& response) {
    // Mode 09: Vehicle information
    switch (request.pid) {
        case 0x02:  // VIN
            // Return placeholder VIN
            memcpy(response.data, "ECU1", 4);
            response.data_len = 4;
            return true;
            
        default:
            return false;
    }
}

// ============================================================================
// CONFIGURATION METHODS
// ============================================================================

bool OBDIIHandler::register_custom_pid(uint8_t pid, custom_pid_handler_t handler) {
    if (!initialized || handler == nullptr) {
        return false;
    }
    
    custom_pid_handlers[pid] = handler;
    
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), 
            "OBDIIHandler: Registered custom PID 0x%02X", pid);
    debug_print(debug_msg);
    
    return true;
}

void OBDIIHandler::unregister_custom_pid(uint8_t pid) {
    custom_pid_handlers[pid] = nullptr;
}

void OBDIIHandler::enable_standard_pid(uint8_t pid, bool enable) {
    uint32_t pid_bit = 1UL << (31 - (pid & 0x1F));
    
    if (pid <= 0x20) {
        if (enable) {
            supported_pids_01_20 |= pid_bit;
        } else {
            supported_pids_01_20 &= ~pid_bit;
        }
    } else if (pid <= 0x40) {
        if (enable) {
            supported_pids_21_40 |= pid_bit;
        } else {
            supported_pids_21_40 &= ~pid_bit;
        }
    } else if (pid <= 0x60) {
        if (enable) {
            supported_pids_41_60 |= pid_bit;
        } else {
            supported_pids_41_60 &= ~pid_bit;
        }
    }
}

bool OBDIIHandler::is_pid_supported(uint8_t pid) {
    uint32_t pid_bit = 1UL << (31 - (pid & 0x1F));
    
    if (pid <= 0x20) {
        return (supported_pids_01_20 & pid_bit) != 0;
    } else if (pid <= 0x40) {
        return (supported_pids_21_40 & pid_bit) != 0;
    } else if (pid <= 0x60) {
        return (supported_pids_41_60 & pid_bit) != 0;
    }
    
    return false;
}

void OBDIIHandler::enable_mode(uint8_t mode, bool enable) {
    uint32_t mode_bit = 1UL << mode;
    
    if (enable) {
        supported_modes |= mode_bit;
    } else {
        supported_modes &= ~mode_bit;
    }
}

bool OBDIIHandler::is_mode_supported(uint8_t mode) {
    uint32_t mode_bit = 1UL << mode;
    return (supported_modes & mode_bit) != 0;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

bool OBDIIHandler::response_to_can_message(const obdii_response_t& response, CAN_message_t& msg) {
    msg.id = OBDII_ECU_RESPONSE_ID;
    msg.len = response.data_len + 2;  // +2 for length and mode
    
    if (msg.len > 8) {
        debug_print("OBDIIHandler: Response too long for CAN message");
        return false;
    }
    
    msg.buf[0] = msg.len - 1;  // Length (excluding length byte itself)
    msg.buf[1] = response.mode;
    
    if (response.data_len > 0) {
        msg.buf[2] = response.pid;
        memcpy(&msg.buf[3], response.data, response.data_len);
    }
    
    #ifdef ARDUINO
    msg.timestamp = micros();
    #endif
    
    return true;
}

uint16_t OBDIIHandler::float_to_obdii_rpm(float rpm) {
    // OBD-II RPM: ((A*256)+B)/4
    // So RPM * 4 = (A*256)+B
    uint16_t scaled_rpm = (uint16_t)(rpm * 4.0f);
    return scaled_rpm;
}

uint8_t OBDIIHandler::float_to_obdii_speed(float speed_mph) {
    // OBD-II speed: direct mapping (km/h)
    // Convert MPH to km/h
    float speed_kmh = speed_mph * 1.60934f;
    if (speed_kmh > 255.0f) speed_kmh = 255.0f;
    return (uint8_t)speed_kmh;
}

uint8_t OBDIIHandler::float_to_obdii_temp(float temp_celsius) {
    // OBD-II temperature: A - 40 (°C)
    int16_t temp_offset = (int16_t)(temp_celsius + 40.0f);
    if (temp_offset < 0) temp_offset = 0;
    if (temp_offset > 255) temp_offset = 255;
    return (uint8_t)temp_offset;
}

uint8_t OBDIIHandler::float_to_obdii_percent(float percent) {
    // OBD-II percentage: A * 100 / 255
    float scaled = (percent * 255.0f) / 100.0f;
    if (scaled > 255.0f) scaled = 255.0f;
    if (scaled < 0.0f) scaled = 0.0f;
    return (uint8_t)scaled;
}

// ============================================================================
// DIAGNOSTICS AND DEBUGGING
// ============================================================================

void OBDIIHandler::reset_statistics() {
    stats = {};  // Zero-initialize all fields
}

void OBDIIHandler::handle_error(const char* error_msg) {
    debug_print(error_msg);
}

void OBDIIHandler::increment_error_count() {
    // Error counting would be added to stats if needed
}

void OBDIIHandler::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void OBDIIHandler::debug_print_request(const obdii_request_t& request) {
    #ifdef ARDUINO
    Serial.print("OBD-II Request: Mode=0x");
    Serial.print(request.mode, HEX);
    Serial.print(" PID=0x");
    Serial.print(request.pid, HEX);
    Serial.print(" DataLen=");
    Serial.println(request.data_len);
    #else
    printf("OBD-II Request: Mode=0x%02X PID=0x%02X DataLen=%d\n", 
           request.mode, request.pid, request.data_len);
    #endif
}

void OBDIIHandler::debug_print_response(const obdii_response_t& response) {
    #ifdef ARDUINO
    Serial.print("OBD-II Response: Mode=0x");
    Serial.print(response.mode, HEX);
    Serial.print(" PID=0x");
    Serial.print(response.pid, HEX);
    Serial.print(" DataLen=");
    Serial.print(response.data_len);
    Serial.print(" Data=");
    for (uint8_t i = 0; i < response.data_len; i++) {
        Serial.print(response.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    #else
    printf("OBD-II Response: Mode=0x%02X PID=0x%02X DataLen=%d Data=", 
           response.mode, response.pid, response.data_len);
    for (uint8_t i = 0; i < response.data_len; i++) {
        printf("%02X ", response.data[i]);
    }
    printf("\n");
    #endif
}

// ============================================================================
// TESTING INTERFACE
// ============================================================================

#ifndef ARDUINO
bool OBDIIHandler::simulate_request(uint8_t mode, uint8_t pid) {
    CAN_message_t msg = {};
    msg.id = OBDII_REQUEST_ID;
    msg.len = 3;
    msg.buf[0] = 0x02;  // Length
    msg.buf[1] = mode;
    msg.buf[2] = pid;
    msg.timestamp = micros();
    
    return process_request(msg);
}

bool OBDIIHandler::simulate_request_message(const CAN_message_t& msg, CAN_message_t& response) {
    // Process request and capture response
    bool result = process_request(msg);
    
    // For testing, we'd need to capture the actual response message
    // This is a simplified version
    if (result) {
        response.id = OBDII_ECU_RESPONSE_ID;
        response.len = 4;  // Example response length
        response.buf[0] = 0x03;  // Length
        response.buf[1] = msg.buf[1] + 0x40;  // Response mode
        response.buf[2] = msg.buf[2];  // PID
        response.buf[3] = 0x00;  // Example data
    }
    
    return result;
}
#endif