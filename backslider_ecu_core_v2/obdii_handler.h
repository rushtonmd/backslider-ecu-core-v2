// obdii_handler.h
// OBD-II protocol handler for external CAN bus
// Supports standard OBD-II modes and PIDs with cache integration

#ifndef OBDII_HANDLER_H
#define OBDII_HANDLER_H

#include "msg_definitions.h"
#include "external_canbus_cache.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "tests/mock_arduino.h"
#endif

// OBD-II Constants
#define OBDII_REQUEST_ID            0x7DF    // Standard OBD-II request ID
#define OBDII_RESPONSE_ID_BASE      0x7E8    // Response ID base (7E8-7EF)
#define OBDII_ECU_RESPONSE_ID       0x7E8    // Our ECU response ID

#define OBDII_MAX_DATA_BYTES        7        // Maximum data bytes in OBD-II message
#define OBDII_POSITIVE_RESPONSE     0x40     // Positive response offset
#define OBDII_NEGATIVE_RESPONSE     0x7F     // Negative response service ID

// OBD-II Service/Mode definitions
#define OBDII_MODE_CURRENT_DATA     0x01     // Show current data
#define OBDII_MODE_FREEZE_FRAME     0x02     // Show freeze frame data
#define OBDII_MODE_DIAGNOSTIC_CODES 0x03     // Show stored diagnostic codes
#define OBDII_MODE_CLEAR_CODES      0x04     // Clear diagnostic codes
#define OBDII_MODE_O2_MONITORING    0x05     // O2 sensor monitoring test results
#define OBDII_MODE_ONBOARD_MONITOR  0x06     // Onboard monitoring test results
#define OBDII_MODE_PENDING_CODES    0x07     // Show pending diagnostic codes
#define OBDII_MODE_CONTROL_SYSTEM   0x08     // Control onboard system
#define OBDII_MODE_VEHICLE_INFO     0x09     // Request vehicle information

// OBD-II Negative Response Codes (NRC)
#define OBDII_NRC_SERVICE_NOT_SUPPORTED         0x11
#define OBDII_NRC_SUBFUNC_NOT_SUPPORTED         0x12
#define OBDII_NRC_REQUEST_OUT_OF_RANGE          0x31
#define OBDII_NRC_CONDITIONS_NOT_CORRECT        0x22
#define OBDII_NRC_REQUEST_SEQUENCE_ERROR        0x24
#define OBDII_NRC_RESPONSE_PENDING              0x78

// OBD-II request structure
struct obdii_request_t {
    uint8_t mode;           // Service/Mode (0x01, 0x02, etc.)
    uint8_t pid;            // Parameter ID
    uint8_t data_len;       // Length of additional data
    uint8_t data[5];        // Additional data (for some modes)
    uint32_t timestamp;     // When request was received
    uint32_t source_id;     // CAN ID of requester
};

// OBD-II response structure
struct obdii_response_t {
    uint8_t mode;           // Response mode (request mode + 0x40)
    uint8_t pid;            // Parameter ID
    uint8_t data_len;       // Length of response data
    uint8_t data[4];        // Response data (up to 4 bytes for most PIDs)
    bool is_valid;          // Whether response is valid
};

// OBD-II statistics
struct obdii_stats_t {
    uint32_t requests_received;     // Total requests received
    uint32_t responses_sent;        // Total responses sent
    uint32_t mode01_requests;       // Mode 01 (current data) requests
    uint32_t supported_pid_requests; // Requests for supported PIDs
    uint32_t unsupported_requests;  // Requests for unsupported PIDs/modes
    uint32_t cache_hits;           // Requests served from cache
    uint32_t cache_misses;         // Requests where cache had no data
    uint32_t negative_responses;   // Negative responses sent
    uint32_t malformed_requests;   // Malformed request messages
};

// Custom PID handler function type
typedef bool (*custom_pid_handler_t)(uint8_t pid, uint8_t* response_data, uint8_t* response_len);

class OBDIIHandler {
public:
    // Constructor
    OBDIIHandler(ExternalCanBusCache* cache);
    ~OBDIIHandler();
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    
    // Initialize OBD-II handler
    bool init();
    
    // Shutdown handler
    void shutdown();
    
    // =========================================================================
    // REQUEST PROCESSING
    // =========================================================================
    
    // Process incoming OBD-II request message
    bool process_request(const CAN_message_t& msg);
    
    // Check if message is an OBD-II request
    bool is_obdii_request(const CAN_message_t& msg);
    
    // =========================================================================
    // RESPONSE GENERATION
    // =========================================================================
    
    // Generate response message for a request
    bool generate_response(const obdii_request_t& request, CAN_message_t& response_msg);
    
    // Send negative response
    bool send_negative_response(const obdii_request_t& request, uint8_t nrc, CAN_message_t& response_msg);
    
    // =========================================================================
    // PID SUPPORT CONFIGURATION
    // =========================================================================
    
    // Register custom PID handler
    bool register_custom_pid(uint8_t pid, custom_pid_handler_t handler);
    
    // Unregister custom PID handler
    void unregister_custom_pid(uint8_t pid);
    
    // Enable/disable specific standard PIDs
    void enable_standard_pid(uint8_t pid, bool enable);
    bool is_pid_supported(uint8_t pid);
    
    // =========================================================================
    // MODE SUPPORT
    // =========================================================================
    
    // Enable/disable OBD-II modes
    void enable_mode(uint8_t mode, bool enable);
    bool is_mode_supported(uint8_t mode);
    
    // =========================================================================
    // DIAGNOSTICS AND STATISTICS
    // =========================================================================
    
    // Get statistics
    const obdii_stats_t& get_statistics() const { return stats; }
    void reset_statistics();
    
    // Status
    bool is_initialized() const { return initialized; }
    uint32_t get_last_request_time() const { return last_request_time; }
    
    // =========================================================================
    // TESTING INTERFACE
    // =========================================================================
    
    #if !defined(ARDUINO) || defined(TESTING)
    // Simulate OBD-II request for testing
    bool simulate_request(uint8_t mode, uint8_t pid);
    bool simulate_request_message(const CAN_message_t& msg, CAN_message_t& response);
    
    // Get internal state for testing
    const obdii_request_t* get_last_request_for_testing() const { return &last_request; }
    #endif
    
private:
    // =========================================================================
    // INTERNAL DATA
    // =========================================================================
    
    ExternalCanBusCache* cache;     // Reference to cache system
    bool initialized;
    uint32_t last_request_time;
    
    // Statistics
    obdii_stats_t stats;
    
    // Last processed request (for debugging)
    obdii_request_t last_request;
    
    // Configuration
    uint32_t supported_modes;       // Bitfield of supported modes
    uint32_t supported_pids_01_20;  // PIDs 0x01-0x20 support bitfield
    uint32_t supported_pids_21_40;  // PIDs 0x21-0x40 support bitfield
    uint32_t supported_pids_41_60;  // PIDs 0x41-0x60 support bitfield
    
    // Custom PID handlers
    custom_pid_handler_t custom_pid_handlers[256];
    
    // =========================================================================
    // PRIVATE METHODS - REQUEST PARSING
    // =========================================================================
    
    // Parse incoming request message
    bool parse_request_message(const CAN_message_t& msg, obdii_request_t& request);
    
    // Validate request format
    bool validate_request(const obdii_request_t& request);
    
    // =========================================================================
    // PRIVATE METHODS - RESPONSE GENERATION
    // =========================================================================
    
    // Handle Mode 01 (current data) requests
    bool handle_mode01_request(const obdii_request_t& request, obdii_response_t& response);
    
    // Handle Mode 03 (diagnostic codes) requests
    bool handle_mode03_request(const obdii_request_t& request, obdii_response_t& response);
    
    // Handle Mode 09 (vehicle info) requests
    bool handle_mode09_request(const obdii_request_t& request, obdii_response_t& response);
    
    // Generate supported PIDs response
    bool generate_supported_pids_response(uint8_t pid_range, obdii_response_t& response);
    
    // =========================================================================
    // PRIVATE METHODS - STANDARD PID HANDLERS
    // =========================================================================
    
    // Standard Mode 01 PID handlers
    bool handle_pid_engine_rpm(obdii_response_t& response);           // PID 0x0C
    bool handle_pid_vehicle_speed(obdii_response_t& response);        // PID 0x0D
    bool handle_pid_coolant_temp(obdii_response_t& response);         // PID 0x05
    bool handle_pid_throttle_position(obdii_response_t& response);    // PID 0x11
    bool handle_pid_intake_air_temp(obdii_response_t& response);      // PID 0x0F
    bool handle_pid_manifold_pressure(obdii_response_t& response);    // PID 0x0B
    bool handle_pid_engine_load(obdii_response_t& response);          // PID 0x04
    
    // =========================================================================
    // PRIVATE METHODS - UTILITY
    // =========================================================================
    
    // Convert response structure to CAN message
    bool response_to_can_message(const obdii_response_t& response, CAN_message_t& msg);
    
    // Pack response data into CAN message
    void pack_response_data(const obdii_response_t& response, CAN_message_t& msg);
    
    // Data conversion utilities
    uint16_t float_to_obdii_rpm(float rpm);
    uint8_t float_to_obdii_speed(float speed_mph);
    uint8_t float_to_obdii_temp(float temp_celsius);
    uint8_t float_to_obdii_percent(float percent);
    
    // Error handling
    void handle_error(const char* error_msg);
    void increment_error_count();
    
    // Debugging
    void debug_print(const char* message);
    void debug_print_request(const obdii_request_t& request);
    void debug_print_response(const obdii_response_t& response);
};

// ============================================================================
// STANDARD OBD-II PID DEFINITIONS
// ============================================================================

// Mode 01 PIDs (Current Data)
#define OBDII_PID_SUPPORTED_01_20       0x00    // Supported PIDs 01-20
#define OBDII_PID_MONITOR_STATUS        0x01    // Monitor status since DTCs cleared
#define OBDII_PID_FREEZE_DTC            0x02    // Freeze DTC
#define OBDII_PID_FUEL_SYSTEM_STATUS    0x03    // Fuel system status
#define OBDII_PID_ENGINE_LOAD           0x04    // Calculated engine load
#define OBDII_PID_COOLANT_TEMP          0x05    // Engine coolant temperature
#define OBDII_PID_SHORT_TERM_FUEL_1     0x06    // Short term fuel trim - Bank 1
#define OBDII_PID_LONG_TERM_FUEL_1      0x07    // Long term fuel trim - Bank 1
#define OBDII_PID_SHORT_TERM_FUEL_2     0x08    // Short term fuel trim - Bank 2
#define OBDII_PID_LONG_TERM_FUEL_2      0x09    // Long term fuel trim - Bank 2
#define OBDII_PID_FUEL_PRESSURE         0x0A    // Fuel pressure
#define OBDII_PID_MANIFOLD_PRESSURE     0x0B    // Intake manifold absolute pressure
#define OBDII_PID_ENGINE_RPM            0x0C    // Engine RPM
#define OBDII_PID_VEHICLE_SPEED         0x0D    // Vehicle speed
#define OBDII_PID_TIMING_ADVANCE        0x0E    // Timing advance
#define OBDII_PID_INTAKE_AIR_TEMP       0x0F    // Intake air temperature
#define OBDII_PID_MAF_RATE              0x10    // MAF air flow rate
#define OBDII_PID_THROTTLE_POSITION     0x11    // Throttle position
#define OBDII_PID_SECONDARY_AIR_STATUS  0x12    // Commanded secondary air status
#define OBDII_PID_O2_SENSORS_PRESENT    0x13    // Oxygen sensors present
#define OBDII_PID_SUPPORTED_21_40       0x20    // Supported PIDs 21-40
#define OBDII_PID_DISTANCE_WITH_MIL     0x21    // Distance traveled with MIL on
#define OBDII_PID_FUEL_RAIL_PRESSURE    0x22    // Fuel rail pressure
#define OBDII_PID_FUEL_RAIL_GAUGE_PRESS 0x23    // Fuel rail gauge pressure
#define OBDII_PID_SUPPORTED_41_60       0x40    // Supported PIDs 41-60
#define OBDII_PID_CONTROL_MODULE_VOLT   0x42    // Control module power supply voltage
#define OBDII_PID_ABSOLUTE_LOAD         0x43    // Absolute load value
#define OBDII_PID_FUEL_AIR_EQUIV_RATIO  0x44    // Fuel-air equivalence ratio
#define OBDII_PID_RELATIVE_THROTTLE     0x45    // Relative throttle position

// Default supported PIDs bitfields
#define DEFAULT_SUPPORTED_PIDS_01_20    0x1E100000  // PIDs 0x04, 0x05, 0x0B, 0x0C, 0x0D, 0x0F, 0x11
#define DEFAULT_SUPPORTED_PIDS_21_40    0x80000000  // PID 0x20 (supported PIDs 21-40)
#define DEFAULT_SUPPORTED_PIDS_41_60    0x00000000  // No additional PIDs supported by default

// Default supported modes bitfield
#define DEFAULT_SUPPORTED_MODES         0x0000000F  // Modes 01, 03, 04, 09

#endif