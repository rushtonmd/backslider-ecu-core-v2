// msg_definitions.h
// CAN-style message definitions for Backslider ECU - Extended CAN ID Architecture

#ifndef MSG_DEFINITIONS_H
#define MSG_DEFINITIONS_H

#include <stdint.h>
#include <string.h>  // for memcpy

// Use FlexCAN message format on Arduino, mock for desktop testing
#ifdef ARDUINO
    #include <FlexCAN_T4.h>
    // Use FlexCAN_T4 extended message structure
    typedef CAN_message_t CANMessage;
#else
    // Mock CAN message structure for desktop testing - extended CAN support
    typedef struct {
        uint32_t id;          // Message ID (29-bit extended)
        uint8_t len;          // Data length (0-8 bytes)
        uint8_t buf[8];       // Data payload
        uint32_t timestamp;   // Timestamp (for compatibility)
        struct {
            uint8_t extended : 1;  // Extended CAN ID flag
            uint8_t remote : 1;    // Remote frame flag
            uint8_t reserved : 6;  // Reserved bits
        } flags;
    } CANMessage;
#endif

// =============================================================================
// EXTENDED CAN ID ARCHITECTURE
// =============================================================================

// 29-bit Extended CAN ID Structure: [ECU_BASE(4)] [SUBSYSTEM(8)] [PARAMETER(17)]
#define ECU_BASE_MASK           0xF0000000  // ECU identifier (4 bits)
#define SUBSYSTEM_MASK          0x0FF00000  // Subsystem identifier (8 bits)
#define PARAMETER_MASK          0x000FFFFF  // Parameter identifier (20 bits)

// ECU Base Addresses (4 bits = 16 possible ECUs)
#define ECU_BASE_PRIMARY        0x10000000  // Primary ECU (engine timing)
#define ECU_BASE_SECONDARY      0x20000000  // Secondary ECU (control/logging)
#define ECU_BASE_DASHBOARD      0x30000000  // Dashboard/display unit
#define ECU_BASE_DATALOGGER     0x40000000  // Data logging ECU
#define ECU_BASE_DIAGNOSTIC     0x50000000  // Diagnostic equipment
#define ECU_BASE_TUNING         0x60000000  // Tuning software/tablet
#define ECU_BASE_RESERVED_1     0x70000000  // Reserved for future use
#define ECU_BASE_RESERVED_2     0x80000000  // Reserved for future use

// Subsystem Identifiers (8 bits = 256 possible subsystems)
#define SUBSYSTEM_FUEL          0x00100000  // Fuel injection system
#define SUBSYSTEM_IGNITION      0x00200000  // Ignition timing system
#define SUBSYSTEM_SENSORS       0x00300000  // Sensor readings
#define SUBSYSTEM_CONFIG        0x00400000  // Configuration parameters
#define SUBSYSTEM_TRANSMISSION  0x00500000  // Transmission control
#define SUBSYSTEM_COOLING       0x00600000  // Cooling system
#define SUBSYSTEM_EXHAUST       0x00700000  // Exhaust system
#define SUBSYSTEM_BOOST         0x00800000  // Turbo/supercharger
#define SUBSYSTEM_STORAGE       0x00900000  // Storage operations
#define SUBSYSTEM_SYSTEM        0x00A00000  // System messages
#define SUBSYSTEM_DEBUG         0x00B00000  // Debug/diagnostic messages
#define SUBSYSTEM_EXTERNAL      0x00C00000  // External communications

// Extended CAN ID Generation Macros
#define MAKE_EXTENDED_CAN_ID(ecu_base, subsystem, parameter) \
    ((ecu_base) | (subsystem) | ((parameter) & PARAMETER_MASK))

#define GET_ECU_BASE(id)        ((id) & ECU_BASE_MASK)
#define GET_SUBSYSTEM(id)       ((id) & SUBSYSTEM_MASK)
#define GET_PARAMETER(id)       ((id) & PARAMETER_MASK)

// Common parameter ID generators
#define SENSOR_ID(sensor_type)          (sensor_type)
#define MAP_CELL_ID(row, col)           (((row) << 8) | (col))
#define CONFIG_ID(config_type)          (config_type)
#define CONTROL_ID(control_type)        (0x100 + (control_type))  // Control messages start at 0x100 to avoid sensor collisions

// =============================================================================
// SENSOR MESSAGE DEFINITIONS
// =============================================================================

// Primary ECU - Sensor readings (real-time, high priority)
#define MSG_ENGINE_RPM          MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x01))
#define MSG_VEHICLE_SPEED       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x02))
#define MSG_COOLANT_TEMP        MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x03))
#define MSG_THROTTLE_POSITION   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x04))
#define MSG_MANIFOLD_PRESSURE   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x05))
#define MSG_AIR_INTAKE_TEMP     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x06))
#define MSG_BATTERY_VOLTAGE     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x07))
#define MSG_OIL_PRESSURE        MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x08))
#define MSG_CRANK_POSITION      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x09))
#define MSG_TIMING_TRIGGER      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x0A))
#define MSG_BRAKE_PEDAL         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SENSORS, SENSOR_ID(0x0B))

// =============================================================================
// CONTROL MESSAGE DEFINITIONS
// =============================================================================

// Primary ECU - Control outputs (medium priority)
#define MSG_IGNITION_TIMING     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x01))
#define MSG_FUEL_PULSE_WIDTH    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x01))
#define MSG_IDLE_TARGET_RPM     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x02))
#define MSG_BOOST_TARGET        MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_BOOST, CONTROL_ID(0x01))

// Engine output controls
#define MSG_IGNITION_COIL_1     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x10))
#define MSG_IGNITION_COIL_2     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x11))
#define MSG_IGNITION_COIL_3     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x12))
#define MSG_IGNITION_COIL_4     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x13))
#define MSG_IGNITION_COIL_5     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x14))
#define MSG_IGNITION_COIL_6     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x15))
#define MSG_IGNITION_COIL_7     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x16))
#define MSG_IGNITION_COIL_8     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, CONTROL_ID(0x17))

#define MSG_FUEL_INJECTOR_1     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x10))
#define MSG_FUEL_INJECTOR_2     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x11))
#define MSG_FUEL_INJECTOR_3     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x12))
#define MSG_FUEL_INJECTOR_4     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x13))
#define MSG_FUEL_INJECTOR_5     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x14))
#define MSG_FUEL_INJECTOR_6     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x15))
#define MSG_FUEL_INJECTOR_7     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x16))
#define MSG_FUEL_INJECTOR_8     MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x17))

// Auxiliary control outputs
#define MSG_IDLE_VALVE_CONTROL  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x20))
#define MSG_FUEL_PUMP_CONTROL   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, CONTROL_ID(0x21))
#define MSG_FAN_CONTROL         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_COOLING, CONTROL_ID(0x01))
#define MSG_A_C_CLUTCH_CONTROL  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_COOLING, CONTROL_ID(0x02))
#define MSG_ALTERNATOR_FIELD    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, CONTROL_ID(0x01))
#define MSG_BOOST_CONTROL       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_BOOST, CONTROL_ID(0x10))
#define MSG_WASTEGATE_CONTROL   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_BOOST, CONTROL_ID(0x11))
#define MSG_SHIFT_LIGHT         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, CONTROL_ID(0x10))
#define MSG_STATUS_LED          MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, CONTROL_ID(0x11))

// Gauge outputs
#define MSG_BOOST_GAUGE         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x01))
#define MSG_TEMP_GAUGE          MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x02))
#define MSG_FUEL_GAUGE          MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x03))
#define MSG_OIL_PRESSURE_GAUGE  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x04))
#define MSG_TACH_OUTPUT         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x05))
#define MSG_SPEEDO_OUTPUT       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, CONTROL_ID(0x06))

// =============================================================================
// TRANSMISSION MESSAGE DEFINITIONS
// =============================================================================

// Transmission sensor inputs
#define MSG_TRANS_FLUID_TEMP    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x01))
#define MSG_PADDLE_UPSHIFT      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x02))
#define MSG_PADDLE_DOWNSHIFT    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x03))
#define MSG_TRANS_PARK_SWITCH   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x04))
#define MSG_TRANS_REVERSE_SWITCH MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x05))
#define MSG_TRANS_NEUTRAL_SWITCH MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x06))
#define MSG_TRANS_DRIVE_SWITCH  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x07))
#define MSG_TRANS_SECOND_SWITCH MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x08))
#define MSG_TRANS_FIRST_SWITCH  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x09))
#define MSG_TRANS_INPUT_SPEED   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x0A))
#define MSG_TRANS_OUTPUT_SPEED  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, SENSOR_ID(0x0B))

// Transmission state messages
#define MSG_TRANS_CURRENT_GEAR  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x01))
#define MSG_TRANS_SHIFT_REQUEST MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x02))
#define MSG_TRANS_STATE_VALID   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x03))
#define MSG_TRANS_DRIVE_GEAR    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x04))
#define MSG_TRANS_OVERRUN_STATE MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x05))

// Transmission output controls
#define MSG_TRANS_SHIFT_SOL_A   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x10))
#define MSG_TRANS_SHIFT_SOL_B   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x11))
#define MSG_TRANS_OVERRUN_SOL   MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x12))
#define MSG_TRANS_PRESSURE_SOL  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x13))
#define MSG_TRANS_LOCKUP_SOL    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_TRANSMISSION, CONTROL_ID(0x14))

// =============================================================================
// SYSTEM MESSAGE DEFINITIONS
// =============================================================================

// System messages (low priority)
#define MSG_HEARTBEAT           MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, 0x01)
#define MSG_SYSTEM_TIME         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, 0x02)
#define MSG_DEBUG_MESSAGE       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_DEBUG, 0x01)
#define MSG_ENGINE_STATUS       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, 0x03)
#define MSG_ERROR_CODES         MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_SYSTEM, 0x04)

// =============================================================================
// MAP CELL DIRECT ADDRESSING
// =============================================================================

// Fuel Map Cells - Direct addressing (30x30 grid)
#define MSG_FUEL_MAP_CELL(row, col) \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_FUEL, MAP_CELL_ID(row, col))

// Ignition Map Cells - Direct addressing (30x30 grid)
#define MSG_IGNITION_MAP_CELL(row, col) \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_IGNITION, MAP_CELL_ID(row, col))

// Boost Map Cells - Direct addressing (20x20 grid)
#define MSG_BOOST_MAP_CELL(row, col) \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_BOOST, MAP_CELL_ID(row, col))

// =============================================================================
// CONFIGURATION PARAMETERS
// =============================================================================

// Fuel system configuration
#define MSG_CONFIG_FUEL_BASE_PRESSURE \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x01))
#define MSG_CONFIG_FUEL_INJECTOR_FLOW \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x02))
#define MSG_CONFIG_FUEL_PUMP_PRESSURE \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x03))

// Ignition system configuration
#define MSG_CONFIG_IGNITION_BASE_TIMING \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x10))
#define MSG_CONFIG_IGNITION_DWELL_TIME \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x11))
#define MSG_CONFIG_IGNITION_COIL_CHARGE \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x12))

// Engine configuration
#define MSG_CONFIG_ENGINE_DISPLACEMENT \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x20))
#define MSG_CONFIG_ENGINE_COMPRESSION \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x21))
#define MSG_CONFIG_ENGINE_REDLINE \
    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_CONFIG, CONFIG_ID(0x22))

// =============================================================================
// STORAGE OPERATIONS
// =============================================================================

// Storage operations using extended CAN IDs as keys
#define MSG_STORAGE_SAVE        MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x01)
#define MSG_STORAGE_LOAD        MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x02)
#define MSG_STORAGE_DELETE      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x03)
#define MSG_STORAGE_COMMIT      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x04)

// Storage responses
#define MSG_STORAGE_SAVE_RESPONSE MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x11)
#define MSG_STORAGE_LOAD_RESPONSE MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x12)
#define MSG_STORAGE_ERROR       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x13)
#define MSG_STORAGE_STATS       MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x14)

// Storage export operations
#define MSG_STORAGE_EXPORT_REQUEST  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x20)
#define MSG_STORAGE_EXPORT_START    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x21)
#define MSG_STORAGE_EXPORT_KEY      MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x22)
#define MSG_STORAGE_EXPORT_COMPLETE MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_STORAGE, 0x23)

// =============================================================================
// STORAGE MESSAGE STRUCTURES
// =============================================================================

// Storage save message (using CAN ID as key)
typedef struct {
    uint32_t storage_key;       // Extended CAN ID used as storage key (4 bytes)
    float value;                // Value to store (4 bytes)
} __attribute__((packed)) storage_save_float_msg_t;

// Storage load message (using CAN ID as key)
typedef struct {
    uint32_t storage_key;       // Extended CAN ID used as storage key (4 bytes)
    float default_value;        // Default value if key not found (4 bytes)
} __attribute__((packed)) storage_load_float_msg_t;

// Storage load response message
typedef struct {
    uint32_t storage_key;       // Extended CAN ID used as storage key (4 bytes)
    float value;                // Retrieved value (4 bytes)
} __attribute__((packed)) storage_load_response_msg_t;

// Storage save response message
typedef struct {
    uint32_t storage_key;       // Extended CAN ID used as storage key (4 bytes)
    uint8_t success;            // 1=success, 0=failed (1 byte)
    uint8_t reserved[3];        // Padding to 8 bytes
} __attribute__((packed)) storage_save_response_msg_t;

// Storage error message
typedef struct {
    uint32_t storage_key;       // Extended CAN ID used as storage key (4 bytes)
    uint8_t error_code;         // Error code (1 byte)
    uint8_t reserved[3];        // Padding to 8 bytes
} __attribute__((packed)) storage_error_msg_t;

// Storage statistics message
typedef struct {
    uint32_t cache_hits;        // Number of cache hits
    uint32_t cache_misses;      // Number of cache misses
    uint32_t disk_writes;       // Number of disk writes
    uint32_t disk_reads;        // Number of disk reads
    uint16_t cache_size;        // Current cache entries
    uint16_t free_space_kb;     // Free storage space (KB)
} storage_stats_msg_t;

// Storage export request message
typedef struct {
    uint8_t export_type;        // 0=all, 1=fuel_map, 2=ignition_map, 3=settings
    uint8_t ecu_filter;         // ECU base filter (0=all, 1=primary, etc.)
    uint8_t subsystem_filter;   // Subsystem filter (0=all, 1=fuel, 2=ignition, etc.)
    uint8_t reserved[5];        // Padding to 8 bytes
} storage_export_request_msg_t;

// Storage export start message
typedef struct {
    uint16_t total_keys;        // Total number of keys to export
    uint8_t success;            // 1=ready, 0=error
    uint8_t reserved[5];        // Padding to 8 bytes
} storage_export_start_msg_t;

// Storage export key-value message
typedef struct {
    uint32_t storage_key;       // Extended CAN ID key
    float value;                // Value
} storage_export_key_msg_t;

// Storage export complete message
typedef struct {
    uint16_t keys_sent;         // Actual number of keys sent
    uint8_t success;            // 1=complete, 0=error
    uint8_t reserved[5];        // Padding to 8 bytes
} storage_export_complete_msg_t;

// =============================================================================
// PARAMETER MESSAGE STRUCTURES (FOR DIRECT CAN ID ACCESS)
// =============================================================================

// Parameter operation flags
#define PARAM_OP_STATUS_BROADCAST   0x00    // ECU→External: Current value broadcast
#define PARAM_OP_READ_REQUEST       0x01    // External→ECU: Request current value
#define PARAM_OP_WRITE_REQUEST      0x02    // External→ECU: Write new value
#define PARAM_OP_READ_RESPONSE      0x03    // ECU→External: Response to read request
#define PARAM_OP_WRITE_ACK          0x04    // ECU→External: Write acknowledgment
#define PARAM_OP_ERROR              0x05    // ECU→External: Operation error

// Parameter operation message (for direct CAN ID access)
typedef struct {
    uint8_t operation;          // Operation type (see PARAM_OP_* flags)
    float value;                // Parameter value (4 bytes)
    uint8_t source_channel;     // Which external interface sent request (1 byte)
    uint8_t request_id;         // Unique request ID for that channel (1 byte)
    uint8_t reserved[1];        // Future use (1 byte)
} __attribute__((packed)) parameter_msg_t;

// =============================================================================
// CHANNEL IDENTIFIERS FOR REQUEST-RESPONSE ROUTING
// =============================================================================

// Channel IDs for routing parameter requests/responses
#define CHANNEL_SERIAL_USB    0x01
#define CHANNEL_SERIAL_1      0x02
#define CHANNEL_SERIAL_2      0x03
#define CHANNEL_CAN_BUS       0x04

// Parameter error codes
#define PARAM_ERROR_SUCCESS             0x00
#define PARAM_ERROR_READ_ONLY           0x01    // Parameter is read-only
#define PARAM_ERROR_OUT_OF_RANGE        0x02    // Value outside valid range
#define PARAM_ERROR_INVALID_OPERATION   0x03    // Invalid operation for this parameter
#define PARAM_ERROR_SYSTEM_BUSY         0x04    // System too busy to process
#define PARAM_ERROR_PERMISSION_DENIED   0x05    // Insufficient permissions
#define PARAM_ERROR_WRITE_FAILED        0x06    // Write operation failed

// Parameter error message
typedef struct {
    uint8_t operation;          // Original operation that failed
    uint8_t error_code;         // Error code (see PARAM_ERROR_* codes)
    float attempted_value;      // Value that was attempted (for write errors)
    uint8_t reserved[2];        // Reserved for future use  
} __attribute__((packed)) parameter_error_msg_t;



// =============================================================================
// CONSTANTS AND ENUMS
// =============================================================================

// Storage error codes
#define STORAGE_ERROR_SUCCESS           0x00
#define STORAGE_ERROR_KEY_NOT_FOUND     0x01
#define STORAGE_ERROR_STORAGE_FULL      0x02
#define STORAGE_ERROR_WRITE_FAILED      0x03
#define STORAGE_ERROR_READ_FAILED       0x04
#define STORAGE_ERROR_INVALID_KEY       0x05
#define STORAGE_ERROR_CACHE_FULL        0x06

// Engine status bitfield definitions
#define ENGINE_STATUS_RUNNING   0x01
#define ENGINE_STATUS_STARTING  0x02
#define ENGINE_STATUS_STOPPED   0x04
#define ENGINE_STATUS_ERROR     0x08
#define ENGINE_STATUS_LIMP_MODE 0x10

// Error code bitfield definitions
#define ERROR_COOLANT_OVERHEAT  0x0001
#define ERROR_LOW_OIL_PRESSURE  0x0002
#define ERROR_MAP_SENSOR_FAULT  0x0004
#define ERROR_RPM_SENSOR_FAULT  0x0008
#define ERROR_IGNITION_FAULT    0x0010
#define ERROR_FUEL_SYSTEM_FAULT 0x0020

// =============================================================================
// HELPER MACROS
// =============================================================================

// Helper macros for packing data into CAN messages
#define MSG_PACK_FLOAT(msg, val) do { \
    float temp = (val); \
    (msg)->len = sizeof(float); \
    memcpy((msg)->buf, &temp, sizeof(float)); \
} while(0)

#define MSG_PACK_UINT32(msg, val) do { \
    uint32_t temp = (val); \
    (msg)->len = sizeof(uint32_t); \
    memcpy((msg)->buf, &temp, sizeof(uint32_t)); \
} while(0)

#define MSG_PACK_UINT16(msg, val) do { \
    uint16_t temp = (val); \
    (msg)->len = sizeof(uint16_t); \
    memcpy((msg)->buf, &temp, sizeof(uint16_t)); \
} while(0)

#define MSG_PACK_UINT8(msg, val) do { \
    uint8_t temp = (val); \
    (msg)->len = sizeof(uint8_t); \
    (msg)->buf[0] = temp; \
} while(0)

// Helper macros for unpacking data from CAN messages
#define MSG_UNPACK_FLOAT(msg) (*(float*)(msg)->buf)
#define MSG_UNPACK_UINT32(msg) (*(uint32_t*)(msg)->buf)
#define MSG_UNPACK_UINT16(msg) (*(uint16_t*)(msg)->buf)
#define MSG_UNPACK_UINT8(msg) ((msg)->buf[0])

// Storage message packing macros
#define MSG_PACK_STORAGE_SAVE_FLOAT(msg, key, val) do { \
    storage_save_float_msg_t data = {0}; \
    data.storage_key = (key); \
    data.value = (val); \
    (msg)->len = sizeof(storage_save_float_msg_t); \
    memcpy((msg)->buf, &data, sizeof(storage_save_float_msg_t)); \
} while(0)

#define MSG_PACK_STORAGE_LOAD_FLOAT(msg, key, def_val) do { \
    storage_load_float_msg_t data = {0}; \
    data.storage_key = (key); \
    data.default_value = (def_val); \
    (msg)->len = sizeof(storage_load_float_msg_t); \
    memcpy((msg)->buf, &data, sizeof(storage_load_float_msg_t)); \
} while(0)

#define MSG_UNPACK_STORAGE_SAVE_FLOAT(msg) ((storage_save_float_msg_t*)(msg)->buf)
#define MSG_UNPACK_STORAGE_LOAD_FLOAT(msg) ((storage_load_float_msg_t*)(msg)->buf)
#define MSG_UNPACK_STORAGE_LOAD_RESPONSE(msg) ((storage_load_response_msg_t*)(msg)->buf)
#define MSG_UNPACK_STORAGE_SAVE_RESPONSE(msg) ((storage_save_response_msg_t*)(msg)->buf)

// Message handler function pointer type
typedef void (*MessageHandler)(const CANMessage* msg);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Check if a CAN ID is extended (29-bit) - all our IDs are extended now
inline bool is_extended_can_id(uint32_t can_id) {
    return (can_id & 0xFFFF0000) != 0;
}

// Create an extended CAN message (all messages are extended now)
inline void create_extended_can_message(CANMessage* msg, uint32_t extended_id, 
                                      const void* data, uint8_t length) {
    msg->id = extended_id;
    msg->len = length;
    if (data && length > 0) {
        memcpy(msg->buf, data, length);
    }
    #ifdef ARDUINO
    // FlexCAN timestamp is 16-bit, so we need to truncate micros() to 16 bits
    msg->timestamp = (uint16_t)(micros() & 0xFFFF);
    msg->flags.extended = 1;
    msg->flags.remote = 0;
    
    // Debug output for timestamp (disabled to reduce serial clutter)
    /*
    static uint32_t last_timestamp_debug = 0;
    uint32_t now = millis();
    if (now - last_timestamp_debug >= 5000) {
        Serial.print("DEBUG: create_extended_can_message - timestamp set to: ");
        Serial.print(msg->timestamp);
        Serial.print(" (micros truncated to 16-bit)");
        Serial.println();
        last_timestamp_debug = now;
    }
    */
    #else
    msg->timestamp = 0;
    msg->flags.extended = 1;
    msg->flags.remote = 0;
    #endif
}

// Backwards compatibility - all messages are extended now
inline void create_standard_can_message(CANMessage* msg, uint32_t standard_id, 
                                       const void* data, uint8_t length) {
    // Convert to extended format for consistency
    create_extended_can_message(msg, standard_id, data, length);
}

#endif