// msg_definitions.h
// CAN-style message definitions for Backslider ECU

#ifndef MSG_DEFINITIONS_H
#define MSG_DEFINITIONS_H

#include <stdint.h>
#include <string.h>  // for memcpy

// Use FlexCAN message format on Arduino, mock for desktop testing
#ifdef ARDUINO
    #include <FlexCAN_T4.h>
    typedef CAN_message_t CANMessage;
#else
    // Mock CAN message structure for desktop testing
    typedef struct {
        uint32_t id;          // Message ID (11-bit)
        uint8_t len;          // Data length (0-8 bytes)
        uint8_t buf[8];       // Data payload
        uint32_t timestamp;   // Timestamp (for compatibility)
    } CANMessage;
#endif

// CAN Message IDs - Organized by priority (lower ID = higher priority)

// High Priority: Critical real-time data (0x010-0x0FF)
#define MSG_ENGINE_RPM          0x010    // Engine RPM (float, Hz)
#define MSG_CRANK_POSITION      0x011    // Crank angle (uint16_t, degrees)
#define MSG_THROTTLE_POSITION   0x012    // Throttle % (float, 0-100)
#define MSG_MANIFOLD_PRESSURE   0x013    // MAP (float, kPa)
#define MSG_TIMING_TRIGGER      0x014    // Timing trigger event (uint8_t)
#define MSG_VEHICLE_SPEED       0x015    // Vehicle speed (float, MPH)
#define MSG_BRAKE_PEDAL         0x016    // Brake pedal state (float, 0=off 1=on)

// Medium Priority: Control commands (0x100-0x1FF)
#define MSG_IGNITION_TIMING     0x100    // Timing advance (float, degrees)
#define MSG_FUEL_PULSE_WIDTH    0x101    // Injection time (float, ms)
#define MSG_IDLE_TARGET_RPM     0x102    // Target idle (uint16_t, RPM)
#define MSG_BOOST_TARGET        0x103    // Boost target (float, kPa)

// Low Priority: Status/diagnostics (0x200-0x2FF)
#define MSG_COOLANT_TEMP        0x200    // Coolant temp (float, °C)
#define MSG_AIR_INTAKE_TEMP     0x201    // IAT (float, °C)
#define MSG_BATTERY_VOLTAGE     0x202    // Battery (float, volts)
#define MSG_OIL_PRESSURE        0x203    // Oil pressure (float, kPa)
#define MSG_ENGINE_STATUS       0x204    // Status flags (uint8_t, bitfield)
#define MSG_ERROR_CODES         0x205    // Error codes (uint16_t, bitfield)

// System Messages (0x300-0x3FF)
#define MSG_HEARTBEAT           0x300    // Module heartbeat (uint32_t, counter)
#define MSG_SYSTEM_TIME         0x301    // System uptime (uint32_t, ms)
#define MSG_DEBUG_MESSAGE       0x302    // Debug info (uint32_t)

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
// TRANSMISSION MESSAGE IDs
// =============================================================================

// Transmission sensor inputs (0x400-0x40F range to avoid conflicts)
#define MSG_TRANS_FLUID_TEMP        0x400   // Transmission fluid temperature
#define MSG_PADDLE_UPSHIFT          0x401   // Upshift paddle pressed
#define MSG_PADDLE_DOWNSHIFT        0x402   // Downshift paddle pressed
#define MSG_TRANS_PARK_SWITCH       0x403   // Park position switch
#define MSG_TRANS_REVERSE_SWITCH    0x404   // Reverse position switch
#define MSG_TRANS_NEUTRAL_SWITCH    0x405   // Neutral position switch
#define MSG_TRANS_DRIVE_SWITCH      0x406   // Drive position switch
#define MSG_TRANS_SECOND_SWITCH     0x407   // Second gear position switch
#define MSG_TRANS_FIRST_SWITCH      0x408   // First gear position switch
#define MSG_TRANS_INPUT_SPEED       0x409   // Transmission input shaft speed (RPM)
#define MSG_TRANS_OUTPUT_SPEED      0x40A   // Transmission output shaft speed (RPM)

// Combined transmission state messages (0x410-0x41F range)
#define MSG_TRANS_CURRENT_GEAR      0x410   // Current gear position
#define MSG_TRANS_SHIFT_REQUEST     0x411   // Shift request (up/down)
#define MSG_TRANS_STATE_VALID       0x412   // Transmission state validity
#define MSG_TRANS_OVERRUN_STATE     0x413   // Current overrun clutch state (0=ENGAGED, 1=DISENGAGED)


// =============================================================================
// OUTPUT CONTROL MESSAGE IDs
// =============================================================================

// Engine output controls (0x500-0x50F range)
#define MSG_IGNITION_COIL_1         0x500   // Ignition coil 1 control
#define MSG_IGNITION_COIL_2         0x501   // Ignition coil 2 control
#define MSG_IGNITION_COIL_3         0x502   // Ignition coil 3 control
#define MSG_IGNITION_COIL_4         0x503   // Ignition coil 4 control
#define MSG_IGNITION_COIL_5         0x504   // Ignition coil 5 control
#define MSG_IGNITION_COIL_6         0x505   // Ignition coil 6 control
#define MSG_IGNITION_COIL_7         0x506   // Ignition coil 7 control
#define MSG_IGNITION_COIL_8         0x507   // Ignition coil 8 control
#define MSG_FUEL_INJECTOR_1         0x508   // Fuel injector 1 control
#define MSG_FUEL_INJECTOR_2         0x509   // Fuel injector 2 control
#define MSG_FUEL_INJECTOR_3         0x50A   // Fuel injector 3 control
#define MSG_FUEL_INJECTOR_4         0x50B   // Fuel injector 4 control
#define MSG_FUEL_INJECTOR_5         0x50C   // Fuel injector 5 control
#define MSG_FUEL_INJECTOR_6         0x50D   // Fuel injector 6 control
#define MSG_FUEL_INJECTOR_7         0x50E   // Fuel injector 7 control
#define MSG_FUEL_INJECTOR_8         0x50F   // Fuel injector 8 control

// Transmission output controls (0x510-0x51F range)
#define MSG_TRANS_SHIFT_SOL_A       0x510   // Shift Solenoid A control
#define MSG_TRANS_SHIFT_SOL_B       0x511   // Shift Solenoid B control
#define MSG_TRANS_OVERRUN_SOL       0x512   // Overrun solenoid control
#define MSG_TRANS_PRESSURE_SOL      0x513   // Line pressure solenoid control (PWM)
#define MSG_TRANS_LOCKUP_SOL        0x514   // Lockup solenoid control

// Auxiliary output controls (0x520-0x52F range)
#define MSG_IDLE_VALVE_CONTROL      0x520   // Idle air control valve
#define MSG_FUEL_PUMP_CONTROL       0x521   // Fuel pump relay control
#define MSG_FAN_CONTROL             0x522   // Cooling fan control
#define MSG_A_C_CLUTCH_CONTROL      0x523   // A/C compressor clutch
#define MSG_ALTERNATOR_FIELD        0x524   // Alternator field control
#define MSG_BOOST_CONTROL           0x525   // Boost control solenoid
#define MSG_WASTEGATE_CONTROL       0x526   // Wastegate control
#define MSG_SHIFT_LIGHT             0x527   // Shift light control
#define MSG_STATUS_LED              0x528   // Status LED control

// Gauge output controls (0x530-0x53F range)
#define MSG_BOOST_GAUGE             0x530   // Boost gauge output
#define MSG_TEMP_GAUGE              0x531   // Temperature gauge output
#define MSG_FUEL_GAUGE              0x532   // Fuel level gauge output
#define MSG_OIL_PRESSURE_GAUGE      0x533   // Oil pressure gauge output
#define MSG_TACH_OUTPUT             0x534   // Tachometer output
#define MSG_SPEEDO_OUTPUT           0x535   // Speedometer output

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

// Message handler function pointer type
typedef void (*MessageHandler)(const CANMessage* msg);

#endif