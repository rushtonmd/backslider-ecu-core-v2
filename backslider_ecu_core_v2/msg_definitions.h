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

// Medium Priority: Control commands (0x100-0x1FF)
#define MSG_IGNITION_TIMING     0x100    // Timing advance (float, degrees)
#define MSG_FUEL_PULSE_WIDTH    0x101    // Injection time (float, ms)
#define MSG_IDLE_TARGET_RPM     0x102    // Target idle (uint16_t, RPM)
#define MSG_BOOST_TARGET        0x103    // Boost target (float, kPa)
#define MSG_FAN_CONTROL         0x104    // Cooling fan (uint8_t, 0-255)

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