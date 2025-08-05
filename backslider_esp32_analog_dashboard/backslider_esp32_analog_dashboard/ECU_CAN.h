/*
 * ECU_CAN.h - CAN Bus ECU Communication Header
 * 
 * Handles all CAN bus communication with ECU parameters
 * For ESP32-S3 Zero with GPIO 7 (TX) and GPIO 8 (RX)
 */

#ifndef ECU_CAN_H
#define ECU_CAN_H

#include <Arduino.h>
#include "driver/twai.h"

// CAN Configuration
#define CAN_TX_PIN GPIO_NUM_7
#define CAN_RX_PIN GPIO_NUM_8

// Protocol Constants
#define READ_REQUEST  0x01
#define READ_RESPONSE 0x03

// ECU Parameter Structure
struct ECUParameter {
    uint32_t can_id;
    const char* name;
    const char* unit;
    float last_value;
    unsigned long last_update;
    bool has_data;
};

// ECU Parameter IDs
#define PARAM_FLUID_TEMP     0x10500001
#define PARAM_CURRENT_GEAR   0x10500101
#define PARAM_DRIVE_GEAR     0x10500104
#define PARAM_VEHICLE_SPEED  0x10300002
#define PARAM_SHIFT_SOL_A    0x10500110
#define PARAM_SHIFT_SOL_B    0x10500111
#define PARAM_OVERRUN_SOL    0x10500112
#define PARAM_PRESSURE_SOL   0x10500113
#define PARAM_LOCKUP_SOL     0x10500114

// Statistics Structure
struct CANStats {
    uint32_t total_requests;
    uint32_t total_responses;
    uint32_t successful_responses;
    unsigned long start_time;
};

// Public Functions
bool CAN_Initialize();
void CAN_Update();
void CAN_PrintStatus();

// Parameter Access Functions
float CAN_GetParameterValue(uint32_t can_id);
bool CAN_IsParameterFresh(uint32_t can_id, unsigned long max_age_seconds);
const char* CAN_GetParameterName(uint32_t can_id);
const char* CAN_GetParameterUnit(uint32_t can_id);
unsigned long CAN_GetParameterAge(uint32_t can_id);

// Convenience Functions for Common Parameters
float CAN_GetFluidTemperature();
float CAN_GetCurrentGear();
float CAN_GetDriveGear();
float CAN_GetVehicleSpeed();
float CAN_GetShiftSolenoidA();
float CAN_GetShiftSolenoidB();
float CAN_GetOverrunSolenoid();
float CAN_GetPressureSolenoid();
float CAN_GetLockupSolenoid();

// Status Functions
bool CAN_IsInitialized();
CANStats CAN_GetStats();

#endif // ECU_CAN_H