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
#define CAN_TX_PIN GPIO_NUM_3
#define CAN_RX_PIN GPIO_NUM_2

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

// ECU Parameter IDs - Custom ECU (Extended IDs)
#define PARAM_FLUID_TEMP     0x10500001
#define PARAM_CURRENT_GEAR   0x10500101
#define PARAM_DRIVE_GEAR     0x10500104
#define PARAM_VEHICLE_SPEED  0x10300002
#define PARAM_SHIFT_SOL_A    0x10500110
#define PARAM_SHIFT_SOL_B    0x10500111
#define PARAM_OVERRUN_SOL    0x10500112
#define PARAM_PRESSURE_SOL   0x10500113
#define PARAM_LOCKUP_SOL     0x10500114

// Haltech CAN IDs (Standard 11-bit IDs)
#define HALTECH_ENGINE_DATA_1    0x360  // RPM, MAP, TPS, Coolant Pressure
#define HALTECH_ENGINE_DATA_2    0x361  // Fuel, Oil, Engine Demand, Wastegate pressures  
#define HALTECH_TEMPERATURE_DATA 0x3E0  // Coolant, Air, Fuel, Oil temperatures

// Virtual Parameter IDs for Haltech data (for internal tracking)
#define PARAM_COOLANT_TEMP   0x999001  // From Haltech 0x3E0
#define PARAM_OIL_PRESSURE   0x999002  // From Haltech 0x361

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

// Haltech Parameter Functions
float CAN_GetCoolantTemperature();  // Coolant temp in Celsius
float CAN_GetOilPressure();         // Oil pressure in kPa (gauge)

// Haltech Data Freshness Functions
bool CAN_IsHaltechTempFresh(unsigned long max_age_seconds);
bool CAN_IsHaltechPressureFresh(unsigned long max_age_seconds);

// Status Functions
bool CAN_IsInitialized();
CANStats CAN_GetStats();

// Add this to the Public Functions section in ECU_CAN.h
void CAN_PrintDetailedDebug();


#endif // ECU_CAN_H