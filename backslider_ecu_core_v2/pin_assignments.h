// pin_assignments.h
// Central pin assignment definitions for Teensy 4.1 ECU
//
// This file contains ALL pin assignments for the ECU project.
// Organized by function to prevent conflicts and make wiring clear.
//
// Teensy 4.1 Pin Reference:
// - Analog: A0-A17 (pins 14-17, 18-23, 24-27, 38-41)
// - Digital: 0-55 (many pins are dual-purpose)
// - Interrupt capable: Most digital pins
// - PWM capable: Multiple pins with hardware timers

#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

// =============================================================================
// ANALOG SENSOR INPUTS (Primary Engine Sensors)
// =============================================================================

// Core engine sensors - high priority analog inputs
#define PIN_TPS                A0    // Throttle Position Sensor (0.5-4.5V)
#define PIN_MAP                A1    // Manifold Absolute Pressure (0.5-4.5V)
#define PIN_MAF                A2    // Mass Air Flow Sensor (0-5V)
#define PIN_CTS                A3    // Coolant Temperature Sensor (thermistor)
#define PIN_IAT                A4    // Intake Air Temperature (thermistor)
#define PIN_BATTERY_VOLTAGE    A5    // Battery Voltage (voltage divider)

// Secondary engine sensors
#define PIN_OIL_PRESSURE       A6    // Oil Pressure Sensor (0.5-4.5V)
#define PIN_FUEL_PRESSURE      A7    // Fuel Rail Pressure (0.5-4.5V)
#define PIN_BOOST_PRESSURE     A8    // Boost/Turbo Pressure (0.5-4.5V)
#define PIN_EGT_1              A9    // Exhaust Gas Temperature #1 (thermocouple)
#define PIN_EGT_2              A10   // Exhaust Gas Temperature #2 (thermocouple)

// Lambda/O2 sensors
#define PIN_LAMBDA_1           A11   // Primary Lambda/O2 Sensor
#define PIN_LAMBDA_2           A12   // Secondary Lambda/O2 Sensor

// Transmission sensors
#define PIN_TRANS_FLUID_TEMP   A13   // Transmission Fluid Temperature (thermistor)
#define PIN_TRANS_INPUT_SPEED  A14   // Transmission Input Shaft Speed (analog)
#define PIN_TRANS_OUTPUT_SPEED A15   // Transmission Output Shaft Speed (analog)
#define PIN_TRANS_LINE_PRESSURE A16  // Transmission Line Pressure (0.5-4.5V)

// Additional sensors
#define PIN_AMBIENT_TEMP       A17   // Ambient Air Temperature (thermistor)

// =============================================================================
// CRITICAL TIMING INPUTS (Interrupt-driven)
// =============================================================================

// Crankshaft position sensors - MUST be interrupt-capable pins
#define PIN_CRANK_PRIMARY      2     // Primary crank position sensor (VR or Hall)
#define PIN_CRANK_SECONDARY    3     // Secondary crank sensor (redundancy)
#define PIN_CRANK_SYNC         5     // Crank sync/index sensor

// Camshaft position sensors
#define PIN_CAM_INTAKE         20    // Intake camshaft position
#define PIN_CAM_EXHAUST        21    // Exhaust camshaft position

// Vehicle speed sensor
#define PIN_VEHICLE_SPEED      6     // Vehicle Speed Sensor (Hall effect)

// Knock sensors (high frequency, interrupt capable)
#define PIN_KNOCK_1            7     // Knock Sensor #1
#define PIN_KNOCK_2            8     // Knock Sensor #2

// =============================================================================
// TRANSMISSION INPUTS (Digital and Analog)
// =============================================================================

// Paddle shifter inputs (manual shifting)
#define PIN_PADDLE_UPSHIFT     16    // Upshift paddle (momentary, ground trigger)
#define PIN_PADDLE_DOWNSHIFT   17    // Downshift paddle (momentary, ground trigger)

// Automatic transmission gear selector inputs
// These read the transmission gear selector switch positions
#define PIN_TRANS_PARK         22    // Park position switch (ground trigger)
#define PIN_TRANS_REVERSE      23    // Reverse position switch (ground trigger)
#define PIN_TRANS_NEUTRAL      24    // Neutral position switch (ground trigger)
#define PIN_TRANS_DRIVE        25    // Drive position switch (ground trigger)
#define PIN_TRANS_SECOND       26    // Second gear position switch (ground trigger)
#define PIN_TRANS_FIRST        27    // First gear position switch (ground trigger)

// Note: Removed PIN_TRANS_SPORT and PIN_TRANS_MANUAL to make room for
// PIN_TRANS_SECOND and PIN_TRANS_FIRST for proper 6-position gear selector

// =============================================================================
// ENGINE CONTROL OUTPUTS (PWM/Digital)
// =============================================================================

// Fuel injection outputs - V8 configuration
#define PIN_INJ_1              28    // Fuel Injector #1 (Cylinder 1)
#define PIN_INJ_2              29    // Fuel Injector #2 (Cylinder 2)
#define PIN_INJ_3              30    // Fuel Injector #3 (Cylinder 3)
#define PIN_INJ_4              31    // Fuel Injector #4 (Cylinder 4)
#define PIN_INJ_5              32    // Fuel Injector #5 (Cylinder 5)
#define PIN_INJ_6              33    // Fuel Injector #6 (Cylinder 6)
#define PIN_INJ_7              34    // Fuel Injector #7 (Cylinder 7)
#define PIN_INJ_8              35    // Fuel Injector #8 (Cylinder 8)

// Ignition outputs - V8 configuration (could be coil-on-plug or wasted spark)
#define PIN_IGN_1              36    // Ignition Coil #1 (Cylinder 1)
#define PIN_IGN_2              37    // Ignition Coil #2 (Cylinder 2)
#define PIN_IGN_3              38    // Ignition Coil #3 (Cylinder 3)
#define PIN_IGN_4              39    // Ignition Coil #4 (Cylinder 4)
#define PIN_IGN_5              9     // Ignition Coil #5 (Cylinder 5)
#define PIN_IGN_6              10    // Ignition Coil #6 (Cylinder 6)
#define PIN_IGN_7              11    // Ignition Coil #7 (Cylinder 7)
#define PIN_IGN_8              12    // Ignition Coil #8 (Cylinder 8)

// =============================================================================
// TRANSMISSION CONTROL OUTPUTS
// =============================================================================

// Transmission solenoid outputs
#define PIN_TRANS_SHIFT_SOL_A  40    // Shift Solenoid A
#define PIN_TRANS_SHIFT_SOL_B  41    // Shift Solenoid B
#define PIN_TRANS_OVERRUN_SOL  42    // Overrun Solenoid

// Transmission pressure control
#define PIN_TRANS_PRESSURE_SOL 43    // Line Pressure Solenoid (PWM)
#define PIN_TRANS_LOCKUP_SOL   44    // Lockup Solenoid

// =============================================================================
// AUXILIARY CONTROL OUTPUTS
// =============================================================================

// Engine auxiliary controls
#define PIN_IDLE_VALVE        46     // Idle Air Control Valve (PWM)
#define PIN_FUEL_PUMP         47     // Fuel Pump Relay
#define PIN_FAN_CONTROL       48     // Cooling Fan Control (PWM)
#define PIN_A_C_CLUTCH        49     // A/C Compressor Clutch
#define PIN_ALTERNATOR_FIELD  50     // Alternator Field Control

// Boost control (turbo/supercharger)
#define PIN_BOOST_CONTROL     51     // Boost Control Solenoid (PWM)
#define PIN_WASTEGATE         52     // Wastegate Control

// =============================================================================
// COMMUNICATION INTERFACES
// =============================================================================

// CAN Bus (built into Teensy 4.1)
#define PIN_CAN1_RX           0      // CAN1 RX (built-in)
#define PIN_CAN1_TX           1      // CAN1 TX (built-in)
#define PIN_CAN2_RX           4      // CAN2 RX (if using second CAN)
#define PIN_CAN2_TX           5      // CAN2 TX (if using second CAN)

// =============================================================================
// DIAGNOSTIC AND DEBUG
// =============================================================================

// LED indicators
#define PIN_STATUS_LED        13     // Built-in LED for status
#define PIN_ERROR_LED         53     // Error indicator LED
#define PIN_HEARTBEAT_LED     54     // Heartbeat indicator LED

// Debug/test points
#define PIN_DEBUG_1           55     // Debug output #1

// =============================================================================
// PIN VALIDATION MACROS
// =============================================================================

// Validate that critical pins are interrupt-capable (only in Arduino environment)
#ifdef ARDUINO
#if !defined(CORE_INT2_PIN) || (PIN_CRANK_PRIMARY != 2)
    #warning "PIN_CRANK_PRIMARY should be on interrupt-capable pin"
#endif

#if !defined(CORE_INT3_PIN) || (PIN_CRANK_SECONDARY != 3)
    #warning "PIN_CRANK_SECONDARY should be on interrupt-capable pin"
#endif
#endif

// =============================================================================
// PIN USAGE SUMMARY (for documentation)
// =============================================================================

/*
ANALOG PINS USAGE:
A0-A5:   Primary engine sensors (TPS, MAP, MAF, CTS, IAT, Battery)
A6-A12:  Secondary engine sensors (Oil, Fuel, Boost, EGT, Lambda)
A13-A16: Transmission sensors (Temp, Input Speed, Output Speed, Line Pressure)
A17:     Ambient temperature

DIGITAL PINS USAGE:
0-1:     CAN bus (reserved)
2-3,5:   Critical timing inputs (crank position)
4:       CAN2 RX (alternative use)
6-8:     Speed and knock sensors
9-12:    Ignition outputs 5-8 (V8 configuration)
13:      Built-in status LED
14-15:   Analog inputs A0-A1 (TPS, MAP)
16-17:   Paddle shifter inputs (upshift/downshift)
18-19:   Analog inputs A2-A3 (MAF, CTS)
20-21:   Cam position sensors
22-27:   Transmission gear selector switches (P,R,N,D,2,1)
28-35:   Fuel injection (8 cylinders) and ignition 1-4
36-39:   Ignition outputs 1-4 (V8 configuration)
40-45:   Transmission control outputs
46-52:   Auxiliary engine controls
53-55:   Status LEDs and debug

V8 ENGINE CONFIGURATION:
Injectors:  Pins 28-35 (8 injectors)
Ignition:   Pins 36-39, 9-12 (8 coils)
  - Can support coil-on-plug or wasted spark configurations
  - Firing order can be configured in software

TRANSMISSION CONFIGURATION:
Inputs:     A13 (fluid temp), 16-17 (paddles), 22-27 (gear selector)
Outputs:    40-45 (solenoid control)
  - Supports 6-position gear selector: Park, Reverse, Neutral, Drive, Second, First
  - Manual paddle shifting via pins 16-17
  - Fluid temperature monitoring via thermistor on A13

INTERRUPT PRIORITIES (suggested):
Highest: Crank position sensors (2, 3, 5)
High:    Cam position sensors (20, 21)
Medium:  Knock sensors (7, 8)
Low:     Vehicle speed (6), Paddle shifters (16, 17)
*/

#endif