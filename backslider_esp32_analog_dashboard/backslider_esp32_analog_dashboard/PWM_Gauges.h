/*
 * PWM_Gauges.h - PWM Gauge Control Module
 * 
 * Controls analog gauges using PWM signals
 * Uses 2N2222 transistors to PWM the ground of gauges
 * 
 * Features:
 * - Multiple gauge channels
 * - Individual calibration per gauge
 * - Smooth transitions
 * - Value mapping and limiting
 */

#ifndef PWM_GAUGES_H
#define PWM_GAUGES_H

#include <Arduino.h>

// Maximum number of gauge channels
#define MAX_GAUGE_CHANNELS 8

// PWM Configuration
#define PWM_FREQUENCY 1000    // 1kHz PWM frequency (good for gauges)
#define PWM_RESOLUTION 12     // 12-bit resolution (0-4095)
#define PWM_MAX_VALUE 4095    // Maximum PWM value

// Gauge Channel Structure
struct GaugeChannel {
    int gpio_pin;           // GPIO pin for PWM output
    int pwm_channel;        // ESP32 PWM channel (0-15)
    bool enabled;           // Channel enabled flag
    
    // Calibration parameters
    float min_input;        // Minimum input value (e.g., 0 kph)
    float max_input;        // Maximum input value (e.g., 200 kph)
    int min_pwm;           // PWM value for minimum input (0-4095)
    int max_pwm;           // PWM value for maximum input (0-4095)
    
    // Current state
    float current_value;    // Last input value
    int current_pwm;       // Current PWM output
    unsigned long last_update;
    
    // Smoothing
    bool smooth_enabled;    // Enable smooth transitions
    float smooth_factor;    // Smoothing factor (0.0-1.0, higher = smoother)
    
    const char* name;       // Gauge name for debugging
};

// Public Functions
bool PWM_Initialize();
int PWM_AddGauge(int gpio_pin, const char* name, float min_input, float max_input, 
                 int min_pwm = 0, int max_pwm = PWM_MAX_VALUE);
void PWM_Update();
void PWM_SetGaugeValue(int channel, float value);
void PWM_SetGaugeValueSmooth(int channel, float value);
void PWM_CalibrateGauge(int channel, float min_input, float max_input, int min_pwm, int max_pwm);
void PWM_EnableSmoothing(int channel, float smooth_factor = 0.9);
void PWM_DisableSmoothing(int channel);
void PWM_SetGaugeEnabled(int channel, bool enabled);
void PWM_TestGauge(int channel, int test_pwm_value);
void PWM_SweepGauge(int channel);
void PWM_PrintStatus();

// Convenience Functions for Common Gauges
int PWM_AddSpeedGauge(int gpio_pin, float max_speed = 200.0);
int PWM_AddTachGauge(int gpio_pin, float max_rpm = 8000.0);
int PWM_AddTempGauge(int gpio_pin, float min_temp = 0.0, float max_temp = 120.0);
int PWM_AddFuelGauge(int gpio_pin); // 0-100%
int PWM_AddVoltGauge(int gpio_pin, float min_volt = 8.0, float max_volt = 16.0);

// Utility Functions
float PWM_GetGaugeValue(int channel);
int PWM_GetGaugePWM(int channel);
bool PWM_IsGaugeEnabled(int channel);
const char* PWM_GetGaugeName(int channel);

// Advanced Functions
void PWM_SetAllGauges(float value);      // Set all gauges to same value
void PWM_TestAllGauges();                // Test sweep all gauges
void PWM_DisableAllGauges();             // Disable all gauge outputs
void PWM_EnableAllGauges();              // Enable all gauge outputs

#endif // PWM_GAUGES_H