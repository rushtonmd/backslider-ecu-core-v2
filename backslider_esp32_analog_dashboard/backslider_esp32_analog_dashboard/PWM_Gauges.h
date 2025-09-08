/*
 * PWM_Gauges.h - Dual-Mode Gauge Control Module
 * 
 * Supports both frequency-based VSS signals and voltage-based analog gauges
 * Auto-detects mode based on frequency range:
 * - max_freq > 255: LEDC frequency mode (for speedometers, VSS)
 * - max_freq ≤ 255: analogWrite voltage mode (for temp, oil pressure, etc.)
 */

#ifndef PWM_GAUGES_H
#define PWM_GAUGES_H

#include <Arduino.h>

// Maximum number of gauge channels
#define MAX_GAUGE_CHANNELS 8

// Gauge modes
#define GAUGE_MODE_FREQUENCY 0  // LEDC frequency mode (for VSS/speedometers)
#define GAUGE_MODE_VOLTAGE   1  // analogWrite voltage mode (for temp/pressure)

// PWM Configuration for frequency-based gauges (VSS)
#define PWM_RESOLUTION 14     // 14-bit resolution for low frequency support (1Hz+)
#define PWM_DUTY_CYCLE 8192   // 50% duty cycle for 14-bit (8192 out of 16383)
#define PWM_MAX_VALUE 16383   // Maximum PWM value for 14-bit

// Analog Configuration for voltage-based gauges
#define ANALOG_MAX_VALUE 255  // Maximum analogWrite value (0-255)

// Dual-Mode Gauge Channel Structure  
struct GaugeChannel {
    int gpio_pin;           // GPIO pin for output
    bool enabled;           // Channel enabled flag
    bool is_frequency_mode; // True = LEDC frequency, False = analogWrite voltage
    
    // Calibration parameters
    float min_input;        // Minimum input value (e.g., 0 mph, 0°C)
    float max_input;        // Maximum input value (e.g., 200 mph, 120°C)
    int min_pwm;           // PWM/analog value for minimum input (legacy)
    int max_pwm;           // PWM/analog value for maximum input (legacy)
    int min_freq;          // Frequency/voltage for minimum input
    int max_freq;          // Frequency/voltage for maximum input
    
    // Current state
    float current_value;    // Last input value
    int current_pwm;       // Current PWM/analog output
    int current_freq;      // Current output frequency/voltage
    unsigned long last_update;
    
    // Smoothing
    bool smooth_enabled;    // Enable smooth transitions
    float smooth_factor;    // Smoothing factor (0.0-1.0, higher = smoother)
    
    const char* name;       // Gauge name for debugging
};

// Public Functions
bool PWM_Initialize();
int PWM_AddGauge(int gpio_pin, const char* name, int mode, float min_input, float max_input, 
                 int min_range = 0, int max_range = 255);
void PWM_Update();
void PWM_SetGaugeValue(int channel, float value);
void PWM_SetGaugeValueSmooth(int channel, float value);
void PWM_CalibrateGauge(int channel, float min_input, float max_input, int min_range, int max_range);
void PWM_EnableSmoothing(int channel, float smooth_factor = 0.9);
void PWM_DisableSmoothing(int channel);
void PWM_SetGaugeEnabled(int channel, bool enabled);
void PWM_TestGauge(int channel, int test_value);
void PWM_SweepGauge(int channel);
void PWM_PrintStatus();

// Utility Functions
float PWM_GetGaugeValue(int channel);
int PWM_GetGaugeOutput(int channel);  // Returns frequency (Hz) or voltage (0-255)
bool PWM_IsGaugeEnabled(int channel);
const char* PWM_GetGaugeName(int channel);
bool PWM_IsFrequencyMode(int channel);  // True if frequency mode, false if voltage mode

// Advanced Functions
void PWM_SetAllGauges(float value);      // Set all gauges to same value
void PWM_TestAllGauges();                // Test sweep all gauge outputs
void PWM_DisableAllGauges();             // Disable all gauge outputs
void PWM_EnableAllGauges();              // Enable all gauge outputs

#endif // PWM_GAUGES_H