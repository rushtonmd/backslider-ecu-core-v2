/*
 * PWM_Gauges.cpp - Dual-Mode Gauge Control Implementation
 * 
 * Supports both:
 * - LEDC frequency mode (for VSS/speedometers): 14-bit, variable frequency
 * - analogWrite voltage mode (for temp/pressure): 8-bit, 0-255 range
 */

#include "PWM_Gauges.h"

// Private Variables
static GaugeChannel gauges[MAX_GAUGE_CHANNELS];
static int num_gauges = 0;
static bool pwm_initialized = false;

// Private Function Declarations
static int mapValue(float input, float in_min, float in_max, int out_min, int out_max);
static float constrain_float(float value, float min_val, float max_val);

// Public Functions Implementation

bool PWM_Initialize() {
    // Initialize all gauge channels
    for (int i = 0; i < MAX_GAUGE_CHANNELS; i++) {
        gauges[i].gpio_pin = -1;
        gauges[i].enabled = false;
        gauges[i].is_frequency_mode = false;
        gauges[i].name = nullptr;
    }
    
    num_gauges = 0;
    pwm_initialized = true;
    
    Serial.println("PWM: ✅ Dual-Mode Gauge System initialized");
    Serial.println("PWM: Supports both LEDC frequency and analogWrite voltage modes");
    
    return true;
}

int PWM_AddGauge(int gpio_pin, const char* name, int mode, float min_input, float max_input, 
                 int min_range, int max_range) {
    if (!pwm_initialized || num_gauges >= MAX_GAUGE_CHANNELS) {
        Serial.printf("PWM: ❌ Cannot add gauge %s - system full or not initialized\n", name);
        return -1;
    }
    
    int channel = num_gauges;
    GaugeChannel* gauge = &gauges[channel];
    
    // Configure gauge channel
    gauge->gpio_pin = gpio_pin;
    gauge->enabled = true;
    gauge->is_frequency_mode = (mode == GAUGE_MODE_FREQUENCY);
    gauge->name = name;
    
    // Calibration
    gauge->min_input = min_input;
    gauge->max_input = max_input;
    gauge->min_freq = min_range;
    gauge->max_freq = max_range;
    gauge->min_pwm = min_range;  // Legacy compatibility
    gauge->max_pwm = max_range;  // Legacy compatibility
    
    // State
    gauge->current_value = min_input;
    gauge->current_freq = min_range;
    gauge->current_pwm = min_range;
    gauge->last_update = millis();
    
    // Smoothing (disabled by default)
    gauge->smooth_enabled = false;
    gauge->smooth_factor = 0.9;
    
    // Configure output based on mode
    if (gauge->is_frequency_mode) {
        // Frequency mode - use LEDC
        if (min_range > 0) {
            ledcAttach(gpio_pin, min_range, PWM_RESOLUTION);
            ledcWrite(gpio_pin, PWM_DUTY_CYCLE);  // 50% duty cycle
        } else {
            pinMode(gpio_pin, OUTPUT);
            digitalWrite(gpio_pin, LOW);
        }
        Serial.printf("PWM: ✅ Added FREQUENCY gauge '%s' on GPIO%d (LEDC)\n", name, gpio_pin);
        Serial.printf("     Input: %.1f-%.1f, Freq: %d-%d Hz\n", 
                      min_input, max_input, min_range, max_range);
    } else {
        // Voltage mode - use analogWrite
        analogWrite(gpio_pin, min_range);
        Serial.printf("PWM: ✅ Added VOLTAGE gauge '%s' on GPIO%d (analogWrite)\n", name, gpio_pin);
        Serial.printf("     Input: %.1f-%.1f, Voltage: %d-%d (0-255)\n", 
                      min_input, max_input, min_range, max_range);
    }
    
    num_gauges++;
    return channel;
}

void PWM_Update() {
    // This function can be called regularly for future features
    // Currently not needed as both LEDC and analogWrite handle everything
}

void PWM_SetGaugeValue(int channel, float value) {
    if (channel < 0 || channel >= num_gauges || !gauges[channel].enabled) {
        return;
    }
    
    GaugeChannel* gauge = &gauges[channel];
    
    // Constrain input value
    value = constrain_float(value, gauge->min_input, gauge->max_input);
    
    // Map to output range
    int output_value = mapValue(value, gauge->min_input, gauge->max_input, 
                               gauge->min_freq, gauge->max_freq);
    
    // Update gauge state
    gauge->current_value = value;
    gauge->current_freq = output_value;
    gauge->current_pwm = output_value;
    gauge->last_update = millis();
    
    // Output based on mode
    if (gauge->is_frequency_mode) {
        // Frequency mode - use LEDC
        if (output_value > 0) {
            ledcDetach(gauge->gpio_pin);
            ledcAttach(gauge->gpio_pin, output_value, PWM_RESOLUTION);
            ledcWrite(gauge->gpio_pin, PWM_DUTY_CYCLE);  // 50% duty cycle
        } else {
            ledcDetach(gauge->gpio_pin);
            pinMode(gauge->gpio_pin, OUTPUT);
            digitalWrite(gauge->gpio_pin, LOW);
        }
        Serial.printf("PWM: %s = %.1f -> %d Hz\n", gauge->name, value, output_value);
    } else {
        // Voltage mode - use analogWrite
        analogWrite(gauge->gpio_pin, output_value);
        Serial.printf("PWM: %s = %.1f -> %d/255 (%.1fV)\n", 
                      gauge->name, value, output_value, (output_value * 3.3f / 255.0f));
    }
}

void PWM_SetGaugeValueSmooth(int channel, float value) {
    if (channel < 0 || channel >= num_gauges || !gauges[channel].enabled) {
        return;
    }
    
    GaugeChannel* gauge = &gauges[channel];
    
    if (gauge->smooth_enabled) {
        // Apply smoothing filter
        float smoothed_value = (gauge->smooth_factor * gauge->current_value) + 
                              ((1.0 - gauge->smooth_factor) * value);
        PWM_SetGaugeValue(channel, smoothed_value);
    } else {
        // No smoothing, direct update
        PWM_SetGaugeValue(channel, value);
    }
}

void PWM_CalibrateGauge(int channel, float min_input, float max_input, int min_range, int max_range) {
    if (channel < 0 || channel >= num_gauges) return;
    
    GaugeChannel* gauge = &gauges[channel];
    gauge->min_input = min_input;
    gauge->max_input = max_input;
    gauge->min_freq = min_range;
    gauge->max_freq = max_range;
    gauge->min_pwm = min_range;      // Legacy compatibility
    gauge->max_pwm = max_range;      // Legacy compatibility
    
    const char* mode_str = gauge->is_frequency_mode ? "Hz" : "V";
    Serial.printf("PWM: Calibrated %s - Input: %.1f-%.1f, Range: %d-%d %s\n", 
                  gauge->name, min_input, max_input, min_range, max_range, mode_str);
}

void PWM_EnableSmoothing(int channel, float smooth_factor) {
    if (channel < 0 || channel >= num_gauges) return;
    
    gauges[channel].smooth_enabled = true;
    gauges[channel].smooth_factor = constrain_float(smooth_factor, 0.0, 1.0);
    
    Serial.printf("PWM: Enabled smoothing for %s (factor=%.2f)\n", 
                  gauges[channel].name, smooth_factor);
}

void PWM_DisableSmoothing(int channel) {
    if (channel < 0 || channel >= num_gauges) return;
    
    gauges[channel].smooth_enabled = false;
    Serial.printf("PWM: Disabled smoothing for %s\n", gauges[channel].name);
}

void PWM_SetGaugeEnabled(int channel, bool enabled) {
    if (channel < 0 || channel >= num_gauges) return;
    
    gauges[channel].enabled = enabled;
    
    if (!enabled) {
        // Set to minimum when disabled
        if (gauges[channel].is_frequency_mode) {
            ledcDetach(gauges[channel].gpio_pin);
            pinMode(gauges[channel].gpio_pin, OUTPUT);
            digitalWrite(gauges[channel].gpio_pin, LOW);
        } else {
            analogWrite(gauges[channel].gpio_pin, 0);
        }
    } else {
        // Re-enable with current value
        PWM_SetGaugeValue(channel, gauges[channel].current_value);
    }
    
    Serial.printf("PWM: %s %s\n", gauges[channel].name, enabled ? "enabled" : "disabled");
}

void PWM_TestGauge(int channel, int test_value) {
    if (channel < 0 || channel >= num_gauges) return;
    
    test_value = constrain(test_value, 0, gauges[channel].is_frequency_mode ? 40000 : 255);
    
    if (gauges[channel].is_frequency_mode) {
        // Frequency mode
        if (test_value > 0) {
            ledcDetach(gauges[channel].gpio_pin);
            ledcAttach(gauges[channel].gpio_pin, test_value, PWM_RESOLUTION);
            ledcWrite(gauges[channel].gpio_pin, PWM_DUTY_CYCLE);
        } else {
            ledcDetach(gauges[channel].gpio_pin);
            pinMode(gauges[channel].gpio_pin, OUTPUT);
            digitalWrite(gauges[channel].gpio_pin, LOW);
        }
        Serial.printf("PWM: Testing %s with %d Hz\n", gauges[channel].name, test_value);
    } else {
        // Voltage mode
        analogWrite(gauges[channel].gpio_pin, test_value);
        Serial.printf("PWM: Testing %s with %d/255 (%.1fV)\n", 
                      gauges[channel].name, test_value, (test_value * 3.3f / 255.0f));
    }
}

void PWM_SweepGauge(int channel) {
    if (channel < 0 || channel >= num_gauges) return;
    
    Serial.printf("PWM: Sweeping %s...\n", gauges[channel].name);
    
    if (gauges[channel].is_frequency_mode) {
        // Frequency sweep
        for (int freq = gauges[channel].min_freq; freq <= gauges[channel].max_freq; freq += 100) {
            if (freq > 0) {
                ledcDetach(gauges[channel].gpio_pin);
                ledcAttach(gauges[channel].gpio_pin, freq, PWM_RESOLUTION);
                ledcWrite(gauges[channel].gpio_pin, PWM_DUTY_CYCLE);
            }
            delay(100);
        }
        
        // Return to minimum
        if (gauges[channel].min_freq > 0) {
            ledcDetach(gauges[channel].gpio_pin);
            ledcAttach(gauges[channel].gpio_pin, gauges[channel].min_freq, PWM_RESOLUTION);
            ledcWrite(gauges[channel].gpio_pin, PWM_DUTY_CYCLE);
        } else {
            ledcDetach(gauges[channel].gpio_pin);
            pinMode(gauges[channel].gpio_pin, OUTPUT);
            digitalWrite(gauges[channel].gpio_pin, LOW);
        }
    } else {
        // Voltage sweep
        for (int volt = gauges[channel].min_freq; volt <= gauges[channel].max_freq; volt += 10) {
            analogWrite(gauges[channel].gpio_pin, volt);
            delay(50);
        }
        
        // Return to minimum
        analogWrite(gauges[channel].gpio_pin, gauges[channel].min_freq);
    }
    
    Serial.printf("PWM: Sweep complete for %s\n", gauges[channel].name);
}

void PWM_PrintStatus() {
    Serial.println("\n📊 === Dual-Mode Gauge Status ===");
    Serial.printf("Active gauge channels: %d/%d\n", num_gauges, MAX_GAUGE_CHANNELS);
    
    for (int i = 0; i < num_gauges; i++) {
        GaugeChannel* gauge = &gauges[i];
        unsigned long age = (millis() - gauge->last_update) / 1000;
        
        const char* mode_str = gauge->is_frequency_mode ? "FREQ" : "VOLT";
        const char* unit_str = gauge->is_frequency_mode ? "Hz" : "/255";
        
        Serial.printf("  %s (GPIO%d, %s): ", gauge->name, gauge->gpio_pin, mode_str);
        Serial.printf("%.1f -> %d%s (%s, %lu sec ago)\n", 
                      gauge->current_value, gauge->current_freq, unit_str,
                      gauge->enabled ? "ON" : "OFF", age);
    }
    Serial.println("==============================\n");
}

// Utility Functions
float PWM_GetGaugeValue(int channel) {
    if (channel < 0 || channel >= num_gauges) return 0.0;
    return gauges[channel].current_value;
}

int PWM_GetGaugeOutput(int channel) {
    if (channel < 0 || channel >= num_gauges) return 0;
    return gauges[channel].current_freq;
}

bool PWM_IsGaugeEnabled(int channel) {
    if (channel < 0 || channel >= num_gauges) return false;
    return gauges[channel].enabled;
}

const char* PWM_GetGaugeName(int channel) {
    if (channel < 0 || channel >= num_gauges) return "Invalid";
    return gauges[channel].name;
}

bool PWM_IsFrequencyMode(int channel) {
    if (channel < 0 || channel >= num_gauges) return false;
    return gauges[channel].is_frequency_mode;
}

// Advanced Functions
void PWM_SetAllGauges(float value) {
    for (int i = 0; i < num_gauges; i++) {
        if (gauges[i].enabled) {
            PWM_SetGaugeValue(i, value);
        }
    }
}

void PWM_TestAllGauges() {
    Serial.println("PWM: Testing all gauge outputs...");
    for (int i = 0; i < num_gauges; i++) {
        if (gauges[i].enabled) {
            PWM_SweepGauge(i);
            delay(500);
        }
    }
    Serial.println("PWM: All gauge tests complete");
}

void PWM_DisableAllGauges() {
    for (int i = 0; i < num_gauges; i++) {
        PWM_SetGaugeEnabled(i, false);
    }
}

void PWM_EnableAllGauges() {
    for (int i = 0; i < num_gauges; i++) {
        PWM_SetGaugeEnabled(i, true);
    }
}

// Private Functions
static int mapValue(float input, float in_min, float in_max, int out_min, int out_max) {
    if (in_max == in_min) return out_min; // Avoid division by zero
    
    float mapped = (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return constrain((int)mapped, out_min, out_max);
}

static float constrain_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}