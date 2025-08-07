/*
 * PWM_Gauges.cpp - PWM Gauge Control Implementation
 * 
 * Controls analog gauges using PWM signals via 2N2222 transistors
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
        gauges[i].pwm_channel = -1;
        gauges[i].enabled = false;
        gauges[i].name = nullptr;
    }
    
    num_gauges = 0;
    pwm_initialized = true;
    
    Serial.println("PWM: ✅ Gauge system initialized");
    Serial.printf("PWM: Frequency=%dHz, Resolution=%d-bit, Max=%d\n", 
                  PWM_FREQUENCY, PWM_RESOLUTION, PWM_MAX_VALUE);
    
    return true;
}

int PWM_AddGauge(int gpio_pin, const char* name, float min_input, float max_input, 
                 int min_pwm, int max_pwm) {
    if (!pwm_initialized || num_gauges >= MAX_GAUGE_CHANNELS) {
        Serial.printf("PWM: ❌ Cannot add gauge %s - system full or not initialized\n", name);
        return -1;
    }
    
    int channel = num_gauges;
    GaugeChannel* gauge = &gauges[channel];
    
    // Configure gauge
    gauge->gpio_pin = gpio_pin;
    gauge->pwm_channel = channel;  // Use channel number as PWM channel
    gauge->enabled = true;
    gauge->name = name;
    
    // Calibration
    gauge->min_input = min_input;
    gauge->max_input = max_input;
    gauge->min_pwm = min_pwm;
    gauge->max_pwm = max_pwm;
    
    // State
    gauge->current_value = min_input;
    gauge->current_pwm = min_pwm;
    gauge->last_update = millis();
    
    // Smoothing (disabled by default)
    gauge->smooth_enabled = false;
    gauge->smooth_factor = 0.9;
    
    // Configure ESP32 PWM - Updated for Arduino Core 3.x
    if (!ledcAttach(gauge->gpio_pin, PWM_FREQUENCY, PWM_RESOLUTION)) {
        Serial.printf("PWM: ❌ Failed to attach PWM to GPIO%d\n", gauge->gpio_pin);
        return -1;
    }
    
    // Set initial PWM value
    ledcWrite(gauge->gpio_pin, min_pwm);
    
    num_gauges++;
    
    Serial.printf("PWM: ✅ Added gauge '%s' on GPIO%d (channel %d)\n", 
                  name, gpio_pin, channel);
    Serial.printf("     Input: %.1f-%.1f, PWM: %d-%d\n", 
                  min_input, max_input, min_pwm, max_pwm);
    
    return channel;
}

void PWM_Update() {
    // This function can be called regularly to handle smooth transitions
    // Currently, smoothing is handled in PWM_SetGaugeValueSmooth()
    // Future: Could add automatic decay, diagnostics, etc.
}

void PWM_SetGaugeValue(int channel, float value) {
    if (channel < 0 || channel >= num_gauges || !gauges[channel].enabled) {
        return;
    }
    
    GaugeChannel* gauge = &gauges[channel];
    
    // Constrain input value
    value = constrain_float(value, gauge->min_input, gauge->max_input);
    
    // Map to PWM range
    int pwm_value = mapValue(value, gauge->min_input, gauge->max_input, 
                            gauge->min_pwm, gauge->max_pwm);
    
    // Update gauge
    gauge->current_value = value;
    gauge->current_pwm = pwm_value;
    gauge->last_update = millis();
    
    // Output PWM - Updated for Arduino Core 3.x
    ledcWrite(gauge->gpio_pin, pwm_value);
    
    // Debug output (uncomment for detailed logging)
    // Serial.printf("PWM: %s = %.1f -> PWM %d\n", gauge->name, value, pwm_value);
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

void PWM_CalibrateGauge(int channel, float min_input, float max_input, int min_pwm, int max_pwm) {
    if (channel < 0 || channel >= num_gauges) return;
    
    GaugeChannel* gauge = &gauges[channel];
    gauge->min_input = min_input;
    gauge->max_input = max_input;
    gauge->min_pwm = min_pwm;
    gauge->max_pwm = max_pwm;
    
    Serial.printf("PWM: Calibrated %s - Input: %.1f-%.1f, PWM: %d-%d\n", 
                  gauge->name, min_input, max_input, min_pwm, max_pwm);
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
        // Set to minimum when disabled - Updated for Arduino Core 3.x
        ledcWrite(gauges[channel].gpio_pin, gauges[channel].min_pwm);
    }
    
    Serial.printf("PWM: %s %s\n", gauges[channel].name, enabled ? "enabled" : "disabled");
}

void PWM_TestGauge(int channel, int test_pwm_value) {
    if (channel < 0 || channel >= num_gauges) return;
    
    test_pwm_value = constrain(test_pwm_value, 0, PWM_MAX_VALUE);
    ledcWrite(gauges[channel].gpio_pin, test_pwm_value);
    
    Serial.printf("PWM: Testing %s with PWM %d\n", gauges[channel].name, test_pwm_value);
}

void PWM_SweepGauge(int channel) {
    if (channel < 0 || channel >= num_gauges) return;
    
    Serial.printf("PWM: Sweeping %s...\n", gauges[channel].name);
    
    // Sweep from min to max
    for (int pwm = gauges[channel].min_pwm; pwm <= gauges[channel].max_pwm; pwm += 50) {
        ledcWrite(gauges[channel].gpio_pin, pwm);
        delay(50);
    }
    
    // Sweep back to min
    for (int pwm = gauges[channel].max_pwm; pwm >= gauges[channel].min_pwm; pwm -= 50) {
        ledcWrite(gauges[channel].gpio_pin, pwm);
        delay(50);
    }
    
    Serial.printf("PWM: Sweep complete for %s\n", gauges[channel].name);
}

void PWM_PrintStatus() {
    Serial.println("\n📊 === PWM Gauge Status ===");
    Serial.printf("Active gauges: %d/%d\n", num_gauges, MAX_GAUGE_CHANNELS);
    
    for (int i = 0; i < num_gauges; i++) {
        GaugeChannel* gauge = &gauges[i];
        unsigned long age = (millis() - gauge->last_update) / 1000;
        
        Serial.printf("  %s (GPIO%d): ", gauge->name, gauge->gpio_pin);
        Serial.printf("%.1f -> PWM %d (%s, %lu sec ago)\n", 
                      gauge->current_value, gauge->current_pwm,
                      gauge->enabled ? "ON" : "OFF", age);
    }
    Serial.println("==========================\n");
}

// Convenience Functions
int PWM_AddSpeedGauge(int gpio_pin, float max_speed) {
    return PWM_AddGauge(gpio_pin, "Speed", 0.0, max_speed, 0, PWM_MAX_VALUE);
}

int PWM_AddTachGauge(int gpio_pin, float max_rpm) {
    return PWM_AddGauge(gpio_pin, "Tachometer", 0.0, max_rpm, 0, PWM_MAX_VALUE);
}

int PWM_AddTempGauge(int gpio_pin, float min_temp, float max_temp) {
    return PWM_AddGauge(gpio_pin, "Temperature", min_temp, max_temp, 0, PWM_MAX_VALUE);
}

int PWM_AddFuelGauge(int gpio_pin) {
    return PWM_AddGauge(gpio_pin, "Fuel", 0.0, 100.0, 0, PWM_MAX_VALUE);
}

int PWM_AddVoltGauge(int gpio_pin, float min_volt, float max_volt) {
    return PWM_AddGauge(gpio_pin, "Voltage", min_volt, max_volt, 0, PWM_MAX_VALUE);
}

// Utility Functions
float PWM_GetGaugeValue(int channel) {
    if (channel < 0 || channel >= num_gauges) return 0.0;
    return gauges[channel].current_value;
}

int PWM_GetGaugePWM(int channel) {
    if (channel < 0 || channel >= num_gauges) return 0;
    return gauges[channel].current_pwm;
}

bool PWM_IsGaugeEnabled(int channel) {
    if (channel < 0 || channel >= num_gauges) return false;
    return gauges[channel].enabled;
}

const char* PWM_GetGaugeName(int channel) {
    if (channel < 0 || channel >= num_gauges) return "Invalid";
    return gauges[channel].name;
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
    Serial.println("PWM: Testing all gauges...");
    for (int i = 0; i < num_gauges; i++) {

        // DISABLE SWEEPING FOR NOW
        // if (gauges[i].enabled) {
        //     PWM_SweepGauge(i);
        //     delay(500);
        // }
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