/*
 * ESP32-S3 Zero Modular CAN ECU Client with Nextion Display & PWM Gauges
 * 
 * Main application file with:
 * - CAN functionality in ECU_CAN.h/.cpp
 * - Nextion functionality in SimpleNextion.h/.cpp  
 * - PWM gauge functionality in PWM_Gauges.h/.cpp
 */

#include "ECU_CAN.h"
#include "SimpleNextion.h"
#include "PWM_Gauges.h"

// Nextion display on Serial2 (you can change pins as needed)
SimpleNextion display(&Serial2);

// Nextion configuration - CHANGE THESE PINS AS NEEDED
#define NEXTION_RX_PIN 4   // ESP32 RX (connect to Nextion TX)
#define NEXTION_TX_PIN 5   // ESP32 TX (connect to Nextion RX)
#define NEXTION_BAUD 9600  // Standard Nextion baud rate

// PWM Gauge configuration - CHANGE THESE PINS AS NEEDED
#define SPEED_GAUGE_PIN GPIO_NUM_6    // PWM output for speed gauge
#define TEMP_GAUGE_PIN GPIO_NUM_9     // PWM output for temperature gauge
#define FUEL_GAUGE_PIN GPIO_NUM_10    // PWM output for fuel/pressure gauge

// Loop performance monitoring
unsigned long loop_counter = 0;
unsigned long last_loop_report = 0;

// PWM Gauge channel IDs
int speed_gauge_channel = -1;
int temp_gauge_channel = -1;
int fuel_gauge_channel = -1;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32-S3 Zero CAN ECU Client with Nextion & PWM Gauges");
    Serial.println("=======================================================");
    
    // Initialize CAN module
    if (CAN_Initialize()) {
        Serial.println("✅ CAN system initialized");
    } else {
        Serial.println("❌ CAN system failed to initialize");
    }
    
    // Initialize PWM Gauge system
    if (PWM_Initialize()) {
        Serial.println("✅ PWM gauge system initialized");
        
        // Add gauges with calibration
        speed_gauge_channel = PWM_AddSpeedGauge(SPEED_GAUGE_PIN, 200.0);  // 0-200 kph
        temp_gauge_channel = PWM_AddTempGauge(TEMP_GAUGE_PIN, 0, 120);    // 0-120°C  
        fuel_gauge_channel = PWM_AddGauge(FUEL_GAUGE_PIN, "Pressure", 0.0, 100.0, 0, 4095); // 0-100%
        
        // Enable smoothing for smoother needle movement
        if (speed_gauge_channel >= 0) {
            PWM_EnableSmoothing(speed_gauge_channel, 0.8); // 0.8 = smooth but responsive
            Serial.printf("✅ Speed gauge on GPIO%d (channel %d)\n", SPEED_GAUGE_PIN, speed_gauge_channel);
        }
        
        if (temp_gauge_channel >= 0) {
            PWM_EnableSmoothing(temp_gauge_channel, 0.9);  // 0.9 = very smooth (temp changes slowly)
            Serial.printf("✅ Temperature gauge on GPIO%d (channel %d)\n", TEMP_GAUGE_PIN, temp_gauge_channel);
        }
        
        if (fuel_gauge_channel >= 0) {
            PWM_EnableSmoothing(fuel_gauge_channel, 0.85); // 0.85 = smooth pressure changes
            Serial.printf("✅ Pressure gauge on GPIO%d (channel %d)\n", FUEL_GAUGE_PIN, fuel_gauge_channel);
        }
        
        // Test all gauges on startup (optional - comment out if not needed)
        Serial.println("🔧 Testing PWM gauges...");
        delay(1000);
        PWM_TestAllGauges();
        
    } else {
        Serial.println("❌ PWM gauge system failed to initialize");
    }
    
    // Initialize Nextion display
    if (display.begin(NEXTION_BAUD, NEXTION_RX_PIN, NEXTION_TX_PIN)) {
        Serial.println("✅ Nextion display initialized");
        
        // Initial display setup
        display.setBrightness(80);  // 80% brightness
        display.setPage(0);         // Go to page 0
        
        // Set initial values
        display.setText("status", "STARTING...");
        display.setTextFloat("speed", 0.0, 1);
        display.setTextInt("gear", 0);
        display.setTextFloat("temp", 0.0, 1);
        
    } else {
        Serial.println("⚠️ Nextion display initialization may have issues");
    }
    
    Serial.println("🚀 System ready!");
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("\n🔧 Hardware Configuration:");
    Serial.printf("   CAN: TX=GPIO%d, RX=GPIO%d\n", 7, 8);
    Serial.printf("   Nextion: TX=GPIO%d, RX=GPIO%d\n", NEXTION_TX_PIN, NEXTION_RX_PIN);
    Serial.printf("   Speed Gauge: GPIO%d\n", SPEED_GAUGE_PIN);
    Serial.printf("   Temp Gauge: GPIO%d\n", TEMP_GAUGE_PIN);
    Serial.printf("   Pressure Gauge: GPIO%d\n", FUEL_GAUGE_PIN);
    Serial.println();
}

void loop() {
    loop_counter++; // Increment loop counter
    
    // Update CAN communication
    CAN_Update();
    
    // Update PWM gauges
    PWM_Update();
    
    // Update Nextion display
    updateDisplay();
    
    // Update PWM gauges with ECU data
    updatePWMGauges();
    
    // Print status every 10 seconds
    static unsigned long last_status = 0;
    if (millis() - last_status >= 10000) {
        CAN_PrintStatus();
        PWM_PrintStatus();
        printSystemStatus();
        last_status = millis();
    }
    
    // No delay - run at maximum speed for best CAN responsiveness!
    // The CAN, display, and PWM functions have their own internal timing
}

void updateDisplay() {
    static unsigned long last_display_update = 0;
    const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // Update every 500ms
    
    if (millis() - last_display_update < DISPLAY_UPDATE_INTERVAL) {
        return; // Not time to update yet
    }
    
    last_display_update = millis();
    
    // Get ECU parameters
    float speed = CAN_GetVehicleSpeed();
    float gear = CAN_GetCurrentGear();
    float temp = CAN_GetFluidTemperature();
    float pressureSol = CAN_GetPressureSolenoid();
    
    // Check if data is fresh (within 5 seconds)
    bool speed_fresh = CAN_IsParameterFresh(PARAM_VEHICLE_SPEED, 5);
    bool gear_fresh = CAN_IsParameterFresh(PARAM_CURRENT_GEAR, 5);
    bool temp_fresh = CAN_IsParameterFresh(PARAM_FLUID_TEMP, 5);
    bool pressure_fresh = CAN_IsParameterFresh(PARAM_PRESSURE_SOL, 5);
    
    // Update display elements
    if (speed_fresh) {
        display.setTextFloat("speed", speed, 1);
        
        // Update speed progress bar (assuming max 200 kph)
        int speedPercent = (int)((speed / 200.0) * 100);
        display.setProgress("speedbar", speedPercent);
        
        // Change color based on speed
        if (speed > 100) {
            display.setForegroundColor("speed", SimpleNextion::COLOR_RED);
        } else if (speed > 50) {
            display.setForegroundColor("speed", SimpleNextion::COLOR_YELLOW);
        } else {
            display.setForegroundColor("speed", SimpleNextion::COLOR_GREEN);
        }
    } else {
        display.setText("speed", "---");
        display.setForegroundColor("speed", SimpleNextion::COLOR_GRAY);
    }
    
    if (gear_fresh) {
        if (gear >= 1 && gear <= 6) {
            display.setTextInt("gear", (int)gear);
            display.setForegroundColor("gear", SimpleNextion::COLOR_WHITE);
        } else {
            display.setText("gear", "N");
            display.setForegroundColor("gear", SimpleNextion::COLOR_YELLOW);
        }
    } else {
        display.setText("gear", "-");
        display.setForegroundColor("gear", SimpleNextion::COLOR_GRAY);
    }
    
    if (temp_fresh) {
        display.setTextFloat("temp", temp, 1);
        
        // Temperature warning colors
        if (temp > 90) {
            display.setForegroundColor("temp", SimpleNextion::COLOR_RED);
        } else if (temp > 70) {
            display.setForegroundColor("temp", SimpleNextion::COLOR_YELLOW);
        } else {
            display.setForegroundColor("temp", SimpleNextion::COLOR_GREEN);
        }
        
        // Temperature gauge (assuming 0-120°C range)
        int tempPercent = (int)((temp / 120.0) * 100);
        display.setProgress("tempbar", tempPercent);
    } else {
        display.setText("temp", "---");
        display.setForegroundColor("temp", SimpleNextion::COLOR_GRAY);
    }
    
    if (pressure_fresh) {
        // Pressure solenoid as percentage
        int pressurePercent = (int)(pressureSol * 100);
        display.setProgress("pressure", pressurePercent);
    }
    
    // Update connection status
    if (CAN_IsInitialized()) {
        CANStats stats = CAN_GetStats();
        if (stats.successful_responses > 0) {
            display.setText("status", "CONNECTED");
            display.setForegroundColor("status", SimpleNextion::COLOR_GREEN);
        } else {
            display.setText("status", "NO DATA");
            display.setForegroundColor("status", SimpleNextion::COLOR_YELLOW);
        }
    } else {
        display.setText("status", "CAN ERROR");
        display.setForegroundColor("status", SimpleNextion::COLOR_RED);
    }
    
    // Force send all commands
    display.flush();
}

void updatePWMGauges() {
    static unsigned long last_gauge_update = 0;
    const unsigned long GAUGE_UPDATE_INTERVAL = 100; // Update every 100ms (10Hz)
    
    if (millis() - last_gauge_update < GAUGE_UPDATE_INTERVAL) {
        return; // Not time to update yet
    }
    
    last_gauge_update = millis();
    
    // Get ECU parameters
    float speed = CAN_GetVehicleSpeed();
    float temp = CAN_GetFluidTemperature();
    float pressure = CAN_GetPressureSolenoid() * 100.0; // Convert to percentage
    
    // Check if data is fresh (within 10 seconds for gauges - they should hold last good value)
    bool speed_fresh = CAN_IsParameterFresh(PARAM_VEHICLE_SPEED, 10);
    bool temp_fresh = CAN_IsParameterFresh(PARAM_FLUID_TEMP, 10);
    bool pressure_fresh = CAN_IsParameterFresh(PARAM_PRESSURE_SOL, 10);
    
    // Update speed gauge
    if (speed_gauge_channel >= 0) {
        if (speed_fresh) {
            PWM_SetGaugeValueSmooth(speed_gauge_channel, speed);
        } else {
            // No fresh data - could either hold last value or go to zero
            // For now, we'll hold the last value (gauges will maintain their position)
        }
    }
    
    // Update temperature gauge  
    if (temp_gauge_channel >= 0) {
        if (temp_fresh) {
            PWM_SetGaugeValueSmooth(temp_gauge_channel, temp);
        }
    }
    
    // Update pressure gauge (fuel gauge repurposed)
    if (fuel_gauge_channel >= 0) {
        if (pressure_fresh) {
            PWM_SetGaugeValueSmooth(fuel_gauge_channel, pressure);
        }
    }
    
    // Debug output (uncomment for detailed gauge monitoring)
    /*
    static unsigned long last_debug = 0;
    if (millis() - last_debug >= 2000) { // Every 2 seconds
        Serial.printf("PWM: Speed=%.1f->%d, Temp=%.1f->%d, Pressure=%.1f->%d\n",
                      speed, PWM_GetGaugePWM(speed_gauge_channel),
                      temp, PWM_GetGaugePWM(temp_gauge_channel), 
                      pressure, PWM_GetGaugePWM(fuel_gauge_channel));
        last_debug = millis();
    }
    */
}

void printSystemStatus() {
    // Print loop performance
    if (millis() - last_loop_report >= 1000) {
        unsigned long loops_per_second = loop_counter;
        Serial.printf("🔄 Main Loop: %lu loops/sec (%.1f kHz) - Uptime: %lu sec\n", 
                      loops_per_second, loops_per_second / 1000.0, millis() / 1000);
        loop_counter = 0;
        last_loop_report = millis();
    }
    
    // Print memory usage
    Serial.printf("💾 Free heap: %d bytes", ESP.getFreeHeap());
    if (ESP.getPsramSize() > 0) {
        Serial.printf(", Free PSRAM: %d bytes", ESP.getFreePsram());
    }
    Serial.println();
    
    // Print Nextion status
    if (display.isInitialized()) {
        Serial.println("📺 Nextion: Connected and updating");
    } else {
        Serial.println("📺 Nextion: Not initialized");
    }
    
    Serial.println(); // Extra line for readability
}

// Example function for testing display elements
void testDisplay() {
    Serial.println("Testing display elements...");
    
    // Test different pages
    display.setPage(0);
    delay(1000);
    
    // Test text updates
    display.setText("speed", "123.4");
    display.setText("gear", "3");
    display.setText("temp", "75.5");
    
    // Test colors
    display.setForegroundColor("speed", SimpleNextion::COLOR_GREEN);
    display.setForegroundColor("gear", SimpleNextion::COLOR_YELLOW);
    display.setForegroundColor("temp", SimpleNextion::COLOR_RED);
    
    // Test progress bars
    for (int i = 0; i <= 100; i += 10) {
        display.setProgress("speedbar", i);
        display.setProgress("tempbar", i);
        delay(200);
    }
    
    Serial.println("Display test complete");
}