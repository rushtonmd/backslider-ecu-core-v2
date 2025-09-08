/*
 * ESP32-S3 Zero Modular CAN ECU Client with Nextion Display & PWM Gauges
 * 
 * Main application file with:
 * - CAN functionality in ECU_CAN.h/.cpp (multi-rate requests)
 * - Nextion functionality in SimpleNextion.h/.cpp  
 * - PWM gauge functionality in PWM_Gauges.h/.cpp
 * - Smart change detection to minimize Nextion traffic
 */

#include "ECU_CAN.h"
#include "SimpleNextion.h"
#include "PWM_Gauges.h"

// Nextion display on Serial2
SimpleNextion display(&Serial2);

// Nextion configuration - CHANGE THESE PINS AS NEEDED
#define NEXTION_RX_PIN 10   // ESP32 RX (connect to Nextion TX)
#define NEXTION_TX_PIN 12   // ESP32 TX (connect to Nextion RX)
#define NEXTION_BAUD 115200  // Standard Nextion baud rate

// Update Rate Constants (in milliseconds)
#define UPDATE_10HZ_INTERVAL    100   // 10Hz updates (100ms)
#define UPDATE_5HZ_INTERVAL     200   // 5Hz updates (200ms)
#define UPDATE_2HZ_INTERVAL     500   // 2Hz updates (500ms)
#define UPDATE_1HZ_INTERVAL     1000  // 1Hz updates (1000ms)
#define UPDATE_05HZ_INTERVAL    2000  // 0.5Hz updates (2000ms)

// Backup update intervals (forced updates even if no change)
#define GEAR_BACKUP_INTERVAL    1000  // Force gear update every 1 second
#define SPEED_BACKUP_INTERVAL   1000  // Force speed update every 1 second
#define TEMP_BACKUP_INTERVAL    5000  // Force temp update every 5 seconds
#define PRESSURE_BACKUP_INTERVAL 1000 // Force pressure update every 1 second

// Change detection thresholds
#define SPEED_CHANGE_THRESHOLD  1.0   // Only update if speed changes by >1 kph
#define TEMP_CHANGE_THRESHOLD   0.5   // Only update if temp changes by >0.5°C
#define PRESSURE_CHANGE_THRESHOLD 5   // Only update if pressure changes by >5%

// Data Freshness Timeouts (in seconds)
#define SPEED_DATA_TIMEOUT      1     // Speed data timeout
#define TEMP_DATA_TIMEOUT       5     // Temperature data timeout
#define GEAR_DATA_TIMEOUT       2     // Gear/solenoid data timeout
#define PRESSURE_DATA_TIMEOUT   2     // Pressure data timeout

// System Status Update Intervals
#define STATUS_REPORT_INTERVAL  10000 // Status report every 10 seconds

// Nextion Widget Names - CHANGE THESE TO MATCH YOUR DISPLAY
#define NEXTION_SPEED_TEXT      "txtSpeed"      // Speed display
#define NEXTION_GEAR_TEXT       "txtGear"       // Current gear display  
#define NEXTION_DRIVE_TEXT      "txtDriveLabel" // Drive mode (P/R/N/D)
#define NEXTION_TEMP_TEXT       "txtTemp"       // Transmission temperature
#define NEXTION_SHIFT1_TEXT     "txtShift1"     // Shift solenoid A
#define NEXTION_SHIFT2_TEXT     "txtShift2"     // Shift solenoid B  
#define NEXTION_LINE_TEXT       "txtLine"       // Line pressure solenoid
#define NEXTION_OVERRUN_TEXT    "txtOverrun"    // Overrun solenoid
#define NEXTION_LOCKUP_TEXT     "txtLockup"     // Lockup solenoid

// Optional: Progress bar widget names (if you have them)
#define NEXTION_SPEED_BAR       "speedbar"      // Speed progress bar
#define NEXTION_TEMP_BAR        "tempbar"       // Temperature progress bar
#define NEXTION_PRESSURE_BAR    "pressure"      // Pressure progress bar

// Optional: Status widget name
#define NEXTION_STATUS_TEXT     "status"        // Connection status

// PWM Gauge configuration - CHANGE THESE PINS AS NEEDED
#define SPEED_GAUGE_PIN 7    // PWM output for speed gauge
#define TEMP_GAUGE_PIN 6     // PWM output for coolant temperature gauge  
#define OIL_GAUGE_PIN 4     // PWM output for oil pressure gauge

// Loop performance monitoring
unsigned long loop_counter = 0;
unsigned long last_loop_report = 0;
unsigned long nextion_commands_sent = 0;
unsigned long nextion_commands_skipped = 0;

// PWM Gauge channel IDs
int speed_gauge_channel = -1;
int coolant_temp_gauge_channel = -1;
int oil_pressure_gauge_channel = -1;

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
        // speed_gauge_channel = PWM_AddSpeedGauge(SPEED_GAUGE_PIN, 200.0);  // 0-200 kph
        // coolant_temp_gauge_channel = PWM_AddTempGauge(TEMP_GAUGE_PIN, 60, 120);    // 60-120°C  
        // oil_pressure_gauge_channel = PWM_AddGauge(OIL_GAUGE_PIN, "Oil Pressure", 0.0, 800.0, 0, 4095); // 0-800 kPa

        // Frequency-based speedometer (LEDC mode)
        speed_gauge_channel = PWM_AddGauge(SPEED_GAUGE_PIN, "Speed", GAUGE_MODE_FREQUENCY, 0.0, 260.0, 20, 750);

        // Voltage-based analog gauges (analogWrite mode)
        coolant_temp_gauge_channel = PWM_AddGauge(TEMP_GAUGE_PIN, "Coolant Temp", GAUGE_MODE_VOLTAGE, 0.0, 130.0, 50, 255);
        oil_pressure_gauge_channel = PWM_AddGauge(OIL_GAUGE_PIN, "Oil Pressure", GAUGE_MODE_VOLTAGE, 0.0, 500.0, 120, 255);

        
        // Enable smoothing for smoother needle movement
        if (speed_gauge_channel >= 0) {
            PWM_EnableSmoothing(speed_gauge_channel, 0.8); // 0.8 = smooth but responsive
            Serial.printf("✅ Speed gauge on GPIO%d (channel %d)\n", SPEED_GAUGE_PIN, speed_gauge_channel);
        }
        
        if (coolant_temp_gauge_channel >= 0) {
            PWM_EnableSmoothing(coolant_temp_gauge_channel, 0.9);  // 0.9 = very smooth (temp changes slowly)
            Serial.printf("✅ Coolant temp gauge on GPIO%d (channel %d)\n", TEMP_GAUGE_PIN, coolant_temp_gauge_channel);
        }
        
        if (oil_pressure_gauge_channel >= 0) {
            PWM_EnableSmoothing(oil_pressure_gauge_channel, 0.85); // 0.85 = smooth pressure changes
            Serial.printf("✅ Oil pressure gauge on GPIO%d (channel %d)\n", OIL_GAUGE_PIN, oil_pressure_gauge_channel);
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
        display.setText(NEXTION_STATUS_TEXT, "STARTING...");
        display.setTextFloat(NEXTION_SPEED_TEXT, 0.0, 1);
        display.setTextInt(NEXTION_GEAR_TEXT, 0);
        display.setText(NEXTION_DRIVE_TEXT, "P");
        display.setTextFloat(NEXTION_TEMP_TEXT, 0.0, 1);
        display.setText(NEXTION_SHIFT1_TEXT, "OFF");
        display.setText(NEXTION_SHIFT2_TEXT, "OFF");
        display.setText(NEXTION_LINE_TEXT, "0%");
        display.setText(NEXTION_OVERRUN_TEXT, "OFF");
        display.setText(NEXTION_LOCKUP_TEXT, "OFF");
        
    } else {
        Serial.println("⚠️ Nextion display initialization may have issues");
    }
    
    Serial.println("🚀 System ready!");
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("\n🔧 Hardware Configuration:");
    Serial.printf("   CAN: TX=GPIO%d, RX=GPIO%d\n", 7, 8);
    Serial.printf("   Nextion: TX=GPIO%d, RX=GPIO%d\n", NEXTION_TX_PIN, NEXTION_RX_PIN);
    Serial.printf("   Speed Gauge: GPIO%d\n", SPEED_GAUGE_PIN);
    Serial.printf("   Coolant Temp Gauge: GPIO%d\n", TEMP_GAUGE_PIN);
    Serial.printf("   Oil Pressure Gauge: GPIO%d\n", OIL_GAUGE_PIN);
    Serial.println();
    
    // Print update strategy configuration
    Serial.println("⏱️ Update Strategy Configuration:");
    Serial.printf("   Speed: 10Hz with %.1f kph change threshold\n", SPEED_CHANGE_THRESHOLD);
    Serial.printf("   Temperature: 1Hz with %.1f°C change threshold\n", TEMP_CHANGE_THRESHOLD);
    Serial.printf("   Gear/Solenoids: Immediate on change + %dms backup\n", GEAR_BACKUP_INTERVAL);
    Serial.printf("   Line Pressure: 1Hz with %d%% change threshold\n", PRESSURE_CHANGE_THRESHOLD);
    Serial.println();
}

void loop() {
    loop_counter++;
    
    // Update CAN communication (handles multi-rate requests internally)
    CAN_Update();
    
    // Update PWM gauges (internal timing)
    //PWM_Update();
    
    // Smart multi-rate display updates with change detection
    updateSpeedDisplay(UPDATE_10HZ_INTERVAL);           // 10Hz with change detection
    updateTemperatureDisplay(UPDATE_1HZ_INTERVAL);      // 1Hz with change detection  
    updateGearDisplay(GEAR_BACKUP_INTERVAL);            // Immediate on change + 1s backup
    updateLinePressureDisplay(UPDATE_1HZ_INTERVAL);     // 1Hz with change detection
    updatePWMGauges();                                 // Multi-rate PWM updates

    //PWM_SetGaugeValue(coolant_temp_gauge_channel, 80);
    //PWM_SetGaugeValue(oil_pressure_gauge_channel, 200);


    // IMPORTANT: Process Nextion command queue (do this AFTER display updates)
    display.update();
    
    // Print status every 10 seconds
    static unsigned long last_status = 0;
    if (millis() - last_status >= STATUS_REPORT_INTERVAL) {
        CAN_PrintStatus();
        CAN_PrintDetailedDebug();
        PWM_PrintStatus();
        printSystemStatus();
        last_status = millis();
    }
    
    // No delay - run at maximum speed for best CAN responsiveness!
}


// Speed display - 10Hz with change detection, integer conversion, and rate limiting
void updateSpeedDisplay(unsigned long update_interval_ms) {
    static unsigned long last_update = 0;
    static unsigned long last_backup_update = 0;
    static int last_speed_int = -999;
    static int last_speed_color = -1;
    
    // Rate limiting constants
    const int MAX_SPEED_CHANGE_PER_CYCLE = 10; // Max 10 kph change per 100ms update
    
    // Check timing
    unsigned long current_time = millis();
    if (current_time - last_update < update_interval_ms) return;
    last_update = current_time;
    
    float speed_raw = CAN_GetVehicleSpeed();
    bool speed_fresh = CAN_IsParameterFresh(PARAM_VEHICLE_SPEED, SPEED_DATA_TIMEOUT);
    
    // Convert to integer with bounds checking and clamping
    int speed_int = 0;
    if (speed_raw >= 0 && speed_raw <= 300) {  // Reasonable speed range
        speed_int = (int)round(speed_raw);
    } else if (speed_raw > 300) {
        speed_int = 0; // Treat obviously invalid readings as zero
        Serial.printf("🐛 Invalid high speed from CAN: %.2f, clamped to 0\n", speed_raw);
    }
    // Negative speeds automatically become 0
    
    // Apply rate limiting (simple hysteresis)
    if (last_speed_int != -999) {
        int speed_difference = speed_int - last_speed_int;
        
        // Clamp the change to maximum allowed per cycle
        if (abs(speed_difference) > MAX_SPEED_CHANGE_PER_CYCLE) {
            if (speed_difference > 0) {
                speed_int = last_speed_int + MAX_SPEED_CHANGE_PER_CYCLE;  // Limit acceleration
            } else {
                speed_int = last_speed_int - MAX_SPEED_CHANGE_PER_CYCLE;  // Limit deceleration  
            }
            Serial.printf("🐌 Speed change rate limited: %d -> %d kph\n", last_speed_int, speed_int);
        }
    }
    
    // Determine current color
    int current_color;
    if (speed_int > 100) {
        current_color = SimpleNextion::COLOR_RED;
    } else if (speed_int > 50) {
        current_color = SimpleNextion::COLOR_YELLOW;
    } else {
        current_color = SimpleNextion::COLOR_GREEN;
    }
    
    // Check if we need to update (integer comparison)
    bool speed_changed = abs(speed_int - last_speed_int) >= (int)SPEED_CHANGE_THRESHOLD;
    bool color_changed = current_color != last_speed_color;
    bool backup_needed = (current_time - last_backup_update) >= SPEED_BACKUP_INTERVAL;
    bool should_update = speed_changed || color_changed || backup_needed;
    
    if (speed_fresh && should_update) {
        // Send as integer - cleaner and faster
        display.setTextInt(NEXTION_SPEED_TEXT, speed_int);
        
        // Update speed progress bar (integer math)
        int speedPercent = (speed_int * 100) / 200;
        if (speedPercent > 100) speedPercent = 100;
        display.setProgress(NEXTION_SPEED_BAR, speedPercent);
        
        // Update color if changed
        if (color_changed) {
            display.setForegroundColor(NEXTION_SPEED_TEXT, current_color);
        }
        
        // Update stored values
        last_speed_int = speed_int;
        last_speed_color = current_color;
        last_backup_update = current_time;
        nextion_commands_sent += 3; // Track commands sent
        
    } else if (speed_fresh) {
        nextion_commands_skipped += 3; // Track commands skipped
    }
    
    // Handle stale data case
    if (!speed_fresh && backup_needed) {
        display.setText(NEXTION_SPEED_TEXT, "---");
        display.setForegroundColor(NEXTION_SPEED_TEXT, SimpleNextion::COLOR_GRAY);
        last_speed_int = -999; // Reset so next valid reading will update
        last_speed_color = SimpleNextion::COLOR_GRAY;
        last_backup_update = current_time;
        nextion_commands_sent += 2;
    }
    
    // Update connection status (only when needed)
    updateConnectionStatus();
}

// Temperature display - 1Hz with change detection
void updateTemperatureDisplay(unsigned long update_interval_ms) {
    static unsigned long last_update = 0;
    static unsigned long last_backup_update = 0;
    static float last_temp = -999.0;
    static int last_temp_color = -1;
    
    unsigned long current_time = millis();
    if (current_time - last_update < update_interval_ms) return;
    last_update = current_time;
    
    float temp = CAN_GetFluidTemperature();
    bool temp_fresh = CAN_IsParameterFresh(PARAM_FLUID_TEMP, TEMP_DATA_TIMEOUT);
    
    // Determine current color
    int current_color;
    if (temp > 90) {
        current_color = SimpleNextion::COLOR_RED;
    } else if (temp > 70) {
        current_color = SimpleNextion::COLOR_YELLOW;
    } else {
        current_color = SimpleNextion::COLOR_GREEN;
    }
    
    // Check if we need to update
    bool temp_changed = abs(temp - last_temp) >= TEMP_CHANGE_THRESHOLD;
    bool color_changed = current_color != last_temp_color;
    bool backup_needed = (current_time - last_backup_update) >= TEMP_BACKUP_INTERVAL;
    bool should_update = temp_changed || color_changed || backup_needed;
    
    if (temp_fresh && should_update) {
        display.setTextFloat(NEXTION_TEMP_TEXT, temp, 1);
        
        // Update color if changed
        if (color_changed) {
            display.setForegroundColor(NEXTION_TEMP_TEXT, current_color);
        }
        
        int tempPercent = (int)((temp / 120.0) * 100);
        display.setProgress(NEXTION_TEMP_BAR, tempPercent);
        
        // Update stored values
        last_temp = temp;
        last_temp_color = current_color;
        last_backup_update = current_time;
        nextion_commands_sent += 3;
        
    } else if (temp_fresh) {
        nextion_commands_skipped += 3;
    }
    
    // Handle stale data
    if (!temp_fresh && backup_needed) {
        display.setText(NEXTION_TEMP_TEXT, "---");
        display.setForegroundColor(NEXTION_TEMP_TEXT, SimpleNextion::COLOR_GRAY);
        last_temp = -999.0;
        last_temp_color = SimpleNextion::COLOR_GRAY;
        last_backup_update = current_time;
        nextion_commands_sent += 2;
    }
}

// Gear/Solenoid display - immediate on change + backup interval
void updateGearDisplay(unsigned long backup_interval_ms) {
    static unsigned long last_backup_update = 0;
    static int last_gear_pos = -1;
    static int last_drive_gear = -1;
    static bool last_overrun_state = false;
    static bool last_lockup_state = false;
    static bool last_gear_fresh = false;
    
    unsigned long current_time = millis();
    
    float gear = CAN_GetCurrentGear();
    float driveGear = CAN_GetDriveGear();
    bool gear_fresh = CAN_IsParameterFresh(PARAM_CURRENT_GEAR, GEAR_DATA_TIMEOUT);
    bool drive_fresh = CAN_IsParameterFresh(PARAM_DRIVE_GEAR, GEAR_DATA_TIMEOUT);
    bool overrun_fresh = CAN_IsParameterFresh(PARAM_OVERRUN_SOL, GEAR_DATA_TIMEOUT);
    bool lockup_fresh = CAN_IsParameterFresh(PARAM_LOCKUP_SOL, GEAR_DATA_TIMEOUT);
    
    // Get current solenoid states
    bool current_overrun_state = overrun_fresh && (CAN_GetOverrunSolenoid() > 0.5);
    bool current_lockup_state = lockup_fresh && (CAN_GetLockupSolenoid() > 0.5);
    
    // Check for changes that require immediate updates
    int current_gear_pos = (int)gear;
    int current_drive_gear = (int)driveGear;
    
    bool gear_changed = (current_gear_pos != last_gear_pos);
    bool drive_gear_changed = (current_drive_gear != last_drive_gear);
    bool overrun_changed = (current_overrun_state != last_overrun_state);
    bool lockup_changed = (current_lockup_state != last_lockup_state);
    bool freshness_changed = (gear_fresh != last_gear_fresh);
    bool backup_needed = (current_time - last_backup_update) >= backup_interval_ms;
    
    // Update immediately on change OR after backup interval
    bool should_update = gear_changed || drive_gear_changed || overrun_changed || 
                        lockup_changed || freshness_changed || backup_needed;
    
    if (should_update) {
        // Update gear display
        if (gear_fresh) {
            switch (current_gear_pos) {
                case 1: // GEAR_PARK
                    display.setText(NEXTION_GEAR_TEXT, "P");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_RED);
                    break;
                    
                case 2: // GEAR_REVERSE  
                    display.setText(NEXTION_GEAR_TEXT, "R");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_RED);
                    break;
                    
                case 3: // GEAR_NEUTRAL
                    display.setText(NEXTION_GEAR_TEXT, "N");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_YELLOW);
                    break;
                    
                case 4: // GEAR_DRIVE
                    {
                        if (drive_fresh) {
                            if (current_drive_gear >= 1 && current_drive_gear <= 4) {
                                display.setTextInt(NEXTION_GEAR_TEXT, current_drive_gear);
                            } else {
                                display.setText(NEXTION_GEAR_TEXT, "D");
                            }
                            display.setVisible(NEXTION_DRIVE_TEXT, true);
                            display.setText(NEXTION_DRIVE_TEXT, "D");
                            display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_GREEN);
                            display.setForegroundColor(NEXTION_DRIVE_TEXT, SimpleNextion::COLOR_GREEN);
                        } else {
                            display.setText(NEXTION_GEAR_TEXT, "D");
                            display.setVisible(NEXTION_DRIVE_TEXT, false);
                            display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_GRAY);
                        }
                    }
                    break;
                    
                case 5: // GEAR_SECOND (Sport/Manual 2nd)
                    display.setText(NEXTION_GEAR_TEXT, "S");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_CYAN);
                    break;
                    
                case 6: // GEAR_FIRST (Sport/Manual 1st) 
                    display.setText(NEXTION_GEAR_TEXT, "F");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_CYAN);
                    break;
                    
                default:
                    display.setText(NEXTION_GEAR_TEXT, "?");
                    display.setVisible(NEXTION_DRIVE_TEXT, false);
                    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_GRAY);
                    break;
            }
        } else {
            display.setText(NEXTION_GEAR_TEXT, "-");
            display.setVisible(NEXTION_DRIVE_TEXT, false);
            display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_GRAY);
        }
        
        // Update shift solenoids (tied to gear changes)
        updateShiftSolenoidDisplays();
        
        // Update overrun and lockup solenoids
        updateOverrunLockupDisplays();
        
        // Store current values for next comparison
        last_gear_pos = current_gear_pos;
        last_drive_gear = current_drive_gear;
        last_overrun_state = current_overrun_state;
        last_lockup_state = current_lockup_state;
        last_gear_fresh = gear_fresh;
        last_backup_update = current_time;
        
        if (gear_changed || drive_gear_changed || overrun_changed || lockup_changed) {
            Serial.printf("🔄 Gear update triggered: G:%d->%d, DG:%d->%d, OR:%d->%d, LU:%d->%d\n",
                         last_gear_pos, current_gear_pos, last_drive_gear, current_drive_gear,
                         last_overrun_state, current_overrun_state, last_lockup_state, current_lockup_state);
        }
    }
}

// Line pressure display - 1Hz with change detection
void updateLinePressureDisplay(unsigned long update_interval_ms) {
    static unsigned long last_update = 0;
    static unsigned long last_backup_update = 0;
    static int last_pressure_percent = -1;
    static int last_pressure_color = -1;
    
    unsigned long current_time = millis();
    if (current_time - last_update < update_interval_ms) return;
    last_update = current_time;
    
    float pressureSol = CAN_GetPressureSolenoid();
    bool pressure_fresh = CAN_IsParameterFresh(PARAM_PRESSURE_SOL, PRESSURE_DATA_TIMEOUT);
    
    if (pressure_fresh) {
        int current_pressure_percent = (int)(pressureSol * 100);
        
        // Determine color
        int current_color;
        if (current_pressure_percent > 80) {
            current_color = SimpleNextion::COLOR_GREEN;
        } else if (current_pressure_percent > 0) {
            current_color = SimpleNextion::COLOR_GREEN;
        } else {
            current_color = SimpleNextion::COLOR_GRAY;
        }
        
        // Check if update needed
        bool pressure_changed = abs(current_pressure_percent - last_pressure_percent) >= PRESSURE_CHANGE_THRESHOLD;
        bool color_changed = current_color != last_pressure_color;
        bool backup_needed = (current_time - last_backup_update) >= PRESSURE_BACKUP_INTERVAL;
        
        if (pressure_changed || color_changed || backup_needed) {
            char pressure_text[16];
            snprintf(pressure_text, sizeof(pressure_text), "LINE: %d%%", current_pressure_percent);
            display.setText(NEXTION_LINE_TEXT, pressure_text);
            display.setProgress(NEXTION_PRESSURE_BAR, current_pressure_percent);
            
            if (color_changed) {
                display.setForegroundColor(NEXTION_LINE_TEXT, current_color);
            }
            
            last_pressure_percent = current_pressure_percent;
            last_pressure_color = current_color;
            last_backup_update = current_time;
            nextion_commands_sent += 3;
            
            if (pressure_changed) {
                Serial.printf("🔧 Pressure changed: %d%% -> %d%%\n", last_pressure_percent, current_pressure_percent);
            }
        } else {
            nextion_commands_skipped += 3;
        }
    } else {
        // Handle stale data
        if ((current_time - last_backup_update) >= PRESSURE_BACKUP_INTERVAL) {
            display.setText(NEXTION_LINE_TEXT, "---");
            display.setForegroundColor(NEXTION_LINE_TEXT, SimpleNextion::COLOR_GRAY);
            last_pressure_percent = -1;
            last_pressure_color = SimpleNextion::COLOR_GRAY;
            last_backup_update = current_time;
            nextion_commands_sent += 2;
        }
    }
}

// Shift solenoid displays (called from gear update)
void updateShiftSolenoidDisplays() {
    static bool last_shift_a_state = false;
    static bool last_shift_b_state = false;
    static bool last_shift_fresh = false;
    
    float shiftSolA = CAN_GetShiftSolenoidA();
    float shiftSolB = CAN_GetShiftSolenoidB();
    bool shift_a_fresh = CAN_IsParameterFresh(PARAM_SHIFT_SOL_A, GEAR_DATA_TIMEOUT);
    bool shift_b_fresh = CAN_IsParameterFresh(PARAM_SHIFT_SOL_B, GEAR_DATA_TIMEOUT);
    bool shift_fresh = shift_a_fresh && shift_b_fresh;
    
    bool current_shift_a_state = shift_a_fresh && (shiftSolA > 0.5);
    bool current_shift_b_state = shift_b_fresh && (shiftSolB > 0.5);
    
    // Update Shift Solenoid A only if changed
    if (current_shift_a_state != last_shift_a_state || shift_fresh != last_shift_fresh) {
        if (shift_a_fresh) {
            display.setText(NEXTION_SHIFT1_TEXT, current_shift_a_state ? "SHIFT 1" : "shift 1");
            display.setForegroundColor(NEXTION_SHIFT1_TEXT, 
                                     current_shift_a_state ? SimpleNextion::COLOR_GREEN : SimpleNextion::COLOR_GRAY);
        } else {
            display.setText(NEXTION_SHIFT1_TEXT, "---");
            display.setForegroundColor(NEXTION_SHIFT1_TEXT, SimpleNextion::COLOR_GRAY);
        }
        last_shift_a_state = current_shift_a_state;
        nextion_commands_sent += 2;
    }
    
    // Update Shift Solenoid B only if changed
    if (current_shift_b_state != last_shift_b_state || shift_fresh != last_shift_fresh) {
        if (shift_b_fresh) {
            display.setText(NEXTION_SHIFT2_TEXT, current_shift_b_state ? "SHIFT 2" : "shift 2");
            display.setForegroundColor(NEXTION_SHIFT2_TEXT, 
                                     current_shift_b_state ? SimpleNextion::COLOR_GREEN : SimpleNextion::COLOR_GRAY);
        } else {
            display.setText(NEXTION_SHIFT2_TEXT, "---");
            display.setForegroundColor(NEXTION_SHIFT2_TEXT, SimpleNextion::COLOR_GRAY);
        }
        last_shift_b_state = current_shift_b_state;
        nextion_commands_sent += 2;
    }
    
    last_shift_fresh = shift_fresh;
}

// Overrun and Lockup solenoid displays (called from gear update)
void updateOverrunLockupDisplays() {
    static bool last_overrun_state = false;
    static bool last_lockup_state = false;
    static bool last_overrun_fresh = false;
    static bool last_lockup_fresh = false;
    
    float overrunSol = CAN_GetOverrunSolenoid();
    float lockupSol = CAN_GetLockupSolenoid();
    bool overrun_fresh = CAN_IsParameterFresh(PARAM_OVERRUN_SOL, GEAR_DATA_TIMEOUT);
    bool lockup_fresh = CAN_IsParameterFresh(PARAM_LOCKUP_SOL, GEAR_DATA_TIMEOUT);
    
    bool current_overrun_state = overrun_fresh && (overrunSol > 0.5);
    bool current_lockup_state = lockup_fresh && (lockupSol > 0.5);
    
    // Update Overrun Solenoid only if changed
    if (current_overrun_state != last_overrun_state || overrun_fresh != last_overrun_fresh) {
        if (overrun_fresh) {
            display.setText(NEXTION_OVERRUN_TEXT, current_overrun_state ? "OVERRUN" : "overrun");
            display.setForegroundColor(NEXTION_OVERRUN_TEXT, 
                                     current_overrun_state ? SimpleNextion::COLOR_GREEN : SimpleNextion::COLOR_GRAY);
        } else {
            display.setText(NEXTION_OVERRUN_TEXT, "---");
            display.setForegroundColor(NEXTION_OVERRUN_TEXT, SimpleNextion::COLOR_GRAY);
        }
        last_overrun_state = current_overrun_state;
        last_overrun_fresh = overrun_fresh;
        nextion_commands_sent += 2;
    }
    
    // Update Lockup Solenoid only if changed
    if (current_lockup_state != last_lockup_state || lockup_fresh != last_lockup_fresh) {
        if (lockup_fresh) {
            display.setText(NEXTION_LOCKUP_TEXT, current_lockup_state ? "LOCKUP" : "lockup");
            display.setForegroundColor(NEXTION_LOCKUP_TEXT, 
                                     current_lockup_state ? SimpleNextion::COLOR_GREEN : SimpleNextion::COLOR_GRAY);
        } else {
            display.setText(NEXTION_LOCKUP_TEXT, "---");
            display.setForegroundColor(NEXTION_LOCKUP_TEXT, SimpleNextion::COLOR_GRAY);
        }
        last_lockup_state = current_lockup_state;
        last_lockup_fresh = lockup_fresh;
        nextion_commands_sent += 2;
    }
}

void updateConnectionStatus() {
    static unsigned long last_status_update = 0;
    static int last_connection_state = -1; // -1=unknown, 0=error, 1=no_data, 2=connected
    
    unsigned long current_time = millis();
    
    // Only check connection status every 500ms to reduce overhead
    if (current_time - last_status_update < 500) return;
    last_status_update = current_time;
    
    int current_connection_state;
    if (CAN_IsInitialized()) {
        CANStats stats = CAN_GetStats();
        if (stats.successful_responses > 0) {
            current_connection_state = 2; // Connected
        } else {
            current_connection_state = 1; // No data
        }
    } else {
        current_connection_state = 0; // Error
    }
    
    // Only update if connection state changed
    if (current_connection_state != last_connection_state) {
        switch (current_connection_state) {
            case 2: // Connected
                display.setText(NEXTION_STATUS_TEXT, "CONNECTED");
                display.setForegroundColor(NEXTION_STATUS_TEXT, SimpleNextion::COLOR_GREEN);
                break;
            case 1: // No data
                display.setText(NEXTION_STATUS_TEXT, "NO DATA");
                display.setForegroundColor(NEXTION_STATUS_TEXT, SimpleNextion::COLOR_YELLOW);
                break;
            case 0: // Error
            default:
                display.setText(NEXTION_STATUS_TEXT, "CAN ERROR");
                display.setForegroundColor(NEXTION_STATUS_TEXT, SimpleNextion::COLOR_RED);
                break;
        }
        
        last_connection_state = current_connection_state;
        nextion_commands_sent += 2;
        Serial.printf("📡 Connection status changed to: %s\n", 
                     (current_connection_state == 2) ? "CONNECTED" : 
                     (current_connection_state == 1) ? "NO DATA" : "CAN ERROR");
    }
}

// Multi-rate PWM gauge updates with configurable intervals
void updatePWMGauges() {
    // Speed gauge - configurable rate with change detection
    static unsigned long last_speed_gauge = 0;
    static float last_pwm_speed = -999.0;
    
    if (millis() - last_speed_gauge >= UPDATE_10HZ_INTERVAL) {
       /// PWM_SetGaugeValueSmooth(speed_gauge_channel, 100);
        float speed = CAN_GetVehicleSpeed();
        

        bool speed_fresh = CAN_IsParameterFresh(PARAM_VEHICLE_SPEED, SPEED_DATA_TIMEOUT);

        // Only update PWM if speed changed significantly
        if (speed_gauge_channel >= 0 && speed_fresh && 
            abs(speed - last_pwm_speed) >= SPEED_CHANGE_THRESHOLD) {
            PWM_SetGaugeValueSmooth(speed_gauge_channel, speed);
            last_pwm_speed = speed;
        }
        last_speed_gauge = millis();
    }
    
    // Temperature gauges - configurable rate with change detection
    static unsigned long last_temp_gauge = 0;
    static float last_pwm_temp = -999.0;
    
    if (millis() - last_temp_gauge >= UPDATE_1HZ_INTERVAL) {
        float coolant_temp = CAN_GetCoolantTemperature();
        bool coolant_fresh = CAN_IsHaltechTempFresh(TEMP_DATA_TIMEOUT);
        
        // Only update PWM if temperature changed significantly
        if (coolant_temp_gauge_channel >= 0 && coolant_fresh && 
            abs(coolant_temp - last_pwm_temp) >= TEMP_CHANGE_THRESHOLD) {
            PWM_SetGaugeValueSmooth(coolant_temp_gauge_channel, coolant_temp);
            last_pwm_temp = coolant_temp;
        }
        last_temp_gauge = millis();
    }
    
    // Pressure gauges - configurable rate with change detection
    static unsigned long last_pressure_gauge = 0;
    static float last_pwm_pressure = -999.0;
    
    if (millis() - last_pressure_gauge >= UPDATE_5HZ_INTERVAL) {
        float oil_pressure = CAN_GetOilPressure();
        bool oil_fresh = CAN_IsHaltechPressureFresh(PRESSURE_DATA_TIMEOUT);
        
        // Only update PWM if pressure changed significantly (5 kPa threshold)
        if (oil_pressure_gauge_channel >= 0 && oil_fresh && 
            abs(oil_pressure - last_pwm_pressure) >= 5.0) {
            PWM_SetGaugeValueSmooth(oil_pressure_gauge_channel, oil_pressure);
            last_pwm_pressure = oil_pressure;
        }
        last_pressure_gauge = millis();
    }
}

void printSystemStatus() {
    // Print loop performance
    if (millis() - last_loop_report >= 1000) {
        unsigned long loops_per_second = loop_counter;
        Serial.printf("🔄 Main Loop: %lu loops/sec (%.1f kHz) - Uptime: %lu sec\n", 
                      loops_per_second, loops_per_second / 1000.0, millis() / 1000);
        
        // Print queue status
        if (display.isInitialized()) {
            size_t queue_size = display.getQueueSize();
            Serial.printf("📺 Nextion Queue: %d commands pending\n", queue_size);
            if (queue_size > 10) {
                Serial.printf("⚠️ Queue getting large - consider reducing update rates\n");
            }
            
            // Print command efficiency stats
            unsigned long total_commands = nextion_commands_sent + nextion_commands_skipped;
            if (total_commands > 0) {
                float efficiency = (float)nextion_commands_skipped / total_commands * 100.0;
                Serial.printf("💡 Command efficiency: %.1f%% skipped (%lu sent, %lu skipped)\n", 
                             efficiency, nextion_commands_sent, nextion_commands_skipped);
            }
        }
        
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

// Optional: Display test function (call from setup if needed)
void testDisplay() {
    Serial.println("Testing display elements...");
    
    display.setPage(0);
    delay(1000);
    
    // Test text updates
    display.setText(NEXTION_SPEED_TEXT, "123.4");
    display.setText(NEXTION_GEAR_TEXT, "3");
    display.setText(NEXTION_TEMP_TEXT, "75.5");
    
    // Test colors
    display.setForegroundColor(NEXTION_SPEED_TEXT, SimpleNextion::COLOR_GREEN);
    display.setForegroundColor(NEXTION_GEAR_TEXT, SimpleNextion::COLOR_YELLOW);
    display.setForegroundColor(NEXTION_TEMP_TEXT, SimpleNextion::COLOR_RED);
    
    // Test progress bars
    for (int i = 0; i <= 100; i += 10) {
        display.setProgress(NEXTION_SPEED_BAR, i);
        display.setProgress(NEXTION_TEMP_BAR, i);
        delay(200);
    }
    
    Serial.println("Display test complete");
}

/*
 * SMART UPDATE SYSTEM SUMMARY:
 * =============================
 * 
 * IMMEDIATE UPDATES (change-triggered):
 * - Gear position changes (P/R/N/D/S/F)
 * - Drive gear changes (1,2,3,4 in Drive mode)
 * - Overrun solenoid state changes (ON/OFF)
 * - Lockup solenoid state changes (ON/OFF)
 * - Shift solenoids (tied to gear changes)
 * 
 * THRESHOLD-BASED UPDATES:
 * - Speed: Only when changes by >1 kph (10Hz check rate)
 * - Temperature: Only when changes by >0.5°C (1Hz check rate)
 * - Line pressure: Only when changes by >5% (1Hz check rate)
 * 
 * BACKUP INTERVALS (prevent stale display):
 * - Gear/solenoids: 1 second backup
 * - Speed: 1 second backup
 * - Temperature: 5 second backup
 * - Line pressure: 1 second backup
 * 
 * EFFICIENCY TRACKING:
 * - Commands sent vs skipped are tracked and reported
 * - Debug messages show when changes trigger updates
 * - Queue size monitoring with warnings
 * 
 * EXPECTED TRAFFIC REDUCTION:
 * - Steady-state driving: ~90% reduction in Nextion commands
 * - Active shifting: Immediate response to gear changes
 * - Highway cruising: Minimal Nextion traffic
 */