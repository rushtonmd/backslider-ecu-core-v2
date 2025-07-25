// main_application.cpp
// Main ECU application implementation

#ifdef TESTING
#include "tests/mock_arduino.h"
extern MockSerial Serial;  // Use the Serial object from the test environment
extern MockSerial Serial1; // Additional serial port for external communications
#else
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#endif

#include "main_application.h"
#include "msg_bus.h"
#include "input_manager.h"
#include "output_manager.h"
#include "external_serial.h"
#include "external_canbus.h"
#include "sensor_calibration.h"
#include "transmission_module.h"
#include "ecu_config.h"

// TODO: Create engine_sensors.h when ready
// TODO: Create transmission_sensors.h when ready

// I2C Device Objects
#if defined(ARDUINO) && !defined(TESTING)
Adafruit_ADS1015 ads1015;
Adafruit_MCP23X17 mcp;
#endif

MainApplication::MainApplication() : storage_manager(&storage_backend), config_manager(&storage_manager) {
    // Constructor initializes storage manager with backend and config manager with storage manager
}

void MainApplication::init() {
    loop_count = 0;
    last_loop_time_us = 0;
    last_status_report_ms = 0;
    
    #ifdef ARDUINO
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("=== Backslider ECU Starting ===");
    
    // Initialize built-in LED for status indication
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on during initialization
    #endif
    
    // Initialize message bus (internal messaging only for now)
    Serial.println("Initializing message bus...");
    g_message_bus.init();  // false = no physical CAN bus yet
    
    // Initialize storage manager
    Serial.println("Initializing storage manager...");
    if (storage_manager.init()) {
        Serial.println("Storage manager initialized successfully");
    } else {
        Serial.println("WARNING: Storage manager initialization failed");
    }
    
    // Initialize configuration manager - MUST BE FIRST after storage
    Serial.println("Initializing configuration manager...");
    if (!config_manager.initialize()) {
        Serial.println("CRITICAL ERROR: Configuration manager initialization failed");
        return;
    }
    
    // Get configuration for system initialization
    const ECUConfiguration& config = config_manager.getConfig();
    
    #ifdef ARDUINO
    // Initialize I2C with loaded configuration
    // Note: Teensy 4.0 uses fixed pins (SDA=18, SCL=19)
    Serial.print("Initializing I2C: SDA=18, SCL=19");
    Serial.print(", Freq=");
    Serial.print(config.i2c.bus_frequency);
    Serial.println("Hz");
    
    Wire.begin();
    Wire.setClock(config.i2c.bus_frequency);
    
    // Initialize ADS1015 ADC if enabled
    if (config.i2c.adc.enabled) {
        Serial.print("Initializing ADS1015 ADC at address 0x");
        Serial.print(config.i2c.adc.address, HEX);
        Serial.print(" (freq=");
        Serial.print(config.i2c.adc.frequency);
        Serial.println("Hz)");
        
        #if defined(ARDUINO) && !defined(TESTING)
        if (!ads1015.begin(config.i2c.adc.address)) {
            Serial.println("ERROR: Failed to initialize ADS1015 ADC!");
            digitalWrite(config.pins.error_led_pin, HIGH);  // Turn on error LED
        } else {
            Serial.println("ADS1015 ADC initialized successfully");
            // Configure ADS1015 for single-ended readings
            ads1015.setGain(GAIN_TWOTHIRDS);  // ±6.144V range
        }
        #else
        Serial.println("ADS1015 ADC initialization skipped (not Arduino)");
        #endif
    } else {
        Serial.println("ADS1015 ADC disabled in configuration");
    }
    
    // Initialize MCP23017 GPIO expander if enabled
    if (config.i2c.gpio_expander.enabled) {
        Serial.print("Initializing MCP23017 GPIO expander at address 0x");
        Serial.print(config.i2c.gpio_expander.address, HEX);
        Serial.print(" (freq=");
        Serial.print(config.i2c.gpio_expander.frequency);
        Serial.println("Hz)");
        
        #if defined(ARDUINO) && !defined(TESTING)
        if (!mcp.begin_I2C(config.i2c.gpio_expander.address)) {
            Serial.println("ERROR: Failed to initialize MCP23017 GPIO expander!");
            digitalWrite(config.pins.error_led_pin, HIGH);  // Turn on error LED
        } else {
            Serial.println("MCP23017 GPIO expander initialized successfully");
            // Configure all pins as inputs with pullup by default
            for (int i = 0; i < 16; i++) {
                mcp.pinMode(i, INPUT_PULLUP);
            }
        }
        #else
        Serial.println("MCP23017 GPIO expander initialization skipped (not Arduino)");
        #endif
    } else {
        Serial.println("MCP23017 GPIO expander disabled in configuration");
    }
    
    // Initialize status LEDs with loaded configuration
    pinMode(config.pins.status_led_pin, OUTPUT);
    pinMode(config.pins.error_led_pin, OUTPUT);  
    pinMode(config.pins.activity_led_pin, OUTPUT);
    
    // Set initial LED states
    digitalWrite(config.pins.status_led_pin, HIGH);   // Status ON during init
    digitalWrite(config.pins.error_led_pin, LOW);     // Error OFF
    digitalWrite(config.pins.activity_led_pin, LOW);  // Activity OFF
    #endif
    
    // Initialize input manager
    Serial.println("Initializing input manager...");
    input_manager_init();

    // Initialize output manager (must be before modules that use outputs)
    Serial.println("Initializing output manager...");
    uint8_t output_init_result = output_manager_init();
    if (output_init_result) {
        Serial.println("Output manager initialized successfully");
    } else {
        Serial.println("WARNING: Output manager initialization failed");
    }

    // Initialize external communications
    Serial.println("Initializing external communications...");
    
    // External serial communication using ECU configuration
    if (g_external_serial.init(ECU_TRANSMISSION_CONFIG.external_serial)) {
        Serial.println("External serial communication initialized");
        Serial.print("  USB: ");
        Serial.println(ECU_TRANSMISSION_CONFIG.external_serial.usb.enabled ? "enabled" : "disabled");
        Serial.print("  Serial1: ");
        Serial.println(ECU_TRANSMISSION_CONFIG.external_serial.serial1.enabled ? "enabled" : "disabled");
        Serial.print("  Serial2: ");
        Serial.println(ECU_TRANSMISSION_CONFIG.external_serial.serial2.enabled ? "enabled" : "disabled");
    } else {
        Serial.println("External serial communication initialization failed");
    }
    
    // External CAN bus for OBD-II and custom devices
    external_canbus_config_t canbus_config = {
        .baudrate = 500000,
        .enable_obdii = true,
        .enable_custom_messages = true,
        .can_bus_number = 1,
        .cache_default_max_age_ms = 1000
    };
    
    if (g_external_canbus.init(canbus_config)) {
        Serial.println("External CAN bus initialized (OBD-II + custom devices)");
    } else {
        Serial.println("WARNING: External CAN bus initialization failed");
    }

    // Initialize transmission module
    Serial.println("Initializing transmission module...");
    uint8_t trans_sensors_registered = transmission_module_init();
    Serial.print("Registered ");
    Serial.print(trans_sensors_registered);
    Serial.println(" transmission sensors");
    
    // TODO: Register engine sensors when engine_sensors.h is created
    // Serial.println("Registering engine sensors...");
    // uint8_t engine_sensors_registered = engine_sensors_init();
    // Serial.print("Registered ");
    // Serial.print(engine_sensors_registered);
    // Serial.println(" engine sensors");
    
    // TODO: Register transmission sensors when ready
    // uint8_t trans_sensors_registered = transmission_sensors_init();
    
    #ifdef ARDUINO
    // Turn off status LED and turn on activity LED when initialization complete
    digitalWrite(config.pins.status_led_pin, LOW);      // Status OFF (init complete)
    digitalWrite(config.pins.activity_led_pin, HIGH);   // Activity ON (running)
    #endif
    
    Serial.println("=== ECU Initialization Complete ===");
    Serial.println("Entering main loop...");
}

void MainApplication::run() {
    uint32_t loop_start_us = micros();
    
    // Update all sensors (each sensor manages its own timing)
    input_manager_update();
    
    // Process message bus (route sensor data to modules)
    g_message_bus.process();

    // Update storage manager (handle storage operations and cache management)
    storage_manager.update();

    // Update output manager (controls all physical outputs)
    output_manager_update();

    // Update transmission module
    transmission_module_update();
    
    // Update external communications (share data with other ECUs and devices)
    g_external_serial.update();
    g_external_canbus.update();
    
    // TODO: Add other module updates here as they're developed
    // fuel_module_update();
    // ignition_module_update();
    
    // Calculate loop timing
    uint32_t loop_end_us = micros();
    last_loop_time_us = loop_end_us - loop_start_us;
    loop_count++;
    
    // Print status report periodically
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_status_report_ms >= 5000) {  // Every 5 seconds
        printStatusReport();
        last_status_report_ms = current_time_ms;
    }
    
    #ifdef ARDUINO
    // Heartbeat LED - non-blocking toggle every 1000 loops (no delays)
    if (loop_count % 1000 == 0) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED state
    }
    #endif
}

void MainApplication::printStatusReport() {
    Serial.println("=== ECU Status Report ===");
    
    // System timing
    Serial.print("Loop count: ");
    Serial.println(loop_count);
    Serial.print("Last loop time: ");
    Serial.print(last_loop_time_us);
    Serial.println(" µs");
    
    // Message bus statistics
    Serial.print("Messages processed: ");
    Serial.println(g_message_bus.getMessagesProcessed());
    Serial.print("Queue overflows: ");
    Serial.println(g_message_bus.getQueueOverflows());
    
    // Input manager statistics
    Serial.print("Total sensors: ");
    Serial.println(input_manager_get_sensor_count());
    Serial.print("Valid sensors: ");
    Serial.println(input_manager_get_valid_sensor_count());
    Serial.print("Sensor updates: ");
    Serial.println(input_manager_get_total_updates());
    Serial.print("Sensor errors: ");
    Serial.println(input_manager_get_total_errors());

    // Output manager statistics
    const output_manager_stats_t* output_stats = output_manager_get_stats();
    Serial.print("Total outputs: ");
    Serial.println(output_stats->total_outputs);
    Serial.print("Output updates: ");
    Serial.println(output_stats->total_updates);
    Serial.print("Output errors: ");
    Serial.println(output_stats->fault_count);

    // External communications statistics
    const external_canbus_stats_t& canbus_stats = g_external_canbus.get_statistics();
    Serial.print("CAN messages sent: ");
    Serial.println(canbus_stats.messages_sent);
    Serial.print("CAN messages received: ");
    Serial.println(canbus_stats.messages_received);
    Serial.print("OBD-II requests: ");
    Serial.println(canbus_stats.obdii_requests);
    Serial.print("Cache size: ");
    Serial.println(g_external_canbus.get_cache_size());

        // Transmission statistics
    Serial.print("Current gear: ");
    Serial.println(transmission_gear_to_string(transmission_get_state()->current_gear));
    Serial.print("Fluid temperature: ");
    Serial.print(transmission_get_state()->fluid_temperature);
    Serial.println("°C");
    Serial.print("Transmission shifts: ");
    Serial.println(transmission_get_shift_count());

    // Storage system statistics
    Serial.print("Storage cache hits: ");
    Serial.println(storage_manager.get_cache_hits());
    Serial.print("Storage cache misses: ");
    Serial.println(storage_manager.get_cache_misses());
    Serial.print("Storage disk writes: ");
    Serial.println(storage_manager.get_disk_writes());
    Serial.print("Storage disk reads: ");
    Serial.println(storage_manager.get_disk_reads());

    #ifdef ARDUINO
    // Print I2C device status (now from input_manager)
    print_i2c_status();
    #endif

    Serial.println("========================");
}