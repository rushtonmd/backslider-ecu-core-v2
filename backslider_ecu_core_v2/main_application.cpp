// main_application.cpp
// Main ECU application implementation

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>

#include "main_application.h"
#include "msg_bus.h"
#include "input_manager.h"
#include "output_manager.h"
#include "external_serial.h"
#include "external_canbus.h"
#include "sensor_calibration.h"
#include "transmission_module.h"
#include "ecu_config.h"
#include "parameter_registry.h"
#include "external_message_broadcasting.h"
#include "pin_assignments.h"

// TODO: Create engine_sensors.h when ready
// TODO: Create transmission_sensors.h when ready



// I2C Device Objects
#if defined(ARDUINO) && !defined(TESTING)
Adafruit_ADS1015 ads1015;
Adafruit_MCP23X17 mcp;

// Helper function to get the correct I2C bus based on device number
TwoWire* getI2CBus(uint8_t device_number) {
    switch (device_number) {
        case 0: return &Wire;   // Primary I2C bus
        case 1: return &Wire;   // TODO: Return Wire1 when available
        case 2: return &Wire;   // TODO: Return Wire2 when available
        default: return &Wire;  // Default to primary
    }
}
#endif

MainApplication::MainApplication() : storage_manager(&storage_backend), config_manager(&storage_manager) {
    // Constructor initializes storage manager with backend and config manager with storage manager
}

void MainApplication::init() {
    loop_count = 0;
    last_loop_time_us = 0;
    last_status_report_ms = 0;
    loops_per_second = 0;
    last_loop_stats_reset_ms = 0;
    external_canbus_initialized = false;
    
    #ifdef ARDUINO
    // Initialize serial communication
    Serial.begin(115200);  // Revert to standard baud rate
    Serial.println("=== Backslider ECU Starting ===");
    
    // Note: LED initialization removed to avoid pin conflicts
    #endif
    
    // Initialize message bus (internal messaging only for now)
    Serial.println("Initializing message bus...");
    Serial.println("  - About to call g_message_bus.init()...");
    g_message_bus.init();  // false = no physical CAN bus yet
    Serial.println("  - g_message_bus.init() completed");
    
    // Set up parameter registry as global broadcast handler for parameter requests
    Serial.println("Setting up parameter registry...");
    g_message_bus.setGlobalBroadcastHandler(ParameterRegistry::handle_parameter_request);
    Serial.println("  - Parameter registry set as global broadcast handler");
    
    // Initialize storage manager
    Serial.println("Initializing storage manager...");
    Serial.println("  - About to call storage_manager.init()...");
    if (storage_manager.init()) {
        Serial.println("  - Storage manager init() returned true");
        Serial.println("Storage manager initialized successfully");
        
        // Run comprehensive storage diagnostics
        Serial.println("  - About to run storage diagnostics...");
        storage_manager.run_storage_diagnostics();
        Serial.println("  - Storage diagnostics completed");
    } else {
        Serial.println("  - Storage manager init() returned false");
        Serial.println("WARNING: Storage manager initialization failed");
    }
    
    // Initialize configuration manager - MUST BE FIRST after storage
    Serial.println("Initializing configuration manager...");
    Serial.println("  - About to call config_manager.initialize()...");
    if (!config_manager.initialize()) {
        Serial.println("  - Config manager initialize() returned false");
        Serial.println("CRITICAL ERROR: Configuration manager initialization failed");
        return;
    }
    Serial.println("  - Config manager initialize() returned true");
    
    // Get configuration for system initialization
    Serial.println("  - About to call config_manager.getConfig()...");
    const ECUConfiguration& config = config_manager.getConfig();
    
    #ifdef ARDUINO
    // Initialize I2C buses based on configuration
    Serial.println("  - About to initialize I2C buses...");
    if (config.i2c.number_of_interfaces > 0) {
        Serial.print("Initializing ");
        Serial.print(config.i2c.number_of_interfaces);
        Serial.println(" I2C interface(s)...");
        
        // Initialize primary I2C bus (Wire) if needed
        if (config.i2c.number_of_interfaces >= 1) {
            Serial.println("  Wire (Primary): SDA=18, SCL=19");
            Wire.begin();
            Wire.setClock(config.i2c.bus_frequency);
        }
        
        // Initialize secondary I2C bus (Wire1) if needed
        if (config.i2c.number_of_interfaces >= 2) {
            Serial.println("  Wire1 (Secondary): SDA=20, SCL=21");
            // Wire1.begin();  // TODO: Enable when Wire1 is available
            // Wire1.setClock(config.i2c.bus_frequency);
        }
        
        // Initialize tertiary I2C bus (Wire2) if needed
        if (config.i2c.number_of_interfaces >= 3) {
            Serial.println("  Wire2 (Tertiary): SDA=24, SCL=25");
            // Wire2.begin();  // TODO: Enable when Wire2 is available
            // Wire2.setClock(config.i2c.bus_frequency);
        }
        
        Serial.println("I2C initialization complete");
    } else {
        Serial.println("I2C initialization skipped - no interfaces configured");
    }
    
    // Initialize ADS1015 ADC if enabled
    Serial.println("  - About to initialize ADS1015...");
    if (config.i2c.adc.enabled) {
        // Validate device number
        if (config.i2c.adc.device_number >= config.i2c.number_of_interfaces) {
            Serial.print("ERROR: ADS1015 device_number (");
            Serial.print(config.i2c.adc.device_number);
            Serial.print(") exceeds available I2C interfaces (");
            Serial.print(config.i2c.number_of_interfaces);
            Serial.println(")");
            digitalWrite(config.pins.error_led_pin, HIGH);
        } else {
            Serial.print("Initializing ADS1015 ADC on I2C bus ");
            Serial.print(config.i2c.adc.device_number);
            Serial.print(" at address 0x");
            Serial.print(config.i2c.adc.address, HEX);
            Serial.print(" (freq=");
            Serial.print(config.i2c.adc.frequency);
            Serial.println("Hz)");
            
            #if defined(ARDUINO) && !defined(TESTING)
            // Note: Adafruit libraries currently use Wire by default
            // TODO: Implement multi-bus support for Adafruit libraries
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
        }
    } else {
        Serial.println("ADS1015 ADC disabled in configuration");
    }
    
    // Initialize MCP23017 GPIO expander if enabled
    Serial.println("  - About to initialize MCP23017...");
    if (config.i2c.gpio_expander.enabled) {
        // Validate device number
        if (config.i2c.gpio_expander.device_number >= config.i2c.number_of_interfaces) {
            Serial.print("ERROR: MCP23017 device_number (");
            Serial.print(config.i2c.gpio_expander.device_number);
            Serial.print(") exceeds available I2C interfaces (");
            Serial.print(config.i2c.number_of_interfaces);
            Serial.println(")");
            digitalWrite(config.pins.error_led_pin, HIGH);
        } else {
            Serial.print("Initializing MCP23017 GPIO expander on I2C bus ");
            Serial.print(config.i2c.gpio_expander.device_number);
            Serial.print(" at address 0x");
            Serial.print(config.i2c.gpio_expander.address, HEX);
            Serial.print(" (freq=");
            Serial.print(config.i2c.gpio_expander.frequency);
            Serial.println("Hz)");
            
            #if defined(ARDUINO) && !defined(TESTING)
            // Note: Adafruit libraries currently use Wire by default
            // TODO: Implement multi-bus support for Adafruit libraries
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
        }
    } else {
        Serial.println("MCP23017 GPIO expander disabled in configuration");
    }
    
    // Initialize status LEDs with loaded configuration
    Serial.println("  - About to initialize LEDs...");
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
    Serial.println("  - About to call input_manager_init()...");
    input_manager_init();

    // Initialize output manager (must be before modules that use outputs)
    Serial.println("Initializing output manager...");
    Serial.println("  - About to call output_manager_init()...");
    uint8_t output_init_result = output_manager_init();
    if (output_init_result) {
        Serial.println("Output manager initialized successfully");
    } else {
        Serial.println("WARNING: Output manager initialization failed");
    }

    // Initialize external communications
    Serial.println("Initializing external communications...");
    
    // Test external serial initialization step by step
    Serial.println("  - About to call g_external_serial.init()...");
    Serial.println("  - Config: USB=");
    Serial.print(ECU_TRANSMISSION_CONFIG.external_serial.usb.enabled ? "enabled" : "disabled");
    Serial.println(", Serial1=");
    Serial.print(ECU_TRANSMISSION_CONFIG.external_serial.serial1.enabled ? "enabled" : "disabled");
    Serial.println(", Serial2=");
    Serial.print(ECU_TRANSMISSION_CONFIG.external_serial.serial2.enabled ? "enabled" : "disabled");
    Serial.println();
    
    bool external_serial_result = g_external_serial.init(ECU_TRANSMISSION_CONFIG.external_serial);
    
    if (external_serial_result) {
        Serial.println("  - External serial init() returned true");
        Serial.println("External serial communication initialized");
    } else {
        Serial.println("  - External serial init() returned false");
        Serial.println("External serial communication initialization failed");
    }
    
    // External CAN bus for OBD-II and custom devices
    if (ECU_TRANSMISSION_CONFIG.external_canbus.enabled) {
        Serial.println("  - About to initialize external CAN bus...");
        external_canbus_initialized = g_external_canbus.init(ECU_TRANSMISSION_CONFIG.external_canbus);
        if (external_canbus_initialized) {
            Serial.println("  - External CAN bus init() returned true");
            Serial.println("External CAN bus initialized");
            Serial.print("  Baudrate: ");
            Serial.print(ECU_TRANSMISSION_CONFIG.external_canbus.baudrate);
            Serial.println(" bps");
            Serial.print("  OBD-II: ");
            Serial.println(ECU_TRANSMISSION_CONFIG.external_canbus.enable_obdii ? "enabled" : "disabled");
            Serial.print("  Custom messages: ");
            Serial.println(ECU_TRANSMISSION_CONFIG.external_canbus.enable_custom_messages ? "enabled" : "disabled");
        } else {
            Serial.println("  - External CAN bus init() returned false");
            Serial.println("WARNING: External CAN bus initialization failed");
        }
    } else {
        Serial.println("  - External CAN bus disabled in configuration - skipping initialization");
        external_canbus_initialized = false;
    }

    // Initialize external message broadcasting FIRST
    Serial.println("Initializing external message broadcasting...");
    ExternalMessageBroadcasting::init();
    
    // Set up external interfaces for broadcasting
    ExternalMessageBroadcasting::set_external_interfaces(&g_external_canbus, &g_external_serial);
    
    // Initialize transmission module
    Serial.println("Initializing transmission module...");
    Serial.println("  - About to call transmission_module_init()...");
    uint8_t trans_sensors_registered = transmission_module_init();
    Serial.println("  - transmission_module_init() completed");
    Serial.print("Registered ");
    Serial.print(trans_sensors_registered);
    Serial.println(" transmission sensors");
    
    // Initialize transmission sensors and controls - transmission_init() is already called above
    // transmission_init();
    
    // TODO: Register engine sensors when engine_sensors.h is created
    
    Serial.println("External message broadcasting initialized - ready for module registrations");
    // Serial.println("Registering engine sensors...");
    // uint8_t engine_sensors_registered = engine_sensors_init();
    // Serial.print("Registered ");
    // Serial.print(engine_sensors_registered);
    // Serial.println(" engine sensors");
    
    // TODO: Register transmission sensors when ready
    // uint8_t trans_sensors_registered = transmission_sensors_init();
    
    #ifdef ARDUINO
    // Note: LED state changes removed to avoid pin conflicts
    #endif
    
    Serial.println("=== ECU Initialization Complete ===");
    Serial.println("Entering main loop...");
}

void MainApplication::run() {
    uint32_t loop_start_us = micros();
    
    #ifdef ARDUINO
    static uint32_t last_run_debug = 0;
    uint32_t run_now = millis();
    if (run_now - last_run_debug >= 5000) {  // Every 5 seconds
        Serial.print("MainApplication: Main loop running - loops/sec: ");
        Serial.println(loops_per_second);
        last_run_debug = run_now;
    }
    #endif
    
    // Update all sensors (each sensor manages its own timing)
    input_manager_update();
    
    // Process message bus (route sensor data to modules)
    #ifdef ARDUINO
    static uint32_t last_process_debug = 0;
    uint32_t now = millis();
    if (now - last_process_debug >= 2000) {  // Every 2 seconds
        Serial.print("MainApplication: Processing message bus - queue size: ");
        Serial.print(g_message_bus.getQueueSize());
        Serial.print(", messages/sec: ");
        Serial.print(g_message_bus.getMessagesPerSecond());
        Serial.print(", total published: ");
        Serial.println(g_message_bus.getMessagesPublished());
        last_process_debug = now;
    }
    #endif
    g_message_bus.process();

    // Update storage manager (handle storage operations and cache management)
    storage_manager.update();

    // Update output manager (controls all physical outputs)
    output_manager_update();

    // Update transmission module
    transmission_module_update();
    
    // Update external communications (share data with other ECUs and devices)
    g_external_serial.update();
    
    // Update external CAN bus only if initialization was successful
    if (external_canbus_initialized) {
        g_external_canbus.update();
    }
    
    // Update external message broadcasting
    ExternalMessageBroadcasting::update();
    
    // Calculate loop timing
    uint32_t loop_end_us = micros();
    last_loop_time_us = loop_end_us - loop_start_us;
    
    // Update loop count and calculate loops per second
    loop_count++;
    uint32_t now_ms = millis();
    if (now_ms - last_loop_stats_reset_ms >= 1000) {  // Every second
        loops_per_second = loop_count;
        loop_count = 0;  // Reset counter
        last_loop_stats_reset_ms = now_ms;
    }
    
    // Publish heartbeat
    // Update loop count for runtime monitoring
    
    #ifdef ARDUINO
    // Note: Heartbeat LED removed to avoid pin conflicts
    #endif
}

void MainApplication::printStatusReport() {
    Serial.println("=== ECU Status Report ===");
    
    // System timing
    Serial.print("Loops per second: ");
    Serial.println(loops_per_second);
    Serial.print("Last loop time: ");
    Serial.print(last_loop_time_us);
    Serial.println(" µs");
    
    // Message bus statistics
    Serial.print("Messages processed: ");
    Serial.println(g_message_bus.getMessagesProcessed());
    Serial.print("Messages published: ");
    Serial.println(g_message_bus.getMessagesPublished());
    Serial.print("Messages per second: ");
    Serial.println(g_message_bus.getMessagesPerSecond());
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