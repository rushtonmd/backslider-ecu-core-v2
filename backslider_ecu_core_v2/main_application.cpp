// main_application.cpp
// Main ECU application implementation

#ifdef TESTING
#include "tests/mock_arduino.h"
extern MockSerial Serial;  // Use the Serial object from the test environment
extern MockSerial Serial1; // Additional serial port for external communications
#else
#include <Arduino.h>
#endif

#include "main_application.h"
#include "msg_bus.h"
#include "input_manager.h"
#include "output_manager.h"
#include "external_serial.h"
#include "external_canbus.h"
#include "sensor_calibration.h"
#include "transmission_module.h"
// #include "engine_sensors.h"  // TODO: Create engine_sensors.h when ready
// #include "transmission_sensors.h"  // TODO: Create transmission_sensors.h when ready

MainApplication::MainApplication() : storage_manager(&storage_backend) {
    // Constructor initializes storage manager with backend
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
    
    // External serial for Arduino-to-Arduino communication
    #ifdef ARDUINO
    #if defined(SERIAL_PORT_HARDWARE1) || defined(HAVE_HWSERIAL1)
    g_external_serial.init(1, &Serial1, 115200);  // Device ID 1, using Serial1 port
    Serial.println("External serial communication initialized");
    #else
    Serial.println("External serial communication (Serial1 not available)");
    #endif
    #else
    // In testing environment, we skip external serial initialization due to type mismatch
    // MockSerial* cannot be passed to HardwareSerial* parameter
    Serial.println("External serial communication (mock mode - init skipped)");
    #endif
    
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
    digitalWrite(LED_BUILTIN, LOW);  // Turn off LED when initialization complete
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
    #ifdef ARDUINO
    #if defined(SERIAL_PORT_HARDWARE1) || defined(HAVE_HWSERIAL1)
    g_external_serial.update();
    #endif
    #else
    // In testing environment, external serial update is skipped (not initialized)
    #endif
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

    Serial.println("========================");
}