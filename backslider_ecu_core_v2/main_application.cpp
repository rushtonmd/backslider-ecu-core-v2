// main_application.cpp
// Main ECU application implementation

#ifdef TESTING
#include "tests/mock_arduino.h"
extern MockSerial Serial;  // Use the Serial object from the test environment
#else
#include <Arduino.h>
#endif

#include "main_application.h"
#include "msg_bus.h"
#include "input_manager.h"
// #include "engine_sensors.h"  // TODO: Create engine_sensors.h when ready
// #include "transmission_sensors.h"  // TODO: Create transmission_sensors.h when ready

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
    g_message_bus.init(false);  // false = no physical CAN bus yet
    
    // Initialize input manager
    Serial.println("Initializing input manager...");
    input_manager_init();
    
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
    
    // TODO: Add other module updates here as they're developed
    // fuel_module_update();
    // ignition_module_update();
    // transmission_module_update();
    
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
    // Heartbeat LED - quick flash every 1000 loops
    if (loop_count % 1000 == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(50);  // Very brief flash
        digitalWrite(LED_BUILTIN, LOW);
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
    
    Serial.println("========================");
}