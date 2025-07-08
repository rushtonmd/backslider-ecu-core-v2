#include <Arduino.h>

/*
 * Backslider ECU Core v2 - Minimal Arduino Sketch
 * 
 * This is the foundation of an automotive ECU (Engine Control Unit) project
 * designed for the Teensy 4.1 microcontroller.
 * 
 * Current Features:
 * - Basic system initialization
 * - Serial communication at 115200 baud
 * - Built-in LED heartbeat indicator
 * - Basic diagnostic output
 * 
 * Future Development:
 * - Unified message bus architecture for internal and CAN communication
 * - Modular ECU control systems (fuel, ignition, sensors)
 * - Real-time performance optimization
 * - Safety-critical system design
 * 
 * Hardware Target: Teensy 4.1 (600 MHz ARM Cortex-M7)
 * 
 * Author: Backslider ECU Project
 * License: See LICENSE file
 */

// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================

// System Configuration
#define SERIAL_BAUD_RATE 115200
#define LED_BLINK_INTERVAL 500  // milliseconds
#define HEARTBEAT_INTERVAL 1000 // milliseconds

// Pin Definitions for Teensy 4.1
#define BUILTIN_LED_PIN 13  // Built-in LED pin

// System Information
#define ECU_VERSION "0.1.0"
#define ECU_BUILD_DATE __DATE__ " " __TIME__

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Timing variables
unsigned long lastLedToggle = 0;
unsigned long lastHeartbeat = 0;
bool ledState = false;

// System state
bool systemInitialized = false;
unsigned long systemStartTime = 0;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Wait for serial connection (optional - comment out for standalone operation)
  while (!Serial && millis() < 3000) {
    // Wait up to 3 seconds for serial connection
  }
  
  // Initialize hardware
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, LOW);
  
  // Print startup banner
  printStartupBanner();
  
  // Initialize system timing
  systemStartTime = millis();
  lastLedToggle = systemStartTime;
  lastHeartbeat = systemStartTime;
  
  // Mark system as initialized
  systemInitialized = true;
  
  Serial.println("ECU initialization complete - entering main loop");
  Serial.println("=====================================");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Handle LED blinking (visual heartbeat)
  if (currentTime - lastLedToggle >= LED_BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(BUILTIN_LED_PIN, ledState);
    lastLedToggle = currentTime;
  }
  
  // Handle serial heartbeat output
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    printHeartbeat(currentTime);
    lastHeartbeat = currentTime;
  }
  
  // Check for serial input (basic command interface)
  if (Serial.available()) {
    handleSerialInput();
  }
  
  // Main ECU processing would go here in future versions
  // For now, just maintain the heartbeat
}

// ============================================================================
// SUPPORT FUNCTIONS
// ============================================================================

void printStartupBanner() {
  Serial.println();
  Serial.println("=====================================");
  Serial.println("    BACKSLIDER ECU CORE v2");
  Serial.println("=====================================");
  Serial.print("Version: ");
  Serial.println(ECU_VERSION);
  Serial.print("Build Date: ");
  Serial.println(ECU_BUILD_DATE);
  Serial.print("Target: Teensy 4.1 (");
  Serial.print(F_CPU / 1000000);
  Serial.println(" MHz)");
  Serial.println("=====================================");
  Serial.println();
  
  Serial.println("System Status:");
  Serial.println("- Serial communication: OK");
  Serial.println("- Built-in LED: OK");
  Serial.println("- System clock: OK");
  Serial.println();
  
  Serial.println("Future Features (Development Roadmap):");
  Serial.println("- Unified message bus architecture");
  Serial.println("- CAN communication interface");
  Serial.println("- Engine sensor processing");
  Serial.println("- Fuel injection control");
  Serial.println("- Ignition timing control");
  Serial.println("- Diagnostic trouble codes");
  Serial.println();
}

void printHeartbeat(unsigned long currentTime) {
  float uptimeSeconds = (currentTime - systemStartTime) / 1000.0;
  
  Serial.print("ECU Heartbeat - Uptime: ");
  Serial.print(uptimeSeconds, 1);
  Serial.print("s, Free RAM: ");
  Serial.print(getFreeRAM());
  Serial.print(" bytes, LED: ");
  Serial.println(ledState ? "ON" : "OFF");
}

void handleSerialInput() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.equalsIgnoreCase("status")) {
    printSystemStatus();
  } else if (command.equalsIgnoreCase("help")) {
    printHelp();
  } else if (command.equalsIgnoreCase("reset")) {
    Serial.println("System reset requested...");
    delay(1000);
    // Software reset for Teensy
    SCB_AIRCR = 0x05FA0004;
  } else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
    Serial.println("Type 'help' for available commands");
  }
}

void printSystemStatus() {
  unsigned long currentTime = millis();
  float uptimeSeconds = (currentTime - systemStartTime) / 1000.0;
  
  Serial.println();
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Uptime: ");
  Serial.print(uptimeSeconds, 1);
  Serial.println(" seconds");
  Serial.print("Free RAM: ");
  Serial.print(getFreeRAM());
  Serial.println(" bytes");
  Serial.print("CPU Frequency: ");
  Serial.print(F_CPU / 1000000);
  Serial.println(" MHz");
  Serial.print("System Initialized: ");
  Serial.println(systemInitialized ? "YES" : "NO");
  Serial.print("Built-in LED State: ");
  Serial.println(ledState ? "ON" : "OFF");
  Serial.println("====================");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println("status  - Show system status");
  Serial.println("help    - Show this help message");
  Serial.println("reset   - Reset the system");
  Serial.println("===========================");
  Serial.println();
}

// Get free RAM (approximate) - Teensy 4.1 compatible
uint32_t getFreeRAM() {
  char top;
  extern char _ebss;
  return &top - &_ebss;
} 