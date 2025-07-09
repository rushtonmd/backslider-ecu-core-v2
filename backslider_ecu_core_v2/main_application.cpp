// FILE 3: main_application.cpp
// =============================
#ifdef TESTING
#include "tests/mock_arduino.h"
#else
#include <Arduino.h>
#endif
#include "main_application.h"

void MainApplication::init() {
    pinMode(LED_BUILTIN, OUTPUT);
    lastBlink = 0;
    ledState = false;
    Serial.println("MainApplication initialized");
}

void MainApplication::run() {
    // Simple LED blink every 500ms
    unsigned long currentTime = millis();
    
    if (currentTime - lastBlink >= 500) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = currentTime;
        
        Serial.print("LED: ");
        Serial.println(ledState ? "ON" : "OFF");
    }
}