// ===================================
#ifdef TESTING
#include "tests/mock_arduino.h"
extern MockSerial Serial;
#else
#include <Arduino.h>
#endif

#include "main_application.h"

MainApplication app;

void setup() {
    Serial.begin(115200);
    // Note: Removed serial wait loop for maximum startup speed
    
    Serial.println("Backslider ECU Core v2 - Starting...");
    app.init();
}

void loop() {
    app.run();
}
