// ===================================
#include "main_application.h"

MainApplication app;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection
    }
    
    Serial.println("Backslider ECU Core v2 - Starting...");
    app.init();
}

void loop() {
    app.run();
}
