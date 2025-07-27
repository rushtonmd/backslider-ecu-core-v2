// ===================================
#include <Arduino.h>

#include "main_application.h"
#include "config_manager.h"
#include "storage_manager.h"
#include "w25q128_storage_backend.h"
#include "ecu_config.h"

// Global instances for modules that need them
W25Q128StorageBackend global_storage_backend(ECU_TRANSMISSION_CONFIG);
StorageManager global_storage_manager(&global_storage_backend);
ConfigManager config_manager(&global_storage_manager);

MainApplication app;

void setup() {
    Serial.begin(115200);
    // Note: Removed serial wait loop for maximum startup speed
    
    Serial.println("Backslider ECU Core v2 - Starting...");
    Serial.println("Backslider ECU Core v2 - About to call app.init()...");
    app.init();
    Serial.println("Backslider ECU Core v2 - app.init() completed successfully");
    Serial.println("Backslider ECU Core v2 - Entering main loop...");
}

void loop() {
    app.run();
}
