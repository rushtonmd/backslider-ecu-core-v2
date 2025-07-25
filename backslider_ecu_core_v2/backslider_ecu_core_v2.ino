// ===================================
#ifdef TESTING
#include "tests/mock_arduino.h"
extern MockSerial Serial;
#else
#include <Arduino.h>
#endif

#include "main_application.h"
#include "config_manager.h"
#include "storage_manager.h"
#include "spi_flash_storage_backend.h"

// Global instances for modules that need them
SPIFlashStorageBackend global_storage_backend;
StorageManager global_storage_manager(&global_storage_backend);
ConfigManager config_manager(&global_storage_manager);

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
