// main_application.h
// Main ECU application class - coordinates all subsystems

#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

#include <stdint.h>
#include "storage_manager.h"
#include "spi_flash_storage_backend.h"
#include "config_manager.h"


class MainApplication {
public:
    MainApplication();
    void init();
    void run();
    
    // Diagnostics
    uint32_t getLoopCount() const { return loop_count; }
    uint32_t getLastLoopTime() const { return last_loop_time_us; }
    
    // System access
    StorageManager& getStorageManager() { return storage_manager; }
    ConfigManager& getConfigManager() { return config_manager; }
    
    // Configuration access helpers
    const ECUConfiguration& getConfig() const { return config_manager.getConfig(); }

private:
    uint32_t loop_count;
    uint32_t last_loop_time_us;
    uint32_t last_status_report_ms;
    bool external_canbus_initialized;
    
    // Core systems (initialized in this order)
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager;
    ConfigManager config_manager;
    
    void printStatusReport();
};



#endif