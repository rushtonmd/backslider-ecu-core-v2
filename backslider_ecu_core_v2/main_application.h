// main_application.h
// Main ECU application class - coordinates all subsystems

#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

#include <stdint.h>
#include "storage_manager.h"
#include "storage_backend.h"

class MainApplication {
public:
    MainApplication();
    void init();
    void run();
    
    // Diagnostics
    uint32_t getLoopCount() const { return loop_count; }
    uint32_t getLastLoopTime() const { return last_loop_time_us; }
    
    // Storage system access
    StorageManager& getStorageManager() { return storage_manager; }

private:
    uint32_t loop_count;
    uint32_t last_loop_time_us;
    uint32_t last_status_report_ms;
    
    // Storage system
    EEPROMStorageBackend storage_backend;
    StorageManager storage_manager;
    
    void printStatusReport();
};

#endif