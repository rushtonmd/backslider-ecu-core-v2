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
    
    // Core systems (initialized in this order)
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager;
    ConfigManager config_manager;
    
    void printStatusReport();
};

// =============================================================================
// I2C DEVICE HELPER FUNCTIONS (Arduino only)
// =============================================================================

#ifdef ARDUINO
// Function to read from ADS1015 ADC
int16_t read_ads1015_channel(uint8_t channel);

// Function to read from MCP23017 GPIO expander
bool read_mcp23017_pin(uint8_t pin);

// Function to write to MCP23017 GPIO expander
void write_mcp23017_pin(uint8_t pin, bool value);

// Function to configure MCP23017 pin mode
void configure_mcp23017_pin(uint8_t pin, uint8_t mode);

// Function to print I2C device status
void print_i2c_status();
#endif

#endif