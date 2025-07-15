#include "config_manager.h"
#include <string.h>

#ifndef ARDUINO
#include "../tests/mock_arduino.h"
extern MockSerial Serial;
#else
#include <Arduino.h>
#endif

// Static configuration storage keys
const char* ConfigManager::CONFIG_KEY_ECU_TYPE = "cfg_ecu_type";
const char* ConfigManager::CONFIG_KEY_ECU_NAME = "cfg_ecu_name";
const char* ConfigManager::CONFIG_KEY_SERIAL_NUMBER = "cfg_serial_num";
const char* ConfigManager::CONFIG_KEY_FIRMWARE_VERSION = "cfg_fw_ver";
const char* ConfigManager::CONFIG_KEY_BOOT_TIMEOUT = "cfg_boot_timeout";

ConfigManager::ConfigManager(StorageManager* storage_mgr) 
    : storage(storage_mgr), config_loaded(false) {
    // Initialize with default config
    memset(&current_config, 0, sizeof(current_config));
}

bool ConfigManager::initialize() {
    if (!storage) {
        Serial.println("ERROR: ConfigManager - No storage manager provided");
        return false;
    }
    
    Serial.println("ConfigManager: Initializing...");
    
    // Try to load configuration from storage first
    if (loadConfigurationFromStorage()) {
        Serial.println("ConfigManager: Configuration loaded from storage");
        config_loaded = true;
    } else {
        Serial.println("ConfigManager: No valid configuration found, loading defaults");
        
        // Load default configuration
        if (!loadDefaultConfiguration()) {
            Serial.println("ERROR: ConfigManager - Failed to load default configuration");
            return false;
        }
        
        // Save default configuration to storage
        if (saveConfigurationToStorage()) {
            Serial.println("ConfigManager: Default configuration saved to storage");
        } else {
            Serial.println("WARNING: ConfigManager - Failed to save default configuration");
        }
        
        config_loaded = true;
    }
    
    // Validate the loaded configuration
    if (!validateConfiguration()) {
        Serial.println("ERROR: ConfigManager - Configuration validation failed");
        return false;
    }
    
    // Print configuration summary
    printConfigurationSummary();
    
    return true;
}

bool ConfigManager::loadDefaultConfiguration() {
    // Load the default transmission configuration
    memcpy(&current_config, &ECU_TRANSMISSION_CONFIG, sizeof(ECUConfiguration));
    
    Serial.println("ConfigManager: Default transmission configuration loaded");
    return true;
}

bool ConfigManager::loadConfigurationFromStorage() {
    if (!storage) return false;
    
    // Try to load basic configuration items
    uint8_t ecu_type_val;
    if (storage->load_data(CONFIG_KEY_ECU_TYPE, &ecu_type_val, sizeof(ecu_type_val))) {
        // Load default config first
        loadDefaultConfiguration();
        
        // Override with stored values
        current_config.ecu_type = (ECUType)ecu_type_val;
        
        // Load other stored values if they exist
        storage->load_data(CONFIG_KEY_ECU_NAME, current_config.ecu_name, sizeof(current_config.ecu_name));
        storage->load_data(CONFIG_KEY_SERIAL_NUMBER, &current_config.serial_number, sizeof(current_config.serial_number));
        storage->load_data(CONFIG_KEY_FIRMWARE_VERSION, current_config.firmware_version, sizeof(current_config.firmware_version));
        storage->load_data(CONFIG_KEY_BOOT_TIMEOUT, &current_config.boot_timeout_ms, sizeof(current_config.boot_timeout_ms));
        
        return true;
    }
    
    return false;
}

bool ConfigManager::saveConfigurationToStorage() {
    if (!storage) return false;
    
    bool success = true;
    
    // Save basic configuration items
    uint8_t ecu_type_val = (uint8_t)current_config.ecu_type;
    success &= storage->save_data(CONFIG_KEY_ECU_TYPE, &ecu_type_val, sizeof(ecu_type_val));
    success &= storage->save_data(CONFIG_KEY_ECU_NAME, current_config.ecu_name, sizeof(current_config.ecu_name));
    success &= storage->save_data(CONFIG_KEY_SERIAL_NUMBER, &current_config.serial_number, sizeof(current_config.serial_number));
    success &= storage->save_data(CONFIG_KEY_FIRMWARE_VERSION, current_config.firmware_version, sizeof(current_config.firmware_version));
    success &= storage->save_data(CONFIG_KEY_BOOT_TIMEOUT, &current_config.boot_timeout_ms, sizeof(current_config.boot_timeout_ms));
    
    return success;
}

bool ConfigManager::validateConfiguration() {
    // Validate ECU type
    if (current_config.ecu_type > ECU_FUEL) {
        Serial.println("ERROR: Invalid ECU type");
        return false;
    }
    
    // Validate pin assignments (basic checks)
    if (current_config.pins.qspi_cs_pin > 39) {
        Serial.println("ERROR: Invalid QSPI CS pin");
        return false;
    }
    
    if (current_config.pins.i2c_sda_pin > 39 || current_config.pins.i2c_scl_pin > 39) {
        Serial.println("ERROR: Invalid I2C pin assignment");
        return false;
    }
    
    // Validate I2C addresses
    if (current_config.i2c.gpio_expander.address > 0x7F) {
        Serial.println("ERROR: Invalid GPIO expander I2C address");
        return false;
    }
    
    if (current_config.i2c.adc.address > 0x7F) {
        Serial.println("ERROR: Invalid ADC I2C address");
        return false;
    }
    
    // Validate frequencies
    if (current_config.i2c.bus_frequency > 1000000) {
        Serial.println("ERROR: I2C frequency too high");
        return false;
    }
    
    return true;
}

void ConfigManager::printConfiguration() {
    Serial.println("\n=== ECU Configuration ===");
    Serial.print("ECU Type: ");
    Serial.print(current_config.ecu_type);
    Serial.print(" (");
    Serial.print(current_config.ecu_name);
    Serial.println(")");
    
    Serial.print("Firmware Version: ");
    Serial.println(current_config.firmware_version);
    
    Serial.print("Serial Number: 0x");
    Serial.println(current_config.serial_number, HEX);
    
    Serial.println("\n--- Pin Assignments ---");
    Serial.print("QSPI CS: ");
    Serial.println(current_config.pins.qspi_cs_pin);
    Serial.print("I2C SDA: ");
    Serial.print(current_config.pins.i2c_sda_pin);
    Serial.print(", SCL: ");
    Serial.println(current_config.pins.i2c_scl_pin);
    Serial.print("CAN TX: ");
    Serial.print(current_config.pins.can_tx_pin);
    Serial.print(", RX: ");
    Serial.println(current_config.pins.can_rx_pin);
    Serial.print("Serial TX: ");
    Serial.print(current_config.pins.ext_serial_tx_pin);
    Serial.print(", RX: ");
    Serial.println(current_config.pins.ext_serial_rx_pin);
    
    Serial.println("\n--- I2C Configuration ---");
    Serial.print("Bus Frequency: ");
    Serial.print(current_config.i2c.bus_frequency);
    Serial.println(" Hz");
    Serial.print("MCP23017 GPIO Expander: 0x");
    Serial.print(current_config.i2c.gpio_expander.address, HEX);
    Serial.print(" (");
    Serial.print(current_config.i2c.gpio_expander.enabled ? "enabled" : "disabled");
    Serial.println(")");
    Serial.print("ADS1115 ADC: 0x");
    Serial.print(current_config.i2c.adc.address, HEX);
    Serial.print(" (");
    Serial.print(current_config.i2c.adc.enabled ? "enabled" : "disabled");
    Serial.println(")");
    
    Serial.println("\n--- SPI Configuration ---");
    Serial.print("QSPI Flash: CS=");
    Serial.print(current_config.spi.qspi_flash.cs_pin);
    Serial.print(", Freq=");
    Serial.print(current_config.spi.qspi_flash.frequency);
    Serial.print(" Hz (");
    Serial.print(current_config.spi.qspi_flash.enabled ? "enabled" : "disabled");
    Serial.println(")");
    
    Serial.println("\n--- Boot Configuration ---");
    Serial.print("Boot Timeout: ");
    Serial.print(current_config.boot_timeout_ms);
    Serial.println(" ms");
    Serial.print("Watchdog: ");
    Serial.println(current_config.enable_watchdog ? "enabled" : "disabled");
    Serial.print("Debug Output: ");
    Serial.println(current_config.enable_debug_output ? "enabled" : "disabled");
    
    Serial.println("\n--- Transmission Settings ---");
    Serial.print("Shift Monitoring: ");
    Serial.println(current_config.transmission.enable_shift_monitoring ? "enabled" : "disabled");
    Serial.print("Pressure Control: ");
    Serial.println(current_config.transmission.enable_pressure_control ? "enabled" : "disabled");
    Serial.print("Temperature Monitoring: ");
    Serial.println(current_config.transmission.enable_temperature_monitoring ? "enabled" : "disabled");
    Serial.print("Shift Debounce: ");
    Serial.print(current_config.transmission.shift_debounce_ms);
    Serial.println(" ms");
    
    Serial.println("========================\n");
}

void ConfigManager::printConfigurationSummary() {
    Serial.println("\n=== Configuration Summary ===");
    Serial.print("ECU: ");
    Serial.print(current_config.ecu_name);
    Serial.print(" v");
    Serial.println(current_config.firmware_version);
    
    Serial.print("I2C: MCP23017(0x");
    Serial.print(current_config.i2c.gpio_expander.address, HEX);
    Serial.print("), ADS1115(0x");
    Serial.print(current_config.i2c.adc.address, HEX);
    Serial.println(")");
    
    Serial.print("Pins: CAN(");
    Serial.print(current_config.pins.can_tx_pin);
    Serial.print("/");
    Serial.print(current_config.pins.can_rx_pin);
    Serial.print("), Serial(");
    Serial.print(current_config.pins.ext_serial_tx_pin);
    Serial.print("/");
    Serial.print(current_config.pins.ext_serial_rx_pin);
    Serial.println(")");
    
    Serial.println("=============================\n");
}

// Runtime configuration updates
bool ConfigManager::updateECUType(ECUType new_type) {
    if (new_type > ECU_FUEL) return false;
    
    current_config.ecu_type = new_type;
    return saveConfigurationToStorage();
}

bool ConfigManager::updateECUName(const char* new_name) {
    if (!new_name || strlen(new_name) >= sizeof(current_config.ecu_name)) return false;
    
    strcpy(current_config.ecu_name, new_name);
    return saveConfigurationToStorage();
}

bool ConfigManager::updateSerialNumber(uint32_t new_serial) {
    current_config.serial_number = new_serial;
    return saveConfigurationToStorage();
}

bool ConfigManager::updateBootTimeout(uint32_t timeout_ms) {
    if (timeout_ms > 30000) return false;  // Max 30 seconds
    
    current_config.boot_timeout_ms = timeout_ms;
    return saveConfigurationToStorage();
}

bool ConfigManager::updateStatusReportInterval(uint32_t interval_ms) {
    if (interval_ms < 100 || interval_ms > 10000) return false;  // 100ms to 10s
    
    current_config.status_report_interval_ms = interval_ms;
    return saveConfigurationToStorage();
}

bool ConfigManager::resetToDefaults() {
    Serial.println("ConfigManager: Resetting to default configuration");
    
    if (!loadDefaultConfiguration()) {
        return false;
    }
    
    if (!saveConfigurationToStorage()) {
        Serial.println("WARNING: Failed to save default configuration");
        return false;
    }
    
    Serial.println("ConfigManager: Reset to defaults complete");
    return true;
}

uint16_t ConfigManager::calculateConfigChecksum() {
    // Simple checksum calculation
    uint16_t checksum = 0;
    uint8_t* data = (uint8_t*)&current_config;
    
    for (size_t i = 0; i < sizeof(ECUConfiguration); i++) {
        checksum += data[i];
    }
    
    return checksum;
} 