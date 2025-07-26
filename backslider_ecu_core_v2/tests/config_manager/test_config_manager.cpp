#include <iostream>
#include <cassert>
#include <cstring>
#include "../mock_arduino.h"
#include "../config_manager.h"
#include "../storage_manager.h"
#include "../spi_flash_storage_backend.h"
#include "../msg_bus.h"

// Global message bus for testing (extern declaration)
extern MessageBus g_message_bus;

// Test helper functions
void print_test_header(const char* test_name) {
    std::cout << "\n=== " << test_name << " ===" << std::endl;
}

void print_test_result(const char* test_name, bool passed) {
    std::cout << (passed ? "✓" : "✗") << " " << test_name << std::endl;
}

// Test configuration manager initialization
bool test_config_manager_initialization() {
    print_test_header("Configuration Manager Initialization");
    
    // Setup storage system
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    
    // Initialize storage first
    g_message_bus.init();
    if (!storage_manager.init()) {
        std::cout << "ERROR: Failed to initialize storage manager" << std::endl;
        return false;
    }
    
    // Initialize config manager
    ConfigManager config_manager(&storage_manager);
    
    // Test initialization
    bool init_result = config_manager.initialize();
    print_test_result("Config manager initialization", init_result);
    
    if (!init_result) {
        return false;
    }
    
    // Verify default configuration is loaded
    const ECUConfiguration& config = config_manager.getConfig();
    print_test_result("ECU type is transmission", config.ecu_type == ECU_TRANSMISSION);
    print_test_result("ECU name is correct", strcmp(config.ecu_name, "Backslider Transmission") == 0);
    print_test_result("Configuration is loaded", config_manager.isConfigurationLoaded());
    
    return init_result && config.ecu_type == ECU_TRANSMISSION && 
           strcmp(config.ecu_name, "Backslider Transmission") == 0;
}

// Test configuration validation
bool test_config_validation() {
    print_test_header("Configuration Validation");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager(&storage_manager);
    config_manager.initialize();
    
    // Test validation of loaded configuration
    bool validation_result = config_manager.validateConfiguration();
    print_test_result("Default configuration validation", validation_result);
    
    // Test pin access methods
    uint8_t can_tx_pin = config_manager.getCANTXPin();
    uint8_t can_rx_pin = config_manager.getCANRXPin();
    print_test_result("CAN TX pin access", can_tx_pin == 1);
    print_test_result("CAN RX pin access", can_rx_pin == 0);
    
    // Test I2C configuration access
    const I2CConfiguration& i2c_config = config_manager.getI2CConfig();
    print_test_result("I2C bus frequency", i2c_config.bus_frequency == 400000);
    
    const I2CDeviceConfig& gpio_config = config_manager.getGPIOExpanderConfig();
    print_test_result("GPIO expander address", gpio_config.address == 0x20);
    print_test_result("GPIO expander enabled", gpio_config.enabled == true);
    
    const I2CDeviceConfig& adc_config = config_manager.getADCConfig();
    print_test_result("ADC address", adc_config.address == 0x48);
    print_test_result("ADC enabled", adc_config.enabled == true);
    
    return validation_result && can_tx_pin == 1 && can_rx_pin == 0 &&
           i2c_config.bus_frequency == 400000 && gpio_config.address == 0x20 &&
           adc_config.address == 0x48;
}

// Test runtime configuration updates
bool test_runtime_updates() {
    print_test_header("Runtime Configuration Updates");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager(&storage_manager);
    config_manager.initialize();
    
    // Test ECU name update
    const char* new_name = "Test Transmission";
    bool name_update = config_manager.updateECUName(new_name);
    print_test_result("ECU name update", name_update);
    
    if (name_update) {
        const char* updated_name = config_manager.getECUName();
        print_test_result("ECU name updated correctly", strcmp(updated_name, new_name) == 0);
    }
    
    // Test serial number update
    uint32_t new_serial = 0x12345678;
    bool serial_update = config_manager.updateSerialNumber(new_serial);
    print_test_result("Serial number update", serial_update);
    
    if (serial_update) {
        uint32_t updated_serial = config_manager.getSerialNumber();
        print_test_result("Serial number updated correctly", updated_serial == new_serial);
    }
    
    // Test boot timeout update
    uint32_t new_timeout = 3000;
    bool timeout_update = config_manager.updateBootTimeout(new_timeout);
    print_test_result("Boot timeout update", timeout_update);
    
    if (timeout_update) {
        uint32_t updated_timeout = config_manager.getBootTimeout();
        print_test_result("Boot timeout updated correctly", updated_timeout == new_timeout);
    }
    
    // Test invalid updates
    bool invalid_timeout = config_manager.updateBootTimeout(50000);  // Too large
    print_test_result("Invalid boot timeout rejected", !invalid_timeout);
    
    return name_update && serial_update && timeout_update && !invalid_timeout;
}

// Test configuration persistence
bool test_config_persistence() {
    print_test_header("Configuration Persistence");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    // First config manager - make changes
    {
        ConfigManager config_manager(&storage_manager);
        config_manager.initialize();
        
        // Make some changes
        config_manager.updateECUName("Persistent Test");
        config_manager.updateSerialNumber(0xABCDEF01);
        config_manager.updateBootTimeout(4000);
        
        // Verify changes
        print_test_result("Changes applied", strcmp(config_manager.getECUName(), "Persistent Test") == 0);
    }
    
    // Second config manager - should load persisted changes
    {
        ConfigManager config_manager2(&storage_manager);
        config_manager2.initialize();
        
        // Check if changes persisted
        bool name_persisted = strcmp(config_manager2.getECUName(), "Persistent Test") == 0;
        bool serial_persisted = config_manager2.getSerialNumber() == 0xABCDEF01;
        bool timeout_persisted = config_manager2.getBootTimeout() == 4000;
        
        print_test_result("ECU name persisted", name_persisted);
        print_test_result("Serial number persisted", serial_persisted);
        print_test_result("Boot timeout persisted", timeout_persisted);
        
        return name_persisted && serial_persisted && timeout_persisted;
    }
}

// Test error handling
bool test_error_handling() {
    print_test_header("Error Handling");
    
    // Test with null storage manager
    ConfigManager config_manager(nullptr);
    bool init_with_null = config_manager.initialize();
    print_test_result("Null storage manager handled", !init_with_null);
    
    // Test with valid storage manager
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager2(&storage_manager);
    bool init_success = config_manager2.initialize();
    print_test_result("Valid initialization", init_success);
    
    // Test invalid ECU name (too long)
    char long_name[50];
    memset(long_name, 'A', sizeof(long_name));
    long_name[49] = '\0';  // Null terminate
    
    bool invalid_name = config_manager2.updateECUName(long_name);
    print_test_result("Long ECU name rejected", !invalid_name);
    
    // Test invalid status report interval
    bool invalid_interval = config_manager2.updateStatusReportInterval(50);  // Too small
    print_test_result("Invalid status interval rejected", !invalid_interval);
    
    return !init_with_null && init_success && !invalid_name && !invalid_interval;
}

// Test factory reset
bool test_factory_reset() {
    print_test_header("Factory Reset");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager(&storage_manager);
    config_manager.initialize();
    
    // Make some changes
    config_manager.updateECUName("Modified Config");
    config_manager.updateSerialNumber(0xDEADBEEF);
    
    // Verify changes
    bool changes_applied = strcmp(config_manager.getECUName(), "Modified Config") == 0;
    print_test_result("Changes applied before reset", changes_applied);
    
    // Perform factory reset
    bool reset_success = config_manager.resetToDefaults();
    print_test_result("Factory reset successful", reset_success);
    
    // Verify reset to defaults
    bool name_reset = strcmp(config_manager.getECUName(), "Backslider Transmission") == 0;
    bool serial_reset = config_manager.getSerialNumber() == 0x54524E53;  // "TRNS"
    
    print_test_result("ECU name reset to default", name_reset);
    print_test_result("Serial number reset to default", serial_reset);
    
    return changes_applied && reset_success && name_reset && serial_reset;
}

// Test transmission-specific settings
bool test_transmission_settings() {
    print_test_header("Transmission-Specific Settings");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager(&storage_manager);
    config_manager.initialize();
    
    // Test transmission settings access
    bool shift_monitoring = config_manager.isShiftMonitoringEnabled();
    bool pressure_control = config_manager.isPressureControlEnabled();
    bool temp_monitoring = config_manager.isTemperatureMonitoringEnabled();
    uint32_t shift_debounce = config_manager.getShiftDebounceMs();
    
    print_test_result("Shift monitoring enabled", shift_monitoring);
    print_test_result("Pressure control enabled", pressure_control);
    print_test_result("Temperature monitoring enabled", temp_monitoring);
    print_test_result("Shift debounce correct", shift_debounce == 50);
    
    // Test watchdog and debug settings
    bool watchdog_enabled = config_manager.isWatchdogEnabled();
    bool debug_enabled = config_manager.isDebugOutputEnabled();
    
    print_test_result("Watchdog enabled", watchdog_enabled);
    print_test_result("Debug output enabled", debug_enabled);
    
    return shift_monitoring && pressure_control && temp_monitoring && 
           shift_debounce == 50 && watchdog_enabled && debug_enabled;
}

// Test SPI configuration access
bool test_spi_configuration() {
    print_test_header("SPI Configuration");
    
    SPIFlashStorageBackend storage_backend;
    StorageManager storage_manager(&storage_backend);
    g_message_bus.init();
    storage_manager.init();
    
    ConfigManager config_manager(&storage_manager);
    config_manager.initialize();
    
    // Test SPI configuration access
    const SPIConfiguration& spi_config = config_manager.getSPIConfig();
    print_test_result("SPI MOSI pin", spi_config.mosi_pin == 11);
    print_test_result("SPI MISO pin", spi_config.miso_pin == 12);
    print_test_result("SPI SCK pin", spi_config.sck_pin == 13);
    
    // Test external SPI flash configuration
    const SPIDeviceConfig& qspi_config = config_manager.getQSPIFlashConfig();
    print_test_result("External flash CS pin", qspi_config.cs_pin == 10);
    print_test_result("External flash frequency", qspi_config.frequency == 25000000);
    print_test_result("External flash enabled", qspi_config.enabled == true);
    
    return spi_config.mosi_pin == 11 && spi_config.miso_pin == 12 &&
           spi_config.sck_pin == 13 && qspi_config.cs_pin == 10 &&
           qspi_config.frequency == 25000000 && qspi_config.enabled;
}

// Main test runner
int main() {
    std::cout << "Starting Configuration Manager Tests..." << std::endl;
    
    int tests_passed = 0;
    int total_tests = 0;
    
    // Run all tests
    total_tests++; if (test_config_manager_initialization()) tests_passed++;
    total_tests++; if (test_config_validation()) tests_passed++;
    total_tests++; if (test_runtime_updates()) tests_passed++;
    total_tests++; if (test_config_persistence()) tests_passed++;
    total_tests++; if (test_error_handling()) tests_passed++;
    total_tests++; if (test_factory_reset()) tests_passed++;
    total_tests++; if (test_transmission_settings()) tests_passed++;
    total_tests++; if (test_spi_configuration()) tests_passed++;
    
    // Print summary
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Tests passed: " << tests_passed << "/" << total_tests << std::endl;
    
    if (tests_passed == total_tests) {
        std::cout << "🎉 All Configuration Manager tests passed!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ Some tests failed!" << std::endl;
        return 1;
    }
} 