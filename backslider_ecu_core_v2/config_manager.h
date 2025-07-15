#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "ecu_config.h"
#include "storage_manager.h"

class ConfigManager {
private:
    ECUConfiguration current_config;
    StorageManager* storage;
    bool config_loaded;
    
    // Configuration storage keys
    static const char* CONFIG_KEY_ECU_TYPE;
    static const char* CONFIG_KEY_ECU_NAME;
    static const char* CONFIG_KEY_SERIAL_NUMBER;
    static const char* CONFIG_KEY_FIRMWARE_VERSION;
    static const char* CONFIG_KEY_BOOT_TIMEOUT;
    
    // Helper methods
    bool loadDefaultConfiguration();
    bool saveConfigurationToStorage();
    bool loadConfigurationFromStorage();
    uint16_t calculateConfigChecksum();
    
public:
    ConfigManager(StorageManager* storage_mgr);
    
    // Initialization - call this FIRST during boot
    bool initialize();
    
    // Configuration access
    const ECUConfiguration& getConfig() const { return current_config; }
    ECUType getECUType() const { return current_config.ecu_type; }
    const char* getECUName() const { return current_config.ecu_name; }
    const char* getFirmwareVersion() const { return current_config.firmware_version; }
    uint32_t getSerialNumber() const { return current_config.serial_number; }
    
    // Pin access helpers
    uint8_t getI2CSDAPin() const { return current_config.pins.i2c_sda_pin; }
    uint8_t getI2CSCLPin() const { return current_config.pins.i2c_scl_pin; }
    uint8_t getCANTXPin() const { return current_config.pins.can_tx_pin; }
    uint8_t getCANRXPin() const { return current_config.pins.can_rx_pin; }
    uint8_t getSerialTXPin() const { return current_config.pins.ext_serial_tx_pin; }
    uint8_t getSerialRXPin() const { return current_config.pins.ext_serial_rx_pin; }
    uint8_t getStatusLEDPin() const { return current_config.pins.status_led_pin; }
    uint8_t getErrorLEDPin() const { return current_config.pins.error_led_pin; }
    uint8_t getActivityLEDPin() const { return current_config.pins.activity_led_pin; }
    
    // I2C configuration access
    const I2CConfiguration& getI2CConfig() const { return current_config.i2c; }
    uint32_t getI2CBusFrequency() const { return current_config.i2c.bus_frequency; }
    const I2CDeviceConfig& getGPIOExpanderConfig() const { return current_config.i2c.gpio_expander; }
    const I2CDeviceConfig& getADCConfig() const { return current_config.i2c.adc; }
    
    // SPI configuration access
    const SPIConfiguration& getSPIConfig() const { return current_config.spi; }
    const SPIDeviceConfig& getQSPIFlashConfig() const { return current_config.spi.qspi_flash; }
    
    // Boot behavior access
    uint32_t getBootTimeout() const { return current_config.boot_timeout_ms; }
    bool isWatchdogEnabled() const { return current_config.enable_watchdog; }
    bool isDebugOutputEnabled() const { return current_config.enable_debug_output; }
    uint32_t getStatusReportInterval() const { return current_config.status_report_interval_ms; }
    
    // Transmission settings access
    bool isShiftMonitoringEnabled() const { return current_config.transmission.enable_shift_monitoring; }
    bool isPressureControlEnabled() const { return current_config.transmission.enable_pressure_control; }
    bool isTemperatureMonitoringEnabled() const { return current_config.transmission.enable_temperature_monitoring; }
    uint32_t getShiftDebounceMs() const { return current_config.transmission.shift_debounce_ms; }
    
    // Runtime configuration updates
    bool updateECUType(ECUType new_type);
    bool updateECUName(const char* new_name);
    bool updateSerialNumber(uint32_t new_serial);
    bool updateBootTimeout(uint32_t timeout_ms);
    bool updateStatusReportInterval(uint32_t interval_ms);
    
    // Configuration validation and diagnostics
    bool validateConfiguration();
    void printConfiguration();
    void printConfigurationSummary();
    
    // Factory reset
    bool resetToDefaults();
    
    // Status
    bool isConfigurationLoaded() const { return config_loaded; }
};

#endif 