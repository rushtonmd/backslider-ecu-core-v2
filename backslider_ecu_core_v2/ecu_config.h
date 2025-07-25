#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include <stdint.h>
#include "external_serial.h"
#include "external_canbus.h"

// =============================================================================
// ECU TYPE DEFINITIONS
// =============================================================================
enum ECUType : uint8_t {
    ECU_HUB = 0,           // Main ECU hub
    ECU_TRANSMISSION = 1,   // Transmission controller
    ECU_GPIO = 2,          // GPIO expansion module
    ECU_WEB_SERVER = 3,    // Web interface module
    ECU_IGNITION = 4,      // Primary ignition controller
    ECU_FUEL = 5           // Fuel injection controller
};

// =============================================================================
// CRITICAL PIN ASSIGNMENTS
// =============================================================================
struct CriticalPinConfig {
    // I2C Bus (Wire - primary I2C)
    uint8_t i2c_sda_pin;
    uint8_t i2c_scl_pin;
    
    // External CAN Bus
    uint8_t can_tx_pin;
    uint8_t can_rx_pin;
    
    // Status LEDs
    uint8_t status_led_pin;
    uint8_t error_led_pin;
    uint8_t activity_led_pin;
};

// =============================================================================
// I2C DEVICE CONFIGURATIONS
// =============================================================================
struct I2CDeviceConfig {
    uint8_t address;
    uint32_t frequency;
    bool enabled;
    uint8_t timeout_ms;
    uint8_t device_number;
};

struct I2CConfiguration {
    uint32_t bus_frequency;        // 100000, 400000, 1000000
    bool internal_pullups;
    uint8_t timeout_ms;
    uint8_t number_of_interfaces;
    
    // Device configurations
    I2CDeviceConfig gpio_expander;  // MCP23017
    I2CDeviceConfig adc;           // ADS1115
    I2CDeviceConfig rtc;           // DS3231 (future)
    I2CDeviceConfig eeprom;        // 24LC256 (future)
};

// =============================================================================
// SPI DEVICE CONFIGURATIONS
// =============================================================================
struct SPIDeviceConfig {
    uint8_t cs_pin;
    uint32_t frequency;
    uint8_t mode;              // 0, 1, 2, 3
    uint8_t bit_order;         // MSBFIRST, LSBFIRST
    bool enabled;
};

struct SPIConfiguration {
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    
    // Device configurations
    SPIDeviceConfig qspi_flash;     // W25Q128
    SPIDeviceConfig sd_card;        // Future
    SPIDeviceConfig can_controller; // Future MCP2515
    SPIDeviceConfig pwm_controller; // Future TLC59711
};

// =============================================================================
// COMPLETE ECU CONFIGURATION
// =============================================================================
struct ECUConfiguration {
    ECUType ecu_type;
    char ecu_name[32];
    char firmware_version[16];
    uint32_t serial_number;
    
    CriticalPinConfig pins;
    I2CConfiguration i2c;
    SPIConfiguration spi;
    external_serial_config_t external_serial;
    external_canbus_config_t external_canbus;
    
    // Boot behavior
    uint32_t boot_timeout_ms;
    bool enable_watchdog;
    bool enable_debug_output;
    uint32_t status_report_interval_ms;
    
    // Transmission-specific settings
    struct {
        bool enable_shift_monitoring;
        bool enable_pressure_control;
        bool enable_temperature_monitoring;
        uint32_t shift_debounce_ms;
    } transmission;
};

// =============================================================================
// CONFIGURATION PRESETS
// =============================================================================
// Default configuration for Transmission Controller
extern const ECUConfiguration ECU_TRANSMISSION_CONFIG;

// Future configurations
// extern const ECUConfiguration ECU_HUB_CONFIG;
// extern const ECUConfiguration ECU_GPIO_CONFIG;

#endif 