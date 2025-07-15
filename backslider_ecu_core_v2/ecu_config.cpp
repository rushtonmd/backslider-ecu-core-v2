#include "ecu_config.h"

#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#ifndef LSBFIRST
#define LSBFIRST 0
#endif

// =============================================================================
// TRANSMISSION CONTROLLER CONFIGURATION
// =============================================================================
const ECUConfiguration ECU_TRANSMISSION_CONFIG = {
    .ecu_type = ECU_TRANSMISSION,
    .ecu_name = "Backslider Transmission",
    .firmware_version = "2.0.0",
    .serial_number = 0x54524E53,  // "TRNS" in hex
    
    // TEENSY 4.0 PIN ALLOCATION SUMMARY (24 total pins: 0-23)
    // =====================================================
    // Used Pins: 0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 18, 19, 22, 23 (17 pins)
    // 
    // Available Pins: 2, 3, 15, 16, 17, 20, 21 (7 pins)
    // 
    // Special Function Pins Available:
    // - Pin 2, 3: Interrupt-capable pins
    // - Pin 15, 16, 17: Analog inputs (A1, A2, A3)
    // - Pin 20, 21: Additional I2C (Wire1) - SDA1, SCL1
    //
    // Note: All remaining pins (2, 3, 15-17, 20, 21) can be used for general I/O
    
    .pins = {
        // QSPI Flash Memory (W25Q128) - Standard Teensy 4.0 QSPI pins
        .qspi_cs_pin = 6,
        .qspi_sck_pin = 14,
        .qspi_io0_pin = 8,
        .qspi_io1_pin = 7,
        .qspi_io2_pin = 9,
        .qspi_io3_pin = 10,
        
        // I2C Bus (Wire - primary I2C) - Standard Teensy 4.0 I2C pins
        .i2c_sda_pin = 18,
        .i2c_scl_pin = 19,
        
        // External Serial - User specified pins
        .ext_serial_tx_pin = 22,
        .ext_serial_rx_pin = 23,
        .ext_serial_rts_pin = 4,   // Optional flow control
        .ext_serial_cts_pin = 5,   // Optional flow control
        
        // External CAN Bus - User specified pins
        .can_tx_pin = 0,
        .can_rx_pin = 1,
        
        // Status LEDs
        .status_led_pin = 13,      // Built-in LED
        .error_led_pin = 12,       // External red LED
        .activity_led_pin = 11     // External green LED
    },
    
    .i2c = {
        .bus_frequency = 400000,   // 400kHz - good balance of speed and reliability
        .internal_pullups = true,  // Use internal pullups
        .timeout_ms = 100,         // 100ms timeout for I2C operations
        
        // MCP23017 I2C GPIO Expander - 16 GPIO pins
        // Address range: 0x20-0x27 (base 0x20 + D0/D1/D2 jumpers)
        .gpio_expander = {
            .address = 0x20,       // Default address (A0=A1=A2=LOW)
            .frequency = 400000,   // 400kHz
            .enabled = true,       // Enable GPIO expander
            .timeout_ms = 50       // 50ms timeout
        },
        
        // ADS1115 16-bit ADC - 4 channel with PGA
        // Address range: 0x48-0x4B (selectable with jumpers)
        .adc = {
            .address = 0x48,       // Default address (ADDR=GND)
            .frequency = 400000,   // 400kHz
            .enabled = true,       // Enable ADC
            .timeout_ms = 50       // 50ms timeout
        },
        
        // Future I2C devices
        .rtc = {
            .address = 0x68,       // DS3231 RTC
            .frequency = 100000,   // 100kHz (slower for RTC)
            .enabled = false,      // Disabled for now
            .timeout_ms = 100
        },
        
        .eeprom = {
            .address = 0x50,       // 24LC256 EEPROM
            .frequency = 100000,   // 100kHz (slower for EEPROM)
            .enabled = false,      // Disabled for now
            .timeout_ms = 200
        }
    },
    
    .spi = {
        // Standard SPI pins for Teensy 4.0
        .mosi_pin = 11,
        .miso_pin = 12,
        .sck_pin = 13,
        
        // QSPI Flash (W25Q128) - 128 MBit / 16 MByte
        .qspi_flash = {
            .cs_pin = 6,           // Same as qspi_cs_pin
            .frequency = 50000000, // 50MHz - conservative speed
            .mode = 0,             // SPI Mode 0
            .bit_order = MSBFIRST, // MSB first
            .enabled = true        // Enable QSPI flash
        },
        
        // Future SPI devices
        .sd_card = {
            .cs_pin = 10,          // Future SD card
            .frequency = 25000000, // 25MHz
            .mode = 0,
            .bit_order = MSBFIRST,
            .enabled = false       // Disabled for now
        },
        
        .can_controller = {
            .cs_pin = 9,           // Future MCP2515 CAN controller
            .frequency = 8000000,  // 8MHz
            .mode = 0,
            .bit_order = MSBFIRST,
            .enabled = false       // Disabled for now
        },
        
        .pwm_controller = {
            .cs_pin = 8,           // Future TLC59711 PWM controller
            .frequency = 10000000, // 10MHz
            .mode = 0,
            .bit_order = MSBFIRST,
            .enabled = false       // Disabled for now
        }
    },
    
    // Boot behavior
    .boot_timeout_ms = 5000,       // 5 second boot timeout
    .enable_watchdog = true,       // Enable watchdog timer
    .enable_debug_output = true,   // Enable debug output
    .status_report_interval_ms = 1000, // 1 second status reports
    
    // Transmission-specific settings
    .transmission = {
        .enable_shift_monitoring = true,   // Monitor transmission shifts
        .enable_pressure_control = true,   // Control line pressure
        .enable_temperature_monitoring = true, // Monitor fluid temperature
        .shift_debounce_ms = 50            // 50ms shift debounce
    }
}; 