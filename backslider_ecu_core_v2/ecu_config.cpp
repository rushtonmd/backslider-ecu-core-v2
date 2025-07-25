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
    
    // TEENSY 4.1 PIN ALLOCATION SUMMARY (55 total pins: 0-54)
    // =====================================================
    // Used Pins: 0, 1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 18, 19, 24, 25 (15 pins)
    // 
    // Available Pins: 7, 8, 9, 14, 15, 16, 17, 20, 21, 22, 23, 26-54 (many pins)
    // 
    // Special Function Pins Available:
    // - Pin 7, 8, 9, 14: Built-in QSPI flash pins (available for other use)
    // - Pin 15, 16, 17: Analog inputs (A1, A2, A3)
    // - Pin 20, 21: Additional I2C (Wire1) - SDA1, SCL1
    // - Pin 22, 23: Available for general use
    // - Pin 53, 54, 55: LED indicators (non-conflicting)
    //
    // Note: Pin 13 is built-in LED and SPI SCK - conflicts with SPI usage
    //       Hardware serial ports (Serial1, Serial2) use predefined pins
    
    .pins = {
        // External CAN Bus (built into Teensy 4.1)
        .can_tx_pin = 0,           // CAN1 TX (built-in)
        .can_rx_pin = 1,           // CAN1 RX (built-in)
        
        // Status LEDs - Disabled to avoid pin conflicts
        .status_led_pin = 0xFF,    // Disabled
        .error_led_pin = 0xFF,     // Disabled
        .activity_led_pin = 0xFF   // Disabled
    },
    
    .i2c = {
        .bus_frequency = 400000,   // 400kHz - good balance of speed and reliability
        .internal_pullups = true,  // Use internal pullups
        .timeout_ms = 100,         // 100ms timeout for I2C operations
        .number_of_interfaces = 1, // 1 interface for now
        
        // MCP23017 I2C GPIO Expander - 16 GPIO pins
        // Address range: 0x20-0x27 (base 0x20 + D0/D1/D2 jumpers)
        .gpio_expander = {
            .address = 0x20,       // Default address (A0=A1=A2=LOW)
            .frequency = 400000,   // 400kHz
            .enabled = true,        // Enable GPIO expander
            .timeout_ms = 50,       // 50ms timeout
            .device_number = 1      // 0 for now
        },
        
        // ADS1115 16-bit ADC - 4 channel with PGA
        // Address range: 0x48-0x4B (selectable with jumpers)
        .adc = {
            .address = 0x48,       // Default address (ADDR=GND)
            .frequency = 400000,   // 400kHz
            .enabled = true,        // Enable ADC
            .timeout_ms = 50,       // 50ms timeout
            .device_number = 1      // 0 for now
        },
        
        // Future I2C devices
        // .rtc = {
        //     .address = 0x68,       // DS3231 RTC
        //     .frequency = 100000,   // 100kHz (slower for RTC)
        //     .enabled = false,      // Disabled for now
        //     .timeout_ms = 100
        // },
        
        // .eeprom = {
        //     .address = 0x50,       // 24LC256 EEPROM
        //     .frequency = 100000,   // 100kHz (slower for EEPROM)
        //     .enabled = false,      // Disabled for now
        //     .timeout_ms = 200
        // }
    },
    
    .spi = {
        // Standard SPI pins for Teensy 4.0
        .mosi_pin = 11,
        .miso_pin = 12,
        .sck_pin = 13,
        
        // External SPI Flash (W25Q128) - 128 MBit / 16 MByte
        // Note: Uses regular SPI bus, not built-in QSPI
        .qspi_flash = {
            .cs_pin = 10,          // SPI CS pin (available pin)
            .frequency = 25000000, // 25MHz - conservative for external flash
            .mode = 0,             // SPI Mode 0
            .bit_order = MSBFIRST, // MSB first
            .enabled = true        // Enable external flash
        },
        
        // Future SPI devices
        // .sd_card = {
        //     .cs_pin = 10,          // Future SD card
        //     .frequency = 25000000, // 25MHz
        //     .mode = 0,
        //     .bit_order = MSBFIRST,
        //     .enabled = false       // Disabled for now
        // },
        
        // .can_controller = {
        //     .cs_pin = 9,           // Future MCP2515 CAN controller
        //     .frequency = 8000000,  // 8MHz
        //     .mode = 0,
        //     .bit_order = MSBFIRST,
        //     .enabled = false       // Disabled for now
        // },
        
        // .pwm_controller = {
        //     .cs_pin = 8,           // Future TLC59711 PWM controller
        //     .frequency = 10000000, // 10MHz
        //     .mode = 0,
        //     .bit_order = MSBFIRST,
        //     .enabled = false       // Disabled for now
        // }
    },
    
    // External Serial Communication
    .external_serial = {
        //USB Serial - High-speed communication for tuning software
        .usb = {
            .enabled = true,
            .baud_rate = 2000000,  // 2 Mbps for fast parameter access
            .tx_enabled = true,
            .rx_enabled = true
        },
        
        // Serial1 - Dashboard communication
        .serial1 = {
            .enabled = false,
            .baud_rate = 1000000,  // 1 Mbps for dashboard data
            .tx_enabled = true,
            .rx_enabled = true
        },
        
        // Serial2 - Reserved for future use (datalogger, etc.)
        .serial2 = {
            .enabled = false,
            .baud_rate = 115200,   // Standard rate for general use
            .tx_enabled = true,
            .rx_enabled = true
        }
    },
    
    // External CAN Bus Communication
    .external_canbus = {
        .enabled = true,             // Completely disable external CAN bus
        .baudrate = 500000,           // 500 kbps - standard automotive CAN
        .enable_obdii = true,        // Disable OBD-II for now
        .enable_custom_messages = true, // Disable custom messages for now
        .can_bus_number = 1,          // CAN1
        .cache_default_max_age_ms = 1000 // 1 second cache timeout
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