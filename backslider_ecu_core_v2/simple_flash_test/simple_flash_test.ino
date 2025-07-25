/*
 * Simple W25Q128 Flash ID Test
 * 
 * Minimal test to read W25Q128 flash ID directly
 */

#include <SPI.h>

const uint8_t CS_PIN = 10;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("=== Simple W25Q128 Flash ID Test ===");
    
    // Initialize SPI
    Serial.println("Initializing SPI...");
    SPI.begin();
    
    // Initialize CS pin
    Serial.println("Setting up CS pin...");
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    
    // Read flash ID directly
    Serial.println("Reading flash ID...");
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x9F); // READ JEDEC ID command
    uint8_t manufacturer = SPI.transfer(0);
    uint8_t memory_type = SPI.transfer(0);
    uint8_t capacity = SPI.transfer(0);
    digitalWrite(CS_PIN, HIGH);
    
    uint32_t flash_id = (manufacturer << 16) | (memory_type << 8) | capacity;
    
    Serial.print("Flash ID: 0x");
    Serial.println(flash_id, HEX);
    Serial.print("Manufacturer: 0x");
    Serial.println(manufacturer, HEX);
    Serial.print("Memory Type: 0x");
    Serial.println(memory_type, HEX);
    Serial.print("Capacity: 0x");
    Serial.println(capacity, HEX);
    
    if (flash_id == 0xEF4018) {
        Serial.println("✓ W25Q128 detected correctly!");
    } else if (flash_id == 0xFFFFFF || flash_id == 0x000000) {
        Serial.println("✗ No device detected");
    } else {
        Serial.println("✗ Unknown device detected");
    }
    
    Serial.println("=== Test Complete ===");
}

void loop() {
    delay(1000);
} 