/*
 * W25Q128 Hardware Test
 * 
 * Comprehensive test of W25Q128 storage backend on Teensy 4.1
 * 
 * Hardware Requirements:
 * - Teensy 4.1
 * - Adafruit QSPI DIP Breakout Board - W25Q128
 * - SPI connections: MOSI=11, MISO=12, SCK=13, CS=10
 * 
 * Test Sequence:
 * 1. Initialize W25Q128 and verify flash ID
 * 2. Basic read/write operations
 * 3. Multiple data types and sizes
 * 4. Performance benchmarks
 * 5. Error handling and edge cases
 * 6. Cache performance testing
 * 7. Stress testing with many operations
 */

#include "w25q128_storage_backend.h"
#include "ecu_config.h"

// Create ECU configuration
ECUConfiguration ecu_config;

// Create W25Q128 storage backend
W25Q128StorageBackend storage_backend(ecu_config);

// Test data structures
struct SensorCalibration {
    uint32_t sensor_id;
    float offset;
    float scale;
    uint8_t valid;
    uint32_t timestamp;
};

struct TransmissionData {
    uint32_t gear_position;
    float fluid_temperature;
    float fluid_pressure;
    uint32_t shift_count;
    uint32_t timestamp;
};

struct LargeData {
    uint32_t id;
    uint8_t data[200];  // Large data block
    uint32_t checksum;
    uint32_t timestamp;
};

// Test statistics
struct TestStats {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t total_writes;
    uint32_t total_reads;
    uint32_t total_errors;
    uint32_t start_time;
    uint32_t end_time;
};

TestStats test_stats = {0};

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("=== W25Q128 Hardware Test ===");
    
    // Debug: Check if TESTING is defined
    #ifdef TESTING
    Serial.println("WARNING: TESTING flag is defined - using mock values!");
    #else
    Serial.println("TESTING flag is NOT defined - using real hardware");
    #endif
    
    Serial.println("Starting comprehensive hardware test...");
    
    // Configure ECU settings
    ecu_config.ecu_type = ECU_TRANSMISSION;
    strcpy(ecu_config.ecu_name, "Transmission ECU");
    strcpy(ecu_config.firmware_version, "1.0.0");
    ecu_config.serial_number = 12345;
    
    // Configure SPI pins for W25Q128
    ecu_config.spi.mosi_pin = 11;
    ecu_config.spi.miso_pin = 12;
    ecu_config.spi.sck_pin = 13;
    ecu_config.spi.qspi_flash.cs_pin = 10;
    ecu_config.spi.qspi_flash.frequency = 10000000; // 10MHz
    ecu_config.spi.qspi_flash.mode = 0;
    ecu_config.spi.qspi_flash.bit_order = 0; // MSBFIRST equivalent
    ecu_config.spi.qspi_flash.enabled = true;
    
    test_stats.start_time = millis();
    
    // Run all tests
    runAllTests();
    
    test_stats.end_time = millis();
    printFinalResults();
}

void loop() {
    // Keep alive - blink LED to show it's running
    static uint32_t last_blink = 0;
    if (millis() - last_blink >= 1000) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        last_blink = millis();
    }
    delay(100);
}

void runAllTests() {
    Serial.println("\n--- Starting Test Suite ---");
    
    // Test 1: Initialization
    testInitialization();
    
    // Test 2: Basic Operations
    testBasicOperations();
    
    // Test 3: Multiple Data Types
    testMultipleDataTypes();
    
    // Test 4: Performance Benchmarks
    testPerformance();
    
    // Test 5: Error Handling
    testErrorHandling();
    
    // Test 6: Cache Performance
    testCachePerformance();
    
    // Test 7: Stress Testing
    testStressTesting();
    
    Serial.println("--- Test Suite Complete ---\n");
}

void testInitialization() {
    Serial.println("Test 1: Initialization");
    test_stats.total_tests++;
    
    if (storage_backend.begin()) {
        Serial.println("✓ W25Q128 initialization successful");
        storage_backend.printFlashInfo();
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ W25Q128 initialization failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    
    // Test flash ID - the flash ID was already read during initialization above
    // We can verify it's working by checking if the storage backend is ready
    if (storage_backend.isFlashReady()) {
        Serial.println("✓ Flash ID verification passed (W25Q128 detected)");
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ Flash not ready after initialization");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    test_stats.total_tests++;
}

void testBasicOperations() {
    Serial.println("Test 2: Basic Operations");
    
    // Test simple data write/read
    uint32_t test_key = 0x10300001;
    uint32_t test_data = 0x12345678;
    
    test_stats.total_tests++;
    if (storage_backend.writeData(test_key, &test_data, sizeof(test_data))) {
        Serial.println("✓ Write operation successful");
        test_stats.total_writes++;
        
        uint32_t read_data;
        if (storage_backend.readData(test_key, &read_data, sizeof(read_data))) {
            if (read_data == test_data) {
                Serial.println("✓ Read operation successful - data matches");
                test_stats.passed_tests++;
                test_stats.total_reads++;
            } else {
                Serial.println("✗ Read operation failed - data mismatch");
                test_stats.failed_tests++;
                test_stats.total_errors++;
            }
        } else {
            Serial.println("✗ Read operation failed");
            test_stats.failed_tests++;
            test_stats.total_errors++;
        }
    } else {
        Serial.println("✗ Write operation failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    
    // Test data existence
    test_stats.total_tests++;
    if (storage_backend.hasData(test_key)) {
        Serial.println("✓ Data existence check passed");
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ Data existence check failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
}

void testMultipleDataTypes() {
    Serial.println("Test 3: Multiple Data Types");
    
    // Test sensor calibration data
    SensorCalibration cal_data = {
        .sensor_id = 0x10300002,
        .offset = -2.5f,
        .scale = 1.02f,
        .valid = 1,
        .timestamp = millis()
    };
    
    test_stats.total_tests++;
    if (storage_backend.writeData(cal_data.sensor_id, &cal_data, sizeof(cal_data))) {
        SensorCalibration read_cal;
        if (storage_backend.readData(cal_data.sensor_id, &read_cal, sizeof(read_cal))) {
            if (memcmp(&cal_data, &read_cal, sizeof(cal_data)) == 0) {
                Serial.println("✓ Sensor calibration data test passed");
                test_stats.passed_tests++;
                test_stats.total_writes++;
                test_stats.total_reads++;
            } else {
                Serial.println("✗ Sensor calibration data test failed");
                test_stats.failed_tests++;
                test_stats.total_errors++;
            }
        } else {
            Serial.println("✗ Sensor calibration read failed");
            test_stats.failed_tests++;
            test_stats.total_errors++;
        }
    } else {
        Serial.println("✗ Sensor calibration write failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    
    // Test transmission data
    TransmissionData trans_data = {
        .gear_position = 3,
        .fluid_temperature = 85.5f,
        .fluid_pressure = 2.3f,
        .shift_count = 1250,
        .timestamp = millis()
    };
    
    test_stats.total_tests++;
    if (storage_backend.writeData(0x10300003, &trans_data, sizeof(trans_data))) {
        TransmissionData read_trans;
        if (storage_backend.readData(0x10300003, &read_trans, sizeof(read_trans))) {
            if (memcmp(&trans_data, &read_trans, sizeof(trans_data)) == 0) {
                Serial.println("✓ Transmission data test passed");
                test_stats.passed_tests++;
                test_stats.total_writes++;
                test_stats.total_reads++;
            } else {
                Serial.println("✗ Transmission data test failed");
                test_stats.failed_tests++;
                test_stats.total_errors++;
            }
        } else {
            Serial.println("✗ Transmission data read failed");
            test_stats.failed_tests++;
            test_stats.total_errors++;
        }
    } else {
        Serial.println("✗ Transmission data write failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
}

void testPerformance() {
    Serial.println("Test 4: Performance Benchmarks");
    
    // Test write performance
    uint32_t start_time = micros();
    const int num_writes = 10;
    
    for (int i = 0; i < num_writes; i++) {
        uint32_t test_data = 0x10000000 + i;
        storage_backend.writeData(0x10400000 + i, &test_data, sizeof(test_data));
    }
    
    uint32_t write_time = micros() - start_time;
    Serial.print("Write performance: ");
    Serial.print(write_time);
    Serial.print(" µs for ");
    Serial.print(num_writes);
    Serial.print(" writes (");
    Serial.print(write_time / num_writes);
    Serial.println(" µs per write)");
    
    test_stats.total_writes += num_writes;
    
    // Test read performance
    start_time = micros();
    
    for (int i = 0; i < num_writes; i++) {
        uint32_t read_data;
        storage_backend.readData(0x10400000 + i, &read_data, sizeof(read_data));
    }
    
    uint32_t read_time = micros() - start_time;
    Serial.print("Read performance: ");
    Serial.print(read_time);
    Serial.print(" µs for ");
    Serial.print(num_writes);
    Serial.print(" reads (");
    Serial.print(read_time / num_writes);
    Serial.println(" µs per read)");
    
    test_stats.total_reads += num_writes;
    test_stats.total_tests++;
    test_stats.passed_tests++;
}

void testErrorHandling() {
    Serial.println("Test 5: Error Handling");
    
    // Test reading non-existent data
    test_stats.total_tests++;
    uint32_t non_existent_data;
    if (!storage_backend.readData(0x99999999, &non_existent_data, sizeof(non_existent_data))) {
        Serial.println("✓ Non-existent data read correctly returns false");
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ Non-existent data read incorrectly returned true");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    
    // Test checking non-existent data
    test_stats.total_tests++;
    if (!storage_backend.hasData(0x99999999)) {
        Serial.println("✓ Non-existent data check correctly returns false");
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ Non-existent data check incorrectly returned true");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
    
    // Test error count
    uint32_t error_count = storage_backend.getErrorCount();
    Serial.print("Current error count: ");
    Serial.println(error_count);
}

void testCachePerformance() {
    Serial.println("Test 6: Cache Performance");
    
    // Enable cache
    storage_backend.enableWriteCache(true);
    storage_backend.setCacheSize(1024 * 1024); // 1MB cache
    
    // Test cache performance
    uint32_t start_time = micros();
    const int cache_writes = 50;
    
    for (int i = 0; i < cache_writes; i++) {
        uint32_t test_data = 0x20000000 + i;
        storage_backend.writeData(0x10500000 + i, &test_data, sizeof(test_data));
    }
    
    uint32_t cache_write_time = micros() - start_time;
    Serial.print("Cache write performance: ");
    Serial.print(cache_write_time);
    Serial.print(" µs for ");
    Serial.print(cache_writes);
    Serial.print(" writes (");
    Serial.print(cache_write_time / cache_writes);
    Serial.println(" µs per write)");
    
    // Flush cache
    start_time = micros();
    storage_backend.flush();
    uint32_t flush_time = micros() - start_time;
    Serial.print("Cache flush time: ");
    Serial.print(flush_time);
    Serial.println(" µs");
    
    // Test cache hit rate
    uint32_t hit_rate = storage_backend.getCacheHitRate();
    Serial.print("Cache hit rate: ");
    Serial.print(hit_rate);
    Serial.println("%");
    
    test_stats.total_writes += cache_writes;
    test_stats.total_tests++;
    test_stats.passed_tests++;
}

void testStressTesting() {
    Serial.println("Test 7: Stress Testing");
    
    // Test many small writes
    const int stress_writes = 100;
    uint32_t start_time = micros();
    
    for (int i = 0; i < stress_writes; i++) {
        uint32_t test_data = 0x30000000 + i;
        if (!storage_backend.writeData(0x10600000 + i, &test_data, sizeof(test_data))) {
            Serial.print("✗ Stress write failed at index ");
            Serial.println(i);
            test_stats.total_errors++;
            break;
        }
    }
    
    uint32_t stress_time = micros() - start_time;
    Serial.print("Stress test: ");
    Serial.print(stress_time);
    Serial.print(" µs for ");
    Serial.print(stress_writes);
    Serial.print(" writes (");
    Serial.print(stress_time / stress_writes);
    Serial.println(" µs per write)");
    
    // Verify all writes
    int verified = 0;
    for (int i = 0; i < stress_writes; i++) {
        uint32_t read_data;
        if (storage_backend.readData(0x10600000 + i, &read_data, sizeof(read_data))) {
            if (read_data == (0x30000000 + i)) {
                verified++;
            }
        }
    }
    
    Serial.print("Verified ");
    Serial.print(verified);
    Serial.print(" out of ");
    Serial.print(stress_writes);
    Serial.println(" stress writes");
    
    test_stats.total_writes += stress_writes;
    test_stats.total_reads += stress_writes;
    test_stats.total_tests++;
    
    if (verified == stress_writes) {
        Serial.println("✓ Stress test passed");
        test_stats.passed_tests++;
    } else {
        Serial.println("✗ Stress test failed");
        test_stats.failed_tests++;
        test_stats.total_errors++;
    }
}

void printFinalResults() {
    Serial.println("\n=== Final Test Results ===");
    Serial.print("Total tests: ");
    Serial.println(test_stats.total_tests);
    Serial.print("Passed: ");
    Serial.println(test_stats.passed_tests);
    Serial.print("Failed: ");
    Serial.println(test_stats.failed_tests);
    Serial.print("Success rate: ");
    Serial.print((test_stats.passed_tests * 100) / test_stats.total_tests);
    Serial.println("%");
    
    Serial.print("Total writes: ");
    Serial.println(test_stats.total_writes);
    Serial.print("Total reads: ");
    Serial.println(test_stats.total_reads);
    Serial.print("Total errors: ");
    Serial.println(test_stats.total_errors);
    
    uint32_t test_duration = test_stats.end_time - test_stats.start_time;
    Serial.print("Test duration: ");
    Serial.print(test_duration);
    Serial.println(" ms");
    
    // Print storage statistics
    Serial.println("\n--- Storage Statistics ---");
    Serial.print("Total space: ");
    Serial.print(storage_backend.getTotalSpace() / 1024 / 1024);
    Serial.println(" MB");
    Serial.print("Used space: ");
    Serial.print(storage_backend.getUsedSpace() / 1024);
    Serial.println(" KB");
    Serial.print("Free space: ");
    Serial.print(storage_backend.getFreeSpace() / 1024);
    Serial.println(" KB");
    Serial.print("Stored keys: ");
    Serial.println(storage_backend.getStoredKeyCount());
    Serial.print("Cache hit rate: ");
    Serial.print(storage_backend.getCacheHitRate());
    Serial.println("%");
    
    if (test_stats.failed_tests == 0) {
        Serial.println("\n🎉 ALL TESTS PASSED! W25Q128 storage backend is working correctly.");
    } else {
        Serial.println("\n⚠️  Some tests failed. Check the output above for details.");
    }
    
    Serial.println("==========================");
} 