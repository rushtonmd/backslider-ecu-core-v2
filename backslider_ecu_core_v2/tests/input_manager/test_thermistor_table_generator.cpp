// test_thermistor_table_generator.cpp
// Test suite for the thermistor table generator functionality

#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>

// Include mock Arduino for testing
#include "../mock_arduino.h"

// Include sensor calibration for interpolate_table function
#include "../../sensor_calibration.h"

// Include the thermistor table generator header
#include "../../thermistor_table_generator.h"

// Simple test framework
int tests_run = 0;
int tests_passed = 0;

#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        std::cout << "  Running test: " #name "... "; \
        tests_run++; \
        test_##name(); \
        tests_passed++; \
        std::cout << "PASSED" << std::endl; \
    } \
    void test_##name()

// Helper function to check if two floats are approximately equal
bool float_equal(float a, float b, float tolerance = 0.01f) {
    return fabsf(a - b) < tolerance;
}

// Test Beta coefficient calculation
TEST(beta_coefficient_calculation) {
    // Test with known thermistor values
    // Using standard 10K thermistor: 10K at 25°C, ~3.3K at 50°C
    float beta = calculate_beta_coefficient(25.0f, 10000.0f, 50.0f, 3300.0f);
    
    // Beta should be around 3900-4000 for typical thermistors
    assert(beta > 3500.0f && beta < 4500.0f);
    
    // Test with transmission fluid temp specs (25°C: 3.5K, 110°C: 0.25K)
    float trans_beta = calculate_beta_coefficient(25.0f, 3500.0f, 110.0f, 250.0f);
    
    // Should be a reasonable Beta value
    assert(trans_beta > 2000.0f && trans_beta < 6000.0f);
}

// Test resistance calculation at different temperatures
TEST(resistance_at_temperature) {
    // Using Beta = 3950 (typical value)
    float beta = 3950.0f;
    float ref_temp = 25.0f;
    float ref_resistance = 10000.0f;
    
    // Test at reference temperature (should return reference resistance)
    float resistance = calculate_resistance_at_temp(25.0f, ref_temp, ref_resistance, beta);
    assert(float_equal(resistance, 10000.0f, 10.0f));
    
    // Test at higher temperature (resistance should be lower)
    resistance = calculate_resistance_at_temp(50.0f, ref_temp, ref_resistance, beta);
    assert(resistance < ref_resistance);
    assert(resistance > 1000.0f);  // Should be reasonable value
    
    // Test at lower temperature (resistance should be higher)
    resistance = calculate_resistance_at_temp(0.0f, ref_temp, ref_resistance, beta);
    assert(resistance > ref_resistance);
    assert(resistance < 100000.0f);  // Should be reasonable value
}

// Test voltage divider calculation
TEST(resistance_to_voltage_conversion) {
    uint16_t pullup = 2200;  // 2.2K pullup
    float vcc = 3.3f;
    
    // Test with equal resistances (should give ~1.65V)
    float voltage = resistance_to_voltage(2200.0f, pullup, vcc);
    assert(float_equal(voltage, 1.65f, 0.1f));
    
    // Test with very high resistance (should approach VCC)
    voltage = resistance_to_voltage(220000.0f, pullup, vcc);
    assert(voltage > 3.0f);
    
    // Test with very low resistance (should approach 0V)
    voltage = resistance_to_voltage(22.0f, pullup, vcc);
    assert(voltage < 0.5f);
    
    // Test transmission fluid temp example (3.5K with 2.2K pullup)
    voltage = resistance_to_voltage(3500.0f, pullup, vcc);
    assert(voltage > 1.8f && voltage < 2.2f);  // Should be reasonable
}

// Test complete table generation
TEST(table_generation_basic) {
    const uint8_t table_size = 10;
    float voltage_table[table_size];
    float temp_table[table_size];
    
    // Generate table for transmission fluid temp
    float beta = generate_thermistor_table(
        25.0f, 3500.0f,        // 25°C: 3.5K ohms
        110.0f, 250.0f,        // 110°C: 0.25K ohms
        2200,                  // 2.2K pullup
        -20.0f, 130.0f,        // -20°C to 130°C range
        table_size,
        voltage_table,
        temp_table
    );
    
    // Verify Beta coefficient is reasonable
    assert(beta > 2000.0f && beta < 6000.0f);
    

    
    // Verify table is properly generated (generated in descending temperature order)
    // Use float_equal for floating point comparison to handle precision issues
    assert(float_equal(temp_table[0], 130.0f, 0.001f));           // First temp should be max (descending order)
    assert(float_equal(temp_table[table_size-1], -20.0f, 0.001f)); // Last temp should be min (descending order)
    
    // Verify temperatures are in descending order (high to low)
    for (uint8_t i = 1; i < table_size; i++) {
        assert(temp_table[i] < temp_table[i-1]);
    }
    
    // Verify voltages are in ascending order (higher temp = lower resistance = lower voltage)
    for (uint8_t i = 1; i < table_size; i++) {
        assert(voltage_table[i] > voltage_table[i-1]);
    }
    
    // Verify voltage values are reasonable (0-3.3V range)
    for (uint8_t i = 0; i < table_size; i++) {
        assert(voltage_table[i] >= 0.0f && voltage_table[i] <= 3.3f);
    }
}

// Test transmission fluid temperature table
TEST(transmission_fluid_temperature_table) {
    const uint8_t table_size = 20;
    float voltage_table[table_size];
    float temp_table[table_size];
    
    // Use transmission fluid temp specifications
    // 25°C: 2.5-4.5K (use 3.5K midpoint)
    // 110°C: 0.22-0.28K (use 0.25K midpoint)
    float beta = generate_thermistor_table(
        25.0f, 3500.0f,        // Reference point 1
        110.0f, 250.0f,        // Reference point 2  
        2200,                  // 2.2K pullup
        -20.0f, 130.0f,        // Operating range
        table_size,
        voltage_table,
        temp_table
    );
    
    // Use beta to avoid unused variable warning
    (void)beta;
    
    // Find the entries closest to our reference temperatures
    int idx_25c = -1, idx_110c = -1;
    for (uint8_t i = 0; i < table_size; i++) {
        if (fabsf(temp_table[i] - 25.0f) < 5.0f) idx_25c = i;
        if (fabsf(temp_table[i] - 110.0f) < 5.0f) idx_110c = i;
    }
    
    // Verify we found reasonable entries
    assert(idx_25c >= 0 && idx_110c >= 0);
    
    // Verify the voltages at reference points are reasonable
    // At 25°C with 3.5K resistance and 2.2K pullup: V = 3.3 * 3500 / (2200 + 3500) ≈ 2.03V
    float expected_25c_voltage = 3.3f * 3500.0f / (2200.0f + 3500.0f);
    assert(float_equal(voltage_table[idx_25c], expected_25c_voltage, 0.2f));
    
    // At 110°C with 250 ohm resistance: V = 3.3 * 250 / (2200 + 250) ≈ 0.34V
    float expected_110c_voltage = 3.3f * 250.0f / (2200.0f + 250.0f);
    assert(float_equal(voltage_table[idx_110c], expected_110c_voltage, 0.2f));
}

// Test edge cases and error conditions
TEST(edge_cases_and_validation) {
    const uint8_t table_size = 5;
    float voltage_table[table_size];
    float temp_table[table_size];
    
    // Test with very small temperature range
    float beta = generate_thermistor_table(
        20.0f, 10000.0f,
        21.0f, 9500.0f,
        2200,
        20.0f, 21.0f,
        table_size,
        voltage_table,
        temp_table
    );
    
    // Should still work, even with small range (generated in descending order)
    assert(beta > 0.0f);
    assert(temp_table[0] == 21.0f);           // First temp should be max (descending order)
    assert(temp_table[table_size-1] == 20.0f); // Last temp should be min (descending order)
    
    // Test with single point table - this tests the edge case handling
    const uint8_t single_size = 1;
    float single_voltage[single_size];
    float single_temp[single_size];
    
    beta = generate_thermistor_table(
        25.0f, 10000.0f,
        50.0f, 5000.0f,
        2200,
        25.0f, 30.0f,  // Range doesn't matter for single point
        single_size,
        single_voltage,
        single_temp
    );
    
    // With single size table, should get exactly the maximum temperature (as per implementation)
    assert(float_equal(single_temp[0], 30.0f, 0.001f));  // Should be the max temperature
    assert(single_voltage[0] > 0.0f && single_voltage[0] < 3.3f);
    
    // Use beta to avoid unused variable warning
    (void)beta;  // Explicitly mark as used
}

// Test that generated table works with existing interpolation
TEST(table_integration_with_interpolation) {
    const uint8_t table_size = 15;
    float voltage_table[table_size];
    float temp_table[table_size];
    
    // Generate a table
    generate_thermistor_table(
        25.0f, 3500.0f,
        110.0f, 250.0f,
        2200,
        0.0f, 120.0f,
        table_size,
        voltage_table,
        temp_table
    );
    

    
    // Verify the table was generated correctly (descending temperature order)
    assert(float_equal(temp_table[0], 120.0f, 0.001f));                  // First temp should be max (descending order)
    assert(float_equal(temp_table[table_size-1], 0.0f, 0.001f));         // Last temp should be min (descending order)
    
    // Verify voltages are in ascending order (thermistor behavior - descending temp = ascending voltage)
    for (uint8_t i = 1; i < table_size; i++) {
        assert(voltage_table[i] > voltage_table[i-1]);
    }
    
    // The voltage table is already in ascending order (low voltage at high temp, high voltage at low temp)
    // The interpolate_table function can handle both ascending and descending order
    // So we can use the tables directly without reversal
    
    // Test basic interpolation functionality - just verify it doesn't crash
    // and returns reasonable values
    float test_voltage = voltage_table[table_size/2];  // Pick a middle voltage
    float result = interpolate_table(voltage_table, temp_table, table_size, test_voltage);
    
    // Just verify the result is within a reasonable temperature range
    assert(result >= -50.0f && result <= 200.0f);  // Very broad range check
    
    // Test that exact table lookups work
    for (uint8_t i = 3; i < table_size - 3; i++) {  // Test middle values only
        float exact_result = interpolate_table(voltage_table, temp_table, table_size, voltage_table[i]);
        // The result should be close to the expected temperature (within 1°C)
        assert(float_equal(exact_result, temp_table[i], 1.0f));
    }
}

// Test mathematical accuracy against known values
TEST(mathematical_accuracy) {
    // Test against known thermistor with published Beta value
    // Standard 10K NTC thermistor typically has Beta ≈ 3950
    
    // Calculate what our function gives
    float calculated_beta = calculate_beta_coefficient(25.0f, 10000.0f, 85.0f, 1000.0f);
    
    // For a 10K thermistor going from 10K at 25°C to 1K at 85°C,
    // the Beta should be around 3900-4000
    assert(calculated_beta > 3800.0f && calculated_beta < 4100.0f);
    
    // Test resistance calculation accuracy
    float test_resistance = calculate_resistance_at_temp(50.0f, 25.0f, 10000.0f, 3950.0f);
    
    // At 50°C, a 10K@25°C thermistor with Beta=3950 should be around 3.3K
    assert(test_resistance > 3000.0f && test_resistance < 4000.0f);
}

// Main test runner
int main() {
    std::cout << "=== Thermistor Table Generator Tests ===" << std::endl;
    
    // Run all tests
    run_test_beta_coefficient_calculation();
    run_test_resistance_at_temperature();
    run_test_resistance_to_voltage_conversion();
    run_test_table_generation_basic();
    run_test_transmission_fluid_temperature_table();
    run_test_edge_cases_and_validation();
    run_test_table_integration_with_interpolation();
    run_test_mathematical_accuracy();
    
    // Print results
    std::cout << std::endl;
    std::cout << "Thermistor Generator Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
    if (tests_passed == tests_run) {
        std::cout << "✅ ALL THERMISTOR GENERATOR TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME THERMISTOR GENERATOR TESTS FAILED!" << std::endl;
        return 1;
    }
}