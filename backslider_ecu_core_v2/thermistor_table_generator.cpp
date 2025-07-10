// thermistor_table_generator.cpp
// Implementation of thermistor table generation functions

#include "thermistor_table_generator.h"
#include <math.h>

// =============================================================================
// IMPLEMENTATION
// =============================================================================

float calculate_beta_coefficient(
    float temp1_c, float resistance1,
    float temp2_c, float resistance2
) {
    // Convert temperatures to Kelvin
    float temp1_k = temp1_c + 273.15f;
    float temp2_k = temp2_c + 273.15f;
    
    // Calculate Beta using: B = ln(R1/R2) / (1/T1 - 1/T2)
    float ln_ratio = logf(resistance1 / resistance2);
    float temp_diff = (1.0f / temp1_k) - (1.0f / temp2_k);
    
    return ln_ratio / temp_diff;
}

float calculate_resistance_at_temp(
    float temp_c, 
    float temp_ref_c, 
    float resistance_ref, 
    float beta
) {
    // Convert temperatures to Kelvin
    float temp_k = temp_c + 273.15f;
    float temp_ref_k = temp_ref_c + 273.15f;
    
    // Calculate resistance using: R = R_ref * exp(B * (1/T - 1/T_ref))
    float temp_diff = (1.0f / temp_k) - (1.0f / temp_ref_k);
    return resistance_ref * expf(beta * temp_diff);
}

float resistance_to_voltage(
    float resistance, 
    uint16_t pullup_ohms, 
    float vcc
) {
    // Voltage divider: V_out = V_cc * R_thermistor / (R_pullup + R_thermistor)
    return vcc * resistance / (pullup_ohms + resistance);
}

float generate_thermistor_table(
    float temp1_c, float resistance1,
    float temp2_c, float resistance2,
    uint16_t pullup_ohms,
    float temp_min_c, float temp_max_c,
    uint8_t table_size,
    float* voltage_table,
    float* temp_table
) {
    // Calculate Beta coefficient from reference points
    float beta = calculate_beta_coefficient(temp1_c, resistance1, temp2_c, resistance2);
    
    // Use first reference point as the reference for calculations
    float temp_ref_c = temp1_c;
    float resistance_ref = resistance1;
    
    // Generate table points
    float temp_step;
    if (table_size <= 1) {
        // Special case: single point table
        temp_step = 0.0f;
    } else {
        temp_step = (temp_max_c - temp_min_c) / (table_size - 1);
    }
    
    for (uint8_t i = 0; i < table_size; i++) {
        // Calculate temperature for this table entry
        float temp_c;
        if (table_size <= 1) {
            // For single point, use the minimum temperature
            temp_c = temp_min_c;
        } else {
            temp_c = temp_min_c + (i * temp_step);
        }
        
        // Calculate resistance at this temperature
        float resistance = calculate_resistance_at_temp(temp_c, temp_ref_c, resistance_ref, beta);
        
        // Convert resistance to voltage (using ADC_VOLTAGE_REF from input_manager_types.h)
        float voltage = resistance_to_voltage(resistance, pullup_ohms, ADC_VOLTAGE_REF);
        
        // Store in tables
        temp_table[i] = temp_c;
        voltage_table[i] = voltage;
    }
    
    return beta;
}