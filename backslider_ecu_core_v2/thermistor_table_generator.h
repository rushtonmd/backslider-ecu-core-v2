// thermistor_table_generator.h
// Helper functions to generate thermistor lookup tables from reference points
//
// This allows easy thermistor calibration by providing just two known
// temperature/resistance points instead of manually creating lookup tables.

#ifndef THERMISTOR_TABLE_GENERATOR_H
#define THERMISTOR_TABLE_GENERATOR_H

#include <stdint.h>
#include "input_manager_types.h"

// =============================================================================
// THERMISTOR TABLE GENERATION FUNCTIONS
// =============================================================================

/**
 * Generate thermistor lookup tables from two reference points using Beta equation
 * 
 * @param temp1_c       First reference temperature (Celsius)
 * @param resistance1   First reference resistance (ohms)
 * @param temp2_c       Second reference temperature (Celsius)  
 * @param resistance2   Second reference resistance (ohms)
 * @param pullup_ohms   Pullup resistor value (ohms)
 * @param temp_min_c    Minimum temperature for table (Celsius)
 * @param temp_max_c    Maximum temperature for table (Celsius)
 * @param table_size    Number of points in generated table
 * @param voltage_table Output array for voltage values (must be pre-allocated)
 * @param temp_table    Output array for temperature values (must be pre-allocated)
 * @return             Beta coefficient calculated from reference points
 */
float generate_thermistor_table(
    float temp1_c, float resistance1,
    float temp2_c, float resistance2,
    uint16_t pullup_ohms,
    float temp_min_c, float temp_max_c,
    uint8_t table_size,
    float* voltage_table,
    float* temp_table
);

/**
 * Calculate Beta coefficient from two temperature/resistance points
 * 
 * @param temp1_c       First temperature (Celsius)
 * @param resistance1   First resistance (ohms)
 * @param temp2_c       Second temperature (Celsius)
 * @param resistance2   Second resistance (ohms)
 * @return              Beta coefficient
 */
float calculate_beta_coefficient(
    float temp1_c, float resistance1,
    float temp2_c, float resistance2
);

/**
 * Calculate resistance at given temperature using Beta equation
 * 
 * @param temp_c        Temperature (Celsius)
 * @param temp_ref_c    Reference temperature (Celsius)
 * @param resistance_ref Reference resistance at temp_ref_c (ohms)
 * @param beta          Beta coefficient
 * @return              Resistance at temp_c (ohms)
 */
float calculate_resistance_at_temp(
    float temp_c, 
    float temp_ref_c, 
    float resistance_ref, 
    float beta
);

/**
 * Convert thermistor resistance to voltage using voltage divider equation
 * 
 * @param resistance    Thermistor resistance (ohms)
 * @param pullup_ohms   Pullup resistor value (ohms)
 * @param vcc           Supply voltage (default 3.3V)
 * @return              Voltage at ADC pin
 */
float resistance_to_voltage(
    float resistance, 
    uint16_t pullup_ohms, 
    float vcc
);

#endif // THERMISTOR_TABLE_GENERATOR_H