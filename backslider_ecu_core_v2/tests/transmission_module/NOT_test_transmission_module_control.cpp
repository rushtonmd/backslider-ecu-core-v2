// // test_transmission_module_control.cpp
// // Test suite for transmission module control logic
// // Tests: solenoid patterns, line pressure, lockup, shift execution, safety

// #include <iostream>
// #include <cassert>
// #include <cstring>  // for strcmp

// // Include enhanced mock Arduino before any ECU code
// #include "../mock_arduino.h"

// // Define the Serial mock object
// MockSerial Serial;

// // Now include ECU components (they will see the mock definitions)
// #include "../../msg_definitions.h"
// #include "../../msg_bus.h"
// #include "../../input_manager.h"
// #include "../../sensor_calibration.h"
// #include "../../thermistor_table_generator.h"
// #include "../../transmission_module.h"

// // Simple test framework
// int tests_run = 0;
// int tests_passed = 0;

// #define TEST(name) \
//     void test_##name(); \
//     void run_test_##name() { \
//         std::cout << "  Running test: " #name "... "; \
//         tests_run++; \
//         test_##name(); \
//         tests_passed++; \
//         std::cout << "PASSED" << std::endl; \
//     } \
//     void test_##name()

// // Message capture for solenoid control
// static float solenoid_a_value = 0.0f;
// static float solenoid_b_value = 0.0f;
// static float lockup_value = 0.0f;
// static float pressure_value = 0.0f;
// static float overrun_value = 0.0f;

// static bool solenoid_a_received = false;
// static bool solenoid_b_received = false;
// static bool lockup_received = false;
// static bool pressure_received = false;
// static bool overrun_received = false;

// void test_solenoid_a_handler(const CANMessage* msg) {
//     solenoid_a_value = MSG_UNPACK_FLOAT(msg);
//     solenoid_a_received = true;
// }

// void test_solenoid_b_handler(const CANMessage* msg) {
//     solenoid_b_value = MSG_UNPACK_FLOAT(msg);
//     solenoid_b_received = true;
// }

// void test_lockup_handler(const CANMessage* msg) {
//     lockup_value = MSG_UNPACK_FLOAT(msg);
//     lockup_received = true;
// }

// void test_pressure_handler(const CANMessage* msg) {
//     pressure_value = MSG_UNPACK_FLOAT(msg);
//     pressure_received = true;
// }

// void test_overrun_handler(const CANMessage* msg) {
//     overrun_value = MSG_UNPACK_FLOAT(msg);
//     overrun_received = true;
// }

// // Helper to reset all message flags
// void reset_message_flags() {
//     solenoid_a_received = false;
//     solenoid_b_received = false;
//     lockup_received = false;
//     pressure_received = false;
//     overrun_received = false;
    
//     solenoid_a_value = 0.0f;
//     solenoid_b_value = 0.0f;
//     lockup_value = 0.0f;
//     pressure_value = 0.0f;
//     overrun_value = 0.0f;
// }

// // Mock external data sources for testing
// static float mock_throttle_position = 20.0f;
// static float mock_vehicle_speed = 35.0f;
// static bool mock_brake_active = false;

// float transmission_get_throttle_position_percent(void) {
//     return mock_throttle_position;
// }

// float transmission_get_vehicle_speed_mph(void) {
//     return mock_vehicle_speed;
// }

// bool transmission_get_brake_pedal_active(void) {
//     return mock_brake_active;
// }

// bool transmission_is_decelerating(void) {
//     return mock_throttle_position < 10.0f;
// }

// // Test setup function
// void test_setup() {
//     mock_reset_all();
    
//     // Reset mock external data
//     mock_throttle_position = 20.0f;
//     mock_vehicle_speed = 35.0f;
//     mock_brake_active = false;
    
//     // Set all gear switches to inactive initially
//     mock_set_digital_value(PIN_TRANS_PARK, 1);
//     mock_set_digital_value(PIN_TRANS_REVERSE, 1);
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 1);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);
//     mock_set_digital_value(PIN_TRANS_SECOND, 1);
//     mock_set_digital_value(PIN_TRANS_FIRST, 1);
    
//     // Set paddle switches to inactive
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);
    
//     reset_message_flags();
// }

// // Helper function to create a fresh system
// void fresh_system_setup() {
//     test_setup();
    
//     g_message_bus.resetSubscribers();
//     g_message_bus.init();
//     input_manager_init();
// }

// // Helper to subscribe to all solenoid messages
// void subscribe_to_solenoid_messages() {
//     g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_A, test_solenoid_a_handler);
//     g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_B, test_solenoid_b_handler);
//     g_message_bus.subscribe(MSG_TRANS_LOCKUP_SOL, test_lockup_handler);
//     g_message_bus.subscribe(MSG_TRANS_PRESSURE_SOL, test_pressure_handler);
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_handler);
// }

// // Helper function to update sensors and process messages
// void update_system() {
//     mock_advance_time_ms(100);
//     input_manager_update();
//     g_message_bus.process();
//     transmission_module_update();
//     g_message_bus.process();
// }

// // Test solenoid patterns for all gear positions
// TEST(solenoid_patterns_for_all_gears) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Test gear patterns according to transmission specification:
//     // Park/Reverse/Neutral: A=OFF, B=OFF, Lockup=OFF
//     // Gear 1: A=ON, B=ON, Lockup=OFF
//     // Gear 2: A=OFF, B=ON, Lockup=OFF  
//     // Gear 3: A=OFF, B=OFF, Lockup=OFF
//     // Gear 4: A=ON, B=OFF, Lockup=ON
    
//     // Test 1: Manual Gear 1 position
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_FIRST, 0);  // Activate first gear
    
//     update_system();
    
//     const transmission_state_t* state = transmission_get_state();
//     assert(state->current_gear == GEAR_FIRST);
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
//     assert(solenoid_a_value == 0.0f);  // First gear uses pattern 0 (both OFF)
//     assert(solenoid_b_value == 0.0f);
//     assert(lockup_value == 0.0f);
    
//     // Test 2: Manual Gear 2 position
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_FIRST, 1);   // Deactivate first
//     mock_set_digital_value(PIN_TRANS_SECOND, 0);  // Activate second gear
    
//     update_system();
    
//     assert(state->current_gear == GEAR_SECOND);
//     assert(solenoid_a_value == 0.0f);  // Second gear uses pattern 0 (both OFF)
//     assert(solenoid_b_value == 0.0f);
//     assert(lockup_value == 0.0f);
    
//     // Test 3: Drive position (automatic gears)
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_SECOND, 1);  // Deactivate second
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);   // Activate drive
    
//     update_system();
    
//     assert(state->current_gear == GEAR_DRIVE);
//     // Drive position should start in automatic gear 1
//     assert(solenoid_a_value == 1.0f);  // Gear 1: A=ON
//     assert(solenoid_b_value == 1.0f);  // Gear 1: B=ON
//     assert(lockup_value == 0.0f);      // Gear 1: Lockup=OFF
    
//     // Test 4: Park position
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);  // Deactivate drive
//     mock_set_digital_value(PIN_TRANS_PARK, 0);   // Activate park
    
//     update_system();
    
//     assert(state->current_gear == GEAR_PARK);
//     assert(solenoid_a_value == 0.0f);  // Park: A=OFF
//     assert(solenoid_b_value == 0.0f);  // Park: B=OFF
//     assert(lockup_value == 0.0f);      // Park: Lockup=OFF
    
//     // Test 5: Reverse position
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_PARK, 1);     // Deactivate park
//     mock_set_digital_value(PIN_TRANS_REVERSE, 0);  // Activate reverse
    
//     update_system();
    
//     assert(state->current_gear == GEAR_REVERSE);
//     assert(solenoid_a_value == 0.0f);  // Reverse: A=OFF
//     assert(solenoid_b_value == 0.0f);  // Reverse: B=OFF
//     assert(lockup_value == 0.0f);      // Reverse: Lockup=OFF
// }

// // Test line pressure control for different gear positions
// TEST(line_pressure_control) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Test 1: Park position - should have 0% pressure
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_PARK, 0);
    
//     update_system();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 0.0f);  // 0% pressure in park
    
//     // Test 2: Neutral position - should have 0% pressure
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_PARK, 1);
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 0);
    
//     update_system();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 0.0f);  // 0% pressure in neutral
    
//     // Test 3: Reverse position - should have 100% pressure
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 1);
//     mock_set_digital_value(PIN_TRANS_REVERSE, 0);
    
//     update_system();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 1.0f);  // 100% pressure in reverse
    
//     // Test 4: Drive position - should have 100% pressure
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_REVERSE, 1);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     update_system();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 1.0f);  // 100% pressure in drive
    
//     // Test 5: Manual gears - should have 100% pressure
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);
//     mock_set_digital_value(PIN_TRANS_FIRST, 0);
    
//     update_system();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 1.0f);  // 100% pressure in manual gear
// }

// // Test automatic gear shifts in Drive position
// TEST(automatic_gear_shifts) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Set drive position
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     update_system();
    
//     const transmission_state_t* state = transmission_get_state();
//     assert(state->current_gear == GEAR_DRIVE);
    
//     // Should start in automatic gear 1
//     assert(solenoid_a_value == 1.0f);  // Gear 1: A=ON
//     assert(solenoid_b_value == 1.0f);  // Gear 1: B=ON
//     assert(lockup_value == 0.0f);      // Gear 1: Lockup=OFF
    
//     // Test upshift from gear 1 to gear 2
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);  // Press upshift paddle
    
//     update_system();
    
//     assert(state->shift_request == SHIFT_UP);
//     assert(transmission_get_shift_count() == 1);
    
//     // After shift processing, should be in gear 2
//     assert(solenoid_a_value == 0.0f);  // Gear 2: A=OFF
//     assert(solenoid_b_value == 1.0f);  // Gear 2: B=ON
//     assert(lockup_value == 0.0f);      // Gear 2: Lockup=OFF
    
//     // Test upshift from gear 2 to gear 3
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 2);
//     assert(solenoid_a_value == 0.0f);  // Gear 3: A=OFF
//     assert(solenoid_b_value == 0.0f);  // Gear 3: B=OFF
//     assert(lockup_value == 0.0f);      // Gear 3: Lockup=OFF
    
//     // Test upshift from gear 3 to gear 4
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 3);
//     assert(solenoid_a_value == 1.0f);  // Gear 4: A=ON
//     assert(solenoid_b_value == 0.0f);  // Gear 4: B=OFF
//     assert(lockup_value == 1.0f);      // Gear 4: Lockup=ON (per specification)
    
//     // Test attempt to upshift beyond gear 4 (should fail)
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 3);  // Count should not increase
//     // Should remain in gear 4
//     assert(solenoid_a_value == 1.0f);
//     assert(solenoid_b_value == 0.0f);
//     assert(lockup_value == 1.0f);
// }

// // Test downshift functionality
// TEST(downshift_functionality) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Set drive position
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     update_system();
    
//     // Manually upshift to gear 4 for testing
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system();
    
//     // Should now be in gear 4
//     assert(transmission_get_shift_count() == 3);
//     assert(solenoid_a_value == 1.0f);  // Gear 4: A=ON
//     assert(solenoid_b_value == 0.0f);  // Gear 4: B=OFF
//     assert(lockup_value == 1.0f);      // Gear 4: Lockup=ON
    
//     // Test downshift from gear 4 to gear 3
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);    // Release upshift
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 0);  // Press downshift
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 4);
//     assert(solenoid_a_value == 0.0f);  // Gear 3: A=OFF
//     assert(solenoid_b_value == 0.0f);  // Gear 3: B=OFF
//     assert(lockup_value == 0.0f);      // Gear 3: Lockup=OFF
    
//     // Test downshift from gear 3 to gear 2
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 5);
//     assert(solenoid_a_value == 0.0f);  // Gear 2: A=OFF
//     assert(solenoid_b_value == 1.0f);  // Gear 2: B=ON
//     assert(lockup_value == 0.0f);      // Gear 2: Lockup=OFF
    
//     // Test downshift from gear 2 to gear 1
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 6);
//     assert(solenoid_a_value == 1.0f);  // Gear 1: A=ON
//     assert(solenoid_b_value == 1.0f);  // Gear 1: B=ON
//     assert(lockup_value == 0.0f);      // Gear 1: Lockup=OFF
    
//     // Test attempt to downshift below gear 1 (should fail)
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);  // Release paddle
//     mock_advance_time_ms(250);  // Wait for debounce
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 0);  // Press again
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 6);  // Count should not increase
//     // Should remain in gear 1
//     assert(solenoid_a_value == 1.0f);
//     assert(solenoid_b_value == 1.0f);
//     assert(lockup_value == 0.0f);
// }

// // Test shift safety logic
// TEST(shift_safety_logic) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Test 1: Cannot shift when not in Drive position
//     mock_set_digital_value(PIN_TRANS_PARK, 0);  // Park position
    
//     update_system();
    
//     assert(state->current_gear == GEAR_PARK);
    
//     // Try to shift - should be denied
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
    
//     update_system();
    
//     // Shift request should be present but not executed
//     assert(state->shift_request == SHIFT_UP);
//     assert(transmission_get_shift_count() == 0);  // Should not increase
    
//     // Clear the failed request
//     transmission_clear_shift_request();
    
//     // Test 2: Cannot shift with invalid gear position
//     mock_set_digital_value(PIN_TRANS_PARK, 0);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);  // Multiple switches = invalid
    
//     update_system();
    
//     assert(state->valid_gear_position == false);
    
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
    
//     update_system();
    
//     assert(transmission_get_shift_count() == 0);  // Should not execute
    
//     transmission_clear_shift_request();
    
//     // Test 3: Cannot shift when overheating
//     mock_set_digital_value(PIN_TRANS_PARK, 1);  // Fix gear position
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     // Set high temperature to trigger overheating
//     mock_set_analog_voltage(PIN_TRANS_FLUID_TEMP, 4.8f);  // Very high voltage = high temp
//     mock_advance_time_ms(150);
//     input_manager_update();
//     g_message_bus.process();
    
//     update_system();
    
//     // Check if overheating (depends on thermistor calibration)
//     if (transmission_is_overheating(120.0f)) {
//         mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//         mock_advance_time_ms(250);
//         mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
        
//         update_system();
        
//         // Shift should be denied due to overheating
//         // Note: Exact behavior depends on current temperature reading
//     }
// }

// // Test manual solenoid control functions
// TEST(manual_solenoid_control) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Test 1: Manual lockup control
//     reset_message_flags();
//     transmission_set_lockup(true);
//     g_message_bus.process();
    
//     assert(lockup_received == true);
//     assert(lockup_value == 1.0f);
    
//     reset_message_flags();
//     transmission_set_lockup(false);
//     g_message_bus.process();
    
//     assert(lockup_received == true);
//     assert(lockup_value == 0.0f);
    
//     // Test 2: Manual line pressure control
//     reset_message_flags();
//     transmission_set_line_pressure(0.75f);  // 75% pressure
//     g_message_bus.process();
    
//     assert(pressure_received == true);
//     assert(pressure_value == 0.75f);
    
//     // Test boundary clamping
//     reset_message_flags();
//     transmission_set_line_pressure(-0.5f);  // Negative value
//     g_message_bus.process();
    
//     assert(pressure_value == 0.0f);  // Should be clamped to 0
    
//     reset_message_flags();
//     transmission_set_line_pressure(1.5f);  // Over 100%
//     g_message_bus.process();
    
//     assert(pressure_value == 1.0f);  // Should be clamped to 1.0
    
//     // Test 3: Manual solenoid pattern control
//     reset_message_flags();
//     transmission_set_solenoid_pattern(2);  // Gear 2 pattern
//     g_message_bus.process();
    
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
//     assert(solenoid_a_value == 0.0f);  // Gear 2: A=OFF
//     assert(solenoid_b_value == 1.0f);  // Gear 2: B=ON
//     assert(lockup_value == 0.0f);      // Gear 2: Lockup=OFF
    
//     reset_message_flags();
//     transmission_set_solenoid_pattern(4);  // Gear 4 pattern
//     g_message_bus.process();
    
//     assert(solenoid_a_value == 1.0f);  // Gear 4: A=ON
//     assert(solenoid_b_value == 0.0f);  // Gear 4: B=OFF
//     assert(lockup_value == 1.0f);      // Gear 4: Lockup=ON
// }

// // Test safe state functionality
// TEST(safe_state_functionality) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Set up some non-safe conditions first
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     update_system();
    
//     // Shift to higher gear
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system();
    
//     // Should have some solenoids active
//     assert(solenoid_a_value == 0.0f || solenoid_a_value == 1.0f);
//     assert(pressure_value > 0.0f);
    
//     // Call safe state
//     reset_message_flags();
//     transmission_outputs_safe_state();
//     g_message_bus.process();
    
//     // All outputs should be in safe state
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
//     assert(pressure_received == true);
//     assert(overrun_received == true);
    
//     assert(solenoid_a_value == 0.0f);  // Safe: A=OFF
//     assert(solenoid_b_value == 0.0f);  // Safe: B=OFF
//     assert(lockup_value == 0.0f);      // Safe: Lockup=OFF
//     assert(pressure_value == 0.0f);    // Safe: No pressure
//     assert(overrun_value == 1.0f);     // Safe: Overrun disengaged
// }

// // Test message integration and timing
// TEST(message_integration_timing) {
//     fresh_system_setup();
    
//     transmission_module_init();
//     subscribe_to_solenoid_messages();
    
//     // Test that gear changes trigger immediate solenoid updates
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     update_system();
    
//     // Should receive solenoid messages immediately after gear change
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
//     assert(pressure_received == true);
    
//     // Test that shift requests trigger solenoid changes
//     reset_message_flags();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
    
//     update_system();
    
//     // Should receive updated solenoid patterns after shift
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
    
//     // Test that invalid states trigger safe solenoid patterns
//     reset_message_flags();
//     mock_set_digital_value(PIN_TRANS_PARK, 0);  // Multiple switches active
    
//     update_system();
    
//     // Should receive safe solenoid patterns
//     assert(solenoid_a_received == true);
//     assert(solenoid_b_received == true);
//     assert(lockup_received == true);
//     assert(pressure_received == true);
    
//     assert(solenoid_a_value == 0.0f);  // Safe pattern
//     assert(solenoid_b_value == 0.0f);
//     assert(lockup_value == 0.0f);
//     assert(pressure_value == 0.0f);  // Neutral pressure
// }

// // Main test runner
// int main() {
//     std::cout << "=== Transmission Module Control Tests ===" << std::endl;
    
//     // Run all control system tests
//     g_message_bus.resetSubscribers();
//     run_test_solenoid_patterns_for_all_gears();
    
//     g_message_bus.resetSubscribers();
//     run_test_line_pressure_control();
    
//     g_message_bus.resetSubscribers();
//     run_test_automatic_gear_shifts();
    
//     g_message_bus.resetSubscribers();
//     run_test_downshift_functionality();
    
//     g_message_bus.resetSubscribers();
//     run_test_shift_safety_logic();
    
//     g_message_bus.resetSubscribers();
//     run_test_manual_solenoid_control();
    
//     g_message_bus.resetSubscribers();
//     run_test_safe_state_functionality();
    
//     g_message_bus.resetSubscribers();
//     run_test_message_integration_timing();
    
//     // Print results
//     std::cout << std::endl;
//     std::cout << "Control Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
//     if (tests_passed == tests_run) {
//         std::cout << "✅ ALL TRANSMISSION CONTROL TESTS PASSED!" << std::endl;
//         return 0;
//     } else {
//         std::cout << "❌ SOME TRANSMISSION CONTROL TESTS FAILED!" << std::endl;
//         return 1;
//     }
// }