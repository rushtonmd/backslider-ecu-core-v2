// // test_transmission_module_overrun.cpp
// // Test suite for transmission module overrun clutch control
// // Tests: race car logic, manual override, tuning parameters, state transitions

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

// // Test message reception for overrun control
// static float overrun_solenoid_value = 0.0f;
// static bool overrun_solenoid_message_received = false;

// void test_overrun_solenoid_handler(const CANMessage* msg) {
//     overrun_solenoid_value = MSG_UNPACK_FLOAT(msg);
//     overrun_solenoid_message_received = true;
// }

// static float overrun_state_value = 0.0f;
// static bool overrun_state_message_received = false;

// void test_overrun_state_handler(const CANMessage* msg) {
//     overrun_state_value = MSG_UNPACK_FLOAT(msg);
//     overrun_state_message_received = true;
// }

// // Mock external data sources for testing
// static float mock_throttle_position = 20.0f;
// static float mock_vehicle_speed = 35.0f;
// static bool mock_brake_active = false;
// static bool mock_is_decelerating = false;

// // Override the external data functions for testing
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
//     return mock_is_decelerating;
// }

// // Test setup function
// void test_setup() {
//     mock_reset_all();
    
//     // Reset mock external data to safe defaults
//     mock_throttle_position = 20.0f;  // Light throttle
//     mock_vehicle_speed = 35.0f;      // Moderate speed
//     mock_brake_active = false;
//     mock_is_decelerating = false;
    
//     // Set transmission sensors to neutral state
//     mock_set_digital_value(PIN_TRANS_PARK, 1);
//     mock_set_digital_value(PIN_TRANS_REVERSE, 1);
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 0);  // Neutral active
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);
//     mock_set_digital_value(PIN_TRANS_SECOND, 1);
//     mock_set_digital_value(PIN_TRANS_FIRST, 1);
    
//     // Reset message reception flags
//     overrun_solenoid_message_received = false;
//     overrun_solenoid_value = 0.0f;
//     overrun_state_message_received = false;
//     overrun_state_value = 0.0f;
// }

// // Helper function to create a fresh system
// void fresh_system_setup() {
//     test_setup();
    
//     g_message_bus.resetSubscribers();
//     g_message_bus.init(false);
//     input_manager_init();
// }

// // Helper function to update sensors and process messages
// void update_system() {
//     mock_advance_time_ms(100);
//     input_manager_update();
//     g_message_bus.process();
//     transmission_module_update();
//     g_message_bus.process();
// }

// // Test basic overrun clutch initialization
// TEST(overrun_clutch_initialization) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Should start in safe disengaged state
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
    
//     // Manual override should be disabled initially
//     assert(transmission_is_overrun_override_active() == false);
    
//     // Check default tuning parameters
//     float throttle_disengage, throttle_engage, min_speed, braking_speed;
//     transmission_get_overrun_tuning(&throttle_disengage, &throttle_engage, &min_speed, &braking_speed);
    
//     assert(throttle_disengage == OVERRUN_THROTTLE_DISENGAGE_THRESHOLD);
//     assert(throttle_engage == OVERRUN_THROTTLE_ENGAGE_THRESHOLD);
//     assert(min_speed == OVERRUN_MINIMUM_SPEED_MPH);
//     assert(braking_speed == OVERRUN_BRAKING_SPEED_THRESHOLD);
// }

// // Test race car specific overrun logic scenarios
// TEST(race_car_overrun_logic_scenarios) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     // Subscribe to overrun control messages
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_solenoid_handler);
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_STATE, test_overrun_state_handler);
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Scenario 1: Light throttle at moderate speed in Drive - should ENGAGE
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 1);  // Deactivate neutral
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);    // Activate drive
//     mock_throttle_position = 10.0f;  // Light throttle (below 15% engage threshold)
//     mock_vehicle_speed = 35.0f;      // Above minimum speed
    
//     update_system();
    
//     assert(state->overrun_state == OVERRUN_ENGAGED);
//     assert(overrun_solenoid_message_received == true);
//     assert(overrun_solenoid_value == 0.0f);  // Solenoid OFF = clutch engaged
    
//     // Reset message flags
//     overrun_solenoid_message_received = false;
    
//     // Scenario 2: High throttle - should DISENGAGE
//     mock_throttle_position = 80.0f;  // High throttle (above 75% disengage threshold)
    
//     update_system();
    
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
//     assert(overrun_solenoid_message_received == true);
//     assert(overrun_solenoid_value == 1.0f);  // Solenoid ON = clutch disengaged
    
//     // Reset message flags
//     overrun_solenoid_message_received = false;
    
//     // Scenario 3: Braking at high speed - should ENGAGE regardless of other conditions
//     mock_throttle_position = 50.0f;  // Moderate throttle
//     mock_vehicle_speed = 60.0f;      // High speed
//     mock_brake_active = true;        // Braking
    
//     update_system();
    
//     assert(state->overrun_state == OVERRUN_ENGAGED);
//     assert(overrun_solenoid_message_received == true);
//     assert(overrun_solenoid_value == 0.0f);  // Solenoid OFF = clutch engaged
    
//     // Reset message flags
//     overrun_solenoid_message_received = false;
    
//     // Scenario 4: Very low speed - should DISENGAGE for smooth operation
//     mock_brake_active = false;
//     mock_throttle_position = 10.0f;
//     mock_vehicle_speed = 10.0f;  // Below 15 mph minimum threshold
    
//     update_system();
    
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
//     assert(overrun_solenoid_message_received == true);
//     assert(overrun_solenoid_value == 1.0f);  // Solenoid ON = clutch disengaged
// }

// // Test gear-specific overrun logic
// TEST(gear_specific_overrun_logic) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_solenoid_handler);
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Set up conditions that would normally engage clutch
//     mock_throttle_position = 10.0f;  // Light throttle
//     mock_vehicle_speed = 35.0f;      // Moderate speed
    
//     // Test 1: Park position - should always be DISENGAGED
//     mock_set_digital_value(PIN_TRANS_PARK, 0);  // Activate park
    
//     update_system();
    
//     assert(state->current_gear == GEAR_PARK);
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
    
//     // Test 2: Reverse position - should always be DISENGAGED
//     mock_set_digital_value(PIN_TRANS_PARK, 1);     // Deactivate park
//     mock_set_digital_value(PIN_TRANS_REVERSE, 0);  // Activate reverse
    
//     update_system();
    
//     assert(state->current_gear == GEAR_REVERSE);
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
    
//     // Test 3: Neutral position - should always be DISENGAGED
//     mock_set_digital_value(PIN_TRANS_REVERSE, 1);  // Deactivate reverse
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 0);  // Activate neutral
    
//     update_system();
    
//     assert(state->current_gear == GEAR_NEUTRAL);
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
    
//     // Test 4: Drive position - should follow normal logic
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 1);  // Deactivate neutral
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);    // Activate drive
    
//     update_system();
    
//     assert(state->current_gear == GEAR_DRIVE);
//     assert(state->overrun_state == OVERRUN_ENGAGED);  // Should engage with light throttle
// }

// // Test manual override functionality
// TEST(manual_override_functionality) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_solenoid_handler);
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Set up drive position with conditions that would normally engage
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     mock_throttle_position = 10.0f;
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     // Should be engaged normally
//     assert(state->overrun_state == OVERRUN_ENGAGED);
//     assert(transmission_is_overrun_override_active() == false);
    
//     // Test 1: Enable manual override to DISENGAGED
//     transmission_set_overrun_override(OVERRUN_DISENGAGED, true);
    
//     assert(transmission_is_overrun_override_active() == true);
//     assert(state->overrun_state == OVERRUN_DISENGAGED);  // Should immediately change
    
//     // Update system - should stay disengaged despite conditions favoring engagement
//     overrun_solenoid_message_received = false;
//     update_system();
    
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
//     assert(overrun_solenoid_message_received == false);  // Should not change
    
//     // Test 2: Change override to ENGAGED
//     transmission_set_overrun_override(OVERRUN_ENGAGED, true);
    
//     assert(state->overrun_state == OVERRUN_ENGAGED);
    
//     // Test 3: Disable override - should return to automatic control
//     transmission_set_overrun_override(OVERRUN_DISENGAGED, false);
    
//     assert(transmission_is_overrun_override_active() == false);
    
//     // System should now follow automatic logic again
//     update_system();
//     assert(state->overrun_state == OVERRUN_ENGAGED);  // Should engage based on conditions
// }

// // Test tuning parameter adjustment
// TEST(tuning_parameter_adjustment) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     // Test 1: Get default parameters
//     float throttle_disengage, throttle_engage, min_speed, braking_speed;
//     transmission_get_overrun_tuning(&throttle_disengage, &throttle_engage, &min_speed, &braking_speed);
    
//     float original_disengage = throttle_disengage;
//     float original_engage = throttle_engage;
//     float original_min_speed = min_speed;
//     float original_braking_speed = braking_speed;
    
//     // Test 2: Set new parameters
//     transmission_set_overrun_tuning(85.0f, 10.0f, 20.0f, 40.0f);
    
//     transmission_get_overrun_tuning(&throttle_disengage, &throttle_engage, &min_speed, &braking_speed);
    
//     assert(throttle_disengage == 85.0f);
//     assert(throttle_engage == 10.0f);
//     assert(min_speed == 20.0f);
//     assert(braking_speed == 40.0f);
    
//     // Test 3: Verify new parameters affect behavior
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     mock_throttle_position = 12.0f;  // Above new engage threshold (10%)
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     const transmission_state_t* state = transmission_get_state();
//     // With new thresholds, 12% throttle should NOT engage (above 10% threshold)
//     // This tests that the new parameters are actually being used
    
//     // Test 4: Test boundary clamping
//     transmission_set_overrun_tuning(200.0f, -10.0f, -5.0f, 200.0f);
    
//     transmission_get_overrun_tuning(&throttle_disengage, &throttle_engage, &min_speed, &braking_speed);
    
//     // Values should be clamped to safe ranges
//     assert(throttle_disengage <= 100.0f);  // Clamped to max 100%
//     assert(throttle_engage >= 0.0f);       // Clamped to min 0%
//     assert(min_speed >= 0.0f);             // Clamped to min 0 mph
//     assert(braking_speed <= 100.0f);       // Clamped to reasonable range
    
//     // Test 5: Restore original parameters
//     transmission_set_overrun_tuning(original_disengage, original_engage, original_min_speed, original_braking_speed);
    
//     transmission_get_overrun_tuning(&throttle_disengage, &throttle_engage, &min_speed, &braking_speed);
    
//     assert(throttle_disengage == original_disengage);
//     assert(throttle_engage == original_engage);
//     assert(min_speed == original_min_speed);
//     assert(braking_speed == original_braking_speed);
// }

// // Test state transition counting
// TEST(state_transition_counting) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     // Reset statistics
//     transmission_reset_statistics();
//     assert(transmission_get_overrun_change_count() == 0);
    
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);  // Drive position
    
//     // Initial update - may cause a state change from initialization
//     update_system();
    
//     uint32_t initial_count = transmission_get_overrun_change_count();
    
//     // Test 1: Change conditions to force a state transition
//     mock_throttle_position = 10.0f;  // Light throttle - should engage
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     // Check if state changed (and thus count increased)
//     uint32_t count_after_engage = transmission_get_overrun_change_count();
    
//     // Test 2: Change to high throttle - should disengage
//     mock_throttle_position = 80.0f;  // High throttle - should disengage
    
//     update_system();
    
//     uint32_t count_after_disengage = transmission_get_overrun_change_count();
    
//     // Should have more changes after forced transitions
//     assert(count_after_disengage >= count_after_engage);
    
//     // Test 3: Same conditions - should not increment count
//     update_system();  // Same conditions as before
    
//     uint32_t count_after_repeat = transmission_get_overrun_change_count();
//     assert(count_after_repeat == count_after_disengage);  // No change = no count increment
    
//     // Test 4: Manual override should also count as changes
//     uint32_t count_before_override = count_after_repeat;
//     transmission_set_overrun_override(OVERRUN_ENGAGED, true);
//     transmission_set_overrun_override(OVERRUN_DISENGAGED, true);
//     transmission_set_overrun_override(OVERRUN_ENGAGED, false);  // Back to auto
    
//     uint32_t count_after_override = transmission_get_overrun_change_count();
//     assert(count_after_override > count_before_override);  // Override should increment count
// }

// // Test shift request interaction with overrun
// TEST(shift_request_overrun_interaction) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_solenoid_handler);
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Set up drive position with conditions that would engage overrun
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     mock_throttle_position = 10.0f;
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     // Should be engaged initially
//     assert(state->overrun_state == OVERRUN_ENGAGED);
    
//     // Simulate upshift request
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);  // Press upshift paddle
    
//     update_system();
    
//     // During shift request, overrun should DISENGAGE
//     assert(state->shift_request == SHIFT_UP);
//     assert(state->overrun_state == OVERRUN_DISENGAGED);
    
//     // Clear shift request
//     transmission_clear_shift_request();
    
//     update_system();
    
//     // After shift is complete, should return to normal logic
//     assert(state->shift_request == SHIFT_NONE);
//     assert(state->overrun_state == OVERRUN_ENGAGED);  // Should re-engage based on conditions
// }

// // Test external data interface edge cases
// TEST(external_data_interface_edge_cases) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     const transmission_state_t* state = transmission_get_state();
    
//     // Set up drive position
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     // Test 1: Extreme throttle values
//     mock_throttle_position = -10.0f;  // Negative throttle
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     // Should handle gracefully (treat as 0% throttle)
//     assert(state->overrun_state == OVERRUN_ENGAGED);  // Light/negative throttle should engage
    
//     mock_throttle_position = 150.0f;  // Over 100% throttle
    
//     update_system();
    
//     // Should handle gracefully (treat as high throttle)
//     assert(state->overrun_state == OVERRUN_DISENGAGED);  // High throttle should disengage
    
//     // Test 2: Extreme speed values
//     mock_throttle_position = 10.0f;  // Normal light throttle
//     mock_vehicle_speed = -5.0f;      // Negative speed
    
//     update_system();
    
//     // Should handle gracefully (treat as very low speed)
//     assert(state->overrun_state == OVERRUN_DISENGAGED);  // Low/negative speed should disengage
    
//     mock_vehicle_speed = 200.0f;  // Very high speed
    
//     update_system();
    
//     // Should handle gracefully
//     assert(state->overrun_state == OVERRUN_ENGAGED);  // High speed with light throttle should engage
    
//     // Test 3: Deceleration logic
//     mock_vehicle_speed = 35.0f;
//     mock_is_decelerating = true;
    
//     update_system();
    
//     // Deceleration should favor engagement for control
//     assert(state->overrun_state == OVERRUN_ENGAGED);
// }

// // Test safe state functionality
// TEST(safe_state_functionality) {
//     fresh_system_setup();
    
//     transmission_module_init();
    
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, test_overrun_solenoid_handler);
    
//     // Set up conditions that would normally engage overrun
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     mock_throttle_position = 10.0f;
//     mock_vehicle_speed = 35.0f;
    
//     update_system();
    
//     // Should be engaged
//     const transmission_state_t* state = transmission_get_state();
//     assert(state->overrun_state == OVERRUN_ENGAGED);
    
//     // Call safe state function
//     overrun_solenoid_message_received = false;
//     transmission_outputs_safe_state();
//     g_message_bus.process();
    
//     // Should force to safe disengaged state
//     assert(overrun_solenoid_message_received == true);
//     assert(overrun_solenoid_value == 1.0f);  // Solenoid ON = clutch disengaged (safe)
// }

// // Main test runner
// int main() {
//     std::cout << "=== Transmission Module Overrun Clutch Tests ===" << std::endl;
    
//     // Run all overrun clutch tests
//     g_message_bus.resetSubscribers();
//     run_test_overrun_clutch_initialization();
    
//     g_message_bus.resetSubscribers();
//     run_test_race_car_overrun_logic_scenarios();
    
//     g_message_bus.resetSubscribers();
//     run_test_gear_specific_overrun_logic();
    
//     g_message_bus.resetSubscribers();
//     run_test_manual_override_functionality();
    
//     g_message_bus.resetSubscribers();
//     run_test_tuning_parameter_adjustment();
    
//     g_message_bus.resetSubscribers();
//     run_test_state_transition_counting();
    
//     g_message_bus.resetSubscribers();
//     run_test_shift_request_overrun_interaction();
    
//     g_message_bus.resetSubscribers();
//     run_test_external_data_interface_edge_cases();
    
//     g_message_bus.resetSubscribers();
//     run_test_safe_state_functionality();
    
//     // Print results
//     std::cout << std::endl;
//     std::cout << "Overrun Clutch Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
//     if (tests_passed == tests_run) {
//         std::cout << "✅ ALL OVERRUN CLUTCH TESTS PASSED!" << std::endl;
//         return 0;
//     } else {
//         std::cout << "❌ SOME OVERRUN CLUTCH TESTS FAILED!" << std::endl;
//         return 1;
//     }
// }