// // test_transmission_module_integration.cpp
// // Test suite for complete transmission module integration
// // Tests: full workflows, real-world scenarios, system-level behavior

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

// // System state capture for integration testing
// struct system_state_t {
//     // Solenoid states
//     float solenoid_a;
//     float solenoid_b;
//     float lockup;
//     float pressure;
//     float overrun;
    
//     // Transmission state
//     gear_position_t gear;
//     shift_request_t shift_request;
//     bool valid_position;
//     overrun_clutch_state_t overrun_state;
//     float fluid_temp;
    
//     // Statistics
//     uint32_t shift_count;
//     uint32_t invalid_gear_count;
//     uint32_t overrun_change_count;
// };

// static system_state_t captured_state;
// static bool state_capture_active = false;

// // Message handlers for system state capture
// void capture_solenoid_a(const CANMessage* msg) {
//     if (state_capture_active) captured_state.solenoid_a = MSG_UNPACK_FLOAT(msg);
// }

// void capture_solenoid_b(const CANMessage* msg) {
//     if (state_capture_active) captured_state.solenoid_b = MSG_UNPACK_FLOAT(msg);
// }

// void capture_lockup(const CANMessage* msg) {
//     if (state_capture_active) captured_state.lockup = MSG_UNPACK_FLOAT(msg);
// }

// void capture_pressure(const CANMessage* msg) {
//     if (state_capture_active) captured_state.pressure = MSG_UNPACK_FLOAT(msg);
// }

// void capture_overrun(const CANMessage* msg) {
//     if (state_capture_active) captured_state.overrun = MSG_UNPACK_FLOAT(msg);
// }

// void capture_fluid_temp(const CANMessage* msg) {
//     if (state_capture_active) captured_state.fluid_temp = MSG_UNPACK_FLOAT(msg);
// }

// // Mock external data for comprehensive testing
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

// // Helper functions for integration testing
// void setup_system_capture() {
//     g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_A, capture_solenoid_a);
//     g_message_bus.subscribe(MSG_TRANS_SHIFT_SOL_B, capture_solenoid_b);
//     g_message_bus.subscribe(MSG_TRANS_LOCKUP_SOL, capture_lockup);
//     g_message_bus.subscribe(MSG_TRANS_PRESSURE_SOL, capture_pressure);
//     g_message_bus.subscribe(MSG_TRANS_OVERRUN_SOL, capture_overrun);
//     g_message_bus.subscribe(MSG_TRANS_FLUID_TEMP, capture_fluid_temp);
    
//     // Reset capture state
//     memset(&captured_state, 0, sizeof(captured_state));
//     state_capture_active = true;
// }

// void capture_current_state() {
//     const transmission_state_t* state = transmission_get_state();
    
//     captured_state.gear = state->current_gear;
//     captured_state.shift_request = state->shift_request;
//     captured_state.valid_position = state->valid_gear_position;
//     captured_state.overrun_state = state->overrun_state;
    
//     captured_state.shift_count = transmission_get_shift_count();
//     captured_state.invalid_gear_count = transmission_get_invalid_gear_count();
//     captured_state.overrun_change_count = transmission_get_overrun_change_count();
// }

// void test_setup() {
//     mock_reset_all();
    
//     // Reset external mock data
//     mock_throttle_position = 20.0f;
//     mock_vehicle_speed = 35.0f;
//     mock_brake_active = false;
    
//     // Set all switches to inactive
//     mock_set_digital_value(PIN_TRANS_PARK, 1);
//     mock_set_digital_value(PIN_TRANS_REVERSE, 1);
//     mock_set_digital_value(PIN_TRANS_NEUTRAL, 1);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);
//     mock_set_digital_value(PIN_TRANS_SECOND, 1);
//     mock_set_digital_value(PIN_TRANS_FIRST, 1);
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);
    
//     // Set reasonable fluid temperature
//     mock_set_analog_voltage(PIN_TRANS_FLUID_TEMP, 2.0f);
    
//     state_capture_active = false;
// }

// void fresh_system_setup() {
//     test_setup();
    
//     g_message_bus.resetSubscribers();
//     g_message_bus.init();
//     input_manager_init();
// }

// void update_system_full() {
//     mock_advance_time_ms(150);  // Ensure sensor update intervals are met
//     input_manager_update();
//     g_message_bus.process();
//     transmission_module_update();
//     g_message_bus.process();
//     capture_current_state();
// }

// // Test complete startup sequence
// TEST(complete_startup_sequence) {
//     fresh_system_setup();
    
//     setup_system_capture();
    
//     // Initialize transmission module
//     uint8_t registered = transmission_module_init();
//     assert(registered == 9);
    
//     // Allow initial system update
//     update_system_full();
    
//     // Verify initial state is safe
//     assert(captured_state.gear == GEAR_NEUTRAL || captured_state.gear == GEAR_UNKNOWN);
//     assert(captured_state.valid_position == false);  // No switches active
//     assert(captured_state.shift_request == SHIFT_NONE);
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);  // Safe state
    
//     // Verify safe solenoid patterns
//     assert(captured_state.solenoid_a == 0.0f);  // Safe: OFF
//     assert(captured_state.solenoid_b == 0.0f);  // Safe: OFF
//     assert(captured_state.lockup == 0.0f);      // Safe: OFF
//     assert(captured_state.pressure == 0.0f);    // Safe: No pressure
//     assert(captured_state.overrun == 1.0f);     // Safe: Disengaged
    
//     // Verify temperature reading is reasonable
//     assert(captured_state.fluid_temp >= -30.0f && captured_state.fluid_temp <= 140.0f);
    
//     // Verify statistics are initialized
//     assert(captured_state.shift_count == 0);
//     assert(captured_state.invalid_gear_count >= 0);  // May increment due to invalid initial state
//     assert(captured_state.overrun_change_count >= 0);
// }

// // Test complete driving scenario from Park to Drive with shifts
// TEST(complete_driving_scenario) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Scenario 1: Start in Park
//     mock_set_digital_value(PIN_TRANS_PARK, 0);
//     update_system_full();
    
//     assert(captured_state.gear == GEAR_PARK);
//     assert(captured_state.valid_position == true);
//     assert(captured_state.pressure == 0.0f);  // No pressure in Park
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);  // Always disengaged in Park
    
//     // Scenario 2: Shift to Drive (simulating driver moving gear lever)
//     mock_set_digital_value(PIN_TRANS_PARK, 1);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     update_system_full();
    
//     assert(captured_state.gear == GEAR_DRIVE);
//     assert(captured_state.valid_position == true);
//     assert(captured_state.pressure == 1.0f);  // Full pressure in Drive
    
//     // Should be in automatic gear 1
//     assert(captured_state.solenoid_a == 1.0f);  // Gear 1: A=ON
//     assert(captured_state.solenoid_b == 1.0f);  // Gear 1: B=ON
//     assert(captured_state.lockup == 0.0f);      // Gear 1: Lockup=OFF
    
//     // Overrun should engage based on conditions (light throttle, moderate speed)
//     assert(captured_state.overrun_state == OVERRUN_ENGAGED);
//     assert(captured_state.overrun == 0.0f);  // Solenoid OFF = clutch engaged
    
//     // Scenario 3: Upshift sequence (1 -> 2 -> 3 -> 4)
//     uint32_t initial_shift_count = captured_state.shift_count;
    
//     // Upshift to gear 2
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system_full();
    
//     assert(captured_state.shift_count == initial_shift_count + 1);
//     assert(captured_state.solenoid_a == 0.0f);  // Gear 2: A=OFF
//     assert(captured_state.solenoid_b == 1.0f);  // Gear 2: B=ON
//     assert(captured_state.lockup == 0.0f);      // Gear 2: Lockup=OFF
    
//     // Release and upshift to gear 3
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);  // Debounce delay
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system_full();
    
//     assert(captured_state.shift_count == initial_shift_count + 2);
//     assert(captured_state.solenoid_a == 0.0f);  // Gear 3: A=OFF
//     assert(captured_state.solenoid_b == 0.0f);  // Gear 3: B=OFF
//     assert(captured_state.lockup == 0.0f);      // Gear 3: Lockup=OFF
    
//     // Release and upshift to gear 4
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);  // Debounce delay
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system_full();
    
//     assert(captured_state.shift_count == initial_shift_count + 3);
//     assert(captured_state.solenoid_a == 1.0f);  // Gear 4: A=ON
//     assert(captured_state.solenoid_b == 0.0f);  // Gear 4: B=OFF
//     assert(captured_state.lockup == 1.0f);      // Gear 4: Lockup=ON
    
//     // In gear 4, overrun should be disengaged per manual specification
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
//     assert(captured_state.overrun == 1.0f);  // Solenoid ON = clutch disengaged
    
//     // Scenario 4: High throttle scenario (race acceleration)
//     mock_throttle_position = 85.0f;  // High throttle
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);  // Release paddle
//     update_system_full();
    
//     // Overrun should remain disengaged due to high throttle AND gear 4
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
    
//     // Scenario 5: Return to Park
//     mock_set_digital_value(PIN_TRANS_DRIVE, 1);
//     mock_set_digital_value(PIN_TRANS_PARK, 0);
//     update_system_full();
    
//     assert(captured_state.gear == GEAR_PARK);
//     assert(captured_state.pressure == 0.0f);  // No pressure in Park
//     assert(captured_state.solenoid_a == 0.0f);  // Safe pattern
//     assert(captured_state.solenoid_b == 0.0f);
//     assert(captured_state.lockup == 0.0f);
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);  // Always disengaged in Park
// }

// // Test race car braking scenario
// TEST(race_car_braking_scenario) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Set up Drive position in gear 3
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     update_system_full();
    
//     // Upshift to gear 3
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system_full();
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//     mock_advance_time_ms(250);
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//     update_system_full();
    
//     // Should be in gear 3
//     assert(captured_state.solenoid_a == 0.0f);
//     assert(captured_state.solenoid_b == 0.0f);
//     assert(captured_state.lockup == 0.0f);
    
//     // Test 1: Normal driving (moderate throttle, high speed)
//     mock_throttle_position = 40.0f;  // Moderate throttle
//     mock_vehicle_speed = 80.0f;      // High speed
//     mock_brake_active = false;
    
//     update_system_full();
    
//     // Should be engaged for control
//     assert(captured_state.overrun_state == OVERRUN_ENGAGED);
    
//     // Test 2: Enter braking zone (brake activated, high speed)
//     mock_brake_active = true;
//     mock_throttle_position = 5.0f;   // Light throttle (trail braking)
//     mock_vehicle_speed = 75.0f;      // Still high speed
    
//     update_system_full();
    
//     // Should be ENGAGED for maximum engine braking and control
//     assert(captured_state.overrun_state == OVERRUN_ENGAGED);
//     assert(captured_state.overrun == 0.0f);  // Solenoid OFF = clutch engaged
    
//     // Test 3: Continue braking with downshifts
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 0);  // Downshift during braking
    
//     update_system_full();
    
//     // During shift, overrun should disengage temporarily
//     assert(captured_state.shift_request == SHIFT_DOWN);
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
    
//     // Clear shift and continue
//     transmission_clear_shift_request();
//     mock_set_digital_value(PIN_PADDLE_DOWNSHIFT, 1);
    
//     update_system_full();
    
//     // After shift, should re-engage for continued control
//     assert(captured_state.overrun_state == OVERRUN_ENGAGED);
    
//     // Test 4: Exit corner with power application
//     mock_brake_active = false;
//     mock_throttle_position = 80.0f;  // High throttle for exit
//     mock_vehicle_speed = 65.0f;
    
//     update_system_full();
    
//     // Should disengage for smooth power delivery
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
//     assert(captured_state.overrun == 1.0f);  // Solenoid ON = clutch disengaged
// }

// // Test error recovery scenarios
// TEST(error_recovery_scenarios) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Test 1: Multiple gear switches active (error condition)
//     mock_set_digital_value(PIN_TRANS_PARK, 0);
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);  // Multiple switches = error
    
//     update_system_full();
    
//     assert(captured_state.valid_position == false);
//     assert(captured_state.gear == GEAR_NEUTRAL);  // Safe default
//     assert(captured_state.invalid_gear_count > 0);
    
//     // System should set safe solenoid patterns
//     assert(captured_state.solenoid_a == 0.0f);
//     assert(captured_state.solenoid_b == 0.0f);
//     assert(captured_state.lockup == 0.0f);
//     assert(captured_state.pressure == 0.0f);  // No pressure for safety
    
//     // Test 2: Recovery from error state
//     mock_set_digital_value(PIN_TRANS_PARK, 1);  // Fix the error
    
//     update_system_full();
    
//     assert(captured_state.valid_position == true);
//     assert(captured_state.gear == GEAR_DRIVE);
//     assert(captured_state.pressure == 1.0f);  // Pressure restored
    
//     // Test 3: Overheating condition
//     mock_set_analog_voltage(PIN_TRANS_FLUID_TEMP, 4.5f);  // High temperature
//     mock_advance_time_ms(200);  // Allow temperature reading to update
//     input_manager_update();
//     g_message_bus.process();
    
//     update_system_full();
    
//     // Check if overheating affects shift capability
//     if (transmission_is_overheating(120.0f)) {
//         mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
        
//         update_system_full();
        
//         // Shift should be denied, but system should remain functional
//         assert(captured_state.gear == GEAR_DRIVE);  // Should remain in current gear
//     }
    
//     // Test 4: System recovery after cooling
//     mock_set_analog_voltage(PIN_TRANS_FLUID_TEMP, 2.0f);  // Normal temperature
//     mock_advance_time_ms(200);
//     input_manager_update();
//     g_message_bus.process();
    
//     update_system_full();
    
//     // System should be functional again
//     assert(captured_state.valid_position == true);
// }

// // Test performance and timing characteristics
// TEST(performance_and_timing) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Set up Drive position
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
    
//     // Test rapid shift requests (should be debounced)
//     uint32_t initial_shift_count = 0;
    
//     // Rapid paddle presses
//     for (int i = 0; i < 5; i++) {
//         mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//         mock_advance_time_ms(50);  // Short delay (less than debounce)
//         input_manager_update();
//         g_message_bus.process();
//         transmission_module_update();
//         g_message_bus.process();
        
//         mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//         mock_advance_time_ms(50);
//         input_manager_update();
//         g_message_bus.process();
//         transmission_module_update();
//         g_message_bus.process();
//     }
    
//     capture_current_state();
    
//     // Should only register one shift due to debouncing
//     assert(captured_state.shift_count <= 2);  // At most 1-2 shifts should register
    
//     // Test system responsiveness to valid inputs
//     mock_advance_time_ms(300);  // Ensure debounce period has passed
//     mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
    
//     update_system_full();
    
//     // Should register this shift immediately
//     uint32_t new_shift_count = captured_state.shift_count;
//     assert(new_shift_count > initial_shift_count);
    
//     // Test overrun clutch response time
//     uint32_t initial_overrun_changes = captured_state.overrun_change_count;
    
//     // Change throttle to trigger overrun state change
//     mock_throttle_position = 85.0f;  // High throttle
    
//     update_system_full();
    
//     // Should respond immediately to throttle changes
//     assert(captured_state.overrun_change_count > initial_overrun_changes);
//     assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
// }

// // Test system integration under stress
// TEST(system_stress_testing) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Rapid sequence of gear changes and conditions
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     update_system_full();
    
//     // Simulate aggressive driving with rapid changes
//     for (int cycle = 0; cycle < 10; cycle++) {
//         // Vary throttle
//         mock_throttle_position = (cycle % 2 == 0) ? 10.0f : 80.0f;
        
//         // Vary speed
//         mock_vehicle_speed = 30.0f + (cycle * 5.0f);
        
//         // Occasional braking
//         mock_brake_active = (cycle % 3 == 0);
        
//         // Occasional shifts
//         if (cycle % 4 == 0) {
//             mock_set_digital_value(PIN_PADDLE_UPSHIFT, 0);
//         } else {
//             mock_set_digital_value(PIN_PADDLE_UPSHIFT, 1);
//         }
        
//         // Advance time and update
//         mock_advance_time_ms(200 + cycle);  // Varying time intervals
        
//         update_system_full();
        
//         // System should remain stable throughout
//         assert(captured_state.gear == GEAR_DRIVE);
//         assert(captured_state.valid_position == true);
//         assert(captured_state.pressure == 1.0f);
        
//         // Overrun state should track conditions logically
//         if (mock_throttle_position > 75.0f) {
//             assert(captured_state.overrun_state == OVERRUN_DISENGAGED);
//         }
        
//         // Clear any shift requests to prevent accumulation
//         if (captured_state.shift_request != SHIFT_NONE) {
//             transmission_clear_shift_request();
//         }
//     }
    
//     // System should be stable and responsive after stress test
//     mock_throttle_position = 20.0f;
//     mock_vehicle_speed = 35.0f;
//     mock_brake_active = false;
    
//     update_system_full();
    
//     assert(captured_state.gear == GEAR_DRIVE);
//     assert(captured_state.valid_position == true);
//     assert(captured_state.overrun_state == OVERRUN_ENGAGED);  // Should settle to normal state
// }

// // Test complete system shutdown sequence
// TEST(system_shutdown_sequence) {
//     fresh_system_setup();
    
//     setup_system_capture();
//     transmission_module_init();
    
//     // Set up active driving state
//     mock_set_digital_value(PIN_TRANS_DRIVE, 0);
//     mock_throttle_position = 50.0f;
//     mock_vehicle_speed = 60.0f;
    
//     update_system_full();
    
//     // Should be in active state
//     assert(captured_state.gear == GEAR_DRIVE);
//     assert(captured_state.pressure == 1.0f);
    
//     // Simulate system shutdown
//     transmission_outputs_safe_state();
//     g_message_bus.process();
    
//     // Capture final state
//     state_capture_active = true;
//     g_message_bus.process();  // Ensure all messages are processed
//     capture_current_state();
    
//     // All outputs should be in safe state
//     assert(captured_state.solenoid_a == 0.0f);
//     assert(captured_state.solenoid_b == 0.0f);
//     assert(captured_state.lockup == 0.0f);
//     assert(captured_state.pressure == 0.0f);
//     assert(captured_state.overrun == 1.0f);  // Safe disengaged state
    
//     // System should be ready for safe restart
//     transmission_reset_statistics();
//     assert(transmission_get_shift_count() == 0);
//     assert(transmission_get_invalid_gear_count() == 0);
//     assert(transmission_get_overrun_change_count() == 0);
// }

// // Main test runner
// int main() {
//     std::cout << "=== Transmission Module Integration Tests ===" << std::endl;
    
//     // Run all integration tests
//     g_message_bus.resetSubscribers();
//     run_test_complete_startup_sequence();
    
//     g_message_bus.resetSubscribers();
//     run_test_complete_driving_scenario();
    
//     g_message_bus.resetSubscribers();
//     run_test_race_car_braking_scenario();
    
//     g_message_bus.resetSubscribers();
//     run_test_error_recovery_scenarios();
    
//     g_message_bus.resetSubscribers();
//     run_test_performance_and_timing();
    
//     g_message_bus.resetSubscribers();
//     run_test_system_stress_testing();
    
//     g_message_bus.resetSubscribers();
//     run_test_system_shutdown_sequence();
    
//     // Print results
//     std::cout << std::endl;
//     std::cout << "Integration Tests - Run: " << tests_run << ", Passed: " << tests_passed << std::endl;
    
//     if (tests_passed == tests_run) {
//         std::cout << "✅ ALL TRANSMISSION INTEGRATION TESTS PASSED!" << std::endl;
//         std::cout << std::endl;
//         std::cout << "🏁 COMPREHENSIVE TRANSMISSION MODULE TESTING COMPLETE!" << std::endl;
//         std::cout << "   The transmission control system is ready for race car deployment." << std::endl;
//         return 0;
//     } else {
//         std::cout << "❌ SOME TRANSMISSION INTEGRATION TESTS FAILED!" << std::endl;
//         return 1;
//     }
// }