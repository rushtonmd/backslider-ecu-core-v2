// output_manager.h
// ECU output management system for Teensy 4.1

/* =============================================================================
 * OUTPUT MANAGER SYSTEM OVERVIEW
 * =============================================================================
 * 
 * The output manager provides a message-driven, hardware-abstracted system for
 * controlling all ECU outputs. It supports multiple output types and provides
 * complete module decoupling through the message bus architecture.
 * 
 * SUPPORTED OUTPUT TYPES:
 * - OUTPUT_PWM: PWM outputs (solenoids, motors, fans)
 * - OUTPUT_DIGITAL: Digital outputs (relays, LEDs, enable signals)
 * - OUTPUT_ANALOG: Analog outputs (gauges, voltage references)
 * - OUTPUT_SPI: SPI-based outputs (shift registers, relay boards)
 * - OUTPUT_VIRTUAL: Virtual outputs (logging, CAN messages, internal logic)
 * 
 * SYSTEM ARCHITECTURE:
 * 1. Modules register their outputs during initialization
 * 2. Modules control outputs by publishing messages to the message bus
 * 3. Output manager subscribes to control messages and updates hardware
 * 4. Hardware abstraction allows any module to control any output type
 * 
 * MESSAGE-DRIVEN CONTROL FLOW:
 * 1. Module wants to control output -> Publishes float value to output's message ID
 * 2. Output manager receives message -> Validates and processes request
 * 3. Output manager updates hardware -> Applies safety checks and limits
 * 4. Statistics and fault tracking -> Monitors performance and errors
 * 
 * EXAMPLE USAGE:
 * 
 * 1. REGISTER OUTPUTS (during initialization):
 *    ```c
 *    // Use predefined message IDs from msg_definitions.h
 *    output_definition_t outputs[] = {
 *        {23, OUTPUT_PWM, {.pwm = {1000, 10, 0.0f, 1.0f, 0.0f, 0}}, MSG_TRANS_TCC_SOL, 0, 0, 50, 0, "TCC_Solenoid"},
 *        {13, OUTPUT_DIGITAL, {.digital = {1, 0, 0}}, MSG_SHIFT_LIGHT, 0, 0, 100, 0, "Shift_Light"},
 *        {A14, OUTPUT_ANALOG, {.analog = {0, 5000, 0}}, MSG_BOOST_GAUGE, 0, 0, 50, 0, "Boost_Gauge"}
 *    };
 *    output_manager_register_outputs(outputs, 3);
 *    ```
 * 
 * 2. CONTROL OUTPUTS (from any module):
 *    ```c
 *    // Set TCC solenoid to 75% duty cycle
 *    g_message_bus.publishFloat(MSG_TRANS_TCC_SOL, 0.75f, false);
 *    
 *    // Turn on shift light
 *    g_message_bus.publishFloat(MSG_SHIFT_LIGHT, 1.0f, false);
 *    
 *    // Set boost gauge to 2.5V (50% of 5V range)
 *    g_message_bus.publishFloat(MSG_BOOST_GAUGE, 2500.0f, false); // millivolts
 *    ```
 * 
 * 3. SAFETY FEATURES:
 *    - Rate limiting prevents excessive updates
 *    - Range checking ensures values stay within configured limits
 *    - Fault detection monitors for hardware failures
 *    - Safe state function returns all outputs to default values
 *    - Enable/disable allows system-wide output control
 * 
 * 4. VIRTUAL OUTPUTS:
 *    Virtual outputs don't control hardware but trigger other actions:
 *    - Logging data to SD card
 *    - Sending CAN messages
 *    - Internal state machine triggers
 *    - Diagnostic outputs
 * 
 * 5. SPI OUTPUTS:
 *    SPI outputs allow expansion beyond Teensy pins:
 *    - Shift registers (74HC595, etc.)
 *    - SPI relay boards
 *    - Multiplexed output boards
 *    - Remote I/O modules
 * 
 * IMPORTANT NOTES:
 * - All output control is asynchronous via messages
 * - Hardware updates occur during output_manager_update() calls
 * - Rate limiting prevents bus flooding and ensures system stability
 * - Complete module decoupling - no module needs to know hardware details
 * - Fault monitoring provides diagnostics for troubleshooting
 * - Statistics tracking enables performance analysis
 * 
 * INTEGRATION:
 * Call output_manager_update() from your main loop to process all pending
 * output updates. The system will handle message processing, hardware updates,
 * safety checks, and fault monitoring automatically.
 * 
 * =============================================================================
 */

#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include "output_manager_types.h"
#include "msg_definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONFIGURATION CONSTANTS
// =============================================================================

#define OUTPUT_MANAGER_MAX_OUTPUTS 256
#define OUTPUT_MANAGER_MAX_FAULTS 64

// =============================================================================
// PUBLIC FUNCTION DECLARATIONS
// =============================================================================

// Initialize the output manager
uint8_t output_manager_init(void);

// Register output definitions with the manager
uint8_t output_manager_register_outputs(const output_definition_t* outputs, uint8_t count);

// Process all pending output updates (call from main loop)
void output_manager_update(void);

// Set all outputs to safe default states
void output_manager_safe_state(void);

// Enable/disable output processing
void output_manager_enable(uint8_t enable);

// Get current output value by index
float output_manager_get_value(uint8_t output_index);

// Set output value directly (for testing)
void output_manager_set_value(uint8_t output_index, float value);

// =============================================================================
// REAL-TIME DIRECT CONTROL (bypasses message queue for critical timing)
// =============================================================================

// Direct ignition control (microsecond precision)
void output_manager_fire_ignition_coil(uint8_t cylinder, uint32_t duration_us);

// Direct injection control (microsecond precision)  
void output_manager_fire_injector(uint8_t cylinder, uint32_t duration_us);

// Direct PWM control (immediate hardware update)
void output_manager_set_pwm_direct(uint8_t output_index, float duty_cycle);

// Schedule future output event (using hardware timers)
uint8_t output_manager_schedule_output(uint8_t output_index, uint32_t delay_us, float value, uint32_t duration_us);

// Cancel scheduled output event
void output_manager_cancel_scheduled_output(uint8_t schedule_id);

// =============================================================================
// INTERRUPT-DRIVEN IGNITION CONTROL
// =============================================================================

// Initialize interrupt-driven ignition system
uint8_t output_manager_init_ignition(uint8_t crank_trigger_pin, const uint8_t* coil_pins, uint8_t cylinder_count);

// Configure ignition timing parameters (called via message bus)
void output_manager_set_ignition_config(const ignition_config_t* config);

// Enable/disable ignition system
void output_manager_enable_ignition(uint8_t enable);

// Crank trigger interrupt handler (internal - called by hardware interrupt)
void output_manager_crank_trigger_isr(void);

// Hardware timer callback for coil turn-off (internal)
void output_manager_coil_off_timer_callback(void);

// =============================================================================
// STATISTICS AND DIAGNOSTICS
// =============================================================================

// Get statistics
const output_manager_stats_t* output_manager_get_stats(void);

// Reset statistics
void output_manager_reset_stats(void);

// Get fault status
uint8_t output_manager_get_fault_count(void);
const output_fault_record_t* output_manager_get_fault(uint8_t fault_index);
void output_manager_clear_faults(void);



#ifdef __cplusplus
}
#endif

#endif // OUTPUT_MANAGER_H 