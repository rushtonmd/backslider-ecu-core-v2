// custom_canbus_manager.h
// Generic CAN Message Mapping and Translation System
//
// This module provides a generic, configurable system for extracting data from
// external CAN bus messages and translating them to internal message bus format.
// It supports any ECU brand and any message format through user configuration.
//
// Key Features:
// - Generic CAN message extraction (1-2 bytes, big/little endian)
// - Configurable scaling and validation
// - Integration with internal message bus
// - Persistent configuration storage
// - No vendor-specific code or dependencies
//
// Usage:
//   1. Configure message mappings using create_can_mapping()
//   2. Add mappings to manager using add_mapping()
//   3. Save configuration using save_configuration()
//   4. Manager automatically handles incoming CAN messages
//
// Example:
//   // Map external CAN ID 0x360 to internal throttle position
//   can_mapping_t throttle_mapping = create_can_mapping(
//       0x360,                      // External CAN ID
//       MSG_THROTTLE_POSITION,      // Internal message ID
//       0,                          // Start at byte 0
//       2,                          // 2 bytes long
//       false,                      // Little endian
//       0.1f,                       // Scale factor (raw * 0.1 = percentage)
//       0.0f,                       // Min value
//       100.0f                      // Max value
//   );
//   g_custom_canbus_manager.add_mapping(throttle_mapping);

#ifndef CUSTOM_CANBUS_MANAGER_H
#define CUSTOM_CANBUS_MANAGER_H

#include <stdint.h>
#include "msg_definitions.h"
#include "msg_bus.h"
#include "storage_manager.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "tests/mock_arduino.h"
#endif

// =============================================================================
// CONFIGURATION STRUCTURES
// =============================================================================

// Generic CAN message mapping (8 bytes - fits in one CAN message)
typedef struct {
    uint32_t external_can_id;      // 4 bytes - Source CAN ID
    uint32_t internal_msg_id;      // 4 bytes - Target internal message ID
} __attribute__((packed)) can_mapping_basic_t;

// Generic extraction parameters (8 bytes)
typedef struct {
    uint8_t byte_start;             // 1 byte - Starting byte position (0-7)
    uint8_t byte_length;            // 1 byte - Number of bytes (1-2)
    uint8_t flags;                  // 1 byte - Extraction flags (endianness, signed, etc.)
    uint8_t reserved;               // 1 byte - Reserved for future use
    float scale_factor;             // 4 bytes - Scale factor (raw_value * scale_factor)
} __attribute__((packed)) can_extraction_params_t;

// Generic validation parameters (8 bytes)
typedef struct {
    float min_value;                // 4 bytes - Minimum valid value
    float max_value;                // 4 bytes - Maximum valid value
} __attribute__((packed)) can_validation_params_t;

// Flags for extraction parameters
#define CAN_EXTRACT_FLAG_BIG_ENDIAN     0x01    // Use big endian byte order
#define CAN_EXTRACT_FLAG_SIGNED         0x02    // Interpret as signed value
#define CAN_EXTRACT_FLAG_OFFSET_BINARY  0x04    // Use offset binary encoding

// Complete mapping definition (runtime structure)
typedef struct {
    can_mapping_basic_t basic;          // Basic CAN ID mapping
    can_extraction_params_t extraction; // Data extraction parameters
    can_validation_params_t validation; // Value validation parameters
    bool enabled;                       // Enable/disable this mapping
} can_mapping_t;

// Statistics structure
typedef struct {
    uint32_t messages_processed;        // Total messages processed
    uint32_t messages_translated;       // Successfully translated messages
    uint32_t validation_errors;         // Values outside valid range
    uint32_t extraction_errors;         // Extraction format errors
    uint32_t unknown_messages;          // Messages with no configured mapping
} custom_canbus_stats_t;

// =============================================================================
// CUSTOM CAN BUS MANAGER CLASS
// =============================================================================

class CustomCanBusManager {
public:
    // Configuration constants
    static const uint8_t MAX_MAPPINGS = 16;     // Maximum number of mappings
    
    // Constructor
    CustomCanBusManager();
    
    // =========================================================================
    // INITIALIZATION AND LIFECYCLE
    // =========================================================================
    
    // Initialize the custom CAN bus manager
    bool init();
    
    // Update function - call from main loop
    void update();
    
    // Shutdown the manager
    void shutdown();
    
    // =========================================================================
    // CONFIGURATION MANAGEMENT
    // =========================================================================
    
    // Add a new CAN message mapping
    bool add_mapping(const can_mapping_t& mapping);
    
    // Remove a mapping by index
    bool remove_mapping(uint8_t index);
    
    // Enable/disable a specific mapping
    bool enable_mapping(uint8_t index, bool enabled);
    
    // Get current configuration
    uint8_t get_mapping_count() const { return mapping_count; }
    bool get_mapping(uint8_t index, can_mapping_t* mapping) const;
    
    // Check if a mapping exists for a specific CAN ID
    bool has_mapping_for_can_id(uint32_t can_id) const;
    
    // Clear all mappings
    void clear_all_mappings();
    
    // =========================================================================
    // PERSISTENT STORAGE
    // =========================================================================
    
    // Save configuration to persistent storage
    bool save_configuration();
    
    // Load configuration from persistent storage
    bool load_configuration();
    
    // =========================================================================
    // STATUS AND DIAGNOSTICS
    // =========================================================================
    
    // Get current statistics
    const custom_canbus_stats_t& get_statistics() const { return stats; }
    
    // Reset statistics
    void reset_statistics();
    
    // Check if manager is initialized
    bool is_initialized() const { return initialized; }
    
    // Debug information
    void print_configuration();
    void print_statistics();
    
    // =========================================================================
    // TESTING INTERFACE
    // =========================================================================
    
    // Public test interface for simulating CAN message reception
    void simulate_can_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    
private:
    // =========================================================================
    // INTERNAL DATA
    // =========================================================================
    
    // Configuration storage
    can_mapping_t mappings[MAX_MAPPINGS];
    uint8_t mapping_count;
    bool initialized;
    
    // Statistics
    custom_canbus_stats_t stats;
    
    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================
    
    // Message processing
    void handle_can_message(uint32_t can_id, const uint8_t* data, uint8_t length);
    static void message_handler_wrapper(uint32_t can_id, const uint8_t* data, uint8_t length);
    
    // Value extraction and validation
    bool extract_value(const uint8_t* data, uint8_t length, const can_mapping_t& mapping, float* value);
    bool validate_value(float value, const can_mapping_t& mapping);
    
    // Mapping management
    int find_mapping_by_can_id(uint32_t can_id);
    bool is_mapping_valid(const can_mapping_t& mapping);
    
    // Storage helpers
    bool save_mapping_to_storage(uint8_t index, const can_mapping_t& mapping);
    bool load_mapping_from_storage(uint8_t index, can_mapping_t* mapping);
    
    // Debug helpers
    void debug_print(const char* message);
    void debug_print_mapping(const can_mapping_t& mapping);
};

// =============================================================================
// CONFIGURATION KEYS FOR STORAGE
// =============================================================================

// Base configuration key for external CAN bus manager
#define CONFIG_EXTERNAL_CANBUS_COUNT    MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, 0x0001)

// Base for individual mapping configurations (each mapping uses 3 storage slots)
#define CONFIG_EXTERNAL_CANBUS_MAPPING_BASE  MAKE_EXTENDED_CAN_ID(ECU_BASE_PRIMARY, SUBSYSTEM_EXTERNAL, 0x0100)

// Individual mapping storage keys
#define CONFIG_EXTERNAL_CANBUS_MAPPING(index)      (CONFIG_EXTERNAL_CANBUS_MAPPING_BASE + (index * 3))
#define CONFIG_EXTERNAL_CANBUS_EXTRACTION(index)   (CONFIG_EXTERNAL_CANBUS_MAPPING_BASE + (index * 3) + 1)
#define CONFIG_EXTERNAL_CANBUS_VALIDATION(index)   (CONFIG_EXTERNAL_CANBUS_MAPPING_BASE + (index * 3) + 2)

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Create a CAN mapping with all parameters
can_mapping_t create_can_mapping(
    uint32_t external_can_id,
    uint32_t internal_msg_id,
    uint8_t byte_start,
    uint8_t byte_length,
    bool is_big_endian,
    float scale_factor,
    float min_value,
    float max_value
);

// Create a simple CAN mapping with default parameters
can_mapping_t create_simple_can_mapping(
    uint32_t external_can_id,
    uint32_t internal_msg_id,
    float scale_factor = 1.0f
);

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================

// Global custom CAN bus manager instance
extern CustomCanBusManager g_custom_canbus_manager;

#endif // CUSTOM_CANBUS_MANAGER_H 