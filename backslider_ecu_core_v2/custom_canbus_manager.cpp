// custom_canbus_manager.cpp
// Generic CAN Message Mapping and Translation System Implementation

#include "custom_canbus_manager.h"
#include "external_canbus.h"
#include "storage_manager.h"

// Global instance
CustomCanBusManager g_custom_canbus_manager;

// External global instances
extern StorageManager g_storage_manager;

// Static pointer for message handler callback
static CustomCanBusManager* g_manager_instance = nullptr;

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

CustomCanBusManager::CustomCanBusManager() :
    mapping_count(0),
    initialized(false)
{
    // Initialize mappings array
    for (uint8_t i = 0; i < MAX_MAPPINGS; i++) {
        mappings[i] = {};
    }
    
    // Initialize statistics
    reset_statistics();
}

bool CustomCanBusManager::init() {
    if (initialized) {
        debug_print("CustomCanBusManager: Already initialized");
        return true;
    }
    
    debug_print("CustomCanBusManager: Initializing...");
    
    #ifdef ARDUINO
    Serial.print("CustomCanBusManager: Mapping count: ");
    Serial.println(mapping_count);
    #else
    printf("CustomCanBusManager: Mapping count: %d\n", mapping_count);
    #endif
    
    // Set global instance pointer for callback
    g_manager_instance = this;
    
    // Load configuration from storage
    // TEMPORARILY COMMENTED OUT FOR DEBUGGING
    // if (!load_configuration()) {
    //     debug_print("CustomCanBusManager: Warning - Could not load configuration");
    //     // Continue with empty configuration
    // }
    
    // Register message handlers for all configured mappings
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (mappings[i].enabled) {
            // Register with external CAN bus system
            if (!g_external_canbus.register_custom_handler(
                mappings[i].basic.external_can_id, 
                message_handler_wrapper)) {
                debug_print("CustomCanBusManager: Warning - Failed to register handler");
            } else {
                #ifdef ARDUINO
                Serial.print("CustomCanBusManager: Successfully registered handler for CAN ID 0x");
                Serial.println(mappings[i].basic.external_can_id, HEX);
                #else
                printf("CustomCanBusManager: Successfully registered handler for CAN ID 0x%08X\n", 
                       mappings[i].basic.external_can_id);
                #endif
            }
        }
    }
    
    initialized = true;
    debug_print("CustomCanBusManager: Initialization complete");
    
    return true;
}

void CustomCanBusManager::update() {
    if (!initialized) {
        return;
    }
    
    // Currently no periodic processing needed
    // All message processing happens in the callback handlers
}

void CustomCanBusManager::shutdown() {
    if (!initialized) {
        return;
    }
    
    // Unregister all handlers
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (mappings[i].enabled) {
            g_external_canbus.unregister_custom_handler(mappings[i].basic.external_can_id);
        }
    }
    
    initialized = false;
    g_manager_instance = nullptr;
    
    debug_print("CustomCanBusManager: Shutdown complete");
}

// =============================================================================
// CONFIGURATION MANAGEMENT
// =============================================================================

bool CustomCanBusManager::add_mapping(const can_mapping_t& mapping) {
    if (!initialized) {
        debug_print("CustomCanBusManager: Not initialized");
        return false;
    }
    
    if (mapping_count >= MAX_MAPPINGS) {
        debug_print("CustomCanBusManager: Maximum mappings reached");
        return false;
    }
    
    if (!is_mapping_valid(mapping)) {
        debug_print("CustomCanBusManager: Invalid mapping configuration");
        return false;
    }
    
    // Check for duplicate CAN ID
    if (find_mapping_by_can_id(mapping.basic.external_can_id) >= 0) {
        debug_print("CustomCanBusManager: CAN ID already mapped");
        return false;
    }
    
    // Add mapping
    mappings[mapping_count] = mapping;
    
    // Register handler if enabled
    if (mapping.enabled) {
        if (!g_external_canbus.register_custom_handler(
            mapping.basic.external_can_id, 
            message_handler_wrapper)) {
            debug_print("CustomCanBusManager: Warning - Failed to register handler");
        }
    }
    
    mapping_count++;
    
    debug_print("CustomCanBusManager: Added mapping");
    debug_print_mapping(mapping);
    
    return true;
}

bool CustomCanBusManager::remove_mapping(uint8_t index) {
    if (!initialized || index >= mapping_count) {
        return false;
    }
    
    // Unregister handler if enabled
    if (mappings[index].enabled) {
        g_external_canbus.unregister_custom_handler(mappings[index].basic.external_can_id);
    }
    
    // Shift remaining mappings down
    for (uint8_t i = index; i < mapping_count - 1; i++) {
        mappings[i] = mappings[i + 1];
    }
    
    // Clear the last mapping
    mappings[mapping_count - 1] = {};
    mapping_count--;
    
    debug_print("CustomCanBusManager: Removed mapping");
    
    return true;
}

bool CustomCanBusManager::enable_mapping(uint8_t index, bool enabled) {
    if (!initialized || index >= mapping_count) {
        return false;
    }
    
    if (mappings[index].enabled == enabled) {
        return true;  // No change needed
    }
    
    mappings[index].enabled = enabled;
    
    if (enabled) {
        // Register handler
        if (!g_external_canbus.register_custom_handler(
            mappings[index].basic.external_can_id, 
            message_handler_wrapper)) {
            debug_print("CustomCanBusManager: Warning - Failed to register handler");
        }
    } else {
        // Unregister handler
        g_external_canbus.unregister_custom_handler(mappings[index].basic.external_can_id);
    }
    
    return true;
}

bool CustomCanBusManager::get_mapping(uint8_t index, can_mapping_t* mapping) const {
    if (!initialized || index >= mapping_count || mapping == nullptr) {
        return false;
    }
    
    *mapping = mappings[index];
    return true;
}

void CustomCanBusManager::clear_all_mappings() {
    if (!initialized) {
        return;
    }
    
    // Unregister all handlers
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (mappings[i].enabled) {
            g_external_canbus.unregister_custom_handler(mappings[i].basic.external_can_id);
        }
    }
    
    // Clear all mappings
    for (uint8_t i = 0; i < MAX_MAPPINGS; i++) {
        mappings[i] = {};
    }
    
    mapping_count = 0;
    
    debug_print("CustomCanBusManager: Cleared all mappings");
}

// =============================================================================
// PERSISTENT STORAGE
// =============================================================================

bool CustomCanBusManager::save_configuration() {
    if (!initialized) {
        return false;
    }
    
    // Save mapping count
    uint8_t count = mapping_count;
    if (!g_storage_manager.save_data(CONFIG_EXTERNAL_CANBUS_COUNT, &count, sizeof(count))) {
        debug_print("CustomCanBusManager: Failed to save mapping count");
        return false;
    }
    
    // Save each mapping (3 storage entries per mapping)
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (!save_mapping_to_storage(i, mappings[i])) {
            debug_print("CustomCanBusManager: Failed to save mapping");
            return false;
        }
    }
    
    debug_print("CustomCanBusManager: Configuration saved successfully");
    return true;
}

bool CustomCanBusManager::load_configuration() {
    // Load mapping count
    uint8_t count = 0;
    if (!g_storage_manager.load_data(CONFIG_EXTERNAL_CANBUS_COUNT, &count, sizeof(count))) {
        // No configuration saved yet - this is normal for first run
        mapping_count = 0;
        return true;
    }
    
    if (count > MAX_MAPPINGS) {
        debug_print("CustomCanBusManager: Invalid mapping count in storage");
        count = MAX_MAPPINGS;  // Clamp to maximum
    }
    
    // Load each mapping
    mapping_count = 0;
    for (uint8_t i = 0; i < count; i++) {
        can_mapping_t mapping = {};
        
        if (load_mapping_from_storage(i, &mapping)) {
            if (is_mapping_valid(mapping)) {
                mappings[mapping_count++] = mapping;
            } else {
                debug_print("CustomCanBusManager: Invalid mapping in storage - skipping");
            }
        } else {
            debug_print("CustomCanBusManager: Failed to load mapping from storage");
        }
    }
    
    debug_print("CustomCanBusManager: Configuration loaded successfully");
    return true;
}

// =============================================================================
// MESSAGE PROCESSING
// =============================================================================

void CustomCanBusManager::handle_can_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    #ifdef ARDUINO
    Serial.print("DEBUG: handle_can_message called for CAN ID 0x");
    Serial.print(can_id, HEX);
    Serial.print(" with ");
    Serial.print(length);
    Serial.println(" bytes");
    #else
    printf("DEBUG: handle_can_message called for CAN ID 0x%08X with %d bytes\n", can_id, length);
    #endif
    
    if (!initialized) {
        #ifdef ARDUINO
        Serial.println("DEBUG: CustomCanBusManager not initialized!");
        #else
        printf("DEBUG: CustomCanBusManager not initialized!\n");
        #endif
        return;
    }
    
    stats.messages_processed++;
    
    // Find mapping for this CAN ID
    int mapping_index = find_mapping_by_can_id(can_id);
    #ifdef ARDUINO
    Serial.print("DEBUG: Mapping index for CAN 0x");
    Serial.print(can_id, HEX);
    Serial.print(": ");
    Serial.println(mapping_index);
    #else
    printf("DEBUG: Mapping index for CAN 0x%08X: %d\n", can_id, mapping_index);
    #endif
    
    if (mapping_index < 0) {
        stats.unknown_messages++;
        #ifdef ARDUINO
        // DEBUG: Show unmapped CAN messages
        Serial.print("CustomCanBusManager: No mapping for CAN 0x");
        Serial.print(can_id, HEX);
        Serial.println(" (unknown message)");
        #else
        printf("CustomCanBusManager: No mapping for CAN 0x%08X (unknown message)\n", can_id);
        #endif
        return;
    }
    
    const can_mapping_t& mapping = mappings[mapping_index];
    
    #ifdef ARDUINO
    Serial.print("DEBUG: Mapping found - enabled: ");
    Serial.println(mapping.enabled ? "YES" : "NO");
    #else
    printf("DEBUG: Mapping found - enabled: %s\n", mapping.enabled ? "YES" : "NO");
    #endif
    
    if (!mapping.enabled) {
        #ifdef ARDUINO
        Serial.println("DEBUG: Mapping is disabled, returning");
        #else
        printf("DEBUG: Mapping is disabled, returning\n");
        #endif
        return;  // Mapping is disabled
    }
    
    // Extract value from CAN message
    float value = 0.0f;
    #ifdef ARDUINO
    Serial.println("DEBUG: About to extract value from CAN data");
    #else
    printf("DEBUG: About to extract value from CAN data\n");
    #endif
    
    if (!extract_value(data, length, mapping, &value)) {
        // Extraction failed - error already logged in extract_value
        #ifdef ARDUINO
        Serial.println("DEBUG: Value extraction failed");
        #else
        printf("DEBUG: Value extraction failed\n");
        #endif
        return;
    }
    
    #ifdef ARDUINO
    Serial.print("DEBUG: Value extracted successfully: ");
    Serial.println(value);
    #else
    printf("DEBUG: Value extracted successfully: %.2f\n", value);
    #endif
    
    // Validate extracted value
    #ifdef ARDUINO
    Serial.print("DEBUG: About to validate value: ");
    Serial.println(value);
    #else
    printf("DEBUG: About to validate value: %.2f\n", value);
    #endif
    
    if (!validate_value(value, mapping)) {
        stats.validation_errors++;
        #ifdef ARDUINO
        Serial.println("DEBUG: Value validation failed");
        #else
        printf("DEBUG: Value validation failed\n");
        #endif
        return;
    }
    
    #ifdef ARDUINO
    Serial.println("DEBUG: Value validation passed");
    #else
    printf("DEBUG: Value validation passed\n");
    #endif
    
    // Publish to internal message bus
    g_message_bus.publishFloat(mapping.basic.internal_msg_id, value);
    
    stats.messages_translated++;
    
    #ifdef ARDUINO
    // DEBUG: Show successful translation
    Serial.print("CustomCanBusManager: CAN 0x");
    Serial.print(can_id, HEX);
    Serial.print(" -> MSG 0x");
    Serial.print(mapping.basic.internal_msg_id, HEX);
    Serial.print(" = ");
    Serial.print(value);
    Serial.println(" (translated successfully)");
    #else
    printf("CustomCanBusManager: CAN 0x%08X -> MSG 0x%08X = %.2f (translated successfully)\n", 
           can_id, mapping.basic.internal_msg_id, value);
    #endif
}

// Static wrapper for message handler callback
void CustomCanBusManager::message_handler_wrapper(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (g_manager_instance != nullptr) {
        g_manager_instance->handle_can_message(can_id, data, length);
    }
}

// =============================================================================
// VALUE EXTRACTION AND VALIDATION
// =============================================================================

bool CustomCanBusManager::extract_value(const uint8_t* data, uint8_t length, const can_mapping_t& mapping, float* value) {
    const auto& extract = mapping.extraction;
    
    // Validate input parameters
    if (data == nullptr || value == nullptr) {
        stats.extraction_errors++;
        return false;
    }
    
    // Validate byte positions
    if (extract.byte_start + extract.byte_length > length) {
        stats.extraction_errors++;
        return false;
    }
    
    // Extract raw value based on byte length
    uint32_t raw_value = 0;
    
    if (extract.byte_length == 1) {
        // Single byte value
        raw_value = data[extract.byte_start];
    } else if (extract.byte_length == 2) {
        // 16-bit value
        if (extract.flags & CAN_EXTRACT_FLAG_BIG_ENDIAN) {
            // Big endian: MSB first
            raw_value = (data[extract.byte_start] << 8) | data[extract.byte_start + 1];
        } else {
            // Little endian: LSB first
            raw_value = (data[extract.byte_start + 1] << 8) | data[extract.byte_start];
        }
    } else {
        // Invalid byte length
        stats.extraction_errors++;
        return false;
    }
    
    // Handle signed values
    if (extract.flags & CAN_EXTRACT_FLAG_SIGNED) {
        if (extract.byte_length == 1) {
            // Convert to signed 8-bit
            int8_t signed_value = (int8_t)raw_value;
            raw_value = (uint32_t)signed_value;
        } else if (extract.byte_length == 2) {
            // Convert to signed 16-bit
            int16_t signed_value = (int16_t)raw_value;
            raw_value = (uint32_t)signed_value;
        }
    }
    
    // Apply scale factor
    *value = (float)raw_value * extract.scale_factor;
    
    return true;
}

bool CustomCanBusManager::validate_value(float value, const can_mapping_t& mapping) {
    const auto& validation = mapping.validation;
    
    // Check range
    if (value < validation.min_value || value > validation.max_value) {
        return false;
    }
    
    return true;
}

// =============================================================================
// MAPPING MANAGEMENT
// =============================================================================

int CustomCanBusManager::find_mapping_by_can_id(uint32_t can_id) {
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (mappings[i].basic.external_can_id == can_id) {
            return i;
        }
    }
    return -1;
}

bool CustomCanBusManager::has_mapping_for_can_id(uint32_t can_id) const {
    for (uint8_t i = 0; i < mapping_count; i++) {
        if (mappings[i].basic.external_can_id == can_id && mappings[i].enabled) {
            return true;
        }
    }
    return false;
}

bool CustomCanBusManager::is_mapping_valid(const can_mapping_t& mapping) {
    // Check basic parameters
    if (mapping.basic.external_can_id == 0 || mapping.basic.internal_msg_id == 0) {
        return false;
    }
    
    // Check extraction parameters
    if (mapping.extraction.byte_start > 7 || 
        mapping.extraction.byte_length == 0 || 
        mapping.extraction.byte_length > 2) {
        return false;
    }
    
    // Check that extraction doesn't exceed CAN message size
    if (mapping.extraction.byte_start + mapping.extraction.byte_length > 8) {
        return false;
    }
    
    // Check scale factor
    if (mapping.extraction.scale_factor == 0.0f) {
        return false;
    }
    
    // Check validation range
    if (mapping.validation.min_value > mapping.validation.max_value) {
        return false;
    }
    
    return true;
}

// =============================================================================
// STORAGE HELPERS
// =============================================================================

bool CustomCanBusManager::save_mapping_to_storage(uint8_t index, const can_mapping_t& mapping) {
    if (index >= MAX_MAPPINGS) {
        return false;
    }
    
    // Save basic mapping
    if (!g_storage_manager.save_data(CONFIG_EXTERNAL_CANBUS_MAPPING(index), 
                                   &mapping.basic, sizeof(mapping.basic))) {
        return false;
    }
    
    // Save extraction parameters
    if (!g_storage_manager.save_data(CONFIG_EXTERNAL_CANBUS_EXTRACTION(index), 
                                   &mapping.extraction, sizeof(mapping.extraction))) {
        return false;
    }
    
    // Save validation parameters
    if (!g_storage_manager.save_data(CONFIG_EXTERNAL_CANBUS_VALIDATION(index), 
                                   &mapping.validation, sizeof(mapping.validation))) {
        return false;
    }
    
    return true;
}

bool CustomCanBusManager::load_mapping_from_storage(uint8_t index, can_mapping_t* mapping) {
    if (index >= MAX_MAPPINGS || mapping == nullptr) {
        return false;
    }
    
    // Load basic mapping
    if (!g_storage_manager.load_data(CONFIG_EXTERNAL_CANBUS_MAPPING(index), 
                                   &mapping->basic, sizeof(mapping->basic))) {
        return false;
    }
    
    // Load extraction parameters
    if (!g_storage_manager.load_data(CONFIG_EXTERNAL_CANBUS_EXTRACTION(index), 
                                   &mapping->extraction, sizeof(mapping->extraction))) {
        return false;
    }
    
    // Load validation parameters
    if (!g_storage_manager.load_data(CONFIG_EXTERNAL_CANBUS_VALIDATION(index), 
                                   &mapping->validation, sizeof(mapping->validation))) {
        return false;
    }
    
    // Set enabled state to true by default
    mapping->enabled = true;
    
    return true;
}

// =============================================================================
// STATUS AND DIAGNOSTICS
// =============================================================================

void CustomCanBusManager::reset_statistics() {
    stats.messages_processed = 0;
    stats.messages_translated = 0;
    stats.validation_errors = 0;
    stats.extraction_errors = 0;
    stats.unknown_messages = 0;
}

void CustomCanBusManager::print_configuration() {
    #ifdef ARDUINO
    Serial.println("=== Custom CAN Bus Manager Configuration ===");
    Serial.print("Initialized: "); Serial.println(initialized ? "Yes" : "No");
    Serial.print("Mapping Count: "); Serial.println(mapping_count);
    Serial.println();
    
    for (uint8_t i = 0; i < mapping_count; i++) {
        Serial.print("Mapping ["); Serial.print(i); Serial.println("]:");
        debug_print_mapping(mappings[i]);
        Serial.println();
    }
    Serial.println("============================================");
    #endif
}

void CustomCanBusManager::print_statistics() {
    #ifdef ARDUINO
    Serial.println("=== Custom CAN Bus Manager Statistics ===");
    Serial.print("Messages Processed: "); Serial.println(stats.messages_processed);
    Serial.print("Messages Translated: "); Serial.println(stats.messages_translated);
    Serial.print("Validation Errors: "); Serial.println(stats.validation_errors);
    Serial.print("Extraction Errors: "); Serial.println(stats.extraction_errors);
    Serial.print("Unknown Messages: "); Serial.println(stats.unknown_messages);
    Serial.println("=========================================");
    #endif
}

// =============================================================================
// DEBUG HELPERS
// =============================================================================

void CustomCanBusManager::debug_print(const char* message) {
    #ifdef ARDUINO
    Serial.println(message);
    #else
    printf("%s\n", message);
    #endif
}

void CustomCanBusManager::debug_print_mapping(const can_mapping_t& mapping) {
    #ifdef ARDUINO
    Serial.print("  External CAN ID: 0x"); Serial.println(mapping.basic.external_can_id, HEX);
    Serial.print("  Internal MSG ID: 0x"); Serial.println(mapping.basic.internal_msg_id, HEX);
    Serial.print("  Byte Start: "); Serial.println(mapping.extraction.byte_start);
    Serial.print("  Byte Length: "); Serial.println(mapping.extraction.byte_length);
    Serial.print("  Big Endian: "); Serial.println((mapping.extraction.flags & CAN_EXTRACT_FLAG_BIG_ENDIAN) ? "Yes" : "No");
    Serial.print("  Scale Factor: "); Serial.println(mapping.extraction.scale_factor);
    Serial.print("  Min Value: "); Serial.println(mapping.validation.min_value);
    Serial.print("  Max Value: "); Serial.println(mapping.validation.max_value);
    Serial.print("  Enabled: "); Serial.println(mapping.enabled ? "Yes" : "No");
    #endif
}

// =============================================================================
// TESTING INTERFACE
// =============================================================================

void CustomCanBusManager::simulate_can_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    #ifdef ARDUINO
    Serial.print("DEBUG: simulate_can_message called for CAN ID 0x");
    Serial.print(can_id, HEX);
    Serial.print(" with ");
    Serial.print(length);
    Serial.println(" bytes");
    #else
    printf("DEBUG: simulate_can_message called for CAN ID 0x%08X with %d bytes\n", can_id, length);
    #endif
    
    handle_can_message(can_id, data, length);
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

can_mapping_t create_can_mapping(
    uint32_t external_can_id,
    uint32_t internal_msg_id,
    uint8_t byte_start,
    uint8_t byte_length,
    bool is_big_endian,
    float scale_factor,
    float min_value,
    float max_value
) {
    can_mapping_t mapping = {};
    
    // Basic mapping
    mapping.basic.external_can_id = external_can_id;
    mapping.basic.internal_msg_id = internal_msg_id;
    
    // Extraction parameters
    mapping.extraction.byte_start = byte_start;
    mapping.extraction.byte_length = byte_length;
    mapping.extraction.flags = is_big_endian ? CAN_EXTRACT_FLAG_BIG_ENDIAN : 0;
    mapping.extraction.scale_factor = scale_factor;
    
    // Validation parameters
    mapping.validation.min_value = min_value;
    mapping.validation.max_value = max_value;
    
    // Enable by default
    mapping.enabled = true;
    
    return mapping;
}

can_mapping_t create_simple_can_mapping(
    uint32_t external_can_id,
    uint32_t internal_msg_id,
    float scale_factor
) {
    return create_can_mapping(
        external_can_id,
        internal_msg_id,
        0,          // Start at byte 0
        2,          // Default to 2 bytes
        false,      // Little endian
        scale_factor,
        0.0f,       // Min value
        65535.0f    // Max value (for 16-bit)
    );
} 