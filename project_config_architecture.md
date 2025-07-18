# Project Configuration Manager Architecture

## Overview

The Project Configuration Manager provides a unified system for managing ECU configuration data (fuel maps, ignition timing, sensor calibrations, etc.) while maintaining compatibility with the existing message bus architecture and CAN-based communication system.

## Key Architectural Decisions

### 1. **Separation of Hardware vs Project Configuration**

```
Hardware Config (ecu_config)     Project Config (project_config)
├── Pin assignments             ├── Engine parameters
├── I2C device addresses        ├── Fuel maps
├── SPI configurations          ├── Ignition timing
├── Hardware-specific settings  ├── Trigger wheel config
└── ECU type definitions        ├── Thermistor tables
                               ├── CAN message definitions
                               ├── Sensor calibrations
                               └── Safety limits
```

### 2. **File Format: JSON for Human Readability**

Project files are stored as human-readable JSON with structured data:

```json
{
  "project_info": {
    "name": "1993 Mustang 5.0L",
    "version": "1.0",
    "created": "2024-01-15",
    "last_modified": "2024-01-20"
  },
  "engine": {
    "displacement_cc": 5000,
    "cylinders": 8,
    "firing_order": "1-5-4-2-6-3-7-8"
  },
  "trigger": {
    "wheel_type": "36-1",
    "missing_teeth": 1,
    "cam_sync": true
  },
  "fuel_map": {
    "rpm_axis": [800, 1200, 1600, 2000, 2400, 3000, 3600, 4200, 4800, 5400, 6000, 6600, 7200],
    "load_axis": [20, 30, 40, 50, 60, 70, 80, 90, 100],
    "values": [
      [10.5, 11.2, 12.1, 13.0, 14.2, 15.1, 16.0, 16.8, 17.5],
      [11.8, 12.5, 13.2, 14.1, 15.3, 16.2, 17.1, 17.9, 18.6],
      [12.2, 13.0, 13.8, 14.7, 16.0, 16.9, 17.8, 18.6, 19.3]
    ]
  },
  "ignition_map": {
    "rpm_axis": [800, 1200, 1600, 2000, 2400, 3000, 3600, 4200, 4800, 5400, 6000, 6600, 7200],
    "load_axis": [20, 30, 40, 50, 60, 70, 80, 90, 100],
    "values": [
      [15.0, 18.0, 22.0, 26.0, 30.0, 32.0, 34.0, 36.0, 38.0],
      [12.0, 15.0, 18.0, 22.0, 26.0, 28.0, 30.0, 32.0, 34.0],
      [10.0, 12.0, 15.0, 18.0, 22.0, 24.0, 26.0, 28.0, 30.0]
    ]
  }
}
```

### 3. **Hybrid Storage Architecture**

**Project File (Human-Readable)**
- Complete structured configuration in JSON format
- Saved to persistent storage (QSPI flash)
- Used for backup, sharing, and initial loading

**Key-Value Store (Runtime)**
- Individual configuration items stored as key-value pairs
- Enables granular updates and efficient storage
- Examples: `"fuel_map.5.10"` = `14.7f`

**In-Memory Configuration**
- Complete project configuration held in memory
- Provides fast access for modules
- Acts as authoritative source during runtime

## Boot Sequence

### Phase 1: System Initialization
```cpp
1. Storage Manager init (loads existing key-value pairs)
2. Message Bus init  
3. Modules init + subscribe to messages (but don't configure yet)
4. Project Config Manager init
```

### Phase 2: Configuration Resolution
```cpp
5. Project Config Manager:
   a. Load project file (if exists)
   b. Fill gaps with key-value store data
   c. Fill remaining gaps with defaults
   d. Create complete in-memory project config
```

### Phase 3: System Configuration
```cpp
6. Emit ALL configuration values to message bus
7. Modules receive messages and configure themselves
8. Save complete project file to persistent storage
```

## Configuration Resolution Logic

```cpp
class ProjectConfigManager {
    ProjectConfiguration resolveConfiguration() {
        ProjectConfiguration config;
        
        // 1. Start with defaults
        config = getDefaultConfiguration();
        
        // 2. Override with project file (if exists)
        if (projectFileExists()) {
            ProjectConfiguration file_config = loadProjectFile();
            config.merge(file_config);
        }
        
        // 3. Override with key-value store (individual tweaks)
        applyKeyValueOverrides(config);
        
        return config;
    }
};
```

## Message Bus Integration

### Unified Message Bus Architecture

The system maintains a single message bus that handles both:
- **Real-time sensor data** (MSG_ENGINE_RPM, MSG_VEHICLE_SPEED, etc.)
- **Configuration updates** (MSG_CONFIG_FUEL_MAP_CELL, MSG_CONFIG_IGNITION_MAP_CELL, etc.)

This ensures consistency across:
- Internal module communication
- Serial client communication  
- CAN bus node communication
- External ECU communication

### Configuration Message Types

#### **Message ID Space Planning**

To avoid ID exhaustion and enable efficient subscriptions, configuration messages are grouped by system:

```cpp
// ID Range Allocation (Standard 11-bit CAN: 0x000-0x7FF)
#define MSG_SENSOR_BASE          0x010   // 0x010-0x0FF (240 IDs) - Real-time sensor data
#define MSG_ACTUATOR_BASE        0x100   // 0x100-0x1FF (256 IDs) - Actuator control  
#define MSG_STATUS_BASE          0x200   // 0x200-0x2FF (256 IDs) - System status/diagnostics
#define MSG_EXTERNAL_BASE        0x300   // 0x300-0x3FF (256 IDs) - External communications
#define MSG_STORAGE_BASE         0x600   // 0x600-0x6FF (256 IDs) - Storage system
#define MSG_CONFIG_BASE          0x700   // 0x700-0x7FF (256 IDs) - Configuration system

// Configuration System Breakdown
// Fuel System Messages
#define MSG_CONFIG_FUEL_MAP_CELL         0x700
#define MSG_CONFIG_FUEL_TRIM_CELL        0x701  
#define MSG_CONFIG_FUEL_SCALAR           0x702
#define MSG_CONFIG_FUEL_STRING           0x703
// 0x704-0x70F reserved for future fuel system messages

// Ignition System Messages  
#define MSG_CONFIG_IGNITION_MAP_CELL     0x710
#define MSG_CONFIG_IGNITION_SCALAR       0x711
#define MSG_CONFIG_DWELL_MAP_CELL        0x712
// 0x713-0x71F reserved for future ignition system messages

// Boost Control Messages
#define MSG_CONFIG_BOOST_MAP_CELL        0x720
#define MSG_CONFIG_BOOST_SCALAR          0x721
// 0x722-0x72F reserved for future boost system messages

// Transmission System Messages
#define MSG_CONFIG_TRANSMISSION_MAP_CELL 0x730
#define MSG_CONFIG_TRANSMISSION_SCALAR   0x731
// 0x732-0x73F reserved for future transmission system messages

// General Configuration Messages
#define MSG_CONFIG_ENGINE_SCALAR         0x740
#define MSG_CONFIG_PROJECT_STRING        0x741
#define MSG_CONFIG_PROJECT_DATETIME      0x742
#define MSG_CONFIG_SENSOR_SCALAR         0x743
// 0x744-0x74F reserved for future general configuration

// Future Systems: 0x750-0x7FF (176 IDs available)
```

#### **Message Structure Definitions**

```cpp
// Universal map cell message (used by all map types)
typedef struct {
    uint16_t map_key;         // Hashed key (e.g., "fuel_map.5.10", "boost_map.3.7")
    uint8_t rpm_index;        // 0-19 (or other range)
    uint8_t load_index;       // 0-19 (or other range)  
    float value;              // Map value
} config_map_cell_msg_t;      // 8 bytes total

// Scalar configuration values
typedef struct {
    uint16_t config_key;      // Hashed key (e.g., "engine.displacement")
    float value;              // Scalar value
    uint16_t reserved;        // For future use
} config_scalar_msg_t;        // 8 bytes total

// String configuration values (max 6 characters)
typedef struct {
    uint16_t config_key;      // Hashed key (e.g., "project.name")
    char value[6];            // String value (null-terminated)
} config_string_msg_t;        // 8 bytes total

// DateTime configuration values (Unix timestamp)
typedef struct {
    uint16_t config_key;      // Hashed key (e.g., "project.created")
    uint32_t timestamp;       // Unix timestamp (seconds since epoch)
    uint16_t reserved;        // For future use
} config_datetime_msg_t;      // 8 bytes total
```

#### **Multiple Map Support**

The hashed key approach enables multiple maps of the same type:

```cpp
// Multiple fuel maps using same message ID
"fuel_map.5.10"        // Primary fuel map, cell [5][10]
"fuel_map_e85.5.10"    // E85 fuel map, cell [5][10]  
"fuel_map_cold.5.10"   // Cold start fuel map, cell [5][10]
"fuel_map_launch.5.10" // Launch fuel map, cell [5][10]

// All use MSG_CONFIG_FUEL_MAP_CELL - distinguished by hashed key
```

### Module Subscription Patterns

#### **System-Specific Subscriptions (Recommended)**

Modules subscribe only to relevant system messages, avoiding unnecessary processing:

```cpp
class FuelModule {
    void init() {
        // Subscribe only to fuel-related configuration messages
        msg_bus.subscribe(MSG_CONFIG_FUEL_MAP_CELL, this->onFuelMapUpdate);
        msg_bus.subscribe(MSG_CONFIG_FUEL_TRIM_CELL, this->onFuelTrimUpdate);
        msg_bus.subscribe(MSG_CONFIG_FUEL_SCALAR, this->onFuelScalarUpdate);
        
        // Subscribe to real-time sensor data
        msg_bus.subscribe(MSG_ENGINE_RPM, this->onRPMUpdate);
        msg_bus.subscribe(MSG_THROTTLE_POSITION, this->onThrottleUpdate);
        
        // Don't get ignition, boost, or transmission config updates
    }
    
    static void onFuelMapUpdate(const CANMessage* msg) {
        config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
        
        // Handle different fuel maps based on hashed key
        if (cell->map_key == crc16("fuel_map.5.10")) {
            primary_fuel_map[cell->rpm_index][cell->load_index] = cell->value;
        } else if (cell->map_key == crc16("fuel_map_e85.5.10")) {
            e85_fuel_map[cell->rpm_index][cell->load_index] = cell->value;
        } else if (cell->map_key == crc16("fuel_map_cold.5.10")) {
            cold_start_fuel_map[cell->rpm_index][cell->load_index] = cell->value;
        }
        
        recalculateFuelDelivery();
    }
    
    static void onFuelScalarUpdate(const CANMessage* msg) {
        config_scalar_msg_t* scalar = (config_scalar_msg_t*)msg->data;
        
        if (scalar->config_key == crc16("fuel.base_pressure")) {
            base_fuel_pressure = scalar->value;
        } else if (scalar->config_key == crc16("fuel.injector_flow")) {
            injector_flow_rate = scalar->value;
        }
        
        recalculateFuelDelivery();
    }
};
```

#### **Ignition Module Example**

```cpp
class IgnitionModule {
    void init() {
        // Subscribe only to ignition-related configuration messages
        msg_bus.subscribe(MSG_CONFIG_IGNITION_MAP_CELL, this->onIgnitionMapUpdate);
        msg_bus.subscribe(MSG_CONFIG_IGNITION_SCALAR, this->onIgnitionScalarUpdate);
        msg_bus.subscribe(MSG_CONFIG_DWELL_MAP_CELL, this->onDwellMapUpdate);
        
        // Subscribe to real-time sensor data
        msg_bus.subscribe(MSG_ENGINE_RPM, this->onRPMUpdate);
        msg_bus.subscribe(MSG_MANIFOLD_PRESSURE, this->onMAPUpdate);
        
        // Don't get fuel, boost, or transmission config updates
    }
    
    static void onIgnitionMapUpdate(const CANMessage* msg) {
        config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
        
        if (cell->map_key == crc16("ignition_map.5.10")) {
            ignition_timing_map[cell->rpm_index][cell->load_index] = cell->value;
        }
        
        updateIgnitionTiming();
    }
};
```

#### **Helper Functions for Convenience**

```cpp
// Helper functions for convenient map subscriptions
class ConfigMessageHelpers {
public:
    // Subscribe to all fuel map updates with callback
    static void subscribe_to_fuel_maps(MessageBus* bus, 
                                      void (*callback)(const char* map_name, uint8_t rpm_idx, uint8_t load_idx, float value)) {
        bus->subscribe(MSG_CONFIG_FUEL_MAP_CELL, [callback](const CANMessage* msg) {
            config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
            
            // Reverse hash to get map name (simplified example)
            const char* map_name = reverse_hash_lookup(cell->map_key);
            callback(map_name, cell->rpm_index, cell->load_index, cell->value);
        });
    }
    
    // Subscribe to all ignition map updates
    static void subscribe_to_ignition_maps(MessageBus* bus,
                                          void (*callback)(const char* map_name, uint8_t rpm_idx, uint8_t load_idx, float degrees)) {
        bus->subscribe(MSG_CONFIG_IGNITION_MAP_CELL, [callback](const CANMessage* msg) {
            config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
            
            const char* map_name = reverse_hash_lookup(cell->map_key);
            callback(map_name, cell->rpm_index, cell->load_index, cell->value);
        });
    }
    
    // Subscribe to scalar configuration updates
    static void subscribe_to_config_scalars(MessageBus* bus, uint16_t message_id,
                                           void (*callback)(const char* key, float value)) {
        bus->subscribe(message_id, [callback](const CANMessage* msg) {
            config_scalar_msg_t* scalar = (config_scalar_msg_t*)msg->data;
            
            const char* key = reverse_hash_lookup(scalar->config_key);
            callback(key, scalar->value);
        });
    }
};

// Simplified module usage
class FuelModule {
    void init() {
        ConfigMessageHelpers::subscribe_to_fuel_maps(&msg_bus, this->onFuelMapUpdate);
        ConfigMessageHelpers::subscribe_to_config_scalars(&msg_bus, MSG_CONFIG_FUEL_SCALAR, this->onFuelScalarUpdate);
    }
    
    static void onFuelMapUpdate(const char* map_name, uint8_t rpm_idx, uint8_t load_idx, float value) {
        if (strcmp(map_name, "fuel_map") == 0) {
            primary_fuel_map[rpm_idx][load_idx] = value;
        } else if (strcmp(map_name, "fuel_map_e85") == 0) {
            e85_fuel_map[rpm_idx][load_idx] = value;
        }
        recalculateFuelDelivery();
    }
    
    static void onFuelScalarUpdate(const char* key, float value) {
        if (strcmp(key, "fuel.base_pressure") == 0) {
            base_fuel_pressure = value;
        }
        recalculateFuelDelivery();
    }
};
```

## Runtime Configuration Updates

### Internal Updates
```cpp
// Project Config Manager receives different types of configuration updates
class ProjectConfigManager {
    void init() {
        // Subscribe to all configuration message types
        msg_bus.subscribe(MSG_CONFIG_FUEL_MAP_CELL, this->onFuelMapUpdate);
        msg_bus.subscribe(MSG_CONFIG_IGNITION_MAP_CELL, this->onIgnitionMapUpdate);
        msg_bus.subscribe(MSG_CONFIG_FUEL_SCALAR, this->onFuelScalarUpdate);
        msg_bus.subscribe(MSG_CONFIG_PROJECT_STRING, this->onProjectStringUpdate);
        msg_bus.subscribe(MSG_CONFIG_PROJECT_DATETIME, this->onProjectDateTimeUpdate);
        // ... etc for all configuration types
    }
    
    void onFuelMapUpdate(const CANMessage* msg) {
        config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
        
        // Update in-memory project config based on hashed key
        const char* key = reverse_hash_lookup(cell->map_key);
        
        // Update appropriate map
        if (strncmp(key, "fuel_map.", 9) == 0) {
            updateFuelMapCell(key, cell->rpm_index, cell->load_index, cell->value);
        } else if (strncmp(key, "fuel_map_e85.", 13) == 0) {
            updateE85FuelMapCell(key, cell->rpm_index, cell->load_index, cell->value);
        }
        
        // Save to key-value store via message bus
        char storage_key[32];
        sprintf(storage_key, "%s.%d.%d", key, cell->rpm_index, cell->load_index);
        
        storage_save_float_msg_t save_msg;
        save_msg.key_hash = crc16(storage_key);
        save_msg.value = cell->value;
        save_msg.priority = 0;  // Normal priority
        save_msg.sender_id = PROJECT_CONFIG_MODULE_ID;
        
        g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_msg, sizeof(save_msg));
    }
    
    void onFuelScalarUpdate(const CANMessage* msg) {
        config_scalar_msg_t* scalar = (config_scalar_msg_t*)msg->data;
        
        const char* key = reverse_hash_lookup(scalar->config_key);
        
        // Update in-memory configuration
        if (strcmp(key, "fuel.base_pressure") == 0) {
            project_config.fuel.base_pressure = scalar->value;
        } else if (strcmp(key, "fuel.injector_flow") == 0) {
            project_config.fuel.injector_flow = scalar->value;
        }
        
        // Save to key-value store via message bus
        storage_save_float_msg_t save_msg;
        save_msg.key_hash = crc16(key);
        save_msg.value = scalar->value;
        save_msg.priority = 0;  // Normal priority
        save_msg.sender_id = PROJECT_CONFIG_MODULE_ID;
        
        g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_msg, sizeof(save_msg));
    }
    
    void onProjectStringUpdate(const CANMessage* msg) {
        config_string_msg_t* str = (config_string_msg_t*)msg->data;
        
        const char* key = reverse_hash_lookup(str->config_key);
        
        if (strcmp(key, "project.name") == 0) {
            strncpy(project_config.project_info.name, str->value, sizeof(str->value));
        }
        
        // Save to key-value store via message bus
        // Note: String values saved as data rather than float
        storage_save_data_msg_t save_msg;
        save_msg.key_hash = crc16(key);
        save_msg.data_size = strlen(str->value) + 1;  // Include null terminator
        save_msg.sender_id = PROJECT_CONFIG_MODULE_ID;
        
        // For strings, we'd need to handle this differently
        // This is a simplified example - actual implementation may vary
        g_message_bus.publish(MSG_STORAGE_SAVE_DATA, &save_msg, sizeof(save_msg));
    }
    
    void onProjectDateTimeUpdate(const CANMessage* msg) {
        config_datetime_msg_t* dt = (config_datetime_msg_t*)msg->data;
        
        const char* key = reverse_hash_lookup(dt->config_key);
        
        if (strcmp(key, "project.modified") == 0) {
            project_config.project_info.last_modified = dt->timestamp;
        }
        
        // Save to key-value store via message bus
        // DateTime values are stored as uint32_t timestamps
        storage_save_data_msg_t save_msg;
        save_msg.key_hash = crc16(key);
        save_msg.data_size = sizeof(uint32_t);
        save_msg.sender_id = PROJECT_CONFIG_MODULE_ID;
        
        // For timestamps, we'd need to handle this as data
        // This is a simplified example - actual implementation may vary
        g_message_bus.publish(MSG_STORAGE_SAVE_DATA, &save_msg, sizeof(save_msg));
    }
};
```

### External Updates (Tuning Software)
```cpp
// Tuning software sends different types of configuration updates
class TuningSoftware {
    // Update fuel map cell
    void updateFuelMapCell(const char* map_name, int rpm, int load, float value) {
        config_map_cell_msg_t cell;
        
        // Create full key name
        char key[32];
        sprintf(key, "%s.%d.%d", map_name, rpm, load);
        
        cell.map_key = crc16(key);
        cell.rpm_index = rpm;
        cell.load_index = load;
        cell.value = value;
        
        sendCANMessage(MSG_CONFIG_FUEL_MAP_CELL, &cell, sizeof(cell));
    }
    
    // Update ignition map cell
    void updateIgnitionMapCell(const char* map_name, int rpm, int load, float degrees) {
        config_map_cell_msg_t cell;
        
        char key[32];
        sprintf(key, "%s.%d.%d", map_name, rpm, load);
        
        cell.map_key = crc16(key);
        cell.rpm_index = rpm;
        cell.load_index = load;
        cell.value = degrees;
        
        sendCANMessage(MSG_CONFIG_IGNITION_MAP_CELL, &cell, sizeof(cell));
    }
    
    // Update scalar configuration
    void updateScalarConfig(const char* key, float value) {
        config_scalar_msg_t scalar;
        scalar.config_key = crc16(key);
        scalar.value = value;
        scalar.reserved = 0;
        
        // Send to appropriate message ID based on key
        if (strncmp(key, "fuel.", 5) == 0) {
            sendCANMessage(MSG_CONFIG_FUEL_SCALAR, &scalar, sizeof(scalar));
        } else if (strncmp(key, "ignition.", 9) == 0) {
            sendCANMessage(MSG_CONFIG_IGNITION_SCALAR, &scalar, sizeof(scalar));
        } else if (strncmp(key, "engine.", 7) == 0) {
            sendCANMessage(MSG_CONFIG_ENGINE_SCALAR, &scalar, sizeof(scalar));
        }
    }
    
    // Update project metadata
    void updateProjectName(const char* name) {
        config_string_msg_t str;
        str.config_key = crc16("project.name");
        strncpy(str.value, name, sizeof(str.value));
        str.value[sizeof(str.value)-1] = '\0';  // Ensure null termination
        
        sendCANMessage(MSG_CONFIG_PROJECT_STRING, &str, sizeof(str));
    }
    
    void updateProjectModified() {
        config_datetime_msg_t dt;
        dt.config_key = crc16("project.modified");
        dt.timestamp = getCurrentUnixTimestamp();
        dt.reserved = 0;
        
        sendCANMessage(MSG_CONFIG_PROJECT_DATETIME, &dt, sizeof(dt));
    }
};

// Example usage:
// tuning_software.updateFuelMapCell("fuel_map", 5, 10, 14.7f);
// tuning_software.updateIgnitionMapCell("ignition_map", 5, 10, 25.0f);
// tuning_software.updateScalarConfig("fuel.base_pressure", 43.5f);
// tuning_software.updateProjectName("93GT");
```

## Benefits of This Architecture

### 1. **Unified Communication**
- Same message format for internal, serial, and CAN communication
- Consistent behavior across all transport methods
- External tuning software uses identical protocol

### 2. **CAN-Compatible**
- All configuration messages fit within 8-byte CAN frame limit
- Enables distributed ECU systems
- Network-friendly for multi-ECU installations

### 3. **Efficient Message ID Management**
- Strategic ID space allocation prevents exhaustion
- System-grouped messages enable targeted subscriptions
- Hashed keys support multiple maps with same message ID
- 176 message IDs still available for future expansion

### 4. **Performance Appropriate**
- Individual cell updates are efficient for rare configuration changes
- Real-time performance not impacted during racing
- Granular updates prevent large data transfers
- Modules only process relevant configuration messages

### 5. **Subscription Efficiency**
- Modules subscribe only to relevant system messages
- No wasted processing on unrelated configuration changes
- Clean separation between fuel, ignition, boost, and transmission systems
- Helper functions provide convenience without sacrificing efficiency

### 6. **Multiple Map Support**
- Same message ID supports multiple maps using hashed keys
- Example: `fuel_map`, `fuel_map_e85`, `fuel_map_cold` all use `MSG_CONFIG_FUEL_MAP_CELL`
- Enables complex configurations without ID proliferation
- Flexible naming conventions support diverse use cases

### 7. **Developer-Friendly**
- Helper functions make map subscriptions easy
- Clean separation of concerns
- Extensible for new map types
- Consistent patterns across all configuration types

### 8. **Robust Storage**
- Hybrid approach provides both structure and flexibility
- Graceful degradation with partial configurations
- Atomic updates prevent corruption
- String length constraints prevent protocol complexity

### 9. **Human-Readable**
- JSON project files can be manually edited
- Easy backup and sharing
- Clear documentation of all parameters
- Short string constraints encourage good UX design

## Complete Flow Example

Here's a complete example showing how a client updates a fuel map cell and how it flows through the entire system:

### 1. **Client Updates Configuration**
```cpp
// Tuning software updates fuel map cell [5][10] to 14.7
config_map_cell_msg_t cell;
cell.map_key = crc16("fuel_map.5.10");
cell.rpm_index = 5;
cell.load_index = 10;
cell.value = 14.7f;

// Send CAN message to ECU
sendCANMessage(MSG_CONFIG_FUEL_MAP_CELL, &cell, sizeof(cell));
```

### 2. **Message Bus Routes to Multiple Subscribers**
```cpp
// Message bus delivers to all subscribers of MSG_CONFIG_FUEL_MAP_CELL:
// - ProjectConfigManager (updates in-memory config + saves to storage)
// - FuelModule (updates live fuel map)
// - Any other modules that need fuel map updates
```

### 3. **ProjectConfigManager Updates Memory and Storage**
```cpp
void ProjectConfigManager::onFuelMapUpdate(const CANMessage* msg) {
    // Update in-memory configuration
    project_config.fuel_map[5][10] = 14.7f;
    
    // Save to storage via message bus
    storage_save_float_msg_t save_msg;
    save_msg.key_hash = crc16("fuel_map.5.10");
    save_msg.value = 14.7f;
    save_msg.priority = 0;
    save_msg.sender_id = PROJECT_CONFIG_MODULE_ID;
    
    g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &save_msg, sizeof(save_msg));
}
```

### 4. **FuelModule Updates Live Map**
```cpp
void FuelModule::onFuelMapUpdate(const CANMessage* msg) {
    config_map_cell_msg_t* cell = (config_map_cell_msg_t*)msg->data;
    
    // Update live fuel map immediately
    primary_fuel_map[cell->rpm_index][cell->load_index] = cell->value;
    
    // Recalculate fuel delivery for current operating conditions
    recalculateFuelDelivery();
}
```

### 5. **StorageManager Persists to Flash/EEPROM**
```cpp
void StorageManager::handle_save_float_message(const CANMessage* msg) {
    // Save to cache immediately
    save_to_cache(save_msg->key_hash, save_msg->value);
    
    // Write to persistent storage (or cache for later commit)
    backend->writeData(save_msg->key_hash, &save_msg->value, sizeof(float));
    
    // Send response back to confirm save
    send_save_response(save_msg->key_hash, true, save_msg->sender_id);
}
```

### 6. **Result Summary**
- **8-byte CAN frame** sent from tuning software
- **In-memory config** updated immediately
- **Live fuel map** updated for immediate use
- **Persistent storage** saves value to flash/EEPROM
- **Response confirmation** sent back to client

This flow demonstrates how the unified message bus enables:
- **Single message** → **Multiple subscribers** → **Coordinated updates**
- **Real-time updates** for immediate effect
- **Persistent storage** for configuration survival across reboots
- **Consistent behavior** across serial, CAN, and internal communication

## Future Enhancements

1. **Map Versioning** - Track configuration changes over time
2. **Compression** - Optimize storage for large maps
3. **Validation** - Real-time parameter range checking
4. **Templating** - Engine-specific default configurations
5. **Synchronization** - Multi-ECU configuration coordination

## Implementation Status

### Core Infrastructure
- [ ] Core ProjectConfigManager class
- [ ] JSON file format parser/serializer
- [ ] Configuration message type definitions
- [ ] Message bus helper functions (ConfigMessageHelpers)
- [ ] Hash key reverse lookup system
- [ ] Boot sequence implementation

### Message System
- [ ] Message ID range allocation in msg_definitions.h
- [ ] Configuration message structures
- [ ] System-specific message routing
- [ ] String length validation (6-char limit)
- [ ] Unix timestamp handling

### Module Integration
- [ ] Module subscription patterns
- [ ] Example implementations (FuelModule, IgnitionModule)
- [ ] Multiple map support per module
- [ ] Scalar configuration handling

### Storage Integration
- [ ] ProjectConfigManager + StorageManager integration via message bus
- [ ] Key-value store bridging using MSG_STORAGE_SAVE_FLOAT/DATA messages
- [ ] Configuration persistence using storage message responses
- [ ] Graceful degradation with partial configs

### External Interface
- [ ] Tuning software protocol examples
- [ ] Serial/CAN configuration update handling
- [ ] Configuration validation
- [ ] Bulk configuration operations

### Testing and Validation
- [ ] Unit tests for configuration manager
- [ ] Message routing tests
- [ ] Storage persistence tests
- [ ] Multiple map functionality tests
- [ ] External protocol tests

### Documentation
- [x] Architecture design document
- [ ] API documentation
- [ ] Integration guide for new modules
- [ ] Tuning software protocol specification 