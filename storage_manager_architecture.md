# Storage Manager Architecture

## Overview

The Storage Manager provides a unified, message-driven interface for persistent storage operations in the ECU system. It abstracts the underlying storage backend (SPI flash with FAT32 filesystem) and provides caching, message bus integration, and distributed storage access.

## Current Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────────┐
│                        Message Bus                              │
│  MSG_STORAGE_SAVE_FLOAT, MSG_STORAGE_LOAD_FLOAT, etc.          │
└─────────────────────────┬───────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────┐
│                   Storage Manager                               │
│  • Message handling                                             │
│  • LRU cache (20 entries, ~240 bytes)                         │
│  • Statistics tracking                                          │
│  • Async request/response                                       │
└─────────────────────────┬───────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────┐
│                SPI Flash Storage Backend                        │
│  • Hash-based directory structure                               │
│  • FAT32 filesystem on 16MB SPI flash                         │
│  • JSON support for structured data                            │
│  • Float array support for maps                                │
└─────────────────────────┬───────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────┐
│                   Physical Storage                              │
│  /keys/XX/XXXX.bin - Individual key-value pairs                │
│  /config/name.json - JSON configuration files                  │
│  /maps/name.map - Binary float arrays with headers             │
└─────────────────────────────────────────────────────────────────┘
```

### Key Features

#### 1. Message Bus Integration
- **Request/Response Model**: Async operations with confirmation messages
- **Network Transparent**: Same interface for local and remote storage access
- **Module Decoupling**: Modules don't need direct storage manager references

#### 2. Efficient Caching
- **LRU Cache**: 20 entries, ~12 bytes each (~240 bytes total)
- **Write-Through**: Immediate persistence for high-priority operations
- **Dirty Tracking**: Periodic commits of modified cache entries
- **Statistics**: Cache hit/miss ratios, disk I/O counters

#### 3. Hash-Based Storage
- **Current**: CRC16 keys for filename generation
- **Directory Structure**: `/keys/XX/XXXX.bin` (XX = upper 8 bits of hash)
- **Collision Resistance**: Currently limited by CRC16 (65,536 values)

#### 4. Multiple Data Types
- **Float Values**: Primary storage type for sensor data and map cells
- **JSON Data**: Human-readable configuration files
- **Float Arrays**: Efficient storage of fuel/ignition maps with size headers
- **Raw Data**: Binary data storage for custom formats

## Current Message Types

### Storage Operations (0x600-0x61F)
```cpp
#define MSG_STORAGE_SAVE_FLOAT      0x600   // Save float value to storage
#define MSG_STORAGE_LOAD_FLOAT      0x601   // Load float value from storage
#define MSG_STORAGE_SAVE_INT        0x602   // Save integer value to storage
#define MSG_STORAGE_LOAD_INT        0x603   // Load integer value from storage
#define MSG_STORAGE_DELETE_KEY      0x604   // Delete key from storage
#define MSG_STORAGE_COMMIT_CACHE    0x605   // Force commit of dirty cache entries
```

### Storage Responses (0x610-0x61F)
```cpp
#define MSG_STORAGE_SAVE_RESPONSE   0x610   // Response to save operation
#define MSG_STORAGE_LOAD_RESPONSE   0x611   // Response to load operation
#define MSG_STORAGE_ERROR           0x612   // Storage operation error
#define MSG_STORAGE_STATS           0x613   // Storage system statistics
```

### Current Message Structures (CRC16)
```cpp
// Storage save float message (8 bytes total)
typedef struct {
    uint16_t key_hash;          // CRC16 hash of key string (2 bytes)
    float value;                // Value to store (4 bytes)
    uint8_t priority;           // 0=cache, 1=immediate write (1 byte)
    uint8_t sender_id;          // Module ID that sent this request (1 byte)
} __attribute__((packed)) storage_save_float_msg_t;

// Storage load float message (8 bytes total)
typedef struct {
    uint16_t key_hash;          // CRC16 hash of key string (2 bytes)
    float default_value;        // Default value if key not found (4 bytes)
    uint8_t sender_id;          // Module ID that sent this request (1 byte)
    uint8_t request_id;         // For matching responses (1 byte)
} __attribute__((packed)) storage_load_float_msg_t;
```

## Distributed Storage Access

### Local Module Access
```cpp
// Module saves data via message bus
CANMessage msg;
MSG_PACK_STORAGE_SAVE_FLOAT(msg, "fuel_map.5.10", 14.7f, 0, FUEL_MODULE_ID);
g_message_bus.publish(MSG_STORAGE_SAVE_FLOAT, &msg, sizeof(msg));
```

### Remote Device Access
```cpp
// Remote Arduino via external serial/CAN
CANMessage msg;
MSG_PACK_STORAGE_SAVE_FLOAT(msg, "oil.temperature", 95.2f, 0, REMOTE_SENSOR_ID);
g_external_serial.send_to_device(MSG_STORAGE_SAVE_FLOAT, &msg, sizeof(msg), MAIN_ECU_ID);
```

### Tuning Software Access
```cpp
// Tuning software via CAN bus
void updateFuelMap(int rpm, int load, float value) {
    char key[32];
    snprintf(key, sizeof(key), "fuel_map.%d.%d", rpm, load);
    
    CANMessage msg;
    MSG_PACK_STORAGE_SAVE_FLOAT(msg, key, value, 1, TUNING_SOFTWARE_ID);
    send_can_message(MSG_STORAGE_SAVE_FLOAT, &msg);
}
```

## Performance Characteristics

### Memory Usage
- **Cache**: 20 entries × 12 bytes = 240 bytes
- **Code Size**: ~4KB compiled
- **Stack Usage**: Minimal (async message-driven)

### Latency
- **Cache Hit**: ~1-5 microseconds
- **Cache Miss**: ~1-10 milliseconds (SPI flash access)
- **Message Bus**: ~5-20 microseconds per message

### Throughput
- **Cache Operations**: ~200,000 ops/second
- **SPI Flash**: ~1,000-10,000 ops/second
- **Practical Limit**: ~100-1,000 storage ops/second

## Scale Challenges - CRC16 Limitations

### Current Collision Risk
- **Map Storage**: 20 maps × 30 × 30 = 18,000 map cells
- **Additional Config**: ~1,000 other configuration keys
- **Total Keys**: ~19,000 unique keys needed
- **CRC16 Space**: Only 65,536 possible values
- **Collision Probability**: Very high (>90% chance)

### Real-World Examples
```cpp
// These could hash to the same CRC16 value:
crc16("fuel_map.15.20") == crc16("ignition_map.3.7")  // Collision!
crc16("boost_map.10.15") == crc16("fuel_map_e85.5.22")  // Collision!
```

## CRC32 Migration Plan

### 1. Message Structure Changes

#### Option A: Pack Fields (Recommended)
```cpp
// New CRC32 storage message (8 bytes total)
typedef struct {
    uint32_t key_hash;          // CRC32 hash of key string (4 bytes)
    float value;                // Value to store (4 bytes)
    uint8_t flags;              // Packed: priority(1bit) + sender_id(7bits)
    uint8_t reserved;           // For future use
} __attribute__((packed)) storage_save_float_msg_t;

// Bit packing macros
#define PACK_FLAGS(priority, sender_id) (((priority) << 7) | ((sender_id) & 0x7F))
#define UNPACK_PRIORITY(flags) ((flags) >> 7)
#define UNPACK_SENDER_ID(flags) ((flags) & 0x7F)
```

**Trade-offs:**
- ✅ Fits in 8-byte CAN frame
- ✅ 4.3 billion hash values (no collisions)
- ⚠️ Sender ID limited to 127 (was 255)
- ⚠️ Priority becomes binary (was 0-255)

#### Option B: Simplified Structure
```cpp
// Minimal CRC32 storage message (8 bytes total)
typedef struct {
    uint32_t key_hash;          // CRC32 hash of key string (4 bytes)
    float value;                // Value to store (4 bytes)
} __attribute__((packed)) storage_save_float_msg_t;
```

**Trade-offs:**
- ✅ Clean, simple structure
- ✅ 4.3 billion hash values
- ❌ No priority control
- ❌ No sender tracking

### 2. Hash Function Implementation
```cpp
// Add CRC32 function alongside existing CRC16
inline uint32_t crc32(const char* data) {
    uint32_t crc = 0xFFFFFFFF;
    while (*data) {
        crc ^= (uint32_t)*data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;  // CRC32 polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}
```

### 3. Storage Backend Changes
```cpp
// Update signature to handle CRC32
void SPIFlashStorageBackend::keyHashToFilename(uint32_t key_hash, char* filename, size_t max_len) {
    // Use upper 16 bits for directory structure (better distribution)
    uint16_t dir_prefix = (key_hash >> 16) & 0xFFFF;
    snprintf(filename, max_len, "/keys/%04X/%08X.bin", dir_prefix, key_hash);
}

// Update method signatures
bool SPIFlashStorageBackend::writeData(uint32_t key_hash, const void* data, size_t size);
bool SPIFlashStorageBackend::readData(uint32_t key_hash, void* data, size_t size);
bool SPIFlashStorageBackend::deleteData(uint32_t key_hash);
bool SPIFlashStorageBackend::hasKey(uint32_t key_hash);
```

### 4. Cache Structure Changes
```cpp
// Update cache entry structure
struct CacheEntry {
    uint32_t key_hash;          // CRC32 hash (was uint16_t)
    float value;                // Cached value
    uint32_t timestamp;         // For LRU eviction
    bool dirty;                 // Needs write to flash
    uint8_t access_count;       // Usage frequency
};
```

## Migration TODOs

### Phase 1: Core Infrastructure
- [ ] Add CRC32 function to msg_definitions.h
- [ ] Update storage message structures (choose Option A or B)
- [ ] Create new MSG_PACK/MSG_UNPACK macros for CRC32
- [ ] Update storage_manager.h cache structure for CRC32

### Phase 2: Storage Backend
- [ ] Update SPIFlashStorageBackend::keyHashToFilename for CRC32
- [ ] Update writeData/readData/deleteData signatures for uint32_t
- [ ] Update hasKey method for CRC32
- [ ] Test directory structure with CRC32 hash distribution

### Phase 3: Storage Manager
- [ ] Update StorageManager message handlers for CRC32
- [ ] Update cache management for CRC32 keys
- [ ] Update statistics tracking
- [ ] Add CRC32 message validation

### Phase 4: Message Bus Integration
- [ ] Add new message IDs for CRC32 operations (or reuse existing)
- [ ] Update message handler registration
- [ ] Test async request/response with CRC32
- [ ] Update helper macros and utility functions

### Phase 5: Module Integration
- [ ] Update fuel_module to use CRC32 storage messages
- [ ] Update transmission_module to use CRC32 storage messages
- [ ] Update config_manager to use CRC32 storage messages
- [ ] Update any other modules using storage

### Phase 6: External Communications
- [ ] Update external_serial to forward CRC32 storage messages
- [ ] Update external_canbus to handle CRC32 storage messages
- [ ] Test distributed storage access with CRC32
- [ ] Update tuning software protocol documentation

### Phase 7: Testing and Validation
- [ ] Unit tests for CRC32 hash collision resistance
- [ ] Integration tests for message bus flow
- [ ] Performance tests for CRC32 calculation overhead
- [ ] Stress tests with 20,000+ keys
- [ ] Distributed storage tests with remote devices

### Phase 8: Migration Strategy
- [ ] Support both CRC16 and CRC32 simultaneously during transition
- [ ] Gradual module migration (fuel → transmission → config)
- [ ] Data migration tool for existing storage
- [ ] Deprecation timeline for CRC16 messages

## Performance Considerations

### CRC32 Overhead
- **Calculation Time**: ~1-2 microseconds per key on Teensy 4.1
- **Memory Overhead**: +2 bytes per cache entry, +2 bytes per message
- **Frequency**: Only during storage operations (~0.1% of total messages)

### Real-Time Impact
- **Real-Time Messages**: Unaffected (MSG_ENGINE_RPM, MSG_THROTTLE_POSITION, etc.)
- **Storage Messages**: Tiny overhead only during rare storage operations
- **Module Operations**: No impact on real-time control loops

### Scalability
- **Current**: 65,536 possible keys (CRC16)
- **After Migration**: 4.3 billion possible keys (CRC32)
- **Practical Limit**: Supports virtually unlimited maps and configuration

## Implementation Notes

### Key Naming Convention
```cpp
// Individual map cells
"fuel_map.{rpm_index}.{load_index}"           // "fuel_map.15.20"
"ignition_map.{rpm_index}.{load_index}"       // "ignition_map.15.20"
"boost_map.{rpm_index}.{load_index}"          // "boost_map.10.15"

// Multiple maps of same type
"fuel_map_e85.{rpm_index}.{load_index}"       // E85 fuel map
"fuel_map_cold.{rpm_index}.{load_index}"      // Cold start fuel map
"fuel_map_launch.{rpm_index}.{load_index}"    // Launch fuel map

// Scalar configuration values
"engine.displacement"                          // 5000.0f
"engine.cylinders"                             // 8.0f
"fuel.base_pressure"                           // 43.5f
"ignition.base_timing"                         // 10.0f

// Sensor calibrations
"sensor.coolant_temp.offset"                   // 0.0f
"sensor.throttle.min"                          // 0.5f
"sensor.throttle.max"                          // 4.5f
```

### Directory Structure After Migration
```
/keys/
├── 0001/
│   ├── 00012345.bin         # CRC32 hash files
│   └── 00016789.bin
├── 0002/
│   ├── 0002ABCD.bin
│   └── 0002CDEF.bin
├── ABCD/
│   ├── ABCD1234.bin
│   └── ABCD5678.bin
└── FFFF/
    ├── FFFF9876.bin
    └── FFFFFEDC.bin
```

## Future Enhancements

### Bulk Operations
- [ ] MSG_STORAGE_SAVE_BULK for efficient map uploads
- [ ] MSG_STORAGE_LOAD_BULK for efficient map downloads
- [ ] Streaming interface for large data transfers

### Advanced Features
- [ ] Key versioning for configuration history
- [ ] Atomic transactions for multi-key updates
- [ ] Compression for large maps
- [ ] Backup/restore functionality

### Monitoring and Diagnostics
- [ ] Storage health monitoring
- [ ] Wear leveling statistics
- [ ] Performance profiling tools
- [ ] Remote diagnostics interface

## References

- Current implementation: `storage_manager.cpp`, `storage_manager.h`
- Storage backend: `spi_flash_storage_backend.cpp`, `spi_flash_storage_backend.h`
- Message definitions: `msg_definitions.h`
- External communication: `external_serial.cpp`, `external_canbus.cpp`
- Test coverage: `tests/storage_manager/` 