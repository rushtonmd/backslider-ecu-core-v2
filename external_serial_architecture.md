# External Serial Communication Module Architecture

## Overview

The External Serial Communication module provides **daisy-chained serial communication** between multiple ECU units while maintaining compatibility with the internal CAN message bus format. This enables distributed ECU systems where multiple units can communicate seamlessly over a single serial connection.

## Architecture

### System Topology

```
Primary ECU ←→ Secondary ECU ←→ Logging ECU ←→ Dashboard ECU
  (ID: 1)       (ID: 2)         (ID: 3)        (ID: 4)
     ↑             ↑               ↑              ↑
 Internal        Internal       Internal       Internal
Message Bus     Message Bus    Message Bus    Message Bus
```

### Key Design Principles

- **Transparent Integration**: Uses same `CANMessage` format as internal message bus
- **Daisy-Chain Topology**: Multiple devices connected in series
- **Bidirectional Communication**: Messages flow both directions through the chain
- **Automatic Routing**: Packets automatically forwarded to destination devices
- **Error Detection**: CRC16 checksums ensure data integrity
- **Rate Limiting**: Prevents bus saturation with configurable message rates

## Packet Format

```
┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐
│ Sync Byte   │ Source ID   │ Dest ID     │ Packet Type │ CAN Message │ Checksum    │
│ (0xAA)      │ (1-254)     │ (1-254/0xFF)│ (1 byte)    │ (variable)  │ (CRC16)     │
└─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘
```

### Device IDs
- `0x01-0xFE`: Individual device addresses
- `0xFF`: Broadcast to all devices  
- `0x00`: Reserved/invalid

### Packet Types
- `PACKET_TYPE_NORMAL (0x01)`: Standard message
- `PACKET_TYPE_ACK (0x02)`: Acknowledgment
- `PACKET_TYPE_NACK (0x03)`: Negative acknowledgment
- `PACKET_TYPE_PING (0x04)`: Ping request
- `PACKET_TYPE_PONG (0x05)`: Ping response
- `PACKET_TYPE_CONFIG (0x06)`: Configuration message
- `PACKET_TYPE_ERROR (0x07)`: Error notification

## Core Features

### 1. Message Forwarding System

**Automatic Rule-Based Forwarding:**
- Configure which internal messages to forward to which devices
- Support for broadcast messages (send to all devices)
- Rate limiting to prevent bus congestion
- Up to 32 forwarding rules per device

**Example:**
```cpp
// Forward engine RPM to Secondary ECU every 50ms
g_external_serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU, 50);

// Broadcast system status to all devices
g_external_serial.subscribe_for_forwarding(MSG_ENGINE_STATUS, DEVICE_ID_BROADCAST);
```

### 2. Daisy-Chain Packet Routing

**Intelligent Packet Forwarding:**
- Devices automatically forward packets not addressed to them
- Source ID tracking prevents infinite loops
- Broadcast packets processed by all devices in chain
- Transparent routing - no configuration required

### 3. Error Detection & Recovery

**Robust Error Handling:**
- CRC16 checksums on all packets
- Timeout detection for incomplete packets
- Buffer overflow protection
- Comprehensive error statistics

### 4. High-Speed Communication

**Optimized for Performance:**
- Default 2 Mbps serial communication
- Circular buffer for efficient data handling
- State machine parser for reliable packet extraction
- Minimal processing overhead

## Usage Examples

### Basic Initialization

```cpp
// Initialize as Primary ECU on Serial2 at 2 Mbps
g_external_serial.init(DEVICE_ID_PRIMARY_ECU, &Serial2, 2000000);

// Set up forwarding rules
g_external_serial.subscribe_for_forwarding(MSG_ENGINE_RPM, DEVICE_ID_SECONDARY_ECU, 50);
g_external_serial.subscribe_for_forwarding(MSG_ENGINE_LOAD, DEVICE_ID_LOGGING_ECU, 100);

// In main loop
void loop() {
    g_external_serial.update();
    // ... other code
}
```

### Advanced Message Routing

```cpp
// Forward sensor data to logging ECU
FORWARD_TO_DEVICE(MSG_COOLANT_TEMP, DEVICE_ID_LOGGING_ECU);
FORWARD_TO_DEVICE(MSG_INTAKE_PRESSURE, DEVICE_ID_LOGGING_ECU);

// Broadcast critical alerts
FORWARD_BROADCAST(MSG_ENGINE_FAULT);
FORWARD_BROADCAST(MSG_OVERHEAT_WARNING);

// Send direct message to specific device
float engine_load = 75.5;
SEND_TO_DEVICE(MSG_ENGINE_LOAD, &engine_load, sizeof(float), DEVICE_ID_DASHBOARD);
```

### Device Health Monitoring

```cpp
// Check if device is responsive
if (g_external_serial.ping_device(DEVICE_ID_SECONDARY_ECU)) {
    Serial.println("Secondary ECU is online");
}

// Monitor communication statistics
uint32_t errors = g_external_serial.get_checksum_errors();
uint32_t sent = g_external_serial.get_packets_sent();
uint32_t received = g_external_serial.get_packets_received();
```

## Technical Implementation

### State Machine Parser

The module uses a robust state machine to parse incoming serial data:

```cpp
typedef enum {
    PARSE_STATE_SYNC,           // Looking for sync byte (0xAA)
    PARSE_STATE_SOURCE,         // Reading source device ID
    PARSE_STATE_DEST,           // Reading destination device ID
    PARSE_STATE_TYPE,           // Reading packet type
    PARSE_STATE_CAN_MSG,        // Reading CAN message payload
    PARSE_STATE_CHECKSUM_HIGH,  // Reading checksum high byte
    PARSE_STATE_CHECKSUM_LOW    // Reading checksum low byte
} parse_state_t;
```

### Circular Buffer Management

**Efficient Data Handling:**
- 1KB receive buffer with head/tail pointers
- Overflow protection and detection
- Lock-free single-producer/single-consumer design

### CRC16 Error Detection

**Data Integrity:**
- Industry-standard CRC16 algorithm
- Lookup table for fast calculation
- Covers entire packet except checksum field

## Integration with Message Bus

### Seamless Message Flow

**Outbound (Internal → Serial):**
1. Module subscribes to internal message bus
2. Forwarding rules determine destination device
3. CANMessage wrapped in SerialPacket with addressing
4. Packet transmitted with error detection

**Inbound (Serial → Internal):**
1. Serial data parsed into packets
2. Checksum validation ensures integrity
3. Packets for this device forwarded to internal bus
4. Other packets forwarded along chain

### Configuration Management

```cpp
// Forwarding rule structure
typedef struct {
    uint32_t msg_id;            // Which message ID to forward
    uint8_t  dest_device_id;    // Where to send it
    uint32_t last_sent_ms;      // Last transmission time
    uint32_t rate_limit_ms;     // Minimum time between sends
    bool     enabled;           // Rule is active
} forwarding_rule_t;
```

## Performance Characteristics

### Throughput
- **Serial Rate**: Up to 2 Mbps (configurable)
- **Packet Overhead**: 7 bytes per CAN message
- **Theoretical Max**: ~180 full CAN messages/second at 2 Mbps

### Latency
- **Parsing Latency**: < 1ms for typical packets
- **Forwarding Delay**: Minimal buffering, immediate forward
- **End-to-End**: < 5ms for 4-device chain

### Memory Usage
- **RAM**: ~1.5KB per instance (buffer + rules)
- **Code**: ~8KB compiled size
- **Rules**: 32 forwarding rules max per device

## Error Handling & Diagnostics

### Comprehensive Statistics

```cpp
// Available diagnostic information
uint32_t packets_sent = g_external_serial.get_packets_sent();
uint32_t packets_received = g_external_serial.get_packets_received();
uint32_t packets_forwarded = g_external_serial.get_packets_forwarded();
uint32_t checksum_errors = g_external_serial.get_checksum_errors();
uint32_t timeout_errors = g_external_serial.get_timeout_errors();
uint32_t buffer_overflows = g_external_serial.get_buffer_overflows();
```

### Error Recovery

**Automatic Recovery:**
- Parser automatically resynchronizes on errors
- Timeout detection prevents hanging on incomplete packets
- Buffer overflow protection maintains system stability
- Statistics tracking for debugging and monitoring

## Hardware Requirements

### Recommended Setup
- **MCU**: Teensy 4.1 or similar ARM Cortex-M7
- **Serial**: Hardware UART (Serial1, Serial2, etc.)
- **Wiring**: Daisy-chain TX→RX, RX→TX between devices
- **Termination**: Proper signal termination for long chains

### Electrical Considerations
- **Signal Levels**: 3.3V UART compatible
- **Distance**: Up to 10 meters with proper cables
- **EMI**: Twisted pair or shielded cables recommended
- **Power**: Separate power for each device

## Future Enhancements

### Planned Features
- **Acknowledgment System**: Reliable message delivery
- **Device Discovery**: Automatic network topology detection
- **Message Prioritization**: QoS for critical messages
- **Encryption**: Secure communication between devices
- **Mesh Networking**: Multiple path routing capability

### Performance Optimizations
- **DMA Integration**: Zero-copy packet handling
- **Compression**: Reduce bandwidth usage
- **Adaptive Rates**: Dynamic baud rate adjustment
- **Smart Buffering**: Predictive message caching

## Conclusion

The External Serial Communication module provides a robust, scalable solution for distributed ECU systems. Its daisy-chain architecture, automatic routing, and seamless integration with the internal message bus make it ideal for complex automotive applications requiring multiple coordinated processing units.

The module's design emphasizes reliability, performance, and ease of use while maintaining compatibility with existing CAN-based message systems. Its comprehensive error handling and diagnostic capabilities ensure robust operation in challenging automotive environments. 