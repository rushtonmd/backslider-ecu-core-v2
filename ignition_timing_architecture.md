# Ignition Timing Architecture
## High-Performance Engine Management for Backslider ECU

### Overview

This document outlines the architecture for implementing high-precision ignition and injection timing in the Backslider ECU while maintaining the elegant message-driven, decoupled design paradigm.

## System Architecture

### Three Core Subsystems

1. **Trigger Wheel Decoder** - Hardware interrupt-driven crank angle tracking
2. **Ignition Timing Controller** - Microsecond-precise coil firing
3. **Injection Timing Controller** - High-precision fuel injection

### Design Philosophy

- **Message Bus for Configuration**: All tuning parameters, timing maps, and non-critical control via message bus
- **Direct Control for Critical Timing**: Microsecond-precise ignition/injection bypasses message queue
- **Hybrid Approach**: Elegant decoupling + performance where it matters

---

## 1. Trigger Wheel Decoder

### Purpose
Convert physical crank sensor signals into precise angle and RPM data for timing calculations.

### Architecture
```
Crank Sensor → Hardware Interrupt → Angle Calculation → RPM Estimation
                                   ↓
                        Update Timing Predictions
```

### Implementation Strategy

#### Hardware Interrupt Handler
```c
void crank_trigger_isr(void) {
    uint32_t now_us = micros();
    
    // Calculate current crank angle (0-720°)
    current_angle = calculate_crank_angle();
    
    // Update RPM from interrupt timing
    uint32_t interval = now_us - last_trigger_time;
    current_rpm = calculate_rpm_from_interval(interval);
    last_trigger_time = now_us;
    
    // Update event queues based on new position
    update_timing_events();
}
```

#### Message Bus Integration
```c
// Publish crank data for other modules
g_message_bus.publishFloat(MSG_ENGINE_RPM, current_rpm, true);
g_message_bus.publishUint16(MSG_CRANK_POSITION, current_angle, true);

// Subscribe to decoder configuration
g_message_bus.subscribe(MSG_TRIGGER_WHEEL_CONFIG, configure_trigger_decoder);
```

### Configuration Parameters (via Message Bus)
- Trigger wheel tooth count
- Missing tooth configuration
- Sync pulse settings
- Angle offset calibration

---

## 2. Ignition Timing Controller

### Purpose
Fire ignition coils at precisely calculated crank angles with microsecond accuracy.

### Real-World Constraints
- **RPM Variation**: Big cams cause ±200 RPM variation during rotation at low RPM
- **Short Prediction Horizon**: Recalculate events every 60° to maintain accuracy
- **Overlapping Events**: Multiple coils dwelling simultaneously at high RPM

### Architecture

#### Three-Layer Control System

**Layer 1: Configuration (Message Bus)**
```c
// Engine module publishes timing maps via message bus
ignition_timing_map_t timing_map = {
    .base_timing = 15.0f,      // 15° BTDC base
    .rpm_advance = 0.5f,       // +0.5° per 1000 RPM
    .load_advance = 0.2f,      // +0.2° per 10% load
    .knock_retard = -2.0f      // -2° when knock detected
};
g_message_bus.publish(MSG_IGNITION_TIMING_MAP, &timing_map, sizeof(timing_map), false);
```

**Layer 2: Event Scheduling (Direct Control)**
```c
// Crank interrupt schedules ignition events
void update_ignition_events(void) {
    // Calculate next firing angle for each cylinder
    for (uint8_t cyl = 0; cyl < 8; cyl++) {
        float advance = calculate_timing_advance(cyl, current_rpm, current_load);
        uint32_t fire_angle = get_cylinder_tdc(cyl) - advance;
        
        if (angle_within_prediction_window(fire_angle)) {
            schedule_ignition_event(cyl, fire_angle);
        }
    }
}
```

**Layer 3: Hardware Execution (25μs Scheduler)**
```c
// High-frequency scheduler executes precise timing
void ignition_scheduler_isr(void) {
    uint32_t current_time = micros();
    
    // Execute all ignition events due now
    for (uint8_t i = 0; i < ignition_event_count; i++) {
        if (ignition_events[i].time_us <= current_time + 25) {
            execute_ignition_event(&ignition_events[i]);
            remove_ignition_event(i);
        }
    }
}
```

### Event Queue Implementation

#### Time-Slotted Array (Fastest Performance)
```c
// 32 slots × 25μs = 800μs lookahead
typedef struct {
    uint8_t cylinder;
    uint8_t action;    // COIL_ON, COIL_OFF
} ignition_event_t;

ignition_event_t ignition_slots[32][4];  // Max 4 events per 25μs slot
uint8_t slot_counts[32];
uint8_t current_slot = 0;

void schedule_ignition_event(uint8_t cylinder, uint32_t time_us) {
    uint8_t slot = (time_us / 25) % 32;
    ignition_slots[slot][slot_counts[slot]++] = {cylinder, COIL_ON};
    
    // Schedule coil OFF event (dwell time later)
    uint8_t off_slot = ((time_us + dwell_time_us) / 25) % 32;
    ignition_slots[off_slot][slot_counts[off_slot]++] = {cylinder, COIL_OFF};
}
```

### Message Bus Integration
```c
// Subscribe to ignition configuration
g_message_bus.subscribe(MSG_IGNITION_TIMING_MAP, update_ignition_config);
g_message_bus.subscribe(MSG_DWELL_TIME_CONFIG, update_dwell_config);
g_message_bus.subscribe(MSG_IGNITION_ENABLE, enable_ignition_system);

// Publish ignition diagnostics
g_message_bus.publishUint32(MSG_IGNITION_EVENTS_FIRED, ignition_event_count, false);
g_message_bus.publishFloat(MSG_ACTUAL_IGNITION_TIMING, last_timing_advance, false);
```

---

## 3. Injection Timing Controller

### Purpose
Control fuel injectors with precise timing and duration.

### Characteristics
- **Longer Prediction Horizon**: Less sensitive to RPM variation than ignition
- **Calculate Once Per Cycle**: Update all 8 injectors every 720°
- **Duration Critical**: Pulse width determines fuel quantity

### Architecture

#### Configuration (Message Bus)
```c
// Fuel module publishes injection parameters
injection_config_t fuel_config = {
    .base_pulse_width_ms = 3.2f,        // Base injection time
    .injector_flow_rate = 550,          // cc/min flow rate
    .fuel_pressure = 58,                // psi
    .dead_time_us = 800,                // Injector opening delay
    .timing_btdc = 320                  // Injection timing (320° BTDC)
};
g_message_bus.publish(MSG_INJECTION_CONFIG, &fuel_config, sizeof(fuel_config), false);
```

#### Event Scheduling
```c
void update_injection_events(void) {
    // Calculate injection timing for all cylinders
    for (uint8_t cyl = 0; cyl < 8; cyl++) {
        uint32_t injection_angle = get_cylinder_tdc(cyl) - fuel_config.timing_btdc;
        uint32_t pulse_width_us = calculate_pulse_width(cyl);
        
        schedule_injection_event(cyl, injection_angle, pulse_width_us);
    }
}
```

#### Shared 25μs Scheduler
```c
void injection_scheduler_isr(void) {
    // Same scheduler as ignition, different event types
    for (uint8_t i = 0; i < injection_event_count; i++) {
        if (injection_events[i].time_us <= current_time + 25) {
            execute_injection_event(&injection_events[i]);
        }
    }
}
```

### Message Bus Integration
```c
// Subscribe to fuel system configuration
g_message_bus.subscribe(MSG_INJECTION_CONFIG, update_injection_config);
g_message_bus.subscribe(MSG_FUEL_TRIM_MAP, update_fuel_trim);
g_message_bus.subscribe(MSG_LAMBDA_FEEDBACK, update_closed_loop_correction);

// Publish injection diagnostics
g_message_bus.publishFloat(MSG_ACTUAL_PULSE_WIDTH, last_pulse_width_ms, false);
g_message_bus.publishUint32(MSG_INJECTION_EVENTS_FIRED, injection_event_count, false);
```

---

## Integration Strategy

### Unified Event Scheduler

#### Single 25μs Timer for All Events
```c
void unified_timing_scheduler_isr(void) {
    uint32_t current_time = micros();
    uint8_t current_slot = (current_time / 25) % 32;
    
    // Execute ALL events in current time slot
    for (uint8_t i = 0; i < event_slot_counts[current_slot]; i++) {
        timing_event_t* event = &event_slots[current_slot][i];
        
        switch (event->type) {
            case IGNITION_COIL_ON:
                digitalWriteFast(ignition_pins[event->cylinder], HIGH);
                break;
            case IGNITION_COIL_OFF:
                digitalWriteFast(ignition_pins[event->cylinder], LOW);
                break;
            case INJECTOR_ON:
                digitalWriteFast(injector_pins[event->cylinder], HIGH);
                break;
            case INJECTOR_OFF:
                digitalWriteFast(injector_pins[event->cylinder], LOW);
                break;
        }
    }
    
    // Clear slot for reuse
    event_slot_counts[current_slot] = 0;
}
```

### Message Bus Architecture

#### Configuration Messages
```c
// Engine timing configuration
#define MSG_IGNITION_TIMING_MAP     0x600
#define MSG_INJECTION_CONFIG        0x601  
#define MSG_DWELL_TIME_CONFIG       0x602
#define MSG_TRIGGER_WHEEL_CONFIG    0x603

// Real-time data
#define MSG_ENGINE_RPM              0x610
#define MSG_CRANK_POSITION          0x611
#define MSG_ENGINE_LOAD             0x612
#define MSG_KNOCK_SENSOR            0x613

// Diagnostic data
#define MSG_IGNITION_EVENTS_FIRED   0x620
#define MSG_INJECTION_EVENTS_FIRED  0x621
#define MSG_TIMING_SCHEDULER_LOAD   0x622
```

### Performance Characteristics

#### Timing Precision
- **Ignition Timing**: ±12.5μs (25μs scheduler ÷ 2)
- **Injection Timing**: ±12.5μs 
- **Crank Position**: ±0.5° (depending on trigger wheel resolution)

#### CPU Load
- **25μs Scheduler**: ~5% CPU load (40kHz interrupt)
- **Crank Interrupt**: Variable, ~0.1% at 6000 RPM
- **Message Bus**: ~1% CPU load for configuration

#### Memory Usage
- **Event Queues**: ~2KB (32 slots × 64 bytes)
- **Timing Maps**: ~4KB (lookup tables)
- **Message Buffers**: ~1KB (existing message bus)

---

## Implementation Guidelines

### Phase 1: Basic Framework
1. Implement 25μs scheduler with time-slotted event queue
2. Add basic crank trigger interrupt handling
3. Create message bus interfaces for configuration
4. Test with single-cylinder timing

### Phase 2: Multi-Cylinder Integration
1. Add complete ignition timing calculation
2. Implement injection timing control
3. Add RPM variation compensation
4. Test with full 8-cylinder operation

### Phase 3: Performance Optimization
1. Optimize event queue performance
2. Add predictive timing algorithms
3. Implement closed-loop timing correction
4. Add comprehensive diagnostics

### Phase 4: Advanced Features
1. Knock detection integration
2. Variable cam timing support
3. Multi-spark ignition
4. Staged injection timing

---

## Real-World Considerations

### RPM Variation Compensation
- **Big Cam Engines**: ±200 RPM variation during rotation
- **Solution**: Recalculate ignition events every 60°
- **Trade-off**: Shorter prediction horizon vs accuracy

### High RPM Operation
- **Event Overlap**: Multiple coils dwelling simultaneously
- **Solution**: Time-slotted scheduler handles unlimited overlaps
- **Limit**: Teensy 4.1 can handle 10,000+ RPM

### Low RPM Stability
- **Challenge**: Long prediction horizons become inaccurate
- **Solution**: Adaptive prediction windows based on RPM
- **Benefit**: Maintains precision across entire RPM range

### Hardware Integration
- **Output Manager**: Provides hardware abstraction layer
- **Message Bus**: Maintains elegant decoupling
- **Direct Control**: Bypasses message queue for critical timing

---

## Conclusion

This architecture provides:
- **Microsecond precision** for ignition and injection timing
- **Elegant message-driven** configuration and monitoring
- **High-performance execution** via optimized event scheduler  
- **Real-world reliability** through adaptive algorithms
- **Scalable design** supporting future enhancements


The system maintains the beautiful decoupling of the message bus paradigm while achieving the performance necessary for high-RPM, precision engine control. 