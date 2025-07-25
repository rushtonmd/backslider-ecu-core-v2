// request_tracker.cpp
// Request tracking system implementation for external communication channels

#include "request_tracker.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "tests/mock_arduino.h"
#endif

// =============================================================================
// CONSTRUCTOR
// =============================================================================

RequestTracker::RequestTracker() : 
    pending_count(0),
    request_id_counter(0),
    timeout_count(0),
    total_requests(0)
{
    // Initialize all pending requests as inactive
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
        pending[i].active = false;
    }
}

// =============================================================================
// REQUEST MANAGEMENT
// =============================================================================

void RequestTracker::add_request(uint8_t channel, uint32_t param_id) {
    if (pending_count >= MAX_PENDING) {
        // Remove oldest request to make room
        remove_request_by_index(0);
    }
    
    // Find first inactive slot
    uint8_t slot = 0;
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
        if (!pending[i].active) {
            slot = i;
            break;
        }
    }
    
    // Add new request
    pending[slot].request_id = get_next_request_id();
    pending[slot].source_channel = channel;
    pending[slot].param_id = param_id;
    pending[slot].timestamp = millis();
    pending[slot].active = true;
    
    pending_count++;
    total_requests++;
}

void RequestTracker::remove_request(uint8_t request_id, uint8_t channel) {
    uint8_t index = find_request_index(request_id, channel);
    if (index < MAX_PENDING) {
        remove_request_by_index(index);
    }
}

void RequestTracker::cleanup_timeouts(uint32_t timeout_ms) {
    uint32_t current_time = millis();
    
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
        if (pending[i].active) {
            if ((current_time - pending[i].timestamp) > timeout_ms) {
                // Request has timed out
                remove_request_by_index(i);
                timeout_count++;
            }
        }
    }
}

// =============================================================================
// REQUEST ID GENERATION
// =============================================================================

uint8_t RequestTracker::get_next_request_id() {
    request_id_counter++;
    if (request_id_counter == 0) {
        request_id_counter = 1; // Skip 0 to avoid confusion
    }
    return request_id_counter;
}

// =============================================================================
// LOOKUP METHODS
// =============================================================================

bool RequestTracker::is_pending_request(uint8_t request_id, uint8_t channel) const {
    uint8_t index = find_request_index(request_id, channel);
    return (index < MAX_PENDING);
}

uint32_t RequestTracker::get_pending_param_id(uint8_t request_id, uint8_t channel) const {
    uint8_t index = find_request_index(request_id, channel);
    if (index < MAX_PENDING) {
        return pending[index].param_id;
    }
    return 0; // Invalid parameter ID
}

// =============================================================================
// STATISTICS
// =============================================================================

void RequestTracker::reset_statistics() {
    timeout_count = 0;
    total_requests = 0;
}

// =============================================================================
// PRIVATE HELPER METHODS
// =============================================================================

void RequestTracker::remove_request_by_index(uint8_t index) {
    if (index >= MAX_PENDING || !pending[index].active) {
        return;
    }
    
    // Mark as inactive
    pending[index].active = false;
    pending_count--;
    
    // Compact the array by moving active requests to the front
    for (uint8_t i = index; i < MAX_PENDING - 1; i++) {
        if (pending[i + 1].active) {
            pending[i] = pending[i + 1];
            pending[i + 1].active = false;
        }
    }
}

uint8_t RequestTracker::find_request_index(uint8_t request_id, uint8_t channel) const {
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
        if (pending[i].active && 
            pending[i].request_id == request_id && 
            pending[i].source_channel == channel) {
            return i;
        }
    }
    return MAX_PENDING; // Not found
} 