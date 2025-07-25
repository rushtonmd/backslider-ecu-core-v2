// request_tracker.h
// Request tracking system for external communication channels
// Tracks pending parameter requests to enable proper response routing

#ifndef REQUEST_TRACKER_H
#define REQUEST_TRACKER_H

#include <stdint.h>
#include "msg_definitions.h"

// =============================================================================
// REQUEST TRACKING STRUCTURES
// =============================================================================

// Pending request structure
struct PendingRequest {
    uint8_t request_id;
    uint8_t source_channel;
    uint32_t param_id;
    uint32_t timestamp;
    bool active;
};

// =============================================================================
// REQUEST TRACKER CLASS
// =============================================================================

class RequestTracker {
public:
    // Configuration constants
    static const uint8_t MAX_PENDING = 16;
    static const uint32_t DEFAULT_TIMEOUT_MS = 5000; // 5 second timeout
    
    // Constructor
    RequestTracker();
    
    // Request management
    void add_request(uint8_t channel, uint32_t param_id);
    void remove_request(uint8_t request_id, uint8_t channel);
    void cleanup_timeouts(uint32_t timeout_ms = DEFAULT_TIMEOUT_MS);
    
    // Request ID generation
    uint8_t get_next_request_id();
    
    // Lookup methods
    bool is_pending_request(uint8_t request_id, uint8_t channel) const;
    uint32_t get_pending_param_id(uint8_t request_id, uint8_t channel) const;
    
    // Statistics
    uint8_t get_pending_count() const { return pending_count; }
    uint32_t get_timeout_count() const { return timeout_count; }
    void reset_statistics();
    
private:
    // Request storage
    PendingRequest pending[MAX_PENDING];
    uint8_t pending_count;
    uint8_t request_id_counter;
    
    // Statistics
    uint32_t timeout_count;
    uint32_t total_requests;
    
    // Helper methods
    void remove_request_by_index(uint8_t index);
    uint8_t find_request_index(uint8_t request_id, uint8_t channel) const;
};

#endif // REQUEST_TRACKER_H 