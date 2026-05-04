// firmware/perimeter/main/link_reporter.h
// GlassHouse v2 — Per-link variance computation and summary reporting

#ifndef LINK_REPORTER_H
#define LINK_REPORTER_H

#include <stdint.h>

#define LINK_MAX_PEERS     4
#define LINK_WINDOW_SIZE   20  // samples per variance window

/**
 * Record a CSI amplitude sample for a specific peer node.
 * Called from CSI callback after MAC filtering.
 * Thread-safe (uses portMUX critical sections).
 */
void link_reporter_record(uint8_t peer_node_id, float amplitude);

/**
 * Start the periodic reporter timer.
 * Sends 10-byte link summary packets every interval_ms to the coordinator.
 */
void link_reporter_start(uint8_t my_node_id, const char *target_ip,
                         uint16_t target_port, uint32_t interval_ms);

#endif
