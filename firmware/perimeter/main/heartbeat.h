// firmware/perimeter/main/heartbeat.h
// GlassHouse v2 — Periodic UDP heartbeat ping to coordinator

#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdint.h>

/**
 * Start the heartbeat timer.
 * Sends a 1-byte UDP ping (0xAA) to the coordinator every interval_ms.
 */
void heartbeat_start(const char *target_ip, uint16_t target_port, uint32_t interval_ms);

#endif
