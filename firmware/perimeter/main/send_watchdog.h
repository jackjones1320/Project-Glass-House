/**
 * @file send_watchdog.h
 * @brief Send-rate-gauge watchdog — detects stuck-data state and recovers.
 *
 * Polls `stream_sender_get_send_ok()` at 1 Hz and escalates recovery when the
 * counter is flat for longer than a per-node-staggered threshold:
 *   1. First soft recovery (esp_wifi_disconnect + esp_wifi_connect) after ~8-9s
 *   2. Second soft recovery after another ~8-9s if still flat
 *   3. Hard reboot (esp_restart via a deferred worker task) after two soft failures
 *
 * A symptom-based design (monitor actual data flow) is used so that
 * ESP-IDF GH-11615 (DISCONNECTED-event suppression on stuck states) and
 * false-negatives from esp_wifi_sta_get_ap_info() do not cause false-fire or
 * missed-fire.
 *
 * Target: cap observable silence at <=10 s on first-attempt recovery, <=30 s
 * if both soft attempts fail and a full reboot is required.
 */
#pragma once

#include <stdint.h>

/**
 * Start the send-rate watchdog. Must be called AFTER stream_sender_init_with()
 * and AFTER wifi_init_sta() have completed. Safe to call unconditionally —
 * the watchdog arms itself only after observing at least one successful send,
 * so boot-time disconnect windows (e.g., initial AP association taking >8s)
 * do not cause false-fire.
 *
 * @param node_id Perimeter node id (1-4). Used to stagger the stall threshold
 *                across nodes (8s / 9s / 8s / 9s) to prevent simultaneous
 *                reassociation storm when all 4 nodes experience correlated
 *                disconnects (e.g., coord reboot).
 */
void send_watchdog_start(uint8_t node_id);
