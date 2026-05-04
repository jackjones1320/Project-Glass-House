/**
 * @file crash_counter.h
 * @brief NVS-backed persistent crash counter for post-mortem diagnosis.
 *
 * On every boot the counter is updated in NVS namespace "diag":
 *   - ESP_RST_POWERON       -> counter reset to 0
 *   - any other reset_reason -> counter incremented (saturating at UINT16_MAX)
 *
 * Telemetry publishes the cached value in the heartbeat-frame `crash_count`
 * field so the host-side decoder can surface it per heartbeat.
 */

#ifndef CRASH_COUNTER_H
#define CRASH_COUNTER_H

#include <stdint.h>

#include "esp_err.h"

/**
 * Initialize the persistent crash counter.
 *
 * Reads the stored counter from NVS namespace "diag", updates it based on
 * esp_reset_reason(), writes it back, and caches the new value for
 * crash_counter_get().
 *
 * Must be called AFTER nvs_flash_init(). Safe to call before WiFi init.
 * Idempotent — second call returns ESP_OK without re-reading NVS.
 *
 * @return ESP_OK on success, ESP_FAIL on NVS error (cached value stays 0).
 */
esp_err_t crash_counter_init(void);

/**
 * Return the cached crash counter value (crashes since last clean POWERON).
 * Returns 0 if crash_counter_init() has not been called or failed.
 */
uint16_t crash_counter_get(void);

#endif /* CRASH_COUNTER_H */
