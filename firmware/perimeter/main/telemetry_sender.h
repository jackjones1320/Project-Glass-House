/**
 * @file telemetry_sender.h
 * @brief Structured telemetry frame (magic 0xC5110009) emitter.
 *
 * A dedicated FreeRTOS task emits a 48-byte little-endian telemetry packet
 * once per second via stream_sender_send(). Absence of telemetry at the host
 * during an outage is itself the diagnostic signal.
 *
 * Wire format (matches frame_decoder.py):
 *   struct '<IBBHIBBHIIIIIIII' (48 bytes, little-endian)
 *   magic(4) node_id(1) reset_reason(1) crash_count(2)
 *   uptime_s(4) wifi_connected(1) _pad1(1) free_heap_kb(2)
 *   s_cb_count(4) s_send_ok(4) s_send_fail(4) s_rate_skip(4)
 *   s_enomem_total_events(4) s_enomem_suppressed(4) s_netdown_fail(4)
 *   rssi_pack(4)
 *
 * crash_count is the NVS-persisted count of crashes since the last
 * ESP_RST_POWERON, sourced from crash_counter_get(). See crash_counter.h
 * for details.
 *
 * Counter field sources:
 *   s_cb_count              — csi_collector_get_cb_count()
 *   s_send_ok/fail/rate_skip/netdown_fail/enomem_events/enomem_suppressed
 *                           — stream_sender_get_*()
 *   rssi_pack               — [current, min, max, reserved] as int8 per byte,
 *                             min/max tracked across the 1-second window
 */

#ifndef TELEMETRY_SENDER_H
#define TELEMETRY_SENDER_H

#include "esp_err.h"

/**
 * Start the structured-telemetry FreeRTOS task.
 *
 * Must be called after nvs_flash_init(), stream_sender_init*() and
 * WiFi STA readiness. Node_id is cached once at task startup via
 * nvs_config_load() — the task does NOT re-read NVS at each emission.
 *
 * Idempotent: second call returns ESP_OK without creating a second task.
 *
 * @return ESP_OK on success, ESP_FAIL if xTaskCreate fails.
 */
esp_err_t telemetry_sender_start(void);

#endif /* TELEMETRY_SENDER_H */
