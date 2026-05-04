/**
 * @file csi_collector.h
 * @brief CSI data collection and binary frame serialization.
 */

#ifndef CSI_COLLECTOR_H
#define CSI_COLLECTOR_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

/** Magic number for the per-frame CSI packet. */
#define CSI_MAGIC 0xC5110001

/** CSI packet header size in bytes. */
#define CSI_HEADER_SIZE 20

/** Maximum frame buffer size (header + 4 antennas * 256 subcarriers * 2 bytes). */
#define CSI_MAX_FRAME_SIZE (CSI_HEADER_SIZE + 4 * 256 * 2)

/** Maximum number of channels in the hop table. */
#define CSI_HOP_CHANNELS_MAX 6

/**
 * Initialize CSI collection.
 * Registers the WiFi CSI callback.
 */
void csi_collector_init(void);

/**
 * Register a peer node for pairwise CSI sensing.
 *
 * Must be called before CSI callbacks start (i.e., before csi_collector_init).
 * Up to MAX_PEER_MACS (4) peers can be registered.
 *
 * @param node_id  Peer's node ID (1-4).
 * @param mac      Peer's 6-byte WiFi MAC address.
 */
void csi_collector_add_peer(uint8_t node_id, const uint8_t *mac);

/**
 * Serialize CSI data into the on-wire binary frame format.
 *
 * @param info   WiFi CSI info from the ESP-IDF callback.
 * @param buf    Output buffer (must be at least CSI_MAX_FRAME_SIZE bytes).
 * @param buf_len Size of the output buffer.
 * @return Number of bytes written, or 0 on error.
 */
size_t csi_serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len);

/**
 * Configure the channel-hop table for multi-band sensing.
 *
 * When hop_count == 1 the collector stays on the single configured channel
 * (backward-compatible with the original single-channel mode).
 *
 * @param channels  Array of WiFi channel numbers (1-14 for 2.4 GHz, 36-177 for 5 GHz).
 * @param hop_count Number of entries in the channels array (1..CSI_HOP_CHANNELS_MAX).
 * @param dwell_ms  Dwell time per channel in milliseconds (>= 10).
 */
void csi_collector_set_hop_table(const uint8_t *channels, uint8_t hop_count, uint32_t dwell_ms);

/**
 * Advance to the next channel in the hop table.
 *
 * Called by the hop timer callback. If hop_count <= 1 this is a no-op.
 * Calls esp_wifi_set_channel() internally.
 */
void csi_hop_next_channel(void);

/**
 * Start the channel-hop timer.
 *
 * Creates an esp_timer periodic callback that fires every dwell_ms
 * milliseconds, calling csi_hop_next_channel(). If hop_count <= 1
 * the timer is not started (single-channel backward-compatible mode).
 */
void csi_collector_start_hop_timer(void);

/**
 * Inject an NDP (Null Data Packet) frame for sensing.
 *
 * Uses esp_wifi_80211_tx() to send a preamble-only frame (~24 us airtime)
 * that triggers CSI measurement at all receivers. This is the
 * "sensing-first" TX stimulus mechanism.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t csi_inject_ndp_frame(void);

/**
 * Start a periodic timer that injects NDP probe frames at the requested cadence.
 *
 * Creates an esp_timer (if not already running) that calls
 * csi_inject_ndp_frame() every @p period_us microseconds. Idempotent —
 * calling again with a different period stops the existing timer and
 * restarts with the new period.
 *
 * Airtime budget: 24 us per NDP * 50 Hz * 4 nodes = 4.8 ms/s = 0.48%
 * channel utilization. Well below any reasonable airtime budget.
 *
 * @param period_us  Period in microseconds (e.g. 20000 -> 50 Hz).
 */
void csi_collector_start_ndp_probe(uint32_t period_us);

/**
 * Stop the periodic NDP probe timer (if running). Idempotent.
 */
void csi_collector_stop_ndp_probe(void);

/**
 * Read-only CSI promiscuous-callback tick counter. Incremented once per
 * callback invocation from the WiFi driver. Consumed by the 0xC5110009
 * telemetry frame. Monotonic from boot; no reset path.
 */
uint32_t csi_collector_get_cb_count(void);

#endif /* CSI_COLLECTOR_H */
