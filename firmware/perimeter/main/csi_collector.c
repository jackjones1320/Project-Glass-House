/**
 * @file csi_collector.c
 * @brief CSI data collection and binary frame serialization.
 *
 * Registers the ESP-IDF WiFi CSI callback and serializes incoming CSI data
 * into the wire frame format for UDP transmission. Also drives optional
 * multi-channel hopping and NDP frame injection for sensing-first TX.
 */

#include "csi_collector.h"
#include "nvs_config.h"
#include "stream_sender.h"
#include "edge_processing.h"
#include "link_reporter.h"

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "sdkconfig.h"

/* Access the global NVS config for MAC filter and channel override. */
extern nvs_config_t g_nvs_config;

/* Multi-MAC peer whitelist for pairwise CSI sensing. */
#define MAX_PEER_MACS 4
static uint8_t s_peer_macs[MAX_PEER_MACS][6];
/* Per-peer last-seen timestamp (us, esp_timer_get_time domain). Updated in
 * the CSI callback when a frame is matched to a peer. Used by ndp_timer_cb
 * to filter unicast NDP targets — only send to peers heard within
 * ACTIVE_PEER_TIMEOUT_US. Initialized to 0 = "never seen" (the timer callback
 * handles bootstrap by falling back to broadcast when no peer is active). */
static int64_t s_peer_last_seen_us[MAX_PEER_MACS] = {0};
#define ACTIVE_PEER_TIMEOUT_US (5LL * 1000000LL)  /* 5 seconds */
static uint8_t s_peer_node_ids[MAX_PEER_MACS];
static uint8_t s_peer_count = 0;
static uint32_t s_mac_drop_count = 0;  /* MAC filter drop counter (telemetry) */

/* Build-time guard — fail early if CSI is not enabled in sdkconfig.
 * Without this, the firmware compiles but crashes at runtime with:
 *   "E (xxxx) wifi:CSI not enabled in menuconfig!"
 * which is confusing for users flashing pre-built binaries. */
#ifndef CONFIG_ESP_WIFI_CSI_ENABLED
#error "CONFIG_ESP_WIFI_CSI_ENABLED must be set in sdkconfig. " \
       "Run: idf.py menuconfig -> Component config -> Wi-Fi -> Enable WiFi CSI, " \
       "or copy sdkconfig.defaults.template to sdkconfig.defaults before building."
#endif

static const char *TAG = "csi_collector";

static uint32_t s_sequence = 0;
static uint32_t s_cb_count = 0;
static uint32_t s_send_ok = 0;
static uint32_t s_send_fail = 0;
static uint32_t s_rate_skip = 0;

/**
 * Minimum interval between UDP sends in microseconds.
 * CSI callbacks can fire hundreds of times per second in promiscuous mode.
 * We cap the send rate to avoid exhausting lwIP packet buffers (ENOMEM).
 * Default: 20 ms = 50 Hz max send rate.
 */
#define CSI_MIN_SEND_INTERVAL_US  (20 * 1000)
static int64_t s_last_send_us = 0;

/* ---- Channel-hop state ---- */

/** Channel hop table (populated from NVS at boot or via set_hop_table). */
static uint8_t  s_hop_channels[CSI_HOP_CHANNELS_MAX] = {1, 6, 11, 36, 40, 44};

/** Number of active channels in the hop table. 1 = single-channel (no hop). */
static uint8_t  s_hop_count   = 1;

/** Dwell time per channel in milliseconds. */
static uint32_t s_dwell_ms    = 50;

/** Current index into s_hop_channels. */
static uint8_t  s_hop_index   = 0;

/** Handle for the periodic hop timer. NULL when timer is not running. */
static esp_timer_handle_t s_hop_timer = NULL;

/** Handle for the periodic NDP-probe timer. NULL when timer is not running. */
static esp_timer_handle_t s_ndp_timer = NULL;

/**
 * Serialize CSI data into the on-wire binary frame format.
 *
 * Layout:
 *   [0..3]   Magic: 0xC5110001 (LE)
 *   [4]      Node ID
 *   [5]      Number of antennas (rx_ctrl.rx_ant + 1 if available, else 1)
 *   [6..7]   Number of subcarriers (LE u16) = len / (2 * n_antennas)
 *   [8..11]  Frequency MHz (LE u32) — derived from channel
 *   [12..15] Sequence number (LE u32)
 *   [16]     RSSI (i8)
 *   [17]     Noise floor (i8)
 *   [18..19] Reserved
 *   [20..]   I/Q data (raw bytes from ESP-IDF callback)
 */
size_t csi_serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    if (info == NULL || buf == NULL || info->buf == NULL) {
        return 0;
    }

    uint8_t n_antennas = 1;  /* ESP32-S3 typically reports 1 antenna for CSI */
    uint16_t iq_len = (uint16_t)info->len;
    uint16_t n_subcarriers = iq_len / (2 * n_antennas);

    size_t frame_size = CSI_HEADER_SIZE + iq_len;
    if (frame_size > buf_len) {
        ESP_LOGW(TAG, "Buffer too small: need %u, have %u", (unsigned)frame_size, (unsigned)buf_len);
        return 0;
    }

    /* Derive frequency from channel number */
    uint8_t channel = info->rx_ctrl.channel;
    uint32_t freq_mhz;
    if (channel >= 1 && channel <= 13) {
        freq_mhz = 2412 + (channel - 1) * 5;
    } else if (channel == 14) {
        freq_mhz = 2484;
    } else if (channel >= 36 && channel <= 177) {
        freq_mhz = 5000 + channel * 5;
    } else {
        freq_mhz = 0;
    }

    /* Magic (LE) */
    uint32_t magic = CSI_MAGIC;
    memcpy(&buf[0], &magic, 4);

    /* Node ID (from NVS runtime config, not compile-time Kconfig) */
    buf[4] = g_nvs_config.node_id;

    /* Number of antennas */
    buf[5] = n_antennas;

    /* Number of subcarriers (LE u16) */
    memcpy(&buf[6], &n_subcarriers, 2);

    /* Frequency MHz (LE u32) */
    memcpy(&buf[8], &freq_mhz, 4);

    /* Sequence number (LE u32) */
    uint32_t seq = s_sequence++;
    memcpy(&buf[12], &seq, 4);

    /* RSSI (i8) */
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;

    /* Noise floor (i8) */
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;

    /* Reserved */
    buf[18] = 0;
    buf[19] = 0;

    /* I/Q data */
    memcpy(&buf[CSI_HEADER_SIZE], info->buf, iq_len);

    return frame_size;
}

/**
 * WiFi CSI callback — invoked by ESP-IDF when CSI data is available.
 */
static void wifi_csi_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;

    /* Track peer_id at function scope so it can be passed through
     * edge_enqueue_csi -> ring slot -> IQ packet. 0 sentinel = "unknown peer"
     * (only happens when no peer whitelist is configured and the
     * single-MAC-filter fallback path is used). */
    uint8_t peer_id_for_enqueue = 0;

    /* Multi-MAC peer whitelist filter + link reporting. */
    if (s_peer_count > 0) {
        int peer_idx = -1;
        for (int i = 0; i < s_peer_count; i++) {
            if (memcmp(info->mac, s_peer_macs[i], 6) == 0) {
                peer_idx = i;
                break;
            }
        }
        if (peer_idx < 0) {
            /* Periodic drop counter for integration debugging. */
            s_mac_drop_count++;
            if (s_mac_drop_count % 1000 == 0) {
                ESP_LOGI(TAG, "MAC filter: dropped %lu non-peer frames",
                         (unsigned long)s_mac_drop_count);
            }
            return;  /* Not from a known peer — drop */
        }
        /* Stamp peer activity for the unicast-NDP filter. */
        s_peer_last_seen_us[peer_idx] = esp_timer_get_time();

        /* Compute mean amplitude across subcarriers for the link-reporter
         * per-link health frame. */
        int8_t *buf = (int8_t *)info->buf;
        int num_sc = info->len / 2;  /* I/Q pairs */
        /* Defensive cap: a pathological info->len (driver may report up to
         * ~32,000 bytes under bug conditions) would force 16,000 sqrtf calls
         * inside the WiFi callback and block RX for ~1.3 ms. Cap at 256
         * subcarriers (512-byte payload) — covers HT20 (64 sc), HT40
         * (128 sc), and dual-LTF variants with margin. */
        if (num_sc > 256) num_sc = 256;
        float sum_amp = 0;
        for (int j = 0; j < num_sc; j++) {
            float i_val = (float)buf[j * 2];
            float q_val = (float)buf[j * 2 + 1];
            float amp = sqrtf(i_val * i_val + q_val * q_val);
            sum_amp += amp;
        }
        float mean_amp = sum_amp / (num_sc > 0 ? num_sc : 1);

        peer_id_for_enqueue = s_peer_node_ids[peer_idx];
        link_reporter_record(s_peer_node_ids[peer_idx], mean_amp);
    } else if (g_nvs_config.filter_mac_set) {
        /* Fallback: original single MAC filter if no peers configured */
        if (memcmp(info->mac, g_nvs_config.filter_mac, 6) != 0) {
            return;
        }
    }

    s_cb_count++;

    if (s_cb_count <= 3 || (s_cb_count % 100) == 0) {
        ESP_LOGI(TAG, "CSI cb #%lu: len=%d rssi=%d ch=%d",
                 (unsigned long)s_cb_count, info->len,
                 info->rx_ctrl.rssi, info->rx_ctrl.channel);
    }

    uint8_t frame_buf[CSI_MAX_FRAME_SIZE];
    size_t frame_len = csi_serialize_frame(info, frame_buf, sizeof(frame_buf));

    if (frame_len > 0) {
        /* Rate-limit UDP sends to avoid ENOMEM from lwIP pbuf exhaustion.
         * In promiscuous mode, CSI callbacks can fire 100-500+ times/sec.
         * We only need 20-50 Hz for the sensing pipeline. */
        int64_t now = esp_timer_get_time();
        if ((now - s_last_send_us) >= CSI_MIN_SEND_INTERVAL_US) {
            int ret = stream_sender_send(frame_buf, frame_len);
            if (ret > 0) {
                s_send_ok++;
                s_last_send_us = now;
            } else {
                s_send_fail++;
                if (s_send_fail <= 5) {
                    ESP_LOGW(TAG, "sendto failed (fail #%lu)", (unsigned long)s_send_fail);
                }
            }
        } else {
            s_rate_skip++;
        }
    }

    /* Enqueue raw I/Q into edge processing ring buffer. peer_id_for_enqueue
     * carries the source perimeter (1-4) so the host-side detector can
     * demultiplex per-link CSI streams. */
    if (info->buf && info->len > 0) {
        edge_enqueue_csi((const uint8_t *)info->buf, (uint16_t)info->len,
                         (int8_t)info->rx_ctrl.rssi, info->rx_ctrl.channel,
                         peer_id_for_enqueue);
    }
}

/**
 * Promiscuous mode callback — required for CSI to fire on all received frames.
 * We don't need the packet content, just the CSI triggered by reception.
 */
static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    /* No-op: CSI callback is registered separately and fires in parallel. */
    (void)buf;
    (void)type;
}

void csi_collector_add_peer(uint8_t node_id, const uint8_t *mac)
{
    if (mac == NULL) {
        ESP_LOGW(TAG, "csi_collector_add_peer: mac is NULL");
        return;
    }
    if (s_peer_count >= MAX_PEER_MACS) {
        ESP_LOGW(TAG, "csi_collector_add_peer: max peers (%d) reached, ignoring node %d",
                 MAX_PEER_MACS, node_id);
        return;
    }
    memcpy(s_peer_macs[s_peer_count], mac, 6);
    s_peer_node_ids[s_peer_count] = node_id;
    s_peer_count++;
    ESP_LOGI(TAG, "Added peer[%d]: node_id=%d mac=%02x:%02x:%02x:%02x:%02x:%02x",
             s_peer_count - 1, node_id,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void csi_collector_init(void)
{
    /* Determine the CSI channel.
     * Priority: 1) NVS override (--channel), 2) connected AP channel, 3) Kconfig default. */
    uint8_t csi_channel = (uint8_t)CONFIG_CSI_WIFI_CHANNEL;

    if (g_nvs_config.csi_channel > 0) {
        /* Explicit NVS override via provision.py --channel */
        csi_channel = g_nvs_config.csi_channel;
        ESP_LOGI(TAG, "Using NVS channel override: %u", (unsigned)csi_channel);
    } else {
        /* Auto-detect from connected AP */
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK && ap_info.primary > 0) {
            csi_channel = ap_info.primary;
            ESP_LOGI(TAG, "Auto-detected AP channel: %u", (unsigned)csi_channel);
        } else {
            ESP_LOGW(TAG, "Could not detect AP channel, using Kconfig default: %u",
                     (unsigned)csi_channel);
        }
    }

    /* Update the hop table's first channel to match. */
    s_hop_channels[0] = csi_channel;

    /* Enable promiscuous mode — required for reliable CSI callbacks.
     * Without this, CSI only fires on frames destined to this station,
     * which may be very infrequent on a quiet network. */
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));

    wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));

    ESP_LOGI(TAG, "Promiscuous mode enabled for CSI capture");

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    if (g_nvs_config.filter_mac_set) {
        ESP_LOGI(TAG, "MAC filter active: %02x:%02x:%02x:%02x:%02x:%02x",
                 g_nvs_config.filter_mac[0], g_nvs_config.filter_mac[1],
                 g_nvs_config.filter_mac[2], g_nvs_config.filter_mac[3],
                 g_nvs_config.filter_mac[4], g_nvs_config.filter_mac[5]);
    }

    ESP_LOGI(TAG, "CSI collection initialized (node_id=%d, channel=%u)",
             g_nvs_config.node_id, (unsigned)csi_channel);
}

/* ---- Channel hopping ---- */

void csi_collector_set_hop_table(const uint8_t *channels, uint8_t hop_count, uint32_t dwell_ms)
{
    if (channels == NULL) {
        ESP_LOGW(TAG, "csi_collector_set_hop_table: channels is NULL");
        return;
    }
    if (hop_count == 0 || hop_count > CSI_HOP_CHANNELS_MAX) {
        ESP_LOGW(TAG, "csi_collector_set_hop_table: invalid hop_count=%u (max=%u)",
                 (unsigned)hop_count, (unsigned)CSI_HOP_CHANNELS_MAX);
        return;
    }
    if (dwell_ms < 10) {
        ESP_LOGW(TAG, "csi_collector_set_hop_table: dwell_ms=%lu too small, clamping to 10",
                 (unsigned long)dwell_ms);
        dwell_ms = 10;
    }

    memcpy(s_hop_channels, channels, hop_count);
    s_hop_count = hop_count;
    s_dwell_ms  = dwell_ms;
    s_hop_index = 0;

    ESP_LOGI(TAG, "Hop table set: %u channels, dwell=%lu ms", (unsigned)hop_count,
             (unsigned long)dwell_ms);
    for (uint8_t i = 0; i < hop_count; i++) {
        ESP_LOGI(TAG, "  hop[%u] = channel %u", (unsigned)i, (unsigned)channels[i]);
    }
}

void csi_hop_next_channel(void)
{
    if (s_hop_count <= 1) {
        /* Single-channel mode: no-op for backward compatibility. */
        return;
    }

    s_hop_index = (s_hop_index + 1) % s_hop_count;
    uint8_t channel = s_hop_channels[s_hop_index];

    /*
     * esp_wifi_set_channel() changes the primary channel.
     * The second parameter is the secondary channel offset for HT40;
     * we use HT20 (no secondary) for sensing.
     */
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Channel hop to %u failed: %s", (unsigned)channel, esp_err_to_name(err));
    } else if ((s_cb_count % 200) == 0) {
        /* Periodic log to confirm hopping is working (not every hop). */
        ESP_LOGI(TAG, "Hopped to channel %u (index %u/%u)",
                 (unsigned)channel, (unsigned)s_hop_index, (unsigned)s_hop_count);
    }
}

/**
 * Timer callback for channel hopping.
 * Called every s_dwell_ms milliseconds from the esp_timer context.
 */
static void hop_timer_cb(void *arg)
{
    (void)arg;
    csi_hop_next_channel();
}

void csi_collector_start_hop_timer(void)
{
    if (s_hop_count <= 1) {
        ESP_LOGI(TAG, "Single-channel mode: hop timer not started");
        return;
    }

    if (s_hop_timer != NULL) {
        ESP_LOGW(TAG, "Hop timer already running");
        return;
    }

    esp_timer_create_args_t timer_args = {
        .callback = hop_timer_cb,
        .arg      = NULL,
        .name     = "csi_hop",
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_hop_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create hop timer: %s", esp_err_to_name(err));
        return;
    }

    uint64_t period_us = (uint64_t)s_dwell_ms * 1000;
    err = esp_timer_start_periodic(s_hop_timer, period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start hop timer: %s", esp_err_to_name(err));
        esp_timer_delete(s_hop_timer);
        s_hop_timer = NULL;
        return;
    }

    ESP_LOGI(TAG, "Hop timer started: period=%lu ms, channels=%u",
             (unsigned long)s_dwell_ms, (unsigned)s_hop_count);
}

/* ---- NDP frame injection ---- */

/**
 * Send a Null Data frame, optionally addressed to a specific peer.
 *
 * @param peer_mac  If non-NULL, Addr1 is set to this MAC (unicast). If NULL,
 *                  Addr1 is broadcast.
 *
 * Broadcast frames are forced to non-HT base rates by 802.11, which prevents
 * HT40 PHY from propagating to CSI capture. Unicast frames let the rate-
 * control module pick higher rates including HT40 MCS, which is required for
 * HT40 to appear at the receiver-side CSI capture.
 *
 * Plain unicast Data Null frames mandate ACK from the destination. To avoid
 * retry pile-up on absent peers, the timer callback (ndp_timer_cb below)
 * filters out peers that have not been heard from recently — only sending
 * unicast NDPs to peers whose last CSI reception was within
 * ACTIVE_PEER_TIMEOUT_US.
 *
 * Note: esp_wifi_80211_tx rejects QoS subtypes (it logs
 * "unsupport QoS frame type: 8c0" and returns ESP_ERR_INVALID_ARG), so only
 * the plain MGMT/DATA/NULL_DATA subtypes are usable here.
 */
esp_err_t csi_inject_ndp_frame_to(const uint8_t *peer_mac)
{
    uint8_t ndp_frame[24];
    memset(ndp_frame, 0, sizeof(ndp_frame));

    /* Frame Control: Type=Data (0x02), Subtype=Null (0x04) -> 0x0048 */
    ndp_frame[0] = 0x48;
    ndp_frame[1] = 0x00;

    /* Addr1 (destination): unicast to peer if provided, else broadcast */
    if (peer_mac != NULL) {
        memcpy(&ndp_frame[4], peer_mac, 6);
    } else {
        memset(&ndp_frame[4], 0xFF, 6);
    }

    /* Addr2 (source): will be overwritten by hardware with own MAC */

    /* Addr3 (BSSID): broadcast (IBSS-style frame). The peer captures via
     * promiscuous mode regardless of Addr3. */
    memset(&ndp_frame[16], 0xFF, 6);

    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, ndp_frame, sizeof(ndp_frame), false);
    if (err != ESP_OK) {
        /* Rate-limit warnings: avoid log floods on transient ENOMEM. */
        static int64_t s_last_warn_us = 0;
        int64_t now = esp_timer_get_time();
        if (now - s_last_warn_us > 1000000) {  /* 1 Hz max */
            ESP_LOGW(TAG, "NDP inject failed: %s", esp_err_to_name(err));
            s_last_warn_us = now;
        }
    }

    return err;
}

/** Broadcast NDP wrapper (used when no peers are registered). */
esp_err_t csi_inject_ndp_frame(void)
{
    return csi_inject_ndp_frame_to(NULL);
}

/* ---- Periodic NDP probe driver ---- */

/**
 * Timer callback for NDP probe injection.
 * Runs in esp_timer task context (not ISR), so calling
 * esp_wifi_80211_tx() here is safe.
 *
 * Strategy: unicast-with-activity-filter. For each registered peer, send a
 * unicast NDP only if that peer has been heard from within
 * ACTIVE_PEER_TIMEOUT_US. Unicast addressing lets rate-control pick HT40
 * MCS rates (which is required for HT40 to propagate to receiver-side CSI
 * capture); filtering avoids absent-peer retry pile-up that can drive
 * ESP_ERR_NO_MEM.
 *
 * Bootstrap: if no peer has been heard within timeout (e.g., at startup or
 * after a mesh outage), fall back to broadcast NDP for re-discovery. The
 * broadcast frame is captured by any peer that is online and stamps their
 * last_seen, after which the callback transitions to per-peer unicast.
 */
static void ndp_timer_cb(void *arg)
{
    (void)arg;
    if (s_peer_count == 0) {
        (void)csi_inject_ndp_frame();
        return;
    }
    int64_t now = esp_timer_get_time();
    int n_active = 0;
    for (uint8_t i = 0; i < s_peer_count; i++) {
        int64_t age = now - s_peer_last_seen_us[i];
        if (s_peer_last_seen_us[i] > 0 && age < ACTIVE_PEER_TIMEOUT_US) {
            (void)csi_inject_ndp_frame_to(s_peer_macs[i]);
            n_active++;
        }
    }
    if (n_active == 0) {
        /* Bootstrap / re-discovery: nothing heard recently → broadcast. */
        (void)csi_inject_ndp_frame();
    }
}

void csi_collector_start_ndp_probe(uint32_t period_us)
{
    /* Airtime budget sanity (see header):
     *   24 us * 50 Hz * 4 nodes = 4.8 ms/s = 0.48% channel utilization.
     * Safe for 2.4 GHz with other traffic present. */

    if (period_us == 0) {
        ESP_LOGW(TAG, "csi_collector_start_ndp_probe: period_us=0, ignoring");
        return;
    }

    /* Idempotent: if a timer already exists, stop + delete and recreate.
     * Keeps the API simple — caller does not need to know prior state. */
    if (s_ndp_timer != NULL) {
        esp_timer_stop(s_ndp_timer);
        esp_timer_delete(s_ndp_timer);
        s_ndp_timer = NULL;
    }

    esp_timer_create_args_t timer_args = {
        .callback        = ndp_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "csi_ndp",
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_ndp_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create NDP timer: %s", esp_err_to_name(err));
        s_ndp_timer = NULL;
        return;
    }

    err = esp_timer_start_periodic(s_ndp_timer, (uint64_t)period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start NDP timer: %s", esp_err_to_name(err));
        esp_timer_delete(s_ndp_timer);
        s_ndp_timer = NULL;
        return;
    }

    /* Log in Hz for operator clarity. Integer division is fine here —
     * any reasonable period (1..1_000_000 us) maps to a usable Hz value. */
    uint32_t hz = (period_us > 0) ? (1000000u / period_us) : 0u;
    ESP_LOGI(TAG, "NDP probe started at %u Hz (period=%u us)",
             (unsigned)hz, (unsigned)period_us);
}

void csi_collector_stop_ndp_probe(void)
{
    if (s_ndp_timer == NULL) {
        return;
    }
    esp_timer_stop(s_ndp_timer);
    esp_timer_delete(s_ndp_timer);
    s_ndp_timer = NULL;
    ESP_LOGI(TAG, "NDP probe stopped");
}

/* Read-only promiscuous-callback tick count, consumed by the 0xC5110009
 * telemetry frame. The counter is incremented inside the WiFi CSI callback;
 * single uint32_t torn-read risk is negligible for observability use. */
uint32_t csi_collector_get_cb_count(void)
{
    return s_cb_count;
}
