/**
 * @file telemetry_sender.c
 * @brief 0xC5110009 structured telemetry frame emitter.
 *
 * Dedicated FreeRTOS task emits a 48-byte packet at 1 Hz via
 * stream_sender_send(). See telemetry_sender.h for the wire format.
 */

#include "telemetry_sender.h"

#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crash_counter.h"     /* NVS-backed crash_count */
#include "csi_collector.h"     /* csi_collector_get_cb_count() */
#include "nvs_config.h"        /* nvs_config_t + nvs_config_load() */
#include "stream_sender.h"     /* stream_sender_send() + counter getters */

static const char *TAG = "telemetry";

#define TELEMETRY_MAGIC        0xC5110009u
#define TELEMETRY_PERIOD_MS    1000
#define TELEMETRY_PACKET_SIZE  48

/* Packed wire-layout struct matching '<IBBHIBBHIIIIIIII' (48 bytes, LE).
 * Field order must match the host-side decoder; the compile-time assertion
 * below catches any silent struct-padding drift. */
typedef struct __attribute__((packed)) {
    uint32_t magic;                  /* 0xC5110009 */
    uint8_t  node_id;                /* 1..4 from NVS (cached) */
    uint8_t  reset_reason;           /* esp_reset_reason() enum (one byte) */
    uint16_t crash_count;            /* crashes since last POWERON (NVS-persisted) */
    uint32_t uptime_s;               /* esp_timer_get_time() / 1_000_000 */
    uint8_t  wifi_connected;         /* 1 if STA up, 0 otherwise */
    uint8_t  _pad1;                  /* reserved, 0 */
    uint16_t free_heap_kb;           /* esp_get_free_heap_size() / 1024 */
    uint32_t s_cb_count;             /* CSI callback tick count (from csi_collector) */
    uint32_t s_send_ok;              /* stream_sender success count */
    uint32_t s_send_fail;            /* stream_sender total failure count */
    uint32_t s_rate_skip;            /* stream_sender backoff-gate skips */
    uint32_t s_enomem_total_events;  /* lwIP pbuf shortages */
    uint32_t s_enomem_suppressed;    /* suppressed ENOMEM log count (in-window) */
    uint32_t s_netdown_fail;         /* R0 counter — non-ENOMEM errno */
    uint32_t rssi_pack;              /* byte-packed [current, min, max, reserved] */
} telemetry_frame_t;

_Static_assert(sizeof(telemetry_frame_t) == TELEMETRY_PACKET_SIZE,
               "telemetry frame must be 48 bytes");

static TaskHandle_t s_task = NULL;

/* Cache node_id once at task startup. nvs_config_load() returns void (silent
 * fallback to Kconfig defaults on missing keys), so there is nothing to
 * check — just read cfg.node_id after the call. */
static uint8_t s_node_id_cache = 0;

/* RSSI window state. rssi_pack carries [current, min, max, reserved]. Min and
 * max are tracked across the 1-second interval between emissions and reset
 * after each pack, so each frame covers the interval since the previous. */
static int8_t s_rssi_current = 0;
static int8_t s_rssi_min     = 127;
static int8_t s_rssi_max     = -128;

static void telemetry_task(void *arg)
{
    (void)arg;

    /* Cache node_id once. nvs_config_load() is silent on failure (falls
     * back to Kconfig defaults internally per nvs_config.h contract). */
    nvs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    nvs_config_load(&cfg);
    s_node_id_cache = cfg.node_id;
    ESP_LOGI(TAG, "telemetry task up (node_id=%u, period=%d ms, magic=0x%08x)",
             (unsigned)s_node_id_cache, TELEMETRY_PERIOD_MS, (unsigned)TELEMETRY_MAGIC);

    telemetry_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.magic = TELEMETRY_MAGIC;

    while (1) {
        /* Populate fields — static counts snapshot at call time; race
         * tolerance is acceptable for observability-only telemetry. */
        frame.node_id         = s_node_id_cache;
        frame.reset_reason    = (uint8_t)esp_reset_reason();
        frame.crash_count     = crash_counter_get();
        frame.uptime_s        = (uint32_t)(esp_timer_get_time() / 1000000);
        frame.wifi_connected  = stream_sender_is_connected() ? 1 : 0;
        frame.free_heap_kb    = (uint16_t)(esp_get_free_heap_size() / 1024);
        frame.s_cb_count      = csi_collector_get_cb_count();
        frame.s_send_ok       = stream_sender_get_send_ok();
        frame.s_send_fail     = stream_sender_get_send_fail();
        frame.s_rate_skip     = stream_sender_get_rate_skip();
        frame.s_enomem_total_events = stream_sender_get_enomem_events();
        frame.s_enomem_suppressed   = stream_sender_get_enomem_suppressed();
        frame.s_netdown_fail  = stream_sender_get_netdown_fail();

        /* Sample current RSSI if associated and update the window. */
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            s_rssi_current = ap.rssi;
            if (ap.rssi < s_rssi_min) s_rssi_min = ap.rssi;
            if (ap.rssi > s_rssi_max) s_rssi_max = ap.rssi;
        }
        /* else: STA not associated; keep previous samples. Absence of
         * telemetry during an outage is the operational signal; stale
         * RSSI in the frame immediately before absence is fine. */

        /* Pack RSSI window into 4 bytes little-endian.
         * Layout: byte[0]=current, byte[1]=min, byte[2]=max, byte[3]=0 */
        uint32_t pack =
            ((uint32_t)(uint8_t)s_rssi_current)      |
            (((uint32_t)(uint8_t)s_rssi_min) <<  8)  |
            (((uint32_t)(uint8_t)s_rssi_max) << 16);
        frame.rssi_pack = pack;

        /* Reset window so each frame covers the interval since the prior. */
        s_rssi_min = 127;
        s_rssi_max = -128;

        /* Emit via stream_sender — its association/skip gate handles
         * STA-disconnected windows. Absence of this frame at the host
         * during an outage is intentional diagnostic behavior. */
        (void)stream_sender_send((const uint8_t *)&frame, sizeof(frame));

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

esp_err_t telemetry_sender_start(void)
{
    if (s_task != NULL) {
        return ESP_OK;  /* idempotent */
    }
    BaseType_t ok = xTaskCreate(telemetry_task, "telemetry", 4096, NULL,
                                tskIDLE_PRIORITY + 2, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed (heap? stack?)");
        s_task = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}
