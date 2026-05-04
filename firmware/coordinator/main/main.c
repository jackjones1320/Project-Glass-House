// GlassHouse  Coordinator — SoftAP + UDP-to-COBS-serial bridge

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/stats.h"  /* lwip_stats global for periodic UDP/mailbox counters */
#include "driver/uart.h"
#include "hal/usb_serial_jtag_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "sdkconfig.h"

/* Compile-time bridge-mode assertion. Kconfig `choice` guarantees exactly one
 * selection at configure time, but this guards against a hand-edited or
 * corrupted sdkconfig. */
#if (defined(CONFIG_GH_BRIDGE_USB) + defined(CONFIG_GH_BRIDGE_WIFI) + defined(CONFIG_GH_BRIDGE_BOTH)) != 1
#error "Exactly one of CONFIG_GH_BRIDGE_USB/WIFI/BOTH must be set (via Kconfig choice in Kconfig.projbuild)"
#endif

static const char *TAG = "coordinator";

#define SOFTAP_SSID       "CSI_NET_V2"   /* WARNING: now an OPEN AP — any STA in range can join. */
#define SOFTAP_CHANNEL    1
#define SOFTAP_MAX_CONN   4              /* one slot per perimeter node; caps rogue auto-joins. */
#define UDP_PORT           4210
// Data output goes via USB-Serial/JTAG (the built-in USB on ESP32-S3).
// We write directly to the hardware FIFO to avoid disrupting the USB connection.
// Log output is redirected to UART1 to avoid corrupting the COBS stream.
#define LOG_UART_NUM       UART_NUM_1
#define LOG_UART_TX_PIN    17
#define LOG_UART_RX_PIN    18
#define UART_BAUD          921600
/* Max forwarded UDP payload size; sized for full CSI frames. */
#define MAX_PKT_SIZE       2100
/* Drop logging cadence: warn on every Nth drop *or* every N seconds,
 * whichever comes first. */
#define DROP_LOG_EVERY_N       100
#define DROP_LOG_EVERY_US      (60LL * 1000LL * 1000LL)  /* 60 s */

// --- Log redirect: send ESP_LOG output to UART1 instead of USB-CDC ---
static int uart1_vprintf(const char *fmt, va_list args)
{
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        uart_write_bytes(LOG_UART_NUM, buf, len);
    }
    return len;
}

// --- Raw USB-Serial/JTAG FIFO write (bypasses driver to avoid USB disconnect) ---
// Keep retries low so a stalled USB host doesn't block the UDP receive loop.
// If the host isn't reading (port closed), drop the packet quickly and move on.
static size_t usb_jtag_write_raw(const uint8_t *data, size_t len)
{
    size_t offset = 0;
    int retries = 0;
    while (offset < len && retries < 5) {
        uint32_t written = usb_serial_jtag_ll_write_txfifo(data + offset, len - offset);
        usb_serial_jtag_ll_txfifo_flush();
        offset += written;
        if (offset < len) {
            vTaskDelay(1); // yield if FIFO was full, let USB drain
            retries++;
        }
    }
    return offset;
}

// --- COBS encoding ---
// Encodes `src` (len bytes) into `dst`. Returns encoded length.
// `dst` must be at least len + len/254 + 1 bytes.
static size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_idx = 0, write_idx = 1, code_idx = 0;
    uint8_t code = 1;

    while (read_idx < len) {
        if (src[read_idx] == 0x00) {
            dst[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
        } else {
            dst[write_idx++] = src[read_idx];
            code++;
            if (code == 0xFF) {
                dst[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
        read_idx++;
    }
    dst[code_idx] = code;
    return write_idx;
}

// --- Operator-WiFi state + STA forward-socket lifecycle ---
typedef struct {
    char     ssid[32];
    char     psk[64];
    char     host_ip[16];
    uint16_t host_port;
} coord_opwifi_t;

static coord_opwifi_t s_opwifi;

#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
/* Forward socket written by WiFi event task (STA_GOT_IP / STA_DISCONNECTED)
 * and read by udp_bridge_task's sendto — every access is guarded by the
 * portMUX to prevent torn reads/writes across cores. */
static portMUX_TYPE s_fwd_sock_mutex = portMUX_INITIALIZER_UNLOCKED;
static int          s_fwd_sock = -1;
static uint32_t     s_reconnect_attempts = 0;
#endif

/* Always called from app_main; the body is a no-op under USB-only so the
 * init sequence is identical across all three bridge modes. */
static esp_err_t coord_opwifi_load(coord_opwifi_t *out)
{
    memset(out, 0, sizeof(*out));

#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
    /* Kconfig fallbacks (only emitted when WIFI/BOTH due to `depends on`). */
    strlcpy(out->ssid,    CONFIG_GH_OPWIFI_SSID,    sizeof(out->ssid));
    strlcpy(out->psk,     CONFIG_GH_OPWIFI_PSK,     sizeof(out->psk));
    strlcpy(out->host_ip, CONFIG_GH_OPWIFI_HOST_IP, sizeof(out->host_ip));
    out->host_port = (uint16_t)CONFIG_GH_OPWIFI_HOST_PORT;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("coord", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* First boot / post-erase: namespace not provisioned. Use Kconfig
         * defaults rather than failing boot on a missing NVS namespace. */
        ESP_LOGI(TAG, "opwifi: NVS namespace 'coord' not found — using Kconfig defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "opwifi: nvs_open failed (%s) — using Kconfig defaults",
                 esp_err_to_name(err));
        return ESP_OK;
    }

    size_t sz;
    sz = sizeof(out->ssid);    (void)nvs_get_str(nvs, "opwifi.ssid",    out->ssid,    &sz);
    sz = sizeof(out->psk);     (void)nvs_get_str(nvs, "opwifi.psk",     out->psk,     &sz);
    sz = sizeof(out->host_ip); (void)nvs_get_str(nvs, "opwifi.host_ip", out->host_ip, &sz);
    (void)nvs_get_u16(nvs, "opwifi.host_port", &out->host_port);
    nvs_close(nvs);

    ESP_LOGI(TAG, "opwifi: ssid='%s' host=%s:%u",
             out->ssid, out->host_ip, (unsigned)out->host_port);
#else
    (void)out; /* USB-only: no-op — loaded values are never consulted. */
#endif
    return ESP_OK;
}

#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
static void coord_wifi_event_handler(void *arg, esp_event_base_t base,
                                     int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "sta: interface started, connecting to '%s'", s_opwifi.ssid);
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "sta: esp_wifi_connect (initial) returned %s",
                     esp_err_to_name(err));
        }
        return;
    }

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Tear down forward socket under critical section; close() is called
         * outside the spinlock since it can block. */
        int old_sock;
        portENTER_CRITICAL(&s_fwd_sock_mutex);
        old_sock = s_fwd_sock;
        s_fwd_sock = -1;
        portEXIT_CRITICAL(&s_fwd_sock_mutex);
        if (old_sock >= 0) close(old_sock);

        /* Bounded exponential backoff. */
        if (CONFIG_GH_OPWIFI_RECONNECT_MAX_ATTEMPTS > 0 &&
            s_reconnect_attempts >= (uint32_t)CONFIG_GH_OPWIFI_RECONNECT_MAX_ATTEMPTS) {
            ESP_LOGE(TAG, "sta: disconnected; max reconnect attempts (%u) reached, giving up",
                     (unsigned)CONFIG_GH_OPWIFI_RECONNECT_MAX_ATTEMPTS);
            return;
        }

        uint32_t attempt  = s_reconnect_attempts++;
        uint32_t delay_ms = (uint32_t)CONFIG_GH_OPWIFI_RECONNECT_MS;
        for (uint32_t i = 0; i < attempt && delay_ms < 30000; i++) {
            delay_ms <<= 1;
        }
        if (delay_ms > 30000) delay_ms = 30000;

        ESP_LOGW(TAG, "sta: disconnected; retrying in %u ms (attempt %u)",
                 (unsigned)delay_ms, (unsigned)(attempt + 1));
        /* Blocks the default event loop task for up to 30 s during backoff.
         * Acceptable on a stable operator hotspot; a dedicated reconnect task
         * or esp_timer one-shot would be a production hardening follow-up. */
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "sta: esp_wifi_connect (retry) returned %s",
                     esp_err_to_name(err));
        }
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        s_reconnect_attempts = 0;

        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            ESP_LOGE(TAG, "sta: got_ip but socket() failed: errno=%d", errno);
            return;
        }

        int stale_sock;
        portENTER_CRITICAL(&s_fwd_sock_mutex);
        stale_sock = s_fwd_sock;
        s_fwd_sock = sock;
        portEXIT_CRITICAL(&s_fwd_sock_mutex);
        if (stale_sock >= 0) close(stale_sock);

        ESP_LOGI(TAG, "sta: connected, ip=" IPSTR,
                 IP2STR(&event->ip_info.ip));
    }
}
#endif

// --- WiFi SoftAP setup ---
static void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();
#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
    esp_netif_create_default_wifi_sta();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* OPEN auth is used here to avoid WPA2 4-way handshake contention that
     * blocks more than one perimeter from associating concurrently under
     * continuous WIFI_PS_NONE traffic. The network is ephemeral and only
     * carries CSI telemetry, so the open-auth tradeoff is acceptable. */
    wifi_config_t ap_config = {
        .ap = {
            .ssid = SOFTAP_SSID,
            .ssid_len = strlen(SOFTAP_SSID),
            .channel = SOFTAP_CHANNEL,
            .password = "",
            .max_connection = SOFTAP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
    /* APSTA mode: SoftAP serves perimeter nodes; STA joins the operator WiFi
     * using credentials loaded from NVS (with Kconfig fallbacks). The single
     * radio time-shares both interfaces, so they must end up on the same
     * channel; operator hotspot SOP is channel 1 to match the SoftAP. */
    wifi_config_t sta_config = { 0 };
    strlcpy((char *)sta_config.sta.ssid,     s_opwifi.ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, s_opwifi.psk,  sizeof(sta_config.sta.password));
    /* Leave .threshold.authmode at default so the driver matches whatever the
     * operator hotspot advertises (OPEN or WPA2-PSK). */

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_START,        &coord_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &coord_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,   IP_EVENT_STA_GOT_IP,         &coord_wifi_event_handler, NULL));

    /* Single-radio ESP32 cannot serve AP and STA on different channels — it
     * time-shares and association auth frames get dropped otherwise. Setting
     * ap.channel=0 lets the driver auto-sync the SoftAP to whatever channel
     * the STA lands on post-association. The USB-only #else branch below
     * keeps SOFTAP_CHANNEL=1 unchanged. */
    ap_config.ap.channel = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* SoftAP runs at 40 MHz (HT40+, secondary above — the only valid
     * orientation on channel 1) to double per-perimeter CSI delay-domain
     * resolution. Paired with perimeter-side AMPDU_RX disabled + 11n. */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));

    ESP_LOGI(TAG, "APSTA started: AP=%s/CH=auto/OPEN MAX=%d HT40+; STA→'%s' (operator)",
             SOFTAP_SSID, SOFTAP_MAX_CONN, s_opwifi.ssid);
#else
    /* USB-only build: SoftAP-only WiFi mode. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* HT40+ to match the APSTA branch above for consistent CSI bandwidth. */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));

    ESP_LOGI(TAG, "SoftAP started: SSID=%s CH=%d HT40+ AUTH=OPEN MAX=%d",
             SOFTAP_SSID, SOFTAP_CHANNEL, SOFTAP_MAX_CONN);
#endif
}

// --- UART setup ---
static void uart_init(void)
{
#if CONFIG_GH_BRIDGE_USB || CONFIG_GH_BRIDGE_BOTH
    /* Redirect ESP log output to UART1 so USB-CDC stays clean for COBS data.
     * Only needed in USB/BOTH modes; in WIFI-only mode USB-CDC is idle and
     * ESP_LOG via USB-Serial/JTAG is the natural observability path (visible
     * via `idf.py monitor`). */
    uart_config_t log_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(LOG_UART_NUM, &log_cfg));
    ESP_ERROR_CHECK(uart_set_pin(LOG_UART_NUM, LOG_UART_TX_PIN, LOG_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LOG_UART_NUM, 1024, 1024, 0, NULL, 0));
    esp_log_set_vprintf((vprintf_like_t)&uart1_vprintf);
#endif /* CONFIG_GH_BRIDGE_USB || CONFIG_GH_BRIDGE_BOTH */
}

// --- UDP listener task ---
static void udp_bridge_task(void *pvParameters)
{
    /* rx_buf sized to hold a full CSI frame.
     * cobs_buf has worst-case COBS expansion: +1 header per 254 bytes + delimiter. */
    static uint8_t rx_buf[MAX_PKT_SIZE];
#if CONFIG_GH_BRIDGE_USB || CONFIG_GH_BRIDGE_BOTH
    static uint8_t cobs_buf[MAX_PKT_SIZE + MAX_PKT_SIZE / 254 + 2];
    uint32_t usb_write_fail_count = 0;
#endif
#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
    uint32_t wifi_sendto_fail_count = 0;
#endif
    uint32_t pkt_count = 0;

    /* 30s-window stats log. Reads come from the global lwip_stats struct
     * which is updated from the lwIP core task; intra-counter tearing on
     * u32 reads is acceptable for diagnostic monotonic counters. */
    int64_t  s_stats_window_start_us = esp_timer_get_time();
    uint32_t s_stats_pkts_at_window_start = 0;
#if LWIP_STATS
    uint32_t s_stats_drop_at_start   = lwip_stats.udp.drop;
    uint32_t s_stats_memerr_at_start = lwip_stats.udp.memerr;
    uint32_t s_stats_mbox_at_start   = lwip_stats.sys.mbox.err;
#endif

    /* Track and periodically log oversized-drop events. */
    uint32_t oversize_drop_count = 0;
    uint32_t oversize_drops_since_log = 0;
    uint32_t oversize_max_seen = 0;
    int64_t  last_drop_log_us = 0;

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP bridge listening on port %d", UDP_PORT);

    while (1) {
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0, NULL, NULL);
        if (len <= 0) continue;
        if ((size_t)len > MAX_PKT_SIZE) {
            /* Count drops, track the largest oversized packet seen, and warn
             * via ESP_LOGW (routed to UART1 by uart1_vprintf) every
             * DROP_LOG_EVERY_N drops or every DROP_LOG_EVERY_US, whichever
             * comes first. */
            oversize_drop_count++;
            oversize_drops_since_log++;
            if ((uint32_t)len > oversize_max_seen) oversize_max_seen = (uint32_t)len;

            int64_t now = esp_timer_get_time();
            bool count_gate = (oversize_drops_since_log >= DROP_LOG_EVERY_N);
            bool time_gate  = (last_drop_log_us != 0) &&
                              ((now - last_drop_log_us) >= DROP_LOG_EVERY_US);
            if (count_gate || time_gate || last_drop_log_us == 0) {
                ESP_LOGW(TAG,
                    "oversize drop: len=%d > MAX_PKT_SIZE=%d (total=%lu, since_last=%lu, max_seen=%lu)",
                    len, MAX_PKT_SIZE,
                    (unsigned long)oversize_drop_count,
                    (unsigned long)oversize_drops_since_log,
                    (unsigned long)oversize_max_seen);
                oversize_drops_since_log = 0;
                last_drop_log_us = now;
            }
            continue;
        }

#if CONFIG_GH_BRIDGE_USB || CONFIG_GH_BRIDGE_BOTH
        /* COBS-over-USB-Serial/JTAG forward path. */
        size_t cobs_len = cobs_encode(rx_buf, (size_t)len, cobs_buf);
        cobs_buf[cobs_len] = 0x00; // COBS delimiter
        size_t written = usb_jtag_write_raw(cobs_buf, cobs_len + 1);
        if (written < cobs_len + 1) {
            usb_write_fail_count++;
        }
#endif

#if CONFIG_GH_BRIDGE_WIFI || CONFIG_GH_BRIDGE_BOTH
        /* Raw UDP forward to operator host. No COBS — UDP is already framed.
         * Read s_fwd_sock under portMUX, then sendto OUTSIDE the critical
         * section to avoid holding the spinlock across a potentially
         * blocking lwIP call. */
        int sock_snapshot;
        portENTER_CRITICAL(&s_fwd_sock_mutex);
        sock_snapshot = s_fwd_sock;
        portEXIT_CRITICAL(&s_fwd_sock_mutex);
        if (sock_snapshot >= 0) {
            struct sockaddr_in dst = {
                .sin_family = AF_INET,
                .sin_addr.s_addr = inet_addr(s_opwifi.host_ip),
                .sin_port = htons(s_opwifi.host_port),
            };
            ssize_t sent = sendto(sock_snapshot, rx_buf, (size_t)len, 0,
                                  (struct sockaddr *)&dst, sizeof(dst));
            if (sent < 0) {
                /* Rate-limited warn: at most one log per second, with the
                 * suppressed-count included in the emitted message. */
                static int64_t  last_warn_us = 0;
                static uint32_t suppressed   = 0;
                wifi_sendto_fail_count++;
                int64_t now_us = esp_timer_get_time();
                if (now_us - last_warn_us > 1000000) {
                    ESP_LOGW(TAG, "fwd sendto failed: errno=%d (suppressed %u similar in last 1s)",
                             errno, (unsigned)suppressed);
                    last_warn_us = now_us;
                    suppressed = 0;
                } else {
                    suppressed++;
                }
            }
        }
#endif

        pkt_count++;
        if (pkt_count % 100 == 0) {
#if CONFIG_GH_BRIDGE_USB
            ESP_LOGI(TAG, "Forwarded %lu packets (%lu USB write failures)",
                     (unsigned long)pkt_count, (unsigned long)usb_write_fail_count);
#elif CONFIG_GH_BRIDGE_WIFI
            ESP_LOGI(TAG, "Forwarded %lu packets (%lu WiFi sendto failures)",
                     (unsigned long)pkt_count, (unsigned long)wifi_sendto_fail_count);
#elif CONFIG_GH_BRIDGE_BOTH
            ESP_LOGI(TAG, "Forwarded %lu packets (USB fails=%lu, WiFi fails=%lu)",
                     (unsigned long)pkt_count,
                     (unsigned long)usb_write_fail_count,
                     (unsigned long)wifi_sendto_fail_count);
#endif
        }

        /* Emit stats every 30s. Log format:
         *   coord_rx_stats: pkts=(\d+) udp\.drop=(\d+) udp\.memerr=(\d+) mbox\.err=(\d+) 30s_pps=([\d.]+) */
#if LWIP_STATS
        int64_t now_us = esp_timer_get_time();
        if ((now_us - s_stats_window_start_us) >= 30LL * 1000000LL) {
            double window_s = (double)(now_us - s_stats_window_start_us) / 1000000.0;
            uint32_t d_pkts = pkt_count - s_stats_pkts_at_window_start;
            double   pps    = window_s > 0 ? (double)d_pkts / window_s : 0.0;
            ESP_LOGI(TAG,
                "coord_rx_stats: pkts=%lu udp.drop=%lu udp.memerr=%lu mbox.err=%lu 30s_pps=%.1f",
                (unsigned long)pkt_count,
                (unsigned long)lwip_stats.udp.drop,
                (unsigned long)lwip_stats.udp.memerr,
                (unsigned long)lwip_stats.sys.mbox.err,
                pps);
            /* Reset snapshot for next window. Absolute values are emitted;
             * downstream analysis computes deltas across consecutive lines. */
            s_stats_window_start_us      = now_us;
            s_stats_pkts_at_window_start = pkt_count;
            s_stats_drop_at_start        = lwip_stats.udp.drop;
            s_stats_memerr_at_start      = lwip_stats.udp.memerr;
            s_stats_mbox_at_start        = lwip_stats.sys.mbox.err;
            (void)s_stats_drop_at_start;
            (void)s_stats_memerr_at_start;
            (void)s_stats_mbox_at_start;
        }
#endif
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uart_init();

    /* Always load opwifi state. Body is a no-op under CONFIG_GH_BRIDGE_USB=y;
     * the loaded struct is only consulted when WIFI or BOTH is active. */
    ESP_ERROR_CHECK(coord_opwifi_load(&s_opwifi));

    wifi_init_softap();

    xTaskCreate(udp_bridge_task, "udp_bridge", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "GlassHouse v2 Coordinator ready");
}
