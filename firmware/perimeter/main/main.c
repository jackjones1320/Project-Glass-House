/**
 * @file main.c
 * @brief ESP32-S3 CSI perimeter node firmware.
 *
 * Initializes NVS, WiFi STA mode, CSI collection, and UDP streaming.
 * CSI frames are serialized into binary packets and forwarded to the
 * coordinator over UDP for relay to the host aggregator.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_app_desc.h"
#include "sdkconfig.h"

#include "csi_collector.h"
#include "stream_sender.h"
#include "nvs_config.h"
#include "edge_processing.h"
#include "heartbeat.h"
#include "link_reporter.h"
#include "telemetry_sender.h"
#include "send_watchdog.h"
#include "crash_counter.h"
#include "power_mgmt.h"

static const char *TAG = "main";

/* Runtime configuration (loaded from NVS or Kconfig defaults). */
nvs_config_t g_nvs_config;

/* Event group bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define MAX_RETRY 10

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_num++;
        if (s_retry_num <= MAX_RETRY) {
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)", s_retry_num, MAX_RETRY);
        } else if (s_retry_num % 50 == 0) {
            ESP_LOGW(TAG, "WiFi reconnect attempt %d (will keep trying)", s_retry_num);
        }
        /* Always retry — perimeter nodes must reconnect if coordinator reboots */
        vTaskDelay(pdMS_TO_TICKS(s_retry_num > MAX_RETRY ? 2000 : 100));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            /* Accept OPEN or WPA2 APs (coordinator may run either). The
             * threshold is the *minimum* security; the actual auth used is
             * whatever the AP advertises. OPEN here lets the STA join an
             * OPEN coordinator even when NVS still has a password from a
             * prior WPA2 provisioning. */
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    /* Copy runtime SSID from NVS config. */
    strncpy((char *)wifi_config.sta.ssid, g_nvs_config.wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);

    /* Coordinator SoftAP is OPEN. ESP-IDF STA, when password is non-empty,
     * attempts WPA2 even if the AP is OPEN — the auth handshake then fails
     * because the AP is not expecting a password. Leaving the password
     * field zeroed forces the STA to associate as OPEN. The NVS password
     * is preserved so a later coordinator with WPA2 can use it; restore the
     * strncpy if/when the coordinator switches back to WPA2. */
    /* strncpy((char *)wifi_config.sta.password, g_nvs_config.wifi_password, sizeof(wifi_config.sta.password) - 1); */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    /* Enable 11b/g/n so HT40 negotiation succeeds with the coordinator's
     * SoftAP. AMPDU_RX is disabled in sdkconfig.defaults to avoid a known
     * RX AMPDU de-aggregation defect in the WiFi driver. */
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* HT40 (40 MHz) bandwidth doubles CSI delay-domain resolution from 15 m
     * (HT20) to 7.5 m. The SoftAP advertises HT40+ (secondary above) on
     * channel 1; the STA negotiates to match. If the SoftAP does not
     * advertise HT40, this call is harmless — the STA stays at the AP's
     * advertised bandwidth. */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));

    ESP_LOGI(TAG, "WiFi STA initialized, connecting to SSID: %s", g_nvs_config.wifi_ssid);

    /* Wait for connection (up to 30s, then proceed — reconnect continues in background) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
    } else {
        ESP_LOGW(TAG, "WiFi not connected after 30s — continuing startup, reconnect in background");
    }
}

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Bump the persistent crash counter based on esp_reset_reason(). Must
     * run AFTER nvs_flash_init() and BEFORE telemetry_sender_start() so the
     * first heartbeat carries the post-boot counter value. */
    (void)crash_counter_init();

    /* Load runtime config (NVS overrides Kconfig defaults) */
    nvs_config_load(&g_nvs_config);

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "ESP32-S3 CSI Node — v%s — Node ID: %d",
             app_desc->version, g_nvs_config.node_id);

    wifi_init_sta();

    if (stream_sender_init_with(g_nvs_config.target_ip, g_nvs_config.target_port) != 0) {
        ESP_LOGE(TAG, "Failed to initialize UDP sender");
        return;
    }

    /* Register peer MACs from NVS before starting CSI. */
    for (int i = 0; i < g_nvs_config.peer_count && i < 4; i++) {
        csi_collector_add_peer(g_nvs_config.peer_node_ids[i],
                               g_nvs_config.peer_macs[i]);
    }
    ESP_LOGI(TAG, "Registered %d peer(s) for pairwise CSI sensing",
             (g_nvs_config.peer_count < 4) ? g_nvs_config.peer_count : 4);

    csi_collector_init();

    /* Periodic NDP broadcast stimulates CSI callbacks on every peer at a
     * controlled cadence. Promiscuous mode also captures CSI from all
     * other 802.11 traffic, so the effective per-peer capture rate is
     * well above the NDP cadence alone. */
    csi_collector_start_ndp_probe(50000);  /* 50 ms period = 20 Hz */

    /* Start heartbeat (CSI stimulus) and link reporter. */
    heartbeat_start(g_nvs_config.target_ip, g_nvs_config.target_port, 50);

#if CONFIG_SAR_MODE
    uint32_t link_interval_ms = CONFIG_SAR_LINK_REPORT_INTERVAL_MS;
#else
    uint32_t link_interval_ms = CONFIG_LINK_REPORT_INTERVAL_MS;
#endif
    link_reporter_start(g_nvs_config.node_id, g_nvs_config.target_ip,
                        g_nvs_config.target_port, link_interval_ms);

    /* Structured telemetry frame 0xC5110009 at 1 Hz. */
    if (telemetry_sender_start() != ESP_OK) {
        ESP_LOGW(TAG, "telemetry_sender_start failed — continuing without telemetry");
    }

    /* Send-rate-gauge watchdog: monitors stream_sender progress and
     * escalates recovery when the send counter is flat. */
    send_watchdog_start(g_nvs_config.node_id);

    /* Start multi-frequency channel hopping if configured in NVS. */
    if (g_nvs_config.channel_hop_count > 1) {
        ESP_LOGI(TAG, "Starting channel hopping: %u channels, dwell=%lu ms",
                 (unsigned)g_nvs_config.channel_hop_count,
                 (unsigned long)g_nvs_config.dwell_ms);
        csi_collector_set_hop_table(
            g_nvs_config.channel_list,
            g_nvs_config.channel_hop_count,
            g_nvs_config.dwell_ms);
    }

    /* Initialize edge processing pipeline. */
    edge_config_t edge_cfg = {
        .tier              = g_nvs_config.edge_tier,
        .presence_thresh   = g_nvs_config.presence_thresh,
        .fall_thresh       = g_nvs_config.fall_thresh,
        .vital_window      = g_nvs_config.vital_window,
        .vital_interval_ms = g_nvs_config.vital_interval_ms,
        .top_k_count       = g_nvs_config.top_k_count,
        .power_duty        = g_nvs_config.power_duty,
    };
    esp_err_t edge_ret = edge_processing_init(&edge_cfg);
    if (edge_ret != ESP_OK) {
        ESP_LOGW(TAG, "Edge processing init failed: %s (continuing without edge DSP)",
                 esp_err_to_name(edge_ret));
    }

    /* Initialize power management. */
    power_mgmt_init(g_nvs_config.power_duty);

    ESP_LOGI(TAG, "CSI streaming active -> %s:%d (edge_tier=%u)",
             g_nvs_config.target_ip, g_nvs_config.target_port,
             g_nvs_config.edge_tier);

    /* Main loop — keep alive */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
