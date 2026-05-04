/**
 * @file stream_sender.c
 * @brief Module: opens UDP socket, sends frames under a mutex, exposes
 *        ok/fail/skip counters.
 */

#include "stream_sender.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"

static const char *TAG = "stream_sender";

static int s_sock = -1;
static struct sockaddr_in s_dest_addr;
static SemaphoreHandle_t s_send_mutex;       /* Serializes stream_sender_send() */

/* Counters exposed via telemetry frame 0xC5110009. */
static uint32_t s_send_ok      = 0;          /* successful sendto() calls */
static uint32_t s_send_fail    = 0;          /* total sendto() failures (all errno classes) */
static uint32_t s_rate_skip    = 0;          /* skipped because STA was not associated */

/* Self-rate-limited failure summary. Fires from the failure path itself when
 * at least FAIL_LOG_INTERVAL_US has elapsed since the last log. */
#define FAIL_LOG_INTERVAL_US (5 * 1000000)   /* 5 seconds */
static int64_t s_last_fail_log_us = 0;
static uint32_t s_fails_since_last_log = 0;


static int sender_init_internal(const char *ip, uint16_t port)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return -1;
    }

    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip, &s_dest_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid target IP: %s", ip);
        close(s_sock);
        s_sock = -1;
        return -1;
    }

    ESP_LOGI(TAG, "UDP sender initialized: %s:%d", ip, port);
    return 0;
}

int stream_sender_init(void)
{
    return sender_init_internal(CONFIG_CSI_TARGET_IP, CONFIG_CSI_TARGET_PORT);
}

int stream_sender_init_with(const char *ip, uint16_t port)
{
    int rc = sender_init_internal(ip, port);
    if (rc != 0) return rc;

    s_send_mutex = xSemaphoreCreateMutex();
    if (s_send_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create send mutex");
        close(s_sock);
        s_sock = -1;
        return ESP_ERR_NO_MEM;
    }

    return 0;
}

int stream_sender_send(const uint8_t *data, size_t len)
{
    if (s_sock < 0) {
        return -1;
    }

    /* Pre-send association gate. Prevents pbuf exhaustion cascade when the
     * STA is briefly disconnected. Not free (the connectivity check takes a
     * wifi-internal mutex) but cheaper than a failed sendto under contention. */
    if (!stream_sender_is_connected()) {
        s_rate_skip++;
        return -1;
    }

    xSemaphoreTake(s_send_mutex, portMAX_DELAY);
    int sent = sendto(s_sock, data, len, 0,
                      (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
    xSemaphoreGive(s_send_mutex);

    if (sent < 0) {
        s_send_fail++;
        s_fails_since_last_log++;

        /* Self-rate-limited summary log. Fires at most once per 5 seconds
         * regardless of failure rate. Reports count-since-last and cumulative.
         * Silence (no failures) is the all-clear signal; no need for a
         * dedicated task to print "no failures" periodically. */
        int64_t now = esp_timer_get_time();
        if (now - s_last_fail_log_us >= FAIL_LOG_INTERVAL_US) {
            ESP_LOGW(TAG, "sendto: %lu failures in last 5s (total: %lu, errno=%d)",
                     (unsigned long)s_fails_since_last_log,
                     (unsigned long)s_send_fail, errno);
            s_fails_since_last_log = 0;
            s_last_fail_log_us = now;
        }
        return -1;
    }

    s_send_ok++;
    return sent;
}

/* Counter getters. The full six-getter surface is preserved so the telemetry
 * frame layout stays ABI-stable; getters that no longer back a live counter
 * return 0 and surface as constant-0 slots in the wire frame. */
uint32_t stream_sender_get_send_ok(void)           { return s_send_ok; }
uint32_t stream_sender_get_send_fail(void)         { return s_send_fail; }
uint32_t stream_sender_get_rate_skip(void)         { return s_rate_skip; }
uint32_t stream_sender_get_netdown_fail(void)      { return 0; }
uint32_t stream_sender_get_enomem_events(void)     { return 0; }
uint32_t stream_sender_get_enomem_suppressed(void) { return 0; }

/* Connected-state helper built on esp_wifi_sta_get_ap_info(). Returns true iff
 * the STA has a current association. Telemetry uses it to populate the
 * wifi_connected field without taking a cross-module event-group dependency. */
bool stream_sender_is_connected(void) {
    wifi_ap_record_t ap;
    return (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
}

void stream_sender_deinit(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
        ESP_LOGI(TAG, "UDP sender closed");
    }
}
