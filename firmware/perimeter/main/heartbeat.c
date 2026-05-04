// firmware/perimeter/main/heartbeat.c
// GlassHouse v2 — Periodic UDP heartbeat ping to coordinator
//
// Creates CSI stimulus: each heartbeat ping is a WiFi frame that
// other perimeter nodes capture via promiscuous mode CSI callback.

#include "heartbeat.h"
#include "stream_sender.h"  /* gate sends on stream_sender_is_connected() */
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "heartbeat";
static int s_sock = -1;
static struct sockaddr_in s_dest;

static void heartbeat_cb(void *arg)
{
    if (s_sock < 0) return;
    /* Skip send when STA is not associated. Prevents this sender path from
     * contributing to a multi-path sendto cascade that can exhaust the lwIP
     * pbuf pool during STA-disconnected state. */
    if (!stream_sender_is_connected()) return;
    uint8_t ping = 0xAA;  // heartbeat marker
    sendto(s_sock, &ping, 1, 0, (struct sockaddr *)&s_dest, sizeof(s_dest));
}

void heartbeat_start(const char *target_ip, uint16_t target_port, uint32_t interval_ms)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "Socket failed");
        return;
    }

    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family = AF_INET;
    s_dest.sin_port = htons(target_port);
    inet_pton(AF_INET, target_ip, &s_dest.sin_addr);

    esp_timer_create_args_t timer_args = {
        .callback = heartbeat_cb,
        .name = "heartbeat",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, interval_ms * 1000));

    ESP_LOGI(TAG, "Heartbeat started: %s:%d every %lu ms",
             target_ip, target_port, (unsigned long)interval_ms);
}
