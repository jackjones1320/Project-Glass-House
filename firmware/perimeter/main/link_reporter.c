// firmware/perimeter/main/link_reporter.c
// GlassHouse v2 — Per-link variance computation and summary reporting
//
// Each perimeter node computes variance of CSI amplitudes per peer link
// using Welford's online algorithm (double precision for stability).
// Reports are 10-byte packed structs sent to the coordinator via UDP.

#include "link_reporter.h"
#include "stream_sender.h"  /* gate sends on stream_sender_is_connected() */
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>
#include <math.h>

static const char *TAG = "link_reporter";

// Per-peer sliding window
typedef struct {
    uint8_t  peer_id;
    float    samples[LINK_WINDOW_SIZE];
    uint16_t count;
    uint16_t write_idx;
    uint8_t  active;
} peer_window_t;

static peer_window_t s_peers[LINK_MAX_PEERS];
static uint8_t s_my_node_id;
static int s_sock = -1;
static struct sockaddr_in s_dest;
static uint32_t s_slot_exhaust_count = 0;  // periodically logged on overflow

// Packed link report: type(1) + node(1) + partner(1) + variance(4) + state(1) + count(2) = 10
#pragma pack(push, 1)
typedef struct {
    uint8_t  type;
    uint8_t  node_id;
    uint8_t  partner_id;
    float    variance;
    uint8_t  state;
    uint16_t sample_count;
} link_report_t;
#pragma pack(pop)

// Thread safety: CSI callback and reporter timer access s_peers concurrently
static portMUX_TYPE s_peers_mux = portMUX_INITIALIZER_UNLOCKED;

static float compute_variance(const float *samples, uint16_t count)
{
    if (count < 2) return 0.0f;
    // Welford's online algorithm for numerical stability
    uint16_t n = (count < LINK_WINDOW_SIZE) ? count : LINK_WINDOW_SIZE;
    double mean = 0.0, m2 = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        double delta = (double)samples[i] - mean;
        mean += delta / (i + 1);
        double delta2 = (double)samples[i] - mean;
        m2 += delta * delta2;
    }
    return (float)(m2 / n);
}

void link_reporter_record(uint8_t peer_node_id, float amplitude)
{
    taskENTER_CRITICAL(&s_peers_mux);

    // Find or allocate peer slot
    peer_window_t *slot = NULL;
    for (int i = 0; i < LINK_MAX_PEERS; i++) {
        if (s_peers[i].active && s_peers[i].peer_id == peer_node_id) {
            slot = &s_peers[i];
            break;
        }
        if (!s_peers[i].active && slot == NULL) {
            slot = &s_peers[i];
        }
    }
    if (slot == NULL) {
        s_slot_exhaust_count++;
        taskEXIT_CRITICAL(&s_peers_mux);
        // Log peer slot exhaustion periodically
        if (s_slot_exhaust_count % 100 == 0) {
            ESP_LOGW(TAG, "Peer slot exhausted %lu times (max %d peers)",
                     (unsigned long)s_slot_exhaust_count, LINK_MAX_PEERS);
        }
        return;
    }

    if (!slot->active) {
        slot->peer_id = peer_node_id;
        slot->count = 0;
        slot->write_idx = 0;
        slot->active = 1;
    }

    slot->samples[slot->write_idx % LINK_WINDOW_SIZE] = amplitude;
    slot->write_idx++;
    if (slot->count < LINK_WINDOW_SIZE) slot->count++;

    taskEXIT_CRITICAL(&s_peers_mux);
}

static void reporter_cb(void *arg)
{
    if (s_sock < 0) return;

    /* Skip entirely when STA is not associated. Prevents this sender path
     * from contributing to a multi-path sendto cascade that can exhaust the
     * lwIP pbuf pool during STA-disconnected state. Snapshot computation is
     * also skipped — no point computing variance for sends that would drop. */
    if (!stream_sender_is_connected()) return;

    // Snapshot peer data under lock, then send outside lock
    // (sendto must not be in critical section)
    link_report_t reports[LINK_MAX_PEERS];
    int report_count = 0;

    taskENTER_CRITICAL(&s_peers_mux);
    for (int i = 0; i < LINK_MAX_PEERS; i++) {
        if (!s_peers[i].active || s_peers[i].count < 2) continue;

        float var = compute_variance(s_peers[i].samples, s_peers[i].count);
        reports[report_count++] = (link_report_t){
            .type = 0x01,
            .node_id = s_my_node_id,
            .partner_id = s_peers[i].peer_id,
            .variance = var,
            .state = (var > 0.003f) ? 1 : 0,
            .sample_count = s_peers[i].count,
        };
    }
    taskEXIT_CRITICAL(&s_peers_mux);

    for (int i = 0; i < report_count; i++) {
        sendto(s_sock, &reports[i], sizeof(link_report_t), 0,
               (struct sockaddr *)&s_dest, sizeof(s_dest));
    }
}

void link_reporter_start(uint8_t my_node_id, const char *target_ip,
                         uint16_t target_port, uint32_t interval_ms)
{
    s_my_node_id = my_node_id;
    memset(s_peers, 0, sizeof(s_peers));

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
        .callback = reporter_cb,
        .name = "link_reporter",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, interval_ms * 1000));

    ESP_LOGI(TAG, "Link reporter started: node %d, %s:%d, every %lu ms",
             my_node_id, target_ip, target_port, (unsigned long)interval_ms);
}
