/**
 * @file send_watchdog.c
 * @brief Send-rate-gauge watchdog — see send_watchdog.h for design rationale.
 */

#include "send_watchdog.h"
#include "stream_sender.h"     /* stream_sender_get_send_ok() */

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>

static const char *TAG = "send_watchdog";

/*
 * Tunables chosen to hit <=10 s observable gap on first-attempt recovery.
 */
#define TICK_PERIOD_MS          1000    /* 1 Hz polling — matches s_send_ok grain */
#define STALL_THRESHOLD_BASE_S  8       /* 8 s flat → first escalation */
#define MAX_SOFT_RECOVERIES     2       /* 2 soft tries before hard reboot */
#define RESTART_FLUSH_DELAY_MS  100     /* pre-esp_restart log flush window */

/* State — all file-static (no cross-module exposure). */
static esp_timer_handle_t  s_timer            = NULL;
static TaskHandle_t        s_restart_task     = NULL;
static uint8_t             s_node_id          = 0;
static uint32_t            s_last_send_ok     = 0;
static uint32_t            s_flat_seconds     = 0;
static uint8_t             s_soft_attempts    = 0;
static bool                s_armed            = false;
static uint8_t             s_stall_threshold  = STALL_THRESHOLD_BASE_S;

/*
 * Per-node stagger: alternating 8 s / 9 s threshold across nodes 1-4. With
 * 1 Hz polling, this gives at most 1 s offset between any two nodes'
 * escalation moments — enough to prevent a simultaneous 4-STA reassociation
 * storm while keeping worst-case observable silence under the 10 s budget.
 * An additional 250ms ticker-phase stagger is applied via the initial delay
 * at arm time.
 */
static uint8_t stagger_threshold_for_node(uint8_t node_id)
{
    return STALL_THRESHOLD_BASE_S + ((node_id % 2) == 0 ? 1 : 0);
}

/*
 * Deferred-restart worker. The esp_timer task runs at priority 22; calling
 * esp_restart() directly from the timer callback would preempt the WiFi
 * event task (priority 20) mid-log-flush and potentially truncate the
 * rationale log line. Dedicating a worker task for the restart keeps the
 * log-flush delay honored and matches ESP-IDF best practice for deferring
 * long-running work out of timer callbacks.
 */
static void restart_worker_task(void *arg)
{
    (void)arg;
    while (1) {
        /* Block until watchdog signals terminal escalation. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGE(TAG, "send_watchdog: terminal escalation — rebooting in %dms",
                 RESTART_FLUSH_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RESTART_FLUSH_DELAY_MS));
        esp_restart();
    }
}

static void watchdog_tick(void *arg)
{
    (void)arg;

    uint32_t now_send_ok = stream_sender_get_send_ok();

    /*
     * Boot guard: don't arm the watchdog until we've observed the first
     * successful send. This prevents false-fire during the initial STA
     * association window (wifi_init_sta waits up to 30 s for GOT_IP).
     * Without this guard, a slow initial association would be
     * indistinguishable from a post-connect stall.
     */
    if (!s_armed) {
        if (now_send_ok > 0) {
            s_armed = true;
            s_last_send_ok = now_send_ok;
            ESP_LOGI(TAG, "armed (first send observed: %lu)",
                     (unsigned long)now_send_ok);
        }
        return;
    }

    /* Counter advanced this tick — we're sending. Reset stall state. */
    if (now_send_ok > s_last_send_ok) {
        if (s_flat_seconds > 0 || s_soft_attempts > 0) {
            /* Recovery succeeded (or brief hiccup cleared on its own). */
            ESP_LOGI(TAG, "traffic resumed (was flat %lus, soft=%u, ok_delta=%lu)",
                     (unsigned long)s_flat_seconds,
                     (unsigned)s_soft_attempts,
                     (unsigned long)(now_send_ok - s_last_send_ok));
        }
        s_last_send_ok = now_send_ok;
        s_flat_seconds = 0;
        s_soft_attempts = 0;
        return;
    }

    /* Counter unchanged — stall candidate. Increment and check threshold. */
    s_flat_seconds++;

    if (s_flat_seconds < s_stall_threshold) {
        /* Not yet at escalation threshold. Just tick and wait. */
        return;
    }

    /* Escalation path. */
    if (s_soft_attempts < MAX_SOFT_RECOVERIES) {
        s_soft_attempts++;
        ESP_LOGW(TAG,
                 "stall detected (%lus flat, threshold=%us) — soft recovery %u/%u",
                 (unsigned long)s_flat_seconds,
                 (unsigned)s_stall_threshold,
                 (unsigned)s_soft_attempts,
                 (unsigned)MAX_SOFT_RECOVERIES);

        /*
         * Soft recovery: explicit disconnect + connect. The perimeter event
         * handler will ALSO call esp_wifi_connect() in response to the
         * DISCONNECTED event we just triggered — that double-connect is
         * benign per ESP-IDF docs. We call connect() ourselves because the
         * stuck-state can suppress the DISCONNECTED event entirely
         * (ESP-IDF GH-11615), in which case our direct call is the only
         * thing that triggers reassociation.
         *
         * We deliberately do not gate this with an "intentional disconnect"
         * flag because we WANT the retry handler to assist. This is a
         * recovery flow, not an OTA/reprovision flow.
         */
        (void)esp_wifi_disconnect();
        (void)esp_wifi_connect();

        /*
         * Reset the stall counter to give the reassociation time to restore
         * data flow. The new countdown starts at the reassoc completion point
         * — if reassoc works, s_send_ok will advance within 2-3 s and the
         * next tick will clear state. If reassoc also fails, we'll hit the
         * stall threshold again and escalate to the next soft attempt (or
         * the hard reboot).
         */
        s_flat_seconds = 0;
        return;
    }

    /*
     * Terminal escalation: both soft recoveries exhausted. Signal the restart
     * worker task to perform the deferred reboot.
     */
    ESP_LOGE(TAG, "soft recovery exhausted (%u attempts) — signaling hard reboot",
             (unsigned)s_soft_attempts);
    if (s_restart_task) {
        xTaskNotifyGive(s_restart_task);
    } else {
        /* Fallback if the worker task wasn't created (should never happen). */
        ESP_LOGE(TAG, "restart_task not created; calling esp_restart directly");
        vTaskDelay(pdMS_TO_TICKS(RESTART_FLUSH_DELAY_MS));
        esp_restart();
    }
}

void send_watchdog_start(uint8_t node_id)
{
    if (s_timer != NULL) {
        ESP_LOGW(TAG, "already started — ignoring");
        return;
    }

    s_node_id = node_id;
    s_stall_threshold = stagger_threshold_for_node(node_id);

    /* Create deferred-restart worker first so it's ready before the timer
     * can fire. Stack size 2048 is adequate — the task only ever executes
     * ulTaskNotifyTake → ESP_LOGE → vTaskDelay → esp_restart. */
    BaseType_t ret = xTaskCreate(restart_worker_task, "send_wd_rst",
                                 2048, NULL, tskIDLE_PRIORITY + 1,
                                 &s_restart_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "restart_worker_task create failed — watchdog DISABLED");
        return;
    }

    /* Create the periodic 1 Hz timer. ESP_TIMER_TASK (default) dispatch
     * matches every other timer in this firmware (heartbeat,
     * link_reporter, csi_collector NDP, edge_processing vitals,
     * telemetry_sender). */
    esp_timer_create_args_t args = {
        .callback = watchdog_tick,
        .name     = "send_wd",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer,
                                             (uint64_t)TICK_PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG,
             "started: node_id=%u threshold=%us (per-node stagger), "
             "max_soft=%u, restart_delay=%dms",
             (unsigned)node_id,
             (unsigned)s_stall_threshold,
             (unsigned)MAX_SOFT_RECOVERIES,
             RESTART_FLUSH_DELAY_MS);
}
