/**
 * @file crash_counter.c
 * @brief NVS-backed persistent crash counter implementation.
 *
 * See crash_counter.h for the contract.
 */

#include "crash_counter.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "crash_counter";

#define NVS_NAMESPACE  "diag"
#define NVS_KEY_COUNT  "crash_n"

static uint16_t s_cached = 0;
static bool     s_ready  = false;

esp_err_t crash_counter_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(%s) failed: %s — counter stays 0",
                 NVS_NAMESPACE, esp_err_to_name(err));
        s_cached = 0;
        return ESP_FAIL;
    }

    uint16_t prev = 0;
    esp_err_t rerr = nvs_get_u16(h, NVS_KEY_COUNT, &prev);
    if (rerr != ESP_OK && rerr != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "nvs_get_u16 failed: %s — treating as 0",
                 esp_err_to_name(rerr));
        prev = 0;
    }

    esp_reset_reason_t reason = esp_reset_reason();
    uint16_t next;
    if (reason == ESP_RST_POWERON) {
        next = 0;
    } else if (prev == UINT16_MAX) {
        next = UINT16_MAX;  /* saturate */
    } else {
        next = (uint16_t)(prev + 1);
    }

    err = nvs_set_u16(h, NVS_KEY_COUNT, next);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs write/commit failed: %s — using in-memory value",
                 esp_err_to_name(err));
    }

    s_cached = next;
    s_ready  = true;
    ESP_LOGI(TAG, "boot reset_reason=%d crash_count %u -> %u",
             (int)reason, (unsigned)prev, (unsigned)next);
    return ESP_OK;
}

uint16_t crash_counter_get(void)
{
    return s_cached;
}
