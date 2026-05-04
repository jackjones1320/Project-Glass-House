/**
 * @file edge_processing.c
 * @brief Edge Intelligence — dual-core CSI processing pipeline.
 *
 * Core 0 (WiFi task): Pushes raw CSI frames into lock-free SPSC ring buffer.
 * Core 1 (DSP task):  Pops frames, runs signal processing pipeline:
 *   1. Phase extraction from I/Q pairs
 *   2. Phase unwrapping (continuous phase)
 *   3. Welford variance tracking per subcarrier
 *   4. Top-K subcarrier selection by variance
 *   5. Biquad IIR bandpass → breathing (0.1-0.5 Hz), heart rate (0.8-2.0 Hz)
 *   6. Zero-crossing BPM estimation
 *   7. Presence detection (adaptive or fixed threshold)
 *   8. Fall detection (phase acceleration)
 *   9. Multi-person vitals via subcarrier group clustering
 *  10. Delta compression (XOR + RLE) for bandwidth reduction
 *  11. Vitals packet broadcast (magic 0xC5110002)
 */

#include "edge_processing.h"
#include "nvs_config.h"
#include "stream_sender.h"

/* Runtime config — declared in main.c, loaded from NVS at boot. */
extern nvs_config_t g_nvs_config;

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "edge_proc";

/* ======================================================================
 * SPSC Ring Buffer (lock-free, single-producer single-consumer)
 * ====================================================================== */

static edge_ring_buf_t s_ring;
static uint32_t s_ring_drops;  /* Frames dropped due to full ring buffer. */

/* Scratch buffers for BPM estimation — moved from stack to static to avoid
 * stack overflow.  process_frame + update_multi_person_vitals combined used
 * ~6.5-7.5 KB of the 8 KB task stack.  These save ~4 KB of stack. */
static float s_scratch_br[EDGE_PHASE_HISTORY_LEN];
static float s_scratch_hr[EDGE_PHASE_HISTORY_LEN];

static inline bool ring_push(const uint8_t *iq, uint16_t len,
                             int8_t rssi, uint8_t channel, uint8_t peer_id)
{
    uint32_t next = (s_ring.head + 1) % EDGE_RING_SLOTS;
    if (next == s_ring.tail) {
        s_ring_drops++;
        return false;  /* Full — drop frame. */
    }

    edge_ring_slot_t *slot = &s_ring.slots[s_ring.head];
    uint16_t copy_len = (len > EDGE_MAX_IQ_BYTES) ? EDGE_MAX_IQ_BYTES : len;
    memcpy(slot->iq_data, iq, copy_len);
    slot->iq_len = copy_len;
    slot->rssi = rssi;
    slot->channel = channel;
    slot->peer_id = peer_id;
    slot->timestamp_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    /* Memory barrier: ensure slot data is visible before advancing head. */
    __sync_synchronize();
    s_ring.head = next;
    return true;
}

static inline bool ring_pop(edge_ring_slot_t *out)
{
    if (s_ring.tail == s_ring.head) {
        return false;  /* Empty. */
    }

    memcpy(out, &s_ring.slots[s_ring.tail], sizeof(edge_ring_slot_t));

    __sync_synchronize();
    s_ring.tail = (s_ring.tail + 1) % EDGE_RING_SLOTS;
    return true;
}

/* ======================================================================
 * Biquad IIR Filter
 * ====================================================================== */

/**
 * Design a 2nd-order Butterworth bandpass biquad.
 *
 * @param bq   Output biquad state.
 * @param fs   Sampling frequency (Hz).
 * @param f_lo Low cutoff frequency (Hz).
 * @param f_hi High cutoff frequency (Hz).
 */
static void biquad_bandpass_design(edge_biquad_t *bq, float fs,
                                   float f_lo, float f_hi)
{
    float w0 = 2.0f * M_PI * (f_lo + f_hi) / 2.0f / fs;
    float bw = 2.0f * M_PI * (f_hi - f_lo) / fs;
    float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw / sinf(w0));

    float a0_inv = 1.0f / (1.0f + alpha);
    bq->b0 =  alpha * a0_inv;
    bq->b1 =  0.0f;
    bq->b2 = -alpha * a0_inv;
    bq->a1 = -2.0f * cosf(w0) * a0_inv;
    bq->a2 =  (1.0f - alpha) * a0_inv;

    bq->x1 = bq->x2 = 0.0f;
    bq->y1 = bq->y2 = 0.0f;
}

static inline float biquad_process(edge_biquad_t *bq, float x)
{
    float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2
            - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1;
    bq->x1 = x;
    bq->y2 = bq->y1;
    bq->y1 = y;
    return y;
}

/* ======================================================================
 * Phase Extraction and Unwrapping
 * ====================================================================== */

/** Extract phase (radians) from an I/Q pair at byte offset. */
static inline float extract_phase(const uint8_t *iq, uint16_t idx)
{
    int8_t i_val = (int8_t)iq[idx * 2];
    int8_t q_val = (int8_t)iq[idx * 2 + 1];
    return atan2f((float)q_val, (float)i_val);
}

/** Unwrap phase to maintain continuity (avoid 2*pi jumps). */
static inline float unwrap_phase(float prev, float curr)
{
    float diff = curr - prev;
    if (diff > M_PI)       diff -= 2.0f * M_PI;
    else if (diff < -M_PI) diff += 2.0f * M_PI;
    return prev + diff;
}

/* ======================================================================
 * Welford Running Statistics
 * ====================================================================== */

static inline void welford_reset(edge_welford_t *w)
{
    w->mean = 0.0;
    w->m2   = 0.0;
    w->count = 0;
}

static inline void welford_update(edge_welford_t *w, double x)
{
    w->count++;
    double delta = x - w->mean;
    w->mean += delta / (double)w->count;
    double delta2 = x - w->mean;
    w->m2 += delta * delta2;
}

static inline double welford_variance(const edge_welford_t *w)
{
    return (w->count > 1) ? (w->m2 / (double)(w->count - 1)) : 0.0;
}

/* ======================================================================
 * Zero-Crossing BPM Estimation
 * ====================================================================== */

/**
 * Estimate BPM from a filtered signal using positive zero-crossings.
 *
 * @param history     Signal buffer (filtered phase).
 * @param len         Number of samples.
 * @param sample_rate Sampling rate in Hz.
 * @return Estimated BPM, or 0 if insufficient crossings.
 */
static float estimate_bpm_zero_crossing(const float *history, uint16_t len,
                                        float sample_rate)
{
    if (len < 4) return 0.0f;

    uint16_t crossings[128];
    uint16_t n_cross = 0;

    for (uint16_t i = 1; i < len && n_cross < 128; i++) {
        if (history[i - 1] <= 0.0f && history[i] > 0.0f) {
            crossings[n_cross++] = i;
        }
    }

    if (n_cross < 2) return 0.0f;

    /* Average period from consecutive crossings. */
    float total_period = 0.0f;
    for (uint16_t i = 1; i < n_cross; i++) {
        total_period += (float)(crossings[i] - crossings[i - 1]);
    }
    float avg_period_samples = total_period / (float)(n_cross - 1);

    if (avg_period_samples < 1.0f) return 0.0f;

    float freq_hz = sample_rate / avg_period_samples;
    return freq_hz * 60.0f;  /* Hz to BPM. */
}

/* ======================================================================
 * DSP Pipeline State
 * ====================================================================== */

/** Edge processing configuration. */
static edge_config_t s_cfg;

/** Per-subcarrier running variance (for top-K selection). */
static edge_welford_t s_subcarrier_var[EDGE_MAX_SUBCARRIERS];

/** Previous phase per subcarrier (for unwrapping). */
static float s_prev_phase[EDGE_MAX_SUBCARRIERS];
static bool  s_phase_initialized;

/** Top-K subcarrier indices (sorted by variance, descending). */
static uint8_t s_top_k[EDGE_TOP_K];
static uint8_t s_top_k_count;

/** Phase history for the primary (highest-variance) subcarrier. */
static float s_phase_history[EDGE_PHASE_HISTORY_LEN];
static uint16_t s_history_len;
static uint16_t s_history_idx;

/** Biquad filters for breathing and heart rate. */
static edge_biquad_t s_bq_breathing;
static edge_biquad_t s_bq_heartrate;

/** Filtered signal histories for BPM estimation. */
static float s_breathing_filtered[EDGE_PHASE_HISTORY_LEN];
static float s_heartrate_filtered[EDGE_PHASE_HISTORY_LEN];

/** Cross-core spinlock for vitals state (timer on Core 0, DSP on Core 1). */
static portMUX_TYPE s_vitals_spinlock = portMUX_INITIALIZER_UNLOCKED;

/** Vitals timer handle (1 Hz, runs on Core 0). */
static esp_timer_handle_t s_vitals_timer = NULL;

/** Latest vitals state. */
static float    s_breathing_bpm;
static float    s_heartrate_bpm;
static float    s_motion_energy;
static float    s_presence_score;
static bool     s_presence_detected;
static bool     s_fall_detected;
static int8_t   s_latest_rssi;
static uint32_t s_frame_count;

/** Previous phase velocity for fall detection (acceleration). */
static float s_prev_phase_velocity;

/** Fall detection debounce state. */
static uint8_t  s_fall_consec_count;   /**< Consecutive frames above threshold. */
static int64_t  s_fall_last_alert_us;  /**< Timestamp of last fall alert (debounce). */

/** Adaptive calibration state. */
static bool     s_calibrated;
static float    s_calib_sum;
static float    s_calib_sum_sq;
static uint32_t s_calib_count;
static float    s_adaptive_threshold;

/** Last vitals send timestamp. */
static int64_t s_last_vitals_send_us;

#if CONFIG_GH_PKT_IQ_ENABLE
/** I/Q streaming rate divider counter — see CONFIG_GH_PKT_IQ_DIVIDER for
 *  the configurable threshold (default 5 -> 10 Hz at 50 Hz CSI). */
static uint8_t s_iq_divider = 0;
#endif

/** Multi-person vitals state. */
static edge_person_vitals_t s_persons[EDGE_MAX_PERSONS];
static edge_biquad_t s_person_bq_br[EDGE_MAX_PERSONS];
static edge_biquad_t s_person_bq_hr[EDGE_MAX_PERSONS];
static float s_person_br_filt[EDGE_MAX_PERSONS][EDGE_PHASE_HISTORY_LEN];
static float s_person_hr_filt[EDGE_MAX_PERSONS][EDGE_PHASE_HISTORY_LEN];

/** Latest vitals packet (thread-safe via volatile copy). */
static volatile edge_vitals_pkt_t s_latest_pkt;
static volatile bool s_pkt_valid;

/* ======================================================================
 * Top-K Subcarrier Selection
 * ====================================================================== */

/**
 * Select top-K subcarriers by variance (descending).
 * Uses partial insertion sort — O(n*K) which is fine for n <= 128.
 */
static void update_top_k(uint16_t n_subcarriers)
{
    uint8_t k = s_cfg.top_k_count;
    if (k > EDGE_TOP_K) k = EDGE_TOP_K;
    if (k > n_subcarriers) k = (uint8_t)n_subcarriers;

    /* Simple selection: find K largest variances. */
    bool used[EDGE_MAX_SUBCARRIERS];
    memset(used, 0, sizeof(used));

    for (uint8_t ki = 0; ki < k; ki++) {
        double best_var = -1.0;
        uint8_t best_idx = 0;

        for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
            if (!used[sc]) {
                double v = welford_variance(&s_subcarrier_var[sc]);
                if (v > best_var) {
                    best_var = v;
                    best_idx = (uint8_t)sc;
                }
            }
        }

        s_top_k[ki] = best_idx;
        used[best_idx] = true;
    }

    s_top_k_count = k;
}

/* ======================================================================
 * Adaptive Presence Calibration
 * ====================================================================== */

static void calibration_update(float motion)
{
    if (s_calibrated) return;

    s_calib_sum += motion;
    s_calib_sum_sq += motion * motion;
    s_calib_count++;

    if (s_calib_count >= EDGE_CALIB_FRAMES) {
        float mean = s_calib_sum / (float)s_calib_count;
        float var = (s_calib_sum_sq / (float)s_calib_count) - (mean * mean);
        float sigma = (var > 0.0f) ? sqrtf(var) : 0.001f;

        s_adaptive_threshold = mean + EDGE_CALIB_SIGMA_MULT * sigma;
        if (s_adaptive_threshold < 0.01f) {
            s_adaptive_threshold = 0.01f;
        }

        s_calibrated = true;
        ESP_LOGI(TAG, "Adaptive calibration complete: mean=%.4f sigma=%.4f "
                 "threshold=%.4f (from %lu frames)",
                 mean, sigma, s_adaptive_threshold,
                 (unsigned long)s_calib_count);
    }
}

/* ======================================================================
 * Multi-Person Vitals
 * ====================================================================== */

/**
 * Update multi-person vitals by assigning top-K subcarriers to person groups.
 *
 * Division strategy: top-K subcarriers are evenly divided among
 * up to EDGE_MAX_PERSONS groups. Each group tracks independent
 * phase history and BPM estimation.
 */
static void update_multi_person_vitals(const uint8_t *iq_data, uint16_t n_sc,
                                       float sample_rate)
{
    if (s_top_k_count < 2) return;

    /* Determine number of active persons based on available subcarriers. */
    uint8_t n_persons = s_top_k_count / 2;
    if (n_persons > EDGE_MAX_PERSONS) n_persons = EDGE_MAX_PERSONS;
    if (n_persons < 1) n_persons = 1;

    uint8_t subs_per_person = s_top_k_count / n_persons;

    for (uint8_t p = 0; p < n_persons; p++) {
        edge_person_vitals_t *pv = &s_persons[p];
        pv->active = true;
        pv->subcarrier_idx = s_top_k[p * subs_per_person];

        /* Average phase across this person's subcarrier group. */
        float avg_phase = 0.0f;
        uint8_t count = 0;
        for (uint8_t s = 0; s < subs_per_person; s++) {
            uint8_t sc_idx = s_top_k[p * subs_per_person + s];
            if (sc_idx < n_sc) {
                avg_phase += extract_phase(iq_data, sc_idx);
                count++;
            }
        }
        if (count > 0) avg_phase /= (float)count;

        /* Unwrap and store in history. */
        if (pv->history_len > 0) {
            uint16_t prev_idx = (pv->history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                                % EDGE_PHASE_HISTORY_LEN;
            avg_phase = unwrap_phase(pv->phase_history[prev_idx], avg_phase);
        }

        pv->phase_history[pv->history_idx] = avg_phase;
        pv->history_idx = (pv->history_idx + 1) % EDGE_PHASE_HISTORY_LEN;
        if (pv->history_len < EDGE_PHASE_HISTORY_LEN) pv->history_len++;

        /* Filter and estimate BPM. */
        float br_val = biquad_process(&s_person_bq_br[p], avg_phase);
        float hr_val = biquad_process(&s_person_bq_hr[p], avg_phase);

        uint16_t idx = (pv->history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                       % EDGE_PHASE_HISTORY_LEN;
        s_person_br_filt[p][idx] = br_val;
        s_person_hr_filt[p][idx] = hr_val;

        /* Estimate BPM when we have enough history. */
        if (pv->history_len >= 64) {
            /* Build contiguous buffer (reuse static scratch to save ~2 KB stack). */
            uint16_t buf_len = pv->history_len;

            for (uint16_t i = 0; i < buf_len; i++) {
                uint16_t ri = (pv->history_idx + EDGE_PHASE_HISTORY_LEN
                               - buf_len + i) % EDGE_PHASE_HISTORY_LEN;
                s_scratch_br[i] = s_person_br_filt[p][ri];
                s_scratch_hr[i] = s_person_hr_filt[p][ri];
            }

            float br = estimate_bpm_zero_crossing(s_scratch_br, buf_len, sample_rate);
            float hr = estimate_bpm_zero_crossing(s_scratch_hr, buf_len, sample_rate);

            /* Sanity clamp. */
            if (br >= 6.0f && br <= 40.0f) pv->breathing_bpm = br;
            if (hr >= 40.0f && hr <= 180.0f) pv->heartrate_bpm = hr;
        }
    }

    /* Mark remaining persons as inactive. */
    for (uint8_t p = n_persons; p < EDGE_MAX_PERSONS; p++) {
        s_persons[p].active = false;
    }
}

/* ======================================================================
 * Raw I/Q Streaming Packet (0xC5110006)
 * ====================================================================== */

#if CONFIG_GH_PKT_IQ_ENABLE
/**
 * Send a raw I/Q streaming packet.
 *
 * Header (10 bytes):
 *   [0..3] Magic 0xC5110006 (LE)
 *   [4]    Node ID (receiver, this perimeter)
 *   [5]    Peer ID (source perimeter that emitted the frame this CSI was sampled from)
 *   [6]    Channel
 *   [7]    Reserved (pad to 4-byte align iq_len)
 *   [8..9] I/Q length (LE u16)
 *   [10..] Raw I/Q bytes
 *
 * Host-side parsers detect v2 (this layout) vs v1 (8-byte header)
 * format via iq_len sanity-check at both byte offsets.
 */
static void send_iq_packet(const uint8_t *iq_data, uint16_t iq_len,
                           uint8_t channel, uint8_t peer_id)
{
    uint16_t safe_len = (iq_len > EDGE_MAX_IQ_BYTES) ? EDGE_MAX_IQ_BYTES : iq_len;

    uint8_t pkt[10 + EDGE_MAX_IQ_BYTES];

    uint32_t magic = EDGE_IQ_MAGIC;
    memcpy(&pkt[0], &magic, 4);
    pkt[4] = g_nvs_config.node_id;
    pkt[5] = peer_id;
    pkt[6] = channel;
    pkt[7] = 0;  /* reserved padding */
    memcpy(&pkt[8], &safe_len, 2);
    memcpy(&pkt[10], iq_data, safe_len);

    stream_sender_send(pkt, 10 + safe_len);
}
#endif /* CONFIG_GH_PKT_IQ_ENABLE */

/* ======================================================================
 * Vitals Timer Callback (runs on Core 0 at 1 Hz)
 * ====================================================================== */

static void vitals_timer_cb(void *arg)
{
    (void)arg;

    /* Snapshot vitals state under spinlock (Core 0 timer vs Core 1 DSP). */
    float motion, presence_score, br_bpm, hr_bpm;
    bool presence, fall;
    int8_t rssi;
    edge_person_vitals_t persons[EDGE_MAX_PERSONS];

    portENTER_CRITICAL(&s_vitals_spinlock);
    motion = s_motion_energy;
    presence_score = s_presence_score;
    presence = s_presence_detected;
    fall = s_fall_detected;
    br_bpm = s_breathing_bpm;
    hr_bpm = s_heartrate_bpm;
    rssi = s_latest_rssi;
    memcpy(persons, (const void *)s_persons, sizeof(persons));
    portEXIT_CRITICAL(&s_vitals_spinlock);

    /* Build vitals packet from snapshot. */
    edge_vitals_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = EDGE_VITALS_MAGIC;
    pkt.node_id = g_nvs_config.node_id;

    pkt.flags = 0;
    if (presence)          pkt.flags |= 0x01;
    if (fall)              pkt.flags |= 0x02;
    if (motion > 0.01f)    pkt.flags |= 0x04;

    pkt.breathing_rate = (uint16_t)(br_bpm * 100.0f);
    pkt.heartrate = (uint32_t)(hr_bpm * 10000.0f);
    pkt.rssi = rssi;

    uint8_t n_active = 0;
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        if (persons[p].active) n_active++;
    }
    pkt.n_persons = n_active;

    pkt.motion_energy = motion;
    pkt.presence_score = presence_score;
    pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* Update thread-safe copy under spinlock. */
    portENTER_CRITICAL(&s_vitals_spinlock);
    s_latest_pkt = pkt;
    s_pkt_valid = true;
    portEXIT_CRITICAL(&s_vitals_spinlock);

    /* Send vitals packet (outside spinlock — sendto can block). Wire
     * emission is gated by CONFIG_GH_PKT_VITALS_ENABLE; on-device vitals
     * computation continues regardless so s_latest_pkt stays current. */
#if CONFIG_GH_PKT_VITALS_ENABLE
    stream_sender_send((const uint8_t *)&pkt, sizeof(pkt));
#endif /* CONFIG_GH_PKT_VITALS_ENABLE */

    /* Periodic vitals log (~10s cadence). */
    static uint8_t s_log_counter = 0;
    if (++s_log_counter >= 10) {
        s_log_counter = 0;
        ESP_LOGI(TAG, "Vitals: br=%.1f hr=%.1f motion=%.4f pres=%s",
                 br_bpm, hr_bpm, motion, presence ? "YES" : "no");
    }
}

/* ======================================================================
 * Main DSP Pipeline (runs on Core 1)
 * ====================================================================== */

static void process_frame(const edge_ring_slot_t *slot)
{
    uint16_t n_subcarriers = slot->iq_len / 2;
    if (n_subcarriers == 0 || n_subcarriers > EDGE_MAX_SUBCARRIERS) return;

    s_frame_count++;
    portENTER_CRITICAL(&s_vitals_spinlock);
    s_latest_rssi = slot->rssi;
    portEXIT_CRITICAL(&s_vitals_spinlock);

    /* Assumed CSI sample rate (~20 Hz for typical ESP32 CSI). */
    const float sample_rate = 20.0f;

    /* --- Step 1-2: Phase extraction + unwrapping per subcarrier --- */
    float phases[EDGE_MAX_SUBCARRIERS];
    for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
        float raw_phase = extract_phase(slot->iq_data, sc);

        if (s_phase_initialized) {
            phases[sc] = unwrap_phase(s_prev_phase[sc], raw_phase);
        } else {
            phases[sc] = raw_phase;
        }
        s_prev_phase[sc] = phases[sc];
    }
    s_phase_initialized = true;

    /* --- Step 3: Welford variance update per subcarrier --- */
    for (uint16_t sc = 0; sc < n_subcarriers; sc++) {
        welford_update(&s_subcarrier_var[sc], (double)phases[sc]);
    }

    /* --- Step 4: Top-K selection (every 100 frames to amortize cost) --- */
    if ((s_frame_count % 100) == 1 || s_top_k_count == 0) {
        update_top_k(n_subcarriers);
    }

    if (s_top_k_count == 0) return;

    /* --- Step 5: Phase of primary (highest-variance) subcarrier --- */
    float primary_phase = phases[s_top_k[0]];

    /* Store in phase history ring buffer. */
    s_phase_history[s_history_idx] = primary_phase;
    s_history_idx = (s_history_idx + 1) % EDGE_PHASE_HISTORY_LEN;
    if (s_history_len < EDGE_PHASE_HISTORY_LEN) s_history_len++;

    /* --- Step 6: Biquad bandpass filtering --- */
    float br_val = biquad_process(&s_bq_breathing, primary_phase);
    float hr_val = biquad_process(&s_bq_heartrate, primary_phase);

    uint16_t filt_idx = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 1)
                        % EDGE_PHASE_HISTORY_LEN;
    s_breathing_filtered[filt_idx] = br_val;
    s_heartrate_filtered[filt_idx] = hr_val;

    /* --- Step 7: BPM estimation (zero-crossing) --- */
    if (s_history_len >= 64) {
        /* Build contiguous buffers from ring (using static scratch to save stack). */
        uint16_t buf_len = s_history_len;

        for (uint16_t i = 0; i < buf_len; i++) {
            uint16_t ri = (s_history_idx + EDGE_PHASE_HISTORY_LEN
                           - buf_len + i) % EDGE_PHASE_HISTORY_LEN;
            s_scratch_br[i] = s_breathing_filtered[ri];
            s_scratch_hr[i] = s_heartrate_filtered[ri];
        }

        float br_bpm = estimate_bpm_zero_crossing(s_scratch_br, buf_len, sample_rate);
        float hr_bpm = estimate_bpm_zero_crossing(s_scratch_hr, buf_len, sample_rate);

        /* Sanity clamp: breathing 6-40 BPM, heart rate 40-180 BPM. */
        portENTER_CRITICAL(&s_vitals_spinlock);
        if (br_bpm >= 6.0f && br_bpm <= 40.0f) s_breathing_bpm = br_bpm;
        if (hr_bpm >= 40.0f && hr_bpm <= 180.0f) s_heartrate_bpm = hr_bpm;
        portEXIT_CRITICAL(&s_vitals_spinlock);
    }

    /* --- Step 8: Motion energy (variance of recent phases) --- */
    if (s_history_len >= 10) {
        float sum = 0.0f, sum2 = 0.0f;
        uint16_t window = (s_history_len < 20) ? s_history_len : 20;
        for (uint16_t i = 0; i < window; i++) {
            uint16_t ri = (s_history_idx + EDGE_PHASE_HISTORY_LEN
                           - window + i) % EDGE_PHASE_HISTORY_LEN;
            float v = s_phase_history[ri];
            sum += v;
            sum2 += v * v;
        }
        float mean = sum / (float)window;
        float motion = (sum2 / (float)window) - (mean * mean);
        if (motion < 0.0f) motion = 0.0f;
        portENTER_CRITICAL(&s_vitals_spinlock);
        s_motion_energy = motion;
        portEXIT_CRITICAL(&s_vitals_spinlock);
    }

    /* --- Step 9: Presence detection --- */
    s_presence_score = s_motion_energy;

    /* Adaptive calibration: learn ambient noise level from first N frames. */
    if (!s_calibrated && s_cfg.presence_thresh == 0.0f) {
        calibration_update(s_motion_energy);
    }

    float threshold = s_cfg.presence_thresh;
    if (threshold == 0.0f && s_calibrated) {
        threshold = s_adaptive_threshold;
    } else if (threshold == 0.0f) {
        threshold = 0.05f;  /* Default until calibrated. */
    }
    portENTER_CRITICAL(&s_vitals_spinlock);
    s_presence_detected = (s_presence_score > threshold);
    portEXIT_CRITICAL(&s_vitals_spinlock);

    /* --- Step 10: Fall detection (phase acceleration + debounce) --- */
    if (s_history_len >= 3) {
        uint16_t i0 = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 1) % EDGE_PHASE_HISTORY_LEN;
        uint16_t i1 = (s_history_idx + EDGE_PHASE_HISTORY_LEN - 2) % EDGE_PHASE_HISTORY_LEN;
        float velocity = s_phase_history[i0] - s_phase_history[i1];
        float accel = fabsf(velocity - s_prev_phase_velocity);
        s_prev_phase_velocity = velocity;

        if (accel > s_cfg.fall_thresh) {
            s_fall_consec_count++;
        } else {
            s_fall_consec_count = 0;
        }

        /* Require EDGE_FALL_CONSEC_MIN consecutive frames above threshold,
         * plus a cooldown period to prevent alert storms. */
        int64_t now_us = esp_timer_get_time();
        int64_t cooldown_us = (int64_t)EDGE_FALL_COOLDOWN_MS * 1000;
        if (s_fall_consec_count >= EDGE_FALL_CONSEC_MIN
            && (now_us - s_fall_last_alert_us) >= cooldown_us)
        {
            portENTER_CRITICAL(&s_vitals_spinlock);
            s_fall_detected = true;
            portEXIT_CRITICAL(&s_vitals_spinlock);
            s_fall_last_alert_us = now_us;
            s_fall_consec_count = 0;
            ESP_LOGW(TAG, "Fall detected! accel=%.4f > thresh=%.4f (consec=%u)",
                     accel, s_cfg.fall_thresh, EDGE_FALL_CONSEC_MIN);
        } else if (s_fall_consec_count == 0) {
            portENTER_CRITICAL(&s_vitals_spinlock);
            s_fall_detected = false;
            portEXIT_CRITICAL(&s_vitals_spinlock);
        }
    }

    /* --- Step 11: Multi-person vitals --- */
    update_multi_person_vitals(slot->iq_data, n_subcarriers, sample_rate);

    /* --- Step 12: Raw I/Q streaming (~50/N Hz via CONFIG_GH_PKT_IQ_DIVIDER) --- */
#if CONFIG_GH_PKT_IQ_ENABLE
    if (++s_iq_divider >= CONFIG_GH_PKT_IQ_DIVIDER) {
        s_iq_divider = 0;
        send_iq_packet(slot->iq_data, slot->iq_len, slot->channel,
                       slot->peer_id);
    }
#endif

}

/* ======================================================================
 * Edge Processing Task (pinned to Core 1)
 * ====================================================================== */

static void edge_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Edge DSP task started on core %d (tier=%u)",
             xPortGetCoreID(), s_cfg.tier);

    edge_ring_slot_t slot;

    /* Maximum frames to process before a longer yield.  On busy LANs
     * (corporate networks, many APs), the ring buffer fills continuously.
     * Without a batch limit the task processes frames back-to-back with
     * only 1-tick yields, which on high frame rates can still starve
     * IDLE1 enough to trip the 5-second task watchdog.  See #266, #321. */

    while (1) {
        uint8_t processed = 0;

        while (processed < EDGE_BATCH_LIMIT && ring_pop(&slot)) {
            process_frame(&slot);
            processed++;
            /* 1-tick yield between frames within a batch. */
            vTaskDelay(1);
        }

        if (processed > 0) {
            /* Post-batch yield: ~20 ms so IDLE1 can run and feed the
             * Core 1 watchdog even under sustained load.  Uses pdMS_TO_TICKS
             * for tick-rate independence (minimum 1 tick). */
            { TickType_t d = pdMS_TO_TICKS(20); vTaskDelay(d > 0 ? d : 1); }
        } else {
            /* No frames available — sleep one full tick.
             * NOTE: pdMS_TO_TICKS(5) == 0 at 100 Hz, which would busy-spin. */
            vTaskDelay(1);
        }
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

bool edge_enqueue_csi(const uint8_t *iq_data, uint16_t iq_len,
                      int8_t rssi, uint8_t channel, uint8_t peer_id)
{
    return ring_push(iq_data, iq_len, rssi, channel, peer_id);
}

bool edge_get_vitals(edge_vitals_pkt_t *pkt)
{
    if (!s_pkt_valid || pkt == NULL) return false;
    memcpy(pkt, (const void *)&s_latest_pkt, sizeof(edge_vitals_pkt_t));
    return true;
}

void edge_get_multi_person(edge_person_vitals_t *persons, uint8_t *n_active)
{
    uint8_t active = 0;
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        if (persons) persons[p] = s_persons[p];
        if (s_persons[p].active) active++;
    }
    if (n_active) *n_active = active;
}

esp_err_t edge_processing_init(const edge_config_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(TAG, "edge_processing_init: cfg is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* Store config. */
    s_cfg = *cfg;

    ESP_LOGI(TAG, "Initializing edge processing (tier=%u, top_k=%u, "
             "vital_interval=%ums, presence_thresh=%.3f)",
             s_cfg.tier, s_cfg.top_k_count,
             s_cfg.vital_interval_ms, s_cfg.presence_thresh);

    /* Reset all state. */
    memset(&s_ring, 0, sizeof(s_ring));
    memset(s_subcarrier_var, 0, sizeof(s_subcarrier_var));
    memset(s_prev_phase, 0, sizeof(s_prev_phase));
    s_phase_initialized = false;
    s_top_k_count = 0;
    s_history_len = 0;
    s_history_idx = 0;
    s_breathing_bpm = 0.0f;
    s_heartrate_bpm = 0.0f;
    s_motion_energy = 0.0f;
    s_presence_score = 0.0f;
    s_presence_detected = false;
    s_fall_detected = false;
    s_latest_rssi = 0;
    s_frame_count = 0;
    s_prev_phase_velocity = 0.0f;
    s_fall_consec_count = 0;
    s_fall_last_alert_us = 0;
    s_last_vitals_send_us = 0;
    s_pkt_valid = false;

    /* Reset calibration state. */
    s_calibrated = false;
    s_calib_sum = 0.0f;
    s_calib_sum_sq = 0.0f;
    s_calib_count = 0;
    s_adaptive_threshold = 0.05f;

    /* Reset multi-person state. */
    memset(s_persons, 0, sizeof(s_persons));
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        s_persons[p].active = false;
    }

    /* Design biquad bandpass filters.
     * Sampling rate ~20 Hz (typical ESP32 CSI callback rate). */
    const float fs = 20.0f;
    biquad_bandpass_design(&s_bq_breathing, fs, 0.1f, 0.5f);
    biquad_bandpass_design(&s_bq_heartrate, fs, 0.8f, 2.0f);

    /* Design per-person filters. */
    for (uint8_t p = 0; p < EDGE_MAX_PERSONS; p++) {
        biquad_bandpass_design(&s_person_bq_br[p], fs, 0.1f, 0.5f);
        biquad_bandpass_design(&s_person_bq_hr[p], fs, 0.8f, 2.0f);
    }

    /* Start vitals timer (1 Hz) — runs at ALL tiers including tier 0. */
    if (s_vitals_timer != NULL) {
        esp_timer_stop(s_vitals_timer);
        esp_timer_delete(s_vitals_timer);
        s_vitals_timer = NULL;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = vitals_timer_cb,
        .name = "vitals_1hz",
    };
    esp_err_t terr = esp_timer_create(&timer_args, &s_vitals_timer);
    if (terr == ESP_OK) {
        esp_timer_start_periodic(s_vitals_timer,
                                 (uint64_t)s_cfg.vital_interval_ms * 1000);
        ESP_LOGI(TAG, "Vitals timer started (%u ms period)",
                 s_cfg.vital_interval_ms);
    } else {
        ESP_LOGE(TAG, "Failed to create vitals timer: %s", esp_err_to_name(terr));
    }

    if (s_cfg.tier == 0) {
        ESP_LOGI(TAG, "Edge tier 0: raw passthrough (no DSP task)");
        return ESP_OK;
    }

    /* Start DSP task on Core 1. */
    BaseType_t ret = xTaskCreatePinnedToCore(
        edge_task,
        "edge_dsp",
        8192,       /* 8 KB stack — sufficient for DSP pipeline. */
        NULL,
        5,          /* Priority 5 — above idle, below WiFi. */
        NULL,
        1           /* Pin to Core 1. */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create edge DSP task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Edge DSP task created on Core 1 (stack=8192, priority=5)");
    return ESP_OK;
}
