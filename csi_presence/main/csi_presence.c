/*
 * csi_presence -- "is there a person in the room" from WiFi CSI.
 *
 * A separate, standalone project from csi_test. It reuses two things we
 * already proved out there:
 *   - the sdkconfig fix (CONFIG_ESP_WIFI_CSI_ENABLED)
 *   - the queue + low-priority-task split that keeps CSI printing off the
 *     WiFi driver's task, so it can't starve the idle task and trip the
 *     watchdog (see csi_test's docs/v1.md §4.2 for the story)
 *
 * The presence-detection algorithm itself is a fresh implementation,
 * adapted from the *approach* used in ruvnet/RuView's
 * firmware/esp32-csi-node/main/edge_processing.c (MIT licensed):
 *   motion energy = variance, over a short time window, of the phase of
 *   whichever single subcarrier is currently the noisiest -- rather than
 *   snapshotting spread-across-subcarriers-in-one-packet the way
 *   csi_test's report does it. See this project's docs/v1.md for why that
 *   distinction matters and what was changed vs. the source approach
 *   (notably: a time-based calibration window instead of a frame-count
 *   one, since this network's packet rate is nowhere near RuView's
 *   assumed steady 20 Hz).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "wifi_credentials.h"

/* ------------------------------------------------------------------ */
/* Tuning knobs                                                        */
/* ------------------------------------------------------------------ */

#define CSI_RECORD_MAX_LEN     512   /* bytes; up to 256 subcarriers */
#define MAX_SUBCARRIERS        (CSI_RECORD_MAX_LEN / 2)
#define CSI_QUEUE_DEPTH        16

#define PHASE_HISTORY_LEN      32    /* ring buffer size */
#define MOTION_WINDOW          20    /* samples used for the variance window */

/*
 * The noisiest-subcarrier pick happens once, after this many warmup
 * frames, and is then kept for the session -- see the fixed-subcarrier
 * note where this is consumed, below, for why it's not periodic.
 *
 * It can't be the full calibration window either: at this network's
 * actual (sparse, irregular) packet rate, waiting that long left
 * s_primary_subcarrier sitting at its default of index 0 for a very long
 * time. Verified on real hardware (2026-08-16 test flash): index 0 is
 * consistently a near-dead edge subcarrier (~0 amplitude, see
 * csi_test/docs/v1.md), so a flat zero-variance phase signal was feeding
 * "motion energy" the whole time -- it could never detect motion, ever,
 * regardless of what happened in the room.
 */
#define TOPK_WARMUP_FRAMES     20

/*
 * Calibration is time-based, not frame-count-based. RuView assumes a
 * steady ~20 Hz capture rate (1200 frames = 60s). Our own capture of this
 * exact network showed wildly irregular gaps (0.2s to 2.6s between
 * packets -- see csi_test/docs/v1.md's capture-analysis section), so a
 * fixed frame count would take an unpredictable amount of wall-clock time.
 * Watching the clock instead makes the calibration window's *meaning*
 * (60 seconds of ambient observation) hold regardless of traffic rate.
 */
#define CALIB_SECONDS           60.0
#define CALIB_MIN_SAMPLES       10     /* don't trust a threshold from too few samples */

#define PRESENCE_SIGMA_MULT     3.0f   /* threshold = ambient mean + N * ambient stddev */
#define PRESENCE_HYST_RATIO     0.5f   /* clear-threshold = enter-threshold * this */
#define PRESENCE_CLEAR_FRAMES   5      /* consecutive low frames before clearing presence */
#define PRESENCE_DEFAULT_THRESH 0.05f  /* used only before calibration completes */

static const char *TAG = "CSI_PRESENCE";

/* ------------------------------------------------------------------ */
/* WiFi + CSI plumbing (same shape as csi_test)                        */
/* ------------------------------------------------------------------ */

static QueueHandle_t csi_queue = NULL;
static uint32_t packet_count = 0;
static uint32_t dropped_count = 0;

typedef struct {
    int64_t timestamp_us;
    int8_t rssi;
    uint8_t channel;
    uint16_t len;
    bool first_word_invalid;
    int8_t buf[CSI_RECORD_MAX_LEN];
} csi_record_t;

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* Fast path: runs in the WiFi driver's task. No printf, no math -- just
 * copy and hand off. See csi_test/docs/v1.md for why this matters. */
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *data)
{
    if (data == NULL || data->buf == NULL) {
        return;
    }

    packet_count++;

    csi_record_t record;
    record.timestamp_us = esp_timer_get_time();
    record.rssi = data->rx_ctrl.rssi;
    record.channel = data->rx_ctrl.channel;
    record.first_word_invalid = data->first_word_invalid;
    record.len = (data->len < CSI_RECORD_MAX_LEN) ? data->len : CSI_RECORD_MAX_LEN;

    memcpy(record.buf, data->buf, record.len);

    if (xQueueSend(csi_queue, &record, 0) != pdTRUE) {
        dropped_count++;
    }
}

/* ------------------------------------------------------------------ */
/* Presence detection state (all owned by csi_presence_task)           */
/* ------------------------------------------------------------------ */

/* Phase unwrapping, per subcarrier index. */
static float s_prev_phase[MAX_SUBCARRIERS];
static bool  s_have_prev_phase[MAX_SUBCARRIERS];
static int   s_prev_num_subcarriers = -1;

/* Welford running variance per subcarrier, accumulated only during the
 * warmup window and used for exactly one noisiest-subcarrier pick -- see
 * the v1 fixed-subcarrier note where this is consumed, below. */
static double s_sc_mean[MAX_SUBCARRIERS];
static double s_sc_m2[MAX_SUBCARRIERS];
static uint32_t s_sc_count[MAX_SUBCARRIERS];
static uint32_t s_frames_since_reeval = 0;
static int s_primary_subcarrier = 0;
static bool s_did_first_reeval = false;

/* Ring buffer of the primary subcarrier's unwrapped phase. */
static float s_phase_history[PHASE_HISTORY_LEN];
static uint16_t s_history_idx = 0;
static uint16_t s_history_len = 0;

/* Adaptive calibration (time-based -- see CALIB_SECONDS comment above). */
static int64_t s_first_frame_us = -1;
static double s_calib_sum = 0.0;
static double s_calib_sum_sq = 0.0;
static uint32_t s_calib_count = 0;
static bool s_calibrated = false;
static float s_adaptive_threshold = 0.0f;

/* Presence flag + hysteresis/debounce state. */
static float s_motion_energy = 0.0f;
static bool s_presence_detected = false;
static uint8_t s_below_count = 0;

static inline float unwrap_phase(float prev, float curr)
{
    float diff = curr - prev;
    while (diff > (float)M_PI)  diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return prev + diff;
}

static void welford_update(int idx, double x)
{
    s_sc_count[idx]++;
    double delta = x - s_sc_mean[idx];
    s_sc_mean[idx] += delta / (double)s_sc_count[idx];
    double delta2 = x - s_sc_mean[idx];
    s_sc_m2[idx] += delta * delta2;
}

static double welford_variance(int idx)
{
    if (s_sc_count[idx] < 2) return 0.0;
    return s_sc_m2[idx] / (double)(s_sc_count[idx] - 1);
}

/*
 * Schmitt-trigger presence decision with a clear-debounce. Same shape as
 * RuView's presence_flag_update -- this bit is a standard control-systems
 * technique (hysteresis + debounce), not project-specific IP, but written
 * fresh here rather than copied.
 */
static bool presence_flag_update(bool prev, float score, float threshold, uint8_t *below_count)
{
    float low_thresh = threshold * PRESENCE_HYST_RATIO;

    if (score > threshold) {
        *below_count = 0;
        return true;
    }
    if (score >= low_thresh) {
        *below_count = 0;
        return prev;
    }
    if (*below_count < 0xFF) (*below_count)++;
    if (!prev) return false;
    if (*below_count >= PRESENCE_CLEAR_FRAMES) {
        *below_count = 0;
        return false;
    }
    return true;
}

static void process_record(const csi_record_t *record)
{
    int start = record->first_word_invalid ? 4 : 0;
    int num_subcarriers = (record->len - start) / 2;
    if (num_subcarriers < 1) return;
    if (num_subcarriers > MAX_SUBCARRIERS) num_subcarriers = MAX_SUBCARRIERS;

    /* If the packet shape changed (e.g. a non-HT frame vs. an HT frame --
     * see csi_test/docs/v1.md §6 on mixed subcarrier counts), the index
     * meaning isn't comparable to the previous frame. Drop unwrap
     * continuity for this frame rather than unwrap across a discontinuity
     * that isn't real motion. Known limitation: this produces a one-frame
     * gap in phase history whenever the packet type switches. */
    bool shape_changed = (num_subcarriers != s_prev_num_subcarriers);
    if (shape_changed) {
        memset(s_have_prev_phase, 0, sizeof(s_have_prev_phase));
        s_prev_num_subcarriers = num_subcarriers;
    }

    float phases[MAX_SUBCARRIERS];
    for (int i = 0; i < num_subcarriers; i++) {
        int idx = start + (i * 2);
        float im = (float)record->buf[idx];
        float re = (float)record->buf[idx + 1];
        float raw_phase = atan2f(im, re);

        float unwrapped = s_have_prev_phase[i]
            ? unwrap_phase(s_prev_phase[i], raw_phase)
            : raw_phase;

        phases[i] = unwrapped;
        s_prev_phase[i] = unwrapped;
        s_have_prev_phase[i] = true;

        welford_update(i, (double)unwrapped);
    }

    /* Pick the noisiest subcarrier ONCE, after a short warmup, then keep it
     * for the rest of the session (until reboot). v1 originally re-picked
     * every TOPK_REEVAL_FRAMES to track "recent behaviour" -- verified on
     * real hardware (2026-08-16) that this is actively harmful: the
     * calibration accumulator (below) sums motion_energy across the whole
     * calibration window regardless of which subcarrier produced it, so a
     * mid-calibration reselection mixes two unrelated subcarriers' noise
     * levels into one meaningless "ambient baseline". A single fixed pick
     * keeps the calibrated threshold coherent. Adaptive re-tracking (with
     * a coordinated recalibration when it fires) is a reasonable v2, not
     * a silent periodic swap. */
    if (!s_did_first_reeval) {
        s_frames_since_reeval++;
        if (s_frames_since_reeval >= TOPK_WARMUP_FRAMES) {
            double best_var = -1.0;
            int best_idx = 0;
            for (int i = 0; i < num_subcarriers; i++) {
                double v = welford_variance(i);
                if (v > best_var) {
                    best_var = v;
                    best_idx = i;
                }
            }
            s_primary_subcarrier = best_idx;
            s_did_first_reeval = true;

            /* The ring buffer up to this point holds phase from whatever
             * subcarrier was tracked before lock-in (index 0's default).
             * Feeding that into a variance window against the newly locked
             * subcarrier produces a fake discontinuity spike on the very
             * next read -- verified on real hardware (2026-08-16): this
             * spike got baked into the calibration average and inflated
             * the learned threshold ~4x above the real steady-state range.
             * Clear it so the first post-lock reading is real. */
            s_history_idx = 0;
            s_history_len = 0;
        } else {
            /* Still warming up: nothing to measure yet. */
            printf("PRESENCE,%lld,%d,%d,%.5f,%.5f,%d,%d\n",
                   (long long)record->timestamp_us, record->rssi,
                   s_primary_subcarrier, 0.0f, 0.0f, 0, 0);
            return;
        }
    }

    if (s_primary_subcarrier >= num_subcarriers) {
        /* Packet shape shrank below our current pick; fall back to 0
         * until the next re-evaluation window. */
        s_primary_subcarrier = 0;
    }

    /* Push the primary subcarrier's phase into the ring buffer. */
    s_phase_history[s_history_idx] = phases[s_primary_subcarrier];
    s_history_idx = (s_history_idx + 1) % PHASE_HISTORY_LEN;
    if (s_history_len < PHASE_HISTORY_LEN) s_history_len++;

    /* Motion energy = variance of the last MOTION_WINDOW samples. */
    if (s_history_len >= 10) {
        uint16_t window = (s_history_len < MOTION_WINDOW) ? s_history_len : MOTION_WINDOW;
        double sum = 0.0, sum2 = 0.0;
        for (uint16_t i = 0; i < window; i++) {
            uint16_t ri = (s_history_idx + PHASE_HISTORY_LEN - window + i) % PHASE_HISTORY_LEN;
            double v = s_phase_history[ri];
            sum += v;
            sum2 += v * v;
        }
        double mean = sum / (double)window;
        double var = (sum2 / (double)window) - (mean * mean);
        s_motion_energy = (var > 0.0) ? (float)var : 0.0f;
    }

    /* Time-based adaptive calibration. Clock starts at lock-in, not boot --
     * same reasoning as the ring-buffer reset above: don't let anything
     * from before we had a real subcarrier pollute "ambient". */
    if (s_first_frame_us < 0) s_first_frame_us = record->timestamp_us;
    double elapsed_s = (record->timestamp_us - s_first_frame_us) / 1e6;

    if (!s_calibrated) {
        if (elapsed_s < CALIB_SECONDS) {
            s_calib_sum += s_motion_energy;
            s_calib_sum_sq += (double)s_motion_energy * s_motion_energy;
            s_calib_count++;
        } else if (s_calib_count >= CALIB_MIN_SAMPLES) {
            double mean = s_calib_sum / (double)s_calib_count;
            double var = (s_calib_sum_sq / (double)s_calib_count) - (mean * mean);
            double sigma = (var > 0.0) ? sqrt(var) : 0.001;
            s_adaptive_threshold = (float)(mean + PRESENCE_SIGMA_MULT * sigma);
            if (s_adaptive_threshold < 0.001f) s_adaptive_threshold = 0.001f;
            s_calibrated = true;

            /* Whatever presence_detected latched to during pre-calibration
             * (using the crude PRESENCE_DEFAULT_THRESH) isn't a real
             * reading against the real threshold -- verified on real
             * hardware (2026-08-16): it stayed latched "present" for
             * PRESENCE_CLEAR_FRAMES frames after calibration completed,
             * a needless startup artifact. Start clean against the real
             * threshold instead of forcing the debounce to work it off. */
            s_presence_detected = false;
            s_below_count = 0;

            ESP_LOGI(TAG, "Calibrated after %.1fs (%lu samples): threshold=%.5f",
                     elapsed_s, (unsigned long)s_calib_count, s_adaptive_threshold);
        } else {
            /* Calibration window elapsed but too few samples arrived
             * (sparse traffic) -- keep collecting rather than committing
             * to a threshold from noise. */
        }
    }

    float threshold = s_calibrated ? s_adaptive_threshold : PRESENCE_DEFAULT_THRESH;
    s_presence_detected = presence_flag_update(s_presence_detected, s_motion_energy,
                                               threshold, &s_below_count);

    printf("PRESENCE,%lld,%d,%d,%.5f,%.5f,%d,%d\n",
           (long long)record->timestamp_us,
           record->rssi,
           s_primary_subcarrier,
           s_motion_energy,
           threshold,
           s_calibrated ? 1 : 0,
           s_presence_detected ? 1 : 0);
}

static void csi_presence_task(void *arg)
{
    csi_record_t record;
    while (1) {
        if (xQueueReceive(csi_queue, &record, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        process_record(&record);
    }
}

/* ------------------------------------------------------------------ */
/* WiFi + app_main (same shape as csi_test)                            */
/* ------------------------------------------------------------------ */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &ip_event_handler, NULL, NULL)
    );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "WiFi connection initiated");
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_record_t));
    if (csi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    xTaskCreate(csi_presence_task, "csi_presence_task", 4096, NULL, 3, NULL);

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
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI Enabled. Calibrating ambient baseline for %.0fs -- keep the room empty.",
             CALIB_SECONDS);
}
