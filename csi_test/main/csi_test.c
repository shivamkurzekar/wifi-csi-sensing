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

/*
 * Set to 1 to print every packet's CSI as a CSV line.
 * Raise this if the serial console can't keep up with the
 * ping rate you're using on the host side.
 */
#define CSI_PRINT_EVERY_N  1

/*
 * Largest CSI payload we'll ever copy out of a callback. The configured
 * LTF types (legacy + HT-LTF + STBC-HT-LTF2) top out well under this.
 */
#define CSI_RECORD_MAX_LEN  512

#define CSI_QUEUE_DEPTH     16

static const char *TAG = "CSI_TEST";

static uint32_t packet_count = 0;
static uint32_t dropped_count = 0;

static QueueHandle_t csi_queue = NULL;

typedef struct {
    int64_t timestamp_us;
    int8_t rssi;
    uint8_t channel;
    uint16_t len;
    bool first_word_invalid;
    int8_t buf[CSI_RECORD_MAX_LEN];
} csi_record_t;

/*
 * Fires once the ESP32 gets an IP address from the router.
 * We need this printed so we know what address to `ping`
 * from another device to generate a steady stream of packets.
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR " <-- ping this address from your host",
                 IP2STR(&event->ip_info.ip));
    }
}

/*
 * Runs inside the WiFi driver's own (high-priority) task. Must be fast:
 * no printf, no floating point formatting, no blocking. We just copy the
 * raw bytes we need into a queue and return immediately. The actual
 * amplitude/phase math and printing happens later in csi_print_task,
 * off the WiFi task, so we never delay/starve the rest of the system.
 */
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *data)
{
    if (data == NULL || data->buf == NULL) {
        return;
    }

    packet_count++;

    if ((packet_count % CSI_PRINT_EVERY_N) != 0) {
        return;
    }

    csi_record_t record;
    record.timestamp_us = esp_timer_get_time();
    record.rssi = data->rx_ctrl.rssi;
    record.channel = data->rx_ctrl.channel;
    record.first_word_invalid = data->first_word_invalid;
    record.len = (data->len < CSI_RECORD_MAX_LEN) ? data->len : CSI_RECORD_MAX_LEN;

    memcpy(record.buf, data->buf, record.len);

    /*
     * Non-blocking send: if csi_print_task can't keep up and the queue
     * is full, drop this sample rather than ever block the WiFi task.
     */
    if (xQueueSend(csi_queue, &record, 0) != pdTRUE) {
        dropped_count++;
    }
}

/*
 * Low-priority task that does all the "slow" work: converting raw
 * (imaginary, real) byte pairs into amplitude/phase and printing a
 * CSV line over serial. Runs independently of the WiFi task.
 */
static void csi_print_task(void *arg)
{
    csi_record_t record;

    while (1) {
        if (xQueueReceive(csi_queue, &record, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /*
         * CSI data is a sequence of signed 8-bit (imaginary, real) pairs,
         * one pair per subcarrier. If first_word_invalid is set, the
         * first 4 bytes (first 2 subcarriers) are garbage and are skipped.
         */
        int start = record.first_word_invalid ? 4 : 0;
        int num_subcarriers = (record.len - start) / 2;

        if (num_subcarriers < 0) {
            num_subcarriers = 0;
        }

        /*
         * "CSI," prefix lets a host-side script pick this line out of
         * the normal ESP-IDF log noise (wifi driver logs, boot logs, etc.)
         * that shares the same serial output.
         *
         * Format:
         * CSI,<timestamp_us>,<rssi>,<channel>,<num_subcarriers>,amp0,phase0,amp1,phase1,...
         */
        printf("CSI,%lld,%d,%d,%d",
               (long long)record.timestamp_us,
               record.rssi,
               record.channel,
               num_subcarriers);

        for (int i = 0; i < num_subcarriers; i++) {
            int idx = start + (i * 2);
            float im = (float)record.buf[idx];
            float re = (float)record.buf[idx + 1];

            float amplitude = sqrtf((im * im) + (re * re));
            float phase = atan2f(im, re);

            printf(",%.3f,%.3f", amplitude, phase);
        }

        printf("\n");
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &ip_event_handler,
            NULL,
            NULL
        )
    );

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );

    ESP_ERROR_CHECK(
        esp_wifi_connect()
    );

    ESP_LOGI(TAG,
             "WiFi connection initiated");
}

void app_main(void)
{
    ESP_ERROR_CHECK(
        nvs_flash_init()
    );

    wifi_init();

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_record_t));
    if (csi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    /*
     * Priority 3: above idle (0) so it always gets to run and feed the
     * watchdog, but well below the WiFi driver task so it never delays it.
     */
    xTaskCreate(csi_print_task, "csi_print_task", 4096, NULL, 3, NULL);

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };

    ESP_ERROR_CHECK(
        esp_wifi_set_csi_config(
            &csi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_csi_rx_cb(
            wifi_csi_cb,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_csi(true)
    );

    ESP_LOGI(TAG,
             "CSI Enabled");
}
