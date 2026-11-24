/*
 * analyse_csi -- a signal-study project. Deliberately does no analysis
 * on-device: it just captures raw CSI as cleanly and cheaply as possible
 * and streams it out as compact binary frames (see csi_wire_protocol.h).
 * All amplitude/phase/whatever-you-want-to-try math happens on the host,
 * in Python, where it's free to change without reflashing the board.
 *
 * Reuses two pieces of infrastructure already proven out in csi_test and
 * csi_presence: the CONFIG_ESP_WIFI_CSI_ENABLED sdkconfig fix, and the
 * queue + low-priority-task split that keeps slow work off the WiFi
 * driver's task (see csi_test/docs/v1.md §4.2 for why that matters).
 */

#include <stdio.h>
#include <string.h>

#include "hal/uart_types.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag_vfs.h"

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
#include "csi_wire_protocol.h"

#define CSI_QUEUE_DEPTH  16

static const char *TAG = "ANALYSE_CSI";

static QueueHandle_t csi_queue = NULL;
static uint32_t packet_count = 0;
static uint32_t dropped_count = 0;

typedef struct {
    int64_t timestamp_us;
    int8_t rssi;
    uint8_t channel;
    uint16_t len;
    bool first_word_invalid;
    int8_t buf[CSI_FRAME_MAX_PAYLOAD];
} csi_record_t;

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/*
 * Runs inside the WiFi driver's own (high-priority) task. Must be fast:
 * no framing, no checksum, no I/O -- just copy the raw bytes we need and
 * return. The actual frame building and serial write happens in
 * csi_stream_task, off the WiFi task's critical path.
 */
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
    record.len = (data->len < CSI_FRAME_MAX_PAYLOAD) ? data->len : CSI_FRAME_MAX_PAYLOAD;

    memcpy(record.buf, data->buf, record.len);

    if (xQueueSend(csi_queue, &record, 0) != pdTRUE) {
        dropped_count++;
    }
}

/*
 * Packs one binary frame per csi_wire_protocol.h and writes it to stdout
 * (the same stream ESP-IDF's own text logs go to -- the Python-side
 * decoder scans for the magic word and validates the checksum precisely
 * because log lines can land in between frames).
 */
static void write_frame(const csi_record_t *record)
{
    static uint8_t frame[CSI_FRAME_MAX_LEN];
    size_t pos = 0;

    frame[pos++] = CSI_FRAME_MAGIC_0;
    frame[pos++] = CSI_FRAME_MAGIC_1;
    frame[pos++] = CSI_FRAME_MAGIC_2;
    frame[pos++] = CSI_FRAME_MAGIC_3;

    size_t checksum_start = pos; /* right after magic */

    uint64_t device_us = (uint64_t)record->timestamp_us;
    for (int i = 0; i < 8; i++) {
        frame[pos++] = (uint8_t)(device_us >> (8 * i));
    }

    frame[pos++] = (uint8_t)record->rssi;
    frame[pos++] = (uint8_t)record->channel;
    frame[pos++] = record->first_word_invalid ? 1 : 0;

    uint16_t payload_len = record->len;
    frame[pos++] = (uint8_t)(payload_len & 0xFF);
    frame[pos++] = (uint8_t)(payload_len >> 8);

    memcpy(&frame[pos], record->buf, payload_len);
    pos += payload_len;

    uint16_t checksum = 0;
    for (size_t i = checksum_start; i < pos; i++) {
        checksum += frame[i];
    }
    frame[pos++] = (uint8_t)(checksum & 0xFF);
    frame[pos++] = (uint8_t)(checksum >> 8);

    fwrite(frame, 1, pos, stdout);
    fflush(stdout);
}

static void csi_stream_task(void *arg)
{
    csi_record_t record;
    while (1) {
        if (xQueueReceive(csi_queue, &record, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        write_frame(&record);
    }
}

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
    /*
     * ESP-IDF's console VFS translates outgoing '\n' (0x0A) to "\r\n" by
     * default, for terminal compatibility -- fine for text logs, fatal
     * for a binary protocol, since 0x0A appears constantly in raw CSI
     * payload bytes. Confirmed on real hardware (2026-08-16): without
     * this, every single frame failed checksum (resync_count climbed,
     * frames_ok stayed at 0) because 0x0A bytes in the payload were
     * silently becoming two bytes (0x0D 0x0A) on the wire, shifting
     * every byte after them. Both console backends are enabled in this
     * build's sdkconfig, so disable translation on both rather than
     * assume which one is actually active.
     */
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_record_t));
    if (csi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    xTaskCreate(csi_stream_task, "csi_stream_task", 4096, NULL, 3, NULL);

    /* Same config already proven on this hardware in csi_test/csi_presence:
     * all LTF types enabled, unsmoothed, auto-scaled. */
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

    ESP_LOGI(TAG, "CSI streaming enabled (binary protocol)");
}
