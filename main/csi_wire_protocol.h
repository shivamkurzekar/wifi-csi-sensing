/*
 * Binary wire protocol for streaming raw CSI over the same serial link
 * used for the console/log output.
 *
 * Why binary instead of the CSV-over-printf approach csi_test uses:
 * formatting ~192 subcarriers as ASCII floats is the expensive part (both
 * in bytes-on-wire and CPU time), not something to just tolerate in a
 * background task. This format sends raw bytes straight from the CSI
 * buffer with no float math and no text formatting on-device at all --
 * amplitude/phase/whatever analysis happens on the host, in Python, where
 * it's free to change without reflashing.
 *
 * Because it shares the same serial link as ESP-IDF's own text logging
 * (WiFi driver logs, boot logs), a frame can end up with ordinary log text
 * interleaved before or after it. The reader is expected to scan for the
 * 4-byte magic word, verify the checksum, and if it fails, slide forward
 * one byte and keep scanning -- this is standard byte-stream frame
 * resynchronization, not a corruption bug. python/analyse_csi/proto.py is
 * the reference decoder and implements exactly this.
 *
 * Frame layout (all multi-byte integers little-endian):
 *
 *   offset  size  field
 *   ------  ----  -----------------------------------------------------
 *   0       4     magic: CSI_FRAME_MAGIC bytes
 *   4       8     device_us   (uint64) esp_timer_get_time() at capture
 *   12      1     rssi        (int8)
 *   13      1     channel     (uint8)
 *   14      1     first_word_invalid (uint8, 0 or 1)
 *   15      2     payload_len (uint16) raw CSI byte length that follows
 *   17      N     payload     raw bytes, exactly wifi_csi_info_t.buf
 *   17+N    2     checksum    (uint16) additive sum, mod 65536, of every
 *                             byte from offset 4 through offset 17+N-1
 *                             (i.e. everything except the magic itself)
 *
 *   total frame size = 19 + payload_len bytes
 */

#pragma once

#include <stdint.h>

#define CSI_FRAME_MAGIC_0  0xC5
#define CSI_FRAME_MAGIC_1  0x10
#define CSI_FRAME_MAGIC_2  0xAC
#define CSI_FRAME_MAGIC_3  0x02

#define CSI_FRAME_MAX_PAYLOAD  512   /* bytes; up to 256 subcarriers */
#define CSI_FRAME_HEADER_LEN   17    /* magic(4) + device_us(8) + rssi(1) + channel(1) + first_word_invalid(1) + payload_len(2) */
#define CSI_FRAME_CHECKSUM_LEN 2
#define CSI_FRAME_MAX_LEN      (CSI_FRAME_HEADER_LEN + CSI_FRAME_MAX_PAYLOAD + CSI_FRAME_CHECKSUM_LEN)
