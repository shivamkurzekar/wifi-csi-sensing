#!/usr/bin/env python3
"""
Reference decoder for analyse_csi's binary wire protocol.

See main/csi_wire_protocol.h in the firmware for the authoritative frame
layout -- this must match it exactly. Because frames share the serial link
with ESP-IDF's own text logs, FrameReader scans for the magic word and
validates the checksum; on mismatch it slides forward one byte and keeps
scanning rather than treating it as fatal. That's normal operation, not
corruption -- a log line landing between two frames looks exactly like
this from the reader's point of view.
"""

import math
import struct

MAGIC = bytes([0xC5, 0x10, 0xAC, 0x02])
HEADER_LEN = 17  # magic(4) + device_us(8) + rssi(1) + channel(1) + first_word_invalid(1) + payload_len(2)
CHECKSUM_LEN = 2
MAX_PAYLOAD = 512


class Frame:
    __slots__ = ("device_us", "rssi", "channel", "first_word_invalid", "payload")

    def __init__(self, device_us, rssi, channel, first_word_invalid, payload):
        self.device_us = device_us
        self.rssi = rssi
        self.channel = channel
        self.first_word_invalid = first_word_invalid
        self.payload = payload  # list of signed ints, raw (im, re) bytes

    def amplitude_phase(self):
        """(amplitudes, phases) as parallel lists, one entry per subcarrier,
        applying the first_word_invalid skip the same way the firmware
        projects (csi_test, csi_presence) do."""
        start = 4 if self.first_word_invalid else 0
        buf = self.payload
        n = (len(buf) - start) // 2
        amps = [0.0] * n
        phases = [0.0] * n
        for i in range(n):
            idx = start + i * 2
            im = buf[idx]
            re = buf[idx + 1]
            amps[i] = math.hypot(im, re)
            phases[i] = math.atan2(im, re)
        return amps, phases


class FrameReader:
    """Feed raw bytes in via feed(bytes) -> list[Frame]."""

    def __init__(self):
        self.buf = bytearray()
        self.resync_count = 0
        self.frames_ok = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while True:
            frame, keep_going = self._try_extract_one()
            if frame is not None:
                frames.append(frame)
            if not keep_going:
                # Genuinely need more bytes from the wire before we can
                # make further progress on this buffer -- stop and wait
                # for the next feed() call.
                break
        return frames

    def _try_extract_one(self):
        """Returns (frame_or_None, keep_going). keep_going=True means the
        buffer was advanced (a frame was extracted, or a bad magic/checksum
        match was skipped past) and it's worth trying again immediately --
        there could be another valid frame right behind it in the same
        feed() call. keep_going=False means we're genuinely blocked
        waiting for more bytes to arrive."""
        buf = self.buf

        idx = buf.find(MAGIC)
        if idx == -1:
            # No magic in the buffer at all. Keep the last 3 bytes in case
            # a magic word is split across two feed() calls.
            if len(buf) > 3:
                del buf[:len(buf) - 3]
            return None, False
        if idx > 0:
            del buf[:idx]  # drop log text / garbage before the magic
            return None, True  # buffer advanced; a real frame may follow immediately

        if len(buf) < HEADER_LEN:
            return None, False  # header not fully arrived yet

        payload_len = struct.unpack_from("<H", buf, 15)[0]

        if payload_len > MAX_PAYLOAD:
            # This magic match was coincidental (e.g. inside log text).
            # Skip past it and keep scanning.
            del buf[:4]
            self.resync_count += 1
            return None, True

        total_len = HEADER_LEN + payload_len + CHECKSUM_LEN
        if len(buf) < total_len:
            return None, False  # payload/checksum not fully arrived yet

        checksum_expected = struct.unpack_from("<H", buf, HEADER_LEN + payload_len)[0]
        checksum_actual = sum(buf[4:HEADER_LEN + payload_len]) & 0xFFFF

        if checksum_actual != checksum_expected:
            # Coincidental magic match. Skip past it and keep scanning --
            # do NOT skip the whole total_len, since a real frame could
            # start anywhere in the bytes we just misidentified.
            del buf[:4]
            self.resync_count += 1
            return None, True

        device_us = struct.unpack_from("<Q", buf, 4)[0]
        rssi = struct.unpack_from("<b", buf, 12)[0]
        channel = buf[13]
        first_word_invalid = bool(buf[14])
        raw_payload = bytes(buf[HEADER_LEN:HEADER_LEN + payload_len])

        del buf[:total_len]
        self.frames_ok += 1

        signed_payload = [(b - 256) if b > 127 else b for b in raw_payload]
        frame = Frame(device_us, rssi, channel, first_word_invalid, signed_payload)
        return frame, True
