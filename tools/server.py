#!/usr/bin/env python3
"""
analyse_csi's live viewer: reads the binary CSI stream off serial,
decodes it with proto.py, and serves a browser page that polls for
updates roughly once a second -- both a scrolling "live" waterfall and a
full-session "till now" summary, from the same running process.

Usage:
    python3 server.py --port /dev/ttyACM0 --http-port 8000

Then open http://<this-machine's-LAN-IP>:8000 from any browser on the
same network (including your phone, if it's on the same WiFi).

Architecture: one background thread continuously reads serial and decodes
frames into shared, lock-protected state; the HTTP server (a separate
thread per request) only ever reads that state to answer /data.json polls.
Neither side blocks the other for more than a few microseconds.
"""

import argparse
import csv
import http.server
import json
import socket
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial

import proto

RECENT_WINDOW = 300          # frames kept for the live waterfall
HISTORY_CAP = 20000          # safety cap on the full-session summary
POLL_HINT_MS = 1000          # how often the page should poll (advisory, sent in payload)


class SharedState:
    def __init__(self, log_path: Path):
        self.lock = threading.Lock()
        self.recent = deque(maxlen=RECENT_WINDOW)   # dicts: t, rssi, channel, amplitudes
        self.history_t = []
        self.history_rssi = []
        self.history_variability = []
        self.frames_ok = 0
        self.resync_count = 0
        self.start_time = time.time()
        self.session_start_iso = datetime.now().isoformat()
        self.first_device_us = None

        # Every frame is written to disk immediately (flushed per row) --
        # the file is safe to read/tail mid-session, same pattern as
        # csi_presence's live_log.py.
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = open(log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "host_time", "t_s", "device_us", "rssi", "channel",
            "first_word_invalid", "num_subcarriers", "amplitudes", "phases",
        ])
        self.log_file.flush()
        print(f"Logging every frame to {log_path}")

    def add_frame(self, frame: proto.Frame):
        amps, phases = frame.amplitude_phase()
        active = [a for a in amps if a != 0.0]
        variability = statistics.pstdev(active) if len(active) > 1 else 0.0

        with self.lock:
            if self.first_device_us is None:
                self.first_device_us = frame.device_us
            t = (frame.device_us - self.first_device_us) / 1_000_000.0

            self.recent.append({
                "t": t,
                "rssi": frame.rssi,
                "channel": frame.channel,
                "amplitudes": amps,
                "phases": phases,
                "variability": round(variability, 3),
            })

            self.history_t.append(t)
            self.history_rssi.append(frame.rssi)
            self.history_variability.append(round(variability, 3))
            if len(self.history_t) > HISTORY_CAP:
                # Drop the oldest 10% rather than one at a time -- cheaper.
                cut = HISTORY_CAP // 10
                del self.history_t[:cut]
                del self.history_rssi[:cut]
                del self.history_variability[:cut]

            self.log_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                round(t, 3),
                frame.device_us,
                frame.rssi,
                frame.channel,
                int(frame.first_word_invalid),
                len(amps),
                ";".join(f"{a:.3f}" for a in amps),
                ";".join(f"{p:.3f}" for p in phases),
            ])
            self.log_file.flush()

    def snapshot(self, resync_count, frames_ok):
        with self.lock:
            return {
                "poll_hint_ms": POLL_HINT_MS,
                "uptime_s": round(time.time() - self.start_time, 1),
                "session_start_iso": self.session_start_iso,
                "frames_ok": frames_ok,
                "resync_count": resync_count,
                "recent": list(self.recent),
                "history": {
                    "t": self.history_t,
                    "rssi": self.history_rssi,
                    "variability": self.history_variability,
                },
            }


def serial_reader_thread(port, baud, state: SharedState, stop_event):
    reader = proto.FrameReader()
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Reading CSI frames from {port} @ {baud}...")
    while not stop_event.is_set():
        chunk = ser.read(4096)
        if not chunk:
            continue
        for frame in reader.feed(chunk):
            state.add_frame(frame)
        state.frames_ok = reader.frames_ok
        state.resync_count = reader.resync_count
    ser.close()


def make_handler(state: SharedState, static_dir: Path):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # quiet -- /data.json is polled every second, don't spam stdout

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                self._serve_file(static_dir / "live_view.html", "text/html")
            elif self.path.startswith("/data.json"):
                self._serve_json()
            else:
                self.send_response(404)
                self.end_headers()

        def _serve_file(self, path: Path, content_type):
            try:
                body = path.read_bytes()
            except FileNotFoundError:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_json(self):
            payload = state.snapshot(state.resync_count, state.frames_ok)
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def local_lan_ip():
    """Best-effort guess at this machine's LAN IP, for the startup message."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--data-dir", default=str(Path(__file__).parent / "data"),
                         help="Where to write the per-session CSV log (default: tools/data/)")
    args = parser.parse_args()

    session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = Path(args.data_dir) / f"session_{session_id}.csv"

    state = SharedState(log_path)
    stop_event = threading.Event()

    reader_thread = threading.Thread(
        target=serial_reader_thread,
        args=(args.port, args.baud, state, stop_event),
        daemon=True,
    )
    reader_thread.start()

    static_dir = Path(__file__).parent
    handler = make_handler(state, static_dir)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", args.http_port), handler)

    ip = local_lan_ip()
    print(f"Serving on http://{ip}:{args.http_port}  (also http://localhost:{args.http_port} locally)")
    print("Ctrl+C to stop.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        httpd.shutdown()
        state.log_file.close()
        print(f"\nStopped. Session log: {log_path}")


if __name__ == "__main__":
    main()
