#!/usr/bin/env python3
"""
Reads the ESP32's serial output directly (bypassing idf.py monitor) and
saves every CSI line to a CSV file, tagged with the host's wall-clock time
so you can correlate "I moved at 14:32:10" with the recorded data later.

Usage:
    python3 capture_csi.py --port /dev/ttyACM0 --out session1.csv

Press Ctrl+C to stop recording.
"""

import argparse
import csv
import datetime
import sys

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="csi_capture.csv")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)

    rows_written = 0

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        # host_time: wall-clock time this line was received (for correlating with your actions)
        # device_us: the ESP32's own timestamp, microseconds since boot
        # rssi, channel, num_subcarriers: from rx_ctrl
        # values: alternating amplitude,phase per subcarrier
        writer.writerow(["host_time", "device_us", "rssi", "channel", "num_subcarriers", "values"])

        print("Recording... press Ctrl+C to stop.")
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except UnicodeDecodeError:
                    continue

                if not line.startswith("CSI,"):
                    # Not a CSI line (boot logs, wifi driver logs, etc). Ignore.
                    continue

                parts = line.split(",")
                # parts[0] == "CSI"
                if len(parts) < 5:
                    continue

                device_us, rssi, channel, num_subcarriers = parts[1:5]
                values = ",".join(parts[5:])
                host_time = datetime.datetime.now().isoformat(timespec="milliseconds")

                writer.writerow([host_time, device_us, rssi, channel, num_subcarriers, values])
                rows_written += 1

                if rows_written % 20 == 0:
                    f.flush()
                    print(f"\r{rows_written} packets recorded (rssi={rssi}, subcarriers={num_subcarriers})", end="")
                    sys.stdout.flush()

        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            print(f"\nStopped. {rows_written} packets saved to {args.out}")
            ser.close()


if __name__ == "__main__":
    main()
