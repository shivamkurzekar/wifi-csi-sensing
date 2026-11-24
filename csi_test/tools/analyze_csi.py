#!/usr/bin/env python3
"""
Parses a CSV produced by capture_csi.py and emits a compact JSON summary
for visualization: a time axis, RSSI, the full amplitude matrix
(packet x subcarrier), and a per-packet "variability" score (the std
of amplitude across active subcarriers) used to spot motion.

Usage:
    python3 analyze_csi.py --in baseline_then_motion.csv --out csi_summary.json
"""

import argparse
import csv
import json
import statistics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="infile", required=True)
    parser.add_argument("--out", dest="outfile", required=True)
    args = parser.parse_args()

    rows = []
    skipped = 0
    with open(args.infile, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                values = [float(v) for v in row["values"].split(",")]
                device_us = int(row["device_us"])
                rssi = int(row["rssi"])
            except (ValueError, KeyError):
                # Malformed row (e.g. a firmware log line landed mid-write
                # on the serial port). Skip it rather than fail the batch.
                skipped += 1
                continue
            amplitudes = values[0::2]
            rows.append({
                "host_time": row["host_time"],
                "device_us": device_us,
                "rssi": rssi,
                "amplitudes": amplitudes,
            })

    if skipped:
        print(f"Skipped {skipped} malformed row(s).")

    if not rows:
        raise SystemExit("No CSI rows found in input file.")

    # Keep only rows with the most common subcarrier count, in case a
    # corrupted line was truncated to a different length.
    lengths = [len(r["amplitudes"]) for r in rows]
    expected_len = statistics.mode(lengths)
    bad_length = sum(1 for l in lengths if l != expected_len)
    if bad_length:
        print(f"Dropping {bad_length} row(s) with unexpected subcarrier count.")
        rows = [r for r in rows if len(r["amplitudes"]) == expected_len]

    t0 = rows[0]["device_us"]
    num_subcarriers = len(rows[0]["amplitudes"])

    # Which subcarrier columns are "active" (not just null/guard padding
    # across the whole capture) -- used only to compute a cleaner variability score.
    active_cols = [
        i for i in range(num_subcarriers)
        if any(r["amplitudes"][i] != 0.0 for r in rows)
    ]

    time_s = []
    rssi = []
    variability = []
    matrix = []

    for r in rows:
        time_s.append(round((r["device_us"] - t0) / 1_000_000, 3))
        rssi.append(r["rssi"])
        active_vals = [r["amplitudes"][i] for i in active_cols]
        variability.append(round(statistics.pstdev(active_vals), 3) if len(active_vals) > 1 else 0.0)
        matrix.append([round(v, 2) for v in r["amplitudes"]])

    summary = {
        "num_packets": len(rows),
        "num_subcarriers": num_subcarriers,
        "duration_s": time_s[-1],
        "time_s": time_s,
        "rssi": rssi,
        "variability": variability,
        "matrix": matrix,
    }

    with open(args.outfile, "w") as f:
        json.dump(summary, f)

    print(f"{len(rows)} packets, {num_subcarriers} subcarriers, "
          f"{time_s[-1]}s duration -> {args.outfile}")


if __name__ == "__main__":
    main()
