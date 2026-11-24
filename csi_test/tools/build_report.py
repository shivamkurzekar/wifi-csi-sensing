#!/usr/bin/env python3
"""
Takes the JSON summary produced by analyze_csi.py and embeds it into
report_template.html, producing a single self-contained HTML file you can
open directly in a browser (heatmap + RSSI/variability charts + raw table).

Usage:
    python3 build_report.py --summary output/summary.json --out output/report.html
"""

import argparse
import pathlib


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True, help="JSON file produced by analyze_csi.py")
    parser.add_argument("--out", required=True, help="Path to write the resulting HTML report to")
    parser.add_argument(
        "--template",
        default=str(pathlib.Path(__file__).parent / "report_template.html"),
        help="HTML template with a __DATA_JSON__ placeholder (defaults to report_template.html next to this script)",
    )
    args = parser.parse_args()

    template = pathlib.Path(args.template).read_text()
    data_json = pathlib.Path(args.summary).read_text()

    if "__DATA_JSON__" not in template:
        raise SystemExit(f"{args.template} has no __DATA_JSON__ placeholder to fill in.")

    out_html = template.replace("__DATA_JSON__", data_json)

    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(out_html)

    print(f"Wrote {out_path} ({len(out_html)} bytes). Open it in any browser.")


if __name__ == "__main__":
    main()
