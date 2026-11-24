# wifi-csi-sensing

Experiments in sensing a room with WiFi, using **Channel State Information
(CSI)** on an ESP32-S3. CSI is the per-subcarrier amplitude and phase the WiFi
radio measures on every packet it receives. Walls, furniture and moving people
all change it, so it can act as a motion and presence sensor without a camera.

Each folder is a standalone ESP-IDF project (ESP-IDF v5.4, target ESP32-S3).

| Project | Question it answers | How |
|---|---|---|
| [`csi_test/`](csi_test) | What does CSI look like? | Firmware streams CSI as CSV over serial. Python tools record it, score how much it varies, and build an interactive HTML report. |
| [`csi_presence/`](csi_presence) | Is someone in the room right now? | Runs entirely on the ESP32. Phase variance of one subcarrier over about 1 s gives a motion score, with a self-calibrating baseline and hysteresis. Prints a live `PRESENCE` line. |
| [`analyse_csi/`](analyse_csi) | What's worth measuring? | Firmware forwards raw CSI bytes in a checksummed binary format and does no math. A Python server decodes them and shows live charts in the browser (RSSI, variability, phase waterfall) and logs CSV per session. |

They were built in that order. Each one reuses ideas proven in the one before
it, but they share no code.

## Docs

- [`docs/csi-learning/`](docs/csi-learning/00-index.md): a five-part
  explanation of why CSI sensing works, from radio waves through OFDM to
  the bytes the ESP32 hands your callback.
- [`docs/esp32s3_cheat_sheet.txt`](docs/esp32s3_cheat_sheet.txt): an ESP32-S3
  quick reference.
- Each project's own `docs/v1.md` covers its design, build steps and
  hardware test results. `csi_test` and `csi_presence` also have
  `docs/02-algorithms-and-concepts.md`, which walks through the algorithms.

## Building a project

```sh
cd csi_test        # or csi_presence / analyse_csi
cp main/wifi_credentials.h.example main/wifi_credentials.h   # add your SSID and password
idf.py set-target esp32s3
idf.py -p /dev/ttyACM0 build flash monitor
```

`wifi_credentials.h` is gitignored. See each project's `docs/v1.md` for the
host-side Python tools.

## History

The three projects started as separate repos and were merged here with
their full commit history (`git subtree`).
