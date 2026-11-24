# 50-packet raw capture — summary

**Captured:** 2026-08-18, live hardware, `main.py`-style ad-hoc capture against the
real ESP32-S3 (WiFi channel 4, RSSI -39 to -40 dBm throughout).
**Raw data:** [`raw50_subcarriers.csv`](raw50_subcarriers.csv) — 9,600 rows
(50 packets × 192 subcarriers each), columns:
`packet_index, device_us, rssi, channel, first_word_invalid, num_subcarriers, subcarrier_index, val1_im, val2_re, amplitude, phase_rad`.

This was captured to answer two concrete questions and to have a real,
checkable example of the amplitude/phase math -- not synthetic data.

## Q1: Is the subcarrier count always the same?

**No**, in general -- confirmed earlier (see `csi_test/docs/v1.md`) with a mix
of 64- and 192-subcarrier packets in the same capture, depending on whether a
received packet was HT (carries all three configured training fields) or
non-HT/legacy (carries only the legacy one).

**In this specific 50-packet sample, every packet happened to be 192
subcarriers** -- a property of what traffic was in flight during this narrow
window, not a guarantee. Don't read "192 every time" as a rule; it's what
this particular sample looked like.

## Q2: If subcarrier s at index k in packet 1, is it index k in packet n?

- **Within packets of the same shape**: yes. The buffer layout is a fixed
  concatenation (`[legacy LTF][HT-LTF][STBC-HT-LTF2]`, ~64 wide each) set by
  the driver, so index `k` means the same physical subcarrier in every
  192-subcarrier packet.
- **Across different shapes** (a 64-subcarrier packet vs. a 192-subcarrier
  one): not confirmed. This is exactly why `csi_presence` resets phase-unwrap
  continuity whenever the packet shape changes between frames -- treated as
  an open risk, not a verified fact.

Table 3 below demonstrates the "same shape -> stable position" half directly:
subcarrier index 10 is tracked across all 50 packets, and its *amplitude*
stays in a tight physically-plausible band throughout, which would not be
true if the reader were jumping between different physical subcarriers frame
to frame.

## The worked example (packet 0, subcarrier 1)

Raw bytes straight off the wire: `im = -1, re = -24`.

```
amplitude = sqrt((-1)^2 + (-24)^2) = sqrt(577) = 24.02
phase     = atan2(-1, -24)          = -3.100 rad  (~ -177.6 deg)
```

Both components negative places the `(re, im)` point in the third quadrant
of the complex plane, just short of the negative real axis -- hence a phase
close to -pi. This is exactly what "phase" means here: the angle of that
raw `(im, re)` pair.

## Table 1 -- per-packet summary (all 50)

Every packet in this sample: 192 subcarriers, channel 4, RSSI -39 or -40 dBm,
`first_word_invalid = 0`. Full per-packet `device_us` timestamps are in the
CSV (`packet_index` 0-49).

## Table 2 -- all subcarriers for packet #0 (first 30 of 192)

| sc# | val1 (im) | val2 (re) | amplitude | phase (rad) |
|---|---|---|---|---|
| 0 | 0 | 0 | 0.000 | 0.000 |
| 1 | 1 | 23 | 23.022 | 0.043 |
| 2 | 1 | 20 | 20.025 | 0.050 |
| 3 | 1 | 19 | 19.026 | 0.053 |
| 4 | 1 | 17 | 17.029 | 0.059 |
| 5 | 0 | 15 | 15.000 | 0.000 |
| 6 | -1 | 13 | 13.038 | -0.077 |
| 7 | -3 | 12 | 12.369 | -0.245 |
| 8 | -4 | 12 | 12.649 | -0.322 |
| 9 | -5 | 12 | 13.000 | -0.395 |
| 10 | -6 | 13 | 14.318 | -0.432 |
| ... | | | | (22 more real values, then a run of zeros -- the null/guard band between LTF groups, same pattern seen in every capture in this project family) |

Full 192 rows for this packet (and all 49 others) are in the CSV.

## Table 3 -- subcarrier index 10, tracked across all 50 packets

| packet# | val1 (im) | val2 (re) | amplitude | phase (rad) |
|---|---|---|---|---|
| 0 | -6 | 13 | 14.318 | -0.432 |
| 1 | -14 | -3 | 14.318 | -1.782 |
| 2 | 12 | -5 | 13.000 | 1.966 |
| 5 | -14 | -3 | 14.318 | -1.782 |
| 10 | -12 | -9 | 15.000 | -2.214 |
| 26 | 0 | -14 | 14.000 | 3.142 |
| 43 | 0 | 14 | 14.000 | 0.000 |
| 49 | 13 | -2 | 13.153 | 1.723 |

(a representative subset; all 50 rows are in the CSV)

**Amplitude** across all 50 packets stays in a tight band, roughly
12.1-15.6 -- physically plausible for a static scene.

**Phase**, over the same 50 packets, is scattered essentially uniformly
across the *entire* -pi to +pi range with no visible slow drift or
clustering.

## The finding: raw absolute phase between independent packets looks
## dominated by per-packet timing/frequency offset, not physical motion

If phase were cleanly tracking the physical channel (as amplitude appears
to, and as the `csi_presence` design assumes), a static scene should produce
phase values that stay close together or drift slowly and smoothly between
consecutive packets -- not jump essentially at random through the full
range every single packet, which is what Table 3 shows.

The likely cause is a real, well-documented phenomenon with commodity WiFi
CSI (not something specific to this codebase): each received packet has its
own small, essentially independent **timing offset (STO)** and **carrier
frequency offset (CFO)** between the transmitter's and receiver's clocks.
That offset shows up directly in the *phase* term but does not affect
*amplitude* the same way -- which lines up exactly with what's measured
here: stable amplitude, scattered phase.

**Why this matters for `csi_presence` specifically:** it tracks one
subcarrier's raw phase frame-to-frame and calibrates a presence threshold
from its variance. If raw phase is dominated by this per-packet offset
rather than real motion, that variance is measuring mostly timing jitter,
not the room. This is a plausible, data-backed explanation for why
`csi_presence`'s calibrated threshold came out well above any real motion
peak measured during its own walk-test (see `csi_presence/docs/v1.md`) --
but it has **not been proven** by a rigorous statistical test, only observed
as a strong pattern in one 50-packet sample. Treat it as the leading
hypothesis for a follow-up investigation, not a confirmed root cause.

**Practical consequence for this project:** a naive "phase waterfall" or
"phase over time" chart, plotting raw phase directly the way the amplitude
waterfall plots raw amplitude, would very likely just show visual noise
rather than anything interpretable. Real CSI-sensing systems typically
apply a phase-calibration step first (e.g. a linear-fit correction across
subcarriers within one packet, to remove the offset before phase becomes
usable across packets). That calibration step does not exist anywhere in
this project family yet -- it's the natural next piece of work before phase
gets its own chart here.
