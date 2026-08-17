# 4. Channel State Information: From Physics to Data Structure

This document is the formal bridge from the physical story in
[`03-wifi-physics-deep-dive.md`](./03-wifi-physics-deep-dive.md) to the
actual bytes this codebase's CSI callback receives. If you've read
document 3, most of §4.1–4.2 here will be a recap; if you're starting
from this document directly, the recap is enough to follow the rest.

## 4.1 Recap: what CSI is

**Channel State Information (CSI)** is a per-subcarrier, per-antenna-pair
measurement of how the physical channel between a transmit antenna and a
receive antenna distorted a known reference signal. For one subcarrier,
it's a single complex number:

```
H = A · e^(jφ)
```

`A` is the amplitude scaling that subcarrier experienced end-to-end
(free-space path loss, reflection loss, constructive/destructive
multipath interference — document 3, §3.3, §3.6); `φ` is the net phase
rotation that subcarrier experienced, dominated by total path length in
fractions of a wavelength (document 3, §3.7). A WiFi frame's preamble
carries training-field subcarrier values whose true, undistorted value
the receiver already knows in advance (document 2, §2.6); comparing what
was actually received against that known value, subcarrier by
subcarrier, is what produces `H`. Document 3 works through the full
receive chain (antenna → RF front end → down-conversion → ADC → FFT →
channel estimation) that produces this value physically; this document
treats `H` as the given, and focuses on it as data.

## 4.2 Why CSI is computed at all — and why it's exposed as a side-channel

No WiFi transmitter deliberately sends "CSI" as a piece of user data.
Channel estimation is computed by *every* OFDM receiver, on *every*
packet, for an entirely different, mandatory reason: **equalization**.

Recall from document 3, §3.5: if `H` is the per-subcarrier channel
estimate and `X'` is an unknown payload symbol transmitted on that same
subcarrier, then the receiver measures `Y' ≈ H · X'` — the channel has
scaled and rotated the transmitted symbol by exactly the same `H` it
measured on the training field (assuming the channel doesn't change much
between the preamble and the payload of one packet, which is normally a
safe assumption over the few-microsecond to low-millisecond span of a
single frame). To recover an estimate of the original transmitted symbol
`X'`, the receiver simply divides:

```
X'_estimate = Y' / H
```

This division — **equalization** — is what undoes the channel's
distortion so the payload's constellation points (document 1, §1.4) land
back close to where they're supposed to be, which is a *precondition*
for correctly demodulating the bits at all. Every WiFi receiver chip
computes `H` internally, on every packet, purely to make this division
possible. It is not an optional feature added for sensing purposes — it
is core to how OFDM demodulation works at all.

What makes CSI-based sensing possible is that some WiFi chip vendors
(including the Espressif ESP32 family used in this codebase) chose to
**expose this already-computed internal value** to application code,
originally as a debug/diagnostic feature — a way to inspect link quality,
verify channel conditions, or debug demodulation issues — rather than a
feature designed for motion sensing. `esp_wifi_set_csi_config()` and
`esp_wifi_set_csi_rx_cb()` in this codebase are exactly this: turning on
and reading out a value the hardware was already computing for its own
internal purposes. This is the sense in which CSI sensing is
**piggybacked on ordinary WiFi traffic** — nothing about the transmitted
signal changes to support sensing; any ordinary packet exchange (in this
project's case, a background `ping` keeping packets flowing) supplies a
steady stream of CSI updates as a side effect of totally unrelated
communication.

## 4.3 From H to `(imaginary, real)` bytes in this codebase

Document 1, §1.3, established that `H = A·e^(jφ)` is exactly the same
information as a Cartesian I/Q pair: `H = I + jQ`, where
`I = A cos φ` (the real part) and `Q = A sin φ` (the imaginary part).
This is precisely the format the ESP32-S3 exposes:

- `wifi_csi_info_t.buf` is a sequence of **signed 8-bit
  `(imaginary, real)` byte pairs, one pair per subcarrier** — i.e., raw
  `(Q, I)` samples, quantized to 8-bit signed integers by the hardware
  (`manu_scale = false` in this codebase's config means the hardware
  picks the scaling/gain applied before quantization automatically,
  rather than firmware specifying a fixed manual scale).
- If `first_word_invalid` is set, the first 4 bytes (the first 2
  subcarriers' worth of pairs) are hardware garbage from the chip's
  extraction pipeline and must be skipped — a hardware quirk, not a
  physical phenomenon, but a real one this codebase's parsing has to
  account for.
- From each `(im, re)` pair, the firmware computes:

  ```c
  float amplitude = sqrtf((im * im) + (re * re));   // = |H| = A
  float phase      = atan2f(im, re);                // = arg(H) = φ
  ```

  `sqrt(im² + re²)` is exactly the Euclidean distance from the origin in
  the I/Q plane (document 1, §1.3's diagram) — the magnitude `A` of the
  complex number. `atan2(im, re)` computes the angle of that point from
  the positive real axis — the phase `φ` — using the two-argument
  arctangent specifically because a plain one-argument `atan(im/re)`
  cannot distinguish which quadrant the point is actually in (e.g., it
  would return the same value for `(im, re) = (1, 1)` as for
  `(im, re) = (-1, -1)`, even though those are opposite points 180°
  apart); `atan2` uses the sign of both arguments to resolve the angle
  correctly across the full ±180° range.

Every subcarrier in every packet's CSI buffer is one such `(im, re)`
pair, and the sequence of ~64 (or ~192, with all three training-field
groups enabled — document 2, §2.4, §2.6–2.7) pairs is simply
`H` measured independently at each subcarrier's frequency for that one
packet.

## 4.4 Phase unwrapping

`atan2` is mathematically defined to return a value in the range
`(-π, +π]` radians (equivalently, (-180°, +180°]). This is a real
mathematical constraint, not a physical one: phase is inherently cyclic
(370° of rotation is physically identical to 10° of rotation — document
1, §1.1), so *any* function that reports "the" phase has to pick some
consistent single-cycle range to report it in, and `atan2`'s convention
is to center that range on zero. The consequence: if the *true* physical
phase drifts smoothly past ±π radians (±180°) from one packet to the
next — which it will, because nothing about the physical multipath
channel obeys `atan2`'s arbitrary cut point — the raw `atan2` output
**appears** to jump discontinuously from near `+π` to near `-π` (or vice
versa), even though physically nothing discontinuous happened at all.

This is called **phase wrapping**, and it is purely an artifact of how
the value is reported, not a real event in the channel. Left uncorrected,
it corrupts any calculation that looks at phase *change* over time
(exactly what a motion-sensing algorithm needs) — a smooth, real phase
drift gets misread as a huge, fake, instantaneous jump every time it
crosses the ±π boundary.

**Phase unwrapping** is the standard fix: walk through a sequence of
phase readings in order, and whenever the difference between consecutive
readings has a magnitude greater than π, add or subtract a full 2π
(360°) to bring it back into a physically continuous range — under the
assumption that the true phase couldn't plausibly have jumped by more
than half a cycle between two consecutive closely-spaced samples (a
reasonable assumption for two WiFi packets received a fraction of a
second apart, since that would require an implausibly large real
physical change in that short a time).

**Worked example.** Consider four consecutive raw `atan2` readings, in
radians:

```
raw:  3.0    3.1    -3.1    -3.0
```

Reading this literally, the phase appears to swing wildly: up slightly,
then drop by more than 6 radians, then rise slightly again. Physically
implausible for a channel that isn't changing violently frame to frame.
Unwrapping, step by step:

1. **3.0 → 3.1**: raw difference = `3.1 - 3.0 = 0.1`. Magnitude ≤ π, so
   no correction needed. Unwrapped value: **3.1**.
2. **3.1 → -3.1**: raw difference = `-3.1 - 3.1 = -6.2`. Magnitude > π
   (6.2 > π ≈ 3.14159), and it's negative, so add 2π:
   `-6.2 + 2π ≈ -6.2 + 6.283 = 0.083`. This corrected difference (0.083)
   is added to the *previous unwrapped value*: `3.1 + 0.083 = 3.183`.
   Unwrapped value: **≈3.18**.
3. **-3.1 → -3.0**: raw difference = `-3.0 - (-3.1) = 0.1`. Magnitude ≤ π,
   no correction. Added to the previous unwrapped value:
   `3.183 + 0.1 = 3.283`. Unwrapped value: **≈3.28**.

```
raw:        3.0     3.1    -3.1    -3.0
unwrapped:  3.0     3.1     3.18    3.28
```

The unwrapped sequence now reads as what it physically was the whole
time: a smooth, small, continuous phase drift — not a violent swing. The
general rule, applied at every step `k`:

```
diff = raw[k] - raw[k-1]
if diff > π:  diff -= 2π
if diff < -π: diff += 2π
unwrapped[k] = unwrapped[k-1] + diff
```

This document's own codebase context notes that `csi_presence` performs
this unwrapping before computing its phase-variance motion metric, and
that `csi_test`'s report generation does not — a gap worth being aware of
if you're looking at raw phase values or a phase-based chart from either
project's tooling and see values that look like they're clipped or
bouncing near ±π; that's very likely wrapping, not a real signal.

## 4.5 Why raw CSI is noisy, and why systems lean on statistics

Even setting aside wrapping (a reporting artifact with a clean fix), raw
CSI amplitude and phase carry real measurement noise that has nothing to
do with the physical environment you're trying to sense:

- **Hardware calibration offsets.** No two radio chains (even on the
  same chip) are perfectly identical — small manufacturing variations in
  the RF front end, mixers, and ADC introduce a fixed or slowly-drifting
  offset in amplitude and phase that's specific to that piece of
  hardware, not the channel being measured.
- **Phase noise.** The local oscillators used in down-conversion
  (document 3, §3.4, step 3) are not perfectly stable single-frequency
  sources — they have some inherent jitter/instability, which shows up
  as small random fluctuations layered onto the true phase value on top
  of whatever the channel itself contributed.
- **AGC steps.** Automatic gain control (document 3, §3.4, step 2)
  adjusts the receiver's amplification in discrete steps as signal
  strength changes. A visible amplitude "jump" between two packets can be
  the AGC re-adjusting its gain setting rather than the physical channel
  itself changing by that amount — this is a receiver-side artifact that
  can look, at a glance, like a real environmental event.
- **Ordinary thermal/electronic noise**, present in any real analog
  measurement, adds small random variation to every sample independent of
  everything above.

None of these sources are exotic — they're the ordinary reality of
working with real analog RF hardware rather than an idealized textbook
signal. The practical consequence is that **no single raw CSI reading —
one amplitude, one phase, from one packet — should be trusted as a direct
measurement of "the room changed."** A real physical event (motion) has
to produce a change large enough, and sustained enough over multiple
packets, to be distinguishable from this baseline noise floor.

This is exactly why both firmware projects in this codebase compute
**statistics over a window of packets** rather than reacting to
individual readings: `csi_test`'s variability score is a standard
deviation across subcarriers within each packet (itself somewhat noise-
resistant because it aggregates many subcarriers at once, though document
3, §3.8, notes this specific metric can be fooled by static spatial
complexity); `csi_presence`'s motion-energy score is a variance of one
subcarrier's *unwrapped* phase across roughly a one-second sliding window
of packets, with an **adaptively calibrated threshold** (learned from an
observed ambient/empty-room baseline, specifically to absorb whatever
hardware-specific noise floor that particular device and environment
happen to have, rather than assuming a fixed universal threshold), and
**hysteresis** (different thresholds for entering vs. leaving the
"presence detected" state, so a score dithering right at the boundary
doesn't flicker the output rapidly). Neither of these is an arbitrary
engineering choice — both follow directly from the fact that raw CSI, at
the single-sample level, mixes a real physical signal with several
independent, unavoidable sources of hardware and RF noise, and separating
the two requires looking at behavior over time, not any one reading in
isolation.

## 4.6 Where to go next

This document, together with documents 1–3, covers the physics and
signal-processing theory behind CSI: what it is, why it exists, how it's
represented as data, and why it needs careful handling (unwrapping,
statistics) before it's useful. For how a specific system turns this
noisy physical signal into an actual presence/motion decision — the
calibration procedure, the subcarrier-selection logic, the hysteresis
state machine, and what real-hardware testing found — see
`projects/csi_presence/docs/v1.md` (the document referred to in this
series' brief as `02-algorithms-and-concepts.md`; at the time of writing
the project's docs directory contains a single file, `v1.md`, covering
that ground). For how another system captures and visualizes raw CSI
over time instead of making a live decision — the amplitude heatmap,
the RSSI/variability time series, and the auto-detected-band heuristic —
see `projects/csi_test/docs/v1.md` (likewise referred to as
`02-algorithms-and-concepts.md` in this series' brief).
