# 2. OFDM and the 802.11 PHY

This document explains why WiFi splits its channel into many narrow
subcarriers instead of using one wide carrier, what makes that scheme
work, and how a real 802.11 frame is structured on the air. It assumes
you've read (or already know) the content of
[`01-waves-and-modulation.md`](./01-waves-and-modulation.md): a wave has
amplitude/frequency/phase, its state can be written as a complex number
`A·e^(jφ)`, and bits are encoded onto a wave via a constellation
(BPSK/QPSK/QAM). This document is the direct bridge from that generic
radio theory to the specific `wifi_csi_config_t` struct used in this
codebase.

## 2.1 The problem with one wide carrier: multipath and ISI

Suppose you tried to send WiFi's full 20 MHz of data on a single carrier,
modulated as fast as possible to cram bits through. A radio signal
indoors never travels a single, clean path from transmitter to receiver
— it reflects off walls, furniture, the floor, ceiling, people, and
arrives at the receiver as many delayed copies of the same signal,
superimposed (this is **multipath**, covered in depth in document 3).
Each copy takes a different physical path length, hence a different
travel time.

If you're transmitting one very fast, wideband signal, each of those
delayed copies smears the trailing edge of one symbol into the leading
edge of the next. This is **inter-symbol interference (ISI)**: the
receiver can no longer cleanly tell where one symbol's energy ends and
the next begins, because reflections are still arriving from *the
previous* symbol while the *next* one starts. The faster you transmit
(the shorter each symbol is), the worse this gets, because the
reflections' delay spread (typically tens to hundreds of nanoseconds
indoors) becomes a larger fraction of the symbol duration.

**OFDM (Orthogonal Frequency-Division Multiplexing)** solves this by
inverting the strategy: instead of one fast wideband carrier, split the
same total bandwidth into many (WiFi: tens to over a hundred) narrow
subcarriers, each modulated *slowly*. Slower modulation means a much
longer symbol duration per subcarrier — long enough that multipath delay
spread becomes a small fraction of the symbol, manageable with a guard
interval (§2.3) instead of destroying the data.

## 2.2 Orthogonality: how you pack subcarriers without them interfering

"Orthogonal" here has a precise, useful meaning, not just "not
interfering" as a hand-wave. Each subcarrier's spectral shape (its energy
as a function of frequency) is not a sharp spike — real, time-limited
signals always spread out in frequency — but a `sinc`-shaped curve
(`sin(x)/x`), which oscillates and crosses zero at regular intervals
away from its center frequency.

OFDM's trick is to space subcarriers *exactly* at the interval where each
subcarrier's sinc-shaped spectrum crosses zero at every other
subcarrier's center frequency:

```
amplitude
   │        ⌢ subcarrier k        ⌢ subcarrier k+1
   │       ╱ ╲                   ╱ ╲
   │      ╱   ╲       ⌢         ╱   ╲
   │     ╱     ╲     ╱ ╲       ╱     ╲
   │    ╱       ╲   ╱   ╲     ╱       ╲
   │───╱─────────╲─╱─────╲───╱─────────╲───────  frequency
       │           │       │
     center       zero   center of k+1
     of k       crossing  (= peak lands exactly
                 of k     on k's zero crossing)
```

At the exact center frequency of any subcarrier, every *other*
subcarrier's spectrum is at zero. This means that when the receiver
samples at exactly the right frequency (via an FFT — see §2.5 and
document 3), it picks up the full contribution of the intended
subcarrier and *nothing* from its neighbors, even though their spectra
technically overlap in the frequency domain. This overlap-without-crosstalk
is what "orthogonal" means, and it's what lets WiFi pack subcarriers much
closer together than if it needed a clean, non-overlapping frequency gap
between each one (which older frequency-division schemes required).

The spacing required for this to hold is `Δf = 1 / T_symbol`, where
`T_symbol` is the duration of the (useful, non-guard) part of one OFDM
symbol. WiFi's legacy OFDM (802.11a/g, and 802.11n non-HT-compatible
mode) uses `T_symbol = 3.2 μs`, giving subcarrier spacing:

```
Δf = 1 / 3.2μs = 312.5 kHz
```

This number — 312.5 kHz — is a physical constant of WiFi OFDM across
802.11a/g/n/ac/ax on the legacy-compatible subcarrier grid, and it's
worth remembering because it's what determines how many subcarriers fit
into a given channel bandwidth (§2.4).

## 2.3 The cyclic prefix (guard interval)

Even with subcarriers spaced slowly enough that each symbol is long, the
first fraction of every symbol at the receiver is still "contaminated" by
the tail-end reflections of the *previous* symbol arriving late. OFDM's
fix is the **cyclic prefix (CP)**, also called the **guard interval
(GI)**: before transmitting each OFDM symbol, copy its own *last* portion
and prepend it to the front.

```
   original symbol (duration T_symbol = 3.2 μs)
   ┌──────────────────────────────────────────┐
   │  a  b  c  d  e  f  g  h  ................│
   └──────────────────────────────────────────┘
                                          copy this tail →
   ┌───────┐┌──────────────────────────────────────────┐
   │..g  h ││  a  b  c  d  e  f  g  h  ................│
   └───────┘└──────────────────────────────────────────┘
     cyclic          original symbol (unchanged)
     prefix
   (duration = guard interval, e.g. 0.8 μs)
```

Two things this achieves:

1. **It absorbs multipath delay spread.** As long as the total delay
   spread of all significant reflections is shorter than the guard
   interval, the "smeared" region at the very start of the received
   symbol falls entirely within the cyclic prefix — which the receiver
   simply discards before running its FFT. Everything after that point is
   ISI-free.
2. **It preserves the math OFDM relies on.** Because the prefix is a
   *copy* of the symbol's own tail rather than silence or arbitrary data,
   the reflected, delayed copies of the signal still look — to the FFT —
   like a clean, undelayed periodic signal, which is a mathematical
   requirement for the orthogonality property in §2.2 to hold in the
   presence of delay.

WiFi historically defines two standard guard interval lengths: **800 ns**
(the "long" or normal GI, mandatory, safe for typical indoor delay
spreads) and **400 ns** (the "short" GI, optional, higher throughput
since less time is spent on redundant prefix data per symbol, but only
safe when the environment's multipath delay spread is small enough not to
overrun it). Choosing a GI is a direct tradeoff: shorter GI wastes less
time on repeated data (higher throughput) but tolerates less multipath
delay before ISI reappears. With `T_symbol = 3.2 μs` and `GI = 0.8 μs`,
total OFDM symbol duration on the air is `4.0 μs` for the long-GI case.

## 2.4 Subcarrier counts in real WiFi channels

A WiFi channel's total occupied bandwidth (20 MHz, 40 MHz, 80 MHz, ...)
divided by the 312.5 kHz subcarrier spacing gives the total number of
subcarrier "slots" available in that bandwidth. Not all of them carry
data:

- **Data subcarriers** carry the actual modulated payload bits.
- **Pilot subcarriers** carry a fixed, known reference pattern (not user
  data), used by the receiver to track residual phase and frequency
  drift across an OFDM symbol.
- **The DC subcarrier** (dead-center, 0 Hz offset from the channel
  center) is left null (unused) because RF hardware imperfections
  (local-oscillator leakage, DC offset in the analog front end) make the
  very center of the channel unreliable to use for data.
- **Guard-band subcarriers** at the upper and lower edges of the
  allocated slots are left null to keep the transmitted signal's energy
  from spilling into adjacent channels — a regulatory and practical
  requirement, since a subcarrier's sinc-shaped spectrum (§2.2) never
  drops to exactly zero energy away from its own band.

For a 20 MHz legacy OFDM channel (802.11a/g), the standard, well-known
split is: 64 total subcarrier slots (of which the FFT actually computes
64 points), with **52 used** (48 data + 4 pilot), 1 DC null, and the
remaining 11 as guard-band nulls at the edges. 802.11n's HT20 mode
(20 MHz, high-throughput) is very similar in structure — also a 64-point
FFT — but uses a slightly different split (commonly cited as 56 used
subcarriers: 52 data + 4 pilot, with fewer guard-band nulls than legacy
OFDM, since 802.11n narrowed the unused guard bands to reclaim a few more
usable subcarriers). 40 MHz HT channels roughly double this to a
128-point FFT with on the order of 114 used subcarriers. These exact
"used" counts vary slightly by 802.11 amendment and are the kind of
figure worth double-checking against the specific standard (802.11n vs.
ac vs. ax) if you need it to the last subcarrier — the concept (64-point
FFT for 20 MHz, split into data/pilot/DC-null/guard-null) is the durable
fact; the exact usable count is a standards-committee decision that has
shifted slightly between amendments.

**Connecting this to the codebase:** real captures in this project
showed **~64 raw values per training-field group** in the CSI buffer,
matching the 64-point FFT size for a 20 MHz channel. This is the
*hardware buffer length* (one raw complex sample per FFT bin, including
some bins that are nominally null/guard and may show near-zero or
garbage amplitude — this is consistent with `csi_test`'s own finding that
subcarrier index 0 was a "near-zero-amplitude edge subcarrier"), not the
"used-for-data" count of 52 or 56 from the standard — the ESP32's CSI
extraction exposes the full raw per-bin channel estimate, not a
filtered-down "data subcarriers only" view. With `lltf_en`, `htltf_en`,
and `stbc_htltf2_en` all enabled, three independent ~64-value groups are
captured per packet — one per training field type, explained in §2.6 —
which is why real HT packets in this codebase show **~192 total values**
(3 × 64), while non-HT/legacy-only frames (e.g. ACKs, which contain only
an L-LTF) show just the one ~64-value group.

## 2.5 From subcarriers to CSI: what the receiver does with all this

Skipping ahead slightly (document 3 covers the full receive chain in
detail): the receiver's job, per OFDM symbol, is to take the
time-domain waveform it captured, strip off the cyclic prefix, and run a
**Fast Fourier Transform (FFT)** on what's left. The FFT is the
mathematical operation that takes a signal that's a mixture of many
frequencies added together and separates it back into one complex number
*per frequency bin* — i.e., exactly one complex sample per subcarrier,
in the `A·e^(jφ)` form from document 1. This is the mechanism that turns
"one antenna's voltage waveform over time" into "one independent
measurement per subcarrier," and it's also precisely what produces a
per-subcarrier **channel estimate** when compared against subcarrier
values the receiver already knows should have been sent (the training
fields, next section) — that channel estimate is CSI, formally defined
in document 4.

## 2.6 The 802.11 PHY preamble and training fields

Every 802.11 frame begins with a standardized preamble whose entire
purpose is to help the receiver *before* it can decode any actual data:
find the signal, synchronize its clock, estimate the channel, and learn
what mode/rate the rest of the frame uses. The relevant fields, roughly
in transmission order for an HT (802.11n) frame in mixed mode
(backward-compatible with older receivers):

```
┌────────┬────────┬────────┬────────┬────────┬─────────┬─────────┬─────────┐
│ L-STF  │ L-LTF  │ L-SIG  │ HT-SIG │ HT-STF │ HT-LTF  │(HT-LTF2)│  Data   │
└────────┴────────┴────────┴────────┴────────┴─────────┴─────────┴─────────┘
  legacy   legacy   legacy    HT       HT       HT LTF   2nd HT-LTF  payload
  short    long     signal   signal   short    (channel   (STBC       symbols
  train    train    field    field    train     estimate   only)
  field    field              (rate,   field       )
                              length)
```

- **L-STF (Legacy Short Training Field).** A short, repetitive known
  pattern, present in *every* 802.11 frame regardless of mode, used for
  coarse packet detection (is a signal present at all?), automatic gain
  control (AGC — setting the receiver's amplifier gain so the signal
  isn't clipped or too weak to resolve), and coarse timing/frequency
  synchronization.
- **L-LTF (Legacy Long Training Field).** A longer known pattern, present
  in every frame, used for fine timing synchronization and the *first*
  channel estimate — this is what `lltf_en` in `wifi_csi_config_t`
  enables capture of. Because it's present in literally every 802.11
  frame (including old 802.11a/g-only frames, ACKs, and anything a
  non-HT-capable device could still decode), it's the one training field
  guaranteed to exist regardless of what kind of packet was received —
  which is exactly why non-HT frames like ACKs still yield a ~64-value
  CSI buffer even when they carry no HT-LTF at all.
- **L-SIG (Legacy Signal field).** Carries the legacy rate and length
  fields — this is what lets even old 802.11a/g hardware that can't
  decode an 802.11n frame's payload still correctly compute *how long*
  the frame will occupy the channel, so it can defer transmission rather
  than collide with it ("legacy protection").
- **HT-SIG (HT Signal field).** 802.11n-specific signaling: MCS index,
  bandwidth (20/40 MHz), whether STBC is used, and other mode
  information the receiver needs before it can correctly interpret the
  HT training fields and payload that follow.
- **HT-STF (HT Short Training Field).** A second short training field,
  present only in HT-mode frames, used to re-tune AGC for the HT portion
  of the frame (which can use a different number of spatial streams than
  the legacy portion implies) before the HT-LTF arrives.
- **HT-LTF (HT Long Training Field).** The channel estimate for the
  actual HT (802.11n) transmission — this is what `htltf_en` enables
  capture of. In multi-spatial-stream transmissions, one HT-LTF symbol is
  transmitted per spatial stream, each on a different, known orthogonal
  pattern, so the receiver can separate and estimate the channel for each
  transmit antenna independently.
- **(second) HT-LTF — STBC.** Explained in §2.7 below —
  `stbc_htltf2_en`.

`ltf_merge_en = true` in this codebase's config tells the hardware to
average the L-LTF-derived and HT-LTF-derived channel estimates into a
single combined estimate where both exist for a given packet, rather than
exposing them as fully separate values — trading a small amount of detail
for a less noisy estimate (averaging independent noisy measurements of
what should be the same physical channel reduces the noise on the
combined estimate, at the cost of not being able to see the LLTF and
HT-LTF estimates as separate readings).

Newer 802.11 amendments extend this same pattern with their own
signaling and training fields — 802.11ac (VHT) adds VHT-SIG-A, a
VHT-STF, VHT-LTF(s), and VHT-SIG-B; 802.11ax (HE) adds HE-SIG-A/B, an
HE-STF, and HE-LTF(s) — each amendment's extra fields exist for the same
underlying reasons (mode signaling for the new capability, a fresh
short/long training pair for the new mode's AGC and channel estimation).
The ESP32-S3's CSI extraction in this codebase is configured around the
legacy and HT (802.11n) fields specifically (`lltf_en`, `htltf_en`,
`stbc_htltf2_en`); this document does not claim precise support details
for VHT/HE CSI extraction on this chip, which is a hardware/driver-level
question outside physics.

## 2.7 STBC and the second HT-LTF

**STBC (Space-Time Block Coding)** is a transmit-diversity technique: the
same data is encoded and spread across multiple transmit antennas *and*
multiple time slots, using a specific known pattern, so that a receiver
with even a single antenna can combine the multiple received copies to
recover the original data more reliably than from any one copy alone.
The core idea is redundancy against fading: if one transmit antenna's
path to the receiver happens to be in deep destructive-interference
fade at a given moment (see document 3 on multipath), a *different*
transmit antenna's independently-faded path may not be, and STBC's
known encoding structure lets the receiver combine both to recover the
signal despite either one alone being unreliable.

Because STBC transmits from (conceptually) more than one antenna path
using an encoding scheme distinct from ordinary spatial multiplexing, the
receiver needs a **separate channel estimate for the STBC-encoded
stream**, on top of whatever ordinary HT-LTF channel estimate it already
has for the main spatial streams — hence a second HT-LTF symbol,
specifically for STBC. `stbc_htltf2_en` in this codebase's config is
what tells the hardware to also capture and expose this second channel
estimate when a received packet used STBC. For CSI-sensing purposes, this
is largely incidental (STBC is a link-reliability feature aimed at
demodulation correctness, not something designed for or against CSI
extraction) — but it's part of why a fully-enabled config captures up to
*three* ~64-value groups per packet (L-LTF, HT-LTF, STBC-HT-LTF2) rather
than one or two, as observed in this project's real captures.

## 2.8 Summary: the bridge to document 3

At this point you have the full vocabulary needed to follow the physical
story in the next document: a WiFi transmitter sends dozens of
simultaneous, orthogonally-spaced subcarriers, each independently
modulated; a standardized preamble of known training-field patterns
(L-LTF, HT-LTF, and optionally a second HT-LTF for STBC) precedes the
actual payload specifically so the receiver can measure how the channel
distorted something it already knew the true value of; and that
per-subcarrier measurement — the channel's effect on each subcarrier,
expressed as one complex number `A·e^(jφ)` per subcarrier — is CSI.
Document 3, [`03-wifi-physics-deep-dive.md`](./03-wifi-physics-deep-dive.md),
now asks: physically, in the real world, what *causes* that channel
distortion, and what does measuring it tell you about a room?
