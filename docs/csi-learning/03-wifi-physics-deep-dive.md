# 3. The Physics of WiFi Sensing: A Full Walkthrough

This is the central document in this series. It follows one continuous
physical story — from "what does a router actually send" to "what does a
phase change in the received signal actually mean about the physical
world" — in the order you'd naturally ask the questions if you were
trying to understand, from scratch, why moving around a room changes the
numbers this codebase prints to a serial console.

It assumes the vocabulary built in the previous two documents: a wave has
amplitude/frequency/phase and can be written as a complex number
`A·e^(jφ)` (document 1); WiFi splits its channel into many orthogonal
subcarriers using OFDM, with known training-field patterns in the
preamble used for channel estimation (document 2). Brief recaps are
given inline where needed so this document is still followable mostly on
its own.

---

## 3.1 What does a WiFi router actually send, physically?

Strip away every protocol detail and a WiFi router transmitting is doing
one physical thing: driving an antenna with a rapidly oscillating
electric current, at a carrier frequency around 2.4 GHz or 5 GHz (see
document 1, §1.2, for why those frequencies), causing the antenna to
radiate that oscillation outward as an electromagnetic wave. This is the
same underlying physics as an AM/FM radio transmitter, a TV broadcast
antenna, or a cell tower — only the frequency and the fine structure of
what's riding on top of the carrier differ.

Typical WiFi transmit power is modest: consumer routers and client
devices are commonly limited (by regulation and by design, to manage
battery life and interference with neighboring networks) to roughly
**tens of milliwatts to a few hundred milliwatts** — often expressed in
dBm, where 20 dBm = 100 mW and 30 dBm = 1 W; most consumer WiFi gear sits
somewhere in the 15–20 dBm range per chain. This is a genuinely tiny
amount of power compared to a broadcast radio or cell tower transmitter,
which is one reason WiFi's range is measured in tens of meters rather
than kilometers — and one reason path loss and multipath (§3.3) matter
so much: there isn't a large power budget to spare, so environmental
effects that attenuate or distort the signal have an outsized practical
effect on what the receiver actually sees.

The antenna itself doesn't radiate uniformly with perfect precision in
every direction (real antennas have some directional pattern, often
close to omnidirectional in the horizontal plane for consumer gear but
never perfectly so), and it doesn't radiate into a vacuum — it radiates
into a room, with walls, furniture, and people already present before
any of what follows even happens.

## 3.2 How does it send data on that wave?

A raw, unmodulated 2.4 GHz oscillation carries no information — it's the
same predictable shape forever (document 1, §1.4). Data gets encoded onto
it via **modulation**: systematically varying the wave's amplitude and/or
phase in a pattern that represents specific bits, using a **constellation**
(BPSK/QPSK/16-QAM/64-QAM/... — document 1, §1.4).

WiFi doesn't do this on one carrier; it does it on dozens of narrow
**subcarriers** simultaneously (**OFDM** — document 2), each carrying its
own modulated data, chosen so their spectra are mathematically orthogonal
and don't interfere with each other despite overlapping in frequency.

A real 802.11 data frame, in transmission order, looks conceptually like:

```
┌───────────────────────────────────────────────────────────────────┐
│  PREAMBLE                                    │  SIGNALING │ PAYLOAD│
│  (known training-field patterns, not data)    │ (mode/rate)│ (data) │
├──────────┬──────────┬──────────┬─────────────┼────────────┼────────┤
│  L-STF   │  L-LTF   │  HT-STF  │  HT-LTF(s)   │  HT-SIG /  │  OFDM  │
│ (find +  │ (timing  │ (re-tune │ (channel     │  L-SIG     │ data   │
│  AGC)    │  sync +  │  AGC)    │  estimate)   │ (rate,     │ symbols│
│          │  1st ch. │          │              │  length,   │        │
│          │  estimate)│         │              │  MCS)      │        │
└──────────┴──────────┴──────────┴─────────────┴────────────┴────────┘
```

The preamble's training fields (L-STF, L-LTF, HT-STF, HT-LTF, and
optionally a second HT-LTF for STBC — document 2, §2.6–2.7) carry
**known** patterns — bit sequences both transmitter and receiver already
agree on in advance, standardized by 802.11. Nothing about *content* is
communicated by the training fields; their entire purpose is to give the
receiver a reference signal whose true, undistorted value it already
knows, so that by comparing what it actually received against what it
knows was sent, it can measure exactly how the channel between the two
radios distorted the signal. That measurement is CSI, formalized
mathematically in §3.5 below and in document 4. Everything after the
signaling fields is the actual payload — the modulated data being
communicated — but by the time the payload arrives, the receiver has
already used the preamble to solve the channel-distortion problem,
because that distortion has to be undone (equalized) before the payload
can be demodulated correctly at all.

## 3.3 What happens to that wave between transmitter and receiver?

This is where the physical story of CSI sensing actually lives. A radio
wave leaving the router's antenna does not travel to the receiving
ESP32-S3 along one clean, single path. It travels along *many* paths
simultaneously:

- The **direct (line-of-sight) path**, straight from transmitter to
  receiver, if nothing physically blocks it.
- **Reflected paths** — off walls, the floor, the ceiling, furniture,
  doors, and people — each of which is a physically longer path than the
  direct one (light/radio travels at a fixed speed, so a longer path
  means a longer travel time), each of which loses some energy on
  reflection (different materials absorb/reflect differently), and each
  of which may also be **phase-shifted** by the reflection itself
  (reflecting off certain materials shifts phase by a fixed amount, e.g.
  close to 180° for a good conductor) on top of the phase shift caused
  simply by the extra path length traveled.

```
                       wall
                    ┌──────────
                    │      ╲
                    │       ╲  reflected path 2
        router      │        ╲
          📡─────────────────────────────────────╲
          │  ╲     direct (line-of-sight) path     ╲
          │   ╲______________________________________╲___📶 ESP32-S3
          │   ╱                                        ╱  (receiver)
          │  ╱      reflected path 1                   ╱
          │ ╱ (off floor/furniture)                   ╱
          └╱____________________________________________╱
                    floor / furniture
```

At the receiving antenna, **all of these arriving copies of the same
transmitted signal add together** — they superimpose, because
electromagnetic fields are additive. Each copy arrives with its own
amplitude (attenuated by distance and reflection loss) and its own phase
(rotated by however many wavelengths of path length it traveled, plus any
reflection-induced phase shift). Whether two arriving copies reinforce
each other (**constructive interference** — their phases roughly align,
so their amplitudes add) or cancel each other out (**destructive
interference** — their phases are roughly opposite, so they subtract) is
extremely sensitive to the exact path lengths involved, because "roughly
aligned" vs. "roughly opposite" phase is determined by path-length
differences measured in *fractions of a wavelength* — and WiFi's
wavelength is only ~12.5 cm at 2.4 GHz (document 1, §1.2). This
phenomenon — many delayed, attenuated, phase-shifted copies of one
transmitted signal summing at the receiver — is called **multipath**,
and it is the single physical mechanism underlying essentially everything
CSI sensing measures.

Two consequences of multipath matter enormously for what follows:

1. **The combined signal the receiver sees depends on the exact geometry
   of the room** — every wall, every piece of furniture, every person's
   position contributes its own reflected path with its own delay,
   attenuation, and phase shift, all summing together. Change *any* of
   those paths — including by moving a person through the room, which
   changes what that person's body reflects, absorbs, or blocks — and the
   sum at the receiver changes.
2. **This is frequency-dependent.** Because "roughly aligned" vs.
   "roughly opposite" phase depends on path length measured in
   wavelengths, and different subcarriers (document 2) sit at slightly
   different frequencies (hence slightly different wavelengths), the same
   physical room geometry can cause constructive interference at one
   subcarrier's frequency and destructive interference at a
   nearby subcarrier's frequency, simultaneously. This is exactly why CSI
   is measured **per subcarrier** rather than as one single number for
   the whole channel — a single "average" signal strength measurement
   (like RSSI, which is a summary of total received power) throws away
   this rich, frequency-resolved structure that per-subcarrier CSI
   preserves.

## 3.4 What happens at the receiving radio?

The ESP32-S3's WiFi receiver takes this messy, multipath-summed analog
waveform and turns it into the per-subcarrier complex numbers this
codebase's callback receives. At a conceptual level (real hardware
details vary and are proprietary to the chip vendor, but the pipeline
below is the standard structure every OFDM receiver implements):

```
┌────────┐   ┌───────────┐   ┌──────────────┐   ┌─────┐   ┌─────────┐   ┌────────────┐
│ Antenna│──▶│ RF front  │──▶│ Down-convert │──▶│ ADC │──▶│  Strip  │──▶│    FFT     │
│(receives│   │ end (LNA, │   │ (mix down to │   │(sam-│   │ cyclic  │   │ (separate  │
│ summed  │   │ filters,  │   │  baseband    │   │ ple │   │ prefix, │   │ into one   │
│ multi-  │   │ AGC)      │   │  I/Q)        │   │ ed  │   │ window  │   │ complex #  │
│ path    │   │           │   │              │   │wave-│   │ one     │   │ per sub-   │
│ signal) │   │           │   │              │   │form)│   │ symbol  │   │ carrier)   │
└────────┘   └───────────┘   └──────────────┘   └─────┘   └─────────┘   └──────┬─────┘
                                                                                 │
                                                                                 ▼
                                                                    ┌─────────────────────┐
                                                                    │ Compare each         │
                                                                    │ subcarrier's FFT     │
                                                                    │ output for a known   │
                                                                    │ training-field       │
                                                                    │ symbol against the   │
                                                                    │ known transmitted    │
                                                                    │ value → per-         │
                                                                    │ subcarrier channel   │
                                                                    │ estimate = CSI       │
                                                                    └─────────────────────┘
```

Step by step:

1. **Antenna** picks up the summed multipath waveform described in §3.3
   — a real physical voltage induced by the incoming electromagnetic
   field.
2. **RF front end** — a low-noise amplifier (LNA) boosts the (typically
   very weak) received signal, filters out energy outside the WiFi band,
   and automatic gain control (AGC, which the L-STF/HT-STF training
   fields exist partly to help set — document 2, §2.6) adjusts
   amplification so the signal fills the receiver's usable dynamic range
   without clipping.
3. **Down-conversion** shifts the signal from its original ~2.4/5 GHz
   carrier frequency down to a much lower "baseband" frequency range that
   the receiver's digital circuitry can actually sample — this step
   also produces the I and Q components (document 1, §1.3) by mixing
   against two local reference oscillators 90° apart in phase.
4. **ADC (analog-to-digital converter)** samples the baseband I/Q
   waveform at discrete time steps, turning it from a continuous physical
   voltage into digital numbers — signed integers, quantized to some bit
   depth — for everything downstream to work with digitally. (The signed
   8-bit `(im, re)` values this codebase reads are, at root, exactly this
   kind of digitized I/Q sample, further processed by the WiFi hardware's
   channel-estimation logic before being handed to your callback.)
5. **Cyclic prefix stripping / symbol windowing** discards the
   guard-interval portion of each received OFDM symbol (document 2, §2.3)
   — the part contaminated by delayed multipath reflections of the
   *previous* symbol — keeping only the clean windowed portion.
6. **FFT** takes that windowed time-domain waveform and mathematically
   separates it into one complex number per subcarrier frequency
   (document 2, §2.5) — this is the step that turns "one antenna's
   voltage over time" into "one independent measurement per subcarrier."
7. **Channel estimation**, specifically for training-field symbols: the
   receiver already knows, by standard, exactly what complex value each
   subcarrier of the L-LTF/HT-LTF *should* equal if the channel were
   perfect (no distortion at all). Dividing the actually-received
   complex value by the known-transmitted complex value, subcarrier by
   subcarrier, gives exactly how much that subcarrier's amplitude was
   scaled and its phase was rotated by everything the wave physically
   passed through — antenna to antenna, including every multipath
   reflection along the way (§3.3). **This per-subcarrier ratio is CSI.**

## 3.5 What does the channel estimate (CSI) actually represent, mathematically?

CSI, per subcarrier, per transmit/receive antenna pair, is one complex
number:

```
H = A · e^(jφ)
```

— exactly the same form as a single wave's state from document 1, §1.3,
because that's precisely what it is: **the transfer function of
everything physically between the transmit antenna and the receive
antenna, at that one specific subcarrier's frequency.** "Transfer
function" here means literally: if `X` is the known complex value that
was transmitted on that subcarrier (from the training field) and `Y` is
what the receiver actually measured, then `H = Y / X`, and for any other
signal sent on that same subcarrier under the same channel conditions,
`Y' ≈ H · X'` approximately holds — this is exactly why computing H lets
a receiver *undo* the channel's distortion (divide the received payload
symbol by `H` to recover an estimate of what was actually sent), which is
the equalization step every OFDM receiver performs on ordinary data,
independent of any sensing use.

`A` (amplitude of `H`) tells you how much that subcarrier's signal was
attenuated or reinforced end-to-end — everything from free-space distance
loss to constructive/destructive multipath summation (§3.3) to
absorption by any obstruction. `φ` (phase of `H`) tells you the *net*
phase rotation accumulated over the entire path — dominated, for a fixed
antenna geometry, by total path length in wavelengths, but also including
any phase shift introduced by reflective materials themselves.

Concretely, in this codebase: `H` for one subcarrier is directly built
from the `(im, re)` byte pair the hardware reports for that subcarrier
slot, with `A = amplitude = sqrt(im² + re²)` and `φ = phase = atan2(im, re)`
— see document 4 for the full data-structure-level treatment, including
the phase-unwrapping issue that raw `atan2` introduces.

## 3.6 What does an amplitude change tell you, physically?

`A` changing, packet to packet, at a given subcarrier means the net
result of every constructive/destructive interference contribution to
that subcarrier changed — which happens whenever *any* of the multipath
paths summing at the receiver change in a way that shifts the balance of
that interference. Two broad physical causes, both observed in this
project's real captures:

- **A reflective path changes without blocking the direct path.** A
  person moving through part of the room — but not standing directly in
  the line-of-sight between router and ESP32-S3 — still changes what
  they reflect or absorb along whatever path grazes them. This adds,
  removes, or shifts one term in the multipath sum, nudging the
  constructive/destructive balance at each subcarrier. RSSI (roughly, the
  receiver's total received power, summed/averaged across the channel)
  can stay essentially unchanged in this case, because a shift in how
  power is distributed across the interference pattern at each
  individual subcarrier's frequency doesn't necessarily change the total
  power much — but the *spread* of amplitude values across different
  subcarriers, or the amplitude at any one subcarrier over time, changes
  measurably, because different subcarriers are affected differently
  (§3.3, point 2: multipath effects are frequency-dependent). This is
  exactly the physical mechanism behind the real capture in this
  project's data showing amplitude/phase "variability" (spread across
  subcarriers within a packet) roughly doubling while RSSI stayed flat —
  interpreted as motion disturbing multipath without blocking the direct
  path.
- **The direct path itself is blocked.** A person (or any sufficiently
  large, absorptive/reflective object) physically stepping into the
  line-of-sight between the two antennas removes or drastically
  attenuates what is normally the strongest single contributor to the
  received signal — the direct path is usually shorter and less
  attenuated than any reflected path, so it typically dominates the sum.
  Removing it doesn't just perturb the interference balance; it removes
  the largest term outright. This shows up as a sharp, large drop in
  received power — matching the real capture in this project's data
  where RSSI dropped from about **-38 dBm to -80/-84 dBm** — a drop of
  roughly 40+ dB, i.e., the received power falling by a factor of
  roughly 10,000×, consistent with losing a dominant direct path rather
  than a minor reflection.

## 3.7 What does a phase change tell you, physically?

Phase encodes **path length, in fractions of a wavelength**, accumulated
over the entire multipath sum reaching that subcarrier. This is the
single most important sensitivity fact in this entire document, so it's
worth working the numbers explicitly.

At 2.4 GHz, wavelength `λ ≈ 12.5 cm` (document 1, §1.2). A full 360°
(2π radians) of phase rotation corresponds to exactly one wavelength of
*round-trip* path-length change for a reflected path (the wave travels
the extra distance out to the reflector and back, so a reflector moving
by distance `d` changes that path's total length by `2d`). Working
backward: how far does a reflecting object have to move to produce a
readily-measurable phase shift, say 30° (a small fraction of a full
cycle, easily distinguished from noise)?

```
360° of phase  ↔  one full wavelength of path-length change (12.5 cm)
30° of phase   ↔  (30/360) × 12.5 cm ≈ 1.04 cm of path-length change
              ↔  ≈ 0.52 cm of actual reflector displacement
                 (since round-trip path length change = 2 × displacement)
```

A measurable phase shift can result from **sub-centimeter** movement of a
reflecting surface — a hand shifting slightly, a chest rising and
falling with breathing, a person shifting their weight. This is why
phase is often described as a *more sensitive* motion indicator than
amplitude: amplitude changes require the interference balance to shift
enough to noticeably change how much energy sums constructively vs.
destructively, which is a comparatively coarse effect — while phase
directly, linearly encodes path length, and path length responds to
*any* motion, however small, of anything reflecting a meaningful amount
of the signal's energy.

This directly explains a deliberate design choice in this codebase.
`csi_test` computes "variability" as the *spread of amplitude across all
subcarriers within one packet* — a single spatial snapshot. But a room
that is simply RF-complex (lots of furniture, lots of reflective
surfaces) can have naturally high amplitude spread across subcarriers
even with **nobody moving at all** — the spread reflects spatial
complexity, not motion, unless you also track it over time. `csi_presence`
instead tracks the **phase of one chosen subcarrier, over time**, and
computes its variance across a roughly one-second sliding window. This
metric is essentially blind to a static room's spatial complexity — a
motionless, cluttered room produces a phase that (aside from noise) does
not change from one packet to the next, however complex the interference
pattern is at any single instant — and it directly measures the thing
that actually indicates motion: **is the path length to that subcarrier's
dominant contributors changing over time.** Given the sensitivity worked
out above (sub-centimeter motion produces measurable phase shift at
2.4 GHz), a time-varying phase is a physically well-targeted proxy for
"something in the room is moving," in a way that a single packet's
amplitude spread is not.

## 3.8 Closing the loop: this is real physics, and it is genuinely hard

None of the above is exotic or speculative — it's the same channel
estimation every OFDM WiFi receiver performs on every packet anyway
(document 4, §4.2), for the mundane purpose of correctly demodulating
data. WiFi sensing does not require any special transmission, any
cooperation from the router beyond normal operation, or any hardware
beyond a standard WiFi radio capable of exposing this already-computed
internal value. "No cameras, no wearables" WiFi sensing works because
multipath and channel estimation are real, well-understood, thoroughly
standard radio-engineering phenomena — not because of any trick specific
to this project.

It is also, honestly, a genuinely hard and indirect measurement, for
reasons that follow directly from everything above:

- **Motion doesn't map to one clean number.** A single moving person
  changes many multipath paths simultaneously, at every subcarrier,
  differently, depending on exactly where they are, what they're near,
  and what they're doing. There is no simple, universal formula from
  "CSI reading" to "distance moved" or "number of people present" — only
  statistical tendencies (more motion → more variance in the affected
  measurements).
- **Multiple physical causes produce overlapping signatures.** As §3.6
  showed, both "motion disturbing a reflective path" and "motion blocking
  the direct path" perturb the received signal, but at very different
  magnitudes and via different mechanisms (RSSI-preserving spread vs. a
  sharp RSSI drop) — telling these apart, or telling either apart from
  ordinary environmental RF noise or interference from other devices in
  the same unlicensed band (document 1, §1.2), requires interpretation,
  not a single threshold on a single raw sample.
- **Real hardware adds its own noise on top of the physical signal** —
  covered in depth in document 4, §4.5 — calibration offsets, phase
  noise, and AGC gain steps all inject variation into raw CSI readings
  that has nothing to do with the physical environment at all.

This is precisely why both firmware projects in this codebase lean on
**statistics computed over time** — variance over a sliding window,
adaptively calibrated thresholds, hysteresis to avoid flicker — rather
than trusting any single raw amplitude or phase reading as a direct,
one-shot measurement of "someone is here." The physics tells you *what
kind* of signal to look for and *why* it should respond to motion; it
does not, on its own, hand you a clean decision boundary. Document 4
picks up exactly here, formalizing CSI as the data structure this
codebase actually works with, and document 4's closing section points to
each project's own documentation for how they turn this noisy physical
signal into a usable decision.
