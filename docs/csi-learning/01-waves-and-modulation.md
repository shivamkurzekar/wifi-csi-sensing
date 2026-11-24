# 1. Waves and Modulation

This document builds up, from nothing, everything you need to understand
a single WiFi subcarrier: what a radio wave physically is, why WiFi lives
where it lives in the spectrum, how to describe a wave's state as a
single complex number, and how digital bits get imprinted onto that wave.
Document 2 ([`02-ofdm-and-80211-phy.md`](./02-ofdm-and-80211-phy.md))
picks up from here and explains why WiFi uses many such waves
simultaneously instead of just one.

## 1.1 What a radio wave physically is

A WiFi router's antenna is, physically, a piece of metal with a voltage
driven across it at a very high frequency — billions of times a second.
Driving current through a conductor that fast makes electrons accelerate
back and forth, and an accelerating charge radiates energy as an
electromagnetic wave: a self-propagating oscillation of electric and
magnetic fields, moving outward through space (or through walls, air,
your body — anything that isn't a perfect conductor) at the speed of
light.

You don't need Maxwell's equations to work with CSI. You need four
numbers that describe the *state* of that oscillation at any point in
space and time:

- **Amplitude** — how strong the field is at its peak. Physically, this
  relates to signal power/energy; a bigger amplitude carries more energy.
- **Frequency** (`f`) — how many oscillations happen per second, in
  hertz (Hz). WiFi frequencies are in the billions of hertz — gigahertz
  (GHz).
- **Wavelength** (`λ`) — the physical distance the wave travels during
  one full oscillation. Frequency and wavelength are locked together by
  the speed of light:

  ```
  c = f · λ
  ```

  where `c ≈ 3 × 10^8 m/s` (in air/vacuum; slightly slower in denser
  media, which matters for propagation through walls but not for the
  order-of-magnitude reasoning here).

- **Phase** (`φ`) — where the wave is *within* its oscillation cycle
  right now: at a peak, at zero crossing on the way up, at a trough, etc.
  Phase is measured as an angle, either in degrees (0°–360°) or radians
  (0–2π), and it's cyclic — 370° means the same thing as 10°.

A pure sinusoidal wave at position/time `t` can be written as:

```
s(t) = A · cos(2π f t + φ)
```

`A` is the amplitude, `f` the frequency, `φ` the phase offset. This one
equation is the entire physical basis of everything that follows — WiFi
sensing, at its core, is about measuring how `A` and `φ` of a received
wave differ from what was transmitted, and inferring what must have
happened in the physical space between transmitter and receiver to cause
that difference.

## 1.2 Why WiFi lives at 2.4 GHz / 5 GHz

WiFi uses radio spectrum in the **ISM bands** (Industrial, Scientific,
and Medical) — frequency ranges that most regulators worldwide have
carved out for unlicensed use, historically set aside for things like
microwave ovens and medical diathermy equipment rather than licensed
broadcast or cellular use. 2.4 GHz (roughly 2.400–2.4835 GHz) and several
chunks of 5 GHz spectrum fall in this category, which is why any
manufacturer can sell a WiFi radio without negotiating spectrum licenses
with a regulator — the tradeoff is that you share the band with
everything else unlicensed nearby (Bluetooth, cordless phones, other
WiFi networks, microwave ovens).

Applying `c = f · λ`, solved for wavelength (`λ = c / f`):

```
2.4 GHz:  λ = (3×10^8 m/s) / (2.4×10^9 Hz) = 0.125 m  = 12.5 cm
5.0 GHz:  λ = (3×10^8 m/s) / (5.0×10^9 Hz) = 0.06 m   = 6 cm
```

This is the single most important physical fact for understanding *why*
CSI-based sensing can detect small motion. A wavelength of 12.5 cm means
that moving a reflecting object — a hand, an arm, a torso — by just a
few centimeters changes the length of the path that wave travels by a
meaningful fraction of a full wavelength. Since phase encodes "where in
the wave's cycle are we," and a full cycle corresponds to one wavelength
of path-length change, a few centimeters of motion is enough to rotate
the received phase by a significant, measurable angle. Document 3
([`03-wifi-physics-deep-dive.md`](./03-wifi-physics-deep-dive.md)) works
this out numerically. For now, the takeaway is: WiFi's wavelength is
*comparable in scale to human-body-sized motion*, which is precisely why
a phone's WiFi radio can be repurposed as a crude motion sensor at all.
If WiFi instead operated at, say, AM-radio wavelengths (hundreds of
meters), a person walking across a room would be physically undetectable
by phase — the path length change would be a negligible fraction of one
cycle.

## 1.3 Representing a wave's state as a complex number (I/Q)

Equation `s(t) = A·cos(2π f t + φ)` is a real-valued signal, but working
with amplitude and phase together as two separate real numbers is
mathematically awkward — every useful operation (combining signals,
tracking how a channel distorts a signal, doing the demodulation math)
becomes far simpler if you represent the pair `(A, φ)` as a single
**complex number**.

This is not a mathematical trick invented for convenience alone — it
reflects how RF hardware actually works. A radio receiver doesn't measure
"amplitude and phase" as two dials; it measures two real voltages
simultaneously, called **I** (in-phase) and **Q** (quadrature), obtained
by mixing the incoming signal against two reference oscillators 90° apart
in phase. **I** is the projection of the wave onto a reference cosine at
that instant; **Q** is the projection onto a reference sine (i.e., the
cosine shifted 90°, hence "quadrature"). Together, `(I, Q)` is a
Cartesian representation of exactly the same information as `(A, φ)` in
polar form:

```
        Q (imaginary axis)
        │
        │      • (I, Q) = (A cos φ, A sin φ)
        │     /
        │    /  A  (amplitude = distance from origin)
        │   /
        │  / φ  (phase = angle from I axis)
        │ /
────────┼──────────────── I (real axis)
        │
```

The complex-number form:

```
z = I + jQ = A · e^(jφ)
```

(`j` here is the engineering notation for the imaginary unit, equivalent
to mathematics' `i` — electrical engineering uses `j` because `i` is
already used for current.) Euler's identity, `e^(jφ) = cos φ + j sin φ`,
is exactly why this works: multiplying an amplitude by `e^(jφ)` produces
a complex number whose real part is `A cos φ` and imaginary part is
`A sin φ` — precisely the I and Q components. This single complex number
fully captures a wave's amplitude and phase at one instant, and it is the
natural unit of representation for a subcarrier's state in every OFDM
system, including WiFi.

**This is exactly the `(imaginary, real)` byte pair in this codebase's
CSI data.** Each subcarrier's raw CSI sample from the ESP32-S3 is a
signed 8-bit `(im, re)` pair — the hardware's I/Q measurement for that
subcarrier, in raw ADC-scaled integer units. The firmware's
`amplitude = sqrt(im² + re²)` is literally computing `|z|` (the distance
from the origin in the diagram above, i.e. `A`), and
`phase = atan2(im, re)` is computing `arg(z)` (the angle, i.e. `φ`, using
the two-argument arctangent that correctly resolves the angle across all
four quadrants — unlike a plain `atan(im/re)`, which cannot distinguish,
e.g., `(1, 1)` from `(-1, -1)`). Every subcarrier's CSI value, at its
core, is just this one complex number.

## 1.4 Modulation: putting digital bits onto a wave

A pure, unmodulated sinusoid carries no information — it's the same
predictable shape forever. To carry data, you need to systematically vary
one or more of `(A, f, φ)` in a way the receiver can decode back into
bits. This is **modulation**. There are three basic families, and it's
worth understanding all three because WiFi actually uses one of them
(QAM) that is best understood as a combination of the other two.

### Amplitude Shift Keying (ASK)

The simplest scheme: vary only the amplitude. Binary ASK (also called
OOK, on-off keying) might transmit a full-amplitude wave for a `1` bit
and nothing (or a low amplitude) for a `0` bit:

```
bit:      1     0     1     1     0
signal:  ___   ___   ___   ___
        |   | |   | |   | |   |
        |   |_|   |_|   |_|   |___
```

ASK is simple to generate and detect but wastes capacity — you're only
using one axis of information (amplitude) — and is highly vulnerable to
noise, since any amplitude fluctuation (which real channels always have,
from fading, interference, etc.) can flip a bit's interpretation.

### Phase Shift Keying (PSK)

Instead of varying amplitude, keep amplitude constant and vary phase.
**BPSK** (binary PSK) uses two phase states, typically 0° and 180°, to
encode one bit per symbol:

```
       Q
       │
  0 ●───┼───● 1        BPSK constellation:
       │            two points, 180° apart,
───────┼─────── I    same amplitude (same distance
       │            from origin), opposite sign.
       │
```

**QPSK** (quadrature PSK) uses four phase states 90° apart, encoding 2
bits per symbol (00, 01, 11, 10, one per quadrant):

```
              Q
              │
        01    │    11
           ●  │  ●
              │
──────────────┼────────────── I
              │
           ●  │  ●
        00    │    10
              │
```

Each dot here is a **constellation point** — a specific `(I, Q)` pair (equivalently, a specific amplitude+phase, though in QPSK all four points share the same amplitude and differ only in phase) that the transmitter is allowed to send, and that the receiver classifies the noisy received symbol against by finding the *nearest* point. Going from BPSK to QPSK doubles the bits carried per symbol for the same symbol rate, at the cost of packing the constellation points closer together in phase — which means less phase-error margin before a symbol gets classified into the wrong quadrant.

### Quadrature Amplitude Modulation (QAM)

QAM generalizes this by varying *both* amplitude and phase together,
placing constellation points in a grid rather than a circle. **16-QAM**
uses 16 points (4 bits per symbol):

```
   Q
   │
●  ●  │  ●  ●
   │
●  ●  │  ●  ●
───────┼──────────── I
●  ●  │  ●  ●
   │
●  ●  │  ●  ●
   │
```

Each point differs from its neighbors in both amplitude (distance from
center) and phase (angle from center), so a symbol now encodes 4 bits at
once. 64-QAM (used in higher WiFi data rates) packs 64 points into the
same space, 8 bits per symbol; 256-QAM and above go further still.

### Why higher-order QAM is more sensitive to noise

This is the central engineering tradeoff of all modulation, and it
matters for understanding WiFi's adaptive rate behavior (and, later, why
CSI amplitude/phase noise matters for sensing): **the more constellation
points you pack into the same amplitude/phase space, the closer together
they are, and the smaller a disturbance has to be before the receiver
misclassifies which point was actually sent.**

Picture the constellation diagrams above as a grid of targets and the
real received symbol as an arrow thrown at that grid with some random
scatter (noise) around the intended point. With only 2 or 4 possible
targets (BPSK/QPSK), spaced far apart, a lot of scatter is tolerable
before the arrow lands closer to the wrong target than the right one.
With 64 or 256 targets packed into the same space, the "safety margin"
around each point shrinks dramatically — the same amount of physical
noise that BPSK shrugs off can push a 256-QAM symbol into the wrong
decision region, corrupting more bits. This is exactly why WiFi (and
essentially every modern radio standard) uses **adaptive modulation and
coding**: a router picks a lower-order scheme like BPSK or QPSK when the
link is noisy or far away (fewer bits per symbol, but very tolerant of
noise) and a higher-order scheme like 256-QAM when the link is clean and
strong (many bits per symbol, but only usable when noise is low). This
is what "MCS index" (modulation and coding scheme) refers to in WiFi rate
tables, and it's visible in `wifi_csi_info_t.rx_ctrl`'s rate/MCS fields
in this codebase's captures, even though the current firmware doesn't
log it.

## 1.5 One subcarrier

Everything in this document — amplitude, phase, I/Q, modulation — applies
to **one** carrier wave at one frequency. A real WiFi channel does not
transmit data on a single carrier this way; it splits the channel into
dozens of narrow, simultaneous carriers called **subcarriers**, each
independently modulated (potentially with a different constellation size
depending on link conditions), and combines them using a technique called
OFDM. That's the subject of the next document,
[`02-ofdm-and-80211-phy.md`](./02-ofdm-and-80211-phy.md). The reason this
matters for CSI: the "per-subcarrier" structure of the data this codebase
receives — `amp0, phase0, amp1, phase1, ...` — is literally one complex
number, of the exact `A·e^(jφ)` form derived in §1.3, per subcarrier,
per packet.
