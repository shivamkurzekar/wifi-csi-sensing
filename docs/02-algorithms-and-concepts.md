# csi_presence — algorithms and concepts, in depth

This document explains, with the underlying math, *why* every piece of
`main/csi_presence.c`'s presence-detection pipeline works — not just what
it does. `docs/v1.md` is the project overview (what the code does, build
instructions, current status); this is its deeper, theory-focused
companion, written for an engineer who knows C and basic statistics but
has never touched WiFi CSI or a streaming/online algorithm before. It
intentionally doesn't restate v1.md's narrative in full — read that first
if you haven't.

Two companion documents are referenced throughout rather than repeated
here:

- [`csi_test`'s `docs/02-algorithms-and-concepts.md`](https://github.com/shivamkurzekar/csi_test/blob/main/docs/02-algorithms-and-concepts.md)
  — the equivalent deep-dive for `csi_test`'s *spatial* variability metric,
  the sibling approach this project deliberately does not use. (A separate
  repo, not a local path -- linked directly on GitHub.)
- [`csi-learning/03-wifi-physics-deep-dive.md`](csi-learning/03-wifi-physics-deep-dive.md)
  — the physics of why a WiFi signal's *phase* is sensitive to motion at
  all. This document assumes that physical picture and does not re-derive
  it.

All code excerpts below are verified against the current
`main/csi_presence.c` as of commit `ce24e7e` (the bug-fix commit discussed
in depth in §7). Constant names and values are quoted exactly; if this
document and the source ever disagree, the source is correct.

---

## 1. The core design idea, and why it differs from `csi_test`

`csi_test` answers "what does CSI look like" by computing, per packet, how
much amplitude varies **across the subcarriers within that one packet** —
a spatial snapshot of the channel at a single instant. That number can be
large in a static, RF-complex room (lots of furniture, lots of multipath)
even with nobody moving, because it never looks at how the channel changes
between packets.

`csi_presence` instead asks a narrower, time-domain question: **is the
channel changing over time?** It tracks one subcarrier's unwrapped phase
across many consecutive packets and computes the variance of that
phase *over the last ~1 second* — a quantity the code calls "motion
energy." A room can be as RF-complex as it likes; if nothing is moving,
each new packet's channel estimate for that subcarrier looks almost
identical to the last one's, and the time-domain variance stays low.
Motion — a body entering multipath, blocking or unblocking a reflection
path — measurably rotates that phase from packet to packet, and the
variance rises.

The physical reason phase is sensitive enough to *use* this way — a
2.4 GHz wave's wavelength is about 12.5 cm, so even a few centimeters of
motion produces a significant fraction of a full phase rotation — is
covered in
[`03-wifi-physics-deep-dive.md`](csi-learning/03-wifi-physics-deep-dive.md).
Take it as given here: phase-over-time is a legitimate, sensitive motion
signal, not a hack.

Everything below is about the five mechanisms that turn "one subcarrier's
phase, over time" into a debounced yes/no presence flag: phase unwrapping,
Welford's variance algorithm, a once-per-session subcarrier pick, a
sliding-window variance ("motion energy"), an adaptive calibration
threshold, and a hysteresis/debounce decision. Sections 4, 5, and 6 also
each end with a hardware bug that hit exactly that mechanism, so the
theory and the bug are presented together rather than as a separate list.

---

## 2. Phase unwrapping

### The problem

Each subcarrier's raw CSI sample is a complex number, stored as an
`(imaginary, real)` byte pair. The code recovers phase with:

```c
float im = (float)record->buf[idx];
float re = (float)record->buf[idx + 1];
float raw_phase = atan2f(im, re);
```

`atan2f` returns a value in `(-π, π]`. That's a fundamental property of
the arctangent, not a bug: a phase angle is only defined up to a multiple
of `2π` (rotating a vector by `2π` gives back the same vector), so any
single-valued angle function has to pick one representative value per
full turn, and `atan2` picks the one in `(-π, π]`.

The problem this creates: if the *true, physical* phase drifts smoothly
past `+π` (say from `+3.0` to `+3.3` radians) because of motion, `atan2f`
reports it as jumping from `+3.0` to `-2.98` — a value that *looks* like a
huge, sudden change (`-2.98 - 3.0 = -5.98` rad) when the real underlying
change was a modest `+0.3` rad. Every downstream computation that treats
"change in phase" as a motion signal (§5's motion energy is exactly this)
would see that wraparound as a fake, enormous spike, indistinguishable
from — or worse than — real motion.

### The unwrap algorithm

The standard fix, phase unwrapping, tracks a *continuous* phase value
frame to frame instead of trusting each frame's raw `atan2` output on its
own. The rule: compute the raw frame-to-frame delta, and if that delta is
outside `(-π, π]` — i.e., larger than a half-turn in either direction —
assume a wraparound occurred and add or subtract a full `2π` turn until it
isn't. A half-turn is the right cutoff because it's the largest change
that can occur between two truly adjacent samples of a continuously
rotating phase without ambiguity: anything closer to `π` than to `0` is
more plausibly explained as "the phase actually only moved a little, and
wrapped," than "it moved almost all the way around."

The implementation:

```c
static inline float unwrap_phase(float prev, float curr)
{
    float diff = curr - prev;
    while (diff > (float)M_PI)  diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return prev + diff;
}
```

Note that this returns `prev + diff`, not `diff` alone: the function
maintains a running *cumulative* unwrapped phase, not a bounded angle.
Over a long session this value is not confined to `(-π, π]` at all — it
can drift to arbitrarily large magnitudes if there's any small consistent
per-frame bias. That's expected and correct for unwrapping; it's worth
keeping in mind in §5, where this value feeds a variance computation.

### Worked example

Take a true continuous phase trajectory of `3.00 → 3.30` rad (a `+0.30`
rad change from one packet to the next — plausible for a small motion
event). `atan2f` cannot report `3.30` directly, since that's outside
`(-π, π]`; it reports the equivalent angle inside that range,
`3.30 - 2π = -2.98319`. So the raw samples the code actually sees are
`prev = 3.00`, `curr = -2.98319`.

Running `unwrap_phase(3.00, -2.98319)`:

```
diff = curr - prev = -2.98319 - 3.00 = -5.98319
-5.98319 < -π (-3.14159)?  yes -> diff += 2π = -5.98319 + 6.28319 = 0.30000
0.30000 in (-π, π]? yes -> stop
return prev + diff = 3.00 + 0.30000 = 3.30000
```

The unwrapped result, `3.30000`, exactly recovers the true continuous
phase — the `+0.30` rad motion-caused change survives unwrapping, while
the naive raw delta (`-5.98` rad) would have looked like an enormous,
implausible jump.

### Per-subcarrier state, and the subcarrier-count edge case

Unwrapping needs a `prev` value *per subcarrier*, since each subcarrier's
phase evolves independently:

```c
static float s_prev_phase[MAX_SUBCARRIERS];
static bool  s_have_prev_phase[MAX_SUBCARRIERS];
static int   s_prev_num_subcarriers = -1;
```

(`MAX_SUBCARRIERS` is `CSI_RECORD_MAX_LEN / 2 = 256` — half the 512-byte
record buffer, since each subcarrier is a 2-byte imaginary/real pair.)

Different 802.11 frame types (e.g. non-HT vs. HT) carry different numbers
of subcarriers in their CSI report, so the packet's subcarrier *count*
can change from one frame to the next (see `csi_test/docs/v1.md` §6 for
the underlying cause). When that happens, index `i` in the new frame does
not refer to the same physical subcarrier as index `i` in the previous
frame — the array is aligned by position, not by any stable frequency
identity. Unwrapping "across" such a change would be comparing two
unrelated quantities and computing a meaningless delta. The code detects
this and drops unwrap continuity for that one frame instead:

```c
bool shape_changed = (num_subcarriers != s_prev_num_subcarriers);
if (shape_changed) {
    memset(s_have_prev_phase, 0, sizeof(s_have_prev_phase));
    s_prev_num_subcarriers = num_subcarriers;
}
```

With `s_have_prev_phase[i]` cleared, the very next frame's phase for every
subcarrier is taken as-is (`raw_phase`, no unwrap), and unwrap continuity
resumes from there. The acknowledged cost, noted directly in the source
comment, is a one-frame gap in phase history whenever packet type
switches — an accepted, documented limitation rather than a bug.

---

## 3. Welford's online variance algorithm

This section is general numerical-methods background — nothing here is
specific to CSI or radio.

### Why not the obvious approaches

**Two-pass:** compute the mean of all `N` samples, then loop again
computing `Σ(x_i − mean)²`. Correct and numerically well-behaved, but it
requires storing all `N` samples (or a second read of the source) before
you can report anything. On a microcontroller ingesting an unbounded
stream of CSI packets for an entire session, "store everything" isn't an
option — there's no natural end to the stream, and no RAM budget for it.

**Single-pass sum-of-squares:** track `Σx_i` and `Σx_i²` as you go, then
compute variance as `E[x²] − (E[x])²` at the end. This *is* single-pass
and needs no history — but it's numerically unstable when the mean is
large relative to the spread of the data, because `Σx_i²` and `(Σx_i)²/N`
become two large, nearly-equal numbers whose difference is a much smaller
number: subtracting them loses precision to floating-point cancellation,
and can even produce a *negative* variance from rounding error.

### Welford's recurrence

Welford's algorithm updates a running mean and a running sum of squared
deviations *from the running mean*, one sample at a time, using only
`O(1)` state regardless of how many samples have been seen — no history
buffer, and no cancellation problem, because it never differences two
independently large accumulators. The recurrence, for sample `x` arriving
as the `n`-th observation:

```
delta   = x - mean
mean    = mean + delta / n
delta2  = x - mean          (uses the *updated* mean)
M2      = M2 + delta * delta2
```

After `n` samples, `M2` is the running sum of squared deviations from the
mean, and the (sample) variance is `M2 / (n − 1)`.

This is exactly what the code does:

```c
static void welford_update(int idx, double x)
{
    s_sc_count[idx]++;
    double delta = x - s_sc_mean[idx];
    s_sc_mean[idx] += delta / (double)s_sc_count[idx];
    double delta2 = x - s_sc_mean[idx];
    s_sc_m2[idx] += delta * delta2;
}

static double welford_variance(int idx)
{
    if (s_sc_count[idx] < 2) return 0.0;
    return s_sc_m2[idx] / (double)(s_sc_count[idx] - 1);
}
```

(The `count < 2` guard exists because sample variance is undefined —
division by zero — for a single observation; the code treats that case as
`0.0` rather than crashing or returning garbage.)

### Worked example

Feed the sequence `1.0, 2.0, 3.0, 4.0` through `welford_update`:

| n | x | delta = x − mean | mean += delta/n | delta2 = x − mean | M2 += delta·delta2 |
|---|-----|-------------------|-----------------|--------------------|---------------------|
| 1 | 1.0 | 1.0 − 0.0 = 1.0   | 0.0 + 1.0/1 = **1.0** | 1.0 − 1.0 = 0.0 | 0.0 + 1.0·0.0 = **0.0** |
| 2 | 2.0 | 2.0 − 1.0 = 1.0   | 1.0 + 1.0/2 = **1.5** | 2.0 − 1.5 = 0.5 | 0.0 + 1.0·0.5 = **0.5** |
| 3 | 3.0 | 3.0 − 1.5 = 1.5   | 1.5 + 1.5/3 = **2.0** | 3.0 − 2.0 = 1.0 | 0.5 + 1.5·1.0 = **2.0** |
| 4 | 4.0 | 4.0 − 2.0 = 2.0   | 2.0 + 2.0/4 = **2.5** | 4.0 − 2.5 = 1.5 | 2.0 + 2.0·1.5 = **5.0** |

Final variance = `M2 / (n − 1) = 5.0 / 3 = 1.6667`.

Check against the direct definition: mean is `2.5`;
`Σ(x − mean)² = 2.25 + 0.25 + 0.25 + 2.25 = 5.0`; sample variance
`= 5.0 / 3 = 1.6667`. Matches exactly — Welford's incremental formula and
the textbook two-pass formula agree, as they must algebraically; Welford
just never needs the samples again after consuming them.

### Why it's the right tool here

`csi_presence` needs, during a short warmup, the variance of *every*
subcarrier's phase simultaneously — up to 256 independent running
variances, updated once per packet, with no way to know in advance how
many packets the warmup will take (packet arrival on this network is
irregular; see §6). Welford is exactly the tool for "streaming,
unbounded-length, many-parallel-accumulators, embedded-memory-budget"
variance — this is the textbook use case it exists for.

One implementation detail worth being precise about: `welford_update` is
called **unconditionally, every frame, for every subcarrier** — not just
during the warmup window:

```c
for (int i = 0; i < num_subcarriers; i++) {
    ...
    welford_update(i, (double)unwrapped);
}
```

The accumulators keep growing for the rest of the session even after the
one-time subcarrier pick (§4) has already happened and consumed them.
That's harmless (a few extra additions per frame) but also pointless
after the pick — the values are never read again. Not a bug, just dead
computation worth knowing about if you're looking for where CPU time
goes.

---

## 4. Subcarrier selection: picking the noisiest one, and why that's subtler than it looks

### The idea

Not every subcarrier is equally useful. Some sit at the edge of the
channel with near-zero amplitude (an OFDM/RF property, not a bug — see
`csi_test/docs/v1.md`); their phase is dominated by noise and carries
little information about the room. Others sit in the middle of strong
multipath and respond clearly to motion. The heuristic: **whichever
subcarrier's phase has the highest variance is probably the most
sensitive to real channel change**, so track only that one.

The pick uses the Welford accumulators from §3, taken over a short
warmup:

```c
if (!s_did_first_reeval) {
    s_frames_since_reeval++;
    if (s_frames_since_reeval >= TOPK_WARMUP_FRAMES) {   /* = 20 */
        double best_var = -1.0;
        int best_idx = 0;
        for (int i = 0; i < num_subcarriers; i++) {
            double v = welford_variance(i);
            if (v > best_var) { best_var = v; best_idx = i; }
        }
        s_primary_subcarrier = best_idx;
        s_did_first_reeval = true;
        ...
    }
}
```

`s_did_first_reeval` latches `true` the first time this runs, and the
pick is never repeated for the rest of the session — deliberately, as the
next two subsections explain.

### Bug 1: waiting too long for the first pick

The warmup length, `TOPK_WARMUP_FRAMES = 20`, exists precisely because the
*original* version of this code waited for a full periodic-reselection
window — 100 frames — before making its very first pick, leaving
`s_primary_subcarrier` at its compile-time default of index `0` the
entire time. On this network's real, sparse, irregular packet arrival
rate (gaps observed from 0.2 s to over 300 s — see §6 and §9), that
default persisted for well over a minute in testing. Index 0 turned out
to be a near-dead edge subcarrier with essentially zero amplitude, so its
phase was numerically almost pure noise around a fixed point — variance
close to zero regardless of what happened in the room. Motion energy
(§5) computed from it read a flat `0.0` the whole time. The system could
never have detected motion during that window, no matter what a person
in the room did.

The fix is the short, warmup-only `TOPK_WARMUP_FRAMES` pick shown above —
fast enough that the default index-0 fallback is only ever active for a
brief, clearly-labeled startup period, not indefinitely.

### Bug 2: periodic reselection silently corrupting calibration

The *original* design didn't just default to index 0 at startup — it also
re-ran the noisiest-subcarrier pick every 100 frames afterward, on the
reasoning that conditions might change and the "best" subcarrier today
might not be the best an hour from now. That's a reasonable-sounding
idea in isolation. It broke something else entirely: the adaptive
calibration accumulator (§6) sums `motion_energy` samples into a single
running mean and variance for the whole calibration window, with **no
record of which subcarrier produced each sample**. If the tracked
subcarrier changes mid-calibration, the accumulator silently blends two
subcarriers' unrelated noise characteristics (different amplitude, different
multipath sensitivity, different baseline variance) into one number that
doesn't correspond to any single physical thing. A "calibrated ambient
baseline" computed that way isn't a baseline for anything in particular.

The fix, already shown above, is to pick once (`s_did_first_reeval`
latches permanently after the first pick) and keep that subcarrier for
the rest of the session.

### The general lesson

Bugs 1 and 2 are two symptoms of the same root cause: **the calibration
accumulator and the subcarrier-selection logic are two separate stateful
subsystems with an implicit, undeclared dependency between them** — the
accumulator's numbers are only meaningful if the subcarrier feeding it
stays fixed, but nothing in the code enforced that invariant. Nothing
would have flagged this at compile time or even in a careful code review
of either subsystem *individually*: the selection logic looks reasonable
on its own ("periodically re-pick the best subcarrier"), and the
calibration logic looks reasonable on its own ("accumulate mean and
variance of the incoming score"). The bug only exists in the interaction.

This is a broadly applicable lesson for stateful embedded systems: when
one accumulator's meaning depends on an upstream source staying stable
(here: "which subcarrier"), that dependency needs to be either
structurally enforced (e.g., stamp each accumulated sample with an
identity, or reset the accumulator whenever the source changes and treat
that as a state transition — see §5's Bug 3 for exactly this pattern
applied elsewhere in this same file) or eliminated (here: don't let the
source change at all, mid-session). A comment saying "don't change this
without also touching that" is not enforcement; it's a bet that whoever
edits the code later reads and honors it.

As a directly-verifiable aside: a structurally similar latent case still
exists elsewhere in the current code, though it has not been observed to
cause a problem and is not one of the three fixed bugs. `shape_changed`
handling (§2) resets `s_have_prev_phase` when the subcarrier count
changes, but it does **not** reset `s_sc_mean` / `s_sc_m2` / `s_sc_count`
— so if a packet-shape change happens to occur during the warmup window
itself, the per-index Welford accumulators would keep mixing samples from
what may be physically different subcarriers before and after the shape
change, for the same reason described above. It's flagged here only as an
example of the same pattern, not as a claim that it has caused observed
misbehavior.

---

## 5. Time-domain motion energy: the sliding-window variance

### The computation

Once a primary subcarrier is locked in, its unwrapped phase is pushed
into a small ring buffer every frame:

```c
#define PHASE_HISTORY_LEN      32    /* ring buffer capacity */
#define MOTION_WINDOW           20    /* variance window, in samples */

static float s_phase_history[PHASE_HISTORY_LEN];
static uint16_t s_history_idx = 0;
static uint16_t s_history_len = 0;
```

"Motion energy" is the variance of the most recent `MOTION_WINDOW` (20)
values in that buffer — computed fresh, from scratch, every frame, once
at least 10 samples are available:

```c
if (s_history_len >= 10) {
    uint16_t window = (s_history_len < MOTION_WINDOW) ? s_history_len : MOTION_WINDOW;
    double sum = 0.0, sum2 = 0.0;
    for (uint16_t i = 0; i < window; i++) {
        uint16_t ri = (s_history_idx + PHASE_HISTORY_LEN - window + i) % PHASE_HISTORY_LEN;
        double v = s_phase_history[ri];
        sum += v;
        sum2 += v * v;
    }
    double mean = sum / (double)window;
    double var = (sum2 / (double)window) - (mean * mean);
    s_motion_energy = (var > 0.0) ? (float)var : 0.0f;
}
```

Two things are worth calling out precisely, because they differ from §3:

- This is the **naive single-pass sum-of-squares** formula
  (`E[x²] − (E[x])²`), *not* Welford, and it uses a **population**
  divisor (`/ window`), not the sample divisor (`/ (window − 1)`)
  `welford_variance` uses. That's a real, if minor, inconsistency in
  which variance estimator is used where in this codebase — it doesn't
  meaningfully change behavior at `window ≈ 20`, but it's not the same
  formula.
- The naive formula's cancellation risk (§3) is not a practical concern
  here specifically because the window is small (≤20 values) and
  recomputed from a *local* buffer, so even though unwrapped phase can
  drift to a large cumulative magnitude over a long session (§2), all 20
  values in any one window are close to each other in magnitude — the
  drift is common to all of them and mostly cancels harmlessly in the
  `mean` term, rather than in a precision-losing subtraction of two large,
  nearly-equal numbers.

The buffer is deliberately sized larger than the window
(`PHASE_HISTORY_LEN = 32` vs. `MOTION_WINDOW = 20`) to leave headroom; the
window read walks backward from the current write position modulo the
buffer size.

### Why ~20 samples

The window size is a direct responsiveness-vs-noise-averaging tradeoff,
the same tradeoff behind every moving-average or moving-variance filter.
Too short a window (say, 3–5 samples) makes the variance estimate itself
noisy — a single stray packet (multipath fading, a nearby unrelated
device, a momentary RF glitch) can dominate a tiny window and produce a
spurious spike that looks like motion. Too long a window (say, 200
samples) smooths real, brief motion events into insignificance and adds
lag between an actual physical event and the score reacting to it. Twenty
samples is presented in the source comments as roughly "a ~1 second
window at typical packet rates" — a reasonable middle ground for a
walking-speed motion event, which typically perturbs the channel for at
least a few tenths of a second.

That framing deserves one honest caveat: the window is a **sample count**,
not a **time span**. `csi_test/docs/v1.md`'s capture analysis found
inter-packet gaps on this network ranging from roughly 0.2 s to several
seconds (and, per §9 below, occasionally much longer). At the sparse end
of that range, "the last 20 samples" can span many seconds of wall-clock
time rather than one — the window's *meaning* stretches and compresses
with traffic rate, unlike the calibration window (§6), which was
deliberately made time-based for exactly this reason. This is a live
architectural inconsistency worth the reader's attention, not something
this document is asserting is broken; it simply hasn't been addressed the
way the calibration window was.

### Bug 3: stale ring-buffer state at the moment of lock-in

Before the primary subcarrier is picked (§4), the ring buffer doesn't
exist in any meaningful sense yet, but nothing prevented frames from
writing into it during warmup using whatever `s_primary_subcarrier`
happened to be (the default, index 0). The instant the real pick happens
and `s_primary_subcarrier` changes to some other index, the *next* frame's
variance window is computed over a buffer that's a mix of old
subcarrier-0 phase values and new phase values from the just-picked
subcarrier. Those two phase sequences are unrelated — different
subcarriers, different physical channel response — so the transition
between them looks, to the variance computation, like an enormous,
instantaneous discontinuity: a fake motion spike that has nothing to do
with anything happening in the room.

In testing, that spike occurred during the calibration window (§6) and
got baked directly into the ambient mean/stddev the calibration was
computing, inflating one measured threshold to roughly 4x the real
steady-state range (65, versus an observed actual range of 4–18, per the
`ce24e7e` commit message).

The fix clears the ring buffer at the exact moment the regime changes:

```c
s_primary_subcarrier = best_idx;
s_did_first_reeval = true;

/* ... clear it so the first post-lock reading is real. */
s_history_idx = 0;
s_history_len = 0;
```

A related, second instance of the same class of bug was fixed alongside
it. `s_presence_detected` could latch `true` during the crude
pre-calibration period (which compares motion energy against
`PRESENCE_DEFAULT_THRESH = 0.05`, a placeholder value that trips easily),
and then take up to `PRESENCE_CLEAR_FRAMES` (5) frames to debounce back
off once the real, calibrated threshold took over — a startup artifact
with no relationship to anything real happening in the room. The fix
resets the flag at the moment calibration actually completes:

```c
s_calibrated = true;
/* Start clean against the real threshold instead of forcing the
 * debounce to work it off. */
s_presence_detected = false;
s_below_count = 0;
```

### The general lesson

This file has two distinct operating regimes with a hard transition
between them: "pre-lock" → "post-lock" (§4), and "pre-calibration" →
"calibrated" (§6). Both transitions had exactly the same shape of bug: a
piece of accumulated state (a ring buffer; a latched boolean) that was
correct *for the old regime* was allowed to survive, unexamined, into the
new regime, where its old values were silently treated as if they applied
to the new one. Any system with discrete mode changes — a config reload
invalidating a cache, a sensor recalibration event, a reconnect resetting
a session — has the same shape of risk: **the default behavior of state
that persists across a regime change is "keep it," and that default is
wrong whenever the state's meaning depended on the regime it was collected
under.** The general fix pattern is also the same in both cases here:
identify the exact moment the regime changes, and explicitly decide what
happens to every piece of carried-over state at that moment, rather than
letting it fall through unexamined.

---

## 6. Adaptive calibration: `mean + k·σ` as an anomaly threshold

### The technique, in general

`threshold = ambient_mean + k × ambient_stddev` is a standard technique
for turning "how does this compare to normal" into a single number,
sometimes called a control limit or a one-sided Z-score threshold. It's
the same idea behind Shewhart control charts in statistical process
control, where a manufacturing process's measurements are watched over
time and a point is flagged "out of control" if it falls outside
`mean ± 3σ` of the process's own historical behavior. The parallel here is
close: the code watches its own "motion energy" signal during a period
presumed to be ambient/quiet, learns what "normal" looks like for *this
specific deployment* (this network, this room, this hardware), and then
flags anything that exceeds normal by a wide-enough margin as
"presence."

### The implementation

Calibration is **time-based**, not frame-count-based — a deliberate
departure from the RuView project this approach was adapted from, which
assumes a steady ~20 Hz capture rate and calibrates over a fixed 1,200
frames. This project's own earlier packet-rate capture found the actual
network's inter-packet gaps ranging from roughly 0.2 s to multiple
seconds (see §9 for an even more extreme observed gap), so a frame-count
window would take an unpredictable amount of wall-clock time — possibly
finishing on a burst of only a few seconds' worth of samples. Watching
the ESP32's own clock instead keeps the calibration window's *meaning* (a
fixed span of ambient observation) stable regardless of how bursty
traffic is:

```c
#define CALIB_SECONDS           60.0
#define CALIB_MIN_SAMPLES       10

if (s_first_frame_us < 0) s_first_frame_us = record->timestamp_us;
double elapsed_s = (record->timestamp_us - s_first_frame_us) / 1e6;

if (!s_calibrated) {
    if (elapsed_s < CALIB_SECONDS) {
        s_calib_sum += s_motion_energy;
        s_calib_sum_sq += (double)s_motion_energy * s_motion_energy;
        s_calib_count++;
    } else if (s_calib_count >= CALIB_MIN_SAMPLES) {
        double mean = s_calib_sum / (double)s_calib_count;
        double var = (s_calib_sum_sq / (double)s_calib_count) - (mean * mean);
        double sigma = (var > 0.0) ? sqrt(var) : 0.001;
        s_adaptive_threshold = (float)(mean + PRESENCE_SIGMA_MULT * sigma);
        if (s_adaptive_threshold < 0.001f) s_adaptive_threshold = 0.001f;
        s_calibrated = true;
        ...
    }
    /* else: window elapsed but too few samples arrived; keep waiting
     * rather than committing to a threshold estimated from noise. */
}
```

`s_first_frame_us` starts the clock at the moment of subcarrier lock-in
(§4/§5), not at device boot — the same "don't let pre-regime state
pollute the new regime" reasoning as Bug 3. `CALIB_MIN_SAMPLES` (10)
guards against committing to a threshold computed from a statistically
meaningless handful of points if the calibration window elapses on a
particularly sparse-traffic run; the code simply keeps accumulating past
`CALIB_SECONDS` in that case instead of calibrating on noise.

### Why `k = 3`, and why that's an open tuning question here

`PRESENCE_SIGMA_MULT = 3.0f`. For a normally-distributed quantity,
approximately 99.7% of samples fall within `mean ± 3σ`, so `k = 3` is a
conservative, widely-used default when the cost of a false positive is
high relative to the cost of a slower true positive: it should trip on
ambient noise only rarely.

There's a subtlety worth naming precisely, though. That 99.7% figure is a
property of the *normal* distribution specifically, and "motion energy"
is not obviously normally distributed. It's a variance estimate — a sum
of squared deviations — which is by construction non-negative and
typically right-skewed (values pile up near a lower bound and have a long
tail toward large values), closer in shape to a chi-squared distribution
than a Gaussian. For a right-skewed distribution, the Gaussian intuition
"only 0.3% of mass lies beyond 3σ" doesn't transfer cleanly: a
heavier-than-Gaussian right tail can put meaningfully more than 0.3% of
probability mass out past `mean + 3σ`, and — more relevant to what was
actually observed — a noisy or skewed ambient sample can inflate the
estimated `σ` itself, pushing `mean + 3σ` higher than a truly
representative "just past normal" bound would sit.

That is a plausible *contributing explanation*, not a confirmed root
cause, for what live testing actually found (§9): a calibrated threshold
of `44.98` sitting well above every value of motion energy observed
during an extended live test, including moments of real ambient
disturbance up to `32.86` — a real signal, clearly larger than the
steady-state baseline, that still never came close to crossing the
threshold. Whether that reflects `k = 3` genuinely being tuned too
conservatively for this specific deployment, or whether it correctly
rejected disturbances that were real but still smaller than an actual
person's presence would produce, is exactly the open question — a
textbook false-positive/false-negative tradeoff, not yet resolved by
data. Lowering `k`, or applying a variance-stabilizing transform (e.g. a
square root or log) to motion energy before thresholding to reduce its
skew, are both plausible next steps; neither has been tried yet.

---

## 7. Hysteresis and debounce (the Schmitt trigger)

### The problem it solves

A raw threshold comparison (`score > threshold ? present : not_present`)
flickers whenever the score dithers near the boundary — normal behavior
for any noisy real-world signal that happens to hover close to a single
cutoff. Consider a score wobbling around a threshold `T = 10`:

```
frame:   1    2    3    4    5    6    7    8    9   10   11   12
score:  12    9   11    8    9   12    9    4    4    4    4    4
```

A single-threshold comparison against `T = 10` produces:

```
frame:      1     2     3     4     5     6     7     8     9    10    11    12
present:  true false  true false false  true false false false false false false
```

That's five state changes in twelve frames from a score that never
strayed far from `T` — exactly the "flicker" this section exists to
prevent. A downstream consumer of this flag (a light switch, a log, an
alert) would be firing constantly.

### The fix: two thresholds plus a debounce counter

A Schmitt trigger uses two thresholds instead of one: an upper "enter"
threshold and a lower "clear" threshold, with the clear threshold set
below the enter threshold so the signal has to drop further than it took
to rise before the flag turns back off. This project adds a debounce
counter on top: even dropping below the clear threshold isn't enough by
itself — it has to stay there for several consecutive frames.

```c
#define PRESENCE_HYST_RATIO     0.5f   /* clear-threshold = enter-threshold * this */
#define PRESENCE_CLEAR_FRAMES   5      /* consecutive low frames before clearing */

static bool presence_flag_update(bool prev, float score, float threshold, uint8_t *below_count)
{
    float low_thresh = threshold * PRESENCE_HYST_RATIO;

    if (score > threshold) {
        *below_count = 0;
        return true;                      /* enter immediately */
    }
    if (score >= low_thresh) {
        *below_count = 0;
        return prev;                      /* dead zone: hold whatever it was */
    }
    if (*below_count < 0xFF) (*below_count)++;
    if (!prev) return false;
    if (*below_count >= PRESENCE_CLEAR_FRAMES) {
        *below_count = 0;
        return false;                     /* clear, after enough consecutive low frames */
    }
    return true;                          /* still counting down */
}
```

Three zones, not two: above `threshold` (enter immediately — presence
should never be *slow* to notice real motion); between `low_thresh` and
`threshold` (a dead zone that just holds the previous state, neither
confirming nor denying); and below `low_thresh` (only here does the
debounce counter advance, and only after `PRESENCE_CLEAR_FRAMES`
consecutive frames down there does the flag actually clear).

### Worked trace

Same score sequence as above, `T = 10`, so `low_thresh = 5`,
`PRESENCE_CLEAR_FRAMES = 5`:

| frame | score | zone | action | `present` | `below_count` |
|-------|-------|------|--------|-----------|----------------|
| 1 | 12 | > T | enter | **true** | 0 |
| 2 | 9  | dead zone (≥5) | hold | true | 0 |
| 3 | 11 | > T | enter | true | 0 |
| 4 | 8  | dead zone | hold | true | 0 |
| 5 | 9  | dead zone | hold | true | 0 |
| 6 | 12 | > T | enter | true | 0 |
| 7 | 9  | dead zone | hold | true | 0 |
| 8 | 4  | < low | count | true | 1 |
| 9 | 4  | < low | count | true | 2 |
| 10 | 4 | < low | count | true | 3 |
| 11 | 4 | < low | count | true | 4 |
| 12 | 4 | < low | **clear** | **false** | 0 |

The flag stays `true` continuously through all the dithering near `T`
(frames 2–7), and only clears at frame 12, once the signal has genuinely
dropped below `low_thresh` and stayed there for five consecutive frames.
One clean state change instead of five spurious ones, from the same input
data.

---

## 8. The three hardware bugs, as one narrative

Sections 4–6 each covered one of these in the context of the mechanism it
broke. Put together, they tell a single story worth stating explicitly,
because it's the most transferable lesson in this codebase:

1. **Stuck on a dead subcarrier** (§4) — invisible from reading the code,
   because "default to index 0 until the first re-eval window" reads as
   perfectly reasonable in isolation. It only became visible as a
   symptom on real hardware: a flat `0.0` motion-energy reading, for over
   a minute, on a live serial monitor.
2. **Periodic reselection corrupting calibration** (§4) — invisible from
   reading either subsystem (selection, calibration) on its own; each
   was individually correct. It only became visible by watching a
   calibrated threshold come out roughly 4x too high compared to the
   steady-state motion-energy values the same live monitor was showing
   moments earlier.
3. **Stale ring-buffer / latched-flag state at a regime transition**
   (§5) — same shape of problem, invisible in isolation, visible only as
   a specific numeric artifact (a threshold inflated to `65` against an
   actual steady-state range of `4–18`) traceable back to one bad reading
   at one specific moment, only findable by correlating serial output
   against wall-clock events (when, exactly, did lock-in happen).

None of these three were compile errors. None were the kind of thing a
reviewer reading `csi_presence.c` top to bottom — even carefully — would
be likely to catch, because each individual piece of logic was locally
sensible; the bug lived in the *interaction* between pieces of state
across time, which is exactly the kind of thing static reading struggles
to simulate. All three were found the same way: flash real firmware onto
real hardware, on a real (sparse, irregular, imperfect) network, and read
the actual serial output against actual wall-clock time.

The general point for the reader: **a clean build, and even a careful
code review, are not validation.** They rule out a different, narrower
class of problems (syntax errors, obviously wrong logic, type mismatches,
violated local invariants). Runtime behavior on real hardware, against
real data with all its irregularity — the packet that arrives 300 seconds
late, the subcarrier that's silently dead, the exact frame at which a
state machine transitions — is a categorically different and necessary
kind of testing, and for a system like this one, whose correctness
depends on temporal state built up over minutes of live, unpredictable
input, no amount of code reading is a substitute for it.

---

## 9. Honest status

Pulling together `docs/v1.md`, the `ce24e7e` commit, and the fullest live
capture available in this repo (`tools/live_presence.log`, an untracked
local test log of 336 consecutive `PRESENCE` readings from one boot
session, produced by `tools/live_log.py`):

**Confirmed.** The system locks onto a real (non-dead) subcarrier within
about 20 frames, as designed. It calibrated to a threshold of `44.97852`.
Across the entire captured session — spanning host-observed wall-clock
time of roughly eleven minutes, including one gap in packet arrival of
over 300 seconds, which is itself a useful data point for how irregular
this network's traffic really is — motion energy ranged from `2.14` to
`32.86`, and the `present` flag read `0` on every single one of the 336
logged frames. The "stays quiet with nobody there" half of the system's
job is confirmed by real data, not just by design intent.

**Not yet confirmed.** No capture in this repo yet includes a controlled
trial with a person actually present and moving near the device while
logging ran. The "correctly flips to `1` when someone is really there"
half of the claim — arguably the more important half — has not been
tested. `docs/v1.md` is explicit about this gap and this document doesn't
overstate it.

**Suspected, unresolved.** The peak motion energy observed so far,
`32.86`, is a real signal — clearly elevated above the steady-state
baseline (roughly `3`–`14` for most of the session) — and it still sits
comfortably below the calibrated threshold of `44.98`. That gap is either
evidence the `k = 3` sigma multiplier (§6) is tuned too conservatively for
this specific deployment (this room, this hardware, this network), or
evidence the threshold is correctly rejecting disturbances that are real
but still smaller than what an actual person would produce. Distinguishing
those two explanations requires the still-missing occupied-room trial
above; until that data exists, this is a genuine open engineering
question, not a solved one, and should be treated as such by anyone
picking this codebase up next.
