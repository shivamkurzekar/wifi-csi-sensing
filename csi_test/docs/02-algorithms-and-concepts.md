# csi_test — Algorithms and Concepts (deep reference)

This is a companion to [`v1.md`](./v1.md), which explains *what the project
does and how to run it*. This document explains *how and why* each piece
works, at the level of detail you'd want before modifying the firmware or
the analysis pipeline yourself. It assumes you know C and Python but have
never touched WiFi CSI, FreeRTOS, or 802.11 internals before.

For the underlying RF/PHY theory — what OFDM subcarriers are, why a WiFi
receiver computes a channel estimate at all, what multipath and line-of-sight
attenuation physically look like — see the physics reference docs at
[`csi-learning/`](../../docs/csi-learning/), in particular
[`csi-learning/00-index.md`](../../docs/csi-learning/00-index.md) for orientation and
[`csi-learning/03-wifi-physics-deep-dive.md`](../../docs/csi-learning/03-wifi-physics-deep-dive.md) and
[`csi-learning/04-channel-state-information.md`](../../docs/csi-learning/04-channel-state-information.md) for the theory
this project is a hands-on instance of. This document does not re-derive
that material — it explains the concrete code that implements it.

(These physics docs are shared across this project and its siblings --
[`csi_presence`](../../csi_presence) and
[`analyse_csi`](../../analyse_csi) -- and live once, in the repo-level
`docs/csi-learning/` folder.)

Everything below was checked against the source as it exists at the time of
writing: `main/csi_test.c`, `tools/capture_csi.py`, `tools/analyze_csi.py`,
`tools/build_report.py`, and `tools/report_template.html`. Where the code
disagrees with the informal description that's been circulating for this
project, the code wins, and the disagreement is called out explicitly.

---

## 1. What this firmware is actually measuring, briefly

A WiFi receiver doesn't just detect "is there a signal or not" — to decode
an OFDM packet it has to know, for every subcarrier (every one of the ~52-242
narrow frequency slices an 802.11 channel is divided into), how much that
subcarrier's amplitude and phase were altered by the trip from transmitter
antenna to receiver antenna. That per-subcarrier complex correction factor
*is* the channel estimate, and it's derived from known reference symbols
embedded in the packet preamble (the "training fields," LTF = Long Training
Field). The receiver needs this purely to equalize the signal before
demodulating it — extracting and exposing it to application code (which is
what `esp_wifi_set_csi_config()` / `esp_wifi_set_csi_rx_cb()` do) is a
side channel that Espressif's WiFi stack happens to expose, not something
802.11 was designed to hand out.

What makes this channel estimate useful for *sensing* rather than just
demodulation is that it's a fingerprint of the physical RF path: how many
reflected copies of the signal arrive (multipath), their relative delays and
phases, and whether the direct line-of-sight path is present or blocked. A
person walking through the room changes some of those reflection paths
slightly (perturbing amplitude/phase on many subcarriers a little) or blocks
the direct path outright (a bigger, more uniform change, often paired with
an RSSI drop). This project captures that channel estimate on every packet,
turns it into interpretable amplitude/phase numbers, and plots it over time
so those physical events become visible as anomalies in the data. The rest
of this document is the mechanics of how that pipeline is built, not the RF
theory behind why it works — see the physics docs linked above for that.

---

## 2. The CSI config struct, field by field

```c
wifi_csi_config_t csi_config = {
    .lltf_en = true,
    .htltf_en = true,
    .stbc_htltf2_en = true,
    .ltf_merge_en = true,
    .channel_filter_en = false,
    .manu_scale = false,
    .shift = false,
};
```

This struct, passed to `esp_wifi_set_csi_config()`, controls which training
fields the radio extracts CSI from and how much post-processing it does
before handing the result to your callback. Every field is a tradeoff; here
is what each one actually buys or costs.

### `lltf_en` — Legacy Long Training Field

Every 802.11 frame, regardless of PHY generation (a/b/g/n/ac...), contains a
legacy preamble for backward compatibility, and that preamble includes the
Legacy LTF. Enabling this means you get a channel estimate from *every*
received packet, HT or not — this is the one field you can always rely on.
Legacy LTF spans the legacy 20 MHz channel width, so it yields on the order
of 64 subcarrier estimates (52 data + pilot subcarriers plus some
DC/guard positions depending on how the driver reports it — see §4 for what
actually lands in `buf`).

### `htltf_en` — HT (802.11n) Long Training Field

Only present in HT (802.11n and later, non-legacy) frames. It's a second,
independent channel estimate computed from a different training symbol than
the Legacy LTF, present only when the transmitter and receiver negotiated
HT. Because more HT-LTF symbols are typically present when MIMO/spatial
streams are in use, enabling this is what makes the CSI buffer noticeably
bigger for HT frames than for legacy frames — see §4.1 below on why HT
packets carry roughly 3x the subcarrier count of non-HT packets in this
capture.

### `stbc_htltf2_en` — second HT-LTF for STBC frames

STBC (Space-Time Block Coding) is a transmit-diversity scheme where the same
data is coded across multiple LTF symbols so a receiver with imperfect
per-antenna information can still recover it. When a packet uses STBC, there
is a second HT-LTF () beyond the plain HT-LTF, and this flag says
"extract that one too, when present." It only ever contributes data for
STBC-coded packets; for everything else it's a no-op.

### `ltf_merge_en` — merge LLTF and HT-LTF into a single estimate

This is the field worth understanding mathematically, not just as a
checkbox. When both a Legacy LTF and an HT-LTF channel estimate exist for
the same packet (i.e. it's an HT frame and `lltf_en && htltf_en`), the
hardware can either report them as two separate, independent measurements
of the same channel, or *merge* them into one. "Merging" here means
averaging the two independent estimates of the same underlying channel
response:

```
H_merged[k] = (H_lltf[k] + H_htltf[k]) / 2      (per subcarrier k, complex-valued)
```

Averaging two independent noisy measurements of the same quantity reduces
the variance of the estimate — classic signal averaging. If the LLTF and
HT-LTF estimates have independent, zero-mean measurement noise of variance
σ², the merged estimate has noise variance σ²/2 (in the ideal case), i.e.
roughly √2 lower noise standard deviation, for free, with zero extra
airtime cost. The tradeoff is the implicit assumption baked into that
average: the channel didn't change between when the LLTF symbol and the
HT-LTF symbol were each received. Both training fields are transmitted
microseconds apart within the same packet's preamble, so on any reasonable
timescale of physical motion (a person walking, a hand moving) this
assumption is safe — the channel is effectively static over a few
microseconds. It would *not* be safe if you were trying to resolve
sub-microsecond channel dynamics, but nothing in this project attempts
that. `csi_test.c` enables this (`.ltf_merge_en = true`), trading a small,
well-justified assumption for a real noise reduction on every HT packet.

### `channel_filter_en` — smoothing across subcarriers

When enabled, the hardware applies a low-pass filter *across adjacent
subcarriers* within one packet's estimate — trading frequency resolution
for reduced per-subcarrier noise, similar in spirit to the LLTF/HT-LTF
merge above but applied spatially (across frequency) instead of temporally
(across two measurements of the same channel). `csi_test.c` sets this to
`false`. The reasoning: this project's whole purpose is to look at how the
*fine-grained, per-subcarrier* pattern changes over time (the heatmap in
§6 is literally a subcarrier-by-time image), so smoothing subcarriers into
each other before you ever see them destroys the exact detail you're
trying to plot. The cost of leaving it off is that each subcarrier's
amplitude/phase carries more raw hardware noise — visible as the
speckled, not perfectly smooth, appearance of the heatmap even during the
"quiet baseline" period of a real capture. If your downstream use case only
cared about a single aggregate number per packet rather than the
per-subcarrier pattern, you'd more plausibly want `channel_filter_en =
true` to get a cleaner number per packet.

### `manu_scale` / `shift` — output scaling

`manu_scale = false` tells the hardware to auto-scale the raw CSI values
into its output format itself (choosing gain automatically). Setting it to
`true` would let the caller pick a manual scale factor and `shift` amount
instead, useful if you need bit-exact reproducibility of the same fixed
scale across captures, or need to match a specific downstream fixed-point
pipeline. Because `manu_scale` is `false` here, `shift` has no effect —
it's read by the driver only in the manual-scaling path — which is exactly
what the comment in `csi_test.c` says (`// unused since manu_scale is
false`). Auto-scaling is the simpler default and is what this project uses;
it does mean the *absolute* amplitude values are not necessarily
comparable, unit-for-unit, across firmware builds or hardware revisions if
the auto-scaling behavior ever changes — only relative changes within one
capture session are meaningful, which is all this project's analysis
(§6) ever relies on.

---

## 3. The producer/consumer queue architecture

This section is a general embedded-systems pattern, not a CSI-specific
trick — it's worth understanding on its own because you will hit the same
shape of problem in almost any embedded project with an interrupt or
driver-priority callback.

### The failure mode

`esp_wifi_set_csi_rx_cb()` registers `wifi_csi_cb()` to run **inside the
WiFi driver's own FreeRTOS task**, which the ESP-IDF WiFi stack runs at a
high priority so it can service incoming packets promptly (packet arrival
is a hard real-time concern — miss too many and the link degrades or
drops). Any code that runs inside that callback runs *at that task's
priority*, borrowing its time slice.

The original version of this firmware called `printf()` directly from
`wifi_csi_cb()` to emit the whole CSV line — timestamp, RSSI, channel, and
`amplitude,phase` for every subcarrier — as soon as a packet arrived. With
LLTF+HT-LTF+STBC-HT-LTF2 all enabled, that's on the order of 192
subcarriers × two `%.3f` floats each, roughly 2.5 KB of formatted text, sent
over a UART running at 115200 baud. At that baud rate transmitting 2.5 KB
takes on the order of 200 ms. Packets were arriving roughly every 200 ms
(driven by a background `ping` on the host). The net effect: the WiFi
task was almost continuously busy formatting and transmitting the previous
packet's line by the time the next packet arrived, leaving it effectively
running back-to-back at high priority with no idle gaps.

### Why that trips a watchdog

ESP-IDF runs a **task watchdog timer** (TWDT) — a hardware/software timer
that expects certain tasks (by default including the idle tasks on each
core) to periodically "check in" (feed the watchdog) within a configured
timeout. The idea is: if a low-priority housekeeping task like idle never
gets to run, something has gone wrong (a higher-priority task is stuck in
an infinite loop, deadlocked, or — as here — simply hogging the CPU) and
the system should be assumed unhealthy. If idle doesn't check in within the
timeout, the watchdog fires, logs `Task watchdog got triggered`, and (in
the default configuration) aborts/reboots the device. This is a
deliberate fail-fast safety mechanism: a device that's silently
unresponsive for seconds at a time is worse than one that visibly crashes
and, ideally, recovers.

In this firmware's original form, the WiFi driver task's high priority plus
its near-continuous busy time from `printf()` starved the idle task of
CPU time long enough to trip exactly this watchdog, and the device would
reboot during capture.

### The fix: decouple with a queue

The standard fix for "a high-priority/interrupt-context callback needs to
hand data to code that's allowed to be slow" is the classic
**producer/consumer pattern**: the fast context (producer) does the
absolute minimum — copy the data somewhere — and a separate, lower-priority
task (consumer) does everything slow, on its own schedule, without ever
blocking the producer.

```c
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *data)
{
    ...
    memcpy(record.buf, data->buf, record.len);
    if (xQueueSend(csi_queue, &record, 0) != pdTRUE) {
        dropped_count++;
    }
}
```

`wifi_csi_cb()` now does one `memcpy` of the raw packet bytes into a
stack-local `csi_record_t`, then hands it to `xQueueSend()` with a timeout
of `0` ticks — i.e. non-blocking: if the queue's ring buffer (depth
`CSI_QUEUE_DEPTH = 16`) is already full, `xQueueSend()` returns immediately
with `pdFALSE` instead of waiting for space, and the callback just
increments a drop counter and returns. No floating point, no formatting, no
I/O, no possibility of blocking — the WiFi task's time in this callback is
now bounded by a fixed-size `memcpy` regardless of system load.

```c
static void csi_print_task(void *arg)
{
    csi_record_t record;
    while (1) {
        if (xQueueReceive(csi_queue, &record, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        ... /* all the float math and printf happens here */
    }
}
```

`csi_print_task` is a separate FreeRTOS task created with priority 3 —
"above idle (0) so it always gets to run and feed the watchdog, but well
below the WiFi driver task so it never delays it," as the source comment
says. It blocks on `xQueueReceive(..., portMAX_DELAY)` until a record is
available, then does the amplitude/phase math and the (slow) `printf()`
entirely outside the WiFi task's critical path. Because it's not the WiFi
task, however long it takes to print a line, it cannot delay the next
packet's reception, and it runs at a priority where idle still gets
scheduled.

### What this design trades away, and what it protects

Non-blocking send with drop-on-full is a deliberate choice, and it's worth
naming its cost explicitly: **under sustained overload (the consumer
falling behind the producer for long enough to fill 16 queue slots), some
CSI samples are silently lost** — `dropped_count` increments but, as of
this version of the firmware, is never logged anywhere, so a real drop
event leaves no visible trace in the serial output; you'd only notice it
indirectly, as gaps in `device_us` timestamps larger than the expected
inter-packet interval when you inspect the CSV later. (This is a small,
real gap in observability — see §8.)

What it protects is more important for this system's actual purpose: the
WiFi task **can never be blocked** by a slow consumer. The alternative — a
blocking `xQueueSend()` with a nonzero timeout — would mean that if
`csi_print_task` ever fell far enough behind (e.g. serial buffer backs up
because nothing is reading the host side), the WiFi task itself would stall
waiting for queue space, which is exactly the failure mode that caused the
original crash, just moved one layer down. Dropping data is the correct
trade here because losing a few CSI samples degrades the *quality* of a
capture; a stalled or watchdog-killed WiFi task can bring down the whole
device and lose the *entire* capture, plus the WiFi link itself. This is
the general lesson: **when a producer must never stall, choose to drop data
under overload rather than backpressure the producer** — the alternative
(bounded blocking, or an unbounded queue) either reintroduces the stall risk
or trades it for unbounded memory growth.

---

## 4. From raw bytes to physical quantities

### Byte layout

`wifi_csi_info_t.buf` (aliased into `csi_record_t.buf`, a `int8_t[512]`
array in this firmware) is a flat byte buffer containing, per subcarrier,
one **signed 8-bit imaginary** byte immediately followed by one **signed
8-bit real** byte:

```
buf = [ im0, re0, im1, re1, im2, re2, ... ]
```

Each byte is a `signed char` (`int8_t`), range −128..127. This is a
quantized fixed-point representation of the complex channel-estimate value
for that subcarrier, `H[k] = re[k] + j*im[k]`, at whatever scale the
hardware's auto-scaling (`manu_scale = false`, see §2) chose for this
packet. Because it's only 8 bits per component, the dynamic range is
limited — this is part of why the analysis code in §6 treats absolute
amplitude values as only meaningful *relative to other values in the same
capture*, not as an absolute physical unit.

### `first_word_invalid`

```c
int start = record.first_word_invalid ? 4 : 0;
```

The ESP32 WiFi hardware's CSI extraction pipeline can mark that the first
32-bit word (4 bytes = 2 subcarriers' worth of `im,re` pairs) of the buffer
is not valid data — a hardware artifact of how the DMA/FFT pipeline
pipelines its output, not something firmware or application code can avoid.
When `data->first_word_invalid` is set, the firmware skips those first 4
bytes entirely before interpreting anything as subcarrier data; those two
subcarrier slots are simply absent from the output rather than reported as
(wrong) zeros. This is a hardware quirk you take as given, not a bug to fix
in this code — it's called out here because if you ever hand-decode a raw
CSI dump without checking this flag, you'll silently misalign every
subsequent subcarrier by two slots.

### Amplitude and phase

```c
float im = (float)record.buf[idx];
float re = (float)record.buf[idx + 1];
float amplitude = sqrtf((im * im) + (re * re));
float phase = atan2f(im, re);
```

Treating `(re, im)` as the Cartesian coordinates of a complex number
`H = re + j*im`, this is just the standard conversion to polar form:

- **amplitude** = `|H| = sqrt(re² + im²)` — the magnitude of the channel's
  effect on that subcarrier, i.e. roughly "how much did this frequency get
  attenuated/amplified passing through the environment." This is the
  quantity plotted in the heatmap and the one the variability score (§6) is
  computed from.
- **phase** = `atan2(im, re)` — the angle of `H` in the complex plane, i.e.
  the phase shift the environment imposed on that subcarrier. `atan2` (not
  plain `atan`) is used because it's the two-argument arctangent that
  correctly resolves the full −π..π range using the signs of both `im` and
  `re`, rather than losing the quadrant information a single-argument
  `atan(im/re)` would (and than dividing by zero when `re == 0`).

Phase is captured and written to the CSV (see §5) but note that neither
`analyze_csi.py` nor the HTML report currently *uses* the phase values —
only amplitude feeds the matrix and the variability score. Phase is
noisier and needs unwrapping/calibration to be directly useful (a detail
covered in the physics docs, not reproduced here); it's captured because
it's cheap to compute alongside amplitude and may be useful for future
analysis, not because the current pipeline does anything with it.

### Why this all happens off the WiFi task

Note that all of the above — the `sqrtf`/`atan2f` floating point calls —
happens inside `csi_print_task`, not `wifi_csi_cb`. This is a second reason
(beyond the `printf` cost) the original design was risky: floating point
math itself is not free, and on some ESP32 variants trapping into the FPU
context from a high-priority ISR-adjacent context has its own subtleties.
Keeping *all* of §4's math in the low-priority consumer task, per §3's
architecture, sidesteps this entirely — the callback truly only ever does
an integer-range `memcpy`.

---

## 5. The CSV wire format

This is the interface contract between firmware and every downstream tool,
so it's worth documenting precisely, as a small spec, rather than just
prose.

### On the serial line (emitted by `csi_print_task`)

```
CSI,<timestamp_us>,<rssi>,<channel>,<num_subcarriers>,amp0,phase0,amp1,phase1,...,amp{n-1},phase{n-1}
```

Produced by:

```c
printf("CSI,%lld,%d,%d,%d",
       (long long)record.timestamp_us, record.rssi, record.channel, num_subcarriers);
for (int i = 0; i < num_subcarriers; i++) {
    printf(",%.3f,%.3f", amplitude, phase);
}
printf("\n");
```

Field by field:

| Field | Type | Source | Notes |
|---|---|---|---|
| literal `CSI` | string | fixed prefix | disambiguates this line from ESP-IDF log lines (boot logs, WiFi driver `ESP_LOGx` output) sharing the same UART. Any host tool must filter on lines starting with `CSI,` and silently ignore everything else. |
| `timestamp_us` | signed 64-bit int, printed via `%lld` | `esp_timer_get_time()` at the moment the callback fired (captured in `wifi_csi_cb`, *not* when `csi_print_task` gets around to printing it) | microseconds since the ESP32 booted; not wall-clock, not synced to any external clock |
| `rssi` | signed int (`%d`) | `data->rx_ctrl.rssi` | received signal strength in dBm, typically negative |
| `channel` | unsigned int printed as `%d` | `data->rx_ctrl.channel` | WiFi channel number the packet was received on |
| `num_subcarriers` | int | computed as `(record.len - start) / 2` | varies packet to packet — see §6.2 for why |
| `amp_i,phase_i` pairs | two floats each, `%.3f` (3 decimal places) | computed per §4 | exactly `num_subcarriers` pairs follow, comma-separated, no trailing delimiter before the newline |

The line is newline-terminated (`\n`) and is otherwise plain ASCII/CSV —
no quoting or escaping is used or needed since none of the fields can
contain a comma.

One important detail: `timestamp_us` is captured in `wifi_csi_cb` at
packet-arrival time and copied through the queue, so it reflects the true
arrival time of the packet, not the (variable, queue-dependent) time at
which `csi_print_task` later formats and emits the line. This matters for
downstream timing analysis — the CSV's `device_us` column is trustworthy as
an arrival timestamp even though the actual `printf()` for that row may
happen noticeably later.

### In the recorded CSV file (written by `capture_csi.py`)

`capture_csi.py` reads each `CSI,...` line, splits it, and re-emits it as a
proper CSV row with one added column:

```
host_time,device_us,rssi,channel,num_subcarriers,values
```

- `host_time` — the **recording machine's** wall-clock time
  (`datetime.datetime.now().isoformat(timespec="milliseconds")`) at the
  moment that line was read off the serial port. This is what lets you
  correlate "I walked past the ESP32 at 14:32:10" (a fact you know in
  human wall-clock time) with a specific row, since `device_us` alone (µs
  since the ESP32's boot) has no relationship to wall-clock time.
- `device_us`, `rssi`, `channel` — copied through unchanged from the
  firmware's line (parsed out of `parts[1:5]`).
- `num_subcarriers` — copied through unchanged, but note: **not
  re-validated** against the actual count of values present. It's whatever
  the firmware said, kept as a separate column for convenience; the
  authoritative count when actually parsing amplitudes downstream is
  `len(values.split(",")) // 2`, computed independently in
  `analyze_csi.py` (see §6.2) — the two could in principle disagree if a
  line was truncated in transit, which is exactly the kind of malformed
  row the analysis step is built to tolerate.
- `values` — the entire remaining `amp0,phase0,amp1,phase1,...` tail,
  rejoined verbatim as one CSV field via `",".join(parts[5:])`. It is
  *not* split into individual amplitude/phase columns at this stage; that
  happens later, in `analyze_csi.py`.

`capture_csi.py` silently drops (does not write) any serial line that
doesn't start with `CSI,` (line 52: `if not line.startswith("CSI,"): continue`),
and drops any `CSI,` line with fewer than 5 comma-separated parts (a
truncated/corrupted line). It also treats undecodable bytes
(`UnicodeDecodeError` on a line's raw bytes; decoding uses
`errors="ignore"` first so this is really a fallback for total failures)
as a reason to skip the line rather than crash the capture. This matters in
practice because the CSI serial stream and ESP-IDF's own log output share
one physical UART; a normal capture session's raw serial output is a mix of
boot logs, WiFi driver diagnostic lines, and `CSI,` lines, and occasionally
a log line and a `CSI,` line can interleave mid-write if timing is
unlucky — the `CSI,`-prefix filter combined with the "skip malformed rows"
tolerance in both `capture_csi.py` and `analyze_csi.py` is the defense
against that.

---

## 6. The analysis pipeline, algorithm by algorithm

### 6.1 `capture_csi.py` — the recording step

The core design decision here is bypassing `idf.py monitor` and talking to
the serial port directly via `pyserial`. `idf.py monitor` is built for
interactive use (colorized log output, keyboard shortcuts to reset/build);
it's not meant to be piped into a file-writing script reliably, and it adds
its own buffering/formatting layer between the raw UART bytes and anything
reading them. Opening the port directly (`serial.Serial(args.port,
args.baud, timeout=1)`) gives this script an unmediated, line-buffered view
of exactly what the ESP32 sent, which is what a script trying to parse a
strict `CSI,...` format needs.

The read loop is a simple blocking `ser.readline()` with a 1-second
per-read timeout — `serial.Serial(..., timeout=1)` means `readline()` gives
up and returns an empty bytes object after 1s of no newline, rather than
hanging forever; the loop just treats an empty read (`if not raw: continue`)
as "nothing arrived yet, try again," which also lets Ctrl+C
(`KeyboardInterrupt`) get noticed promptly instead of being stuck inside an
indefinite blocking read.

Rows are flushed to disk every 20 rows (`if rows_written % 20 == 0:
f.flush()`) rather than after every row or only at exit — a middle ground
between "lose almost nothing if the process is killed uncleanly" and "don't
pay a flush syscall on every single packet."

### 6.2 `analyze_csi.py` — turning a CSV into a plottable summary

**Step 1 — parse and tolerate malformed rows.** Each row's `values` field is
split on commas and converted to floats; `device_us` and `rssi` are parsed
as ints. Any row that fails to parse (`ValueError`) or is missing an
expected column (`KeyError`) is counted in `skipped` and dropped, rather
than aborting the whole batch — consistent with the "the serial stream can
have glitches" reality described in §5.

**Step 2 — filter to the modal subcarrier count, and why.** Not every
captured packet has the same CSI buffer length. Two different reasons for
that combine in a real capture:

1. *Legitimate, expected variation*: whether a given received frame is an
   HT frame or not. Because `csi_config` enables `lltf_en`, `htltf_en`, and
   `stbc_htltf2_en` together, an HT data frame's CSI buffer is really
   **three concatenated channel-estimate groups** — legacy LTF, HT-LTF, and
   (for STBC frames) the second HT-LTF — each on the order of 64
   subcarriers wide, for roughly 192 subcarriers total. A non-HT frame
   (e.g. a management/control frame, or a data frame sent at a legacy
   rate) only ever has the legacy-LTF group, so its buffer works out to
   roughly 64 subcarriers. Both are "correct" outputs of the hardware for
   the frame type actually received — they're not corruption, they're
   different frame types producing different-shaped CSI.
2. *Actual corruption*: a line truncated or garbled somewhere between the
   `printf()` and the file on disk (see §5) would also produce an
   unexpected length, indistinguishable at this stage from case 1 by
   length alone.

The code can't cheaply tell these two cases apart per-row, so it takes the
pragmatic route: compute `statistics.mode(lengths)` — the single most
common subcarrier count across all successfully parsed rows — and keep only
rows matching that count, dropping everything else (`bad_length` count is
printed). In a typical capture where the host is pinging the ESP32
steadily and most received frames are HT data frames, the mode will be the
~192-subcarrier HT count, and the ~64-subcarrier non-HT frames (along with
any truly corrupted rows) get dropped from *this analysis* — they remain
untouched in the raw CSV. This trades completeness (you lose roughly a
third of packets in a typical run — see the known-limitations note in
`v1.md`, restated with reasoning in §8 below) for a clean, rectangular
matrix that every downstream computation (the heatmap, the variability
score) can assume without special-casing ragged rows.

**Step 3 — identify "active" subcarrier columns.** Some subcarrier
positions are structurally always (or almost always) zero across an entire
capture — guard-band or null subcarriers that carry no signal by design
(part of the OFDM structure covered in the physics docs), or hardware
buffer positions that never get populated for this configuration. Including
those in a spread statistic would just dilute it with a run of exact zeros
that never change, understating how much the *real* signal-carrying
subcarriers moved. `analyze_csi.py` computes this once, before the
per-packet loop:

```python
active_cols = [
    i for i in range(num_subcarriers)
    if any(r["amplitudes"][i] != 0.0 for r in rows)
]
```

i.e. a column is "active" if it is nonzero in *at least one* packet in the
whole capture — a fairly permissive definition (it only excludes columns
that are zero in literally every single packet), which is deliberate: it's
cheap, requires no domain knowledge of exactly which subcarrier indices are
pilot/null/guard positions for this specific PHY mode, and errs on the side
of keeping data rather than risking excluding a subcarrier that's
legitimately meaningful.

**Step 4 — the variability score.** For each packet, over only the active
columns:

```python
variability.append(round(statistics.pstdev(active_vals), 3) if len(active_vals) > 1 else 0.0)
```

`statistics.pstdev` is the **population** standard deviation (divides the
sum of squared deviations by `N`, not `N-1` as the *sample* standard
deviation `statistics.stdev` would). This is a deliberate, correct choice
here, not an oversight: the active-subcarrier amplitudes in one packet
*are* the entire population you care about for that packet — you're not
trying to use this packet's subcarriers as a sample to infer some larger
population's spread, so there's no "N-1 correction for estimating a
population from a sample" step to make. (In practice, with ~150+ active
subcarriers, the N vs N-1 difference is negligible anyway — this is a
correctness note, not a practically significant one.)

Why standard deviation specifically, rather than the alternatives:

- **Range (max − min)**: extremely sensitive to a single outlier
  subcarrier — one noisy or pilot-boosted subcarrier could dominate the
  whole score regardless of what every other subcarrier is doing, which
  would make the score respond to hardware quirks on one frequency rather
  than to the environment as a whole.
- **A percentile spread (e.g. 90th − 10th percentile)**: more robust to
  single outliers than range, but more expensive to compute (needs a sort)
  and, more importantly, drops information — it only looks at two order
  statistics and ignores how the rest of the distribution moved.
- **Standard deviation**: uses every active subcarrier's value, is cheap to
  compute in one pass, and — critically for the *use case* here — responds
  in the right direction to the physical phenomenon being detected. A
  quiet, stable RF environment produces amplitudes that cluster tightly
  around their per-subcarrier baseline (low spread across subcarriers in a
  given snapshot, since the channel is doing roughly the same thing to
  every subcarrier's average level, only differing by the fixed
  frequency-selective fading pattern). Motion perturbs many different
  subcarriers by different, semi-random amounts simultaneously (each
  reflection path is affected differently), which increases the spread of
  amplitude values within a single packet. Standard deviation is exactly
  "how spread out are these values," making it a direct, computationally
  cheap proxy for "how much did this packet's channel snapshot deviate
  from a flat, uniform pattern" — which is the observable this project
  cares about. It is a *summary* statistic, not a spatially localized one:
  it deliberately throws away *which* subcarrier changed, in exchange for
  one number per packet that's simple to threshold and plot. That's an
  intentional simplification for this project's goal ("did something
  change," not "what exactly changed, physically").

The `len(active_vals) > 1` guard avoids a divide-by-zero/undefined-spread
edge case (`pstdev` needs at least 2 data points) and defaults to `0.0`
variability in that degenerate case, which should essentially never trigger
in practice given the typical ~150+ active subcarrier count.

**Output.** The JSON summary carries `num_packets`, `num_subcarriers`,
`duration_s` (the last entry of `time_s`), `time_s[]` (seconds since the
*first row surviving all filters*, i.e. `(device_us - t0) / 1e6`, using
`t0 = rows[0]["device_us"]` — device clock, not `host_time`), `rssi[]`,
`variability[]`, and the full `matrix[][]` of (rounded to 2 decimals)
amplitudes, one row per surviving packet, one column per subcarrier.

### 6.3 `build_report.py` — templating, not "building" in any complex sense

This script does exactly one thing: read `report_template.html`, verify it
contains the literal placeholder string `__DATA_JSON__`, and do a plain
string `.replace()` of that placeholder with the raw JSON text from
`analyze_csi.py`'s output file, then write the result out. There is no
templating engine, no escaping step beyond what's already true of JSON
(valid JSON embedded directly inside a `<script>` tag as a JS expression is
itself valid JS, since JSON is a subset of JS object/array/string/number
literal syntax) — this is a deliberately minimal, dependency-free approach:
the output is one flat HTML file with everything (styles, script, and data)
inlined, that opens directly in a browser with no server and no build step.

### 6.4 `report_template.html` — the visualization algorithms

Everything below runs client-side, in the browser, against the embedded
`DATA` object.

**Heatmap color domain — 95th-percentile clamp.** The heatmap draws one
`fillRect` per `(packet, subcarrier)` cell, colored along a 13-stop
sequential light-to-dark-blue ramp (`seqColor`, linear interpolation
between adjacent hex stops in the array). The color's position on that ramp
is `v / domainMax`, and `domainMax` is deliberately **not** the true
maximum amplitude in the whole matrix — it's the 95th percentile:

```js
const flat = [];
for (const row of DATA.matrix) for (const v of row) flat.push(v);
flat.sort((a,b)=>a-b);
const domainMax = flat[Math.floor(flat.length * 0.95)] || 1;
```

This is a standard robust-scaling technique for visualizing data with a
long right tail. Certain subcarrier positions (pilot subcarriers, in
particular) legitimately carry much higher amplitude than the bulk of
ordinary data subcarriers by design (part of the OFDM structure — see the
physics docs), and outlier hardware noise spikes can also occur. If the
color domain were clamped to the true max, those few very bright cells
would map to the top of the ramp and every ordinary-magnitude cell would be
compressed into a narrow, visually similar low-to-mid range of the ramp —
you'd see one or two bright dots and a sea of nearly uniform pale blue,
with all the *interesting* variation illegible because it all landed in a
narrow slice of the color scale. Clamping the domain to the 95th percentile
means the top 5% of cells simply saturate (all read as the same "max"
color instead of being individually distinguishable above that point), but
the other 95% of cells now spread across the *entire* ramp, making
ordinary variation visible. This is a visual clarity tradeoff, deliberately
made — a handful of individually-very-bright cells become indistinguishable
from each other, in exchange for all typical cells becoming distinguishable
from each other, which is what you want when the point of the chart is
"can I see a pattern change over time," not "can I read off the exact
value of the single brightest cell."

**Line chart gap breaking.** The RSSI and variability SVG line charts break
the path (`M` instead of `L` in the SVG path data) whenever the gap between
consecutive samples' `time_s` exceeds 1.5 seconds:

```js
if (i > 0 && times[i] - times[i-1] > gapThreshold) segStart = true;
```

This prevents the chart from drawing a straight interpolated line across a
real gap in the data (e.g. a lull in ping traffic, or a burst of
non-HT/dropped packets per §6.2) as if it were a smooth, continuous
transition — which it isn't; there's no data there at all, and drawing a
line through it would visually imply a value that was never measured.

**Anomaly-band heuristic.** Both charts shade "bands" — time ranges where
the series value crossed a threshold derived from the *first 15 seconds* of
the capture, treated as a baseline:

```js
const baselineN = DATA.time_s.filter(t => t < 15).length || 10;
const baseVar = DATA.variability.slice(0, baselineN);
const baseRssi = DATA.rssi.slice(0, baselineN);
const varThreshold = mean(baseVar) + 2*std(baseVar);
const rssiThreshold = mean(baseRssi) - 8;
```

Two different thresholding rules are used for the two series, and this is
worth being precise about since it's easy to assume (incorrectly) that both
use the same "2 standard deviations" rule:

- **Variability**: `mean(baseline) + 2 * std(baseline)`, a genuine
  mean-plus-2-sigma threshold. `std()` here (a small helper defined in the
  template's own `<script>`, not `analyze_csi.py`'s `pstdev`) is likewise a
  population standard deviation (divides by `N`, no Bessel's correction) —
  computed independently, in JS, over just the first-15-seconds slice.
  `findBands(DATA.variability, DATA.time_s, varThreshold, true)` (the
  `above=true` case) then finds contiguous runs of samples strictly above
  that threshold.
- **RSSI**: `mean(baseline) − 8` — a **fixed 8 dBm offset below the
  baseline mean**, not a standard-deviation-based rule at all.
  `findBands(..., rssiThreshold, false)` (`above=false`) then finds
  contiguous runs strictly *below* that threshold. This is a deliberately
  different, coarser heuristic: RSSI in a stable link is usually far
  steadier than CSI variability sample-to-sample (dominated by transmit
  power control and a fairly static path loss, whereas variability is
  already a spread statistic that fluctuates more naturally), so a fixed
  dBm offset is treated as a simpler, adequate proxy for "signal path
  meaningfully obstructed" without needing to first establish how noisy the
  RSSI baseline itself is.

`findBands` itself is a straightforward single-pass run-length encoder over
a boolean "is this sample past the threshold" predicate: it walks the
series once, opens a band when the predicate turns true, closes it (and
records `[startTime, endTime]`) when the predicate turns false again or the
series ends, and discards single-sample blips (`if (i - 1 > start)` — a
band must span more than one sample to be recorded, filtering out one-off
noise spikes from becoming a shaded region of their own).

**What this heuristic assumes, and where it misleads.** This is explicitly
called out in the report's own footer text
(`"Bands are computed from the data itself ... not from any recorded
ground-truth"`), and it's worth being precise about *why* that caveat
matters:

- It assumes the **first 15 seconds are representative of "normal"** —
  if something was already happening during that window (the environment
  wasn't actually quiet, or the WiFi link hadn't stabilized yet — e.g. a
  weak RSSI on connect that improves as retransmission/rate-adaptation
  settles), the baseline itself is contaminated, and everything downstream
  (both threshold values) inherits that error silently.
- The `baselineN = ... || 10` fallback (if fewer than one full sample
  arrived before the 15s mark — a slow start) means very short or sparse
  captures can end up computing a baseline over as few as 10 samples,
  which for a spread statistic like variability is a fairly small,
  noise-sensitive sample to derive a `mean + 2*std` threshold from.
- It's a purely **retrospective, statistical** trigger, not tied to any
  independently recorded ground-truth event (see §8 — there is no
  "movement started here" marker mechanism in this pipeline at all). A
  band means "this differs from what came before it in this capture," not
  "we know something physically happened here" — those two claims usually
  coincide in a deliberately-staged capture like the one in §7, but nothing
  in the pipeline can distinguish a real environmental change from, say, a
  slow WiFi rate/power adaptation transient, a router reconfiguring
  transmit power, or a change in ambient RF interference from an unrelated
  device.
- Because RSSI uses a fixed 8 dBm offset rather than a statistically
  calibrated one, a link with a genuinely noisy baseline (say, ±6 dBm
  swing even when nothing is happening) would trip false RSSI bands
  regularly, while a very stable link with a real but modest 5 dBm drop
  would never trigger one at all — the threshold is not adaptive to how
  noisy this *particular* capture's baseline actually was, unlike the
  variability threshold.

---

## 7. Reading the real capture: a case study

A real 45-second capture on this hardware (steady ~10 pkt/s driven by a
background `ping` from another device on the same network) produced a
report showing three distinct phases. Walking through why each phase looks
the way it does, physically, is the most concrete way to understand what
the pipeline is actually surfacing.

**Phase 1 — quiet baseline (variability roughly 7–8).** With nobody moving
near the RF path between the ESP32 and the router, the multipath
environment — the set of reflective surfaces (walls, furniture) the signal
bounces off on its way to the receiver — is essentially static from packet
to packet. Each subcarrier's amplitude is governed almost entirely by the
fixed, frequency-selective fading pattern of that static environment
(different subcarriers naturally have different amplitude because
constructive/destructive interference between multipath copies depends on
frequency), plus a modest, roughly constant amount of hardware/measurement
noise. The per-packet spread across active subcarriers (the variability
score, §6.2) is therefore low and roughly flat over time — this is exactly
the "steady horizontal bands" pattern the heatmap's own description text
calls out, and it establishes the mean/std baseline the report's
anomaly-band heuristic (§6.4) is computed from.

**Phase 2 — elevated variability (roughly 13–15, on the order of double
baseline), RSSI unaffected.** A period consistent with someone moving *near*
the signal path without stepping directly into the line-of-sight beam
between the two devices. Motion perturbs the *multipath* component of the
channel: a moving body reflects, scatters, or slightly delays some of the
non-direct signal paths, and because different subcarriers respond
differently to a given change in one reflection path's delay/phase (this
is exactly what "frequency-selective fading" means), the result is that
many, but not all, subcarriers' amplitudes shift — some up, some down, by
different amounts — increasing the packet-to-packet and within-packet
spread. Crucially, the *direct*, line-of-sight path is undisturbed during
this phase, so the total received power (RSSI, which is dominated by
whichever path carries the most energy — usually the direct path when one
exists) stays essentially at its baseline level. This is the signature of
"CSI variability moved, RSSI didn't": a real physical disturbance in the
multipath environment that never blocked the direct path.

**Phase 3 — sharp RSSI drop (baseline around −38 dBm to roughly −80 to
−84 dBm), variability spiking further.** A qualitatively different event:
RSSI dropping by roughly 40+ dB is consistent with the *direct* line of
sight itself being physically obstructed — a person (or object) stepping
directly between the ESP32 and the router, attenuating or blocking the
strongest path rather than merely perturbing weaker reflected ones. Once
the direct path is blocked (or heavily attenuated), the received signal is
dominated by whatever weaker, more variable reflected paths remain, which
both drives RSSI down sharply (much less total energy arriving) and pushes
CSI variability even higher than phase 2 (the surviving signal is now
composed almost entirely of the more chaotic, easily-perturbed reflected
components, with none of the previously-dominant stable direct-path
contribution to anchor the amplitude pattern). This is exactly the
"RSSI drop usually means something obstructed the line of sight" reading
the report's own RSSI-chart description gives, and it's the phase where
both of the report's independent anomaly-band heuristics (§6.4) would be
expected to fire together — variability crossing its `mean + 2σ` threshold
and RSSI crossing its `mean − 8 dBm` threshold over the same, or an
overlapping, time window.

The general lesson this case study teaches: **variability and RSSI are
sensitive to physically different things** — variability responds to
*any* multipath disturbance including subtle ones that don't touch the
direct path, while RSSI responds primarily to whether the strongest
(usually direct) path is intact. A capture where variability rises but
RSSI doesn't is a meaningfully different physical event from one where both
move together, and the two-chart-plus-heatmap layout in the report exists
specifically so you can tell those two cases apart by eye rather than
relying on a single collapsed number.

---

## 8. Known gaps

This restates the "Known limitations / v2 candidates" from `v1.md`'s
closing section, with the reasoning behind why each one is a real,
non-cosmetic limitation rather than a minor nicety.

**No ground-truth event markers.** `capture_csi.py` records `host_time`
(wall-clock) per row, which in principle lets you correlate a capture
against a manually-kept log of "I moved at 14:32:10" written after the
fact from memory. But there is no in-band mechanism (e.g. a keypress that
writes a tagged marker row into the same CSV) to record "movement started
here" *at the moment it happens*, synchronized to the exact same
timestream the rest of the data uses. The consequence is concrete: the
anomaly-band heuristic in §6.4 is the *only* automated signal pointing at
"something happened here," and validating it currently means eyeballing
band boundaries against a human's (increasingly fuzzy, if reconstructed
after the fact) memory of when they moved — there's no way to
mechanically check the heuristic's precision/recall against a real
labelled event log, which is exactly the kind of validation you'd want
before trusting this pipeline for anything beyond exploratory analysis.

**Roughly a third of captured packets (the non-HT, ~64-subcarrier ones) are
dropped entirely by `analyze_csi.py`'s modal-length filter (§6.2).** This
isn't just "fewer data points" — it's a systematic exclusion correlated
with *frame type*, not random sampling noise, so if non-HT frames tend to
arrive under different conditions than HT frames (for instance, if the
WiFi rate-adaptation algorithm falls back to legacy rates specifically
under poor channel conditions), the dropped packets are not a
representative subset of "what happened during the capture" — they may be
disproportionately drawn from exactly the interesting, degraded-channel
moments the whole pipeline is trying to detect. At minimum, the RSSI and
timing information from those dropped packets (valid and available even
without a usable amplitude matrix) is currently thrown away along with the
CSI amplitudes; a future version could keep RSSI/timing from all packets
in the report while restricting only the amplitude-matrix/variability
computation to the modal-length subset.

**Only `rssi` and `channel` are pulled out of `rx_ctrl` by the firmware.**
`wifi_csi_info_t.rx_ctrl` also carries fields such as rate/MCS index, an
STBC flag, and a noise-floor estimate (per the `esp_wifi_types_native.h`
struct definition), none of which are currently read or logged. This
matters because several of them would directly disambiguate open questions
elsewhere in this document: the MCS/rate field would let you *directly*
confirm (rather than infer from subcarrier count) whether a given packet
used a legacy or HT rate, and hence exactly why its CSI buffer is short —
removing the ambiguity between "legitimately a non-HT frame" and
"corrupted row" described in §6.2. The noise floor would let variability
be normalized against actual RF noise conditions instead of assumed
roughly constant. None of this is implemented; it's available for free (no
additional hardware capability needed) whenever it's worth the added CSV
columns and parsing.

**Single AP, single ESP32 — no multi-node or multi-antenna comparison.**
Everything in this pipeline characterizes one wireless link (ESP32 station
↔ one access point) from one vantage point. Physical events that don't
lie on, or near, that specific single path between those two specific
points may be weakly visible or invisible entirely — a person moving in a
part of the room that doesn't intersect any significant multipath
component of *this* link simply won't perturb it much. Distinguishing "no
motion happened" from "motion happened somewhere this single link is
insensitive to" is not possible from this data alone; that generally
requires either multiple links (multiple AP/STA pairs, or multiple
antennas) observed simultaneously, which this project does not yet
attempt.
