# WiFi CSI Sensing — Physics & Signal Processing Reference

This is a five-part reference explaining **why** WiFi Channel State
Information (CSI) sensing works, from the physics of a radio wave up to
the specific bytes the ESP32-S3 hands your callback in `csi_test`,
`csi_presence`, and `analyse_csi`. It is written for engineers who can
already read C and Python but have never had to reason about phase,
subcarriers, or multipath.

These documents are deliberately separated from each firmware project's
own documentation, and are duplicated identically into all three
projects' repos (each is self-contained on GitHub rather than depending
on a shared local folder). The split is:

- **This directory (`docs/csi-learning/`)** — *why the signal behaves the
  way it does*. Physics, math, and signal-processing theory. No project
  source code, no build instructions, nothing that changes when the
  firmware changes.
- **Each project's own `docs/`** — *how a specific piece of code turns
  that physics into a decision*. Build instructions, the actual C
  structures, the specific algorithm each project runs, real-hardware
  test results, known bugs:
  - [`csi_test`](https://github.com/shivamkurzekar/csi_test/tree/main/docs) —
    [`v1.md`](https://github.com/shivamkurzekar/csi_test/blob/main/docs/v1.md)
    (overview, build/run) and
    [`02-algorithms-and-concepts.md`](https://github.com/shivamkurzekar/csi_test/blob/main/docs/02-algorithms-and-concepts.md)
    (deep dive)
  - [`csi_presence`](https://github.com/shivamkurzekar/csi_presence/tree/master/docs) —
    [`v1.md`](https://github.com/shivamkurzekar/csi_presence/blob/master/docs/v1.md) and
    [`02-algorithms-and-concepts.md`](https://github.com/shivamkurzekar/csi_presence/blob/master/docs/02-algorithms-and-concepts.md)
  - [`analyse_csi`](https://github.com/shivamkurzekar/analyse_csi/tree/master/docs) —
    [`v1.md`](https://github.com/shivamkurzekar/analyse_csi/blob/master/docs/v1.md)
    (no separate deep-dive doc yet for this project)

Read this directory first if a term like "subcarrier," "phase
unwrapping," or "channel estimate" is doing a lot of unexplained work in
the project docs. Read the project docs first if you just want to flash a
board and see it work, and come back here when you hit a "why does it do
that" question the project docs don't answer.

## Reading order

1. **[`01-waves-and-modulation.md`](./01-waves-and-modulation.md)** —
   Start here if radio is a black box to you. Covers what an
   electromagnetic wave physically is, why WiFi uses the frequencies it
   uses, how to represent a wave's state as a complex number (I/Q), and
   how digital bits get encoded onto a carrier wave (ASK/PSK/QAM). Ends at
   the level of a single subcarrier.

2. **[`02-ofdm-and-80211-phy.md`](./02-ofdm-and-80211-phy.md)** — Why WiFi
   doesn't use one wave but dozens of them at once (OFDM), what makes that
   trick work (orthogonality, the cyclic prefix), and how a real 802.11
   frame is structured — preamble, training fields, signaling, payload.
   This is where the doc explicitly connects `lltf_en`, `htltf_en`,
   `stbc_htltf2_en`, and `ltf_merge_en` to specific parts of that frame
   structure.

3. **[`03-wifi-physics-deep-dive.md`](./03-wifi-physics-deep-dive.md)** —
   The main narrative. A single continuous story, from "what does a
   router physically send" through "what happens to that signal bouncing
   around a room" to "what does an amplitude change versus a phase change
   actually tell you about the physical world." This is the document that
   directly explains the RSSI-drop-and-variability-spike story observed in
   real captures, and why `csi_presence` chose phase-over-time instead of
   amplitude-spread-across-subcarriers.

4. **[`04-channel-state-information.md`](./04-channel-state-information.md)**
   — The formal bridge from physics to the data structure you actually
   receive in code. Defines CSI precisely, maps the math directly onto the
   `(imaginary, real)` byte pairs and `amplitude`/`phase` computation used
   in this codebase, and covers phase unwrapping in detail with a worked
   example. Ends by pointing to the project docs for what happens to this
   data next.

You can read 03 on its own if you just want the physical intuition and
are willing to take "OFDM," "subcarrier," and "training field" as given
terms — each later document has a short recap of anything it depends on
from an earlier one. But 01 → 02 → 03 → 04 in order is the intended path,
and each document assumes you could have read the ones before it.

## What these documents are not

They are not a 802.11 standard reference, not a MATLAB/Python CSI
toolkit tutorial, and not a guide to any specific vendor's chip. Numbers
are given to the precision that's useful for building physical intuition;
where the exact figure depends on which 802.11 amendment or hardware
you're looking at, that's called out explicitly rather than asserted as a
single universal constant.
