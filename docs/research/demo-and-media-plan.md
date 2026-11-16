---
type: Research
title: Demo and media plan
description: How each milestone and each slice inside it gets a short, honest, watchable demo, starting with the headless milestone 1.
timestamp: 2026-11-16T23:59:00Z
status: active
tags: [demo, media, process]
---

# Demo and media plan

**TL;DR.** Every milestone ends with a demo, and so does every slice that is worth showing. The habit is the point: build a little, show it, and keep a trail of clips that doubles as progress notes and motivation. Milestone 1 has no renderer, so its demos are small HTML pages that replay data the core dumped. Full-size videos come once the core runs in the browser (milestone 2).

## The habit

- **Every plan phase that proves a visible or measurable thing gets a "Demo:" line** naming what the viewer will see and the one number or moment that carries it.
- **A clip is 20 to 90 seconds,** one idea, a caption with the real number. Honest by default: show the failure case beside the success, and say what is approximate.
- **Highlights are tagged `[demo]`** in the engineering record, with the file name of the clip and one line on why it is interesting. That list is the raw material for a longer progress video.
- **Evidence stays local.** Clips, frames and dumps live in a gitignored `artifacts/` folder (the repo is public and regenerable output is never committed). A scenario must be re-runnable from the repo with one command, so any clip can be remade.
- **Effort rule:** a demo is part of "done" for the phase, but it is allowed to be small. Skipping one needs a one-line reason in the record.

- **Time box:** about 30 minutes per slice demo and one session for a milestone wrap. Media must not eat the solver work.
- **Honest labels:** the player always shows a fixed "replay, recorded at N ms per step" label, and captions carry the commit hash. A replay is never presented as live.
- **One look:** palette, font and caption position are decided once (in the first player) so clips stitch together. Each clip also gets a still thumbnail.
- **Scenario registry:** one scenario name gives one command gives one trace, so any clip can be remade.
- **Bloopers:** each milestone keeps a short clip of honest solver failures. They are cheap and entertaining.

## How media is made before there is a renderer (milestone 1)

1. The core gets a small headless example that runs a named scenario and writes a **trace**: node positions (and per-beam stress) for every step, as plain JSON, into `artifacts/traces/`.
2. A static HTML page in `demo/` plays any trace on a 2D canvas (a side or top view of the 3D data), with a scrubber, a speed control and an overlay for the numbers. No framework, no build step beyond what already exists.
3. The page is recorded to an MP4 with the narrated-capture tooling the owner already uses, or just a plain screen capture.
4. Later (milestone 2) the same page gains the real wasm core and Three.js, and the scenarios become live instead of replayed. The trace player stays as the way to replay recorded runs, which milestone 2 needs anyway.

This costs one small example binary and one HTML page, shared by every milestone 1 demo. It is plan step 1.2b, limited to half a session. The trace writer is an example (or feature-gated module) that only reads world state, so `core/` and determinism are untouched.

### Trace format

- **Header (versioned):** format version, scenario name, core commit, timestep, substep rule, `Real` precision, seed, units, node count, beam list (node pairs, rest length, yield and break limits).
- **Per step:** step index, node positions, per-beam strain or stress, state hash, substeps used, total energy. These cover what the hash covers, so a mismatch can be located by eye.
- **Sparse per step:** events (break, yield, freeze, with ids) and commands (the inputs applied). With commands included, the trace is also the milestone 2 replay file.
- **Topology changes are events,** never re-indexing, so beam ids stay stable after breaks.
- **Size:** JSON for milestone 1, with a keep-every-Nth-step option. A binary body under a JSON header comes later; the player reads through one loader function so the body can change.

## Milestone 1 slices

| Phase | Demo | The moment | The number on screen |
| --- | --- | --- | --- |
| 1.1 layout | Skip | | |
| 1.2 step and hash | Two identical runs side by side, hashes ticking in lockstep. Then nudge one node by a billionth: falling points change colour the moment the runs diverge | The split | Hash per step |
| 1.3 solver | A truss square with and without its diagonal brace (one folds, one holds), and the same scene at 1, 4 and 16 substeps side by side, mushy to stiff | The brace holding | Substeps chosen |
| 1.4 cantilever | The truss sags under its load next to the textbook prediction. Then a chart: error against substep count | The two tips meeting, with a ghost outline of the textbook curve | Measured against textbook tip deflection, percent error, tolerance stated even if disappointing |
| 1.5 yield and break | Same load, three outcomes: springs back, stays bent, snaps. Beams coloured by stress, with a live stress-strain plot beside the beam and slow motion on the snap | The snap, and the event log line | Strain per beam |
| 1.6 guards | Inject a bad number into a bridge: a real unguarded build flag blows up, ours freezes and reports (no faked animation; drop the comparison if the flag is not built). Plus the ten-minute energy trace, flat | The freeze, with the blown-up version beside it | Total energy over ten minutes |
| 1.7 benchmark | A 10,000-beam structure collapsing in replay (labelled as a replay), with the benchmark table beside it | The collapse | Beams, substeps, ms per step |
| Milestone wrap | One 3 to 5 minute video stitching the best clip from each phase | | |

## Later milestones (first guess, revised when each starts)

| Milestone | Demo story | Notes |
| --- | --- | --- |
| 2 bridge builder | Build a bridge in the browser, roll a load across it, then under-build one and watch it fail. Replay it exactly, as a ghost overlay on a fresh run, perfectly coincident | First real video; the shareable one |
| 3 3D on terrain | A tower and a bridge collapse on terrain from two camera angles, the same collapse twice with identical debris | Camera work matters here |
| 4 parts and assembly | Assemble a rocket from parts, launch, then the hard landing that snaps a leg, with a slow-motion stress view of the snap | The "test of fun" is the clip |
| 5 rigid by default | The same tall rocket, all-live against rigid-by-default, wobble side by side, with the frame-time saving | A comparison clip |
| 6 aerodynamics | A plane flexes in a tight turn and loses a wing | |
| 7 ground and water | Rover leaving ruts, a crater, a boat on waves and a hull flooding | |
| 8 collision and threads | Two craft collide; the same run on one thread and on many, hashes equal | Determinism as a headline |
| 9 visual layer | Seam tearing, debris and splashes | Judged by eye, so it is mostly media |

## Open questions

- Host for live demos from milestone 2: a plain Three.js page in this repo (recommended in review: the repo is public and must not depend on a private workspace). A separate Angular harness could later be a second consumer of the wasm API, not the host. Confirm when milestone 2 starts.
- Where finished clips are shared, if anywhere, is the owner's call. Nothing is published from the repo.
