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

## How media is made before there is a renderer (milestone 1)

1. The core gets a small headless example that runs a named scenario and writes a **trace**: node positions (and per-beam stress) for every step, as plain JSON, into `artifacts/traces/`.
2. A static HTML page in `demo/` plays any trace on a 2D canvas (a side or top view of the 3D data), with a scrubber, a speed control and an overlay for the numbers. No framework, no build step beyond what already exists.
3. The page is recorded to an MP4 with the narrated-capture tooling the owner already uses, or just a plain screen capture.
4. Later (milestone 2) the same page gains the real wasm core and Three.js, and the scenarios become live instead of replayed. The trace player stays as the way to replay recorded runs, which milestone 2 needs anyway.

This costs one small example binary and one HTML page, shared by every milestone 1 demo.

## Milestone 1 slices

| Phase | Demo | The moment | The number on screen |
| --- | --- | --- | --- |
| 1.1 layout | Skip, or a still: the packed island buffer drawn as a table | Optional | Nodes, beams, bytes |
| 1.2 step and hash | Two identical runs side by side, hashes ticking in lockstep. Then nudge one node by a billionth and watch the hashes split | The split | Hash per step |
| 1.3 solver | A hanging chain and a swinging beam: it swings like rope, loses energy only through damping | The swing settling | Substeps chosen |
| 1.4 cantilever | The truss sags under its load next to the textbook prediction. Then a chart: error against substep count | The two tips meeting | Measured against textbook tip deflection, percent error |
| 1.5 yield and break | Same load, three outcomes: springs back, stays bent, snaps. Beams coloured by stress | The snap, and the event log line | Strain per beam |
| 1.6 guards | Inject a bad number into a bridge: a naive solver blows up, ours freezes and reports. Plus the ten-minute energy trace, flat | The freeze, with the blown-up version beside it | Total energy over ten minutes |
| 1.7 benchmark | Scale up: 100, 1,000, 10,000 beams stepping live, with ms per step | The frame-time number climbing | Beams, substeps, ms per step |
| Milestone wrap | One 3 to 5 minute video stitching the best clip from each phase | | |

## Later milestones (first guess, revised when each starts)

| Milestone | Demo story | Notes |
| --- | --- | --- |
| 2 bridge builder | Build a bridge in the browser, roll a load across it, then under-build one and watch it fail. Replay it exactly | First real video; the shareable one |
| 3 3D on terrain | A tower and a bridge collapse on terrain from two camera angles | Camera work matters here |
| 4 parts and assembly | Assemble a rocket from parts, launch, then the hard landing that snaps a leg | The "test of fun" is the clip |
| 5 rigid by default | The same tall rocket, all-live against rigid-by-default, wobble side by side, with the frame-time saving | A comparison clip |
| 6 aerodynamics | A plane flexes in a tight turn and loses a wing | |
| 7 ground and water | Rover leaving ruts, a crater, a boat on waves and a hull flooding | |
| 8 collision and threads | Two craft collide; the same run on one thread and on many, hashes equal | Determinism as a headline |
| 9 visual layer | Seam tearing, debris and splashes | Judged by eye, so it is mostly media |

## Open questions

- Which host runs the live demo from milestone 2: a plain Three.js page, or the owner's existing test harness? Decide when milestone 2 starts; the milestone 1 trace player does not depend on the answer.
- Where finished clips are shared, if anywhere, is the owner's call. Nothing is published from the repo.
