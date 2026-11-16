---
type: Plan
title: Bootstrap ribcage
description: Repository setup and the milestone order from the design spec.
timestamp: 2026-11-16T19:05:47.257Z
status: active
record: /docs/engineering-records/001_bootstrap.md
tags: [bootstrap, setup]
---

# Plan 001: Bootstrap ribcage

Evidence: [ER 001](/docs/engineering-records/001_bootstrap.md). Design: [spec](/docs/research/node-beam-engine-design-spec.md).

## Phases

- [x] Scaffold the repo via `ae repo new` and register the Æ workspace.
- [x] Add the Rust core crate (`core/`) and the design spec.

### Milestone 1: solver core and tools

All in `core/`, headless, no rendering or browser code. The [core layout rules](/docs/research/core-data-layout.md) are binding: read them before phase 1.1. Each phase ends with passing tests and ticks the matching check in the [roadmap](/ROADMAP.md).

- [x] 1.1 Data layout: 3D nodes and beams in flat arrays, the packed island buffer, and one passing test.
- [ ] 1.2 Fixed 60 Hz step with gravity, a state hash, and the "same inputs, same hash" test.
- [ ] 1.3 XPBD beam solve with damping and a rule-based substep count.
- [ ] 1.4 Cantilever test against the textbook deflection. Riskiest phase: the tolerance is measured here.
- [ ] 1.5 Yield and break with break events, and their two tests.
- [ ] 1.6 Stability guards: movement clamps, bad-number freeze, and the ten-minute no-energy-gain test.
- [ ] 1.7 Own sine and cosine, and the benchmark scene (beams, substeps, ms per step).

Milestones 2-9 are in the [roadmap](/ROADMAP.md). Each gets its own plan and record when it starts.

## Open questions

- [x] Is the core 3D from the start, or generic over dimension? Decided: 3D from the start.
- [ ] Move "rigid body with anchor nodes" from milestone 5 to the start of milestone 4? Recommended yes; see the [rigid-by-default brief](/docs/research/rigid-by-default-design-brief.md). Does not block milestone 1.

The seven wider open questions are at the end of the spec. None blocks milestone 1.
