---
type: Plan
title: Bootstrap node-beam-physics
description: Repository setup and the milestone order from the design spec.
timestamp: 2026-11-16T19:05:47.257Z
status: active
record: /docs/engineering-records/001_bootstrap.md
tags: [bootstrap, setup]
---

# Plan 001: Bootstrap node-beam-physics

Evidence: [ER 001](/docs/engineering-records/001_bootstrap.md). Design: [spec](/docs/research/node-beam-engine-design-spec.md).

## Phases

- [x] Scaffold the repo via `ae repo new` and register the Æ workspace.
- [x] Add the Rust core crate (`core/`) and the design spec.
- [ ] Milestone 1: solver core and tools (nodes, beams, XPBD, fixed step, yield and break, headless tests, benchmark).
- [ ] Milestone 2: 2D bridge builder with stress heat-map.
- [ ] Milestone 3: 3D port rendered in Three.js.
- [ ] Milestone 4: parts and assembly. First vertical slice ends here: a hard landing snaps a leg and the craft tips over.
- [ ] Later milestones 5-9 as listed in the spec.

## Open questions

The seven open questions are at the end of the spec. None blocks milestone 1.
