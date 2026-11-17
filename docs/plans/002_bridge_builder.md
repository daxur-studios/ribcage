---
type: Plan
title: 2D bridge builder
description: Milestone 2 - the core in the browser as WebAssembly, and a small bridge editor on top of it.
timestamp: 2026-11-17T09:00:00.000Z
status: active
record: /docs/engineering-records/002_bridge_builder.md
tags: [milestone-2, wasm, editor]
---

# Plan 002: 2D bridge builder

Evidence: [ER 002](/docs/engineering-records/002_bridge_builder.md). Design: [spec](/docs/research/node-beam-engine-design-spec.md), milestone 2. Layout rules still binding: [core layout](/docs/research/core-data-layout.md), rule 8 (buffers at the boundary) above all.

**Scope [assumption]:** the bridge builder is a **test bed** for the engine, not a shippable mini-game, until the owner says otherwise. It gets the debug tools (stress colours, time controls, record and replay) and no menus, levels, budgets or scoring. If it becomes a mini-game, that is a new check in this milestone, not a rewrite.

**Planar scenes.** The core is 3D (Plan 001). The editor keeps every node at `z = 0` and every force in the plane, which keeps `z` exactly zero (checked in 2.2).

## Phases

Each phase ends with passing tests, ticks the matching check in the [roadmap](/ROADMAP.md) and logs a line in the record.

- [x] 2.1 The core as WebAssembly: a C-style boundary (`core/src/ffi.rs`), a `wasm32` build, a loader in `src/`, and a Node test showing the benchmark scene gives the same state hash as the native build. Also the first WebAssembly timing. (Proven in Node; the roadmap check "in the browser" is ticked in 2.3, when a page loads it.)
- [ ] 2.2 The host world in TypeScript: typed views that survive memory growth, the command buffer in (rule 8), events and render positions out, and a headless bridge scene with a load that rolls across. A strong bridge holds it and an under-built bridge breaks.
- [ ] 2.3 Editor page: place nodes and beams with the mouse, run, pause, single-step and slow down time; beams coloured by stress. Browser build step chosen here (no bundler is installed yet).
- [ ] 2.4 Record and replay: the command lists per step are saved, and a replay gives the same state hash, in Node and in the browser.

## Open questions

- [ ] Mini-game or test bed? Assumed test bed above; the owner can reverse it at any time.
- [ ] Does the editor need undo? Not planned; decide in 2.3.
