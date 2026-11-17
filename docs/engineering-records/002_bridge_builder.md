---
type: EngineeringRecord
title: 2D bridge builder
description: Decisions, findings and verification evidence for milestone 2.
timestamp: 2026-11-17T09:00:00.000Z
plans: [/docs/plans/002_bridge_builder.md]
tags: [milestone-2, wasm, editor]
---

# ER 002: 2D bridge builder

## 2026-11-17

- **[action]** Milestone 2 started. Plan 002 written from the roadmap checks: 2.1 WebAssembly build and loader, 2.2 host world and bridge scene, 2.3 editor page, 2.4 record and replay. Milestone 1's record (ER 001) is closed at phase 1.7.
- **[assumption]** The bridge builder is a test bed, not a shippable mini-game (the open question from the spec, due at the start of this milestone). The owner said "proceed" to a recommendation of test bed, so work follows it; nothing here blocks switching later.
- **[decision]** Planning and building this milestone are both Sonnet's, as the roadmap table says. No Opus step is nominated.

## 2026-11-17 (phase 2.1)

- **[action]** Phase 2.1 done. `core/src/ffi.rs` is the boundary as a C-style interface (`rc_*` functions: host new and free, add node and beam, add force, step, state hash, node and beam counts, positions, events, render buffer, and the benchmark scene). `src/core.ts` loads the module from its bytes (Node and browser alike) and wraps the exports as typed methods; `test/wasm.test.ts` (7 tests, 1 opt-in) and `core/tests/ffi.rs` (5 tests) exercise the same functions natively and in WebAssembly. `pnpm test` now builds the wasm first (`pnpm build:wasm`, a release build for `wasm32-unknown-unknown`); `RIBCAGE_SLOW=1 pnpm test` adds the reported scene. `cargo test`, `pnpm test` and `pnpm typecheck` pass. `Vec3` is now `repr(C)`, so positions can be read in place.
- **[decision]** Raw C-style exports, no `wasm-bindgen` or `wasm-pack`. The module has no imports (a test checks it), is 63 KB, and has no generated glue to keep deterministic; the host reads output arrays straight out of linear memory. The cost is one hand-kept signature list (`ABI_VERSION`, checked at load) instead of generated bindings, fine for a boundary this small. Revisit only if the boundary grows past what a list can hold.
- **[decision]** A `Host` owns one world and is passed as a pointer: no global state (determinism list), several worlds possible. Positions out are `f32` triples written by `rc_write_render` once per rendered frame (layout rule 8), plus the full-precision array for tests and the recorder. The floating origin is the world origin until a scene needs one. **[assumption]**
- **[decision]** Hashes and counts cross the boundary as 64-bit integers, which JavaScript receives as signed `BigInt`; the loader converts with `BigInt.asUintN(64, ...)` so hashes print the same as the native `{:016x}`. (First test run failed on exactly this.)
- **[finding]** **Native and WebAssembly agree bit for bit.** The small benchmark scene after 120 steps (`0x935259e4d2c6ff87`), a hand-built hanging beam after 30 steps (`0xb62c7e7dd415f3a1`) and the reported 11,136-beam scene after 720 steps (`0x871092299f3cea56`, the end hash from ER 001, phase 1.7) give identical hashes natively and in Node's WebAssembly. That covers the own sin/cos (the scene is built with them), the solver, damping, and the yield and break code; the first check that milestone 1's determinism list holds across the two builds. The same goldens are written in both test files.
- **[finding]** **First WebAssembly timing**, reported scene, 17 substeps, 600 steps after 120 warm-up, one run in Node v22 on the same noisy machine: mean 8.8 ms, median 7.4 ms, p95 12.8 ms per step. The native release build, run just before: mean 6.4 ms (33 ns per beam solve). So WebAssembly in Node costs roughly 1.2 to 1.4 times native here. One run each; read it as "close to native", not as a figure. A browser's engine is not measured yet (2.3).
- **[finding]** `pnpm typecheck` rejects a Node `Buffer` as `BufferSource` (its buffer may be shared); the test copies the file into a plain `Uint8Array`. Browser code passes an `ArrayBuffer` from `fetch` and is not affected.
- **[question]** The roadmap check "the core runs in the browser as WebAssembly" is not ticked yet: it is proven in Node, and a browser needs the TypeScript compiled and served. Phase 2.3 chooses how (plain `tsc` emit with no bundler is the leading option: no new dependency). Not blocking 2.2.
