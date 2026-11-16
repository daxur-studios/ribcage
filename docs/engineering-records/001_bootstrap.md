---
type: EngineeringRecord
title: Bootstrap ribcage
description: Initial scaffold provenance and verification log for ribcage.
timestamp: 2026-11-16T19:05:47.257Z
plans: [/docs/plans/001_bootstrap.md]
tags: [bootstrap, setup]
---

# ER 001: Bootstrap ribcage

## 2026-11-16

- **[action]** Created ribcage with `ae repo new` and generated initial `ae.repo.json` alignment.
- **[action]** Added Cargo workspace with `core/` (`beam-core`), and saved the design spec under `docs/research/`.
- **[decision]** Used the `node-ts` profile for the TypeScript host; the Rust core is added by hand because `ae repo new` has no Rust profile yet.
- **[verification]** `cargo test` passes (1 test).
