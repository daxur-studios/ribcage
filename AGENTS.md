# ribcage

Repository instructions for AI coding agents.

<!-- ae-managed:start id="ae-standards" sha256="3b4faa5773e78bf17c7f90ee44ee1af4bffc857afb370304acdcdf9d5251b13b" -->
## Æ workspace conventions

- **CLI:** Use `ae` (not legacy `iae`) for worktrees (`ae wt`), environments (`ae env`), and repo checks (`ae repo check`).
- **Plans & Engineering Records:** Pair active plans in `docs/plans/NNN_slug.md` with append-only Engineering Records in `docs/engineering-records/NNN_slug.md` (`docs/runbook/` is legacy only).
- **Git safety:** Multiple agents may share this tree. Never run `git stash`, `git reset`, `git clean`, or `git push --force`; commit only files you edited.
- **Public repository:** everything committed is world-readable, including docs, plans, engineering records, commit messages and history. Never commit personal details, local machine paths, emails, tokens, keys or session transcripts. Never name private repositories, products or their paths; use generic wording such as "a consuming game". Regenerable output (screenshots, renders, benchmark dumps) stays local in a gitignored `artifacts/` folder.
<!-- ae-managed:end -->

## This repo

Real-time node-beam physics engine for a KSP-like building game: Rust core (`core/`, crate `beam-core`) compiled to WebAssembly, with a TypeScript host (`src/`) that Three.js reads. The design is [docs/research/node-beam-engine-design-spec.md](docs/research/node-beam-engine-design-spec.md); the work order is [Plan 001](docs/plans/001_bootstrap.md).

- **Tests:** `cargo test` (Rust core, headless) and `pnpm test` / `pnpm typecheck` (TypeScript).
- **Rules from the spec:** deterministic by default (fixed iteration order, no frame-time dependence, own sin/cos); no rendering or browser code in `core/`; build the debug tools and headless tests before any gameplay feature.
