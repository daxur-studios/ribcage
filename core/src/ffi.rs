//! The boundary (layout rule 8) as a C-style interface: plain integers and floats in, raw
//! pointers into the core's own arrays out. The WebAssembly build exports these functions and the
//! TypeScript host (`src/core.ts`) calls them; native tests call the same functions, so the
//! boundary is exercised without a browser. No `wasm-bindgen`: fewer moving parts, no generated
//! glue to keep deterministic, and the host reads output arrays in place.
//!
//! Rules of the interface:
//! - A [`Host`] owns one world (no global state). `rc_host_new` makes it, `rc_host_free` ends it.
//! - Indices are the world's stable `u32` indices. Integers are `u32` and `u64` only.
//! - Every pointer returned points into the host and stays valid until the next call that
//!   changes the thing it points at (a step, an add, `rc_write_render`). WebAssembly memory can
//!   also move when it grows, so the host re-reads its memory view after every call that may
//!   allocate and never keeps a view across one.
//! - A bad argument (a beam between the same node twice, a dead node) traps, as a panic does.
//!   The editor checks first; the trap is only the last line.
//!
//! **[assumption]** Output positions are `f32` relative to the world origin, which is also the
//! floating origin until a scene is large enough to need one (milestone 3 or later).

use crate::bench::{apply_gust, bench_scene, BenchSpec};
use crate::command::{apply_commands, Command, COMMAND_BYTES};
use crate::event::Event;
use crate::real::{Real, Vec3};
use crate::sim::{state_hash, step};
use crate::world::{BeamSpec, World};

/// Bumped by any change to an exported function's signature, to a record layout or to the set of
/// command kinds. 3 added the terrain commands (kinds 7 and 8); 4 added the thruster commands
/// (kinds 9 and 10) and puts thrusters in the state hash (only when there are any). Nothing else
/// changed, so a recording made at ABI 2 or 3 still means the same thing and replays
/// (`src/recording.ts`).
pub const ABI_VERSION: u32 = 4;

/// Bytes per command record, for the host to check against its own layout.
#[no_mangle]
pub extern "C" fn rc_command_bytes() -> u32 {
    COMMAND_BYTES as u32
}

/// One world plus the buffers handed out to the host.
pub struct Host {
    world: World,
    /// Node positions as `f32`, three per node, filled by [`rc_write_render`].
    render: Vec<f32>,
    /// Command records the host writes into, applied by [`rc_apply_commands`].
    commands: Vec<Command>,
    results: Vec<u32>,
    /// Beam ends (two per beam slot) and strain ratios, filled by [`rc_write_beams`].
    beam_ends: Vec<u32>,
    beam_ratio: Vec<f32>,
    /// Nodes that [`rc_bench_step`] pushes (empty for an ordinary host).
    load_nodes: Vec<u32>,
}

impl Host {
    fn with(world: World, load_nodes: Vec<u32>) -> Host {
        Host {
            world,
            render: Vec::new(),
            commands: Vec::new(),
            results: Vec::new(),
            beam_ends: Vec::new(),
            beam_ratio: Vec::new(),
            load_nodes,
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }
}

#[no_mangle]
pub extern "C" fn rc_abi_version() -> u32 {
    ABI_VERSION
}

/// A new empty host. The pointer is an integer to JavaScript.
#[no_mangle]
pub extern "C" fn rc_host_new() -> *mut Host {
    Box::into_raw(Box::new(Host::with(World::new(), Vec::new())))
}

/// A host holding the benchmark scene (`standard` non-zero: the reported scene; zero: the small
/// test scene), stepped with [`rc_bench_step`].
#[no_mangle]
pub extern "C" fn rc_bench_new(standard: u32) -> *mut Host {
    let spec = if standard != 0 { BenchSpec::STANDARD } else { BenchSpec::SMALL };
    let (world, load_nodes) = bench_scene(spec).into_parts();
    Box::into_raw(Box::new(Host::with(world, load_nodes)))
}

/// # Safety
/// `h` came from `rc_host_new` or `rc_bench_new` and is not used again.
#[no_mangle]
pub unsafe extern "C" fn rc_host_free(h: *mut Host) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// Adds a node and returns its index. `inv_mass` of zero pins it.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_add_node(h: *mut Host, x: Real, y: Real, z: Real, inv_mass: Real, radius: Real) -> u32 {
    (*h).world.add_node(Vec3::new(x, y, z), inv_mass, radius).index
}

/// Adds a beam between two live, different nodes and returns its index.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_add_beam(
    h: *mut Host,
    a: u32,
    b: u32,
    rest: Real,
    compliance: Real,
    damping: Real,
    yield_strain: Real,
    break_strain: Real,
) -> u32 {
    let spec = BeamSpec { rest, compliance, damping, yield_strain, break_strain };
    (*h).world.add_beam(a, b, spec).index
}

/// Adds an external force to a node for the coming step (rule 7).
///
/// # Safety
/// `h` is a live host pointer and `node` an index below `rc_node_count`.
#[no_mangle]
pub unsafe extern "C" fn rc_add_force(h: *mut Host, node: u32, fx: Real, fy: Real, fz: Real) {
    (*h).world.add_force(node, Vec3::new(fx, fy, fz));
}

/// One fixed 60 Hz step with the default substep rule. Returns the largest substep count used.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_step(h: *mut Host) -> u32 {
    step(&mut (*h).world)
}

/// The benchmark scene's gust, then one step. Same as `rc_step` for a host with no load nodes.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_bench_step(h: *mut Host) -> u32 {
    let host = &mut *h;
    apply_gust(&mut host.world, &host.load_nodes);
    step(&mut host.world)
}

/// The state hash (FNV-1a over the raw bits, see `sim::state_hash`), as a 64-bit integer.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_state_hash(h: *const Host) -> u64 {
    state_hash(&(*h).world)
}

/// Steps taken so far.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_step_count(h: *const Host) -> u64 {
    (*h).world.step_count
}

/// Node slots, live or dead.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_node_count(h: *const Host) -> u32 {
    (*h).world.node_count()
}

/// Beam slots, live, broken or dead.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_beam_count(h: *const Host) -> u32 {
    (*h).world.beam_count()
}

/// Node positions at full precision: `rc_node_count` triples of `Real`. For tests and the
/// recorder; rendering reads [`rc_write_render`] instead.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_pos_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.pos.as_ptr()
}

/// Events of the last step: `rc_event_count` records of 24 bytes (`u32 kind`, `u32 index`,
/// `u64 step`, `f64 value`, see `event::Event`). Valid until the next step.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_event_count(h: *const Host) -> u32 {
    (*h).world.events.len() as u32
}

/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_events_ptr(h: *const Host) -> *const Event {
    (*h).world.events.as_ptr()
}

/// Makes room for `count` command records (see `command::Command`, [`COMMAND_BYTES`] each) and
/// returns a pointer to the first. The host writes the records there, then calls
/// [`rc_apply_commands`]. The records are zeroed (kind 0, always rejected) until written. Memory
/// may move on this call, so the host re-reads its view afterwards.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_commands_reserve(h: *mut Host, count: u32) -> *mut Command {
    let host = &mut *h;
    host.commands.clear();
    host.commands.resize(count as usize, Command::default());
    host.commands.as_mut_ptr()
}

/// Applies the `count` records last reserved, in order (layout rule 8), and returns how many were
/// rejected. Per-record results are at [`rc_results_ptr`].
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_apply_commands(h: *mut Host) -> u32 {
    let host = &mut *h;
    apply_commands(&mut host.world, &host.commands, &mut host.results)
}

/// One `u32` per applied record: the new index for an add, 1 for another accepted command,
/// `u32::MAX` for a rejected one. Valid until the next apply.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_results_ptr(h: *const Host) -> *const u32 {
    (*h).results.as_ptr()
}

/// Node flags, one `u32` per node slot (`world::NODE_*`).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_node_flags_ptr(h: *const Host) -> *const u32 {
    (*h).world.node_flags.as_ptr()
}

/// Beam flags, one `u32` per beam slot (`world::BEAM_*`: alive, broken, yielded).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_beam_flags_ptr(h: *const Host) -> *const u32 {
    (*h).world.beam_flags.as_ptr()
}

/// Fills the beam buffers, one entry per beam slot, and returns the ends: two `u32` per beam
/// (node indices). Strain ratios are then at [`rc_beam_ratio_ptr`]. Dead and broken beams read
/// zero. Call once per rendered frame, after the step.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_write_beams(h: *mut Host) -> *const u32 {
    let host = &mut *h;
    let w = &host.world;
    host.beam_ends.clear();
    host.beam_ratio.clear();
    for k in 0..w.beam_count() as usize {
        host.beam_ends.push(w.beam_a[k]);
        host.beam_ends.push(w.beam_b[k]);
        let live = w.beam_alive(k as u32) && w.rest0[k] != 0.0;
        let ratio = if live {
            let len = (w.pos[w.beam_b[k] as usize] - w.pos[w.beam_a[k] as usize]).length();
            let total = (len - w.rest0[k]) / w.rest0[k];
            let unit = if w.yield_strain[k] > 0.0 { w.yield_strain[k] } else { w.break_strain[k] };
            total / unit
        } else {
            0.0
        };
        host.beam_ratio.push(ratio as f32);
    }
    host.beam_ends.as_ptr()
}

/// One `f32` per beam slot, filled by [`rc_write_beams`]: the beam's signed total strain
/// `(length - rest0) / rest0` in units of its yield strain (1 is at yield; negative is
/// compression), or in units of its break strain when it has no yield strain.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_beam_ratio_ptr(h: *const Host) -> *const f32 {
    (*h).beam_ratio.as_ptr()
}

/// Fills the render buffer with node positions as `f32` triples (one per node slot) and returns
/// a pointer to it. Call once per rendered frame, not per step.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_write_render(h: *mut Host) -> *const f32 {
    let host = &mut *h;
    host.render.clear();
    for p in &host.world.pos {
        host.render.push(p.x as f32);
        host.render.push(p.y as f32);
        host.render.push(p.z as f32);
    }
    host.render.as_ptr()
}
