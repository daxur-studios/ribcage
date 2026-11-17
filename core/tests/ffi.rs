//! The C-style boundary, called natively. The same functions are exported from the WebAssembly
//! build; `test/wasm.test.ts` checks the same golden hashes there, which is what shows the two
//! builds give the same bits.

use beam_core::bench::{bench_scene, BenchSpec};
use beam_core::event::{Event, EVENT_BREAK};
use beam_core::ffi::*;
use beam_core::real::Vec3;
use beam_core::sim::{state_hash, step};
use beam_core::world::{BeamSpec, World};

/// State hash of the small benchmark scene after 120 steps. The TypeScript test holds the same
/// number; if the solver changes on purpose, both are re-recorded together.
pub const SMALL_BENCH_120: u64 = 0x935259e4d2c6ff87;

/// State hash of the hand-built scene in `hand_built_scene_matches_direct_world` after 30 steps.
pub const HAND_BUILT_30: u64 = 0xb62c7e7dd415f3a1;

/// The same scene built through plain function calls and through the boundary agrees.
#[test]
fn bench_through_the_boundary_matches_the_direct_run() {
    let mut s = bench_scene(BenchSpec::SMALL);
    for _ in 0..120 {
        s.apply_load();
        step(&mut s.world);
    }
    let direct = state_hash(&s.world);

    unsafe {
        let h = rc_bench_new(0);
        for _ in 0..120 {
            rc_bench_step(h);
        }
        assert_eq!(rc_step_count(h), 120);
        assert_eq!(rc_state_hash(h), direct);
        assert_eq!(rc_node_count(h), BenchSpec::SMALL.node_count());
        assert_eq!(rc_beam_count(h), BenchSpec::SMALL.beam_count());
        rc_host_free(h);
    }
    assert_eq!(direct, SMALL_BENCH_120, "golden: {direct:#018x}");
}

/// Building by hand through the boundary gives the same world as building it directly.
#[test]
fn hand_built_scene_matches_direct_world() {
    let mut w = World::new();
    let spec = BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.5, yield_strain: 0.01, break_strain: 0.05 };
    let a = w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    let b = w.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
    w.add_beam(a.index, b.index, spec);
    for _ in 0..30 {
        step(&mut w);
    }

    unsafe {
        let h = rc_host_new();
        let na = rc_add_node(h, 0.0, 0.0, 0.0, 0.0, 0.1);
        let nb = rc_add_node(h, 1.0, 0.0, 0.0, 1.0, 0.1);
        let beam = rc_add_beam(h, na, nb, 1.0, 1e-6, 0.5, 0.01, 0.05);
        assert_eq!((na, nb, beam), (0, 1, 0));
        for _ in 0..30 {
            rc_step(h);
        }
        assert_eq!(rc_state_hash(h), state_hash(&w));
        assert_eq!(rc_state_hash(h), HAND_BUILT_30, "golden: {:#018x}", rc_state_hash(h));
        // The raw position array is the world's: node 1 hangs straight down from its start.
        let pos = std::slice::from_raw_parts(rc_pos_ptr(h), rc_node_count(h) as usize);
        assert_eq!(pos[1], w.pos[1]);
        rc_host_free(h);
    }
}

/// The render buffer holds the positions rounded to 32 bits, three per node.
#[test]
fn render_buffer_is_positions_as_f32() {
    unsafe {
        let h = rc_host_new();
        rc_add_node(h, 0.1, 2.5, 0.0, 0.0, 0.1);
        rc_add_node(h, -3.0, 4.25, 1.0, 1.0, 0.1);
        let p = rc_write_render(h);
        let v = std::slice::from_raw_parts(p, 6);
        assert_eq!(v, &[0.1f64 as f32, 2.5, 0.0, -3.0, 4.25, 1.0]);
        rc_host_free(h);
    }
}

/// An overstretched beam breaks; the event is readable in place through the boundary.
#[test]
fn events_are_readable_in_place() {
    unsafe {
        let h = rc_host_new();
        let a = rc_add_node(h, 0.0, 0.0, 0.0, 0.0, 0.1);
        let b = rc_add_node(h, 1.5, 0.0, 0.0, 1.0, 0.1);
        // Rest length 1 and a node at 1.5: 50% strain, far past the 5% break strain.
        rc_add_beam(h, a, b, 1.0, 1e-6, 0.5, 0.01, 0.05);
        rc_step(h);
        assert_eq!(rc_event_count(h), 1);
        let e: Event = *rc_events_ptr(h);
        assert_eq!((e.kind, e.index, e.step), (EVENT_BREAK, 0, 1));
        assert_eq!(std::mem::size_of::<Event>(), 24);
        rc_host_free(h);
    }
}

#[test]
fn abi_version_is_exported() {
    assert_eq!(rc_abi_version(), ABI_VERSION);
}
