//! The C-style boundary, called natively. The same functions are exported from the WebAssembly
//! build; `test/wasm.test.ts` checks the same golden hashes there, which is what shows the two
//! builds give the same bits.

use beam_core::bench::{bench_scene, BenchSpec};
use beam_core::command::{
    apply_commands, Command, CMD_ADD_BEAM, CMD_ADD_NODE, CMD_ADD_THRUSTER, CMD_SET_TERRAIN, CMD_SET_THROTTLE, CMD_TERRAIN_HEIGHTS,
};
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

/// A terrain-only world: 5 by 4 points, 2 m apart from (-4, 1), friction 0.8, heights
/// `(k % 7) * 0.5` for grid index k. The same records are built in `test/wasm.test.ts`.
pub const TERRAIN_ONLY: u64 = 0x424288d9b2e8abb4;

fn terrain_commands() -> Vec<Command> {
    let rec = |kind, a, b, f| Command { kind, a, b, reserved: 0, f };
    let mut cmds = vec![rec(CMD_SET_TERRAIN, 5, 4, [-4.0, 1.0, 2.0, 0.8, 0.0, 0.0])];
    let all: Vec<f64> = (0..20).map(|k| (k % 7) as f64 * 0.5).collect();
    for (n, chunk) in all.chunks(6).enumerate() {
        let mut f = [0.0; 6];
        f[..chunk.len()].copy_from_slice(chunk);
        cmds.push(rec(CMD_TERRAIN_HEIGHTS, n as u32 * 6, chunk.len() as u32, f));
    }
    cmds
}

/// Terrain sent through the boundary hashes the same as terrain applied directly.
#[test]
fn terrain_through_the_boundary_matches_the_direct_world() {
    let cmds = terrain_commands();
    let mut w = World::new();
    let mut results = Vec::new();
    apply_commands(&mut w, &cmds, &mut results);
    assert!(results.iter().all(|&r| r == 1));
    step(&mut w);
    let direct = state_hash(&w);

    unsafe {
        let h = rc_host_new();
        let dst = rc_commands_reserve(h, cmds.len() as u32);
        std::ptr::copy_nonoverlapping(cmds.as_ptr(), dst, cmds.len());
        assert_eq!(rc_apply_commands(h), 0, "no record rejected");
        rc_step(h);
        assert_eq!(rc_state_hash(h), direct);
        assert_eq!(rc_state_hash(h), TERRAIN_ONLY, "golden: {:#018x}", rc_state_hash(h));
        rc_host_free(h);
    }
}

/// The terrain of [`TERRAIN_ONLY`] with things on it, after 120 steps: a triangle of three
/// 10 kg nodes dropped onto the bumps, a node spawned under a slope, and a node spawned under
/// the ground past the grid's corner (the flat edge rule). Covers landing, friction on a slope
/// and recovery from overlap. The same records are built in `test/wasm.test.ts`.
/// Re-recorded in phase 3.3, when the triangle's beams gained midpoint contacts (it was
/// `0x05e2e896a698c2f5` with node contacts alone).
pub const TERRAIN_SCENE_120: u64 = 0x2bde_f035_f12d_0c86;

fn terrain_scene_commands() -> Vec<Command> {
    let rec = |kind, a, b, f| Command { kind, a, b, reserved: 0, f };
    let mut cmds = terrain_commands();
    for [x, y, z, inv_mass, radius] in
        [[-1.0, 4.0, 3.0, 0.1, 0.2], [0.0, 4.0, 3.5, 0.1, 0.2], [-0.5, 4.8, 4.0, 0.1, 0.2], [3.5, 2.0, 6.5, 1.0, 0.1], [6.0, 1.0, 0.0, 1.0, 0.1]]
    {
        cmds.push(rec(CMD_ADD_NODE, 0, 0, [x, y, z, inv_mass, radius, 0.0]));
    }
    for (a, b) in [(0, 1), (1, 2), (2, 0)] {
        cmds.push(rec(CMD_ADD_BEAM, a, b, [1.1, 1e-6, 50.0, 0.5, 1.0, 0.0]));
    }
    cmds
}

/// Ground contact gives the same bits through the boundary as applied directly, and matches
/// the golden that the WebAssembly build is held to.
#[test]
fn terrain_scene_through_the_boundary_matches_the_direct_world() {
    let cmds = terrain_scene_commands();
    let mut w = World::new();
    let mut results = Vec::new();
    apply_commands(&mut w, &cmds, &mut results);
    assert!(!results.contains(&u32::MAX), "a record was rejected: {results:?}");
    for _ in 0..120 {
        step(&mut w);
    }
    let direct = state_hash(&w);
    // Everything ended on the ground: no node below its radius by more than a millimetre.
    let t = w.terrain.as_ref().unwrap();
    for i in 0..w.node_count() as usize {
        let gap = t.plane_at(w.pos[i].x, w.pos[i].z).distance(w.pos[i]) - w.radius[i];
        assert!(gap > -1e-3 && gap < 0.05, "node {i}: gap {gap}");
    }

    unsafe {
        let h = rc_host_new();
        let dst = rc_commands_reserve(h, cmds.len() as u32);
        std::ptr::copy_nonoverlapping(cmds.as_ptr(), dst, cmds.len());
        assert_eq!(rc_apply_commands(h), 0, "no record rejected");
        for _ in 0..120 {
            rc_step(h);
        }
        assert_eq!(rc_state_hash(h), direct);
        assert_eq!(rc_state_hash(h), TERRAIN_SCENE_120, "golden: {:#018x}", rc_state_hash(h));
        rc_host_free(h);
    }
}

/// A thrusting craft after 120 steps: three 10 to 20 kg nodes in a triangle at height 50 m
/// (nothing to land on), two thrusters, and the throttles changed mid-run (thruster 0 full from the
/// start, thruster 1 at 0.6 from step 20, thruster 0 down to 0.3 from step 70). The same
/// records are built in `test/wasm.test.ts`.
pub const THRUST_SCENE_120: u64 = 0x765d_0f44_7551_4944;

fn thrust_scene_setup() -> Vec<Command> {
    let rec = |kind, a, b, f| Command { kind, a, b, reserved: 0, f };
    let mut cmds = Vec::new();
    for [x, y, z, inv_mass] in [[0.0, 50.0, 0.0, 0.1], [1.5, 50.0, 0.0, 0.1], [0.75, 50.0, 1.3, 0.05]] {
        cmds.push(rec(CMD_ADD_NODE, 0, 0, [x, y, z, inv_mass, 0.1, 0.0]));
    }
    for (a, b) in [(0, 1), (1, 2), (2, 0)] {
        cmds.push(rec(CMD_ADD_BEAM, a, b, [1.5, 1e-6, 50.0, 0.5, 1.0, 0.0]));
    }
    cmds.push(rec(CMD_ADD_THRUSTER, 0, 1, [400.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    cmds.push(rec(CMD_ADD_THRUSTER, 2, 1, [250.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    cmds.push(rec(CMD_SET_THROTTLE, 0, 0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    cmds
}

/// The throttle commands applied before step `s` (0-based), if any.
fn thrust_scene_at(s: u32) -> Vec<Command> {
    let rec = |kind, a, b, f| Command { kind, a, b, reserved: 0, f };
    match s {
        20 => vec![rec(CMD_SET_THROTTLE, 1, 0, [0.6, 0.0, 0.0, 0.0, 0.0, 0.0])],
        70 => vec![rec(CMD_SET_THROTTLE, 0, 0, [0.3, 0.0, 0.0, 0.0, 0.0, 0.0])],
        _ => Vec::new(),
    }
}

/// Thrust through the boundary gives the same bits as applied directly, and matches the golden
/// that the WebAssembly build is held to.
#[test]
fn thrust_scene_through_the_boundary_matches_the_direct_world() {
    let mut w = World::new();
    let mut results = Vec::new();
    let setup = thrust_scene_setup();
    apply_commands(&mut w, &setup, &mut results);
    assert!(!results.contains(&u32::MAX), "a record was rejected: {results:?}");
    for s in 0..120 {
        let more = thrust_scene_at(s);
        if !more.is_empty() {
            apply_commands(&mut w, &more, &mut results);
            assert!(!results.contains(&u32::MAX));
        }
        step(&mut w);
    }
    let direct = state_hash(&w);
    // It flew: the craft moved in +x and its thrusters were on.
    assert!(w.pos[0].x > 0.5, "x = {}", w.pos[0].x);

    unsafe {
        let h = rc_host_new();
        let dst = rc_commands_reserve(h, setup.len() as u32);
        std::ptr::copy_nonoverlapping(setup.as_ptr(), dst, setup.len());
        assert_eq!(rc_apply_commands(h), 0, "no record rejected");
        for s in 0..120 {
            let more = thrust_scene_at(s);
            if !more.is_empty() {
                let dst = rc_commands_reserve(h, more.len() as u32);
                std::ptr::copy_nonoverlapping(more.as_ptr(), dst, more.len());
                assert_eq!(rc_apply_commands(h), 0, "no record rejected");
            }
            rc_step(h);
        }
        assert_eq!(rc_state_hash(h), direct);
        assert_eq!(rc_state_hash(h), THRUST_SCENE_120, "golden: {:#018x}", rc_state_hash(h));
        rc_host_free(h);
    }
}

#[test]
fn abi_version_is_exported() {
    assert_eq!(rc_abi_version(), ABI_VERSION);
}
