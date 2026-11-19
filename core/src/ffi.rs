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
use crate::maths::Quat;
use crate::real::{Real, Vec3};
use crate::sim::{physics_hash, state_hash, step};
use crate::world::{BeamSpec, World};

/// Bumped by any change to an exported function's signature, to a record layout or to the set of
/// command kinds. 3 added the terrain commands (kinds 7 and 8); 4 added the thruster commands
/// (kinds 9 and 10) and puts thrusters in the state hash (only when there are any). 5 added the
/// segment commands (kinds 11 and 12), the segment readers and `rc_physics_hash`, and puts segments
/// in the state hash (only once a segment exists). 6 added the rigid-body commands (kinds 13 and
/// 14) and the body readers, and puts bodies in both hashes (only while a body is alive). 7 added
/// the joint commands (kinds 15 and 16) and the joint readers, and puts joints in both hashes (only
/// once a joint exists). 8 added the stress estimate's switch and readers (`rc_set_estimate`,
/// `rc_est_*`), which touch neither hash nor any command. 9 added sleeping (command kind 17,
/// `CMD_SET_SLEEPING`, the wake and sleep events, kinds 4 and 5) and its readers (`rc_sleep_*`),
/// and puts the sleep state in the state hash (only once the command has run). 10 added splitting
/// into vessels (command kind 18, `CMD_BREAK_BEAM`, the split and vessel-segment events, kinds 6
/// and 7) and its readers (`rc_vessel_*`), and puts each node's vessel in the state hash (only
/// once a split has happened). Nothing else changed, so a recording made at ABI 2 to 9 still
/// means the same thing and replays (`src/recording.ts`).
pub const ABI_VERSION: u32 = 10;

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
    /// One segment's mass, centre and inertia, filled by [`rc_segment_props`].
    segment_props: Vec<Real>,
    /// Vessel per node slot or per segment, filled by [`rc_vessel_nodes`] and [`rc_vessel_segments`].
    vessel_ids: Vec<u32>,
    /// One vessel's mass, momentum and angular momentum, filled by [`rc_vessel_props`].
    vessel_props: Vec<Real>,
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
            segment_props: Vec::new(),
            vessel_ids: Vec::new(),
            vessel_props: Vec::new(),
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
    let spec = if standard != 0 {
        BenchSpec::STANDARD
    } else {
        BenchSpec::SMALL
    };
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
pub unsafe extern "C" fn rc_add_node(
    h: *mut Host,
    x: Real,
    y: Real,
    z: Real,
    inv_mass: Real,
    radius: Real,
) -> u32 {
    (*h).world
        .add_node(Vec3::new(x, y, z), inv_mass, radius)
        .index
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
    let spec = BeamSpec {
        rest,
        compliance,
        damping,
        yield_strain,
        break_strain,
    };
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

/// The state hash without the segment section (`sim::physics_hash`): equal to [`rc_state_hash`]
/// until a segment exists. The hash that proves segments change no physics.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_physics_hash(h: *const Host) -> u64 {
    physics_hash(&(*h).world)
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
            let unit = if w.yield_strain[k] > 0.0 {
                w.yield_strain[k]
            } else {
                w.break_strain[k]
            };
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

/// Segments added so far (plan 005).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_count(h: *const Host) -> u32 {
    (*h).world.segment_count()
}

/// Seams added so far: one per segment that has a parent.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_seam_count(h: *const Host) -> u32 {
    (*h).world.seam_count()
}

/// Each node slot's segment, one `u32` (`u32::MAX` for none).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_node_segment_ptr(h: *const Host) -> *const u32 {
    (*h).world.node_segment.as_ptr()
}

/// Segment flags, one `u32` per segment (`segment::SEG_*`).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_flags_ptr(h: *const Host) -> *const u32 {
    (*h).world.seg_flags.as_ptr()
}

/// Each segment's parent segment, one `u32` (`u32::MAX` for a root).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_parent_ptr(h: *const Host) -> *const u32 {
    (*h).world.seg_parent.as_ptr()
}

/// Each segment's seam to its parent, one `u32` (`u32::MAX` for a root).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_seam_ptr(h: *const Host) -> *const u32 {
    (*h).world.seg_seam.as_ptr()
}

/// Each segment's stored mass, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_mass_ptr(h: *const Host) -> *const Real {
    (*h).world.seg_mass.as_ptr()
}

/// Each seam's parent segment, one `u32`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_seam_a_ptr(h: *const Host) -> *const u32 {
    (*h).world.seam_a.as_ptr()
}

/// Each seam's child segment, one `u32`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_seam_b_ptr(h: *const Host) -> *const u32 {
    (*h).world.seam_b.as_ptr()
}

/// Each seam's axial area, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_seam_area_ptr(h: *const Host) -> *const Real {
    (*h).world.seam_area.as_ptr()
}

/// Each seam's bending section, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_seam_bending_ptr(h: *const Host) -> *const Real {
    (*h).world.seam_bending.as_ptr()
}

/// Body slots, live or not (plan 005, phase 5.2): the segment count once a body was made, else zero. A slot is numbered as its body's lowest segment.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_slot_count(h: *const Host) -> u32 {
    (*h).world.body_slot_count()
}

/// Each body slot's flags, one `u32` (`body::BODY_ALIVE`).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_flags_ptr(h: *const Host) -> *const u32 {
    (*h).world.body_flags.as_ptr()
}

/// Each body slot's centre of mass, world frame.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_centre_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.body_centre.as_ptr()
}

/// Each body slot's rotation, four `Real` `w x y z` (body frame to world).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_rot_ptr(h: *const Host) -> *const Quat {
    (*h).world.body_rot.as_ptr()
}

/// Each body slot's linear velocity.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_vel_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.body_vel.as_ptr()
}

/// Each body slot's angular velocity, world frame.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_omega_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.body_omega.as_ptr()
}

/// Each body slot's mass, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_body_mass_ptr(h: *const Host) -> *const Real {
    (*h).world.body_mass.as_ptr()
}

/// Number of joints (never removed, so an index is stable).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_joint_count(h: *const Host) -> u32 {
    (*h).world.joint_count()
}

/// Each joint's angle at the end of the last step, radians from its build pose, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_joint_angle_ptr(h: *const Host) -> *const Real {
    (*h).world.joint_angle.as_ptr()
}

/// Each joint's drive torque in the last substep of the last step, newton-metres, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_joint_torque_ptr(h: *const Host) -> *const Real {
    (*h).world.joint_torque.as_ptr()
}

/// Each joint's largest drive torque magnitude over the last step's substeps, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_joint_peak_ptr(h: *const Host) -> *const Real {
    (*h).world.joint_peak.as_ptr()
}

/// Each node slot's body, one `u32` (`u32::MAX` for a loose node).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_node_body_ptr(h: *const Host) -> *const u32 {
    (*h).world.node_body.as_ptr()
}

/// Each segment's body, one `u32` (`u32::MAX` when not rigid).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_body_ptr(h: *const Host) -> *const u32 {
    (*h).world.seg_body.as_ptr()
}

/// Computes one segment's mass, centre and inertia from the current positions and returns ten
/// `Real`: mass, centre x y z, then inertia `xx yy zz xy xz yz` (products negated, see
/// `segment::SegmentProps`). Null when the segment does not exist or has no mass. Valid until the
/// next call to this function.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_segment_props(h: *mut Host, segment: u32) -> *const Real {
    let host = &mut *h;
    match host.world.segment_centre_and_inertia(segment) {
        None => std::ptr::null(),
        Some(p) => {
            host.segment_props.clear();
            host.segment_props
                .extend_from_slice(&[p.mass, p.centre.x, p.centre.y, p.centre.z]);
            host.segment_props.extend_from_slice(&p.inertia);
            host.segment_props.as_ptr()
        }
    }
}

/// Switches the stress estimate on (`on != 0`) or off (phase 5.7). Read-only: it changes no
/// hash and is not a command, so recordings do not hold it. Its outputs fill from the next step.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_set_estimate(h: *mut Host, on: u32) {
    let est = &mut (*h).world.est;
    est.on = on != 0;
    if !est.on {
        *est = crate::estimate::Estimate::default();
    }
}

/// Seams the estimate covered in the last step (0 when off or before a step with it on); the
/// length of every `rc_est_seam_*` array. The segment array has `rc_segment_count` entries then.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_count(h: *const Host) -> u32 {
    (*h).world.est.seam_ratio.len() as u32
}

/// Per seam: force from the parent side on the child's subtree over the last step, three `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_force_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.est.seam_force.as_ptr()
}

/// Per seam: the matching moment about the seam's centre, three `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_moment_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.est.seam_moment.as_ptr()
}

/// Per seam: the point the moment is taken about, three `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_centre_ptr(h: *const Host) -> *const Vec3 {
    (*h).world.est.seam_centre.as_ptr()
}

/// Per seam: the estimated worst load ratio of its beams, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_ratio_ptr(h: *const Host) -> *const Real {
    (*h).world.est.seam_ratio.as_ptr()
}

/// Per seam: the measured worst load ratio of the same beams (NaN when none is solved), one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_seam_measured_ptr(h: *const Host) -> *const Real {
    (*h).world.est.seam_measured.as_ptr()
}

/// Per segment: the worst estimated ratio over the seams it touches, one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_est_segment_ratio_ptr(h: *const Host) -> *const Real {
    (*h).world.est.seg_ratio.as_ptr()
}

/// Segments the sleep state covers (phase 5.8): the segment count once sleeping was switched on
/// and a step ran, else zero. The length of `rc_sleep_asleep_ptr` and `rc_sleep_ratio_ptr`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_sleep_segment_count(h: *const Host) -> u32 {
    (*h).world.sleep.asleep.len() as u32
}

/// Per segment: 1 when sleeping put it to sleep and it is still rigid, one `u32`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_sleep_asleep_ptr(h: *const Host) -> *const u32 {
    (*h).world.sleep.asleep.as_ptr()
}

/// Per segment: the last step's load reading as a fraction of yield (measured when live, the
/// estimate with its margin when rigid), one `Real`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_sleep_ratio_ptr(h: *const Host) -> *const Real {
    (*h).world.sleep.ratio.as_ptr()
}

/// Switch passes in the last step (each a wake pass or a sleep pass).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_sleep_check_count(h: *const Host) -> u32 {
    (*h).world.sleep.checks.len() as u32
}

/// The last step's switch passes, 15 `Real` each: kind (1 wake, 2 sleep), then momentum,
/// angular momentum about the origin and kinetic energy before, then the same after.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_sleep_checks_ptr(h: *const Host) -> *const Real {
    (*h).world.sleep.checks.as_ptr() as *const Real
}

/// Vessel ids handed out so far (phase 5.10): one more than the highest, 1 before the first split.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_ids(h: *const Host) -> u32 {
    (*h).world.vessel_ids()
}

/// Per node slot: its vessel, one `u32` (`u32::MAX` for a dead slot or a node added since the
/// last check). As many as `rc_node_count`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_nodes(h: *mut Host) -> *const u32 {
    let host = &mut *h;
    let n = host.world.node_count();
    host.vessel_ids.clear();
    for i in 0..n {
        let v = host.world.node_vessel(i);
        host.vessel_ids.push(v);
    }
    host.vessel_ids.as_ptr()
}

/// Per segment: its vessel, one `u32` (`u32::MAX` for a segment with no live node). As many as
/// `rc_segment_count`.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_segments(h: *mut Host) -> *const u32 {
    let host = &mut *h;
    let n = host.world.segment_count();
    host.vessel_ids.clear();
    for s in 0..n {
        let v = host.world.segment_vessel(s);
        host.vessel_ids.push(v);
    }
    host.vessel_ids.as_ptr()
}

/// One vessel now, seven `Real`: mass (its segments' stored masses plus its massive nodes with no
/// segment), momentum, then angular momentum about the origin (riders move with their body).
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_props(h: *mut Host, vessel: u32) -> *const Real {
    let host = &mut *h;
    let (p, l) = host.world.vessel_momentum(vessel);
    let m = host.world.vessel_mass(vessel);
    host.vessel_props.clear();
    host.vessel_props.extend_from_slice(&[m, p.x, p.y, p.z, l.x, l.y, l.z]);
    host.vessel_props.as_ptr()
}

/// Body divisions in the last step: 0 or 1.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_check_count(h: *const Host) -> u32 {
    (*h).world.vessel.checks.len() as u32
}

/// The last step's body division, 15 `Real` (as `rc_sleep_checks_ptr`, kind 3): totals before
/// and after.
///
/// # Safety
/// `h` is a live host pointer.
#[no_mangle]
pub unsafe extern "C" fn rc_vessel_checks_ptr(h: *const Host) -> *const Real {
    (*h).world.vessel.checks.as_ptr() as *const Real
}
