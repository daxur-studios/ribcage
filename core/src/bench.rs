//! The benchmark scene (milestone 1, phase 1.7): a fixed stress scene that reports beams,
//! substeps and milliseconds per step. This module only builds the scene and its load; the
//! clock lives in `core/examples/bench.rs`, because `std::time` is not available in the
//! WebAssembly build and the core never reads time (determinism list).
//!
//! The scene is the spec's tank or fuselage generator, repeated: a column of rings, each ring a
//! polygon of nodes (placed with the core's own sine and cosine, so the scene is bit-identical
//! natively and in WebAssembly), with hoop beams along each ring, lengthwise beams and
//! both diagonals in every quad between rings (one diagonal alone is a twist-and-crush
//! mechanism and buckles; the spec's tank has cross-bracing). The bottom ring is pinned, the
//! top ring carries a heavy payload, and a sinusoidal sideways gust pushes the top. Columns
//! stand apart, so each one is its own island. Every number is fixed here; change one and the
//! benchmark's history is not comparable.

use crate::maths::{sin, sin_cos, TAU};
use crate::real::{Real, Vec3};
use crate::world::{BeamSpec, World};

/// Shape of the scene.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BenchSpec {
    /// Independent columns (islands).
    pub columns: u32,
    /// Rings per column, the pinned base ring included.
    pub rings: u32,
    /// Nodes per ring.
    pub sides: u32,
    /// Axial stiffness `EA` of every beam in newtons: compliance is `rest / EA`.
    pub ea: Real,
}

impl BenchSpec {
    /// The reported scene: 8 columns of 30 rings of 12 nodes, 11,136 beams. **[assumption]**
    /// The size is a stand-in for "a busy scene" until the owner sets the performance budget;
    /// the stiffness gives about 17 substeps at the rule's default, a typical count.
    pub const STANDARD: BenchSpec = BenchSpec { columns: 8, rings: 30, sides: 12, ea: 500_000.0 };
    /// A small scene for tests.
    pub const SMALL: BenchSpec = BenchSpec { columns: 2, rings: 6, sides: 8, ea: 500_000.0 };

    /// Beams in the scene (the base ring has no hoop beams: both ends pinned).
    pub fn beam_count(&self) -> u32 {
        self.columns * (self.rings - 1) * self.sides * 4
    }

    /// Nodes in the scene.
    pub fn node_count(&self) -> u32 {
        self.columns * self.rings * self.sides
    }
}

/// Column radius in metres, ring spacing in metres, gap between columns in metres.
const RADIUS: Real = 2.0;
const SPACING: Real = 0.5;
const GAP: Real = 6.0;
/// Node masses in kilograms: body and the top ring's payload share.
const BODY_MASS: Real = 2.0;
const TOP_MASS: Real = 8.0;
/// Beam damping in newton-seconds per metre, yield and break strain.
const DAMPING: Real = 50.0;
const YIELD: Real = 0.01;
const BREAK: Real = 0.05;
/// Gust: newtons on each top node, and its period in steps (two seconds).
const GUST_N: Real = 10.0;
const GUST_PERIOD: u64 = 120;

/// The world plus the node list the load acts on.
pub struct BenchScene {
    pub world: World,
    pub spec: BenchSpec,
    top_nodes: Vec<u32>,
}

/// Builds the scene. Beams start at their rest length, so nothing is stressed but gravity.
pub fn bench_scene(spec: BenchSpec) -> BenchScene {
    assert!(spec.columns > 0 && spec.rings > 1 && spec.sides > 2, "degenerate benchmark scene");
    let mut w = World::new();
    let mut top_nodes = Vec::new();
    let per_column = spec.rings * spec.sides;
    for c in 0..spec.columns {
        let cx = c as Real * (2.0 * RADIUS + GAP);
        for r in 0..spec.rings {
            for k in 0..spec.sides {
                let (s, co) = sin_cos(TAU * k as Real / spec.sides as Real);
                let p = Vec3::new(cx + RADIUS * co, r as Real * SPACING, RADIUS * s);
                let mass = if r == 0 {
                    0.0
                } else if r + 1 == spec.rings {
                    1.0 / TOP_MASS
                } else {
                    1.0 / BODY_MASS
                };
                let h = w.add_node(p, mass, 0.05);
                if r + 1 == spec.rings {
                    top_nodes.push(h.index);
                }
            }
        }
        let node = |r: u32, k: u32| c * per_column + r * spec.sides + k % spec.sides;
        for r in 0..spec.rings {
            for k in 0..spec.sides {
                if r > 0 {
                    add(&mut w, node(r, k), node(r, k + 1), spec.ea);
                }
                if r + 1 < spec.rings {
                    add(&mut w, node(r, k), node(r + 1, k), spec.ea);
                    add(&mut w, node(r, k), node(r + 1, k + 1), spec.ea);
                    add(&mut w, node(r, k + 1), node(r + 1, k), spec.ea);
                }
            }
        }
    }
    debug_assert_eq!(w.beam_count(), spec.beam_count());
    BenchScene { world: w, spec, top_nodes }
}

fn add(w: &mut World, a: u32, b: u32, ea: Real) {
    let rest = (w.pos[b as usize] - w.pos[a as usize]).length();
    w.add_beam(a, b, BeamSpec { rest, compliance: rest / ea, damping: DAMPING, yield_strain: YIELD, break_strain: BREAK });
}

impl BenchScene {
    /// Pushes the top ring sideways for the coming step: `10 N · sin(2π · step / 120)` on each top
    /// node along x, through [`World::add_force`]. A function of the step counter only.
    pub fn apply_load(&mut self) {
        let phase = (self.world.step_count % GUST_PERIOD) as Real / GUST_PERIOD as Real;
        let f = Vec3::new(GUST_N * sin(TAU * phase), 0.0, 0.0);
        for &n in &self.top_nodes {
            self.world.add_force(n, f);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{state_hash, step, step_with_rule};
    use crate::solver::SubstepRule;

    fn run(rule: Option<SubstepRule>, steps: u32) -> (BenchScene, u64) {
        let mut s = bench_scene(BenchSpec::SMALL);
        for _ in 0..steps {
            s.apply_load();
            match rule {
                None => {
                    step(&mut s.world);
                }
                Some(r) => {
                    step_with_rule(&mut s.world, r);
                }
            }
        }
        let h = state_hash(&s.world);
        (s, h)
    }

    #[test]
    fn counts_match_the_spec() {
        let s = bench_scene(BenchSpec::STANDARD);
        assert_eq!(s.world.beam_count(), 11_136);
        assert_eq!(s.world.node_count(), 2_880);
        let small = bench_scene(BenchSpec::SMALL);
        assert_eq!(small.world.beam_count(), BenchSpec::SMALL.beam_count());
    }

    /// The scene holds still enough to be a fair benchmark: no events, no freeze, nothing
    /// clamped, one island per column, and the same bits twice.
    #[test]
    fn scene_is_calm_and_deterministic() {
        let (a, ha) = run(None, 120);
        let (_, hb) = run(None, 120);
        assert_eq!(ha, hb);
        assert!(a.world.events.is_empty());
        assert_eq!(a.world.clamped_nodes, 0);
        for p in &a.world.pos {
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        }
        let mut s = bench_scene(BenchSpec::SMALL);
        let stats = step_with_rule(&mut s.world, SubstepRule::DEFAULT);
        assert_eq!(stats.islands, BenchSpec::SMALL.columns);
    }

    /// The reported scene stays whole for the whole benchmark (warm-up plus timed steps) at both
    /// rules: no break, no yield, no freeze, one island per column. A scene that tears apart
    /// would make the timing depend on when it tore. Slow in a debug build: run in release.
    #[test]
    #[ignore = "slow; run with --release --include-ignored"]
    fn standard_scene_stays_intact_at_both_rules() {
        for rule in [SubstepRule::DEFAULT, SubstepRule { c: 0.5, max: 64 }] {
            let mut s = bench_scene(BenchSpec::STANDARD);
            for _ in 0..720 {
                s.apply_load();
                let stats = step_with_rule(&mut s.world, rule);
                assert_eq!(stats.islands, BenchSpec::STANDARD.columns, "{rule:?} step {}", s.world.step_count);
                assert!(s.world.events.is_empty(), "{rule:?}: {:?}", s.world.events);
            }
        }
    }

    /// The rule given as a value is the default rule: same bits as `step`.
    #[test]
    fn default_rule_is_the_default_step() {
        let (_, a) = run(None, 60);
        let (_, b) = run(Some(SubstepRule::DEFAULT), 60);
        assert_eq!(a, b);
    }
}
