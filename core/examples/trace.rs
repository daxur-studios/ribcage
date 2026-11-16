//! Headless trace dump for demos. Reads world state only; the core is untouched.
//! Usage: cargo run -p beam-core --example trace -- <scenario> [keep_every_n]
//! Writes JSON traces into artifacts/traces/ (gitignored, regenerable).
//! Format notes: docs/research/demo-and-media-plan.md.

use beam_core::island::build_islands;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{gravity_at, state_hash, step, step_with_substeps};
use beam_core::solver::{substep_count, MAX_SUBSTEPS, MIN_SUBSTEPS, SUBSTEP_C};
use beam_core::event::EVENT_BREAK;
use beam_core::world::{BeamSpec, World, BEAM_BROKEN};
use beam_core::{STEP_HZ, STEP_SECONDS};
use std::fmt::Write as _;
use std::fs;
use truss::{apply_tip_load, build, textbook_tip, TrussSpec};

#[path = "../tests/support/truss.rs"]
mod truss;

const FORMAT_VERSION: u32 = 1;

/// The scenario registry: one name, one command, one set of traces.
const SCENARIOS: &[&str] = &["hash-split", "brace", "substeps", "cantilever", "snap"];

fn base_scene() -> World {
    let mut w = World::new();
    let s = BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.1, yield_strain: 0.01, break_strain: 0.05 };
    let a = w.add_node(Vec3::new(0.0, 10.0, 0.0), 0.0, 0.1);
    let b = w.add_node(Vec3::new(1.0, 10.0, 0.3), 1.0, 0.1);
    let c = w.add_node(Vec3::new(2.0, 11.0, -0.2), 0.5, 0.1);
    let d = w.add_node(Vec3::new(3.0, 10.5, 0.1), 1.0, 0.1);
    w.add_beam(a.index, b.index, s);
    w.add_beam(b.index, c.index, s);
    w.add_beam(c.index, d.index, s);
    w
}

/// A 1 m square of 1 kg nodes standing on two pinned feet, optionally with one diagonal.
/// Beams: compliance 1e-5 (100 kN/m), damping about a tenth of one link's critical value.
fn square(braced: bool) -> World {
    let mut w = World::new();
    let s = BeamSpec { rest: 1.0, compliance: 1e-5, damping: 60.0, yield_strain: 0.01, break_strain: 0.05 };
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.05);
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.05);
    w.add_node(Vec3::new(1.0, 1.0, 0.0), 1.0, 0.05);
    w.add_node(Vec3::new(0.0, 1.0, 0.0), 1.0, 0.05);
    w.add_beam(0, 1, s);
    w.add_beam(1, 2, s);
    w.add_beam(2, 3, s);
    w.add_beam(3, 0, s);
    if braced {
        let diag = BeamSpec { rest: (2.0 as Real).sqrt(), ..s };
        w.add_beam(0, 2, diag);
    }
    w
}

/// A 4 m flat cantilever truss (8 bays of 0.5 m, 0.5 m deep), pinned at the left end, with
/// a 10 kg load on each tip node. Beams: compliance 1e-6 (1 MN/m), light damping.
fn cantilever() -> World {
    let mut w = World::new();
    let bays = 8u32;
    let d = 0.5;
    for i in 0..=bays {
        let x = i as Real * d;
        let inv = if i == 0 { 0.0 } else if i == bays { 0.1 } else { 1.0 };
        w.add_node(Vec3::new(x, 0.0, 0.0), inv, 0.03); // bottom: 2i
        w.add_node(Vec3::new(x, d, 0.0), inv, 0.03); // top: 2i + 1
    }
    let s = |rest: Real| BeamSpec { rest, compliance: 1e-6, damping: 200.0, yield_strain: 0.01, break_strain: 0.05 };
    let diag = (2.0 as Real).sqrt() * d;
    for i in 0..bays {
        let (b0, t0, b1, t1) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
        w.add_beam(b0, b1, s(d));
        w.add_beam(t0, t1, s(d));
        w.add_beam(b1, t1, s(d));
        w.add_beam(b0, t1, s(diag));
    }
    w
}

/// A 1 m beam hanging from a pinned node under a 1 kg mass, critically damped, with a rubbery
/// 4% yield and 15% break strain so the stretch is visible on screen. Natural frequency 60 rad/s,
/// critically damped. It does not settle within one 60 Hz step (a critically damped response
/// after `ω t` = 1 has covered `1 - 2/e` = 26% of the way: 0.79% strain of a 3% load), so the numbers in the snap
/// demo are dynamic results of a slow load ramp, not static readings. Its yield force is
/// `0.04 / compliance` = 144 N; a load above that flows at `(F - 144 N) / 1 kg`.
fn hanging_beam() -> World {
    let mut w = World::new();
    let omega: Real = 60.0;
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.05);
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0, 0.05);
    let spec = BeamSpec { rest: 1.0, compliance: 1.0 / (omega * omega), damping: 2.0 * omega, yield_strain: 0.04, break_strain: 0.15 };
    w.add_beam(0, 1, spec);
    w
}

/// Downward pull on the hanging beam's mass that stretches it by `strain` (elastic, settled).
fn pull_for(strain: Real) -> Real {
    let omega: Real = 60.0;
    strain * omega * omega - 9.81
}

/// Kinetic plus gravitational plus elastic energy, in joules.
fn total_energy(w: &World) -> Real {
    let mut e = 0.0;
    for i in 0..w.node_count() {
        let u = i as usize;
        if !w.node_alive(i) || w.inv_mass[u] == 0.0 {
            continue;
        }
        let m = 1.0 / w.inv_mass[u];
        e += 0.5 * m * w.vel[u].length_squared() - m * gravity_at(w.pos[u]).y * w.pos[u].y;
    }
    for b in 0..w.beam_count() {
        let u = b as usize;
        if !w.beam_alive(b) || w.compliance[u] == 0.0 {
            continue;
        }
        let c = (w.pos[w.beam_b[u] as usize] - w.pos[w.beam_a[u] as usize]).length() - w.rest[u];
        e += 0.5 * c * c / w.compliance[u];
    }
    e
}

fn commit() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}

/// How a run picks its substeps: the deterministic rule, or a fixed count for comparisons.
#[derive(Clone, Copy)]
enum Substeps {
    Rule,
    Fixed(u32),
}

/// One run: a label, a length, how substeps are picked, and inputs applied before each step.
struct Run<'a> {
    label: String,
    steps: u32,
    substeps: Substeps,
    /// "hash": the panes are the same scene and their hashes should match.
    /// "none": the panes are different scenes on purpose; the player skips the comparison.
    compare: &'a str,
    before_step: &'a dyn Fn(&mut World, u32),
    /// Reference points the player draws in every pane, as `(label, x, y, nodes)`: when
    /// `nodes` is not empty the player also prints those nodes' mean height next to `y`.
    marks: Vec<(String, Real, Real, Vec<u32>)>,
}

fn no_input(_: &mut World, _: u32) {}

/// Runs one world and returns the trace JSON.
fn record(scenario: &str, run: &Run, mut w: World, every: u32) -> String {
    let (rule, mut used) = match run.substeps {
        Substeps::Rule => (
            format!("rule: smallest n with omega_max*h <= {SUBSTEP_C}, min {MIN_SUBSTEPS}, max {MAX_SUBSTEPS}"),
            substep_count(&build_islands(&w)[0]),
        ),
        Substeps::Fixed(n) => (format!("fixed {n}"), n),
    };
    let mut o = String::new();
    let _ = write!(
        o,
        "{{\"version\":{FORMAT_VERSION},\"scenario\":\"{scenario}\",\"label\":\"{}\",\"core_commit\":\"{}\",\
\"step_hz\":{STEP_HZ},\"step_seconds\":{STEP_SECONDS},\"substeps\":{used},\"substep_rule\":\"{rule}\",\
\"compare\":\"{}\",\"precision\":\"f64\",\"seed\":0,\"units\":\"m,s,kg\",\"keep_every\":{every},\"nodes\":{},\"marks\":[",
        run.label,
        commit(),
        run.compare,
        w.node_count()
    );
    for (k, (label, x, y, nodes)) in run.marks.iter().enumerate() {
        if k > 0 {
            o.push(',');
        }
        let list: Vec<String> = nodes.iter().map(|n| n.to_string()).collect();
        let _ = write!(o, "{{\"label\":\"{label}\",\"x\":{x},\"y\":{y},\"nodes\":[{}]}}", list.join(","));
    }
    o.push_str("],\"beams\":[");
    for b in 0..w.beam_count() as usize {
        if b > 0 {
            o.push(',');
        }
        let _ = write!(
            o,
            "{{\"a\":{},\"b\":{},\"rest\":{},\"yield\":{},\"break\":{}}}",
            w.beam_a[b], w.beam_b[b], w.rest[b], w.yield_strain[b], w.break_strain[b]
        );
    }
    o.push_str("],\"steps\":[");
    // Events since the last kept step, so keep_every never drops one.
    let mut pending = String::new();
    for s in 0..=run.steps {
        if s > 0 {
            (run.before_step)(&mut w, s);
            used = match run.substeps {
                Substeps::Rule => step(&mut w),
                Substeps::Fixed(n) => step_with_substeps(&mut w, n),
            };
            for e in &w.events {
                let kind = if e.kind == EVENT_BREAK { "break" } else { "yield" };
                if !pending.is_empty() {
                    pending.push(',');
                }
                let _ = write!(pending, "{{\"kind\":\"{kind}\",\"beam\":{},\"step\":{},\"value\":{}}}", e.index, e.step, e.value);
            }
        }
        if s % every != 0 && s != run.steps {
            continue;
        }
        if s > 0 {
            o.push(',');
        }
        let _ = write!(o, "{{\"i\":{},\"hash\":\"{:016x}\",\"substeps\":{used},\"energy\":{},\"pos\":[", w.step_count, state_hash(&w), total_energy(&w));
        for n in 0..w.pos.len() {
            if n > 0 {
                o.push(',');
            }
            let p = w.pos[n];
            let _ = write!(o, "{},{},{}", p.x, p.y, p.z);
        }
        o.push_str("],\"strain\":[");
        for b in 0..w.beam_count() as usize {
            if b > 0 {
                o.push(',');
            }
            let d = w.pos[w.beam_b[b] as usize] - w.pos[w.beam_a[b] as usize];
            let _ = write!(o, "{}", (d.length() - w.rest[b]) / w.rest0[b]);
        }
        // Permanent set per beam: how far yield has moved the rest length, as a strain of rest0.
        // Optional field (the player ignores a trace without it); `broken` lists dead beams.
        o.push_str("],\"set\":[");
        for b in 0..w.beam_count() as usize {
            if b > 0 {
                o.push(',');
            }
            let _ = write!(o, "{}", (w.rest[b] - w.rest0[b]) / w.rest0[b]);
        }
        o.push_str("],\"broken\":[");
        let dead: Vec<String> = (0..w.beam_count()).filter(|&b| w.beam_flags[b as usize] & BEAM_BROKEN != 0).map(|b| b.to_string()).collect();
        o.push_str(&dead.join(","));
        let _ = write!(o, "],\"events\":[{pending}],\"commands\":[]}}");
        pending.clear();
    }
    o.push_str("]}");
    o
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let scenario = args.get(1).map(String::as_str).unwrap_or("");
    let every: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1).max(1);
    if !SCENARIOS.contains(&scenario) {
        eprintln!("usage: trace <scenario> [keep_every_n]\nscenarios: {}", SCENARIOS.join(", "));
        std::process::exit(2);
    }
    fs::create_dir_all("artifacts/traces").expect("create artifacts/traces");
    let mut out: Vec<(String, String)> = Vec::new();
    let run = |label: &str, steps: u32, substeps: Substeps, compare: &'static str, before_step: &'static dyn Fn(&mut World, u32)| Run {
        label: label.to_string(),
        steps,
        substeps,
        compare,
        before_step,
        marks: Vec::new(),
    };
    match scenario {
        "hash-split" => {
            // Two identical runs; at step 60 one node of run B is nudged by a billionth of a metre.
            let nudge: &'static dyn Fn(&mut World, u32) = &|w: &mut World, s: u32| {
                if s == 60 {
                    w.pos[1].x += 1e-9;
                }
            };
            out.push((format!("{scenario}-a"), record(scenario, &run("run A", 180, Substeps::Rule, "hash", &no_input), base_scene(), every)));
            out.push((format!("{scenario}-b"), record(scenario, &run("run B (node 1 nudged 1e-9 m at step 60)", 180, Substeps::Rule, "hash", nudge), base_scene(), every)));
        }
        "brace" => {
            // The same 3 N sideways push on the top-left node for half a second, then gravity only.
            let push: &'static dyn Fn(&mut World, u32) = &|w: &mut World, s: u32| {
                if s <= 30 {
                    w.add_force(3, Vec3::new(3.0, 0.0, 0.0));
                }
            };
            out.push((format!("{scenario}-open"), record(scenario, &run("square, no diagonal", 240, Substeps::Rule, "none", push), square(false), every)));
            out.push((format!("{scenario}-braced"), record(scenario, &run("square with diagonal brace", 240, Substeps::Rule, "none", push), square(true), every)));
        }
        "substeps" => {
            // The same loaded cantilever truss at fixed 1, 4 and 16 substeps, then the rule's count.
            for n in [1u32, 4, 16] {
                let label = format!("{n} substep{}", if n == 1 { "" } else { "s" });
                out.push((format!("{scenario}-{n:02}"), record(scenario, &run(&label, 240, Substeps::Fixed(n), "none", &no_input), cantilever(), every)));
            }
            let n = substep_count(&build_islands(&cantilever())[0]);
            out.push((format!("{scenario}-rule"), record(scenario, &run(&format!("rule: {n} substeps"), 240, Substeps::Rule, "none", &no_input), cantilever(), every)));
        }
        "cantilever" => {
            // The phase 1.4 truss (6 m, 0.5 m square section, 200 N on the tip) settling at
            // fixed 8, 16 and 32 substeps and at the rule's count. Beam damping is zero; a
            // test-side drag of 10 N·s/m per node settles it, as in the test. The marker is
            // the textbook tip: the tip frame's centre height minus the formula's deflection.
            let spec = TrussSpec::standard();
            let load: &'static dyn Fn(&mut World, u32) = &|w: &mut World, _| {
                let spec = TrussSpec::standard();
                apply_tip_load(w, &spec);
                for i in 0..w.pos.len() {
                    if w.inv_mass[i] != 0.0 {
                        let v = w.vel[i];
                        w.add_force(i as u32, v * -10.0);
                    }
                }
            };
            let tip_y = 0.5 * spec.width - textbook_tip(&spec);
            let tips: Vec<u32> = spec.tip_nodes().iter().map(|&t| t as u32).collect();
            let rule = substep_count(&build_islands(&build(&spec))[0]);
            for (name, label, n) in [("08", "8 substeps".to_string(), Substeps::Fixed(8)), ("16", "16 substeps".into(), Substeps::Fixed(16)), ("rule", format!("rule: {rule} substeps"), Substeps::Rule), ("32", "32 substeps".into(), Substeps::Fixed(32))] {
                let mut r = run(&label, 240, n, "none", load);
                r.marks.push(("textbook tip".into(), spec.length(), tip_y, tips.clone()));
                out.push((format!("{scenario}-{name}"), record(scenario, &r, build(&spec), every)));
            }
        }
        "snap" => {
            // The same hanging beam under three slowly growing pulls (120 steps up, 120 down, then
            // rest): one that stays below its yield force, one that rises just past it and comes
            // back, and one that keeps growing. A beam carries at most its yield force, so any
            // load held above it makes the mass accelerate at the excess force until the beam
            // snaps; only a brief excess leaves a small permanent set.
            let ramp = |peak: Real, down: bool| -> &'static dyn Fn(&mut World, u32) {
                Box::leak(Box::new(move |w: &mut World, s: u32| {
                    let rising = s.min(120) as Real / 120.0;
                    let k = if down { rising - s.saturating_sub(120).min(120) as Real / 120.0 } else { s as Real / 120.0 };
                    if k > 0.0 {
                        w.add_force(1, Vec3::new(0.0, -pull_for(peak * k), 0.0));
                    }
                    // A demo-side floor 1.8 m down, so the snapped mass stays in frame.
                    if w.pos[1].y < -1.8 {
                        w.pos[1].y = -1.8;
                        w.vel[1] = Vec3::ZERO;
                    }
                }))
            };
            out.push((format!("{scenario}-a"), record(scenario, &run("springs back: load peaks at 3% strain", 300, Substeps::Rule, "none", ramp(0.03, true)), hanging_beam(), every)));
            out.push((format!("{scenario}-b"), record(scenario, &run("stays bent: load peaks at 4.1% strain", 300, Substeps::Rule, "none", ramp(0.041, true)), hanging_beam(), every)));
            out.push((format!("{scenario}-c"), record(scenario, &run("snaps: load keeps growing", 300, Substeps::Rule, "none", ramp(0.06, false)), hanging_beam(), every)));
        }
        _ => unreachable!(),
    }
    for (name, json) in out {
        let path = format!("artifacts/traces/{name}.json");
        fs::write(&path, json).expect("write trace");
        println!("wrote {path}");
    }
}
