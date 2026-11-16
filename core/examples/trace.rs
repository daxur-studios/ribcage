//! Headless trace dump for demos. Reads world state only; the core is untouched.
//! Usage: cargo run -p beam-core --example trace -- <scenario> [keep_every_n]
//! Writes JSON traces into artifacts/traces/ (gitignored, regenerable).
//! Format notes: docs/research/demo-and-media-plan.md.

use beam_core::island::build_islands;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{gravity_at, state_hash, step, step_with_substeps};
use beam_core::solver::{substep_count, MAX_SUBSTEPS, MIN_SUBSTEPS, SUBSTEP_C};
use beam_core::world::{BeamSpec, World};
use beam_core::{STEP_HZ, STEP_SECONDS};
use std::fmt::Write as _;
use std::fs;

const FORMAT_VERSION: u32 = 1;

/// The scenario registry: one name, one command, one set of traces.
const SCENARIOS: &[&str] = &["hash-split", "brace", "substeps"];

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
\"compare\":\"{}\",\"precision\":\"f64\",\"seed\":0,\"units\":\"m,s,kg\",\"keep_every\":{every},\"nodes\":{},\"beams\":[",
        run.label,
        commit(),
        run.compare,
        w.node_count()
    );
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
    for s in 0..=run.steps {
        if s > 0 {
            (run.before_step)(&mut w, s);
            used = match run.substeps {
                Substeps::Rule => step(&mut w),
                Substeps::Fixed(n) => step_with_substeps(&mut w, n),
            };
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
        o.push_str("],\"events\":[],\"commands\":[]}");
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
        _ => unreachable!(),
    }
    for (name, json) in out {
        let path = format!("artifacts/traces/{name}.json");
        fs::write(&path, json).expect("write trace");
        println!("wrote {path}");
    }
}
