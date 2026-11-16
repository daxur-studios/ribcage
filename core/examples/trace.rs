//! Headless trace dump for demos. Reads world state only; the core is untouched.
//! Usage: cargo run -p beam-core --example trace -- <scenario> [keep_every_n]
//! Writes JSON traces into artifacts/traces/ (gitignored, regenerable).
//! Format notes: docs/research/demo-and-media-plan.md.

use beam_core::real::{Real, Vec3};
use beam_core::sim::{gravity_at, state_hash, step, SUBSTEPS};
use beam_core::world::{BeamSpec, World};
use beam_core::{STEP_HZ, STEP_SECONDS};
use std::fmt::Write as _;
use std::fs;

const FORMAT_VERSION: u32 = 1;

/// The scenario registry: one name, one command, one set of traces.
const SCENARIOS: &[&str] = &["hash-split"];

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

/// Runs one world for `steps` steps and returns the trace JSON.
fn record(scenario: &str, label: &str, mut w: World, steps: u32, every: u32, nudge_at: Option<u32>) -> String {
    let mut o = String::new();
    let _ = write!(
        o,
        "{{\"version\":{FORMAT_VERSION},\"scenario\":\"{scenario}\",\"label\":\"{label}\",\"core_commit\":\"{}\",\
\"step_hz\":{STEP_HZ},\"step_seconds\":{STEP_SECONDS},\"substeps\":{SUBSTEPS},\"substep_rule\":\"fixed placeholder\",\
\"precision\":\"f64\",\"seed\":0,\"units\":\"m,s,kg\",\"keep_every\":{every},\"nodes\":{},\"beams\":[",
        commit(),
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
    for s in 0..=steps {
        if nudge_at == Some(s) {
            w.pos[1].x += 1e-9;
        }
        if s > 0 {
            step(&mut w);
        }
        if s % every != 0 && s != steps {
            continue;
        }
        if s > 0 {
            o.push(',');
        }
        let _ = write!(o, "{{\"i\":{},\"hash\":\"{:016x}\",\"substeps\":{SUBSTEPS},\"energy\":{},\"pos\":[", w.step_count, state_hash(&w), total_energy(&w));
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
            let len = d.length_squared().sqrt();
            let _ = write!(o, "{}", (len - w.rest[b]) / w.rest0[b]);
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
    match scenario {
        "hash-split" => {
            // Two identical runs; at step 60 one node of run B is nudged by a billionth of a metre.
            out.push((format!("{scenario}-a"), record(scenario, "run A", base_scene(), 180, every, None)));
            out.push((format!("{scenario}-b"), record(scenario, "run B (node 1 nudged 1e-9 m at step 60)", base_scene(), 180, every, Some(60))));
        }
        _ => unreachable!(),
    }
    for (name, json) in out {
        let path = format!("artifacts/traces/{name}.json");
        fs::write(&path, json).expect("write trace");
        println!("wrote {path}");
    }
}
