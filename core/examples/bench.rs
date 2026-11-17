//! Benchmark runner (milestone 1, phase 1.7): beams, substeps and milliseconds per step on the
//! fixed scene in `beam_core::bench`, at the default substep rule and at `c = 0.5` for the cost
//! comparison. Native timing only; the core itself never reads a clock.
//!
//! Usage: cargo run --release -p beam-core --example bench -- [--steps N] [--repeats N] [--small]
//! Prints a table and writes artifacts/bench/bench.json (gitignored, regenerable).
//! Debug builds are far slower; only a release number means anything. Native only: the
//! WebAssembly cost is measured when the core first runs in the browser (milestone 2).

use beam_core::bench::{bench_scene, BenchSpec};
use beam_core::island::{build_islands, copy_back};
use beam_core::sim::{state_hash, step_with_rule, StepStats};
use beam_core::solver::SubstepRule;
use std::fmt::Write as _;
use std::fs;
use std::time::Instant;

const WARMUP_STEPS: u32 = 120;

struct Run {
    stats: StepStats,
    /// Milliseconds per step, in step order.
    ms: Vec<f64>,
    build_ms: f64,
    copy_ms: f64,
    hash: u64,
}

fn run(spec: BenchSpec, rule: SubstepRule, steps: u32) -> Run {
    let mut s = bench_scene(spec);
    let mut stats = StepStats::default();
    for _ in 0..WARMUP_STEPS {
        s.apply_load();
        stats = step_with_rule(&mut s.world, rule);
    }
    let mut ms = Vec::with_capacity(steps as usize);
    for _ in 0..steps {
        s.apply_load();
        let t = Instant::now();
        stats = step_with_rule(&mut s.world, rule);
        ms.push(t.elapsed().as_secs_f64() * 1e3);
    }
    // The two parts of a step that are not the solve, timed alone on the final state.
    let reps = 50;
    let t = Instant::now();
    for _ in 0..reps {
        std::hint::black_box(build_islands(&s.world));
    }
    let build_ms = t.elapsed().as_secs_f64() * 1e3 / reps as f64;
    let islands = build_islands(&s.world);
    let t = Instant::now();
    for _ in 0..reps {
        for isl in &islands {
            copy_back(&mut s.world, isl);
        }
    }
    let copy_ms = t.elapsed().as_secs_f64() * 1e3 / reps as f64;
    let hash = state_hash(&s.world);
    Run { stats, ms, build_ms, copy_ms, hash }
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

struct Summary {
    mean: f64,
    median: f64,
    p95: f64,
    max: f64,
}

fn summarize(ms: &[f64]) -> Summary {
    let mut v = ms.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Summary { mean: ms.iter().sum::<f64>() / ms.len() as f64, median: pct(&v, 0.5), p95: pct(&v, 0.95), max: *v.last().unwrap() }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let num = |flag: &str, default: u32| {
        args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(default)
    };
    let steps = num("--steps", 600);
    let repeats = num("--repeats", 3);
    let spec = if args.iter().any(|a| a == "--small") { BenchSpec::SMALL } else { BenchSpec::STANDARD };
    let rules = [("c = 1 (default, max 32)", SubstepRule::DEFAULT), ("c = 0.5 (max 64)", SubstepRule { c: 0.5, max: 64 })];

    println!(
        "scene: {} columns x {} rings x {} nodes: {} nodes, {} beams; {} steps timed after {} warm-up, {} repeats{}",
        spec.columns,
        spec.rings,
        spec.sides,
        spec.node_count(),
        spec.beam_count(),
        steps,
        WARMUP_STEPS,
        repeats,
        if cfg!(debug_assertions) { "  [DEBUG BUILD: numbers meaningless]" } else { "" }
    );
    println!();
    println!(
        "{:<24} {:>8} {:>10} {:>14} {:>9} {:>9} {:>9} {:>9} {:>14}",
        "rule", "islands", "substeps", "beam solves", "mean ms", "median", "p95", "max", "ns / solve"
    );
    let mut json = String::new();
    let mut best_means = Vec::new();
    for (name, rule) in rules {
        let mut best: Option<(Summary, Run)> = None;
        for _ in 0..repeats {
            let r = run(spec, rule, steps);
            let s = summarize(&r.ms);
            if best.as_ref().map_or(true, |(b, _)| s.mean < b.mean) {
                best = Some((s, r));
            }
        }
        let (s, r) = best.unwrap();
        let ns_per_solve = s.mean * 1e6 / r.stats.beam_solves as f64;
        println!(
            "{:<24} {:>8} {:>10} {:>14} {:>9.3} {:>9.3} {:>9.3} {:>9.3} {:>14.1}",
            name, r.stats.islands, r.stats.max_substeps, r.stats.beam_solves, s.mean, s.median, s.p95, s.max, ns_per_solve
        );
        println!(
            "{:<24} build islands {:.3} ms, copy back {:.3} ms ({:.1}% of the step outside the solve); end hash {:016x}",
            "",
            r.build_ms,
            r.copy_ms,
            100.0 * (r.build_ms + r.copy_ms) / s.mean,
            r.hash
        );
        best_means.push(s.mean);
        let _ = write!(
            json,
            "{}{{\"rule\":\"{}\",\"c\":{},\"max\":{},\"islands\":{},\"substeps\":{},\"beam_solves\":{},\"mean_ms\":{:.4},\"median_ms\":{:.4},\"p95_ms\":{:.4},\"max_ms\":{:.4},\"build_ms\":{:.4},\"copy_ms\":{:.4}}}",
            if json.is_empty() { "" } else { "," },
            name,
            rule.c,
            rule.max,
            r.stats.islands,
            r.stats.max_substeps,
            r.stats.beam_solves,
            s.mean,
            s.median,
            s.p95,
            s.max,
            r.build_ms,
            r.copy_ms
        );
    }
    println!();
    println!("c = 0.5 costs {:.2}x the step time of c = 1 on this scene.", best_means[1] / best_means[0]);

    let doc = format!(
        "{{\"beams\":{},\"nodes\":{},\"steps\":{},\"rules\":[{}]}}\n",
        spec.beam_count(),
        spec.node_count(),
        steps,
        json
    );
    let _ = fs::create_dir_all("artifacts/bench");
    let _ = fs::write("artifacts/bench/bench.json", doc);
}
