//! Scene statistics (milestone 3, phase 3.9): replays recordings (`.rcrd`, see `src/recording.ts`)
//! natively and reports nodes, beams, substeps, ground contacts and milliseconds per step, and
//! whether the final hash equals the recording's. The browser page cannot show contact counts
//! (the boundary does not carry them), and a replay is the same commands, so the counts are the
//! page's too. Native timing only; the core itself never reads a clock.
//!
//! Usage: cargo run --release -p beam-core --example scene_stats -- <file.rcrd>...
//! Debug builds are far slower; only a release number means anything.

use beam_core::command::{apply_commands, Command, COMMAND_BYTES};
use beam_core::real::Real;
use beam_core::sim::{state_hash, step_with_rule};
use beam_core::solver::SubstepRule;
use beam_core::world::World;
use std::fs;
use std::time::Instant;

const HEADER_BYTES: usize = 40;

fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}

fn command_at(b: &[u8]) -> Command {
    let w = std::mem::size_of::<Real>();
    let mut f = [0.0; 6];
    for (i, x) in f.iter_mut().enumerate() {
        *x = Real::from_le_bytes(b[16 + i * w..16 + (i + 1) * w].try_into().unwrap());
    }
    Command { kind: u32_at(b, 0), a: u32_at(b, 4), b: u32_at(b, 8), reserved: u32_at(b, 12), f }
}

/// The command lists of a recording (`steps + 1` of them) and its final hash.
fn read_recording(bytes: &[u8]) -> (Vec<Vec<Command>>, u64) {
    assert_eq!(&bytes[0..4], b"RCRD", "not a recording");
    assert_eq!(u32_at(bytes, 12) as usize, COMMAND_BYTES, "record size differs from this build");
    let steps = u32_at(bytes, 16) as usize;
    let cps = u32_at(bytes, 24) as usize;
    let hash = u64::from_le_bytes(bytes[32..40].try_into().unwrap());
    let mut o = HEADER_BYTES + cps * 8;
    let mut lists = Vec::with_capacity(steps + 1);
    for _ in 0..=steps {
        let n = u32_at(bytes, o) as usize;
        o += 4;
        lists.push((0..n).map(|i| command_at(&bytes[o + i * COMMAND_BYTES..])).collect());
        o += n * COMMAND_BYTES;
    }
    assert_eq!(o, bytes.len(), "extra bytes at the end");
    (lists, hash)
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn main() {
    let files: Vec<String> = std::env::args().skip(1).collect();
    if files.is_empty() {
        eprintln!("usage: scene_stats <file.rcrd>...");
        std::process::exit(2);
    }
    if cfg!(debug_assertions) {
        println!("[DEBUG BUILD: numbers meaningless]");
    }
    println!(
        "{:<22} {:>6} {:>6} {:>6} {:>8} {:>8} {:>9} {:>9} {:>9} {:>9} {:>9} {:>8} {:>7}",
        "run", "nodes", "beams", "steps", "max sub", "mean ctc", "peak ctc", "peak bm", "mean ms", "p95 ms", "max ms", "ctc/beam", "hash"
    );
    for f in &files {
        let bytes = fs::read(f).expect("cannot read file");
        let (lists, want) = read_recording(&bytes);
        let mut world = World::new();
        let mut results = Vec::new();
        apply_commands(&mut world, &lists[0], &mut results);
        let (mut ms, mut max_sub, mut contacts, mut peak, mut peak_bm) = (Vec::new(), 0, 0u64, 0, 0);
        // Contact solves (each contact once per substep) against beam solves, summed over the run.
        let (mut ctc_solves, mut beam_solves) = (0u64, 0u64);
        for list in &lists[1..] {
            // `lists[k]` is applied before step k, the setup (list 0) before step 0 (see the layout above).
            apply_commands(&mut world, list, &mut results);
            let t = Instant::now();
            let s = step_with_rule(&mut world, SubstepRule::DEFAULT);
            ms.push(t.elapsed().as_secs_f64() * 1e3);
            max_sub = max_sub.max(s.max_substeps);
            contacts += (s.contacts + s.beam_contacts) as u64;
            ctc_solves += (s.contacts + s.beam_contacts) as u64 * s.max_substeps as u64;
            beam_solves += s.beam_solves;
            peak = peak.max(s.contacts);
            peak_bm = peak_bm.max(s.beam_contacts);
        }
        let mut v = ms.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let name = f.rsplit(['/', '\\']).next().unwrap_or(f).trim_end_matches(".rcrd");
        println!(
            "{:<22} {:>6} {:>6} {:>6} {:>8} {:>8.1} {:>9} {:>9} {:>9.3} {:>9.3} {:>9.3} {:>7.1}% {:>7}",
            name,
            world.node_count(),
            world.beam_count(),
            ms.len(),
            max_sub,
            contacts as f64 / ms.len() as f64,
            peak,
            peak_bm,
            ms.iter().sum::<f64>() / ms.len() as f64,
            pct(&v, 0.95),
            v.last().unwrap(),
            100.0 * ctc_solves as f64 / beam_solves.max(1) as f64,
            if state_hash(&world) == want { "equal" } else { "DIFFERS" },
        );
    }
}
