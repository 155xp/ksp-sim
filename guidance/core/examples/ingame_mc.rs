//! Monte Carlo of the in-game demo craft (ingame.rs) from the KSC pad, with dispersions and
//! unknown-to-guidance model errors: `cargo run --release --example ingame_mc -- [n] [seed] [baseline|gfold]`
//! MC_TRACE=k replays flight k with the phase log; MC_VERBOSE=1 prints every flight.
use ksp_guidance::craft::Rng;
use ksp_guidance::ingame;
use ksp_guidance::mission::{run, sample_dispersion, Flight, GuidanceConfig, LandingMode, Outcome, Shared};
use rayon::prelude::*;
use std::sync::Arc;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let n: usize = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(200);
    let seed: u64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let landing = if a.get(3).map(|s| s.as_str()) == Some("gfold") { LandingMode::Gfold } else { LandingMode::Baseline };
    let cfg = GuidanceConfig { landing, ..Default::default() };
    let sh = Arc::new(Shared::new(ingame::built().expect("demo craft builds")));
    let mut rng = Rng(seed);
    let mut flights: Vec<Flight> = (0..n).map(|_| Flight::new(sh.clone(), sample_dispersion(&mut rng, true), cfg)).collect();
    if let Some(k) = std::env::var("MC_TRACE").ok().and_then(|s| s.parse::<usize>().ok()) {
        let mut x = flights.swap_remove(k);
        x.cfg.record = true;
        run(&mut x);
        let mut last = 255.0;
        for r in &x.rec {
            if r[4] != last || std::env::var("MC_ALL").is_ok() {
                println!("  t {:7.1}  ENU ({:9.0} {:9.0} {:8.0})  phase {}  thr {:.2}", r[0], r[1], r[2], r[3], r[4], r[5]);
                last = r[4];
            }
        }
        println!("{:?}\n{:#?}", x.outcome, x.stats);
        return;
    }
    let t0 = std::time::Instant::now();
    flights.par_iter_mut().for_each(run);
    let el = t0.elapsed().as_secs_f64();
    let mut counts = [0usize; 6];
    let mut miss = vec![];
    let mut vz = vec![];
    for (i, x) in flights.iter().enumerate() {
        counts[x.outcome as usize] += 1;
        let s = &x.stats;
        let m = (s.final_e.powi(2) + s.final_n.powi(2)).sqrt();
        if x.outcome == Outcome::Landed {
            miss.push(m);
            vz.push(-s.td_vz);
        }
        if std::env::var("MC_VERBOSE").is_ok() || (x.outcome != Outcome::Landed && i < 2000) && std::env::var("MC_FAILS").is_ok() {
            println!("#{i:<5} {:?} miss {m:7.2} td_vz {:5.2} td_vh {:5.2} tilt {:4.1} ign_err {:6.1} entry {:6.1} bb_end {:6.1} prop {:5.2} sep {:5.0}m {:4.0}m/s apo {:5.0}",
                x.outcome, s.td_vz, s.td_vh, s.td_tilt_deg, s.ign_err, s.entry_err, s.bb_end_err, s.prop_left, s.sep_alt, s.sep_speed, s.apo_alt);
        }
    }
    let pct = |v: &mut Vec<f64>, p: f64| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * p).round() as usize] }
    };
    println!("{} flights in {:.1} s: landed {} crashed {} tipped {} timeout {} noreturn {}", flights.len(), el, counts[1], counts[2], counts[3], counts[4], counts[5]);
    println!("landed miss: p50 {:.2} p90 {:.2} p99 {:.2} max {:.2} m", pct(&mut miss, 0.5), pct(&mut miss, 0.9), pct(&mut miss, 0.99), pct(&mut miss, 1.0));
    println!("touchdown vz: p50 {:.2} p99 {:.2} max {:.2} m/s", pct(&mut vz, 0.5), pct(&mut vz, 0.99), pct(&mut vz, 1.0));
}
