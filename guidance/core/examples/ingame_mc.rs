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
    let margin: Option<f64> = std::env::var("MC_MARGIN").ok().and_then(|s| s.parse().ok());
    let mut flights: Vec<Flight> = (0..n)
        .map(|_| {
            let mut d = sample_dispersion(&mut rng, true);
            if std::env::var("MC_NOMINAL").is_ok() {
                // the in-game flight: nominal ascent; model errors, separation kicks, time of day vary
                let nd = ingame::nominal_dispersion(d.ut);
                let drag_scale: f64 = std::env::var("MC_DRAG_SCALE").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
                d = ksp_guidance::mission::Dispersion { sep_dv: d.sep_dv, sep_w: d.sep_w, thrust_err: d.thrust_err, drag_err: d.drag_err * drag_scale, ..nd };
            }
            if let Some(m) = margin {
                d.meco_margin = m;
            }
            Flight::new(sh.clone(), d, cfg)
        })
        .collect();
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
    if std::env::var("MC_BRIDGE").is_ok() {
        // the guidance sees only what Bridge.cs sends (round trip through the bridge mapping) and
        // runs through Flight::step_external; the sim plays the game
        flights.par_iter_mut().for_each(|f| fly_via_bridge(f));
    } else {
        flights.par_iter_mut().for_each(run);
    }
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
            println!("#{i:<5} {:?} miss {m:7.2} td_vz {:5.2} td_vh {:5.2} tilt {:4.1} ign_err {:6.1} entry {:6.1} bb_end {:6.1} prop {:5.2} sep {:5.0}m {:4.0}m/s apo {:5.0} margin {:.2} load {:.2} kick {:.0}/{:.1} turn {:.0}/{:.2} ut {:.0} derr {:.3} terr {:.3} sepdv {:.2}",
                x.outcome, s.td_vz, s.td_vh, s.td_tilt_deg, s.ign_err, s.entry_err, s.bb_end_err, s.prop_left, s.sep_alt, s.sep_speed, s.apo_alt, x.disp.meco_margin, x.disp.fuel_load, x.disp.kick_speed, x.disp.kick_deg, x.disp.turn_alt, x.disp.turn_exp, x.disp.ut % 21600.0, x.disp.drag_err, x.disp.thrust_err, x.disp.sep_dv.norm());
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

fn fly_via_bridge(f: &mut Flight) {
    use ksp_core::sim;
    let mut truth = f.clone();
    let mut g = f.clone();
    g.external = true;
    while !g.done() {
        let spec = *g.spec();
        let x = ingame::bridge_from_state(&truth.st, &spec);
        let mut st = ingame::state_from_bridge(&x, &spec);
        // contact / crash status as the sim decided it (in the game: landed flag, vessel lost)
        st.status = truth.st.status;
        let ctl = g.step_external(st, 0.0);
        if g.done() {
            break;
        }
        // the sim plays the game: same truth spec as the guidance's current configuration
        truth.st.brakes = g.st.brakes;
        let tspec = if g.phase == ksp_guidance::mission::Phase::Ascent { truth.truth_stack } else { truth.truth_booster };
        sim::tick(&tspec, &truth.prm, &mut truth.st, &ctl);
    }
    *f = g;
}
