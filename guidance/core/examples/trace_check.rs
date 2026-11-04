//! Compare an in-game demo trace (ingame/plugin/TraceLogger.cs, SimHarness format) with ksp-core,
//! tick by tick, with ksp-core's own trace tools (as crates/ksp-core/examples/error_table.rs):
//! one-step acceleration errors and 1-s open-loop windows over the airborne part, the touchdown,
//! and the game's dumped geometry vs this repo's sim craft (`KSPG_CRAFT=heavy` -> heavy_craft).
//! usage: cargo run --release --example trace_check -- <telemetry dir> <trace name>
use ksp_core::sim::Params;
use ksp_core::trace::{one_step, touchdowns, windowed, Trace};
use ksp_core::vessel::VesselSpec;
use ksp_guidance::ingame;
use std::path::Path;

fn pct(v: &[f64], p: f64) -> f64 {
    let mut s: Vec<f64> = v.iter().copied().filter(|x| x.is_finite()).collect();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if s.is_empty() { f64::NAN } else { s[((s.len() - 1) as f64 * p) as usize] }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(Path::new(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let prm = Params::default();
    // geometry: game dump vs the repo's craft
    let mine = if ingame::heavy() { VesselSpec::compile(&ingame::heavy_craft().unwrap()) } else { ingame::built().unwrap().stack };
    let r0 = t.csv.c("lf")[10];
    let res = [r0, t.csv.c("ox")[10], t.csv.c("mono")[10], t.csv.c("ec")[10]];
    let (mg, cg, ig) = spec.mass_props(&res);
    let (mm, cm, im) = mine.mass_props(&res);
    println!("parts game {} sim {}; mass game {:.3} sim {:.3} t; CoM game ({:.3},{:.3},{:.3}) sim ({:.3},{:.3},{:.3})", spec.n_parts, mine.n_parts, mg, mm, cg.x, cg.y, cg.z, cm.x, cm.y, cm.z);
    println!("inertia diag game ({:.0} {:.0} {:.0}) sim ({:.0} {:.0} {:.0}) t m^2", ig[0][0], ig[1][1], ig[2][2], im[0][0], im[1][1], im[2][2]);
    // airborne span (no wheel collider grounded for 25 ticks)
    let c = &t.csv;
    let nw = (0..8).filter(|k| c.has(&format!("wc{k}_grounded"))).count();
    let air = |i: usize| (0..nw).all(|k| c.c(&format!("wc{k}_grounded"))[i] < 0.5);
    let mut s = 10;
    while s + 25 < c.n && !(s..s + 25).all(air) {
        s += 1;
    }
    let mut e = s;
    while e < c.n && air(e) {
        e += 1;
    }
    let e_air = if e > s + 100 && e < c.n { e - 50 } else { e };
    println!("airborne ticks {s}..{e} ({:.1} s)", (e - s) as f64 * 0.02);
    for (name, sp) in [("game geometry", &spec), ("repo craft", &mine)] {
        let (_, ea, ew) = one_step(&t, sp, &prm, s, e_air);
        let w = windowed(&t, sp, &prm, s, e_air, 50);
        let pm = w.iter().map(|x| x.1).fold(0.0, f64::max);
        let am = w.iter().map(|x| x.3).fold(0.0, f64::max);
        println!("[{name}] one-step accel err median {:.2e} p95 {:.2e} m/s^2; ang accel err median {:.2e} p95 {:.2e} rad/s^2; 1-s windows pos max {:.4} m att max {:.3} deg",
            pct(&ea, 0.5), pct(&ea, 0.95), pct(&ew, 0.5), pct(&ew, 0.95), pm, am);
        let tds = touchdowns(&t, sp, &Params { crash: false, ..Params::default() }, 100, 250);
        for td in tds.iter().take(1) {
            println!("[{name}] touchdown tick game {} sim {}; pos err at touchdown {:.4} m, +5 s {:.4} m", td.game_tick, td.sim_tick, td.pos_err_td, td.pos_err_end);
        }
    }
}
