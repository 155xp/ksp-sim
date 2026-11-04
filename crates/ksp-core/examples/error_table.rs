//! Per-test error table against the in-game traces (VALIDATION.md section 6).
//! cargo run --release -p ksp-core --example error_table -- [traces dir]
use ksp_core::sim::Params;
use ksp_core::trace::{one_step, replay_attitude_locked, touchdowns, windowed, Errors, Trace};
use std::path::PathBuf;

const CASES: &[(&str, &str, &str)] = &[
    ("v2", "Ascent80", "ascent to 80 km"),
    ("v2", "HopperHover", "hover 30 m, land"),
    ("v2", "HopperDrop2", "drop onto pad (9.2 m/s)"),
    ("v2", "HopperSuicide", "suicide burn, 16 m/s nozzle strike"),
    ("v3", "HopperSuicide", "suicide burn to pad"),
    ("v7", "HopperSuicide", "suicide burn to pad (placement logged)"),
    ("v6", "HopperPadLand", "hop + pad landing"),
    ("v7", "HopperPadLand", "hop + pad landing (placement logged)"),
    ("v2", "HopperHop", "suborbital hop (flight)"),
    ("v7", "HopperHop", "hop, lands on sloped grass, tips over"),
];

fn pct(v: &[f64], p: f64) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    s[((s.len() as f64 - 1.0) * p) as usize]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let root = PathBuf::from(args.get(1).cloned().unwrap_or_else(|| "data/traces".into()));
    let prm = Params::default();
    println!("| test | trace | airborne s | one-step accel err median / p95 (m/s^2) | ang. accel err median (rad/s^2) | 1-s windows: pos max (m), att max (deg) | touchdown tick (game/sim) | pos err at touchdown / +2 s / +5 s (m) |");
    println!("|---|---|---|---|---|---|---|---|");
    for (b, n, what) in CASES {
        let d = root.join(b);
        if !d.join(format!("{n}.csv")).exists() {
            continue;
        }
        let t = Trace::load(&d, n).unwrap();
        let spec = t.spec();
        let c = &t.csv;
        let nw = (0..8).filter(|k| c.has(&format!("wc{k}_grounded"))).count();
        let air = |i: usize| (0..nw).all(|k| c.c(&format!("wc{k}_grounded"))[i] < 0.5);
        let mut s = 10;
        if nw > 0 {
            while s + 25 < c.n && !(s..s + 25).all(air) {
                s += 1;
            }
        } else {
            let a0 = c.c("alt")[s];
            while s < c.n && c.c("alt")[s] < a0 + 1.0 {
                s += 1;
            }
        }
        let mut e = s;
        while e < c.n && (nw == 0 || air(e)) {
            e += 1;
        }
        let e_air = if e > s + 100 && e < c.n { e - 50 } else { e };
        let (_, ea, ew) = one_step(&t, &spec, &prm, s, e_air);
        let w = windowed(&t, &spec, &prm, s, e_air, 50);
        let pm = w.iter().map(|x| x.1).fold(0.0, f64::max);
        let am = w.iter().map(|x| x.3).fold(0.0, f64::max);
        let pre = if *n == "HopperHop" { 25 } else { 100 };
        let tds = touchdowns(&t, &spec, &Params { crash: false, ..Params::default() }, pre, 250);
        let tds2 = touchdowns(&t, &spec, &Params { crash: false, ..Params::default() }, pre, 100);
        let (tdc, tde) = match (tds.first(), tds2.first()) {
            (Some(a), Some(b2)) => (format!("{} / {}", a.game_tick, a.sim_tick), format!("{:.4} / {:.4} / {:.4}", a.pos_err_td, b2.pos_err_end, a.pos_err_end)),
            _ => ("-".into(), "-".into()),
        };
        println!(
            "| {what} | {b}/{n} | {:.1} | {:.2e} / {:.2e} | {:.2e} | {:.4}, {:.3} | {tdc} | {tde} |",
            (e - s) as f64 * 0.02,
            pct(&ea, 0.5),
            pct(&ea, 0.95),
            pct(&ew, 0.5),
            pm,
            am
        );
        if *n == "Ascent80" {
            let r = replay_attitude_locked(&t, &spec, &prm, s, e);
            eprintln!("Ascent80 attitude-locked full replay ({:.0} s): pos max {:.2} m, vel max {:.3} m/s, LF err end {:.3}", (e - s) as f64 * 0.02, Errors::max(&r.pos), Errors::max(&r.vel), r.fuel.last().unwrap());
        }
    }
}
