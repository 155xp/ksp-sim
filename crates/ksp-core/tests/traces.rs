//! Regression tests against in-game KSP 1.12.5 ground truth (SimHarness traces).
//!
//! Traces live in `data/traces/<batch>/` (gitignored, game-derived). Each test is skipped when
//! its trace is absent, so `cargo test` stays green on machines without the data. Run with
//! `KSP_SIM_TRACES=/path/to/traces` to point elsewhere. Bounds are a little above the errors
//! measured when the test was written (see VALIDATION.md); a speed or refactor change that
//! makes any of them fail has changed the physics.

use ksp_core::sim::Params;
use ksp_core::trace::{one_step, touchdowns, windowed, Trace};
use std::path::PathBuf;

fn dir() -> PathBuf {
    std::env::var("KSP_SIM_TRACES")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/traces"))
}

fn load(batch: &str, name: &str) -> Option<Trace> {
    let d = dir().join(batch);
    if !d.join(format!("{name}.csv")).exists() {
        eprintln!("skipping {batch}/{name}: no trace");
        return None;
    }
    Some(Trace::load(&d, name).expect("trace"))
}

fn median(v: &[f64]) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    s[s.len() / 2]
}

/// Airborne span [start, end): from the first 25-tick wheel-free window to the next contact.
fn airborne(t: &Trace) -> (usize, usize) {
    let c = &t.csv;
    let nw = (0..8).filter(|k| c.has(&format!("wc{k}_grounded"))).count();
    let air = |i: usize| (0..nw).all(|k| c.c(&format!("wc{k}_grounded"))[i] < 0.5);
    let mut start = 10;
    if nw > 0 {
        while start + 25 < c.n && !(start..start + 25).all(air) {
            start += 1;
        }
    } else {
        let a0 = c.c("alt")[start];
        while start < c.n && c.c("alt")[start] < a0 + 1.0 {
            start += 1;
        }
    }
    let mut end = start;
    while end < c.n && (nw == 0 || air(end)) {
        end += 1;
    }
    (start, end)
}

struct Bounds {
    accel_median: f64,
    win1_pos_max: f64,
    win1_att_max_deg: f64,
}

fn check_flight(batch: &str, name: &str, b: Bounds) {
    let Some(t) = load(batch, name) else { return };
    let spec = t.spec();
    let prm = Params::default();
    let (s, e) = airborne(&t);
    // stop 1 s short of the first contact: contact transients are checked by check_touchdown
    let e = if e > s + 100 && e < t.csv.n { e - 50 } else { e };
    let (_, ea, _) = one_step(&t, &spec, &prm, s, e);
    let am = median(&ea);
    let w = windowed(&t, &spec, &prm, s, e, 50);
    let pm = w.iter().map(|x| x.1).fold(0.0, f64::max);
    let att = w.iter().map(|x| x.3).fold(0.0, f64::max);
    eprintln!("{batch}/{name}: one-step accel median {am:.6}, 1-s window pos max {pm:.4} m, att max {att:.3} deg");
    assert!(am < b.accel_median, "{name}: one-step accel median {am} >= {}", b.accel_median);
    assert!(pm < b.win1_pos_max, "{name}: 1-s window position error {pm} >= {}", b.win1_pos_max);
    assert!(att < b.win1_att_max_deg, "{name}: 1-s window attitude error {att} >= {}", b.win1_att_max_deg);
}

fn check_touchdown(batch: &str, name: &str, td_pos: f64, end_pos: f64) {
    let Some(t) = load(batch, name) else { return };
    let spec = t.spec();
    let tds = touchdowns(&t, &spec, &Params { crash: false, ..Params::default() }, 100, 250);
    assert!(!tds.is_empty(), "{name}: no touchdown found");
    let td = &tds[0];
    eprintln!("{batch}/{name}: touchdown tick game {} sim {}, pos err at td {:.4} m, after 5 s {:.4} m", td.game_tick, td.sim_tick, td.pos_err_td, td.pos_err_end);
    assert!((td.sim_tick as i64 - td.game_tick as i64).abs() <= 1, "{name}: touchdown tick {} vs {}", td.sim_tick, td.game_tick);
    assert!(td.pos_err_td < td_pos, "{name}: position error at touchdown {} >= {td_pos}", td.pos_err_td);
    assert!(td.pos_err_end < end_pos, "{name}: position error 5 s after touchdown {} >= {end_pos}", td.pos_err_end);
}

#[test]
fn ascent_to_80km() {
    check_flight("v2", "Ascent80", Bounds { accel_median: 2e-4, win1_pos_max: 0.02, win1_att_max_deg: 0.2 });
}

#[test]
fn powered_hover() {
    check_flight("v2", "HopperHover", Bounds { accel_median: 2e-3, win1_pos_max: 0.02, win1_att_max_deg: 0.3 });
    check_touchdown("v2", "HopperHover", 0.02, 0.05);
}

#[test]
fn suicide_burn() {
    check_flight("v2", "HopperSuicide", Bounds { accel_median: 5e-3, win1_pos_max: 0.02, win1_att_max_deg: 0.2 });
    check_touchdown("v2", "HopperSuicide", 0.02, 0.6);
    check_touchdown("v3", "HopperSuicide", 0.02, 0.2);
}

#[test]
fn drop_tests() {
    check_flight("v2", "HopperDrop2", Bounds { accel_median: 1e-3, win1_pos_max: 0.02, win1_att_max_deg: 0.3 });
    check_touchdown("v2", "HopperDrop2", 0.02, 0.5);
}

#[test]
fn pad_landing() {
    check_flight("v6", "HopperPadLand", Bounds { accel_median: 5e-3, win1_pos_max: 0.1, win1_att_max_deg: 2.0 });
    check_touchdown("v6", "HopperPadLand", 0.05, 0.1);
}

#[test]
fn suborbital_hop() {
    check_flight("v2", "HopperHop", Bounds { accel_median: 8e-3, win1_pos_max: 0.1, win1_att_max_deg: 1.5 });
}
