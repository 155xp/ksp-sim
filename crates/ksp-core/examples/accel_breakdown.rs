//! One-step acceleration error decomposed by phase: cargo run --release -p ksp-core --example accel_breakdown -- <dir> <test>
use ksp_core::sim::{self, Params};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let prm = Params::default();
    let c = &t.csv;
    let mut bins: std::collections::BTreeMap<String, Vec<(f64, f64, f64, f64)>> = Default::default();
    for i in 12..c.n - 1 {
        let mut st = state_at(&t, &spec, i);
        let info = sim::tick(&spec, &prm, &mut st, &control(&t, i));
        let (_, v) = com_state(&spec, &st);
        let dv = (v - c.v("vsrf", i + 1)) / sim::FIXED_DT;
        let vel = c.v("vsrf", i);
        let up = c.v("up", i).normalized();
        let vhat = vel.normalized();
        let alt = c.c("alt")[i];
        let thrust = c.c("e0_thrust")[i];
        let phase = format!("{}{:>6.0}km{}", if thrust > 0.0 { "burn " } else { "coast" }, (alt / 10000.0).floor() * 10.0, if c.c("p_kpa")[i] > 0.0 { "" } else { " vac" });
        bins.entry(phase).or_default().push((dv.norm(), dv.dot(vhat), dv.dot(up), info.thrust - thrust));
    }
    println!("| phase | ticks | |da| median | along v | along up | thrust err (kN) median |");
    for (k, v) in bins {
        let med = |j: usize| { let mut s: Vec<f64> = v.iter().map(|x| [x.0, x.1, x.2, x.3][j]).collect(); s.sort_by(|a, b| a.partial_cmp(b).unwrap()); s[s.len() / 2] };
        println!("| {k} | {} | {:.5} | {:+.5} | {:+.5} | {:+.4} |", v.len(), med(0), med(1), med(2), med(3));
    }
}
