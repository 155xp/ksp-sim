//! Ablation profile of sim::tick (single thread): ns per tick with subsystems toggled.
use ksp_core::math::{v3, V3};
use ksp_core::sim::{self, Control, Params};
use ksp_core::vessel::{Craft, VesselSpec};
use std::time::Instant;
fn run(spec: &VesselSpec, prm: &Params, st0: &sim::EnvState, ctl: &Control, n: usize) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        let mut st = *st0;
        let t = Instant::now();
        for _ in 0..n {
            sim::tick(spec, prm, &mut st, ctl);
            if st.ticks % 200 == 0 { st = *st0; }
        }
        best = best.min(t.elapsed().as_nanos() as f64 / n as f64);
    }
    best
}
fn main() {
    let spec = VesselSpec::compile(&Craft::hopper());
    let (_e, _n, u) = sim::pad_enu();
    let q = sim::attitude(u, v3(0.0, 0.0, 1.0));
    let air = sim::state_at_pad(&spec, v3(10.0, 5.0, 800.0), v3(1.0, 0.0, -40.0), q, V3::ZERO, 0.0);
    let ground = sim::state_landed(&spec, 0.0);
    let ctl = Control { throttle: 0.6, pitch: 0.1, yaw: -0.05, roll: 0.0 };
    let base = Params::default();
    let n = 200_000;
    for (label, st) in [("airborne", air), ("landed", ground)] {
        println!("{label}: full {:.0} ns", run(&spec, &base, &st, &ctl, n));
        println!("  no aero         {:.0} ns", run(&spec, &Params { aero: false, ..base }, &st, &ctl, n));
        println!("  no sun temp     {:.0} ns", run(&spec, &Params { sun_temperature: false, ..base }, &st, &ctl, n));
        println!("  no ground       {:.0} ns", run(&spec, &Params { ground: false, ..base }, &st, &ctl, n));
        println!("  sphere terrain  {:.0} ns", run(&spec, &Params { terrain: false, ..base }, &st, &ctl, n));
    }
    // the same hopper with 4 airbrakes on its tank (ModuleAeroSurface path)
    let mut c = Craft::hopper();
    for k in 0..4 {
        c.surface_attach_editor(1, ksp_core::parts::airbrake(), 90.0 * k as f64, 0.625, 0.5, ksp_core::parts::AIRBRAKE_NODE.0, ksp_core::parts::AIRBRAKE_NODE.1);
    }
    let ab = VesselSpec::compile(&c);
    let mut st = sim::state_at_pad(&ab, v3(10.0, 5.0, 800.0), v3(1.0, 0.0, -40.0), q, V3::ZERO, 0.0);
    println!("airborne + 4 airbrakes stowed {:.0} ns", run(&ab, &base, &st, &ctl, n));
    st.brakes = true;
    st.brake_deflection = 70.0;
    println!("airborne + 4 airbrakes deployed {:.0} ns", run(&ab, &base, &st, &ctl, n));
    let t = Instant::now();
    let mut acc = 0.0;
    for k in 0..n { acc += spec.mass_props_full(&[180.0 - (k % 100) as f64, 220.0, 10.0, 50.0]).0; }
    println!("mass_props_full {:.0} ns ({acc:.0})", t.elapsed().as_nanos() as f64 / n as f64);
}
