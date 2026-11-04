use ksp_core::aero::mach_terms;
use ksp_core::atmo;
use ksp_core::body::KERBIN;
use ksp_core::math::v3;
use ksp_core::sim;
use ksp_core::vessel::{Craft, VesselSpec};
use std::hint::black_box;
use std::time::Instant;
fn t<F: FnMut(usize) -> f64>(name: &str, n: usize, mut f: F) {
    let s = Instant::now();
    let mut acc = 0.0;
    for i in 0..n { acc += f(i); }
    println!("{name:24} {:.1} ns ({})", s.elapsed().as_nanos() as f64 / n as f64, black_box(acc) as i64 % 7);
}
fn main() {
    let spec = VesselSpec::compile(&Craft::hopper());
    let n = 2_000_000;
    t("mach_terms", n, |i| mach_terms(0.2 + (i % 1000) as f64 * 1e-4).tip);
    let atm = atmo::atm_state(KERBIN, 300.0, 60.0, 0.0);
    let mt = mach_terms(atm.mach);
    let q = sim::attitude(v3(0.26, -0.96, 0.0), v3(0.0, 0.0, 1.0));
    t("part_aero (1 part)", n, |i| sim::part_aero(&spec, i % 6, q, v3(0.1, 55.0 + (i % 10) as f64, 3.0), &atm, &mt).drag);
    t("atm_state", n, |i| atmo::atm_state(KERBIN, 300.0 + (i % 100) as f64, 60.0, 0.0).rho);
    t("pressure_kpa", n, |i| atmo::pressure_kpa(KERBIN, 300.0 + (i % 100) as f64));
    t("mass_props_full", n, |i| spec.mass_props_full(&[180.0 - (i % 50) as f64, 220.0, 10.0, 50.0]).0);
    t("sun_dir_fixed", n / 10, |i| sim::sun_dir_fixed(KERBIN, i as f64).0.x);
}
