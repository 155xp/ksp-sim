//! Heavy booster geometry check: leg feet vs nozzle bottom, mass, settle on the pad.
use ksp_core::sim::{self, Control, Params};
use ksp_guidance::{ingame, nav};
fn main() {
    let c = ingame::heavy_craft().unwrap();
    let spec = ksp_core::vessel::VesselSpec::compile(&c);
    let res = spec.full_resources();
    let (m, com, _) = spec.mass_props(&res);
    let feet: Vec<f64> = (0..spec.n_legs as usize).map(|k| spec.legs[k].foot().y).collect();
    let e = &spec.engines[0];
    let noz = (0..e.n_nozzles as usize).map(|j| e.nozzles[j].y).fold(f64::INFINITY, f64::min);
    println!("parts {} mass {:.1} t (dry {:.1}) com_y {:.2} bottom_y {:.2} nozzle_y {:.2} feet_y {:?} hull {}", spec.n_parts, m, spec.dry_mass(), com.y, spec.bottom_y, noz, feet.iter().map(|f| (f * 100.0).round() / 100.0).collect::<Vec<_>>(), spec.n_hull);
    let prm = Params::default();
    let mut st = sim::state_landed(&spec, 1000.0);
    for _ in 0..500 {
        sim::tick(&spec, &prm, &mut st, &Control::default());
    }
    let n = nav::nav(&spec, &st);
    println!("settled: status {} alt {:.3} tilt {:.3} deg", st.status, n.alt, n.up.dot(n.x.normalized()).acos().to_degrees());
}
