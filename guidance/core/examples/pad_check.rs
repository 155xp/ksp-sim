use ksp_core::sim::{self, Control, Params};
use ksp_guidance::{ingame, nav};
fn main() {
    let b = ingame::built().unwrap();
    for (name, spec) in [("stack", &b.stack), ("booster", &b.booster)] {
        let prm = Params::default();
        let mut st = sim::state_landed(spec, 1000.0);
        println!("{name}: n_parts {} bottom_y {:.3} legs {} status {} terrain {} crash {}", spec.n_parts, spec.bottom_y, spec.n_legs, st.status, prm.terrain, prm.crash);
        for k in 0..250 {
            let info = sim::tick(spec, &prm, &mut st, &Control::default());
            if k % 50 == 0 || st.status == 3 {
                let n = nav::nav(spec, &st);
                println!("  tick {k} status {} impact {:.2} alt {:.3} thrust {:.1}", st.status, st.impact_speed, n.alt, info.thrust);
            }
            if st.status == 3 { break; }
        }
    }
}
