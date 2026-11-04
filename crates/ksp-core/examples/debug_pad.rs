use ksp_core::{math::v3, sim::*, vessel::*};
fn main() {
    let spec = VesselSpec::compile(&Craft::hopper());
    let (_e, _n, u) = pad_enu();
    let mut st = state_landed(&spec, 0.0);
    println!("up check {:?} vs {:?}", st.q.rotate(v3(0.0,1.0,0.0)), u);
    for k in 0..spec.n_legs as usize { println!("leg {k} foot {:?} axis {:?}", spec.legs[k].foot, spec.legs[k].axis); }
    for k in 0..spec.n_hull as usize { println!("hull {k} {:?}", spec.hull[k]); }
    println!("bottom_y {}", spec.bottom_y);
    let prm = Params::default();
    for i in 0..600 {
        let inf = tick(&spec, &prm, &mut st, &Control::default());
        if i % 50 == 0 { println!("t{i} alt {:.4} legs {} up {:.5} wn {:.2e} vs {:.2e} st {} imp {}", inf.alt, inf.legs_in_contact, st.q.rotate(v3(0.0,1.0,0.0)).dot(u), st.w.norm(), inf.v_srf.norm(), st.status, st.impact_speed); }
    }
}
