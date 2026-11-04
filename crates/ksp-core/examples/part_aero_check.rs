//! Per-part drag (part.dragScalar) and body-lift scale vs the game, at logged states.
use ksp_core::aero::{mach_terms, BODY_LIFT_MULTIPLIER};
use ksp_core::atmo;
use ksp_core::body::KERBIN;
use ksp_core::sim;
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let ticks: Vec<usize> = a[3..].iter().map(|x| x.parse().unwrap()).collect();
    let spec = t.spec();
    let c = &t.csv;
    for &i in &ticks {
        let st = state_at(&t, &spec, i);
        let (xc, vc) = com_state(&spec, &st);
        // atmosphere as the game had it (logged vessel values)
        let atm = atmo::AtmState { p_kpa: c.c("p_kpa")[i], t: c.c("t_atm")[i], rho: c.c("rho")[i], c: c.c("sos")[i], mach: c.c("mach")[i], q_kpa: c.c("q_kpa")[i],
            pseudo_re_drag_mult: { let s = vc.norm(); ksp_core::curve::eval(ksp_core::aero::DRAG_PSEUDOREYNOLDS, (c.c("rho")[i] * s) as f32 as f64) } };
        let mt = mach_terms(atm.mach);
        println!("tick {i}: alt {:.1} mach {:.3} rho {:.4}", c.c("alt")[i], atm.mach, atm.rho);
        let (_, com_l, _) = spec.mass_props(&st.res);
        let rc = st.q.rotate(com_l);
        for k in 0..spec.n_parts as usize {
            let pc = st.q.rotate(spec.part_com[k]);
            let vp = vc + st.w.cross(pc - rc);
            let pa = sim::part_aero(&spec, k, st.q, vp, &atm, &mt);
            let bls = pa.q * spec.part_body_lift[k] * BODY_LIFT_MULTIPLIER * mt.body_lift_mach;
            println!("  part {k}: drag sim {:.5} game {:.5} | bodyLiftScalar sim {:.5} game {:.5} | lift |F| {:.5} | dir err {:.3} deg",
                pa.drag, c.c(&format!("p{k}_drag"))[i], bls, c.c(&format!("p{k}_lift"))[i], pa.lift.norm(),
                pa.dir.dot(ksp_core::math::v3(c.c(&format!("p{k}_dvx"))[i], c.c(&format!("p{k}_dvy"))[i], c.c(&format!("p{k}_dvz"))[i]).normalized()).clamp(-1.0, 1.0).acos().to_degrees());
        }
        let _ = (xc, KERBIN);
    }
}
