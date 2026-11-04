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
            let gl = if c.has(&format!("p{k}_blx")) {
                let bs = c.c(&format!("p{k}_lift"))[i];
                ksp_core::math::v3(c.c(&format!("p{k}_blx"))[i], c.c(&format!("p{k}_bly"))[i], c.c(&format!("p{k}_blz"))[i]) / if bs != 0.0 { bs } else { 1.0 }
            } else { ksp_core::math::V3::ZERO };
            println!("  part {k}: lift sim ({:+.4},{:+.4},{:+.4}) game ({:+.4},{:+.4},{:+.4})", pa.lift.x, pa.lift.y, pa.lift.z, gl.x, gl.y, gl.z);
            println!("  part {k}: drag sim {:.5} game {:.5} | bodyLiftScalar sim {:.5} game {:.5} | lift |F| {:.5} | dir err {:.3} deg",
                pa.drag, c.c(&format!("p{k}_drag"))[i], bls, c.c(&format!("p{k}_lift"))[i], pa.lift.norm(),
                pa.dir.dot(ksp_core::math::v3(c.c(&format!("p{k}_dvx"))[i], c.c(&format!("p{k}_dvy"))[i], c.c(&format!("p{k}_dvz"))[i]).normalized()).clamp(-1.0, 1.0).acos().to_degrees());
        }
        let mut tot = ksp_core::math::V3::ZERO;
        for k in 0..spec.n_parts as usize {
            let pc = st.q.rotate(spec.part_com[k]);
            let vp = vc + st.w.cross(pc - rc);
            let pa = sim::part_aero(&spec, k, st.q, vp, &atm, &mt);
            let td = (st.q.rotate(spec.part_cop[k]) - rc).cross(pa.dir * (-pa.drag));
            let tl = (st.q.rotate(spec.part_col[k]) - rc).cross(pa.lift);
            tot = tot + td + tl;
            let tdl = st.q.conj().rotate(td);
            let tll = st.q.conj().rotate(tl);
            println!("  part {k} torque root-local: drag ({:+.3},{:+.3},{:+.3}) lift ({:+.3},{:+.3},{:+.3})", tdl.x, tdl.y, tdl.z, tll.x, tll.y, tll.z);
        }
        let tl = st.q.conj().rotate(tot);
        let (_, _, il) = spec.mass_props(&st.res);
        println!("  total aero torque root-local ({:+.3},{:+.3},{:+.3}); I diag ({:.3},{:.3},{:.3}); com_l {:?}", tl.x, tl.y, tl.z, il[0][0], il[1][1], il[2][2], com_l);
        let wl = st.q.conj().rotate(st.w);
        println!("  w root-local {:?}", wl);
        let (_, _, it, own) = spec.mass_props_full(&st.res);
        let mut orb = it;
        for a in 0..3 { for b in 0..3 { orb[a][b] -= own[a][b]; } }
        let iw = ksp_core::math::v3(orb[0][0]*wl.x + orb[0][1]*wl.y + orb[0][2]*wl.z, orb[1][0]*wl.x + orb[1][1]*wl.y + orb[1][2]*wl.z, orb[2][0]*wl.x + orb[2][1]*wl.y + orb[2][2]*wl.z);
        let gy = wl.cross(iw);
        println!("  gyro w x I_orb w (root-local) ({:+.5},{:+.5},{:+.5}); I_orb diag ({:.3},{:.3},{:.3}) offdiag xy {:.4} xz {:.4} yz {:.4}", gy.x, gy.y, gy.z, orb[0][0], orb[1][1], orb[2][2], orb[0][1], orb[0][2], orb[1][2]);
        let _ = (xc, KERBIN);
    }
}
// (torque decomposition appended)
