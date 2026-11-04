//! Which tick's actuator state does the game's angular acceleration respond to?
//! Fits I*alpha_game - tau_passive = sum over lags of (a_k * tau_rw[i-k] + b_k * tau_gimbal[i-k]).
use ksp_core::aero::mach_terms;
use ksp_core::atmo;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::sim;
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let (s0, s1): (usize, usize) = (a[3].parse().unwrap(), a[4].parse().unwrap());
    let spec = t.spec();
    let c = &t.csv;
    let e = &spec.engines[0];
    let mut rows: Vec<[f64; 7]> = vec![]; // y, rw0, rw1, rw2, g0, g1, g2  (z axis, root-local)
    let tau_rw = |i: usize| -> V3 { v3(-c.c("rw0_x")[i], c.c("rw0_y")[i], c.c("rw0_z")[i]) };
    let tau_g = |i: usize, com_l: V3| -> V3 {
        let r = e.gimbal_range_deg;
        let gq = Quat::angle_axis(c.c("g0_ax")[i] * r, v3(1.0, 0.0, 0.0)) * Quat::angle_axis(-c.c("g0_az")[i] * r, v3(0.0, 0.0, 1.0));
        let f = gq.rotate(e.dir) * c.c("e0_thrust")[i];
        (e.pos - com_l).cross(f)
    };
    for i in s0..s1 {
        let st = state_at(&t, &spec, i);
        let (_, vc) = com_state(&spec, &st);
        let (_, com_l, il, own) = spec.mass_props_full(&st.res);
        let rc = st.q.rotate(com_l);
        let atm = atmo::AtmState { p_kpa: c.c("p_kpa")[i], t: c.c("t_atm")[i], rho: c.c("rho")[i], c: c.c("sos")[i], mach: c.c("mach")[i], q_kpa: c.c("q_kpa")[i],
            pseudo_re_drag_mult: ksp_core::curve::eval(ksp_core::aero::DRAG_PSEUDOREYNOLDS, (c.c("rho")[i] * vc.norm()) as f32 as f64) };
        let mt = mach_terms(atm.mach);
        let mut tau_p = V3::ZERO;
        for k in 0..spec.n_parts as usize {
            let pc = st.q.rotate(spec.part_com[k]);
            let pa = sim::part_aero(&spec, k, st.q, vc + st.w.cross(pc - rc), &atm, &mt);
            tau_p = tau_p + (st.q.rotate(spec.part_cop[k]) - rc).cross(pa.dir * (-pa.drag)) + (st.q.rotate(spec.part_col[k]) - rc).cross(pa.lift);
        }
        let tau_pl = st.q.conj().rotate(tau_p);
        let wl = st.q.conj().rotate(st.w);
        let wl1 = st.q.conj().rotate(-c.v("w", i + 1));
        let al = (wl1 - wl) / 0.02;
        let mut orb = il;
        for x in 0..3 { for y in 0..3 { orb[x][y] -= own[x][y]; } }
        let mv = |m: &[[f64; 3]; 3], v: V3| v3(m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z, m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z, m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z);
        let lhs = mv(&il, al) + wl.cross(mv(&orb, wl)) - tau_pl;
        for (axis, pick) in [(0usize, 'x'), (2, 'z')] {
            let g = |v: V3| if axis == 0 { v.x } else { v.z };
            let _ = pick;
            rows.push([g(lhs), g(tau_rw(i)), g(tau_rw(i - 1)), g(tau_rw(i - 2)), g(tau_g(i, com_l)), g(tau_g(i - 1, com_l)), g(tau_g(i - 2, com_l))]);
        }
    }
    // least squares over subsets
    for (name, cols) in [("rw0+g0", vec![1, 4]), ("rw1+g1", vec![2, 5]), ("rw0+g1", vec![1, 5]), ("rw1+g0", vec![2, 4]), ("rw1+g2", vec![2, 6]), ("rw2+g2", vec![3, 6]), ("all", vec![1, 2, 3, 4, 5, 6])] {
        let n = cols.len();
        let mut ata = vec![vec![0.0; n]; n];
        let mut aty = vec![0.0; n];
        for r in &rows {
            for p in 0..n { aty[p] += r[cols[p]] * r[0]; for q in 0..n { ata[p][q] += r[cols[p]] * r[cols[q]]; } }
        }
        // solve by Gauss
        let mut m = ata.clone(); let mut b = aty.clone();
        for p in 0..n { let piv = m[p][p]; for q in p..n { m[p][q] /= piv; } b[p] /= piv; for r2 in 0..n { if r2 != p { let f = m[r2][p]; for q in p..n { m[r2][q] -= f * m[p][q]; } b[r2] -= f * b[p]; } } }
        let rms = (rows.iter().map(|r| { let pr: f64 = (0..n).map(|p| b[p] * r[cols[p]]).sum(); (r[0] - pr).powi(2) }).sum::<f64>() / rows.len() as f64).sqrt();
        println!("{name:8}: coef {:?} rms {rms:.4}", b.iter().map(|x| (x * 1000.0).round() / 1000.0).collect::<Vec<_>>());
    }
}
