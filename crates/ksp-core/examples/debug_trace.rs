//! Per-tick diagnostic of a replay: cargo run --release -p ksp-core --example debug_trace -- <dir> <test> <start> <n>
use ksp_core::math::v3;
use ksp_core::sim::{self, Params};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let start: usize = a[3].parse().unwrap();
    let n: usize = a[4].parse().unwrap();
    let spec = t.spec();
    let prm = Params::default();
    let mut st = state_at(&t, &spec, start);
    let c = &t.csv;
    let (m0, com0, _) = spec.mass_props(&st.res);
    println!("spec mass {m0:.5} log mass {:.5}; com_l {:?}", c.c("mass")[start], com0);
    let rc = st.q.rotate(com0);
    println!("sim com {:?}\nlog com {:?}\nsim vcom {:?}\nlog vsrf {:?}", st.x + rc, c.v("com", start), st.v + st.w.cross(rc), c.v("vsrf", start));
    println!("log root {:?} rootv {:?}", c.v("root", start), c.v("rootv", start));
    for i in start..start + n {
        let ctl = control(&t, i);
        let vprev = c.v("vsrf", i);
        let (_, com_l, _) = spec.mass_props(&st.res);
        let rc = st.q.rotate(com_l);
        let vs0 = st.v + st.w.cross(rc);
        let info = sim::tick(&spec, &prm, &mut st, &ctl);
        let (_, com_l, _) = spec.mass_props(&st.res);
        let rc = st.q.rotate(com_l);
        let vs1 = st.v + st.w.cross(rc);
        let up = (c.v("com", i)).normalized();
        let alog = (c.v("vsrf", i + 1) - vprev) / 0.02;
        let asim = (vs1 - vs0) / 0.02;
        println!("{i}: thr c={:.3} log_e={:.3} | thrust log {:.3} sim {:.3} | a_up log {:.4} sim {:.4} | a_h log {:.4} sim {:.4} | dir log {:?} q {:.4} {:.4}",
            ctl.throttle, c.c("e0_thr")[i], c.c("e0_thrust")[i], info.thrust, alog.dot(up), asim.dot(up),
            (alog - up * alog.dot(up)).norm(), (asim - up * asim.dot(up)).norm(), c.v("e0_dir", i), c.c("q_kpa")[i], info.atm.q_kpa);
        let _ = v3(0.0, 0.0, 0.0);
    }
}
