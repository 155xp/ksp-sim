//! Attitude-dynamics diagnostic: control -> actuators -> angular acceleration, game vs sim.
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
    let c = &t.csv;
    for i in start..start + n {
        let mut st = state_at(&t, &spec, i);
        let w0 = st.w;
        let ctl = control(&t, i);
        let info = sim::tick(&spec, &prm, &mut st, &ctl);
        let wl = -c.v("w", i + 1);
        let al_log = st.q.conj().rotate((wl - w0) / 0.02);
        let al_sim = st.q.conj().rotate((st.w - w0) / 0.02);
        let mut s0 = state_at(&t, &spec, i);
        s0.rw_in = st.rw_in;
        sim::tick(&spec, &prm, &mut s0, &sim::Control { throttle: ctl.throttle, ..Default::default() });
        let al0 = s0.q.conj().rotate((s0.w - w0) / 0.02);
        println!("   no-control alpha ({:+.4},{:+.4},{:+.4})  -> control part ({:+.4},{:+.4},{:+.4})", al0.x, al0.y, al0.z, al_sim.x - al0.x, al_sim.y - al0.y, al_sim.z - al0.z);
        println!("{i} ctl p{:+.3} y{:+.3} r{:+.3} | rw log ({:+.3},{:+.3},{:+.3}) sim ({:+.3},{:+.3},{:+.3}) | gimbal log ({:+.3},{:+.3},{:+.3}) sim ({:+.3},{:+.3}) | alpha log ({:+.4},{:+.4},{:+.4}) sim ({:+.4},{:+.4},{:+.4}) th {:.1}",
            ctl.pitch, ctl.yaw, ctl.roll, c.c("rw0_x")[i], c.c("rw0_y")[i], c.c("rw0_z")[i], st.rw_in.x, st.rw_in.y, st.rw_in.z,
            c.c("g0_ax")[i], c.c("g0_ay")[i], c.c("g0_az")[i], st.gimbal[0][0], st.gimbal[0][1],
            al_log.x, al_log.y, al_log.z, al_sim.x, al_sim.y, al_sim.z, info.thrust);
        let _ = v3(0.0, 0.0, 0.0);
    }
}
