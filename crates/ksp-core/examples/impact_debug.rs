//! Touchdown transient: game vs sim vertical speed and leg jounce, tick by tick.
use ksp_core::sim::{self, Params};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let td: usize = a[3].parse().unwrap();
    let spec = t.spec();
    let c = &t.csv;
    let prm = Params { crash: false, ..Params::default() };
    let start = td - 5;
    let mut st = state_at(&t, &spec, start);
    for i in start..td + 20 {
        sim::tick(&spec, &prm, &mut st, &control(&t, i));
        let k = i + 1;
        let (xc, vc) = com_state(&spec, &st);
        let up = xc.normalized();
        print!("{k}: vz game {:+.3} sim {:+.3} | h err {:+.4} |", c.v("vsrf", k).dot(up), vc.dot(up), (xc - c.v("com", k)).dot(up));
        for l in 0..3 {
            print!(" leg{l} x g{:.3}/s{:.3} F g{:.1}/s{:.1}", c.c(&format!("ws{l}_cd")).get(k + 1).cloned().unwrap_or(f64::NAN), st.legs[l].x, c.c(&format!("wc{l}_force")).get(k + 1).cloned().unwrap_or(f64::NAN), st.legs[l].hit_force);
        }
        println!(" thr {:.2}", c.c("e0_thrust")[i]);
    }
}
