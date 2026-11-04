//! Leg-contact one-step diagnostic.
use ksp_core::sim::{self, Params};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let i: usize = a[3].parse().unwrap();
    let spec = t.spec();
    let c = &t.csv;
    for k in 0..spec.n_legs as usize {
        let l = &spec.legs[k];
        println!("leg {k}: anchor {:?} up {:?} r {} d {}", l.anchor, l.up, l.radius, l.travel);
    }
    let mut st = state_at(&t, &spec, i);
    for k in 0..spec.n_legs as usize {
        println!(" seed leg {k}: {:?}", st.legs[k]);
    }
    let (_, v0) = com_state(&spec, &st);
    sim::tick(&spec, &Params::default(), &mut st, &control(&t, i));
    let (_, v1) = com_state(&spec, &st);
    let up = c.v("com", i).normalized();
    println!("a_up sim {:.4} log {:.4}", (v1 - v0).dot(up) / 0.02, (c.v("vsrf", i + 1) - c.v("vsrf", i)).dot(up) / 0.02);
    for k in 0..spec.n_legs as usize {
        println!(" after leg {k}: x {:.4} F {:.3} k {} c {} tgt {} | log next: cd {:.4} F {:.3} k {} c {}", st.legs[k].x, st.legs[k].hit_force, st.legs[k].k, st.legs[k].c, st.legs[k].target,
            c.c(&format!("ws{k}_cd"))[i + 1], c.c(&format!("wc{k}_force"))[i + 1], c.c(&format!("wc{k}_spring"))[i + 1], c.c(&format!("wc{k}_damper"))[i + 1]);
    }
}
