//! One-step errors per row: cargo run --example onestep_dump -- dir name start end stride
use ksp_core::sim::Params;
use ksp_core::trace::{one_step, Trace};
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    if std::env::var("NO_SCENERY").is_ok() {
        t.gshift.iter_mut().for_each(|g| *g = ksp_core::math::V3::ZERO);
        t.ksc_dz = 0.0;
    }
    let spec = t.spec();
    let (s, e, k): (usize, usize, usize) = (a[3].parse().unwrap(), a[4].parse().unwrap(), a[5].parse().unwrap());
    let (ks, ea, ew) = one_step(&t, &spec, &Params::default(), s, e);
    let c = &t.csv;
    for j in (0..ks.len()).step_by(k) {
        let i = ks[j];
        println!("row {i}: accel err {:.4} ang err {:.4} | pitch {:+.3} yaw {:+.3} roll {:+.3} thr {:.2} | w {:.4} {:.4} {:.4}", ea[j], ew[j], c.c("c_pitch")[i], c.c("c_yaw")[i], c.c("c_roll")[i], c.c("c_thr")[i], c.c("w_x")[i], c.c("w_y")[i], c.c("w_z")[i]);
    }
}
