//! Sim thrust direction / application point vs the game's logged thrustTransform, per tick.
use ksp_core::math::{v3, Quat};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let c = &t.csv;
    let e = &spec.engines[0];
    let mut errs = vec![];
    for i in 20..c.n - 1 {
        if c.c("e0_thrust")[i] <= 0.0 {
            continue;
        }
        let st = state_at(&t, &spec, i);
        // the game's actuation at row i is what this tick's thrust used
        let r = e.gimbal_range_deg;
        let gq = Quat::angle_axis(c.c("g0_ax")[i] * r, v3(1.0, 0.0, 0.0)) * Quat::angle_axis(-c.c("g0_az")[i] * r, v3(0.0, 0.0, 1.0));
        let dir = st.q.rotate(gq.rotate(e.dir));
        let ld = c.v("e0_dir", i).normalized();
        errs.push((dir.dot(ld).clamp(-1.0, 1.0).acos().to_degrees(), i));
        if i % 150 == 0 {
            let dl = st.q.conj().rotate(ld - dir);
            println!("tick {i} thrust {:.1} kN | game-sim dir diff in root frame ({:+.2e}, {:+.2e}, {:+.2e}) | gimbal act ({:+.4},{:+.4})", c.c("e0_thrust")[i], dl.x, dl.y, dl.z, c.c("g0_ax")[i], c.c("g0_az")[i]);
        }
    }
    errs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let n = errs.len();
    println!("thrust dir err deg: median {:.5} p90 {:.5} max {:.5} (tick {})", errs[n / 2].0, errs[n * 9 / 10].0, errs[n - 1].0, errs[n - 1].1);
}
