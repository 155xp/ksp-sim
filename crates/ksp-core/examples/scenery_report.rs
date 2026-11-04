//! Scenery placement per trace: per-session KSC offset and the PQS translation statistics.
//! cargo run --release --example scenery_report -- <dir/name> ...
use ksp_core::trace::Trace;
use std::path::Path;
fn main() {
    let up = ksp_core::ksc::KSC_PAD_DECK_BODY_FIXED.normalized();
    let mut all = vec![];
    for a in std::env::args().skip(1) {
        let p = Path::new(&a);
        let t = Trace::load(p.parent().unwrap(), p.file_name().unwrap().to_str().unwrap()).unwrap();
        let r: Vec<f64> = t.gshift.iter().map(|g| g.dot(up)).collect();
        let (lo, hi) = r.iter().fold((f64::MAX, f64::MIN), |(l, h), &x| (l.min(x), h.max(x)));
        println!("{a}: ksc_dz {:+.4} m | PQS radial at spawn {:+.4}, at end {:+.4}, range [{lo:+.4}, {hi:+.4}] | |shift| max {:.2} m", t.ksc_dz, r[0], r[r.len() - 1], t.gshift.iter().map(|g| g.norm()).fold(0.0, f64::max));
        all.push(t.ksc_dz + r[0]);
    }
    println!("deck offset at spawn (ksc_dz + PQS radial), vs survey: {:?}", all.iter().map(|x| format!("{x:+.4}")).collect::<Vec<_>>());
}
