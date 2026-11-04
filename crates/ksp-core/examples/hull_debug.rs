use ksp_core::sim::{self, Params};
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let i: usize = a[3].parse().unwrap();
    let spec = t.spec();
    let prm = Params::default();
    let st = state_at(&t, &spec, i);
    for k in 0..spec.n_hull as usize {
        let p = st.x + st.q.rotate(spec.hull[k]);
        println!("hull {k} root-local {:?} height {:.4} tol {}", spec.hull[k], sim::ground_height(&prm, p), spec.hull_tolerance[k]);
    }
    for k in 0..spec.n_legs as usize {
        let p = st.x + st.q.rotate(spec.legs[k].piston_stop());
        println!("piston {k} height {:.4}  foot height {:.4}", sim::ground_height(&prm, p), sim::ground_height(&prm, st.x + st.q.rotate(spec.legs[k].foot())));
    }
}
