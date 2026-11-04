//! 1-s window errors over a trace's airborne span: cargo run --example windows_dump -- dir name start end
use ksp_core::sim::Params;
use ksp_core::trace::{windowed, Trace};
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let (s, e): (usize, usize) = (a[3].parse().unwrap(), a[4].parse().unwrap());
    for w in windowed(&t, &spec, &Params::default(), s, e, 50) {
        if w.1 > 0.02 || w.3 > 0.3 {
            println!("window @{}: pos {:.4} vel {:.4} att {:.3} | alt {:.1} ralt {:.1} thr {:.2} grounded {} {} {}", w.0, w.1, w.2, w.3, t.csv.c("alt")[w.0], t.csv.c("ralt")[w.0], t.csv.c("c_thr")[w.0],
                t.csv.c("wc0_grounded")[w.0], t.csv.c("wc1_grounded")[w.0], t.csv.c("wc2_grounded")[w.0]);
        }
    }
}
