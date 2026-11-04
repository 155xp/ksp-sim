//! Game wheel-hit points vs the sim ground: height of each logged WheelHit.point above the sim
//! terrain (should be ~0 if the sim ground matches the game's collider surface there).
use ksp_core::sim::Params;
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let prm = Params::default();
    for pair in a[1..].chunks(2) {
        let t = Trace::load(&PathBuf::from(&pair[0]), &pair[1]).unwrap();
        let c = &t.csv;
        let ksc = ksp_core::ksc::KSC_PAD_DECK_BODY_FIXED;
        println!("{}/{}: ksc_dz {:+.4} m", pair[0], pair[1], t.ksc_dz);
        for k in 0..8 {
            if !c.has(&format!("wc{k}_hx")) {
                break;
            }
            let (mut s, mut nn, mut lo, mut hi) = (0.0, 0, f64::MAX, f64::MIN);
            let mut last = None;
            for i in 0..c.n {
                if c.c(&format!("wc{k}_grounded"))[i] > 0.5 {
                    let h = ksp_core::math::v3(c.c(&format!("wc{k}_hx"))[i], c.c(&format!("wc{k}_hy"))[i], c.c(&format!("wc{k}_hz"))[i]);
                    let sc = if std::env::var("HG_RAW").is_ok() { ksp_core::sim::Scenery::default() } else { ksp_core::sim::Scenery { shift: t.gshift[i], ksc_dz: t.ksc_dz } };
                    let g = ksp_core::sim::ground_height_sc(&prm, &sc, h);
                    s += g; nn += 1; lo = lo.min(g); hi = hi.max(g);
                    last = Some(h);
                    if std::env::var("HG_SERIES").is_ok() && k == 0 && (nn < 12 || nn % 25 == 0) {
                        let d = h - ksc;
                        let e = d.dot(ksp_core::math::v3(0.96389896173181655, 0.26626826993152286, 0.0));
                        let no = d.dot(ksp_core::math::v3(-0.00045174951374689527, 0.0016353465149094525, -0.99999856078104088));
                        println!("   row {i}: e {e:.3} n {no:.3} h {g:+.5} cd {:.4} F {:.2}", c.c("ws0_cd").get(i).cloned().unwrap_or(f64::NAN), c.c("wc0_force")[i]);
                    }
                }
            }
            if let Some(h) = last {
                let d = h - ksc;
                println!("{}/{} leg {k}: n {nn} hit height above sim ground mean {:+.4} m [{:+.4}, {:+.4}], last hit {:.2} m from pad centre", pair[0], pair[1], s / nn as f64, lo, hi, (d - ksc.normalized() * d.dot(ksc.normalized())).norm());
            }
        }
    }
}
