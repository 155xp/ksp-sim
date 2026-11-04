//! Sim prediction for the in-game crash series (harness/kos/harness/crashdrop.ks): the hopper
//! (in-game geometry from a trace) hovers still at h = v^2 / 2g above its resting height, the
//! engine is cut, it falls onto the pad. Prints crashed / survived and the impact speed.
//! cargo run --release --example crash_predict -- data/traces/v2 HopperDrop2
use ksp_core::math::{v3, V3};
use ksp_core::sim::{self, Control, Params};
use ksp_core::trace::{com_state, Trace};
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let prm = Params::default();
    let (e, n, u) = sim::pad_enu();
    let q = sim::attitude(u, n);
    let off = Control::default();
    // resting CoM height above the deck
    let mut st = sim::state_at_pad(&spec, v3(2.0, 0.0, 4.0), V3::ZERO, q, V3::ZERO, 0.0);
    for _ in 0..1500 {
        sim::tick(&spec, &Params { crash: false, ..prm }, &mut st, &off);
    }
    let (xc, _) = com_state(&spec, &st);
    let rest = (xc - ksp_core::ksc::KSC_PAD_DECK_BODY_FIXED).dot(u);
    println!("resting CoM height {rest:.3} m above the deck");
    for v in [10.0f64, 12.0, 13.0, 14.0, 16.0, 19.0, 22.0, 26.0, 30.0, 35.0, 40.0] {
        let h = v * v / (2.0 * 9.81);
        let mut st = sim::state_at_pad(&spec, v3(2.0, 0.0, rest + h), V3::ZERO, q, V3::ZERO, 0.0);
        let (mut crashed_at, mut vtd, mut xmax) = (None, None, 0f32);
        for k in 0..400 {
            let (_, vc) = com_state(&spec, &st);
            sim::tick(&spec, &prm, &mut st, &off);
            if vtd.is_none() && st.legs.iter().any(|l| l.grounded) {
                vtd = Some(-vc.dot(u));
            }
            xmax = st.legs.iter().map(|l| l.x).fold(xmax, f32::max);
            if st.status == sim::STATUS_CRASHED {
                crashed_at = Some(k);
                break;
            }
        }
        let _ = e;
        println!(
            "v {v:4.1} (h {h:6.3} m): touchdown {:.2} m/s, max leg compression {xmax:.3} m, max normal impact {:.2} m/s -> {}",
            vtd.unwrap_or(f64::NAN),
            st.impact_speed,
            match crashed_at { Some(k) => format!("CRASHED at tick {k}"), None => "survived".into() }
        );
    }
}
