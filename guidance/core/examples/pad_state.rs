//! Where the sim puts the demo craft on the pad (CoM ENU offset from the pad, altitude), for
//! comparing with the game state the bridge logged on the pad.
use ksp_core::sim::{self, Control, Params};
use ksp_guidance::{ingame, nav};
fn main() {
    let b = ingame::built().unwrap();
    let prm = Params::default();
    let mut st = sim::state_landed(&b.stack, 1000.0);
    for _ in 0..500 {
        sim::tick(&b.stack, &prm, &mut st, &Control::default());
    }
    let n = nav::nav(&b.stack, &st);
    let (e, nn) = nav::Pad::ksc().ground_offset(n.x);
    println!("sim settled: status {} alt {:.3} e {:.3} n {:.3} tilt {:.3} deg", st.status, n.alt, e, nn, n.up.dot(n.x.normalized()).acos().to_degrees());
}
