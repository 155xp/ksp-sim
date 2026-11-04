//! Thrust point relative to the CoM: game (e0_pos = thrustTransform - CoMD) vs sim.
use ksp_core::trace::*;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let spec = t.spec();
    let c = &t.csv;
    for i in a[3..].iter().map(|x| x.parse::<usize>().unwrap()) {
        let st = state_at(&t, &spec, i);
        let (m, com_l, _) = spec.mass_props(&st.res);
        let sim_r = spec.engines[0].pos - com_l;
        let game_r = st.q.conj().rotate(c.v("e0_pos", i));
        println!("tick {i}: mass sim {m:.4} game {:.4} | thrust point - CoM (root-local) sim ({:+.4},{:+.4},{:+.4}) game ({:+.4},{:+.4},{:+.4}) | com_l {:?}",
            c.c("mass")[i], sim_r.x, sim_r.y, sim_r.z, game_r.x, game_r.y, game_r.z, com_l);
    }
}
