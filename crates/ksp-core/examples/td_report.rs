//! Touchdown replay report with leg compression vs the game: td_report <dir> <name>...
use ksp_core::sim::{self, Params};
use ksp_core::trace::{control, state_at, touchdowns, Trace};
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(&a[1]);
    for n in &a[2..] {
        let t = Trace::load(&dir, n).unwrap();
        let spec = t.spec();
        let prm = Params { crash: false, ..Params::default() };
        let tds = touchdowns(&t, &spec, &prm, 100, 250);
        let Some(td) = tds.first() else { println!("{n}: no touchdown"); continue };
        // compression series: sim replay from 100 ticks before vs the game's contactDepth
        let c = &t.csv;
        let start = td.game_tick - 100;
        let mut st = state_at(&t, &spec, start);
        let (mut gmax, mut smax, mut kg, mut ks) = (0f64, 0f32, 0usize, 0usize);
        for j in start..(td.game_tick + 60).min(c.n - 1) {
            sim::tick(&spec, &prm, &mut st, &control(&t, j));
            for k in 0..spec.n_legs as usize {
                let g = c.c(&format!("ws{k}_cd"))[j + 1];
                if g.is_finite() && g > gmax { gmax = g; kg = j + 1; }
                if st.legs[k].x > smax { smax = st.legs[k].x; ks = j + 1; }
            }
        }
        // with the crash rules on: does the sim lose the vessel (and when)?
        let mut sc = state_at(&t, &spec, start);
        let mut crash_tick = None;
        for j in start..c.n - 1 {
            sim::tick(&spec, &Params::default(), &mut sc, &control(&t, j));
            if sc.status == sim::STATUS_CRASHED {
                crash_tick = Some(j + 1);
                break;
            }
        }
        let died = (0..c.n).find(|&i| c.c("nparts")[i] < c.c("nparts")[0]);
        println!("{n}: game parts lost at {:?}, sim crash at {:?}", died, crash_tick);
        println!("{n}: horiz err +5 s {:.4} m, att err +5 s {:.3} deg, vel err max {:.3} m/s", td.horiz_err_end, td.att_err_end_deg, td.vel_err_max);
        println!("{n}: touchdown game {} sim {} (vz game {:.2} sim {:.2}), pos err td {:.4} +5 s {:.4} m | peak compression game {gmax:.3} m @{kg}, sim {smax:.3} m @{ks}",
            td.game_tick, td.sim_tick, td.game_vz, td.sim_vz, td.pos_err_td, td.pos_err_end);
    }
}
