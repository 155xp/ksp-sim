//! Horizontal CoM velocity after touchdown, game vs sim replay: slide_dump <dir> <name> <td_row>
use ksp_core::sim::{self, Params};
use ksp_core::trace::{com_state, control, state_at, Trace};
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = Trace::load(&PathBuf::from(&a[1]), &a[2]).unwrap();
    let td: usize = a[3].parse().unwrap();
    let mut spec = t.spec();
    if std::env::var("NO_HULL").is_ok() {
        spec.n_hull = 0;
    }
    let prm = Params { crash: false, ..Params::default() };
    let c = &t.csv;
    let (e, n, u) = sim::pad_enu();
    let mut st = state_at(&t, &spec, td - 100);
    for j in td - 100..td + 120 {
        sim::tick(&spec, &prm, &mut st, &control(&t, j));
        let k = j + 1;
        if std::env::var("LEGS").is_ok() && k >= td && k <= td + 6 {
            for m in 0..spec.n_legs as usize {
                let g = |n: &str| c.c(&format!("{n}"))[k];
                println!("   tick {k} leg {m}: game cd {:.4} F {:.2} k {} c {} | mod spr {} dmp {} vp_k {} vp_c {} rtk {} rtc {} ab {} | sim x {:.4} F {:.2} k {} c {} | spr {} dmp {} rate {} drate {} rt {} rtd {} ab {}",
                    g(&format!("ws{m}_cd")), g(&format!("wc{m}_force")), g(&format!("wc{m}_spring")), g(&format!("wc{m}_damper")),
                    g(&format!("ws{m}_spring")), g(&format!("ws{m}_damper")), g(&format!("ws{m}_vp_k")), g(&format!("ws{m}_vp_c")), g(&format!("ws{m}_vp_rtk")), g(&format!("ws{m}_vp_rtc")), g(&format!("ws{m}_ab")),
                    st.legs[m].x, st.legs[m].hit_force, st.legs[m].k, st.legs[m].c, st.legs[m].m_spring, st.legs[m].m_damper, st.legs[m].rate, st.legs[m].drate, st.legs[m].rt, st.legs[m].rtd, st.legs[m].auto_boost);
            }
        }
        if std::env::var("HULL").is_ok() && k >= td && k <= td + 8 {
            let sc = sim::Scenery { shift: st.ground_shift, ksc_dz: st.ksc_dz };
            for h in 0..spec.n_hull as usize {
                let p = st.x + st.q.rotate(spec.hull[h]);
                let g = sim::ground_height_sc(&prm, &sc, p);
                if g < 0.3 {
                    println!("   tick {k} hull {h} (part {} tol {}) gap {g:+.4} local {:?}", spec.hull_part[h], spec.hull_tolerance[h], spec.hull[h]);
                }
            }
        }
        if k >= td - 2 && (k < td + 12 || k % 10 == 0) {
            let (_, vs) = com_state(&spec, &st);
            let vg = c.v("vsrf", k);
            let f = |k2: usize, m: usize| c.c(&format!("ws{m}_ltf_x"))[k2];
            println!("tick {k:4} (+{:3}): game vE {:+.3} vN {:+.3} vU {:+.3} | sim vE {:+.3} vN {:+.3} vU {:+.3} | tire0 x game {:+.2} sim {:+.2} | x0 game {:.3} sim {:.3}",
                k as i64 - td as i64, vg.dot(e), vg.dot(n), vg.dot(u), vs.dot(e), vs.dot(n), vs.dot(u), f(k, 0), st.legs[0].tire[0], c.c("ws0_cd")[k], st.legs[0].x);
        }
    }
}
