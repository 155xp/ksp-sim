//! One-step check of the suspension chain against the game: seed each leg from the log, run
//! wheel::update_spring with the logged inputs, compare native spring/damper/target and the
//! module state with the next logged row.
use ksp_core::trace::*;
use ksp_core::wheel;
use std::path::PathBuf;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(&a[1]);
    for name in &a[2..] {
        let t = Trace::load(&dir, name).unwrap();
        let spec = t.spec();
        let c = &t.csv;
        let (mut n, mut ok_k, mut ok_c, mut ok_t, mut ok_m) = (0, 0, 0, 0, 0);
        let mut shown = 0;
        for i in 12..c.n - 1 {
            let st = state_at(&t, &spec, i + 1); // legs seeded: hit from row i+1, hidden from row i
            let ng = (0..spec.n_legs as usize).filter(|&k| st.legs[k].grounded).count() as u32;
            for k in 0..spec.n_legs as usize {
                let mut ls = st.legs[k];
                let inp = wheel::SuspInputs { vessel_mass: c.c("mass")[i + 1], landed: c.c("landed")[i + 1] > 0.5, n_grounded: ng,
                    gee_asl: 1.0003416049313476, max_surface_gee_asl: 1.7005807760832998, g_mag: 9.81, dt: 0.02 };
                // the seeded native values are from row i; run tick i+1's update
                wheel::update_spring(&spec.legs[k], &mut ls, &inp);
                let lk = c.c(&format!("wc{k}_spring"))[i + 1] as f32;
                let lc = c.c(&format!("wc{k}_damper"))[i + 1] as f32;
                let lt = c.c(&format!("wc{k}_target"))[i + 1] as f32;
                let lm = c.c(&format!("ws{k}_spring"))[i + 1] as f32;
                n += 1;
                ok_k += (ls.k == lk) as u32;
                ok_c += (ls.c == lc) as u32;
                ok_t += ((ls.target - lt).abs() < 1e-6) as u32;
                ok_m += (ls.m_spring == lm) as u32;
                if (ls.k != lk || ls.m_spring != lm) && shown < 6 {
                    shown += 1;
                    println!("  {name} tick {} leg {k}: sim k {} m_spring {} | game k {} m_spring {} | grounded {} x {:.4} sc_log {:.4} smg_log {:.3}",
                        i + 1, ls.k, ls.m_spring, lk, lm, ls.grounded, ls.x, c.c(&format!("ws{k}_sc"))[i + 1], c.c(&format!("ws{k}_smg"))[i + 1]);
                }
            }
        }
        println!("{name}: {n} leg-ticks; native spring exact {:.2}%, damper {:.2}%, target {:.2}%, module spring {:.2}%",
            100.0 * ok_k as f64 / n as f64, 100.0 * ok_c as f64 / n as f64, 100.0 * ok_t as f64 / n as f64, 100.0 * ok_m as f64 / n as f64);
    }
}
