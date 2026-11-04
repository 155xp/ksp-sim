//! Quick Monte Carlo summary: `cargo run --release --example mc -- [n_designs] [per_design] [size|0] [baseline|gfold] [seed]`
use ksp_guidance::fleet::{make_fleet, run_all};
use ksp_guidance::mission::{GuidanceConfig, LandingMode, Outcome};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let nd: usize = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(20);
    let pd: usize = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(5);
    let size: Option<u8> = a.get(3).and_then(|s| s.parse().ok()).filter(|&s| s > 0);
    let landing = if a.get(4).map(|s| s.as_str()) == Some("gfold") { LandingMode::Gfold } else { LandingMode::Baseline };
    let seed: u64 = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(42);
    let cfg = GuidanceConfig { landing, ..Default::default() };
    let t0 = std::time::Instant::now();
    let mut f = make_fleet(seed, nd, pd, size, cfg, false);
    if let Some(k) = std::env::var("MC_TRACE").ok().and_then(|s| s.parse::<usize>().ok()) {
        // fly one flight of the fleet alone, with the phase log
        let mut x = f.swap_remove(k);
        x.cfg.record = true;
        ksp_guidance::mission::run(&mut x);
        let mut last = 255.0;
        for r in &x.rec {
            if r[4] != last {
                println!("  t {:7.1}  ENU ({:9.0} {:9.0} {:8.0})  phase {}  thr {:.2}", r[0], r[1], r[2], r[3], r[4], r[5]);
                last = r[4];
            }
        }
        println!("{} {:?}\n{:#?}", x.sh.built.design.label(), x.outcome, x.stats);
        return;
    }
    run_all(&mut f);
    let el = t0.elapsed().as_secs_f64();
    let mut counts = [0usize; 6];
    let mut miss = vec![];
    for (idx, x) in f.iter().enumerate() {
        counts[x.outcome as usize] += 1;
        let s = &x.stats;
        let m = (s.final_e.powi(2) + s.final_n.powi(2)).sqrt();
        if x.outcome == Outcome::Landed && s.divert == 0 && s.abort == 0 {
            miss.push(m);
        }
        if std::env::var("MC_VERBOSE").is_ok() {
            println!("#{:<4} {:>3}t {:<22} {:?}{} miss {:7.1} vz {:5.1} vh {:5.1} tilt {:4.1} ign_err {:6.1} entry {:6.1} bb_end {:6.1} prop {:5.2}/{:5.1} sep {:5.0}m {:4.0}m/s",
                idx, x.sh.built.design.size, x.sh.built.design.label(), x.outcome, if s.abort > 0 { "(abort)" } else if s.divert > 0 { "(divert)" } else { "" }, m, s.td_vz, s.td_vh, s.td_tilt_deg, s.ign_err, s.entry_err, s.bb_end_err, s.prop_left, s.prop_cap, s.sep_alt, s.sep_speed);
        }
    }
    miss.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let pct = |p: f64| if miss.is_empty() { f64::NAN } else { miss[((miss.len() - 1) as f64 * p) as usize] };
    println!("{} flights in {:.1} s: landed {} crashed {} tipped {} timeout {} noreturn {}", f.len(), el, counts[1], counts[2], counts[3], counts[4], counts[5]);
    println!("landed miss: p50 {:.2} p90 {:.2} p99 {:.2} max {:.2} m", pct(0.5), pct(0.9), pct(0.99), pct(1.0));
}
