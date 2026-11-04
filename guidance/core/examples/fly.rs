//! Fly single RTLS missions and print the phase log: `cargo run --release --example fly -- [seed] [size] [mode]`
use ksp_guidance::craft::{build, sample_design, Rng};
use ksp_guidance::mission::{run, sample_dispersion, Flight, GuidanceConfig, LandingMode, Shared};
use std::sync::Arc;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let seed: u64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let size: Option<u8> = a.get(2).and_then(|s| s.parse().ok()).filter(|&s| s > 0);
    let mode = match a.get(3).map(|s| s.as_str()) {
        Some("gfold") => LandingMode::Gfold,
        _ => LandingMode::Baseline,
    };
    let mut rng = Rng(seed);
    let d = sample_design(&mut rng, size);
    let b = build(&d).unwrap();
    let (mw, _, _) = b.booster.mass_props(&b.booster.full_resources());
    let (ms, _, _) = b.stack.mass_props(&b.stack.full_resources());
    println!("design {:?}\n booster wet {:.1} t dry {:.1} t, stack {:.1} t, {} parts", d.label(), mw, b.booster.dry_mass(), ms, b.stack.n_parts);
    let sh = Arc::new(Shared::new(b));
    let disp = sample_dispersion(&mut rng, false);
    let cfg = GuidanceConfig { landing: mode, record: true, ..Default::default() };
    let mut f = Flight::new(sh, disp, cfg);
    let t0 = std::time::Instant::now();
    run(&mut f);
    let el = t0.elapsed().as_secs_f64();
    let mut last = 255.0;
    for r in &f.rec {
        if r[4] != last {
            println!("  t {:7.1}  ENU ({:9.0} {:9.0} {:8.0})  phase {}  thr {:.2}", r[0], r[1], r[2], r[3], r[4], r[5]);
            last = r[4];
        }
    }
    println!("outcome {:?}  ({:.2} s wall, {:.0} s flight)", f.outcome, el, f.st.ut - f.disp.ut);
    println!("{:#?}", f.stats);
}
