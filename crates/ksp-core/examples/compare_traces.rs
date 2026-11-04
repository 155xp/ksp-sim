//! Replay in-game traces open-loop and print the error over time.
//! cargo run --release -p ksp-core --example compare_traces -- <trace dir> [test ...]
use ksp_core::sim::Params;
use ksp_core::trace::{touchdowns, one_step, replay, replay_attitude_locked, windowed, Errors, Trace};
use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(args.get(1).cloned().unwrap_or_else(|| "data/traces".into()));
    let mut names: Vec<String> = args[2..].to_vec();
    if names.is_empty() {
        for e in std::fs::read_dir(&dir).unwrap() {
            let p = e.unwrap().path();
            if p.extension().is_some_and(|x| x == "csv") {
                names.push(p.file_stem().unwrap().to_string_lossy().into_owned());
            }
        }
        names.sort();
    }
    for n in names {
        let t = Trace::load(&dir, &n).expect("load");
        let spec = t.spec();
        let prm = Params::default();
        // airborne segment: from the first tick (after 10 of history) with no wheel grounded for
        // 25 ticks, to the next tick where any wheel is grounded again
        let nw = (0..8).filter(|k| t.csv.has(&format!("wc{k}_grounded"))).count();
        let air = |i: usize| (0..nw).all(|k| t.csv.c(&format!("wc{k}_grounded"))[i] < 0.5);
        let n = t.csv.n;
        let mut start = 10;
        if nw > 0 {
            while start + 25 < n && !(start..start + 25).all(air) {
                start += 1;
            }
        }
        // no legs: skip the pad phase (until the CoM has risen 1 m)
        if nw == 0 {
            let a0 = t.csv.c("alt")[start];
            while start < n && t.csv.c("alt")[start] < a0 + 1.0 {
                start += 1;
            }
        }
        let mut end = start;
        while end < n && (nw == 0 || air(end)) {
            end += 1;
        }
        let (e, _) = replay(&t, &spec, &prm, start, end);
        println!("## {}: airborne ticks {start}..{end} ({:.1} s)", t.name, e.ticks.len() as f64 * 0.02);
        println!("| t (s) | pos err (m) | vel err (m/s) | att err (deg) | LF err |");
        println!("|---|---|---|---|---|");
        let m = e.ticks.len();
        for frac in [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0] {
            let k = ((m as f64 * frac) as usize).clamp(1, m) - 1;
            println!("| {:.2} | {:.4} | {:.4} | {:.3} | {:.4} |", (e.ticks[k] - start) as f64 * 0.02, e.pos[k], e.vel[k], e.att_deg[k], e.fuel[k]);
        }
        println!("max: pos {:.4} m, vel {:.4} m/s, att {:.3} deg", Errors::max(&e.pos), Errors::max(&e.vel), Errors::max(&e.att_deg));
        let (_, ea, ew) = one_step(&t, &spec, &prm, start, end);
        let pct = |v: &[f64], p: f64| {
            let mut s = v.to_vec();
            s.sort_by(|a, b| a.partial_cmp(b).unwrap());
            s[((s.len() as f64 - 1.0) * p) as usize]
        };
        println!("one-step accel err (m/s^2): median {:.5} p99 {:.4} max {:.4}; ang accel err (rad/s^2): median {:.5} p99 {:.4}",
            pct(&ea, 0.5), pct(&ea, 0.99), Errors::max(&ea), pct(&ew, 0.5), pct(&ew, 0.99));
        for win in [50usize, 250] {
            let w = windowed(&t, &spec, &prm, start, end, win);
            if w.is_empty() {
                continue;
            }
            let mp = w.iter().map(|x| x.1).fold(0.0, f64::max);
            let mv = w.iter().map(|x| x.2).fold(0.0, f64::max);
            let ma = w.iter().map(|x| x.3).fold(0.0, f64::max);
            let med = |k: usize| { let mut s: Vec<f64> = w.iter().map(|x| [x.1, x.2, x.3][k]).collect(); s.sort_by(|a, b| a.partial_cmp(b).unwrap()); s[s.len() / 2] };
            let mut ws = w.clone();
            ws.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());
            let worst: Vec<String> = ws.iter().take(3).map(|x| format!("t{}({:.2}deg, alt {:.0}, thr {:.0})", x.0, x.3, t.csv.c("alt")[x.0], t.csv.c("e0_thrust").get(x.0).cloned().unwrap_or(0.0))).collect();
            println!("   worst windows: {}", worst.join(", "));
            println!("{:.0}-s windows ({}): pos median {:.4} max {:.4} m | vel median {:.4} max {:.4} m/s | att median {:.3} max {:.3} deg", win as f64 * 0.02, w.len(), med(0), mp, med(1), mv, med(2), ma);
        }
        for td in touchdowns(&t, &spec, &prm, 100, 250) {
            println!("touchdown: game tick {} vz {:.3} m/s | sim tick {} vz {:.3} m/s | pos err at td {:.3} m, after 5 s {:.3} m (horiz {:.3}) | max vel err {:.3} m/s | att err end {:.2} deg",
                td.game_tick, td.game_vz, td.sim_tick, td.sim_vz, td.pos_err_td, td.pos_err_end, td.horiz_err_end, td.vel_err_max, td.att_err_end_deg);
        }
        let el = replay_attitude_locked(&t, &spec, &prm, start, end);
        let m = el.ticks.len();
        println!("attitude-locked full replay: final pos {:.3} m vel {:.4} m/s; max pos {:.3} m vel {:.4} m/s\n", el.pos[m - 1], el.vel[m - 1], Errors::max(&el.pos), Errors::max(&el.vel));
    }
}
