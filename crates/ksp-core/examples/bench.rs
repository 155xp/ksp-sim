//! cargo run --release -p ksp-core --example bench
use ksp_core::batch::*;
use ksp_core::sim::Params;
use ksp_core::vessel::{Craft, VesselSpec};
use std::time::Instant;

fn main() {
    let spec = VesselSpec::compile(&Craft::hopper());
    let threads = rayon::current_num_threads();
    println!("rayon threads: {threads}");
    for &n in &[1usize, 1024, 65536] {
        let cfg = TaskConfig { substeps: 1, max_steps: 100_000, ..Default::default() };
        let mut b = Batch::new(spec, Params::default(), cfg, n, 1);
        let mut obs = vec![0.0; n * OBS_DIM];
        b.reset(&mut obs);
        let (mut rew, mut te, mut tr, mut oc, mut fd, mut fs, mut fr) =
            (vec![0.0; n], vec![0u8; n], vec![0u8; n], vec![0u8; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        let mut act = vec![0.0f32; n * ACT_DIM];
        for i in 0..n { act[i * ACT_DIM] = 0.6; act[i * ACT_DIM + 1] = 0.1; }
        let iters = (20_000_000 / n).clamp(20, 200_000);
        let warm = iters / 10 + 1;
        for _ in 0..warm {
            b.step(&act, StepOut { obs: &mut obs, reward: &mut rew, terminated: &mut te, truncated: &mut tr, outcome: &mut oc, final_dist: &mut fd, final_speed: &mut fs, final_return: &mut fr });
        }
        let t0 = Instant::now();
        for _ in 0..iters {
            b.step(&act, StepOut { obs: &mut obs, reward: &mut rew, terminated: &mut te, truncated: &mut tr, outcome: &mut oc, final_dist: &mut fd, final_speed: &mut fs, final_return: &mut fr });
        }
        let dt = t0.elapsed().as_secs_f64();
        let sps = (iters * n) as f64 / dt;
        println!("n={n:>6}: {:>12.0} env-ticks/s ({:.3} us per env-tick, {:.1} sim-seconds/s per env at n=1)", sps, 1e6 / sps, sps * 0.02 / n as f64);
    }
}
