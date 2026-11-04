//! RL hook: a batched terminal-phase environment. Episodes start from real landing-burn
//! ignition states (classic guidance flown from separation, paused at ignition) and the policy
//! commands the landing burn through `LandingMode::External`: a normalised non-gravitational
//! acceleration request in pad ENU (units of the craft's current max thrust acceleration),
//! optionally as a residual on top of the classic baseline. Attitude control, thrust allocation
//! (lift-aware) and the craft model stay classic, so the policy sees a craft-agnostic interface.
//! Observations (`Flight::terminal_obs`) carry the craft parameters, so one policy can be
//! conditioned across the whole booster distribution.

use crate::craft::Rng;
use crate::mission::{Flight, LandingMode, Outcome, Phase, TERM_OBS_DIM};
use rayon::prelude::*;

pub const ACT_DIM: usize = 3;

#[derive(Clone, Copy, Debug)]
pub struct RewardCfg {
    pub success: f64,
    pub crash: f64,
    /// per metre of miss / per m/s of excess touchdown speed at landing
    pub miss: f64,
    pub speed: f64,
    /// per fraction of the booster's propellant used during the burn
    pub fuel: f64,
}

impl Default for RewardCfg {
    fn default() -> Self {
        RewardCfg { success: 10.0, crash: -10.0, miss: 0.5, speed: 2.0, fuel: 2.0 }
    }
}

pub struct TerminalBatch {
    pub pool: Vec<Flight>,
    pub slots: Vec<Flight>,
    rng: Rng,
    pub ticks_per_step: u32,
    pub residual: bool,
    pub reward: RewardCfg,
    potential: Vec<f64>,
    prop0: Vec<f64>,
    pub max_steps: u32,
    steps: Vec<u32>,
}

/// Per-step outputs (length N or N * dim).
pub struct TermOut<'a> {
    pub obs: &'a mut [f32],
    pub reward: &'a mut [f32],
    pub done: &'a mut [u8],
    pub outcome: &'a mut [u8],
    pub miss: &'a mut [f32],
    pub td_vz: &'a mut [f32],
    pub td_vh: &'a mut [f32],
    pub fuel: &'a mut [f32],
}

fn shaping(f: &Flight) -> f64 {
    let o = f.terminal_obs();
    // closer and slower horizontally (scale-free, see terminal_obs)
    -((o[0] as f64).hypot(o[1] as f64) + (o[3] as f64).hypot(o[4] as f64))
}

fn prop(f: &Flight) -> f64 {
    crate::nav::propellant(&f.sh.built.booster, &f.st.res).0
}

impl TerminalBatch {
    /// `pool`: paused-at-ignition flights to sample episode starts from.
    pub fn new(mut pool: Vec<Flight>, n: usize, seed: u64, residual: bool, ticks_per_step: u32) -> TerminalBatch {
        pool.retain(|f| f.paused && f.phase == Phase::LandingBurn);
        assert!(!pool.is_empty(), "no ignition states in the pool");
        for f in pool.iter_mut() {
            f.cfg.landing = LandingMode::External { residual };
            f.cfg.record = false;
        }
        let mut b = TerminalBatch { slots: Vec::with_capacity(n), pool, rng: Rng(seed), ticks_per_step, residual, reward: RewardCfg::default(), potential: vec![0.0; n], prop0: vec![0.0; n], max_steps: 3000, steps: vec![0; n] };
        for i in 0..n {
            let f = b.sample();
            b.potential[i] = shaping(&f);
            b.prop0[i] = prop(&f);
            b.slots.push(f);
        }
        b
    }

    fn sample(&mut self) -> Flight {
        let k = (self.rng.next_u64() % self.pool.len() as u64) as usize;
        let mut f = self.pool[k].clone();
        f.paused = false;
        f
    }

    pub fn len(&self) -> usize {
        self.slots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    pub fn observe(&self, obs: &mut [f32]) {
        obs.par_chunks_mut(TERM_OBS_DIM).zip(self.slots.par_iter()).for_each(|(o, f)| o.copy_from_slice(&f.terminal_obs()));
    }

    /// Reset every slot to a fresh episode start.
    pub fn reset(&mut self, obs: &mut [f32]) {
        for i in 0..self.len() {
            let f = self.sample();
            self.potential[i] = shaping(&f);
            self.prop0[i] = prop(&f);
            self.steps[i] = 0;
            self.slots[i] = f;
        }
        self.observe(obs);
    }

    /// Advance all slots by `ticks_per_step` physics ticks with `actions` (N x 3, clipped to
    /// [-1, 1]); finished episodes auto-reset (same-step reset, as the sim's VecEnv).
    pub fn step(&mut self, actions: &[f32], out: TermOut) {
        let n = self.len();
        assert_eq!(actions.len(), n * ACT_DIM);
        let tps = self.ticks_per_step;
        let rw = self.reward;
        let max_steps = self.max_steps;
        let res: Vec<(f64, bool, u8, f32, f32, f32, f32)> = self
            .slots
            .par_iter_mut()
            .zip(self.potential.par_iter_mut())
            .zip(self.prop0.par_iter())
            .zip(self.steps.par_iter_mut())
            .enumerate()
            .map(|(i, (((f, pot), p0), st))| {
                for k in 0..ACT_DIM {
                    f.ext_action[k] = (actions[i * ACT_DIM + k] as f64).clamp(-1.0, 1.0);
                }
                for _ in 0..tps {
                    if f.done() {
                        break;
                    }
                    f.tick();
                }
                *st += 1;
                if !f.done() && *st >= max_steps {
                    f.outcome = Outcome::Timeout;
                    f.phase = Phase::Done;
                }
                let mut r;
                if f.done() {
                    let s = &f.stats;
                    let miss = s.final_e.hypot(s.final_n);
                    let used = (*p0 - prop(f)) / f.stats.prop_cap.max(1e-9);
                    r = match f.outcome {
                        Outcome::Landed => rw.success - rw.miss * miss.min(50.0) - rw.speed * ((-s.td_vz - 2.0).max(0.0) + s.td_vh),
                        _ => rw.crash - 0.1 * miss.min(200.0),
                    } - rw.fuel * used;
                    (r, true, f.outcome as u8, miss as f32, s.td_vz as f32, s.td_vh as f32, used as f32)
                } else {
                    let phi = shaping(f);
                    r = phi - *pot;
                    *pot = phi;
                    let _ = &mut r;
                    (r, false, 0, 0.0, 0.0, 0.0, 0.0)
                }
            })
            .collect();
        for (i, (r, d, oc, miss, vz, vh, fu)) in res.into_iter().enumerate() {
            out.reward[i] = r as f32;
            out.done[i] = d as u8;
            out.outcome[i] = oc;
            out.miss[i] = miss;
            out.td_vz[i] = vz;
            out.td_vh[i] = vh;
            out.fuel[i] = fu;
            if d {
                let f = self.sample();
                self.potential[i] = shaping(&f);
                self.prop0[i] = prop(&f);
                self.steps[i] = 0;
                self.slots[i] = f;
            }
        }
        self.observe(out.obs);
    }
}
