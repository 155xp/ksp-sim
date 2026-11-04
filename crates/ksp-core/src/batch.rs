//! Batched, struct-of-arrays stepping of N environments (the RL hot path).
//!
//! State lives in one column per `EnvState` field (`Soa`). A step gathers env i's columns into
//! registers (`EnvState`), runs `substeps` physics ticks, computes obs / reward, scatters back.
//! Envs are independent, so the loop is a rayon `par_iter` over env indices; the identical shape
//! (gather -> pure kernel -> scatter, one thread per env) ports directly to CUDA.

use crate::math::{v3, Quat, V3};
use crate::parts::N_RES;
use crate::sim::{self, Control, EnvState, Frame, Params, TickInfo, VEL_SMOOTH_LEN};
use crate::vessel::{VesselSpec, MAX_ENGINES};
use rayon::prelude::*;

macro_rules! soa {
    ($($f:ident : $t:ty),* $(,)?) => {
        /// One column per `EnvState` field.
        #[derive(Clone, Debug, Default)]
        pub struct Soa { $(pub $f: Vec<$t>,)* }
        impl Soa {
            pub fn new(n: usize) -> Self { Soa { $($f: vec![<$t>::default(); n],)* } }
            #[inline(always)]
            pub fn len(&self) -> usize { self.ut.len() }
            /// # Safety: `i < len`, and no other thread touches index `i` concurrently.
            #[inline(always)]
            unsafe fn load(p: &SoaPtr, i: usize) -> EnvState {
                EnvState { $($f: *p.$f.add(i),)* }
            }
            #[inline(always)]
            unsafe fn store(p: &SoaPtr, i: usize, s: &EnvState) {
                $(*p.$f.add(i) = s.$f;)*
            }
            fn ptr(&mut self) -> SoaPtr { SoaPtr { $($f: self.$f.as_mut_ptr(),)* } }
            pub fn get(&self, i: usize) -> EnvState { EnvState { $($f: self.$f[i],)* } }
            pub fn set(&mut self, i: usize, s: &EnvState) { $(self.$f[i] = s.$f;)* }
        }
        struct SoaPtr { $($f: *mut $t,)* }
    };
}

soa! {
    ut: f64, frame: Frame, x: V3, v: V3, q: Quat, w: V3,
    res: [f64; N_RES], thr: [f64; MAX_ENGINES], gimbal: [[f64; 2]; MAX_ENGINES], flameout: u8,
    rw_in: V3, smooth_buf: [[f32; 3]; VEL_SMOOTH_LEN], smooth_idx: u8, vel_spiking: bool,
    last_vel_provisional: bool, last_vel: [f32; 3], last_vel_idx: u8, drift_frames: u8,
    status: u8, settled_ticks: u16, ticks: u32, impact_speed: f32, toff: f32,
    legs: [crate::wheel::LegState; crate::vessel::MAX_LEGS], ground_shift: V3, ksc_dz: f64, hull_touch: u64,
    brakes: bool, brake_deflection: f32,
}

// Raw column pointers are only dereferenced at disjoint indices (one env per task).
unsafe impl Send for SoaPtr {}
unsafe impl Sync for SoaPtr {}

pub const OBS_DIM: usize = 22;
pub const ACT_DIM: usize = 4;

/// Reset distribution and episode settings for the pad-landing task.
#[derive(Clone, Copy, Debug)]
pub struct TaskConfig {
    /// physics ticks (0.02 s each) per env step
    pub substeps: u32,
    pub max_steps: u32,
    /// CoM height above the pad deck at reset (m)
    pub alt_min: f64,
    pub alt_max: f64,
    /// max horizontal offset from the pad at reset (m)
    pub radius: f64,
    /// descent speed range at reset (m/s, positive = down)
    pub vdown_min: f64,
    pub vdown_max: f64,
    /// max horizontal speed at reset (m/s)
    pub vh_max: f64,
    /// max initial tilt from vertical (deg) and angular rate (rad/s)
    pub tilt_max_deg: f64,
    pub w_max: f64,
    /// initial fuel fraction range
    pub fuel_min: f64,
    pub fuel_max: f64,
    /// UT at reset is uniform in [0, ut_max) (sun position -> temperature)
    pub ut_max: f64,
    /// out-of-bounds: horizontal distance / height (m)
    pub oob_radius: f64,
    pub oob_alt: f64,
    /// scenery placement jitter (see sim::Scenery, DESIGN.md "Scenery placement"), uniform per
    /// episode, 0 = nominal surveyed placement (no extra RNG draw):
    /// - ground_shift_max: all scenery along the pad normal in [-max, max] m (PQS re-rounding at
    ///   the last FloatingOrigin shift; measured within +-0.022 m, sigma 0.009 m)
    /// - ksc_dz_max: KSC colliders only, in [-max, max] m (per game session; measured 0 in 7 of 9
    ///   game sessions, +0.037 and -0.013 m in the other two)
    pub ground_shift_max: f64,
    pub ksc_dz_max: f64,
}

impl Default for TaskConfig {
    fn default() -> Self {
        TaskConfig {
            substeps: 5,
            max_steps: 1500,
            alt_min: 600.0,
            alt_max: 1500.0,
            radius: 250.0,
            vdown_min: 20.0,
            vdown_max: 80.0,
            vh_max: 15.0,
            tilt_max_deg: 10.0,
            w_max: 0.05,
            fuel_min: 0.6,
            fuel_max: 1.0,
            ut_max: 0.0,
            oob_radius: 3000.0,
            oob_alt: 5000.0,
            ground_shift_max: 0.0,
            ksc_dz_max: 0.0,
        }
    }
}

#[inline(always)]
fn splitmix(x: &mut u64) -> u64 {
    *x = x.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = *x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}
#[inline(always)]
fn unif(rng: &mut u64, a: f64, b: f64) -> f64 {
    a + (b - a) * ((splitmix(rng) >> 11) as f64 * (1.0 / (1u64 << 53) as f64))
}

/// Sample a reset state (pure function of the rng state).
pub fn sample_reset(spec: &VesselSpec, cfg: &TaskConfig, rng: &mut u64) -> EnvState {
    let r = cfg.radius * unif(rng, 0.0, 1.0).sqrt();
    let th = unif(rng, 0.0, core::f64::consts::TAU);
    let h = unif(rng, cfg.alt_min, cfg.alt_max);
    let vh = cfg.vh_max * unif(rng, 0.0, 1.0).sqrt();
    let vth = unif(rng, 0.0, core::f64::consts::TAU);
    let vd = unif(rng, cfg.vdown_min, cfg.vdown_max);
    let (e, n, u) = sim::pad_enu();
    let tilt = cfg.tilt_max_deg.to_radians() * unif(rng, 0.0, 1.0).sqrt();
    let tdir = unif(rng, 0.0, core::f64::consts::TAU);
    let up_dir = u * tilt.cos() + (e * tdir.cos() + n * tdir.sin()) * tilt.sin();
    let yaw = unif(rng, 0.0, core::f64::consts::TAU);
    let q = sim::attitude(up_dir, e * yaw.cos() + n * yaw.sin());
    let w = v3(unif(rng, -1.0, 1.0), unif(rng, -1.0, 1.0), unif(rng, -1.0, 1.0)) * cfg.w_max;
    let ut = unif(rng, 0.0, cfg.ut_max.max(0.0));
    let mut st = sim::state_at_pad(
        spec,
        v3(r * th.cos(), r * th.sin(), h),
        v3(vh * vth.cos(), vh * vth.sin(), -vd),
        q,
        w,
        ut,
    );
    let f = unif(rng, cfg.fuel_min, cfg.fuel_max);
    st.res[0] *= f;
    st.res[1] *= f;
    // CoM moved with the fuel load: re-place so the sampled CoM position holds
    let full = spec.mass_props(&spec.full_resources()).1;
    let now = spec.mass_props(&st.res).1;
    st.x = st.x + st.q.rotate(full - now);
    if cfg.ground_shift_max > 0.0 {
        st.ground_shift = u * unif(rng, -cfg.ground_shift_max, cfg.ground_shift_max);
    }
    if cfg.ksc_dz_max > 0.0 {
        st.ksc_dz = unif(rng, -cfg.ksc_dz_max, cfg.ksc_dz_max);
    }
    st
}

/// Per-env episode bookkeeping that is not physics state.
#[derive(Clone, Debug, Default)]
pub struct Episode {
    pub rng: Vec<u64>,
    pub steps: Vec<u32>,
    pub potential: Vec<f32>,
    pub ret: Vec<f32>,
}

/// Outcome codes reported at episode end.
pub const OUT_NONE: u8 = 0;
pub const OUT_LANDED: u8 = 1;
pub const OUT_CRASHED: u8 = 2;
pub const OUT_OOB: u8 = 3;
pub const OUT_TIMEOUT: u8 = 4;

/// Output buffers of one batched step (all length N or N*dim).
pub struct StepOut<'a> {
    pub obs: &'a mut [f32],
    pub reward: &'a mut [f32],
    pub terminated: &'a mut [u8],
    pub truncated: &'a mut [u8],
    /// outcome code of the episode that just ended (OUT_*), else 0
    pub outcome: &'a mut [u8],
    /// final horizontal distance to pad centre / touchdown speed for ended episodes
    pub final_dist: &'a mut [f32],
    pub final_speed: &'a mut [f32],
    pub final_return: &'a mut [f32],
}

struct EpPtr {
    rng: *mut u64,
    steps: *mut u32,
    potential: *mut f32,
    ret: *mut f32,
}
unsafe impl Send for EpPtr {}
unsafe impl Sync for EpPtr {}

struct OutPtr {
    obs: *mut f32,
    reward: *mut f32,
    terminated: *mut u8,
    truncated: *mut u8,
    outcome: *mut u8,
    final_dist: *mut f32,
    final_speed: *mut f32,
    final_return: *mut f32,
}
unsafe impl Send for OutPtr {}
unsafe impl Sync for OutPtr {}

pub struct Batch {
    pub spec: VesselSpec,
    pub prm: Params,
    pub cfg: TaskConfig,
    pub soa: Soa,
    pub ep: Episode,
    /// last tick diagnostics per env (mach, q) for observations
    last_mach: Vec<f32>,
    last_q: Vec<f32>,
}

/// Observation of one env (see OBS_DIM layout in DESIGN.md / python docstring).
#[inline]
pub fn observe(spec: &VesselSpec, prm: &Params, st: &EnvState, mach: f32, qkpa: f32, out: &mut [f32]) {
    let (e, n, u) = sim::pad_enu();
    let (_, com_l, _) = spec.mass_props(&st.res);
    let r_com = st.q.rotate(com_l);
    let (x_com, v_srf) = if st.frame == Frame::Rotating {
        (st.x + r_com, st.v + st.w.cross(r_com))
    } else {
        let (x, v) = sim::inertial_to_fixed(prm.body, st.ut, st.x + r_com, st.v + st.w.cross(r_com));
        (x, v)
    };
    let d = x_com - crate::ksc::KSC_PAD_DECK_BODY_FIXED;
    let up = st.q.rotate(v3(0.0, 1.0, 0.0));
    let fw = st.q.rotate(v3(0.0, 0.0, 1.0));
    let wl = st.q.conj().rotate(st.w);
    let ground_r = prm.body.radius + prm.ground_alt;
    let mut low = f64::INFINITY;
    for k in 0..spec.n_legs as usize {
        let p = st.x + st.q.rotate(spec.legs[k].foot());
        low = low.min(sim::ground_height_sc(prm, &sim::Scenery { shift: st.ground_shift, ksc_dz: st.ksc_dz }, p));
    }
    if spec.n_legs == 0 {
        low = sim::ground_height_sc(prm, &sim::Scenery { shift: st.ground_shift, ksc_dz: st.ksc_dz }, x_com);
    }
    let _ = ground_r;
    let fuel = if spec.res_cap_total[0] > 0.0 { st.res[0] / spec.res_cap_total[0] } else { 0.0 };
    let (m, _, _) = spec.mass_props(&st.res);
    let legs = {
        let mut c = 0;
        for k in 0..spec.n_legs as usize {
            if st.legs[k].grounded {
                c += 1;
            }
        }
        c as f64 / spec.n_legs.max(1) as f64
    };
    let o = [
        d.dot(e), d.dot(n), d.dot(u),
        v_srf.dot(e), v_srf.dot(n), v_srf.dot(u),
        up.dot(e), up.dot(n), up.dot(u),
        fw.dot(e), fw.dot(n), fw.dot(u),
        wl.x, wl.y, wl.z,
        low, fuel, st.thr[0], m, legs, mach as f64, qkpa as f64,
    ];
    for (k, v) in o.iter().enumerate() {
        out[k] = *v as f32;
    }
}

/// Shaping potential: closer, slower and more upright is better.
#[inline]
fn potential(obs: &[f32]) -> f32 {
    let dh = (obs[0] * obs[0] + obs[1] * obs[1]).sqrt();
    let h = obs[15].max(0.0);
    let spd = (obs[3] * obs[3] + obs[4] * obs[4] + obs[5] * obs[5]).sqrt();
    -(dh / 50.0) - (h / 200.0) - (spd / 20.0) - 2.0 * (1.0 - obs[8])
}

impl Batch {
    pub fn new(spec: VesselSpec, prm: Params, cfg: TaskConfig, n: usize, seed: u64) -> Batch {
        let mut b = Batch {
            spec,
            prm,
            cfg,
            soa: Soa::new(n),
            ep: Episode { rng: vec![0; n], steps: vec![0; n], potential: vec![0.0; n], ret: vec![0.0; n] },
            last_mach: vec![0.0; n],
            last_q: vec![0.0; n],
        };
        for i in 0..n {
            let mut s = seed ^ (i as u64).wrapping_mul(0xA24BAED4963EE407);
            splitmix(&mut s);
            b.ep.rng[i] = s;
        }
        b
    }

    pub fn len(&self) -> usize {
        self.soa.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Reset all envs; writes observations.
    pub fn reset(&mut self, obs: &mut [f32]) {
        let n = self.len();
        for i in 0..n {
            let st = sample_reset(&self.spec, &self.cfg, &mut self.ep.rng[i]);
            self.soa.set(i, &st);
            self.ep.steps[i] = 0;
            self.ep.ret[i] = 0.0;
            observe(&self.spec, &self.prm, &st, 0.0, 0.0, &mut obs[i * OBS_DIM..(i + 1) * OBS_DIM]);
            self.ep.potential[i] = potential(&obs[i * OBS_DIM..(i + 1) * OBS_DIM]);
        }
    }

    /// Step all envs with `actions` (N x ACT_DIM: throttle in [0,1], pitch, yaw, roll in [-1,1]).
    /// Envs whose episode ended are auto-reset in the same call (obs is then the new episode's
    /// first observation; the outcome arrays describe the episode that ended).
    pub fn step(&mut self, actions: &[f32], out: StepOut) {
        let n = self.len();
        assert_eq!(actions.len(), n * ACT_DIM);
        assert_eq!(out.obs.len(), n * OBS_DIM);
        let spec = &self.spec;
        let prm = &self.prm;
        let cfg = &self.cfg;
        let sp = self.soa.ptr();
        let ep = EpPtr {
            rng: self.ep.rng.as_mut_ptr(),
            steps: self.ep.steps.as_mut_ptr(),
            potential: self.ep.potential.as_mut_ptr(),
            ret: self.ep.ret.as_mut_ptr(),
        };
        let op = OutPtr {
            obs: out.obs.as_mut_ptr(),
            reward: out.reward.as_mut_ptr(),
            terminated: out.terminated.as_mut_ptr(),
            truncated: out.truncated.as_mut_ptr(),
            outcome: out.outcome.as_mut_ptr(),
            final_dist: out.final_dist.as_mut_ptr(),
            final_speed: out.final_speed.as_mut_ptr(),
            final_return: out.final_return.as_mut_ptr(),
        };
        let mach = self.last_mach.as_mut_ptr() as usize;
        let qk = self.last_q.as_mut_ptr() as usize;
        let (sp, ep, op) = (&sp, &ep, &op);
        let body = |i: usize| unsafe {
            let a = &actions[i * ACT_DIM..(i + 1) * ACT_DIM];
            let ctl = Control { throttle: a[0] as f64, pitch: a[1] as f64, yaw: a[2] as f64, roll: a[3] as f64 };
            let mut st = Soa::load(sp, i);
            let mut info = TickInfo::default();
            for _ in 0..cfg.substeps {
                info = sim::tick(spec, prm, &mut st, &ctl);
                if st.status == sim::STATUS_CRASHED || st.status == sim::STATUS_LANDED {
                    break;
                }
            }
            let (m_i, q_i) = (info.atm.mach as f32, info.atm.q_kpa as f32);
            *(mach as *mut f32).add(i) = m_i;
            *(qk as *mut f32).add(i) = q_i;
            let o = core::slice::from_raw_parts_mut(op.obs.add(i * OBS_DIM), OBS_DIM);
            observe(spec, prm, &st, m_i, q_i, o);
            let steps = *ep.steps.add(i) + 1;
            *ep.steps.add(i) = steps;
            let phi = potential(o);
            let mut r = phi - *ep.potential.add(i) - 0.002 * ctl.throttle.clamp(0.0, 1.0) as f32;
            *ep.potential.add(i) = phi;
            let dh = (o[0] * o[0] + o[1] * o[1]).sqrt();
            let spd = (o[3] * o[3] + o[4] * o[4] + o[5] * o[5]).sqrt();
            let mut outc = OUT_NONE;
            if st.status == sim::STATUS_LANDED {
                outc = OUT_LANDED;
                r += 10.0 + 10.0 * (1.0 - dh / 10.0).max(0.0);
            } else if st.status == sim::STATUS_CRASHED || (st.status == sim::STATUS_CONTACT && o[8] < 0.5) {
                // destroyed, or tipped over (> 60 deg) while touching the ground
                outc = OUT_CRASHED;
                r -= 10.0;
            } else if dh > cfg.oob_radius as f32 || o[2] > cfg.oob_alt as f32 {
                outc = OUT_OOB;
                r -= 10.0;
            } else if steps >= cfg.max_steps {
                outc = OUT_TIMEOUT;
            }
            *ep.ret.add(i) += r;
            *op.reward.add(i) = r;
            *op.terminated.add(i) = (outc == OUT_LANDED || outc == OUT_CRASHED) as u8;
            *op.truncated.add(i) = (outc == OUT_OOB || outc == OUT_TIMEOUT) as u8;
            *op.outcome.add(i) = outc;
            if outc != OUT_NONE {
                *op.final_dist.add(i) = dh;
                *op.final_speed.add(i) = st.impact_speed.max(spd);
                *op.final_return.add(i) = *ep.ret.add(i);
                let nst = sample_reset(spec, cfg, &mut *ep.rng.add(i));
                observe(spec, prm, &nst, 0.0, 0.0, o);
                *ep.potential.add(i) = potential(o);
                *ep.steps.add(i) = 0;
                *ep.ret.add(i) = 0.0;
                Soa::store(sp, i, &nst);
            } else {
                *op.final_dist.add(i) = 0.0;
                *op.final_speed.add(i) = 0.0;
                *op.final_return.add(i) = 0.0;
                st.impact_speed = 0.0;
                Soa::store(sp, i, &st);
            }
        };
        if n >= 64 {
            (0..n).into_par_iter().with_min_len(32).for_each(body);
        } else {
            (0..n).for_each(body);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vessel::Craft;

    fn run(n: usize, steps: usize, seed: u64) -> (Vec<f32>, u32) {
        let spec = VesselSpec::compile(&Craft::hopper());
        let mut b = Batch::new(spec, Params::default(), TaskConfig::default(), n, seed);
        let mut obs = vec![0.0; n * OBS_DIM];
        b.reset(&mut obs);
        let (mut rew, mut te, mut tr, mut oc, mut fd, mut fs, mut fr) =
            (vec![0.0; n], vec![0u8; n], vec![0u8; n], vec![0u8; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        let mut ends = 0;
        // simple suicide-burn-ish controller: full throttle when low and fast
        for _ in 0..steps {
            let mut act = vec![0.0f32; n * ACT_DIM];
            for i in 0..n {
                let o = &obs[i * OBS_DIM..];
                let vz = o[5];
                let h = o[15];
                act[i * ACT_DIM] = if vz < -(0.15 * h + 2.0) { 1.0 } else { 0.25 };
            }
            b.step(&act, StepOut { obs: &mut obs, reward: &mut rew, terminated: &mut te, truncated: &mut tr, outcome: &mut oc, final_dist: &mut fd, final_speed: &mut fs, final_return: &mut fr });
            ends += oc.iter().filter(|&&c| c != 0).count() as u32;
        }
        (obs, ends)
    }

    #[test]
    fn deterministic_and_parallel_equals_serial() {
        let (a, ea) = run(200, 300, 7);
        let (b, eb) = run(200, 300, 7);
        assert_eq!(a, b);
        assert_eq!(ea, eb);
        // env i's trajectory doesn't depend on the batch size (n=200 uses rayon, n=1 doesn't)
        let (c, _) = run(1, 300, 7);
        assert_eq!(&a[..OBS_DIM], &c[..OBS_DIM]);
        assert!(ea > 0, "some episodes should end in 300 steps");
    }
}
