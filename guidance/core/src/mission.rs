//! One return-to-launch-site flight: ascent script -> separation -> flip -> boostback ->
//! coast -> aero-steered descent -> landing burn -> touchdown, flown in the full ksp-core sim.
//!
//! The guidance only ever reads the *nominal* vessel spec and the sim state; the sim integrates a
//! *truth* spec that may carry unknown model errors (thrust, drag) for robustness runs.

use crate::attitude::{self, capacity, effectiveness};
use crate::craft::{Built, Rng};
use crate::gfold;
use crate::landing;
use crate::nav::{aero_force, atm_at, nav, p_atm, propulsion, propellant, touchdown_height, usable_propellant, AeroTables, Nav, Pad, OMEGA};
use crate::predict::{landing_need, predict, LandingBurnModel, PredCraft, PredOpts, PredResult};
use ksp_core::body::KERBIN;
use ksp_core::engine::ENGINE_G0;
use ksp_core::ksc;
use ksp_core::math::{v3, V3};
use ksp_core::sim::{self, Control, EnvState, Params, FIXED_DT, STATUS_CRASHED, STATUS_FLYING, STATUS_LANDED};
use ksp_core::vessel::VesselSpec;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Phase {
    Ascent = 0,
    Flip = 1,
    Boostback = 2,
    Coast = 3,
    Descent = 4,
    LandingBurn = 5,
    Touchdown = 6,
    Done = 7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Outcome {
    Running = 0,
    Landed = 1,
    Crashed = 2,
    TippedOver = 3,
    Timeout = 4,
    /// the booster could not reach the pad (boostback infeasible / ran dry before touchdown)
    NoReturn = 5,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LandingMode {
    /// suicide-burn rule (vertical) + ZEM/ZEV divert (horizontal), closed-loop throttle
    Baseline,
    /// convex powered-descent guidance (lossless convexification), re-planned, tracked
    Gfold,
    /// actions from outside (RL hook): normalised acceleration command, optionally as a residual
    /// on top of the baseline command
    External { residual: bool },
}

#[derive(Clone, Copy, Debug)]
pub struct GuidanceConfig {
    pub landing: LandingMode,
    /// suicide-burn ignition: ignite when the stop-at-pad throttle reaches this fraction
    pub f_ign: f64,
    /// touchdown descent rate (m/s)
    pub v_td: f64,
    /// record a decimated trajectory
    pub record: bool,
    /// stop the flight at landing-burn ignition (for building terminal-phase start states)
    pub stop_at_ignition: bool,
}

impl Default for GuidanceConfig {
    fn default() -> Self {
        GuidanceConfig { landing: LandingMode::Baseline, f_ign: 0.7, v_td: 1.5, record: false, stop_at_ignition: false }
    }
}

/// Per-flight random dispersions (separation state comes from these + the ascent).
#[derive(Clone, Copy, Debug, Default)]
pub struct Dispersion {
    pub ut: f64,
    pub heading_deg: f64,
    pub kick_speed: f64,
    pub kick_deg: f64,
    pub turn_alt: f64,
    pub turn_exp: f64,
    /// MECO when booster propellant falls to this multiple of the estimated RTLS need
    pub meco_margin: f64,
    /// booster propellant load at liftoff (fraction)
    pub fuel_load: f64,
    /// separation velocity / angular-rate kicks (m/s ENU, rad/s world)
    pub sep_dv: V3,
    pub sep_w: V3,
    /// unknown-to-guidance model errors in the truth spec (fractions)
    pub thrust_err: f64,
    pub drag_err: f64,
}

pub fn sample_dispersion(rng: &mut Rng, model_errors: bool) -> Dispersion {
    Dispersion {
        ut: rng.unif(0.0, 21600.0),
        heading_deg: 90.0 + rng.unif(-4.0, 4.0),
        kick_speed: rng.unif(40.0, 90.0),
        kick_deg: rng.unif(2.0, 8.0),
        turn_alt: rng.unif(45_000.0, 75_000.0),
        turn_exp: rng.unif(0.4, 0.65),
        meco_margin: rng.unif(1.15, 1.45),
        fuel_load: rng.unif(0.85, 1.0),
        sep_dv: v3(rng.normal(), rng.normal(), rng.normal()) * 1.5,
        sep_w: v3(rng.normal(), rng.normal(), rng.normal()) * 0.02,
        thrust_err: if model_errors { rng.normal() * 0.02 } else { 0.0 },
        drag_err: if model_errors { rng.normal() * 0.05 } else { 0.0 },
    }
}

/// Design-level data shared by every flight of a design (nominal specs and aero tables).
pub struct Shared {
    pub built: Built,
    pub tables: AeroTables,
    pub m_dry: f64,
}

impl Shared {
    pub fn new(built: Built) -> Shared {
        let mut low = built.booster.full_resources();
        low[0] *= 0.15;
        low[1] *= 0.15;
        let tables = AeroTables::build(&built.booster, &low);
        let m_dry = built.booster.dry_mass();
        Shared { built, tables, m_dry }
    }
}

fn truth_spec(nominal: &VesselSpec, d: &Dispersion) -> VesselSpec {
    let mut s = *nominal;
    for k in 0..s.n_engines as usize {
        s.engines[k].max_fuel_flow *= 1.0 + d.thrust_err;
        s.engines[k].min_fuel_flow *= 1.0 + d.thrust_err;
    }
    for i in 0..s.n_parts as usize {
        for a in s.part_faces[i].area.iter_mut() {
            *a *= 1.0 + d.drag_err;
        }
    }
    s
}

/// Summary of one flight (all SI: m, m/s, t, s).
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub sep_t: f64,
    pub sep_alt: f64,
    pub sep_speed: f64,
    pub sep_vh: f64,
    pub sep_vz: f64,
    pub sep_downrange: f64,
    pub sep_mass: f64,
    pub sep_prop_frac: f64,
    pub bb_dv_plan: f64,
    pub bb_dv_used: f64,
    pub bb_end_err: f64,
    pub apo_alt: f64,
    pub entry_err: f64,
    pub ign_alt: f64,
    pub ign_speed: f64,
    pub ign_err: f64,
    pub ign_t: f64,
    pub td_t: f64,
    pub td_e: f64,
    pub td_n: f64,
    pub td_vz: f64,
    pub td_vh: f64,
    pub td_tilt_deg: f64,
    pub final_e: f64,
    pub final_n: f64,
    pub final_tilt_deg: f64,
    pub prop_left: f64,
    pub prop_cap: f64,
    pub dv_left: f64,
    pub landing_dv: f64,
    pub max_q_descent: f64,
    pub gfold_solves: u32,
    pub gfold_fail: u32,
    /// descent trim burns flown, and their total delta-v (m/s)
    pub trim_burns: u32,
    pub trim_dv: f64,
}

/// Recorded trajectory sample: t, pad-ENU x/y/z (tangent frame at the pad), phase, throttle.
pub type RecSample = [f32; 6];

/// The landing divert aims to be done this long before touchdown (s).
pub const TAIL_S: f64 = 3.0;
/// Highest planned landing-burn deceleration (in g above hover).
pub const MAX_LANDING_G: f64 = 3.0;

/// Terminal-phase observation for an external policy (see `Flight::terminal_obs`).
pub const TERM_OBS_DIM: usize = 20;

#[derive(Clone)]
pub struct Flight {
    pub sh: Arc<Shared>,
    /// stopped at landing-burn ignition (`stop_at_ignition`): resumable from this state
    pub paused: bool,
    pub truth_stack: VesselSpec,
    pub truth_booster: VesselSpec,
    pub disp: Dispersion,
    pub cfg: GuidanceConfig,
    pub prm: Params,
    pub st: EnvState,
    pub phase: Phase,
    pub outcome: Outcome,
    pub stats: Stats,
    pub rec: Vec<RecSample>,
    // guidance memory
    t_phase: f64,
    guid_timer: f64,
    rec_timer: f64,
    cmd_dir: V3,
    cmd_thr: f64,
    bb_dv: V3,
    bb_fail: u32,
    aoa: (f64, f64),
    entry_dir: V3,
    contact_t: f64,
    /// boostback aim offset from the pad (east, north, m): long when the return arc stays in the
    /// thick air, so the leftover error is a shortfall a retro trim can remove
    aim_bias: (f64, f64),
    /// time limit of the current boostback / trim burn (s, phase time)
    burn_limit: f64,
    hold_h: f64,
    t_settle: f64,
    last_thrust: f64,
    prop_at_ign: f64,
    pub gplan: Option<gfold::Plan>,
    /// external terminal-phase action (normalised ENU acceleration command)
    pub ext_action: [f64; 3],
}

/// `KSPG_DEBUG=1` prints guidance internals (single-flight debugging).
pub fn debug() -> bool {
    static D: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *D.get_or_init(|| std::env::var("KSPG_DEBUG").is_ok_and(|v| v == "1"))
}

#[inline]
fn tilt_deg(n: &Nav) -> f64 {
    n.up.dot(n.x.normalized()).clamp(-1.0, 1.0).acos().to_degrees()
}

impl Flight {
    pub fn new(sh: Arc<Shared>, disp: Dispersion, cfg: GuidanceConfig) -> Flight {
        let truth_stack = truth_spec(&sh.built.stack, &disp);
        let truth_booster = truth_spec(&sh.built.booster, &disp);
        let prm = Params::default();
        // on the pad, legs at static compression, booster propellant at the sampled load
        let mut st = sim::state_landed(&sh.built.stack, disp.ut);
        st.res[0] *= disp.fuel_load;
        st.res[1] *= disp.fuel_load;
        let pad = Pad::ksc();
        Flight {
            sh,
            paused: false,
            truth_stack,
            truth_booster,
            disp,
            cfg,
            prm,
            st,
            phase: Phase::Ascent,
            outcome: Outcome::Running,
            stats: Stats::default(),
            rec: vec![],
            t_phase: 0.0,
            guid_timer: 0.0,
            rec_timer: 0.0,
            cmd_dir: pad.u,
            cmd_thr: 0.0,
            bb_dv: V3::ZERO,
            bb_fail: 0,
            aoa: (0.0, 0.0),
            entry_dir: pad.u,
            contact_t: 0.0,
            aim_bias: (0.0, 0.0),
            burn_limit: f64::INFINITY,
            hold_h: 2.0,
            t_settle: 1.0,
            last_thrust: 0.0,
            prop_at_ign: 0.0,
            gplan: None,
            ext_action: [0.0; 3],
        }
    }

    /// Nominal spec of the current configuration (what the guidance believes).
    pub fn spec(&self) -> &VesselSpec {
        if self.phase == Phase::Ascent { &self.sh.built.stack } else { &self.sh.built.booster }
    }

    fn truth(&self) -> &VesselSpec {
        if self.phase == Phase::Ascent { &self.truth_stack } else { &self.truth_booster }
    }

    fn pred_craft(&self, n: &Nav) -> PredCraft<'_> {
        PredCraft { spec: &self.sh.built.booster, tables: &self.sh.tables, m_dry: self.sh.m_dry, h_td: touchdown_height(&self.sh.built.booster, n.com_l), toff: self.st.toff as f64, body_x: n.q.rotate(v3(1.0, 0.0, 0.0)) }
    }

    /// Ignition throttle actually used: the configured fraction, but never planning a landing
    /// deceleration above MAX_LANDING_G (a high-TWR booster otherwise gets a burn too short to
    /// divert and settle its attitude). `m` is the landing mass estimate.
    fn f_ign_eff(&self, m: f64) -> f64 {
        self.burn_model().threshold(m)
    }

    fn burn_model(&self) -> LandingBurnModel {
        let t_pad = propulsion(&self.sh.built.booster, p_atm(ksc::KSC_PAD_DECK_ALT)).thrust;
        LandingBurnModel { f_ign: self.cfg.f_ign, max_g: MAX_LANDING_G, t_pad, v_td: self.cfg.v_td }
    }

    fn set_phase(&mut self, p: Phase) {
        self.phase = p;
        self.t_phase = 0.0;
        self.guid_timer = 0.0;
    }

    pub fn done(&self) -> bool {
        self.phase == Phase::Done || self.paused
    }

    /// Predicted powered touchdown (landing-burn model) from the current state + impulse dv.
    fn pip(&self, n: &Nav, dv: V3, aoa: (f64, f64)) -> PredResult {
        self.pip_at(n, n.v + dv, n.m, aoa)
    }

    fn pip_at(&self, n: &Nav, v: V3, m: f64, aoa: (f64, f64)) -> PredResult {
        let c = self.pred_craft(n);
        let aoa_torque = if aoa != (0.0, 0.0) { self.aoa_torque(n).max(1e-9) } else { 0.0 };
        predict(&c, &PredOpts { aoa, aoa_torque, burn: Some(self.burn_model()), sim_offset: true, ..Default::default() }, n.x, v, m)
    }

    /// Touchdown error vs the current aim point (pad + boostback bias while boosting back).
    fn pip_err(&self, p: &PredResult) -> (f64, f64) {
        let (e, n) = Pad::ksc().ground_offset(p.x);
        if matches!(self.phase, Phase::Flip | Phase::Boostback) && self.stats.trim_burns == 0 {
            (e - self.aim_bias.0, n - self.aim_bias.1)
        } else {
            (e, n)
        }
    }

    // ---------------------------------------------------------------------------------------
    // ascent
    // ---------------------------------------------------------------------------------------

    /// Return budget from the current ascent state: solve the boostback (impulsive, warm-started)
    /// and fly the predicted landing burn on the booster alone. Returns (propellant the return
    /// needs, propellant the booster has) in tonnes; the ascent cuts off when
    /// have <= meco_margin * need.
    fn rtls_budget(&mut self, n_stack: &Nav) -> (f64, f64) {
        let b = self.sh.built.booster;
        let nb = nav(&b, &self.st);
        let _ = n_stack;
        let m_b = nb.m;
        let have = m_b - self.sh.m_dry;
        let guess = if self.bb_dv.norm2() > 0.0 { self.bb_dv } else { self.boostback_guess(&nb) };
        let Some((dv, err)) = self.solve_boostback(&nb, guess, m_b, None) else {
            if debug() {
                eprintln!("  budget: boostback solve failed");
            }
            return (f64::INFINITY, have);
        };
        self.bb_dv = dv;
        if err > 100.0 {
            return (f64::INFINITY, have);
        }
        // finite burn + flip losses: 10 % on the boostback
        let ve_vac = propulsion(&b, 0.0).isp * ENGINE_G0;
        let m_after = m_b * (-1.1 * dv.norm() / ve_vac).exp();
        let p = self.pip_at(&nb, nb.v + dv, m_after, (0.0, 0.0));
        if debug() {
            eprintln!("  t {:.1} budget: dv {:.0} err {:.1} m_b {:.2} m_after {:.2} m_td {:.2} ok {} dry {} have {:.2}", self.st.ut - self.disp.ut, dv.norm(), err, m_b, m_after, p.m, p.ok, p.dry, have);
        }
        if !p.ok || p.dry {
            return (f64::INFINITY, have);
        }
        (m_b - p.m, have)
    }

    fn ascent_dir(&self, n: &Nav) -> V3 {
        let up = n.x.normalized();
        let pad = Pad::ksc();
        let az = self.disp.heading_deg.to_radians();
        let horiz = (pad.e * az.sin() + pad.n * az.cos()).normalized();
        let horiz = (horiz - up * horiz.dot(up)).normalized();
        let spd = n.v.norm();
        if spd < self.disp.kick_speed {
            return up;
        }
        let prog = (self.disp.kick_deg.max(90.0 * (n.alt / self.disp.turn_alt).max(0.0).powf(self.disp.turn_exp))).min(88.0);
        // stay near surface prograde in the thick air
        let vz = n.v.dot(up);
        let pro = (n.v - up * vz).norm().atan2(vz).to_degrees();
        let q = atm_at(n.alt, spd, self.st.toff as f64).q_kpa;
        let lim = if q > 3.0 { 4.0 } else { 10.0 };
        let phi = prog.clamp(pro - lim, pro + lim).clamp(0.0, 89.0).to_radians();
        up * phi.cos() + horiz * phi.sin()
    }

    fn separate(&mut self, n: &Nav) {
        let pad = Pad::ksc();
        let b = &self.sh.built.booster;
        self.stats.sep_t = self.st.ut - self.disp.ut;
        self.stats.sep_alt = n.alt;
        self.stats.sep_speed = n.v.norm();
        let up = n.x.normalized();
        self.stats.sep_vz = n.v.dot(up);
        self.stats.sep_vh = (n.v - up * self.stats.sep_vz).norm();
        let (de, dn) = pad.ground_offset(n.x);
        self.stats.sep_downrange = (de * de + dn * dn).sqrt();
        let (p, cap) = propellant(b, &self.st.res);
        self.stats.sep_prop_frac = p / cap;
        self.stats.sep_mass = b.mass_props(&self.st.res).0;
        // separation kicks
        let dv = n.to_state_frame(pad.vec_from_enu(self.disp.sep_dv));
        self.st.v += dv;
        self.st.w += self.disp.sep_w;
        self.set_phase(Phase::Flip);
        self.bb_dv = V3::ZERO;
    }

    // ---------------------------------------------------------------------------------------
    // boostback
    // ---------------------------------------------------------------------------------------

    /// Newton (min-norm, 3 unknowns / 2 equations) on the powered touchdown error vs an
    /// impulsive velocity change, warm-started from `guess`.
    /// `align`: instead of the minimum-norm member of the 1-D solution family, pick the one best
    /// aligned with this direction (used while burning, so trimming never needs another flip).
    fn solve_boostback(&self, n: &Nav, guess: V3, m: f64, align: Option<V3>) -> Option<(V3, f64)> {
        let mut dv = guess;
        let mut last_err = f64::INFINITY;
        for _it in 0..10 {
            let p0 = self.pip_at(n, n.v + dv, m, (0.0, 0.0));
            if !p0.ok {
                return None;
            }
            let e0 = self.pip_err(&p0);
            let en = (e0.0 * e0.0 + e0.1 * e0.1).sqrt();
            if debug() {
                eprintln!("    bb it {_it}: dv ({:.1} {:.1} {:.1}) |{:.1}| err ({:.1}, {:.1}) t {:.0} ign {} dry {}", dv.x, dv.y, dv.z, dv.norm(), e0.0, e0.1, p0.t, p0.ignited, p0.dry);
            }
            last_err = en;
            if en < 0.3 {
                break;
            }
            let h = (0.001 * dv.norm()).max(0.3);
            let mut jac = [[0.0; 3]; 2];
            for (k, ax) in [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)].iter().enumerate() {
                let p = self.pip_at(n, n.v + dv + *ax * h, m, (0.0, 0.0));
                if !p.ok {
                    return None;
                }
                let e = self.pip_err(&p);
                jac[0][k] = (e.0 - e0.0) / h;
                jac[1][k] = (e.1 - e0.1) / h;
            }
            // step = -J^T (J J^T)^-1 e
            let r0 = v3(jac[0][0], jac[0][1], jac[0][2]);
            let r1 = v3(jac[1][0], jac[1][1], jac[1][2]);
            let (a, b, d) = (r0.dot(r0), r0.dot(r1), r1.dot(r1));
            let det = a * d - b * b;
            if det.abs() < 1e-12 {
                return None;
            }
            let l0 = (d * e0.0 - b * e0.1) / det;
            let l1 = (-b * e0.0 + a * e0.1) / det;
            let mut step = -(r0 * l0 + r1 * l1);
            let nul = r0.cross(r1).normalized();
            match align {
                // move along the null direction to minimise the part of dv off the axis
                Some(u) => {
                    let d = dv + step;
                    let pd = d - u * d.dot(u);
                    let pn = nul - u * nul.dot(u);
                    if pn.norm2() > 1e-12 {
                        step -= nul * (pd.dot(pn) / pn.norm2());
                    }
                }
                // pull toward the minimum-norm solution
                None => step -= nul * (0.5 * nul.dot(dv + step)),
            }
            let sn = step.norm();
            if sn > 300.0 {
                step = step * (300.0 / sn);
            }
            dv += step;
        }
        Some((dv, last_err))
    }

    fn boostback_guess(&self, n: &Nav) -> V3 {
        let pad = Pad::ksc();
        let up = n.x.normalized();
        let vz = n.v.dot(up);
        let vh = n.v - up * vz;
        let (de, dn) = pad.ground_offset(n.x);
        let d = pad.e * de + pad.n * dn;
        let g = KERBIN.gm() / n.x.norm2();
        let t = (vz.max(0.0) + (vz.max(0.0).powi(2) + 2.0 * g * n.alt).sqrt()) / g + 30.0;
        d * (-1.15 / t) - vh
    }

    /// Throttle needed so the gimbal gives `alpha_need` (rad/s^2) on top of the wheels.
    fn control_throttle(&self, n: &Nav, axis: V3, alpha_need: f64) -> f64 {
        let spec = self.spec();
        let i_n = axis.dot(v3(
            n.inertia_l[0][0] * axis.x + n.inertia_l[0][1] * axis.y + n.inertia_l[0][2] * axis.z,
            n.inertia_l[1][0] * axis.x + n.inertia_l[1][1] * axis.y + n.inertia_l[1][2] * axis.z,
            n.inertia_l[2][0] * axis.x + n.inertia_l[2][1] * axis.y + n.inertia_l[2][2] * axis.z,
        ));
        let e0 = effectiveness(spec, &self.st, n, 0.0);
        let e1 = effectiveness(spec, &self.st, n, 1.0);
        // the aero moment at this state eats into the authority (worst case: it opposes)
        let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
        let (_, tq) = aero_force(spec, n.com_l, n.q, n.v, V3::ZERO, &atm);
        let a_aero = n.q.conj().rotate(tq).norm() / i_n;
        let need = alpha_need + 1.5 * a_aero;
        let a_rw = capacity(&e0.b, axis) / i_n;
        let a_g = (capacity(&e1.b, axis) - capacity(&e0.b, axis)) / i_n;
        if a_rw >= need || a_g <= 0.0 {
            return 0.0;
        }
        ((need - a_rw) / a_g).clamp(0.0, 1.0)
    }

    // ---------------------------------------------------------------------------------------
    // descent
    // ---------------------------------------------------------------------------------------

    /// Aero moment the reaction wheels can hold (kN m), half their capacity (the rest damps).
    fn aoa_torque(&self, n: &Nav) -> f64 {
        let e0 = effectiveness(self.spec(), &self.st, n, 0.0);
        0.5 * capacity(&e0.b_rw, v3(1.0, 0.0, 0.0)).min(capacity(&e0.b_rw, v3(0.0, 0.0, 1.0)))
    }

    /// Max angle of attack the reaction wheels can hold against the aero moment now (deg), for a
    /// tilt toward body-fixed direction `t` (None: roll average).
    fn aoa_limit(&self, n: &Nav, t: Option<V3>) -> f64 {
        let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
        let phi = t.map(|t| crate::nav::tilt_azimuth(n.q.rotate(v3(1.0, 0.0, 0.0)), -n.v.normalized(), t));
        self.sh.tables.alpha_limit(atm.mach, atm.q_kpa, self.aoa_torque(n), phi)
    }

    fn descent_guidance(&mut self, n: &Nav) {
        // the predictor assumes an engine-first attitude: while the booster is still swinging
        // round (e.g. right after an in-atmosphere boostback) its predictions are meaningless
        let aoa_now = n.up.dot(-n.v.normalized()).clamp(-1.0, 1.0).acos().to_degrees();
        if aoa_now > 30.0 {
            self.aoa = (0.0, 0.0);
            return;
        }
        let p0 = self.pip(n, V3::ZERO, self.aoa);
        if !p0.ok {
            return;
        }
        let e0 = self.pip_err(&p0);
        // command: fraction of the holdable angle of attack along the path (|.| <= 1)
        let amax = 1.0;
        // Jacobian of the touchdown error vs the (east, north) AoA command
        let h = 0.1;
        let pe = self.pip(n, V3::ZERO, (self.aoa.0 + h, self.aoa.1));
        let pn = self.pip(n, V3::ZERO, (self.aoa.0, self.aoa.1 + h));
        if !pe.ok || !pn.ok {
            return;
        }
        let (ee, en) = (self.pip_err(&pe), self.pip_err(&pn));
        let j = [[(ee.0 - e0.0) / h, (en.0 - e0.0) / h], [(ee.1 - e0.1) / h, (en.1 - e0.1) / h]];
        // damped least squares: da = -J^T (J J^T + mu I)^-1 e
        let scale = j[0][0].powi(2) + j[0][1].powi(2) + j[1][0].powi(2) + j[1][1].powi(2);
        if scale < 1e-12 {
            return;
        }
        let mu = 1e-3 * scale;
        let jjt = [[j[0][0] * j[0][0] + j[0][1] * j[0][1] + mu, j[0][0] * j[1][0] + j[0][1] * j[1][1]], [j[0][0] * j[1][0] + j[0][1] * j[1][1], j[1][0] * j[1][0] + j[1][1] * j[1][1] + mu]];
        let det = jjt[0][0] * jjt[1][1] - jjt[0][1] * jjt[1][0];
        let y = ((jjt[1][1] * e0.0 - jjt[0][1] * e0.1) / det, (-jjt[1][0] * e0.0 + jjt[0][0] * e0.1) / det);
        let da = (-(j[0][0] * y.0 + j[1][0] * y.1), -(j[0][1] * y.0 + j[1][1] * y.1));
        let mut a = (self.aoa.0 + da.0, self.aoa.1 + da.1);
        let m = (a.0 * a.0 + a.1 * a.1).sqrt();
        if m > amax {
            a = (a.0 * amax / m, a.1 * amax / m);
        }
        // steering saturated and still well off: a short trim burn along (within 20 deg of) the
        // current thrust axis, solved like the boostback, if the propellant budget allows
        let en = (e0.0 * e0.0 + e0.1 * e0.1).sqrt();
        if m >= amax * 0.99 && en > 30.0 && aoa_now < 20.0 && n.alt > p0.ign_h + 2000.0 && self.stats.trim_burns < 3 {
            if let Some((dv, err)) = self.solve_boostback(n, n.up * 2.0, n.m, Some(n.up)) {
                let ang = dv.normalized().dot(n.up).clamp(-1.0, 1.0).acos();
                let ve = propulsion(self.spec(), p_atm(n.alt)).isp * ENGINE_G0;
                let spare = n.m - self.sh.m_dry - 1.3 * (n.m - p0.m);
                let cost = n.m * (1.0 - (-dv.norm() / ve).exp());
                if debug() {
                    eprintln!("  trim? e {en:.0} -> {err:.1} dv {:.1} ang {:.1} cost {cost:.2} spare {spare:.2}", dv.norm(), ang.to_degrees());
                }
                if err < 0.3 * en && ang < 20f64.to_radians() && cost < 0.5 * spare {
                    self.bb_dv = dv;
                    self.stats.trim_burns += 1;
                    self.stats.trim_dv += dv.norm();
                    let pr = propulsion(self.spec(), p_atm(n.alt));
                    self.set_phase(Phase::Boostback);
                    self.burn_limit = 2.0 * crate::predict::burn_time(pr.thrust, pr.isp, n.m, dv.norm()) + 3.0;
                    return;
                }
            }
        }
        if debug() {
            let tilt = n.up.dot(-n.v.normalized()).clamp(-1.0, 1.0).acos().to_degrees();
            eprintln!("  t {:.1} alt {:.0} v {:.0} err ({:.1},{:.1}) J [{:.2} {:.2}; {:.2} {:.2}] aoa_cmd ({:.2},{:.2}) lim {:.2} actual_aoa {:.1} ign_h {:.0}", self.st.ut - self.disp.ut, n.alt, n.v.norm(), e0.0, e0.1, j[0][0], j[0][1], j[1][0], j[1][1], a.0, a.1, self.aoa_limit(n, None), tilt, p0.ign_h);
        }
        self.aoa = a;
    }

    fn descent_dir(&self, n: &Nav) -> V3 {
        let pad = Pad::ksc();
        let vh = n.v.normalized();
        let base = -vh;
        let (ae, an) = self.aoa;
        let am = (ae * ae + an * an).sqrt();
        if am < 1e-9 {
            return base;
        }
        let t = pad.e * (ae / am) + pad.n * (an / am);
        let tp = (t - vh * t.dot(vh)).normalized();
        let m = am.min(1.0) * self.aoa_limit(n, Some(tp));
        if m < 1e-6 {
            return base;
        }
        let a = m.to_radians();
        base * a.cos() + tp * a.sin()
    }

    // ---------------------------------------------------------------------------------------
    // landing burn
    // ---------------------------------------------------------------------------------------

    /// Local landing frame: position relative to the touchdown point (ENU at the pad, with the
    /// vertical as spherical height above touchdown height) and velocity in pad ENU.
    fn landing_state(&self, n: &Nav) -> (V3, V3, f64) {
        let pad = Pad::ksc();
        let h_td = touchdown_height(self.spec(), n.com_l);
        let (de, dn) = pad.ground_offset(n.x);
        let h = n.x.norm() - (KERBIN.radius + ksc::KSC_PAD_DECK_ALT) - h_td;
        (v3(de, dn, h), pad.vec_to_enu(n.v), h_td)
    }

    /// Non-thrust accelerations in pad ENU: (gravity + centrifugal, Coriolis, drag at the
    /// current attitude).
    fn natural_parts(&self, n: &Nav) -> (V3, V3, V3) {
        let pad = Pad::ksc();
        let r2 = n.x.norm2();
        let g = n.x * (-KERBIN.gm() / (r2 * r2.sqrt())) - OMEGA.cross(OMEGA.cross(n.x));
        let cor = -(OMEGA.cross(n.v) * 2.0);
        let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
        let (fa, _) = aero_force(self.spec(), n.com_l, n.q, n.v, V3::ZERO, &atm);
        (pad.vec_to_enu(g), pad.vec_to_enu(cor), pad.vec_to_enu(fa / n.m))
    }


    /// Constant-throttle stop solution for the vertical channel from the current state.
    /// The stop is planned to end `hold_height` above touchdown; the last stretch is flown at the
    /// constant touchdown rate (~1.5 s of it), so touchdown speed doesn't hinge on the final metre.
    fn hold_height(&self) -> f64 {
        self.hold_h
    }

    /// Time this craft needs to swing 10 degrees (rest to rest) at throttle `u`, from its wheels
    /// and gimbal torque and its current inertia.
    fn swing_time(&self, n: &Nav, u: f64) -> f64 {
        let e = effectiveness(self.spec(), &self.st, n, u);
        let ixx = n.inertia_l[0][0].max(n.inertia_l[2][2]).max(1e-9);
        let alpha = capacity(&e.b, v3(1.0, 0.0, 0.0)).min(capacity(&e.b, v3(0.0, 0.0, 1.0))) / ixx;
        (2.0 * (2.0 * 10f64.to_radians() / alpha.max(1e-6)).sqrt()).min(6.0)
    }

    /// Height of the constant-rate final segment: 1.5 s plus one 10 degree swing at hover
    /// throttle, so the craft can still null drift there.
    fn update_hold_height(&mut self, n: &Nav) {
        let pr = propulsion(self.spec(), p_atm(n.alt));
        let u_h = (n.m * 9.81 / pr.thrust.max(1e-9)).clamp(0.05, 1.0);
        let t_settle = self.swing_time(n, u_h);
        self.t_settle = t_settle;
        self.hold_h = self.cfg.v_td.max(0.5) * (1.5 + t_settle).clamp(1.5, 8.0);
    }

    /// Tilt limit from vertical during the landing burn: 30 deg high up, tapering to 12 deg at the
    /// hold height, then the leg-footprint limit in the final segment.
    fn tilt_limit(&self, n: &Nav) -> f64 {
        let (r, _, _) = self.landing_state(n);
        let hh = self.hold_height();
        let td = self.touchdown_tilt(n);
        if r.z <= hh {
            return td;
        }
        let lo = td.max(12f64.to_radians());
        lo + (30f64.to_radians() - lo) * ((r.z - hh) / (4.0 * hh)).clamp(0.0, 1.0)
    }

    /// Max tilt at touchdown from the leg footprint: 30 % of the tip-over angle.
    fn touchdown_tilt(&self, n: &Nav) -> f64 {
        let spec = self.spec();
        let r_feet = (0..spec.n_legs as usize).map(|k| (spec.legs[k].foot().x.powi(2) + spec.legs[k].foot().z.powi(2)).sqrt()).fold(f64::INFINITY, f64::min);
        let h = touchdown_height(spec, n.com_l).max(0.1);
        if !r_feet.is_finite() {
            return 2f64.to_radians();
        }
        (0.3f64 * (r_feet / h).atan()).clamp(2f64.to_radians(), 8f64.to_radians())
    }

    fn stop(&self, n: &Nav) -> landing::Stop {
        let (r, v, h_td) = self.landing_state(n);
        let vh = (v.x * v.x + v.y * v.y).sqrt();
        let hh = self.hold_height();
        let mut s = landing::stop_throttle(self.spec(), &self.sh.tables, self.st.toff as f64, h_td + hh, r.z - hh, v.z, vh, n.m, self.sh.m_dry, self.cfg.v_td);
        s.t_go += hh / self.cfg.v_td.max(0.1);
        s
    }

    /// Baseline: requested non-gravitational acceleration (thrust + aero, pad ENU, m/s^2).
    /// Vertical: the constant-throttle stop solution (re-solved every tick) plus the drag it
    /// assumed. Horizontal: ZEM/ZEV to rest over the pad with the stop solution's time to go,
    /// then velocity nulling for the last seconds.
    fn baseline_accel(&self, n: &Nav) -> V3 {
        let (r, v, _) = self.landing_state(n);
        let v_td = self.cfg.v_td;
        let (g, cor, _) = self.natural_parts(n);
        let grav = g + cor;
        let pr = propulsion(self.spec(), p_atm(n.alt));
        let a_max = pr.thrust / n.m;
        let st = self.stop(n);
        let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
        let spd = v.norm().max(1e-6);
        let d0 = atm.q_kpa * self.sh.tables.cd(atm.mach, 0.0) * atm.pseudo_re_drag_mult / n.m;
        let hh = self.hold_height();
        let az = if r.z > hh && v.z < -v_td {
            st.u.min(1.0) * a_max + d0 * (-v.z / spd)
        } else {
            // final metres: hold the touchdown rate (drag is negligible here)
            -grav.z + 3.0 * (-v_td - v.z)
        };
        // horizontal: ZEM/ZEV to rest over the pad, timed to finish before the hold segment plus
        // one attitude swing of this craft. Its effective time to go never drops below
        // ~5 swing times: ZEM/ZEV is a PD with gains 6/t^2, 4/t, and an outer loop faster than the
        // attitude loop that realises it would just oscillate. In the hold, saturating the
        // footprint tilt limit, the drift is killed first (touchdown speed matters more than the
        // last metre of position).
        let t_td = if r.z > hh { st.t_go } else { r.z.max(0.0) / v_td.max(0.1) };
        let t_sw = if r.z > hh { self.swing_time(n, st.u.clamp(0.05, 1.0)) } else { self.t_settle };
        let t_min = (3.0 * t_sw).max(1.0);
        let t_h = (t_td - hh / v_td.max(0.1) - self.t_settle).max(t_min);
        let zem = v3(-6.0 * r.x / (t_h * t_h) - 4.0 * v.x / t_h, -6.0 * r.y / (t_h * t_h) - 4.0 * v.y / t_h, 0.0);
        let lim = self.tilt_limit(n).tan() * az.max(0.0);
        let mut ah = if r.z > hh || (zem.x * zem.x + zem.y * zem.y).sqrt() <= lim { zem } else { v3(-v.x, -v.y, 0.0) * (1.0 / (2.0 * self.t_settle).max(0.8)) };
        ah.x -= grav.x;
        ah.y -= grav.y;
        v3(ah.x, ah.y, az)
    }

    /// Requested non-gravitational acceleration (pad ENU) -> (thrust direction, throttle), with
    /// the craft's lift and drag at the angle of attack it implies (`landing::allocate`), a tilt
    /// limit from vertical, and vertical priority when thrust runs short.
    fn allocate_cmd(&self, n: &Nav, a_req: V3) -> (V3, f64) {
        let pad = Pad::ksc();
        let pr = propulsion(self.spec(), p_atm(n.alt));
        if pr.thrust <= 0.0 {
            return (pad.u, 0.0);
        }
        let vb = n.v;
        let spd = vb.norm();
        let up = n.x.normalized();
        // flow axis (engine-first); straight up when nearly stopped
        let w = if spd > 3.0 { -vb / spd } else { up };
        let a = pad.vec_from_enu(a_req);
        let a_w = a.dot(w);
        let ap_vec = a - w * a_w;
        let a_p = ap_vec.norm();
        let t_hat = if a_p > 1e-9 { ap_vec / a_p } else { V3::ZERO };
        let atm = atm_at(n.alt, spd, self.st.toff as f64);
        let (q, mach, pre) = if spd > 3.0 { (atm.q_kpa, atm.mach, atm.pseudo_re_drag_mult) } else { (0.0, 0.0, 1.0) };
        // largest angle of attack the wheels + gimbal (at the throttle this burn runs) can hold
        let thr_est = (n.m * a_w / pr.thrust).clamp(0.05, 1.0);
        let e1 = effectiveness(self.spec(), &self.st, n, thr_est);
        let tau = 0.6 * capacity(&e1.b, v3(1.0, 0.0, 0.0)).min(capacity(&e1.b, v3(0.0, 0.0, 1.0)));
        let phi = if t_hat.norm2() > 0.0 { Some(crate::nav::tilt_azimuth(n.q.rotate(v3(1.0, 0.0, 0.0)), w, t_hat)) } else { None };
        let a_hold = if q > 0.0 { self.sh.tables.alpha_limit(mach, q, tau, phi).min(25.0) } else { 25.0 };
        let (alpha, u, got) = landing::allocate(&self.sh.tables, mach, q, pre, pr.thrust, n.m, a_w, a_p, a_hold.max(0.5), phi);
        if debug() && self.st.ticks % 10 == 0 {
            eprintln!("     alloc q {:.1} M {:.2} a_w {:.1} a_p {:.2} -> alpha {:.2} deg u {:.2} got {:.2}", q, mach, a_w, a_p, alpha.to_degrees(), u, got);
        }
        // the box-shaped cubes also push sideways at off-axis azimuths: rotate the tilt plane
        // about the flow axis so the side force cancels (two fixed-point passes)
        let mut t_use = t_hat;
        if t_hat.norm2() > 0.0 && q > 0.0 && alpha.abs() > 1e-4 {
            let ad = alpha.to_degrees();
            for _ in 0..2 {
                let ph = crate::nav::tilt_azimuth(n.q.rotate(v3(1.0, 0.0, 0.0)), w, t_use * alpha.signum());
                let cs = q * self.sh.tables.cs_at(mach, ad.abs(), Some(ph)) * alpha.signum();
                let along = pr.thrust * u * alpha.sin() + q * self.sh.tables.cl_at(mach, ad.abs(), Some(ph)) * alpha.signum();
                if along.abs() < 1e-9 {
                    break;
                }
                let delta = (cs / along).atan();
                let s_hat = w.cross(t_use);
                t_use = (t_use * delta.cos() - s_hat * delta.sin()).normalized();
            }
        }
        let mut dir = if t_use.norm2() > 0.0 { w * alpha.cos() + t_use * alpha.sin() } else { w };
        let max_tilt = self.tilt_limit(n);
        let c = dir.dot(up);
        if c < max_tilt.cos() {
            let h = (dir - up * c).normalized();
            dir = up * max_tilt.cos() + h * max_tilt.sin();
        }
        // throttle: deliver the vertical part through the current (lagging) axis
        let up_z = n.up.dot(up).max(0.3);
        let d_z = if spd > 3.0 { atm.q_kpa * self.sh.tables.cd(atm.mach, 0.0) * pre * (-vb.dot(up) / spd) } else { 0.0 };
        let need_z = (n.m * a.dot(up) - d_z).max(0.0);
        let thr = (need_z / (pr.thrust * up_z)).max(u * 0.0).clamp(0.0, 1.0);
        (dir, thr)
    }

    /// Observation for an external terminal-phase policy (craft-conditioned, scale-free):
    /// position / velocity relative to the touchdown point, scaled by the craft's own stopping
    /// scales, plus the craft parameters the classic guidance reads.
    pub fn terminal_obs(&self) -> [f32; TERM_OBS_DIM] {
        let n = nav(self.spec(), &self.st);
        let (r, v, h_td) = self.landing_state(&n);
        let pr = propulsion(self.spec(), p_atm(n.alt));
        let a_max = pr.thrust / n.m;
        let g = 9.81;
        let v_s = (2.0 * (a_max - g).max(0.5) * r.z.max(1.0)).sqrt().max(1.0);
        let l_s = r.z.max(10.0);
        let base = self.baseline_accel(&n);
        let up = Pad::ksc().vec_to_enu(n.up);
        let eff = effectiveness(self.spec(), &self.st, &n, 1.0);
        let i_n = n.inertia_l[0][0].max(1e-9);
        let alpha = capacity(&eff.b, v3(1.0, 0.0, 0.0)) / i_n;
        let (p, cap) = propellant(self.spec(), &self.st.res);
        let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
        let o = [
            r.x / l_s, r.y / l_s, (r.z / 1000.0), v.x / v_s, v.y / v_s, v.z / v_s,
            up.x, up.y, up.z,
            a_max / g, (n.m * g / pr.thrust.max(1e-9)), alpha.min(10.0), (self.sh.tables.cd(0.3, 0.0) / n.m).min(10.0),
            p / cap.max(1e-9), (h_td / 10.0), atm.q_kpa / 50.0,
            base.x / a_max, base.y / a_max, base.z / a_max, self.st.thr[0],
        ];
        let mut out = [0f32; TERM_OBS_DIM];
        for (k, x) in o.iter().enumerate() {
            out[k] = *x as f32;
        }
        out
    }

    // ---------------------------------------------------------------------------------------
    // the tick
    // ---------------------------------------------------------------------------------------

    fn record(&mut self, n: &Nav, force: bool) {
        if !self.cfg.record {
            return;
        }
        let period = match self.phase {
            Phase::Descent | Phase::LandingBurn | Phase::Touchdown => 0.2,
            _ => 1.0,
        };
        if force || self.rec_timer <= 0.0 {
            let p = Pad::ksc().to_enu(n.x);
            self.rec.push([(self.st.ut - self.disp.ut) as f32, p.x as f32, p.y as f32, p.z as f32, self.phase as u8 as f32, self.cmd_thr as f32]);
            self.rec_timer = period;
        }
    }

    /// Advance one 0.02 s physics tick (guidance + control + sim).
    pub fn tick(&mut self) {
        if self.done() {
            return;
        }
        let n = nav(self.spec(), &self.st);
        let dt = FIXED_DT;
        self.guid_timer -= dt;
        self.rec_timer -= dt;
        self.t_phase += dt;
        let due = self.guid_timer <= 0.0;
        self.record(&n, false);
        let mut aero_ff = true;
        match self.phase {
            Phase::Ascent => {
                self.cmd_dir = self.ascent_dir(&n);
                self.cmd_thr = 1.0;
                if due {
                    let (p, cap) = propellant(&self.sh.built.booster, &self.st.res);
                    // the budget only matters once the booster is past half its load
                    self.guid_timer = if p > 0.6 * cap { 2.0 } else { 0.5 };
                    let (need, have) = if p > 0.6 * cap { (0.0, 1.0) } else { self.rtls_budget(&n) };
                    let vz = n.v.dot(n.x.normalized());
                    if (self.t_phase > 10.0 && have <= self.disp.meco_margin * need) || p < 0.02 * cap || (vz < 0.0 && n.alt > 1000.0) {
                        self.separate(&n);
                        self.record(&n, true);
                        return self.tick_sim(Control::default());
                    }
                }
            }
            Phase::Flip | Phase::Boostback => {
                if due {
                    self.guid_timer = if self.phase == Phase::Flip { 0.5 } else { 0.2 };
                    let guess = if self.bb_dv.norm2() > 0.0 { self.bb_dv } else { self.boostback_guess(&n) };
                    // final trim only: keep the residual on the thrust axis
                    let trim = self.phase == Phase::Boostback && (self.stats.trim_burns > 0 || self.bb_dv.norm() < (0.05 * self.stats.bb_dv_plan).max(5.0));
                    let align = if trim { Some(n.up) } else { None };
                    match self.solve_boostback(&n, guess, n.m, align) {
                        Some((dv, _err)) => {
                            self.bb_dv = dv;
                            self.bb_fail = 0;
                            if self.stats.bb_dv_plan == 0.0 {
                                self.stats.bb_dv_plan = dv.norm();
                                // arc apogee in the dense air: aim past the pad by 5 % of the
                                // current downrange (<= 800 m)
                                let p = self.pip(&n, dv, (0.0, 0.0));
                                if p.ok && p.apo < 50_000.0 {
                                    let (de, dn) = Pad::ksc().ground_offset(n.x);
                                    let d = (de * de + dn * dn).sqrt().max(1.0);
                                    let b = (0.05 * d).min(800.0);
                                    self.aim_bias = (-de / d * b, -dn / d * b);
                                }
                            }
                        }
                        None => self.bb_fail += 1,
                    }
                }
                let dvn = self.bb_dv.norm();
                if dvn > 1e-9 {
                    self.cmd_dir = self.bb_dv / dvn;
                }
                let pr = propulsion(self.spec(), p_atm(n.alt));
                let err = n.up.dot(self.cmd_dir).clamp(-1.0, 1.0).acos();
                if self.phase == Phase::Flip {
                    // gimbal-assisted flip if the wheels alone are too slow (aim: ~20 s for 180 deg)
                    let axis = n.q.conj().rotate(n.up.cross(self.cmd_dir)).normalized();
                    let alpha_need = 4.0 * std::f64::consts::PI / (20.0f64 * 20.0);
                    self.cmd_thr = if axis.norm2() > 0.0 { self.control_throttle(&n, axis, alpha_need) } else { 0.0 };
                    if err < 10f64.to_radians() && self.t_phase > 1.0 {
                        self.set_phase(Phase::Boostback);
                    }
                    if self.t_phase > 120.0 {
                        self.set_phase(Phase::Boostback);
                    }
                } else {
                    let tau_end = 1.5;
                    let thr = (dvn * n.m / (pr.thrust * tau_end)).clamp(0.0, 1.0);
                    self.cmd_thr = if err < 20f64.to_radians() { thr.max(pr.throttle_min) } else { 0.05 };
                    let (p_left, _) = propellant(self.spec(), &self.st.res);
                    // done: residual below 0.25 m/s; or the residual points off the thrust axis
                    // (chasing it would mean another flip) and is either tiny or already worth
                    // less than 20 m at the pad (descent steering and the landing divert take it);
                    // or the burn has run far past its plan
                    let off_axis = err > 20f64.to_radians();
                    let pip_ok = off_axis && {
                        let pp = self.pip(&n, V3::ZERO, (0.0, 0.0));
                        let e = self.pip_err(&pp);
                        (e.0 * e.0 + e.1 * e.1).sqrt() < 20.0
                    };
                    let t_plan = crate::predict::burn_time(pr.thrust, pr.isp, self.stats.sep_mass, self.stats.bb_dv_plan);
                    // in trim mode the residual is kept on the axis; pointing backwards = overshot
                    let overshot = (self.stats.trim_burns > 0 || dvn < (0.05 * self.stats.bb_dv_plan).max(5.0)) && self.bb_dv.dot(n.up) <= 0.0;
                    let limit = if self.stats.trim_burns > 0 { self.burn_limit } else { 2.0 * t_plan + 30.0 };
                    // a trim never runs into the landing burn
                    let (rr, _, _) = self.landing_state(&n);
                    let landing_due = self.stats.trim_burns > 0 && landing_need(self.spec(), &self.sh.tables, self.st.toff as f64, n.x, n.v, n.m, rr.z, self.cfg.v_td).0 >= 0.8 * self.f_ign_eff(n.m);
                    if dvn < 0.25 || overshot || landing_due || (off_axis && dvn < 0.5) || pip_ok || self.t_phase > limit || p_left <= 1e-6 || self.bb_fail > 25 {
                        let m_sep = self.stats.sep_mass;
                        let ve = propulsion(self.spec(), 0.0).isp * ENGINE_G0;
                        self.stats.bb_dv_used = ve * (m_sep / n.m).ln();
                        let pp = self.pip(&n, V3::ZERO, (0.0, 0.0));
                        let e = self.pip_err(&pp);
                        self.stats.bb_end_err = (e.0 * e.0 + e.1 * e.1).sqrt();
                        self.cmd_thr = 0.0;
                        self.set_phase(Phase::Coast);
                    }
                }
                if debug() && self.st.ticks % 25 == 0 {
                    eprintln!("  BB phase {:?} t {:.1} alt {:.0} dv_rem {:.1} err_ang {:.1} thr {:.2} fail {}", self.phase, self.st.ut - self.disp.ut, n.alt, self.bb_dv.norm(), err.to_degrees(), self.cmd_thr, self.bb_fail);
                }
                // decrement the remaining dv by what this tick's thrust will deliver
                if self.phase == Phase::Boostback && self.cmd_thr > 0.0 {
                    let a = n.up * (pr.thrust * self.cmd_thr / n.m);
                    self.bb_dv -= a * dt;
                }
            }
            Phase::Coast => {
                self.cmd_thr = 0.0;
                let up = n.x.normalized();
                let vz = n.v.dot(up);
                self.stats.apo_alt = self.stats.apo_alt.max(n.alt);
                if due {
                    self.guid_timer = 1.0;
                    if n.alt > KERBIN.atmosphere_depth {
                        let c = self.pred_craft(&n);
                        let p = predict(&c, &PredOpts { stop_alt: Some(KERBIN.atmosphere_depth - 1.0), sim_offset: true, ..Default::default() }, n.x, n.v, n.m);
                        if p.ok {
                            self.entry_dir = -p.v.normalized();
                        }
                    } else {
                        self.entry_dir = -n.v.normalized();
                    }
                }
                self.cmd_dir = self.entry_dir;
                // in the air (descending, or a low apogee still inside the atmosphere): steer
                let _ = vz;
                if n.alt < KERBIN.atmosphere_depth {
                    let pp = self.pip(&n, V3::ZERO, (0.0, 0.0));
                    let e = self.pip_err(&pp);
                    self.stats.entry_err = (e.0 * e.0 + e.1 * e.1).sqrt();
                    self.set_phase(Phase::Descent);
                }
            }
            Phase::Descent => {
                self.cmd_thr = 0.0;
                self.stats.apo_alt = self.stats.apo_alt.max(n.alt);
                let atm = atm_at(n.alt, n.v.norm(), self.st.toff as f64);
                self.stats.max_q_descent = self.stats.max_q_descent.max(atm.q_kpa);
                if due {
                    self.guid_timer = 0.5;
                    self.descent_guidance(&n);
                }
                self.cmd_dir = self.descent_dir(&n);
                // ignition check (every tick once the quick estimate is close)
                let (r, _, _) = self.landing_state(&n);
                let (quick, _) = landing_need(self.spec(), &self.sh.tables, self.st.toff as f64, n.x, n.v, n.m, r.z, self.cfg.v_td);
                let f_ign = self.f_ign_eff(n.m);
                if quick >= f_ign {
                    self.update_hold_height(&n);
                    if debug() {
                        eprintln!("  ignition: quick {:.3} stop-solver {:.3} f_ign {:.3}", quick, self.stop(&n).u, f_ign);
                    }
                    self.stats.ign_alt = n.alt;
                    self.stats.ign_speed = n.v.norm();
                    let pp = self.pip(&n, V3::ZERO, (0.0, 0.0));
                    let e = self.pip_err(&pp);
                    self.stats.ign_err = (e.0 * e.0 + e.1 * e.1).sqrt();
                    self.stats.ign_t = self.st.ut - self.disp.ut;
                    self.prop_at_ign = propellant(self.spec(), &self.st.res).0;
                    self.set_phase(Phase::LandingBurn);
                    if self.cfg.stop_at_ignition {
                        self.paused = true;
                        return;
                    }
                }
            }
            Phase::LandingBurn => {
                let a_cmd = match self.cfg.landing {
                    LandingMode::Baseline => self.baseline_accel(&n),
                    LandingMode::Gfold => self.gfold_accel(&n),
                    LandingMode::External { residual } => {
                        let pr = propulsion(self.spec(), p_atm(n.alt));
                        let amax = pr.thrust / n.m;
                        let ext = v3(self.ext_action[0], self.ext_action[1], self.ext_action[2]) * amax;
                        if residual { self.baseline_accel(&n) + ext } else { ext }
                    }
                };
                let (dir, thr) = self.allocate_cmd(&n, a_cmd);
                self.cmd_dir = dir;
                self.cmd_thr = thr;
                if debug() && (self.st.ticks % 10 == 0 || n.alt < 120.0) {
                    let (r, v, _) = self.landing_state(&n);
                    let up = Pad::ksc().vec_to_enu(n.up);
                    eprintln!("  LB t {:.1} r ({:.1} {:.1} {:.1}) v ({:.1} {:.1} {:.1}) a_cmd ({:.1} {:.1} {:.1}) thr {:.2} up ({:.3} {:.3} {:.3}) m {:.2}", self.st.ut - self.disp.ut, r.x, r.y, r.z, v.x, v.y, v.z, a_cmd.x, a_cmd.y, a_cmd.z, thr, up.x, up.y, up.z, n.m);
                }
                aero_ff = true;
            }
            Phase::Touchdown => {
                self.cmd_thr = 0.0;
                self.cmd_dir = n.x.normalized();
            }
            Phase::Done => {}
        }
        let o = attitude::point(self.spec(), &self.st, &n, self.cmd_dir, self.cmd_thr, aero_ff);
        if debug() && self.phase == Phase::LandingBurn && self.st.ticks % 10 == 0 {
            eprintln!("     att err {:.1} deg alpha_avail {:.3} ctl p {:.2} y {:.2} r {:.2}", o.err.to_degrees(), o.alpha_avail, o.ctl.pitch, o.ctl.yaw, o.ctl.roll);
        }
        self.tick_sim(o.ctl);
    }

    fn tick_sim(&mut self, ctl: Control) {
        let truth = *self.truth();
        let prm = self.prm;
        let info = sim::tick(&truth, &prm, &mut self.st, &ctl);
        self.last_thrust = info.thrust;
        let n = nav(self.spec(), &self.st);
        let elapsed = self.st.ut - self.disp.ut;
        if self.phase != Phase::Ascent && self.phase != Phase::Touchdown && self.st.status != STATUS_FLYING {
            // first ground contact: touchdown statistics
            let (r, v, _) = self.landing_state(&n);
            self.stats.td_t = elapsed;
            self.stats.td_e = r.x;
            self.stats.td_n = r.y;
            self.stats.td_vz = v.z;
            self.stats.td_vh = (v.x * v.x + v.y * v.y).sqrt();
            self.stats.td_tilt_deg = tilt_deg(&n);
            self.contact_t = 0.0;
            self.record(&n, true);
            self.set_phase(Phase::Touchdown);
        }
        if self.phase == Phase::Touchdown {
            self.contact_t += FIXED_DT;
            let tilt = tilt_deg(&n);
            let end = if self.st.status == STATUS_CRASHED {
                Some(Outcome::Crashed)
            } else if tilt > 45.0 {
                Some(Outcome::TippedOver)
            } else if self.st.status == STATUS_LANDED {
                Some(Outcome::Landed)
            } else if self.contact_t > 20.0 {
                Some(Outcome::Timeout)
            } else {
                None
            };
            if let Some(o) = end {
                self.finish(&n, o);
            }
        } else if self.st.status == STATUS_CRASHED {
            self.finish(&n, Outcome::Crashed);
        } else if elapsed > 2400.0 {
            self.finish(&n, Outcome::Timeout);
        }
    }

    fn finish(&mut self, n: &Nav, o: Outcome) {
        let pad = Pad::ksc();
        let (e, nn) = pad.ground_offset(n.x);
        self.stats.final_e = e;
        self.stats.final_n = nn;
        self.stats.final_tilt_deg = tilt_deg(n);
        let spec = *self.spec();
        let spec = &spec;
        let (p, cap) = propellant(spec, &self.st.res);
        self.stats.prop_left = p;
        self.stats.prop_cap = cap;
        let ve = propulsion(spec, p_atm(n.alt)).isp * ENGINE_G0;
        let m = n.m;
        // usable propellant: whichever of fuel / oxidiser runs out first
        let usable = usable_propellant(spec, &self.st.res).min(p);
        self.stats.dv_left = ve * (m / (m - usable).max(1e-9)).ln();
        if self.prop_at_ign > 0.0 {
            let m_ign = m + (self.prop_at_ign - p);
            self.stats.landing_dv = ve * (m_ign / m).ln();
        }
        // landed but nowhere near the pad and short of fuel: classify as no-return
        let miss = (e * e + nn * nn).sqrt();
        self.outcome = if o == Outcome::Landed && miss > 2000.0 { Outcome::NoReturn } else { o };
        self.record(n, true);
        self.phase = Phase::Done;
    }

    /// G-FOLD tracking command (pad ENU thrust acceleration).
    fn gfold_accel(&mut self, n: &Nav) -> V3 {
        let (r, v, _) = self.landing_state(n);
        let t_now = self.st.ut;
        // hand over to the baseline's terminal logic early enough for this craft's attitude lag
        let handover = gfold::TERMINAL_TGO.max(3.0 * self.swing_time(n, 0.5));
        let need_plan = match &self.gplan {
            None => true,
            Some(p) => t_now - p.t0 >= gfold::REPLAN_DT && p.t_go(t_now) > handover,
        };
        let hh = self.hold_height();
        if need_plan && r.z > hh + 3.0 {
            let prob = self.gfold_problem(n, r, v);
            let warm = self.gplan.as_ref().map(|p| (p.tf - (t_now - p.t0)).max(1.0));
            match gfold::plan(&prob, warm) {
                Some(mut p) => {
                    if debug() {
                        let n_ = p.r.len() - 1;
                        eprintln!("  GFOLD t {:.1} r ({:.1} {:.1} {:.1}) v ({:.1} {:.1} {:.1}) -> tf {:.2} pinpoint {} rN ({:.1} {:.1} {:.1}) mf {:.2}", t_now - self.disp.ut, r.x, r.y, r.z, v.x, v.y, v.z, p.tf, p.pinpoint, p.r[n_].x, p.r[n_].y, p.r[n_].z, p.z[n_].exp());
                    }
                    p.t0 = t_now;
                    self.gplan = Some(p);
                    self.stats.gfold_solves += 1;
                }
                None => {
                    self.stats.gfold_fail += 1;
                }
            }
        }
        match &self.gplan {
            Some(p) if p.t_go(t_now) > handover && r.z > hh => {
                let (rp, vp, up, dp) = p.at(t_now);
                // requested non-gravitational acceleration: planned thrust + planned drag, plus
                // the gravity / Coriolis the plan assumed vs now, plus PD on the plan
                let (g, cor, _) = self.natural_parts(n);
                let plan_grav = p.g - p.omega_enu.cross(vp) * 2.0;
                let kp = 0.25;
                let kd = 1.0;
                up + dp + (plan_grav - (g + cor)) + (rp - r) * kp + (vp - v) * kd
            }
            _ => self.baseline_accel(n),
        }
    }

    fn gfold_problem(&self, n: &Nav, r: V3, v: V3) -> gfold::Problem<'_> {
        let spec = self.spec();
        let pr = propulsion(spec, p_atm(n.alt));
        let pr_pad = propulsion(spec, p_atm(ksc::KSC_PAD_DECK_ALT));
        let thrust = pr.thrust.min(pr_pad.thrust);
        let isp = pr.isp.min(pr_pad.isp);
        let (g, _, _) = self.natural_parts(n);
        // same deceleration cap as the baseline (MAX_LANDING_G above hover)
        let t_cap = (1.0 + MAX_LANDING_G) * 9.81 * n.m;
        gfold::Problem {
            r0: r,
            v0: v,
            m0: n.m,
            m_dry: self.sh.m_dry,
            h_end: self.hold_height(),
            t_vert: 0.0,
            t_max: (thrust * gfold::THRUST_MARGIN).min(t_cap),
            t_min: thrust * pr.throttle_min,
            alpha: 1.0 / (isp * ENGINE_G0),
            g,
            v_td: self.cfg.v_td,
            tables: &self.sh.tables,
            toff: self.st.toff as f64,
            omega_enu: Pad::ksc().vec_to_enu(OMEGA),
        }
    }
}

/// Run a flight to completion (single thread).
pub fn run(f: &mut Flight) {
    while !f.done() {
        f.tick();
    }
}
