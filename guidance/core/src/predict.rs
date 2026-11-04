//! Impact-point predictor: a point-mass trajectory in Kerbin's rotating frame with gravity,
//! Coriolis and centrifugal terms, the sim's atmosphere (pressure / temperature curves + the
//! state's sun offset), the craft's own drag-cube tables (drag x pseudo-Reynolds, body lift vs
//! angle of attack) and, optionally, a modelled landing burn (gravity turn, suicide-burn throttle
//! rule) so the predicted point is where the booster would actually *touch down*.
//!
//! Integrator: RK4 with an altitude/speed-adaptive step. ksp-core integrates with PhysX-style
//! semi-implicit Euler at 0.02 s, whose trajectory equals the exact one started from
//! v0 + a0 * dt / 2; the predictor applies that half-step offset so it tracks the sim, not the ODE.

use crate::nav::{atm_at, p_atm, propulsion, AeroTables, Pad, OMEGA};
use ksp_core::body::KERBIN;
use ksp_core::engine::ENGINE_G0;
use ksp_core::ksc;
use ksp_core::math::V3;
use ksp_core::sim::FIXED_DT;
use ksp_core::vessel::VesselSpec;

/// Craft data the predictor needs (all read from the spec / state, nothing tuned per rocket).
#[derive(Clone, Copy)]
pub struct PredCraft<'a> {
    pub spec: &'a VesselSpec,
    pub tables: &'a AeroTables,
    /// dry mass (t): the burn model stops when propellant runs out
    pub m_dry: f64,
    /// CoM height above the ground at touchdown (m)
    pub h_td: f64,
    /// GetAtmoThermalStats temperature offset (K) cached in the sim state
    pub toff: f64,
    /// vessel root +X axis (body-fixed) at the prediction start: roll is assumed held, which
    /// fixes the body azimuth of any tilt (ZERO = use roll-averaged tables)
    pub body_x: V3,
}

#[derive(Clone, Copy, Debug)]
pub struct LandingBurnModel {
    /// ignite when the throttle the burn needs reaches min(f_ign, max_g * g * m / T_pad)
    pub f_ign: f64,
    pub max_g: f64,
    /// thrust at the pad (kN), for the deceleration cap
    pub t_pad: f64,
    /// target descent rate at touchdown (m/s)
    pub v_td: f64,
}

impl LandingBurnModel {
    pub fn threshold(&self, m: f64) -> f64 {
        if self.t_pad <= 0.0 { self.f_ign } else { self.f_ign.min(self.max_g * 9.81 * m / self.t_pad) }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PredOpts {
    /// angle-of-attack command as a horizontal ENU tilt vector (east, north): the thrust axis
    /// (nominally -v_hat, engine-first) is tilted toward this direction. With `aoa_torque` > 0
    /// the vector is a fraction (|.| <= 1) of the largest angle the craft can hold at each point
    /// of the path, else degrees. Unpowered flight only (the landing burn flies at zero AoA).
    pub aoa: (f64, f64),
    pub burn: Option<LandingBurnModel>,
    /// constant extra thrust burn for the first `burn_time` s (kN vector, body-fixed), boostback
    pub thrust: V3,
    pub burn_time: f64,
    pub max_time: f64,
    /// apply the semi-implicit-Euler half-step velocity offset
    pub sim_offset: bool,
    /// stop when descending through this altitude (m ASL) instead of at the ground
    pub stop_alt: Option<f64>,
    /// aero torque (kN m) the craft can hold against: the commanded angle of attack is clipped
    /// along the path to where |moment| <= this (0 = no limit)
    pub aoa_torque: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PredResult {
    /// body-fixed CoM position at touchdown height (or where the run stopped)
    pub x: V3,
    pub v: V3,
    pub t: f64,
    pub m: f64,
    /// height above the touchdown CoM height and altitude where the landing burn ignited
    pub ign_h: f64,
    pub ign_t: f64,
    pub ignited: bool,
    /// true if the trajectory reached the ground (vs timed out)
    pub ok: bool,
    /// true if the modelled burn ran out of propellant
    pub dry: bool,
    /// max dynamic pressure along the path (kPa) and highest altitude reached (m)
    pub max_q: f64,
    pub apo: f64,
}

#[derive(Clone, Copy)]
struct S {
    x: V3,
    v: V3,
    m: f64,
}

fn ground_r() -> f64 {
    KERBIN.radius + ksc::KSC_PAD_DECK_ALT
}

/// Acceleration (m/s^2) and mass rate for point-mass state `s` with landing-burn throttle `u`
/// along `bdir`, or extra thrust `f_ext` (kN).
#[inline]
fn deriv(c: &PredCraft, o: &PredOpts, pad: &Pad, s: &S, u: f64, bdir: V3, f_ext: V3, mdot_ext: f64) -> (V3, f64, f64) {
    let r2 = s.x.norm2();
    let r = r2.sqrt();
    let alt = r - KERBIN.radius;
    let mut a = s.x * (-KERBIN.gm() / (r2 * r)) - OMEGA.cross(s.v) * 2.0 - OMEGA.cross(OMEGA.cross(s.x));
    let spd = s.v.norm();
    let mut q_kpa = 0.0;
    let mut dm = 0.0;
    if alt < KERBIN.atmosphere_depth && spd > 1e-6 {
        let atm = atm_at(alt, spd, c.toff);
        if atm.rho > 0.0 {
            q_kpa = atm.q_kpa;
            let vh = s.v / spd;
            let (ae, an) = o.aoa;
            let lam = (ae * ae + an * an).sqrt();
            let mut amag = if u > 0.0 { 0.0 } else { lam };
            let (tp, phi) = if lam > 1e-12 {
                let t = pad.e * (ae / lam) + pad.n * (an / lam);
                let tp = (t - vh * t.dot(vh)).normalized();
                let phi = if c.body_x.norm2() > 0.0 { Some(crate::nav::tilt_azimuth(c.body_x, -vh, tp)) } else { None };
                (tp, phi)
            } else {
                (V3::ZERO, None)
            };
            if amag > 0.0 && o.aoa_torque > 0.0 {
                amag = lam.min(1.0) * c.tables.alpha_limit(atm.mach, q_kpa, o.aoa_torque, phi);
            }
            let cd = c.tables.cd_at(atm.mach, amag, phi);
            let mut f = vh * (-(q_kpa * cd * atm.pseudo_re_drag_mult));
            if amag > 1e-9 {
                f += tp * (q_kpa * c.tables.cl_at(atm.mach, amag, phi));
                if phi.is_some() {
                    f += (-vh).cross(tp) * (q_kpa * c.tables.cs_at(atm.mach, amag, phi));
                }
            }
            a += f / s.m;
        }
    }
    if u > 0.0 {
        let pr = propulsion(c.spec, p_atm(alt));
        a += bdir * (pr.thrust * u / s.m);
        dm -= pr.mdot * u;
    }
    if f_ext.norm2() > 0.0 {
        a += f_ext / s.m;
        dm -= mdot_ext;
    }
    (a, dm, q_kpa)
}

/// Height above the touchdown CoM height.
#[inline]
fn height(c: &PredCraft, x: V3) -> f64 {
    x.norm() - ground_r() - c.h_td
}

/// The landing-burn requirement at (x, v, m, h): the constant throttle of a retrograde burn that
/// brings the vessel from here to `v_td` at the touchdown height, from an energy balance along
/// the path: thrust work = kinetic + potential energy - drag work. The drag work is integrated
/// (8-point midpoint rule) through the real atmosphere below, with v^2 falling linearly along the
/// path, so both the fading of drag as the vessel slows and its rise as the air thickens count.
/// Thrust is taken at mid-burn pressure. Returns (throttle as a fraction of max thrust, thrust
/// direction = -v_hat). Cheap enough to run every predictor step; within a few percent of the
/// full constant-throttle stop solution (`landing::stop_throttle`).
pub fn landing_need(spec: &VesselSpec, tables: &AeroTables, toff: f64, x: V3, v: V3, m: f64, h: f64, v_td: f64) -> (f64, V3) {
    let r = x.norm();
    let up = x / r;
    let vz = v.dot(up);
    let spd = v.norm();
    if vz >= -v_td || spd < 1e-6 {
        return (0.0, up);
    }
    let cos_t = (-vz / spd).max(0.05);
    let hh = h.max(0.1);
    let path = hh / cos_t;
    let alt0 = r - KERBIN.radius;
    let g = KERBIN.gm() / (r * r);
    let v0s = spd * spd;
    let vfs = v_td * v_td / (cos_t * cos_t);
    let mut w_drag = 0.0;
    const N: usize = 8;
    for k in 0..N {
        let f = (k as f64 + 0.5) / N as f64;
        let vs = (v0s - (v0s - vfs) * f).max(0.0);
        let a = alt0 - hh * f;
        let sp = vs.sqrt();
        let atm = atm_at(a, sp, toff);
        if atm.rho > 0.0 {
            w_drag += atm.q_kpa * tables.cd(atm.mach, 0.0) * atm.pseudo_re_drag_mult * (path / N as f64);
        }
    }
    let e_need = 0.5 * m * (v0s - vfs) + m * g * hh - w_drag;
    let pr = propulsion(spec, p_atm(alt0 - 0.5 * hh));
    if pr.thrust <= 0.0 {
        return (f64::INFINITY, -v / spd);
    }
    ((e_need / (pr.thrust * path)).max(0.0), -v / spd)
}

/// Predict the trajectory from body-fixed CoM state (x, v) and mass m.
pub fn predict(c: &PredCraft, o: &PredOpts, x: V3, v: V3, m: f64) -> PredResult {
    let pad = Pad::ksc();
    let mut s = S { x, v, m };
    let mdot_ext = if o.burn_time > 0.0 && o.thrust.norm2() > 0.0 {
        let pr = propulsion(c.spec, p_atm(x.norm() - KERBIN.radius));
        pr.mdot * (o.thrust.norm() / pr.thrust.max(1e-9))
    } else {
        0.0
    };
    if o.sim_offset {
        let fe = if o.burn_time > 0.0 { o.thrust } else { V3::ZERO };
        let (a0, _, _) = deriv(c, o, &pad, &s, 0.0, V3::ZERO, fe, 0.0);
        s.v += a0 * (0.5 * FIXED_DT);
    }
    let mut res = PredResult::default();
    let mut t = 0.0;
    let mut ignited = false;
    let max_t = if o.max_time > 0.0 { o.max_time } else { 3000.0 };
    let mut h = height(c, s.x);
    while t < max_t {
        let r = s.x.norm();
        let alt = r - KERBIN.radius;
        let spd = s.v.norm();
        let vz = s.v.dot(s.x / r);
        // step size: coarse in vacuum, finer low and fast, finest in the burn
        let mut dt = if alt > KERBIN.atmosphere_depth { 2.0 } else { (0.02 * h / spd.max(1.0)).clamp(0.05, 0.5) };
        if o.burn_time > 0.0 && t < o.burn_time {
            dt = dt.min(o.burn_time - t).max(1e-3);
        }
        if ignited {
            dt = dt.min(0.1);
        }
        // landing-burn throttle and direction (held over the step)
        let mut u = 0.0;
        let mut bdir = V3::ZERO;
        if let Some(b) = o.burn {
            if vz < 0.0 || ignited {
                let (need, dir) = landing_need(c.spec, c.tables, c.toff, s.x, s.v, s.m, h, b.v_td);
                bdir = dir;
                if !ignited && need >= b.threshold(s.m) && alt < KERBIN.atmosphere_depth {
                    ignited = true;
                    res.ign_h = h;
                    res.ign_t = t;
                    dt = dt.min(0.1);
                }
                if ignited {
                    u = need.clamp(0.0, 1.0);
                    if s.m <= c.m_dry {
                        u = 0.0;
                        res.dry = true;
                    }
                }
            }
        }
        let fe = if o.burn_time > 0.0 && t < o.burn_time { o.thrust } else { V3::ZERO };
        let md = if fe.norm2() > 0.0 { mdot_ext } else { 0.0 };
        // RK4
        let f = |st: &S| deriv(c, o, &pad, st, u, bdir, fe, md);
        let (a1, m1, q1) = f(&s);
        let s2 = S { x: s.x + s.v * (0.5 * dt), v: s.v + a1 * (0.5 * dt), m: s.m + m1 * 0.5 * dt };
        let (a2, m2, _) = f(&s2);
        let s3 = S { x: s.x + s2.v * (0.5 * dt), v: s.v + a2 * (0.5 * dt), m: s.m + m2 * 0.5 * dt };
        let (a3, m3, _) = f(&s3);
        let s4 = S { x: s.x + s3.v * dt, v: s.v + a3 * dt, m: s.m + m3 * dt };
        let (a4, m4, _) = f(&s4);
        let nx = s.x + (s.v + s2.v * 2.0 + s3.v * 2.0 + s4.v) * (dt / 6.0);
        let nv = s.v + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (dt / 6.0);
        let nm = (s.m + (m1 + 2.0 * m2 + 2.0 * m3 + m4) * (dt / 6.0)).max(c.m_dry.min(s.m));
        res.max_q = res.max_q.max(q1);
        res.apo = res.apo.max(alt);
        let nh = height(c, nx);
        if let Some(sa) = o.stop_alt {
            if nx.norm() - KERBIN.radius < sa && nv.dot(nx) < 0.0 {
                res.x = nx;
                res.v = nv;
                res.t = t + dt;
                res.m = nm;
                res.ok = true;
                return res;
            }
        }
        // burn finished: stopped above the ground -> assume a vertical drop at v_td from there
        if let Some(b) = o.burn {
            if ignited && nv.dot(nx.normalized()) >= -b.v_td && nh > 0.0 {
                res.x = nx - nx.normalized() * nh;
                res.v = nv;
                res.t = t + dt + nh / b.v_td.max(0.1);
                res.m = nm;
                res.ok = true;
                res.ignited = true;
                return res;
            }
        }
        if nh <= 0.0 {
            // interpolate the crossing
            let f = (h / (h - nh)).clamp(0.0, 1.0);
            res.x = s.x + (nx - s.x) * f;
            res.v = s.v + (nv - s.v) * f;
            res.t = t + dt * f;
            res.m = s.m + (nm - s.m) * f;
            res.ok = true;
            res.ignited = ignited;
            return res;
        }
        s = S { x: nx, v: nv, m: nm };
        h = nh;
        t += dt;
    }
    res.x = s.x;
    res.v = s.v;
    res.t = t;
    res.m = s.m;
    res.ignited = ignited;
    res
}

/// Burn time (s) for a velocity change dv at full throttle from mass m (rocket equation).
pub fn burn_time(thrust: f64, isp: f64, m: f64, dv: f64) -> f64 {
    let ve = isp * ENGINE_G0;
    let mdot = thrust / ve;
    m * (1.0 - (-dv / ve).exp()) / mdot
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::{nav, AeroTables};
    use ksp_core::math::v3;
    use ksp_core::sim::{self, Control, Params};
    use ksp_core::vessel::Craft;

    /// Unpowered fall: the predictor's touchdown point matches the full 6-DOF sim (attitude held
    /// engine-first by a rate-free reset each tick so the comparison isolates translation).
    #[test]
    fn ballistic_matches_sim() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let mut res = spec.full_resources();
        res[0] *= 0.1;
        res[1] *= 0.1;
        let tables = AeroTables::build(&spec, &res);
        let pad = Pad::ksc();
        let prm = Params { ground: false, sun_temperature: false, ..Params::default() };
        let v0 = v3(120.0, -40.0, 60.0);
        let q = sim::attitude(-(pad.vec_from_enu(v0)).normalized(), pad.n);
        let mut st = sim::state_at_pad(&spec, v3(-3000.0, 1000.0, 12000.0), v0, q, V3::ZERO, 0.0);
        st.res = res;
        let n0 = nav(&spec, &st);
        let h_td = crate::nav::touchdown_height(&spec, n0.com_l);
        let c = PredCraft { spec: &spec, tables: &tables, m_dry: spec.dry_mass(), h_td, toff: 0.0, body_x: n0.q.rotate(v3(1.0, 0.0, 0.0)) };
        let p = predict(&c, &PredOpts { sim_offset: true, ..Default::default() }, n0.x, n0.v, n0.m);
        assert!(p.ok);
        // fly the sim, re-pointing engine-first along the airflow every tick
        let mut t = 0.0;
        loop {
            let n = nav(&spec, &st);
            if n.x.norm() - (KERBIN.radius + ksc::KSC_PAD_DECK_ALT) - h_td <= 0.0 {
                break;
            }
            st.q = sim::attitude(-n.v.normalized(), pad.n);
            st.w = V3::ZERO;
            let r = st.q.rotate(n.com_l);
            st.x = n.x - r;
            st.v = n.v;
            sim::tick(&spec, &prm, &mut st, &Control::default());
            t += sim::FIXED_DT;
        }
        let n = nav(&spec, &st);
        let (e1, n1) = pad.ground_offset(n.x);
        let (e2, n2) = pad.ground_offset(p.x);
        let err = ((e1 - e2).powi(2) + (n1 - n2).powi(2)).sqrt();
        println!("sim t {t:.2} pred t {:.2}; err {err:.2} m ({e1:.1},{n1:.1}) vs ({e2:.1},{n2:.1})", p.t);
        assert!(err < 5.0, "{err}");
        assert!((p.t - t).abs() < 0.2);
    }
}

#[cfg(test)]
mod aoa_tests {
    use super::*;
    use crate::nav::{atm_at, nav, AeroTables};
    use ksp_core::math::v3;
    use ksp_core::sim::{self, Control, Params};
    use ksp_core::vessel::Craft;

    /// Same as `ballistic_matches_sim` but flying a constant tilt fraction of the holdable angle
    /// of attack (attitude imposed every tick), to check the lift / AoA-drag model.
    #[test]
    fn aoa_steering_matches_sim() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let mut res = spec.full_resources();
        res[0] *= 0.1;
        res[1] *= 0.1;
        let tables = AeroTables::build(&spec, &res);
        let pad = Pad::ksc();
        let prm = Params { ground: false, sun_temperature: false, ..Params::default() };
        let v0 = v3(150.0, -40.0, -350.0);
        let q = sim::attitude(-(pad.vec_from_enu(v0)).normalized(), pad.n);
        let mut st = sim::state_at_pad(&spec, v3(-3000.0, 1000.0, 15000.0), v0, q, V3::ZERO, 0.0);
        st.res = res;
        let n0 = nav(&spec, &st);
        let h_td = crate::nav::touchdown_height(&spec, n0.com_l);
        let torque = 2.5;
        // tilt command (east, north) fraction; override with KSPG_TEST_LAM="e,n"
        let lam = std::env::var("KSPG_TEST_LAM").ok().and_then(|s| {
            let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (v.len() == 2).then(|| (v[0], v[1]))
        }).unwrap_or((0.8, 0.3));
        let c = PredCraft { spec: &spec, tables: &tables, m_dry: spec.dry_mass(), h_td, toff: 0.0, body_x: n0.q.rotate(v3(1.0, 0.0, 0.0)) };
        let p = predict(&c, &PredOpts { sim_offset: true, aoa: lam, aoa_torque: torque, ..Default::default() }, n0.x, n0.v, n0.m);
        let p0 = predict(&c, &PredOpts { sim_offset: true, ..Default::default() }, n0.x, n0.v, n0.m);
        loop {
            let n = nav(&spec, &st);
            if n.x.norm() - (KERBIN.radius + ksc::KSC_PAD_DECK_ALT) - h_td <= 0.0 {
                break;
            }
            let vh = n.v.normalized();
            let atm = atm_at(n.alt, n.v.norm(), 0.0);
            let lm = (lam.0 * lam.0 + lam.1 * lam.1).sqrt();
            let t = (pad.e * lam.0 + pad.n * lam.1) / lm;
            let tp = (t - vh * t.dot(vh)).normalized();
            let phi = crate::nav::tilt_azimuth(st.q.rotate(v3(1.0, 0.0, 0.0)), -vh, tp);
            let a = lm.min(1.0) * tables.alpha_limit(atm.mach, atm.q_kpa, torque, Some(phi));
            let dir = -vh * a.to_radians().cos() + tp * a.to_radians().sin();
            st.q = sim::attitude(dir, pad.n);
            st.w = V3::ZERO;
            st.x = n.x - st.q.rotate(n.com_l);
            st.v = n.v;
            sim::tick(&spec, &prm, &mut st, &Control::default());
        }
        let n = nav(&spec, &st);
        let (e1, n1) = pad.ground_offset(n.x);
        let (e2, n2) = pad.ground_offset(p.x);
        let (e3, n3) = pad.ground_offset(p0.x);
        let err = ((e1 - e2).powi(2) + (n1 - n2).powi(2)).sqrt();
        let moved = ((e2 - e3).powi(2) + (n2 - n3).powi(2)).sqrt();
        println!("aoa: sim ({e1:.1},{n1:.1}) pred ({e2:.1},{n2:.1}) err {err:.2} m; steering moved it ({:.1},{:.1})", e2 - e3, n2 - n3);
        // a few km of lift-driven displacement predicted to within a few percent
        assert!(err < 0.05 * moved, "{err} of {moved}");
    }
}
