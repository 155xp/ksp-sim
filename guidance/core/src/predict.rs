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
}

#[derive(Clone, Copy, Debug)]
pub struct LandingBurnModel {
    /// ignite when the throttle the suicide rule needs reaches this fraction
    pub f_ign: f64,
    /// target descent rate at touchdown (m/s)
    pub v_td: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PredOpts {
    /// constant angle of attack command (deg) as a horizontal ENU tilt vector (east, north):
    /// the vessel's -up axis is tilted from the airflow toward this direction
    pub aoa: (f64, f64),
    pub burn: Option<LandingBurnModel>,
    /// constant extra thrust burn for the first `burn_time` s (kN vector, body-fixed), boostback
    pub thrust: V3,
    pub burn_time: f64,
    pub max_time: f64,
    /// apply the semi-implicit-Euler half-step velocity offset
    pub sim_offset: bool,
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
    /// max dynamic pressure along the path (kPa)
    pub max_q: f64,
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

/// Acceleration (m/s^2) and mass rate for point-mass state `s` with throttle `u` along `-v_hat`
/// (landing burn) or extra thrust `f_ext` (kN).
#[inline]
fn deriv(c: &PredCraft, o: &PredOpts, pad: &Pad, s: &S, u: f64, f_ext: V3, mdot_ext: f64) -> (V3, f64, f64) {
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
            let amag = (ae * ae + an * an).sqrt();
            let cd = c.tables.cd(atm.mach, amag);
            let mut f = vh * (-(q_kpa * cd * atm.pseudo_re_drag_mult));
            if amag > 1e-9 {
                let t = pad.e * (ae / amag) + pad.n * (an / amag);
                let tp = (t - vh * t.dot(vh)).normalized();
                f += tp * (q_kpa * c.tables.cl(atm.mach, amag));
            }
            a += f / s.m;
        }
    }
    if u > 0.0 && spd > 1e-6 {
        let pr = propulsion(c.spec, p_atm(alt));
        a += s.v * (-(pr.thrust * u / (s.m * spd)));
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

/// Throttle the suicide-burn rule needs to stop at h = 0 from (h, v) along -v_hat, accounting for
/// drag's vertical component; also returns cos(theta) of the thrust from vertical.
pub fn suicide_throttle(spec: &VesselSpec, tables: &AeroTables, toff: f64, x: V3, v: V3, m: f64, h: f64, v_td: f64) -> f64 {
    let up = x.normalized();
    let vz = v.dot(up);
    let r = x.norm();
    let g = KERBIN.gm() / (r * r);
    if vz >= -v_td {
        return 0.0;
    }
    let spd = v.norm();
    let alt = r - KERBIN.radius;
    let cos_t = (-vz / spd).max(0.05);
    let atm = atm_at(alt, spd, toff);
    let d_up = if atm.rho > 0.0 { atm.q_kpa * tables.cd(atm.mach, 0.0) * atm.pseudo_re_drag_mult * cos_t } else { 0.0 };
    let a_stop = (vz * vz - v_td * v_td) / (2.0 * h.max(0.1));
    let pr = propulsion(spec, p_atm(alt));
    if pr.thrust <= 0.0 {
        return f64::INFINITY;
    }
    (m * (a_stop + g) - d_up) / (pr.thrust * cos_t)
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
        let (a0, _, _) = deriv(c, o, &pad, &s, 0.0, fe, 0.0);
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
        // landing-burn throttle (held over the step)
        let mut u = 0.0;
        if let Some(b) = o.burn {
            if vz < 0.0 {
                let need = suicide_throttle(c.spec, c.tables, c.toff, s.x, s.v, s.m, h, b.v_td);
                if !ignited && need >= b.f_ign && alt < KERBIN.atmosphere_depth {
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
        let f = |st: &S| deriv(c, o, &pad, st, u, fe, md);
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
        let nh = height(c, nx);
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
        let c = PredCraft { spec: &spec, tables: &tables, m_dry: spec.dry_mass(), h_td, toff: 0.0 };
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
