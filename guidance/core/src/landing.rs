//! Landing-burn sizing for the baseline (suicide-burn) law.
//!
//! `stop_throttle` answers: what *constant* vertical throttle fraction, held from now, brings
//! the descent rate to `v_td` exactly at the touchdown height? It forward-simulates the vertical
//! channel with the craft's real thrust (Isp at the local pressure), mass flow, gravity and the
//! vertical part of its drag (drag-cube tables), and bisects on the throttle. The baseline law
//! ignites when that throttle reaches `f_ign` and then re-solves it every tick (closed loop), so
//! fading drag, falling mass and changing Isp are all accounted for without tuned gains. The same
//! simulation gives the time to go used by the horizontal ZEM/ZEV divert.

use crate::nav::{atm_at, p_atm, propulsion, AeroTables};
use ksp_core::body::KERBIN;
use ksp_core::ksc;
use ksp_core::vessel::VesselSpec;

#[derive(Clone, Copy, Debug)]
pub struct Stop {
    /// constant vertical-thrust fraction of max thrust (may exceed 1 = cannot stop in time)
    pub u: f64,
    /// burn time at that throttle (s)
    pub t_go: f64,
}

/// Height above touchdown reached when the vertical speed has been brought to -v_td at constant
/// vertical throttle u (negative: hits the ground first, as -remaining speed^2 / (2 g)), and time.
#[allow(clippy::too_many_arguments)]
fn sim_stop(spec: &VesselSpec, tables: &AeroTables, toff: f64, h_td: f64, h0: f64, vz0: f64, vh0: f64, m0: f64, m_dry: f64, v_td: f64, u: f64) -> (f64, f64) {
    let (mut h, mut vz, mut m) = (h0, vz0, m0);
    let ratio = if vz0 < -1e-6 { vh0 / -vz0 } else { 0.0 };
    let t_est = 2.0 * h0.max(1.0) / (-vz0 + v_td).max(1.0);
    let dt = (t_est / 80.0).clamp(0.005, 0.1);
    let mut t = 0.0;
    let g_r = KERBIN.radius + ksc::KSC_PAD_DECK_ALT + h_td;
    while t < 600.0 {
        let alt = ksc::KSC_PAD_DECK_ALT + h_td + h;
        let r = g_r + h;
        let g = KERBIN.gm() / (r * r);
        let pr = propulsion(spec, p_atm(alt));
        let vh = ratio * -vz;
        let spd = (vz * vz + vh * vh).sqrt();
        let atm = atm_at(alt, spd, toff);
        let drag_z = if atm.rho > 0.0 && spd > 1e-6 { atm.q_kpa * tables.cd(atm.mach, 0.0) * atm.pseudo_re_drag_mult * (-vz / spd) } else { 0.0 };
        let thrust = if m > m_dry { pr.thrust * u } else { 0.0 };
        let az = (thrust + drag_z) / m - g;
        let vn = vz + az * dt;
        let hn = h + vn * dt;
        m -= pr.mdot * u * dt;
        if vn >= -v_td {
            // interpolate to the moment vz = -v_td
            let f = ((-v_td - vz) / (vn - vz)).clamp(0.0, 1.0);
            return (h + (hn - h) * f, t + dt * f);
        }
        if hn <= 0.0 {
            // ground first: report the overshoot as an equivalent negative height
            let f = (h / (h - hn)).clamp(0.0, 1.0);
            let vg = vz + (vn - vz) * f;
            return (-(vg * vg - v_td * v_td) / (2.0 * g), t + dt * f);
        }
        h = hn;
        vz = vn;
        t += dt;
    }
    (h, t)
}

/// Constant vertical throttle that stops at the touchdown height (bisection, 14 steps).
#[allow(clippy::too_many_arguments)]
pub fn stop_throttle(spec: &VesselSpec, tables: &AeroTables, toff: f64, h_td: f64, h: f64, vz: f64, vh: f64, m: f64, m_dry: f64, v_td: f64) -> Stop {
    if vz >= -v_td || h <= 0.0 {
        return Stop { u: 0.0, t_go: 0.0 };
    }
    let f = |u: f64| sim_stop(spec, tables, toff, h_td, h, vz, vh, m, m_dry, v_td, u);
    let (h1, t1) = f(1.0);
    if h1 < 0.0 {
        // cannot stop even at full throttle: report the deficit as u > 1
        let (h_hi, _) = f(1.5);
        let u = if h_hi > h1 { 1.0 + 0.5 * (-h1 / (h_hi - h1)) } else { 2.0 };
        return Stop { u, t_go: t1 };
    }
    let (mut lo, mut hi) = (0.0, 1.0);
    let mut t_go = t1;
    for _ in 0..14 {
        let mid = 0.5 * (lo + hi);
        let (hm, tm) = f(mid);
        if hm < 0.0 {
            lo = mid;
        } else {
            hi = mid;
            t_go = tm;
        }
    }
    Stop { u: hi, t_go }
}

/// Thrust allocation that knows about body lift: find the throttle `u` and the tilt `alpha` of the
/// thrust axis away from the airflow axis w_hat = -v_hat (toward t_hat, signed) such that
/// thrust + aero (drag along w_hat, lift along t_hat from the craft tables) gives the requested
/// non-gravitational acceleration with components `a_w` (along w_hat) and `a_p` (along t_hat,
/// >= 0). At high dynamic pressure lift can exceed the thrust's side component and reverse the
/// sense of a tilt; the search picks the smallest |alpha| that delivers `a_p` with u in [0, 1].
/// Returns (alpha_rad, u, achieved a_p).
#[allow(clippy::too_many_arguments)]
pub fn allocate(tables: &AeroTables, mach: f64, q_kpa: f64, pre: f64, thrust: f64, m: f64, a_w: f64, a_p: f64, alpha_max_deg: f64, phi: Option<f64>) -> (f64, f64, f64) {
    if thrust <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    let eval = |a_deg: f64| -> (f64, f64) {
        let aa = a_deg.abs();
        // a negative alpha tilts away from t_hat: azimuth + 180
        let ph = phi.map(|p| if a_deg < 0.0 { p + 180.0 } else { p });
        let d = q_kpa * tables.cd_at(mach, aa, ph) * pre;
        let l = q_kpa * tables.cl_at(mach, aa, ph) * a_deg.signum();
        let ar = a_deg.to_radians();
        let u = ((m * a_w - d) / (thrust * ar.cos())).clamp(0.0, 1.0);
        (u, (thrust * u * ar.sin() + l) / m)
    };
    let (u0, p0) = eval(0.0);
    if (p0 - a_p).abs() < 1e-9 {
        return (0.0, u0, p0);
    }
    let step = 0.25;
    let n = (alpha_max_deg / step) as i32;
    let mut best = (0.0, u0, p0);
    let mut best_err = (p0 - a_p).abs();
    // scan outward in both senses; first crossing (smallest |alpha|) wins
    let mut prev = [(0.0, u0, p0); 2];
    for k in 1..=n {
        for (si, sgn) in [1.0, -1.0].iter().enumerate() {
            let a = sgn * k as f64 * step;
            let (u, p) = eval(a);
            let (pa, _, pp) = prev[si];
            if (pp - a_p) * (p - a_p) <= 0.0 && (p - pp).abs() > 1e-12 {
                let f = (a_p - pp) / (p - pp);
                let ai = pa + (a - pa) * f;
                let (ui, pi) = eval(ai);
                return (ai.to_radians(), ui, pi);
            }
            let e = (p - a_p).abs();
            if e < best_err {
                best_err = e;
                best = (a, u, p);
            }
            prev[si] = (a, u, p);
        }
    }
    (best.0.to_radians(), best.1, best.2)
}
