//! Craft-agnostic attitude control: point the vessel's thrust axis (root +Y) along a desired
//! direction using whatever torque the craft has *right now*.
//!
//! Every tick the controller rebuilds the control-effectiveness matrix B (root-frame torque per
//! unit pitch / yaw / roll input) from the spec and the current thrust: reaction wheels
//! (`ModuleReactionWheel` torque, only with ElectricCharge) plus each engine's gimbal at full
//! deflection (ksp-core's gimbal rotation about root X / Z, including its CoM-side sign flip).
//! The aerodynamic torque at the current state (replicated cube model) is fed forward.
//! Pointing uses a braking-curve rate command sized from the torque actually available about the
//! error axis, so a 1.25 m hopper and a 3.75 m booster run the same law; only dimensionless shape
//! constants (margins, time constants of the inner rate loop) appear below.

use crate::nav::{aero_force, atm_at, Nav};
use ksp_core::engine::{self, ENGINE_G0};
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::Res;
use ksp_core::sim::{Control, EnvState};
use ksp_core::vessel::VesselSpec;

/// Fraction of the available angular acceleration the braking curve plans with.
pub const BRAKE_FRACTION: f64 = 0.5;
/// Linear-region time constant of the pointing loop (s) and of the inner rate loop (s).
pub const POINT_TAU: f64 = 0.3;
pub const RATE_TAU: f64 = 0.12;
/// Rate limit (rad/s).
pub const MAX_RATE: f64 = 0.8;

#[derive(Clone, Copy, Debug, Default)]
pub struct Effectiveness {
    /// columns: torque (kN m, root frame) for pitch = 1, yaw = 1, roll = 1
    pub b: [V3; 3],
    /// reaction-wheel only part of `b`
    pub b_rw: [V3; 3],
}

fn engine_thrust(spec: &VesselSpec, st: &EnvState, k: usize, throttle: f64, alt_of: impl Fn(V3) -> f64) -> f64 {
    let e = &spec.engines[k];
    let pos_w = st.q.rotate(e.pos);
    let p = ksp_core::atmo::pressure_kpa(ksp_core::body::KERBIN, alt_of(st.x + pos_w)) * ksp_core::atmo::KPA_TO_ATM;
    e.max_fuel_flow * throttle * engine::isp(e, p) * ENGINE_G0
}

/// Control effectiveness at throttle `throttle`.
pub fn effectiveness(spec: &VesselSpec, st: &EnvState, nav: &Nav, throttle: f64) -> Effectiveness {
    let mut out = Effectiveness::default();
    let ec = st.res[Res::ElectricCharge as usize];
    if ec > 1e-6 {
        // ksp-core sim.rs 3b (game-measured): root torque = (-pitch*Tx, +roll*Ty, +yaw*Tz)
        let t = spec.rw_torque;
        out.b_rw = [v3(-t.x, 0.0, 0.0), v3(0.0, 0.0, t.z), v3(0.0, t.y, 0.0)];
    }
    out.b = out.b_rw;
    if throttle > 0.0 {
        let r = ksp_core::body::KERBIN.radius;
        for k in 0..spec.n_engines as usize {
            let e = &spec.engines[k];
            let thrust = engine_thrust(spec, st, k, throttle, |x| x.norm() - r);
            if thrust <= 0.0 || e.gimbal_range_deg <= 0.0 {
                continue;
            }
            // ksp-core sim.rs 3a (game-measured): pitch about root +X by s*range, yaw about root
            // +Z by -s*range, thrust transform swinging about the gimbal pivot
            if e.n_nozzles > 1 {
                // multi-nozzle engine (ksp-core sim.rs 3a): each nozzle gimbals itself, the roll
                // input mixed into the off-axis nozzles' pitch / yaw (ModuleGimbal.GimbalRotation)
                let share = thrust / e.n_nozzles as f64;
                let r = e.gimbal_range_deg;
                let tq = |j: usize, ax: f64, az: f64| -> V3 {
                    let gq = Quat::angle_axis(ax, v3(1.0, 0.0, 0.0)) * Quat::angle_axis(-az, v3(0.0, 0.0, 1.0));
                    (e.nozzles[j] - nav.com_l).cross(gq.rotate(e.dir) * share)
                };
                for j in 0..e.n_nozzles as usize {
                    let s = if nav.com_l.y > e.nozzles[j].y { 1.0 } else { -1.0 };
                    let [rx, rz] = e.nozzle_roll[j];
                    let t0 = tq(j, 0.0, 0.0);
                    out.b[0] += tq(j, s * r, 0.0) - t0;
                    out.b[1] += tq(j, 0.0, s * r) - t0;
                    out.b[2] += tq(j, (rx * r).clamp(-r, r), (rz * r).clamp(-r, r)) - t0;
                }
                continue;
            }
            let sgn = if nav.com_l.y > e.pos.y { 1.0 } else { -1.0 };
            let tq = |gq: Quat| -> V3 { (e.pivot + gq.rotate(e.pos - e.pivot) - nav.com_l).cross(gq.rotate(e.dir) * thrust) };
            let t0 = tq(Quat::IDENTITY);
            out.b[0] += tq(Quat::angle_axis(sgn * e.gimbal_range_deg, v3(1.0, 0.0, 0.0))) - t0;
            out.b[1] += tq(Quat::angle_axis(-sgn * e.gimbal_range_deg, v3(0.0, 0.0, 1.0))) - t0;
        }
    }
    out
}

#[inline]
fn mat_vec(m: &[[f64; 3]; 3], v: V3) -> V3 {
    v3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z)
}

/// Solve the 3x3 system M x = r (Cramer); None if singular.
pub fn solve3(m: [[f64; 3]; 3], r: V3) -> Option<V3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-300 {
        return None;
    }
    let rv = [r.x, r.y, r.z];
    let mut out = [0.0; 3];
    for c in 0..3 {
        let mut mc = m;
        for row in 0..3 {
            mc[row][c] = rv[row];
        }
        let d = mc[0][0] * (mc[1][1] * mc[2][2] - mc[1][2] * mc[2][1]) - mc[0][1] * (mc[1][0] * mc[2][2] - mc[1][2] * mc[2][0]) + mc[0][2] * (mc[1][0] * mc[2][1] - mc[1][1] * mc[2][0]);
        out[c] = d / det;
    }
    Some(v3(out[0], out[1], out[2]))
}

/// Max torque magnitude achievable along unit axis `n` with |u_i| <= 1.
pub fn capacity(b: &[V3; 3], n: V3) -> f64 {
    b.iter().map(|c| c.dot(n).abs()).sum()
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AttitudeOut {
    pub ctl: Control,
    /// pointing error (rad)
    pub err: f64,
    /// angular acceleration available about the error axis (rad/s^2)
    pub alpha_avail: f64,
}

/// Torque-allocating pointing controller. `dir` is the desired thrust-axis direction
/// (body-fixed); `throttle` is passed through (and used for the gimbal authority).
pub fn point(spec: &VesselSpec, st: &EnvState, nav: &Nav, dir: V3, throttle: f64, aero_ff: bool) -> AttitudeOut {
    let eff = effectiveness(spec, st, nav, throttle);
    let i_l = &nav.inertia_l;
    let w = nav.w_l;
    let y = v3(0.0, 1.0, 0.0);
    let d_l = nav.q.conj().rotate(dir.normalized());
    let e = y.cross(d_l);
    let s = e.norm();
    let c = y.dot(d_l);
    let theta = s.atan2(c);
    let n = if s > 1e-9 {
        e / s
    } else if c < 0.0 {
        // anti-aligned: turn about whichever horizontal axis we already rotate about, else X
        let wh = v3(w.x, 0.0, w.z);
        if wh.norm() > 1e-6 { wh.normalized() } else { v3(1.0, 0.0, 0.0) }
    } else {
        V3::ZERO
    };
    // aero torque at the current state (root frame)
    let tau_aero = if aero_ff && nav.alt < ksp_core::body::KERBIN.atmosphere_depth {
        let atm = atm_at(nav.alt, nav.v.norm(), st.toff as f64);
        let (_, tq) = aero_force(spec, nav.com_l, nav.q, nav.v, nav.q.rotate(w), &atm, st.brake_deflection);
        nav.q.conj().rotate(tq)
    } else {
        V3::ZERO
    };
    let mut omega_des = V3::ZERO;
    let mut alpha_avail = 0.0;
    if n.norm2() > 0.0 {
        let i_n = n.dot(mat_vec(i_l, n)).max(1e-9);
        alpha_avail = capacity(&eff.b, n) / i_n;
        let a = (BRAKE_FRACTION * (alpha_avail - tau_aero.dot(n).abs() / i_n)).max(0.05 * alpha_avail).max(1e-6);
        let tb = POINT_TAU;
        let mag = a * tb * ((1.0 + 2.0 * theta / (a * tb * tb)).sqrt() - 1.0);
        omega_des = n * mag.min(MAX_RATE);
    }
    // inner rate loop (roll is just damped)
    let alpha_cmd = (omega_des - w) / RATE_TAU;
    let tau = mat_vec(i_l, alpha_cmd) + w.cross(mat_vec(i_l, w)) - tau_aero;
    // damped least squares allocation: u = B^T (B B^T + lambda I)^-1 tau
    let b = &eff.b;
    let mut bbt = [[0.0; 3]; 3];
    let col = |k: usize| [b[k].x, b[k].y, b[k].z];
    for r in 0..3 {
        for cc in 0..3 {
            bbt[r][cc] = (0..3).map(|k| col(k)[r] * col(k)[cc]).sum();
        }
    }
    let tr = bbt[0][0] + bbt[1][1] + bbt[2][2];
    let lam = 1e-6 * tr.max(1e-12);
    for (k, row) in bbt.iter_mut().enumerate() {
        row[k] += lam;
    }
    let mut u = [0.0; 3];
    if let Some(z) = solve3(bbt, tau) {
        for (k, uk) in u.iter_mut().enumerate() {
            *uk = b[k].dot(z).clamp(-1.0, 1.0);
        }
    }
    AttitudeOut { ctl: Control { throttle, pitch: u[0], yaw: u[1], roll: u[2] }, err: theta, alpha_avail }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::{nav, Pad};
    use ksp_core::sim::{self, Params};
    use ksp_core::vessel::Craft;

    /// The hopper turns 90 degrees in vacuum on its reaction wheel and settles.
    #[test]
    fn hopper_turns_and_settles() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let pad = Pad::ksc();
        let prm = Params { ground: false, aero: false, ..Params::default() };
        let q = sim::attitude(pad.u, pad.n);
        let mut st = sim::state_at_pad(&spec, v3(0.0, 0.0, 50_000.0), V3::ZERO, q, V3::ZERO, 0.0);
        let target = pad.e;
        let mut err = 0.0;
        for _ in 0..1500 {
            let nv = nav(&spec, &st);
            let o = point(&spec, &st, &nv, target, 0.0, false);
            err = o.err;
            sim::tick(&spec, &prm, &mut st, &o.ctl);
        }
        assert!(err.to_degrees() < 0.5, "{}", err.to_degrees());
        assert!(st.w.norm() < 0.01);
    }

    /// Gimbal-only pointing under thrust (no reaction wheel EC).
    #[test]
    fn gimbal_points_under_thrust() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let pad = Pad::ksc();
        let prm = Params { ground: false, aero: false, ..Params::default() };
        let q = sim::attitude(pad.u, pad.n);
        let mut st = sim::state_at_pad(&spec, v3(0.0, 0.0, 50_000.0), V3::ZERO, q, V3::ZERO, 0.0);
        st.res[Res::ElectricCharge as usize] = 0.0;
        let target = (pad.u + pad.e * 0.3).normalized();
        let mut err = 1.0;
        for _ in 0..600 {
            let nv = nav(&spec, &st);
            let o = point(&spec, &st, &nv, target, 0.3, false);
            err = o.err;
            sim::tick(&spec, &prm, &mut st, &o.ctl);
        }
        assert!(err.to_degrees() < 0.5, "{}", err.to_degrees());
    }
}
