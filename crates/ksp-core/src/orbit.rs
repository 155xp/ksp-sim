//! Two-body (patched-conic) Kepler orbits, as used by KSP for on-rails propagation and for the
//! vessel's orbit readout.
//!
//! Source cross-reference: `Orbit.UpdateFromStateVectors`, `Orbit.getRelativePositionAtUT`,
//! `Orbit.solveEccentricAnomaly` (decomp Orbit.cs). KSP's Orbit works in the "Zup" planetarium
//! frame, which is the sim's inertial frame here (body-centred, non-rotating, +z = north).
//! Elements use KSP's conventions: inclination/LAN/argPe in degrees, mean anomaly in rad,
//! LAN measured from the inertial +x axis.

use crate::math::{v3, V3};
use core::f64::consts::{PI, TAU};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Elements {
    pub sma: f64,
    pub ecc: f64,
    pub inc_deg: f64,
    pub lan_deg: f64,
    pub arg_pe_deg: f64,
    /// mean anomaly at `epoch` (rad)
    pub mean_anomaly_at_epoch: f64,
    pub epoch: f64,
}

/// State vector (r, v) -> elements at time `ut`. Handles elliptic and hyperbolic orbits.
pub fn elements_from_state(r: V3, v: V3, mu: f64, ut: f64) -> Elements {
    let h = r.cross(v);
    let rn = r.norm();
    let e_vec = v.cross(h) / mu - r / rn;
    let ecc = e_vec.norm();
    let energy = 0.5 * v.norm2() - mu / rn;
    let sma = -mu / (2.0 * energy);
    let inc = (h.z / h.norm()).clamp(-1.0, 1.0).acos();
    let n = v3(-h.y, h.x, 0.0); // node vector z x h
    let nn = n.norm();
    let lan = if nn > 1e-12 * h.norm() {
        let l = (n.x / nn).clamp(-1.0, 1.0).acos();
        if n.y < 0.0 { TAU - l } else { l }
    } else {
        0.0
    };
    // equatorial orbits: KSP puts the node direction on +x
    let node_dir = if nn > 1e-12 * h.norm() { n / nn } else { v3(1.0, 0.0, 0.0) };
    let arg_pe = if ecc > 1e-11 {
        let mut w = (node_dir.dot(e_vec) / ecc).clamp(-1.0, 1.0).acos();
        let s = h.normalized().dot(node_dir.cross(e_vec));
        if s < 0.0 {
            w = TAU - w;
        }
        w
    } else {
        0.0
    };
    // true anomaly
    let pe_dir = if ecc > 1e-11 { e_vec / ecc } else { node_dir };
    let q_dir = h.normalized().cross(pe_dir);
    let nu = r.dot(q_dir).atan2(r.dot(pe_dir));
    let m = if ecc < 1.0 {
        let ea = 2.0 * (((1.0 - ecc) / (1.0 + ecc)).sqrt() * (nu / 2.0).tan()).atan();
        (ea - ecc * ea.sin()).rem_euclid(TAU)
    } else {
        let ha = 2.0 * (((ecc - 1.0) / (ecc + 1.0)).sqrt() * (nu / 2.0).tan()).atanh();
        ecc * ha.sinh() - ha
    };
    Elements {
        sma,
        ecc,
        inc_deg: inc.to_degrees(),
        lan_deg: lan.to_degrees(),
        arg_pe_deg: arg_pe.to_degrees(),
        mean_anomaly_at_epoch: m,
        epoch: ut,
    }
}

impl Elements {
    pub fn mean_motion(&self, mu: f64) -> f64 {
        (mu / self.sma.abs().powi(3)).sqrt()
    }
    pub fn period(&self, mu: f64) -> f64 {
        if self.ecc < 1.0 { TAU / self.mean_motion(mu) } else { f64::INFINITY }
    }
    pub fn periapsis(&self) -> f64 {
        self.sma * (1.0 - self.ecc)
    }
    pub fn apoapsis(&self) -> f64 {
        if self.ecc < 1.0 { self.sma * (1.0 + self.ecc) } else { f64::INFINITY }
    }

    /// Kepler propagation: (r, v) at `ut`.
    pub fn state_at(&self, mu: f64, ut: f64) -> (V3, V3) {
        let n = self.mean_motion(mu);
        let m = self.mean_anomaly_at_epoch + n * (ut - self.epoch);
        let e = self.ecc;
        let (px, py, vx, vy);
        if e < 1.0 {
            let ea = solve_kepler_elliptic(m.rem_euclid(TAU), e);
            let (s, c) = ea.sin_cos();
            let b = self.sma * (1.0 - e * e).sqrt();
            px = self.sma * (c - e);
            py = b * s;
            let edot = n / (1.0 - e * c);
            vx = -self.sma * s * edot;
            vy = b * c * edot;
        } else {
            let ha = solve_kepler_hyperbolic(m, e);
            let (sh, ch) = (ha.sinh(), ha.cosh());
            let a = self.sma.abs();
            let b = a * (e * e - 1.0).sqrt();
            px = a * (e - ch);
            py = b * sh;
            let hdot = n / (e * ch - 1.0);
            vx = -a * sh * hdot;
            vy = b * ch * hdot;
        }
        let (r, v) = (v3(px, py, 0.0), v3(vx, vy, 0.0));
        (self.perifocal_to_inertial(r), self.perifocal_to_inertial(v))
    }

    fn perifocal_to_inertial(&self, p: V3) -> V3 {
        let (so, co) = self.lan_deg.to_radians().sin_cos();
        let (si, ci) = self.inc_deg.to_radians().sin_cos();
        let (sw, cw) = self.arg_pe_deg.to_radians().sin_cos();
        let x = (co * cw - so * sw * ci) * p.x + (-co * sw - so * cw * ci) * p.y;
        let y = (so * cw + co * sw * ci) * p.x + (-so * sw + co * cw * ci) * p.y;
        let z = (sw * si) * p.x + (cw * si) * p.y;
        v3(x, y, z)
    }
}

/// Newton iteration on E - e sin E = M (elliptic). Starting guess per Danby.
pub fn solve_kepler_elliptic(m: f64, e: f64) -> f64 {
    let mut ea = if e < 0.8 { m } else { PI };
    for _ in 0..50 {
        let f = ea - e * ea.sin() - m;
        let d = f / (1.0 - e * ea.cos());
        ea -= d;
        if d.abs() < 1e-15 {
            break;
        }
    }
    ea
}

/// Newton iteration on e sinh H - H = M (hyperbolic).
pub fn solve_kepler_hyperbolic(m: f64, e: f64) -> f64 {
    let mut h = (2.0 * m.abs() / e + 1.8).ln() * m.signum();
    for _ in 0..100 {
        let f = e * h.sinh() - h - m;
        let d = f / (e * h.cosh() - 1.0);
        h -= d;
        if d.abs() < 1e-15 * (1.0 + h.abs()) {
            break;
        }
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::KERBIN_GM;

    fn roundtrip(r: V3, v: V3) {
        let el = elements_from_state(r, v, KERBIN_GM, 100.0);
        let (r2, v2) = el.state_at(KERBIN_GM, 100.0);
        assert!((r2 - r).norm() < 1e-6 * r.norm(), "{r:?} {r2:?} {el:?}");
        assert!((v2 - v).norm() < 1e-6 * v.norm(), "{v:?} {v2:?}");
    }

    #[test]
    fn state_roundtrip() {
        roundtrip(v3(700_000.0, 0.0, 0.0), v3(0.0, 2_300.0, 100.0));
        roundtrip(v3(-500_000.0, 400_000.0, 30_000.0), v3(-1500.0, -1700.0, 400.0));
        roundtrip(v3(700_000.0, 10.0, 0.0), v3(0.0, 4_000.0, 0.0)); // hyperbolic
        roundtrip(v3(0.0, 680_000.0, 0.0), v3(-2_279.0, 0.0, 0.0)); // ~circular equatorial
    }

    #[test]
    fn circular_orbit_speed_and_period() {
        // 80 km circular LKO: v = sqrt(mu/r) = 2278.93 m/s (published LKO speed ~2279 m/s)
        let r = 680_000.0;
        let v = (KERBIN_GM / r).sqrt();
        assert!((v - 2278.93).abs() < 0.01, "{v}");
        let el = elements_from_state(v3(r, 0.0, 0.0), v3(0.0, v, 0.0), KERBIN_GM, 0.0);
        assert!(el.ecc < 1e-9);
        assert!((el.period(KERBIN_GM) - TAU * r / v).abs() < 1e-6);
        // half a period later we're on the other side
        let (r2, _) = el.state_at(KERBIN_GM, el.period(KERBIN_GM) / 2.0);
        assert!((r2 - v3(-r, 0.0, 0.0)).norm() < 1e-3);
    }

    #[test]
    fn escape_velocity() {
        // Kerbin surface escape velocity: published 3431.03 m/s
        let ve = (2.0 * KERBIN_GM / 600_000.0f64).sqrt();
        assert!((ve - 3431.03).abs() < 0.01, "{ve}");
    }
}
