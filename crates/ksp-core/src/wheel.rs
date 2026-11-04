//! Landing-leg suspension and tire contact, as stock KSP 1.12.5 + Unity do it.
//!
//! Chain (decomp, de-obfuscated reading notes in DESIGN.md):
//! - `SuspensionLoadBalancer.FixedUpdate`: sprungMassGravity S = max(T * round(100/n)/100, 0.5),
//!   T = vessel mass (rounded to 0.1 t*g when landed), n = grounded wheels.
//! - `ModuleWheelSuspension.SuspensionSpringUpdate` (only while grounded): auto-boost,
//!   BoostCurve, one-shot auto springTweakable / damperTweakable, damperFudge,
//!   spring = clamp(stw * springRatio * S * BoostCurve(boost, boostRatio + autoBoost)),
//!   damper = sqrt(clamp(spring * damperRatio * fudge)) * dtw, both Mathf.Round'ed, written to
//!   the VPWheelCollider only past the 3 / 1.05 hysteresis.
//! - `VPWheelCollider.UpdateWheelCollider`: native spring = max(runtimeSpringRate, max(W/d, 0.1)),
//!   damper = max(runtimeDamperRate, 0.1), targetPosition = clamp(W/spring/d, 0.1, 1) slewed by
//!   +-0.02 per tick (springSlerpRate), W = WheelCollider.sprungMass * |g|; *then* runtime = the
//!   module's current rates (so PhysX sees the module's value one tick late).
//! - Unity WheelCollider (PhysX, native): ray from the collider transform along -up of length
//!   d + r, jounce x = d + r - hitDistance, force = k (x - target d) + c dx/dt (may be negative),
//!   reported as WheelHit.force; applied along the ground normal (measured in game: at rest the
//!   reported force equals k (x - target d) * (normal . axis)).
//! - Vehicle Physics Pro tire (VehicleBase.CalculateDownforce / CalculateLocalFrame / Wheel block,
//!   ApplyTireForce) for a braked leg (ModuleWheelLock: brake torque 1000): load =
//!   max(hit.force cos(contactAngle), 10), tangential velocity V at the contact, requested force
//!   Fext = -0.5 (hit.force / g) V / dt (+ slope-hold term), limited by the Smooth friction curve
//!   (adherent (0.01, 0.1), peak (2, 1), limit (7, 1)) times load; lerped at 10*dt; applied at the
//!   contact point.
//!
//! Units: tonnes, kN, metres, seconds. Wheel state is float, like the game.

use crate::math::V3;

/// Per-leg, per-env dynamic state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct LegState {
    /// WheelHit from the last PhysX step
    pub grounded: bool,
    pub x: f32,
    pub hit_force: f32,
    /// native WheelCollider spring / damper / targetPosition
    pub k: f32,
    pub c: f32,
    pub target: f32,
    /// ModuleWheelSuspension private spring / damper and tweakables
    pub m_spring: f32,
    pub m_damper: f32,
    pub stw: f32,
    pub dtw: f32,
    pub auto_boost: f32,
    pub auto_set: bool,
    /// VPWheelCollider.springRate / damperRate and their one-tick-late runtime copies
    pub rate: f32,
    pub drate: f32,
    pub rt: f32,
    pub rtd: f32,
    /// last applied (lerped) tire force (sideways, forward)
    pub tire: [f32; 2],
}

impl LegState {
    pub fn fresh() -> LegState {
        LegState { stw: 1.0, dtw: 1.0, target: f32::NAN, ..Default::default() }
    }
}

/// Static leg data (from the in-game WheelCollider dump / part cfg).
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct LegSpec {
    /// wheel-collider transform position and up axis, root frame
    pub anchor: V3,
    pub up: V3,
    /// unit "forward" of the wheel transform (tire frame), root frame
    pub fwd: V3,
    pub radius: f64,
    /// suspensionDistance (already scaled by rescaleFactor) and the cfg value
    pub travel: f64,
    pub travel_cfg: f64,
    pub spring_ratio: f64,
    pub damper_ratio: f64,
    pub boost_ratio: f64,
    pub maximum_load: f64,
    /// WheelCollider.sprungMass (Unity-computed; = leg rigidbody mass for one wheel per rb)
    pub sprung_mass: f64,
    pub crash_tolerance: f64,
    /// ModuleWheelBase minimumdownForce for legs
    pub min_down_force: f64,
}

impl LegSpec {
    /// Contact point at full extension (anchor - up * (travel + radius)), root frame.
    pub fn foot(&self) -> V3 {
        self.anchor - self.up * (self.travel + self.radius)
    }
}

#[inline(always)]
fn round_half_even(x: f32) -> f32 {
    // Mathf.Round == System.Math.Round(double) to-even on .5
    let r = x.round();
    if (x - x.trunc()).abs() == 0.5 { 2.0 * (x / 2.0).round() } else { r }
}
#[inline(always)]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}
#[inline(always)]
fn inv_lerp(a: f32, b: f32, v: f32) -> f32 {
    if a == b { 0.0 } else { ((v - a) / (b - a)).clamp(0.0, 1.0) }
}
/// Math.Round(x, 2), to-even.
#[inline(always)]
fn round2(x: f32) -> f32 {
    (round_half_even(((x as f64) * 100.0) as f32) as f64 / 100.0) as f32
}

/// `ModuleWheelSuspension.BoostCurve`.
#[inline]
pub fn boost_curve(b: f32, power: f32) -> f32 {
    let b = b.clamp(0.0, 2.0);
    let p = power.clamp(0.01, 1.0);
    let v = (1.0 / (1.0 - 2.0 / b.powf(p)).abs()).clamp(0.01, 1e7);
    round_half_even(v * 10.0) / 10.0
}

/// Vessel-level inputs of the suspension update.
#[derive(Clone, Copy, Debug)]
pub struct SuspInputs {
    pub vessel_mass: f64,
    pub landed: bool,
    pub n_grounded: u32,
    pub gee_asl: f64,
    pub max_surface_gee_asl: f64,
    pub g_mag: f64,
    pub dt: f64,
}

/// `SuspensionLoadBalancer` + `ModuleWheelSuspension` + `VPWheelCollider.UpdateWheelCollider`
/// for one leg, one tick (before the PhysX step).
pub fn update_spring(l: &LegSpec, s: &mut LegState, inp: &SuspInputs) {
    let dt = inp.dt as f32;
    if s.grounded {
        // SuspensionLoadBalancer
        let mut t = inp.vessel_mass;
        if inp.landed {
            t = (t * inp.gee_asl * 10.0).round() / 10.0;
        }
        let n = inp.n_grounded.max(1) as f32;
        let share = round_half_even(100.0 / n) / 100.0;
        let smg = ((t * share as f64) as f32).max(0.5);
        // ModuleWheelSuspension.SuspensionSpringUpdate
        let m = inp.vessel_mass as f32;
        let sc = (s.x as f64 / l.travel) as f32;
        let num = m * sc;
        let clamp_max = lerp(5000.0, 10000.0, inv_lerp(50.0, 200.0, m));
        if sc > 0.8 {
            s.auto_boost = (s.auto_boost + 0.2 * dt).clamp(0.85, 2.0);
        } else if sc < 0.4 {
            s.auto_boost = (s.auto_boost - 0.2 * dt).max(0.0);
        }
        let boost = 1.0 + (sc * boost_curve(inp.gee_asl as f32, 1.0)).clamp(-1.0, 0.5);
        let rearm = !s.auto_set || (l.maximum_load > 0.0 && num > l.maximum_load as f32);
        if rearm {
            s.dtw = if inp.gee_asl < 1.0 {
                1.1
            } else {
                round2(lerp(1.0, 2.0, inv_lerp(1.0, inp.max_surface_gee_asl as f32, inp.gee_asl as f32)))
            };
            if s.m_spring > 0.0 {
                let v = l.travel_cfg as f32 * s.m_spring / 9.80665;
                s.stw = round2(lerp(1.0, 3.0, inv_lerp(0.0, l.maximum_load as f32 * 2.0, v)));
                s.auto_set = true;
            }
        }
        let fudge = lerp(1.0, 15.0, inv_lerp(10.0, 100.0, num));
        let mut spring = (s.stw * l.spring_ratio as f32 * smg * boost_curve(boost, l.boost_ratio as f32 + s.auto_boost)).clamp(0.01, clamp_max);
        let mut damper = (spring * l.damper_ratio as f32 * fudge).clamp(0.01, clamp_max).sqrt() * s.dtw;
        if inp.gee_asl > 1.5 {
            spring *= 3.0;
            damper *= 1.2;
        }
        s.m_damper = round_half_even(damper).max(0.1);
        s.m_spring = round_half_even(spring).max(0.1);
        if (s.m_spring - s.rate).abs() > 3.0 {
            s.rate = s.m_spring;
        }
        if (s.m_damper - s.drate).abs() > 1.05 {
            s.drate = s.m_damper;
        }
    }
    // VPWheelCollider.UpdateWheelCollider (runtime = previous tick's rates)
    let w = (l.sprung_mass * inp.g_mag) as f32;
    let d = l.travel as f32;
    s.k = s.rt.max((w / d).max(0.1));
    s.c = s.rtd.max(0.1);
    let tgt = (w / s.k / d).clamp(0.1, 1.0);
    s.target = if s.target.is_finite() && (tgt - s.target).abs() > 0.02 {
        if s.target > tgt { s.target - 0.02 } else { s.target + 0.02 }
    } else {
        tgt
    };
    s.rt = s.rate;
    s.rtd = s.drate;
}

/// VPP Smooth friction coefficient mu(slip) for the leg curve (UpdateFriction, autoFriction off).
#[inline]
pub fn mu(slip: f32) -> f32 {
    const A: (f32, f32) = (0.01, 0.1);
    const P: (f32, f32) = (2.0, 1.0);
    const L: (f32, f32) = (7.0, 1.0);
    let cubic = |x: f32, p0: (f32, f32), p1: (f32, f32)| {
        let t = (x - p0.0) / (p1.0 - p0.0);
        let (t2, t3) = (t * t, t * t * t);
        p0.1 * (2.0 * t3 - 3.0 * t2 + 1.0) + p1.1 * (-2.0 * t3 + 3.0 * t2)
    };
    if slip <= A.0 {
        A.1
    } else if slip < P.0 {
        cubic(slip, A, P)
    } else if slip < L.0 {
        cubic(slip, P, L)
    } else {
        L.1
    }
}

/// Locked-wheel tire force (sideways, forward) from VPP's Wheel block with a 1000 kN m brake:
/// the requested force Fext is clipped per axis to what friction provides at the current slip
/// (adherent below 0.01 m/s, where the available force scales with slip).
#[inline]
pub fn tire_force(v: [f32; 2], fext: [f32; 2], load: f32) -> [f32; 2] {
    let smag = (v[0] * v[0] + v[1] * v[1]).sqrt();
    let cap = if smag < 1e-6 {
        [0.0, 0.0]
    } else if smag < 0.01 {
        // adherent: force = -(mu(a) load / a) * slip_adherent
        let k = mu(0.01) * load / 0.01;
        [k * v[0].abs(), k * (0.01f32 * 0.01 - v[0] * v[0]).max(0.0).sqrt()]
    } else {
        let f = mu(smag) * load;
        [f * v[0].abs() / smag, f * v[1].abs() / smag]
    };
    [fext[0].clamp(-cap[0], cap[0]), fext[1].clamp(-cap[1], cap[1])]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boost_curve_values() {
        assert_eq!(boost_curve(1.0003416, 1.0), 1.0);
        assert_eq!(boost_curve(1.267, 0.7), 1.4);
        assert_eq!(boost_curve(1.5, 1.0), 3.0);
    }

    #[test]
    fn steady_spring_matches_game() {
        // HopperDrop (v1 trace): landed 4.8678 t on 3 legs, compression x = 0.3174 m of 1.1875,
        // springTweakable 1.04 (set at spawn) -> logged native spring 78, target 0.1
        let l = LegSpec { travel: 1.1875, travel_cfg: 0.95, spring_ratio: 33.0, damper_ratio: 2.2, boost_ratio: 0.7, maximum_load: 40.0, sprung_mass: 0.1, ..Default::default() };
        let mut s = LegState { grounded: true, x: 0.3174, stw: 1.04, dtw: 1.0, auto_set: true, target: 0.1, ..LegState::fresh() };
        let inp = SuspInputs { vessel_mass: 4.8678, landed: true, n_grounded: 3, gee_asl: 1.0003416049313476, max_surface_gee_asl: 1.7005807760832998, g_mag: 9.81, dt: 0.02 };
        for _ in 0..3 {
            update_spring(&l, &mut s, &inp);
        }
        assert_eq!(s.m_spring, 78.0);
        assert_eq!(s.k, 78.0);
        assert!((s.target - 0.1).abs() < 1e-6);
    }
}
