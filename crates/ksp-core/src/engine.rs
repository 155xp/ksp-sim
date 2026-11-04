//! `ModuleEngines` per-tick model (throttle response, Isp vs pressure, fuel flow, flameout).
//!
//! Sources (decomp ModuleEngines.cs): field `g = 9.80665f` (:281); OnLoad maxFuelFlow =
//! maxThrust / (atmosphereCurve.Evaluate(0) * g) (:1048); `UpdateThrottle` (:2977);
//! `RequiredPropellantMass` (:3292): realIsp = atmosphereCurve(part.staticPressureAtm),
//! massFlow = maxFuelFlow * throttle * flowMultiplier, thrust = massFlow * realIsp * g;
//! `RequestPropellant` (:794) / `CheckDeprived` (:750): flameout if any propellant has less than
//! ignitionThreshold * request; `Flameout` (:674) zeroes currentThrottle; `ThrustUpdate` (:3605):
//! force = -thrustTransform.forward * finalThrust at the transform position.

use crate::curve::eval;
use crate::parts::N_RES;
use crate::vessel::EngineSpec;

/// `ModuleEngines.g` is a float field: 9.80665f widened to double.
pub const ENGINE_G0: f64 = 9.80665f32 as f64;
/// `FlightIntegrator`/ModuleEngines flameout bar on the flow multiplier (`flameoutBar`).
pub const FLAMEOUT_BAR: f64 = 0.07;

/// Engine Isp at static pressure `p_atm` (`atmosphereCurve.Evaluate((float)p)`).
#[inline]
pub fn isp(e: &EngineSpec, p_atm: f64) -> f64 {
    eval(&e.isp_keys[..e.n_isp_keys as usize], p_atm as f32 as f64) as f32 as f64
}

/// `UpdateThrottle` for the plain (non-alternate) path. `req` already includes thrustPercentage.
#[inline]
pub fn update_throttle(e: &EngineSpec, cur: f64, req: f64, dt: f64) -> f64 {
    let throttle_min = if e.max_fuel_flow > 0.0 { e.min_fuel_flow / e.max_fuel_flow } else { 0.0 };
    let req = if throttle_min != 0.0 { throttle_min + (1.0 - throttle_min) * req } else { req };
    if !e.use_response {
        return req;
    }
    if (cur - req).abs() <= 1e-4 {
        req
    } else if cur < req {
        cur + (req - cur) * (e.accel_speed * dt).clamp(0.0, 1.0)
    } else {
        cur + (req - cur) * (e.decel_speed * dt).clamp(0.0, 1.0)
    }
}

/// One tick of propellant draw + thrust. `res` is the vessel resource pool (units), drawn in place.
/// Returns (finalThrust kN, flamed_out, current throttle after a flameout reset).
#[inline]
pub fn burn(e: &EngineSpec, throttle: f64, p_atm: f64, res: &mut [f64; N_RES], dt: f64) -> (f64, bool, f64) {
    if throttle <= 0.0 {
        return (0.0, false, throttle);
    }
    let real_isp = isp(e, p_atm);
    let flow_mult = 1.0; // no atmChangeFlow/velCurve on stock rockets
    if flow_mult < FLAMEOUT_BAR {
        return (0.0, true, 0.0);
    }
    let mass_flow = e.max_fuel_flow * throttle * flow_mult; // t/s
    let resulting_thrust = mass_flow * real_isp * ENGINE_G0;
    let mass = mass_flow * dt;
    // RequestPropellant: deprived check first, then draw and average the received fractions
    let mut n = 0.0;
    for r in 0..N_RES {
        let u = e.units_per_ton[r];
        if u > 0.0 {
            let req = mass * u;
            if res[r] < req * e.ignition_threshold {
                return (0.0, true, 0.0);
            }
            n += 1.0;
        }
    }
    let mut frac_sum = 0.0;
    for r in 0..N_RES {
        let u = e.units_per_ton[r];
        if u > 0.0 {
            let req = mass * u;
            let got = req.min(res[r]).max(0.0);
            res[r] -= got;
            frac_sum += got / req;
        }
    }
    let frac = (frac_sum / n).clamp(0.0, 1.0);
    (resulting_thrust * frac, false, throttle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vessel::{Craft, VesselSpec};

    fn spec_of(def: crate::parts::PartDef) -> EngineSpec {
        let c = Craft { parts: vec![crate::vessel::Placed { def, pos: crate::math::V3::ZERO, rot: crate::math::Quat::IDENTITY, stack: vec![], parent: None }] };
        VesselSpec::compile(&c).engines[0]
    }

    /// Published stock numbers (KSP wiki / in-game part info, 1.12): vac and ASL thrust.
    #[test]
    fn thrust_vs_pressure_matches_published() {
        for (def, vac, asl, isp_vac, isp_asl) in [
            (crate::parts::swivel(), 215.0, 167.969, 320.0, 250.0),
            (crate::parts::terrier(), 60.0, 14.783, 345.0, 85.0),
        ] {
            let e = spec_of(def);
            let mut res = [1e9; N_RES];
            let (t_vac, ..) = burn(&e, 1.0, 0.0, &mut res, 0.02);
            let (t_asl, ..) = burn(&e, 1.0, 1.0, &mut res, 0.02);
            assert!((t_vac - vac).abs() < 1e-4, "{t_vac}");
            assert!((t_asl - asl).abs() < 1e-3, "{t_asl} vs {asl}");
            assert_eq!(isp(&e, 0.0), isp_vac);
            assert_eq!(isp(&e, 1.0), isp_asl);
        }
    }

    #[test]
    fn propellant_draw_and_flameout() {
        let e = spec_of(crate::parts::swivel());
        let mut res = [180.0, 220.0, 0.0, 0.0];
        let (_, out, _) = burn(&e, 1.0, 0.0, &mut res, 1.0);
        assert!(!out);
        // 0.068512 t/s of a 0.9:1.1 mixture with density 0.01 t per (0.9+1.1) "mixture unit"
        let lf = 180.0 - res[0];
        let ox = 220.0 - res[1];
        assert!((lf / ox - 0.9 / 1.1).abs() < 1e-12);
        assert!(((lf + ox) * 0.005 - e.max_fuel_flow).abs() < 1e-12);
        let mut empty = [0.0, 220.0, 0.0, 0.0];
        assert!(burn(&e, 1.0, 0.0, &mut empty, 0.02).1);
    }
}
