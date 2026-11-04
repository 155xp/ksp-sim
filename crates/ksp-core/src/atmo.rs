//! Atmosphere: pressure / temperature / density / speed of sound, exactly as stock KSP computes
//! them for a vessel each FixedUpdate.
//!
//! Sources (decomp CelestialBody.cs, FlightIntegrator.cs):
//! - `CelestialBody.GetPressure(alt)` (:498): kPa; 0 at/above atmosphereDepth; curve evaluated
//!   at `(float)alt` (or alt/depth when normalized).
//! - `CelestialBody.GetTemperature(alt)` (:563), `GetFullTemperature(alt, offset)` (:2234):
//!   T = T(alt) + atmosphereTemperatureSunMultCurve((float)alt) * offset.
//! - `CelestialBody.GetDensity(P, T)` (:611): rho = P*1000*M / (R*T).
//! - `CelestialBody.GetSpeedOfSound(P, rho)` (:645): sqrt(gamma*P*1000/rho).
//! - `FlightIntegrator.CalculatePressure` (:1766): vessel pressure is 0 above atmosphereDepth.
//! - `FlightIntegrator.CalculateConstantsAtmosphere` (:1988): q = 0.0005*rho*v^2 (kPa),
//!   mach = v / c, pseudoReynolds = rho * v.
//! - `CelestialBody.GetAtmoThermalStats` (:1917): the latitude / time-of-day temperature offset.

use crate::body::{RawBody, IDEAL_GAS_CONSTANT};
use crate::curve::eval;
use crate::math::V3;

/// kPa -> atm, `PhysicsGlobals.KpaToAtmospheres` (1/101.325; CelestialBody.cs:452).
pub const KPA_TO_ATM: f64 = 0.009869232667160128;
/// `PhysicsGlobals.SpaceTemperature` (Physics.cfg `spaceTemperature = 4`).
pub const SPACE_TEMPERATURE: f64 = 4.0;

/// `FloatCurve.Evaluate(float)`: f32 argument and result.
#[inline(always)]
fn feval(keys: &[crate::curve::Key], x: f64) -> f64 {
    eval(keys, x as f32 as f64) as f32 as f64
}

/// `CelestialBody.GetPressure` (kPa).
#[inline]
pub fn pressure_kpa(b: &RawBody, alt: f64) -> f64 {
    if !b.atmosphere || alt >= b.atmosphere_depth {
        return 0.0;
    }
    if b.atm_use_pressure_curve {
        if b.atm_pressure_curve_is_normalized {
            let f = feval(b.pressure_curve, alt / b.atmosphere_depth).clamp(0.0, 1.0);
            (b.atm_pressure_sea_level as f32 * f as f32) as f64 // Mathf.Lerp(0, P0, f)
        } else {
            feval(b.pressure_curve, alt)
        }
    } else {
        b.atm_pressure_sea_level
            * (1.0 - b.atm_temperature_lapse_rate * alt / b.atm_temperature_sea_level)
                .powf(b.atm_gas_mass_lapse_rate)
    }
}

/// `CelestialBody.GetTemperature` (K), without the sun/latitude offset.
#[inline]
pub fn temperature(b: &RawBody, alt: f64) -> f64 {
    if alt >= b.atmosphere_depth {
        return 0.0;
    }
    if b.atm_use_temperature_curve {
        if b.atm_temperature_curve_is_normalized {
            let f = feval(b.temperature_curve, alt / b.atmosphere_depth);
            SPACE_TEMPERATURE + (b.atm_temperature_sea_level - SPACE_TEMPERATURE) * f
        } else {
            feval(b.temperature_curve, alt)
        }
    } else {
        b.atm_temperature_sea_level - b.atm_temperature_lapse_rate * alt
    }
}

/// `CelestialBody.GetFullTemperature(alt, atmosphereTemperatureOffset)`.
#[inline]
pub fn full_temperature(b: &RawBody, alt: f64, offset: f64) -> f64 {
    temperature(b, alt) + feval(b.atmosphere_temperature_sun_mult_curve, alt) * offset
}

/// `CelestialBody.GetDensity` (kg/m^3).
#[inline]
pub fn density(b: &RawBody, p_kpa: f64, t: f64) -> f64 {
    if p_kpa <= 0.0 || t <= 0.0 {
        return 0.0;
    }
    p_kpa * 1000.0 * b.atm_molar_mass / (IDEAL_GAS_CONSTANT * t)
}

/// `CelestialBody.GetSpeedOfSound` (m/s).
#[inline]
pub fn speed_of_sound(b: &RawBody, p_kpa: f64, rho: f64) -> f64 {
    if p_kpa <= 0.0 || rho <= 0.0 {
        return 0.0;
    }
    (b.atm_adiabatic_index * p_kpa * 1000.0 / rho).sqrt()
}

/// `CelestialBody.GetAtmoThermalStats` temperature offset (K, before the altitude multiplier).
///
/// `sun_dir`: unit vector vessel -> sun; `up`: vessel's radial up; `spin_axis`: body north
/// (`bodyTransform.up`); `rot_sign`: sign(rotationPeriod) (+1 for stock bodies);
/// `ecc_frac`: (r - Pe)/(Ap - Pe) of the sun-orbiting ancestor or `None` if circular;
/// `true_anomaly_deg` of that ancestor. All vectors in one common (inertial) frame.
pub fn temperature_offset(
    b: &RawBody,
    sun_dir: V3,
    up: V3,
    spin_axis: V3,
    true_anomaly_deg: f64,
    ecc_frac: Option<f64>,
) -> f64 {
    if !b.atmosphere || b.is_star {
        return 0.0;
    }
    let d1 = sun_dir.dot(spin_axis);
    let d2 = spin_axis.dot(up);
    let acos0 = |d: f64| {
        let a = d.acos();
        if a.is_nan() { if d > 0.0 { 0.0 } else { core::f64::consts::PI } } else { a }
    };
    let colat0 = acos0(d2);
    let sun_polar = acos0(d1);
    let max_f = (1.0 + (sun_polar - colat0).cos()) * 0.5;
    let min_f = (1.0 + (sun_polar + colat0).cos()) * 0.5;
    let colat = if colat0 > core::f64::consts::FRAC_PI_2 { core::f64::consts::PI - colat0 } else { colat0 };
    let lat_deg = (core::f64::consts::FRAC_PI_2 - colat) * 57.295780181884766;
    // Quaternion.AngleAxis(45 * sign(rotationPeriod), up_spin) * upAxis: 45 deg "afternoon lag"
    let q = crate::math::Quat::angle_axis(45.0, spin_axis);
    let tod_f = (1.0 + sun_dir.dot(q.rotate(up))) * 0.5;
    let diff = max_f - min_f;
    let sun_f = if diff > 0.001 {
        let s = (tod_f - min_f) / diff;
        if s.is_nan() { if tod_f > 0.5 { 1.0 } else { 0.0 } } else { s }
    } else {
        min_f + diff * 0.5
    };
    let ta = (true_anomaly_deg + 360.0) % 360.0;
    let mut off = feval(b.latitude_temperature_bias_curve, lat_deg)
        + feval(b.latitude_temperature_sun_mult_curve, lat_deg) * sun_f
        + feval(b.axial_temperature_sun_bias_curve, ta) * feval(b.axial_temperature_sun_mult_curve, lat_deg);
    if let Some(e) = ecc_frac {
        off += feval(b.eccentricity_temperature_bias_curve, e);
    }
    off
}

/// Vessel-level atmospheric state for one tick (FlightIntegrator.CalculateConstantsAtmosphere).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AtmState {
    pub p_kpa: f64,
    pub t: f64,
    pub rho: f64,
    pub c: f64,
    pub mach: f64,
    pub q_kpa: f64,
    /// DRAG_PSEUDOREYNOLDS(rho * v)
    pub pseudo_re_drag_mult: f64,
}

/// `spd` is the (smoothed) surface-relative speed.
#[inline]
pub fn atm_state(b: &RawBody, alt: f64, spd: f64, temp_offset: f64) -> AtmState {
    let p = if b.atmosphere && alt <= b.atmosphere_depth { pressure_kpa(b, alt) } else { 0.0 };
    if p <= 0.0 {
        // CalculateConstantsVacuum
        return AtmState { t: SPACE_TEMPERATURE, pseudo_re_drag_mult: 1.0, ..Default::default() };
    }
    let t = full_temperature(b, alt, temp_offset);
    let rho = density(b, p, t);
    let c = speed_of_sound(b, p, rho);
    let mach = if c > 0.0 { spd / c } else { 0.0 };
    AtmState {
        p_kpa: p,
        t,
        rho,
        c,
        mach,
        q_kpa: 0.0005 * rho * spd * spd,
        pseudo_re_drag_mult: feval(crate::aero::DRAG_PSEUDOREYNOLDS, rho * spd),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::KERBIN;

    #[test]
    fn kerbin_sea_level() {
        let p = pressure_kpa(KERBIN, 0.0);
        assert!((p - 101.325).abs() < 1e-4, "{p}");
        assert!((p * KPA_TO_ATM - 1.0).abs() < 1e-6);
        // CelestialBody.Start: atmDensityASL = GetDensity(GetPressure(0), GetTemperature(0))
        let t0 = temperature(KERBIN, 0.0);
        let rho0 = density(KERBIN, p, t0);
        println!("T0 = {t0} K, rho0 = {rho0} kg/m3, c0 = {}", speed_of_sound(KERBIN, p, rho0));
        assert!((t0 - 288.15).abs() < 1e-3, "{t0}");
        assert!((rho0 - 1.2250).abs() < 1e-3, "{rho0}");
    }

    #[test]
    fn kerbin_top_of_atmosphere() {
        assert_eq!(pressure_kpa(KERBIN, 70_000.0), 0.0);
        assert!(pressure_kpa(KERBIN, 69_999.0) > 0.0);
        assert_eq!(atm_state(KERBIN, 80_000.0, 2000.0, 0.0).rho, 0.0);
    }

    #[test]
    fn pressure_monotonic() {
        let mut last = f64::INFINITY;
        for i in 0..700 {
            let p = pressure_kpa(KERBIN, i as f64 * 100.0);
            assert!(p < last, "alt {}", i * 100);
            last = p;
        }
    }
}
