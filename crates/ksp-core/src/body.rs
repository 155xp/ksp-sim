//! Celestial bodies: serialized prefab fields (bodies_data.rs, generated) plus the quantities
//! the game derives from them at load time.

use crate::bodies_data;
use crate::curve::Key;

/// `PhysicsGlobals.GravitationalAcceleration` (PhysicsGlobals.cs:161): standard g0, used for
/// GeeASL -> GM and for Isp <-> exhaust velocity.
pub const G0: f64 = 9.80665;
/// Gravitational constant hard-coded in `CelestialBody.Awake` (CelestialBody.cs:722-723).
pub const GRAV_CONSTANT: f64 = 6.67408e-11;
/// `PhysicsGlobals.idealGasConstant` (PhysicsGlobals.cs:149), J/(mol K).
pub const IDEAL_GAS_CONSTANT: f64 = 8.31447;

/// Serialized `CelestialBody` fields (+ its `OrbitDriver.orbit`), stock prefab.
#[derive(Clone, Copy, Debug)]
pub struct RawBody {
    pub name: &'static str,
    /// index into `bodies_data::ALL` of `orbit.referenceBody`
    pub parent: Option<usize>,
    pub gee_asl: f64,
    pub radius: f64,
    pub rotates: bool,
    /// serialized `rotationPeriod`; a *solar* day when `solar_rotation_period`
    pub rotation_period: f64,
    pub solar_rotation_period: bool,
    pub initial_rotation_deg: f64,
    pub tidally_locked: bool,
    pub inverse_rot_threshold_altitude: f64,
    pub clamp_inverse_rot_threshold: bool,
    pub atmosphere: bool,
    pub atmosphere_contains_oxygen: bool,
    pub atmosphere_depth: f64,
    pub atm_temperature_sea_level: f64,
    pub atm_pressure_sea_level: f64,
    pub atm_molar_mass: f64,
    pub atm_adiabatic_index: f64,
    pub atm_temperature_lapse_rate: f64,
    pub atm_gas_mass_lapse_rate: f64,
    pub atm_use_temperature_curve: bool,
    pub atm_temperature_curve_is_normalized: bool,
    pub atm_use_pressure_curve: bool,
    pub atm_pressure_curve_is_normalized: bool,
    pub max_axial_dot: f64,
    pub radius_atmo_factor: f64,
    pub ocean: bool,
    pub has_solid_surface: bool,
    pub is_star: bool,
    pub orbit_inclination_deg: f64,
    pub orbit_eccentricity: f64,
    pub orbit_sma: f64,
    pub orbit_lan_deg: f64,
    pub orbit_arg_pe_deg: f64,
    pub orbit_mean_anomaly_at_epoch: f64,
    pub orbit_epoch: f64,
    pub pressure_curve: &'static [Key],
    pub temperature_curve: &'static [Key],
    pub latitude_temperature_bias_curve: &'static [Key],
    pub latitude_temperature_sun_mult_curve: &'static [Key],
    pub axial_temperature_sun_bias_curve: &'static [Key],
    pub axial_temperature_sun_mult_curve: &'static [Key],
    pub atmosphere_temperature_sun_mult_curve: &'static [Key],
    pub eccentricity_temperature_bias_curve: &'static [Key],
}

impl RawBody {
    /// `CelestialBody.Awake`: Mass = R^2 * GeeASL * g0 / G (CelestialBody.cs:722).
    pub fn mass(&self) -> f64 {
        self.radius * self.radius * (self.gee_asl * G0) / GRAV_CONSTANT
    }
    /// `CelestialBody.Awake`: gravParameter = Mass * G (CelestialBody.cs:723).
    pub fn gm(&self) -> f64 {
        self.mass() * GRAV_CONSTANT
    }
    pub fn parent(&self) -> Option<&'static RawBody> {
        self.parent.map(|i| &bodies_data::ALL[i])
    }
    /// `CelestialBody.Start`: SOI = a * (M / M_parent)^0.4 (CelestialBody.cs:777).
    pub fn sphere_of_influence(&self) -> f64 {
        match self.parent() {
            Some(p) => self.orbit_sma * (self.mass() / p.mass()).powf(0.4),
            None => f64::INFINITY,
        }
    }
    /// Orbital period around the parent, 2*pi*sqrt(|a|^3 / mu_parent).
    pub fn orbital_period(&self) -> f64 {
        match self.parent() {
            Some(p) => core::f64::consts::TAU * (self.orbit_sma.abs().powi(3) / p.gm()).sqrt(),
            None => f64::INFINITY,
        }
    }
    /// Sidereal rotation period. `CelestialBody.Start` (CelestialBody.cs:783-788) converts a
    /// solar day: P_sid = P_sol * T_orb / (T_orb + P_sol).
    pub fn sidereal_period(&self) -> f64 {
        if self.solar_rotation_period && self.parent.is_some() {
            let t = self.orbital_period();
            self.rotation_period * t / (t + self.rotation_period)
        } else {
            self.rotation_period
        }
    }
    /// Spin rate (rad/s) about the body's north (+z of the body frame).
    pub fn angular_velocity(&self) -> f64 {
        if self.rotates { core::f64::consts::TAU / self.sidereal_period() } else { 0.0 }
    }
}

pub fn by_name(name: &str) -> Option<&'static RawBody> {
    bodies_data::ALL.iter().find(|b| b.name == name)
}

pub const KERBIN: &RawBody = &bodies_data::KERBIN;

// --- Named Kerbin constants (values checked against the derivations by tests below) ----------

/// Kerbin equatorial radius (m). Serialized `CelestialBody.Radius` (= PQS.radius).
pub const KERBIN_RADIUS: f64 = 600_000.0;
/// Kerbin GM (m^3/s^2): GeeASL(1.0003416049313476) * g0 * R^2.
pub const KERBIN_GM: f64 = 3.5316000e12;
/// Kerbin sidereal rotation period (s), from the 21600 s solar day.
pub const KERBIN_SIDEREAL_DAY: f64 = 21549.425183089825;
/// Kerbin sphere of influence (m).
pub const KERBIN_SOI: f64 = 84_159_286.47963;
/// Kerbin atmosphere depth (m). Serialized `atmosphereDepth`.
pub const KERBIN_ATMOSPHERE_DEPTH: f64 = 70_000.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kerbin_constants() {
        let k = KERBIN;
        assert_eq!(k.radius, KERBIN_RADIUS);
        assert_eq!(k.atmosphere_depth, KERBIN_ATMOSPHERE_DEPTH);
        assert!((k.gm() - KERBIN_GM).abs() / KERBIN_GM < 1e-12, "{}", k.gm());
        assert!((k.sidereal_period() - KERBIN_SIDEREAL_DAY).abs() < 1e-8, "{}", k.sidereal_period());
        assert!((k.sphere_of_influence() - KERBIN_SOI).abs() < 1e-3, "{}", k.sphere_of_influence());
        assert_eq!(k.inverse_rot_threshold_altitude, 100_000.0);
        assert_eq!(k.parent().unwrap().name, "Sun");
    }

    /// Published values (KSP wiki, MechJeb/kOS readouts of stock 1.12.x).
    #[test]
    fn published_body_values() {
        let mun = by_name("Mun").unwrap();
        assert!((mun.gm() - 6.5138398e10).abs() / 6.5138398e10 < 1e-7);
        assert!((mun.sphere_of_influence() - 2_429_559.1).abs() < 1.0);
        let minmus = by_name("Minmus").unwrap();
        assert!((minmus.gm() - 1.7658000e9).abs() / 1.7658000e9 < 1e-7);
        let sun = by_name("Sun").unwrap();
        assert!((sun.gm() - 1.1723328e18).abs() / 1.1723328e18 < 1e-7);
        // Kerbin orbital period 9 203 545 s (426.08 days)
        assert!((KERBIN.orbital_period() - 9_203_544.6).abs() < 1.0, "{}", KERBIN.orbital_period());
    }
}
