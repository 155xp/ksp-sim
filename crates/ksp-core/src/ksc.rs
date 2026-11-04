//! Kerbal Space Center launch pad geometry, re-derived exactly the way KSP 1.12.5 places it.
//!
//! Provenance (all from the stock Unity asset `KSP_x64_Data/sharedassets9.assets`, read with
//! `tools/extract_ksc.py`; logic from the decompiled Assembly-CSharp):
//!
//! 1. `PQSCity.Orientate` (KSC GameObject, PQSCity MonoBehaviour path_id 20638):
//!      localPosition = normalize(repositionRadial) * (pqs.radius + repositionRadiusOffset)
//!      localRotation = FromToRotation(reorientInitialUp, localPosition.normalized)
//!                      * AngleAxis(reorientFinalAngle, up)
//!    with repositionToSphere = true, repositionToSphereSurface = false.
//! 2. `NestedPrefabSpawner` on the `SpaceCenter` child (identity transform) instantiates the KSC
//!    facility prefab in its place (pos/rot/scale copied from the placeholder = identity).
//! 3. `LaunchPad` facility transform inside that prefab (Transform path_id 10897).
//! 4. `Upgradeables.UpgradeableObject` instantiates the current level prefab in place of the
//!    `Facility` transform (copies its localPosition/localRotation = 0/identity; the prefab keeps
//!    its own localScale), renamed `Facility`.
//! 5. `LaunchSite` "LaunchPad" (PSystemSetup, level9) uses spawnTransformURL
//!    `Facility/LaunchPad_spawn`; `LaunchSite.SpawnPoint.SetSpawnPointLatLonAlt` calls
//!    `CelestialBody.GetLatLonAlt(spawnPointTransform.position)`.
//! 6. `CelestialBody.GetLatLonAlt`: r = BodyFrame.WorldToLocal(worldPos - bodyPos).xzy;
//!    alt = |r| - Radius; lat = asin(r.z/|r|); lon = atan2(r.y, r.x).
//! 7. `ShipConstruction.PutShipToGround`: the craft's lowest collider point is placed exactly at
//!    the spawn point, then rotated by the spawn rotation. The pad deck (MeshCollider) lies a bit
//!    below the spawn point, so a freshly spawned craft drops ~0.12 m onto the deck.
//!
//! Sim body-fixed frame ("BodyFrame", KSP Zup convention, right-handed):
//!   +x -> (lat 0, lon 0), +y -> (lat 0, lon +90 E), +z -> north pole (spin axis).
//!   A Unity body-local vector u maps to it as u.xzy.

use crate::math::{v3, Quat, V3};

// ---------------------------------------------------------------------------------------------
// Raw serialized values (sharedassets9.assets). f32 values are written as their exact f64 widening.
// ---------------------------------------------------------------------------------------------

/// Kerbin `PQS.radius` (PQS MonoBehaviour path_id 20628) == `CelestialBody.Radius`.
pub const KERBIN_PQS_RADIUS: f64 = 600000.0;
/// KSC `PQSCity.repositionRadial` (Unity body-local, left-handed, y = north).
pub const KSC_REPOSITION_RADIAL: V3 = v3(157000.0, -1000.0, -570000.0);
/// KSC `PQSCity.repositionRadiusOffset` (serialized as double; value 42.7f widened).
pub const KSC_REPOSITION_RADIUS_OFFSET: f64 = 42.70000076293945;
/// KSC `PQSCity.reorientInitialUp`.
pub const KSC_REORIENT_INITIAL_UP: V3 = v3(0.0, 1.0, 0.0);
/// KSC `PQSCity.reorientFinalAngle` (deg).
pub const KSC_REORIENT_FINAL_ANGLE_DEG: f64 = -15.0;
/// `LaunchPad` facility transform (path_id 10897) localPosition / localRotation.
pub const LAUNCHPAD_LOCAL_POS: V3 = v3(450.0000305175781, 24.799999237060547, 0.0);
pub const LAUNCHPAD_LOCAL_ROT: Quat = Quat::new(0.0, 0.7071068286895752, 0.0, 0.7071068286895752);
/// Level-prefab root localScale (identical for all three LaunchPad level prefabs).
pub const LAUNCHPAD_LEVEL_SCALE: V3 = v3(0.9999997615814209, 1.0, 0.9999997615814209);

/// Launch pad upgrade level. Sandbox and science mode use the top level (`Level3`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PadLevel {
    /// prefab `LaunchPadLevel1` (career start)
    Level1,
    /// prefab `LaunchPadLevel2`
    Level2,
    /// prefab `LaunchPadLevel4` (fully upgraded; sandbox)
    Level3,
}

impl PadLevel {
    /// `LaunchPad_spawn` localPosition inside the level prefab.
    pub const fn spawn_local(self) -> V3 {
        match self {
            PadLevel::Level1 => v3(0.0, 0.03500000014901161, -2.6937255859375),
            PadLevel::Level2 => v3(0.0, 2.3034000396728516, -1.9686279296875),
            PadLevel::Level3 => v3(0.0, 4.900000095367432, -2.644989252090454),
        }
    }
    /// Height (prefab-local y) of the highest pad MeshCollider triangle directly under the spawn
    /// point (ray cast by tools/extract_ksc.py): L1 `collider`, L2 `LP_barsAlpha`, L3 `Launch Pad`.
    pub const fn deck_local_y(self) -> f64 {
        match self {
            PadLevel::Level1 => -0.10043706300282196,
            PadLevel::Level2 => 2.106259047985084,
            PadLevel::Level3 => 4.781314390259304,
        }
    }
}

/// `LaunchPad_spawn` localRotation (same in all levels): -90 deg about local up.
pub const LAUNCHPAD_SPAWN_LOCAL_ROT: Quat = Quat::new(0.0, -0.7071068286895752, 0.0, 0.7071068286895752);

// ---------------------------------------------------------------------------------------------
// Derived named constants (computed by `derive_pad` below in f64; checked by tests).
// ---------------------------------------------------------------------------------------------

/// Kerbin radius (m). `CelestialBody.Radius`, Kerbin, sharedassets9.
pub const KERBIN_RADIUS: f64 = 600000.0;
/// Kerbin `CelestialBody.rotationPeriod` as serialized (a *solar* day, `solarRotationPeriod`=true).
pub const KERBIN_SOLAR_DAY: f64 = 21600.0;
/// Kerbin sidereal rotation period (s): `CelestialBody.Start` converts the solar day with
/// P_sid = P_sol * T_orb / (T_orb + P_sol), T_orb = 2*pi*sqrt(a^3 / mu_sun).
pub const KERBIN_SIDEREAL_DAY: f64 = 21549.425183089825;
/// Kerbin `CelestialBody.initialRotation` (deg); rotationAngle(UT) = (initialRotation + 360*UT/P) % 360.
pub const KERBIN_INITIAL_ROTATION_DEG: f64 = 90.0;
/// Kerbin spin rate (rad/s) about +z (north) of the body frame: 2*pi / P_sid.
pub const KERBIN_OMEGA: f64 = core::f64::consts::TAU / KERBIN_SIDEREAL_DAY;

/// KSC launch pad (top level, `LaunchPadLevel4` prefab) spawn point, as `GetLatLonAlt` yields it.
pub const KSC_PAD_LAT_DEG: f64 = -0.09720779075939527;
pub const KSC_PAD_LON_DEG: f64 = -74.5576753437982;
/// Altitude ASL of the spawn point (= where the craft's lowest collider point is put).
pub const KSC_PAD_SPAWN_ALT: f64 = 72.5667520618299;
/// Altitude ASL of the pad deck collider surface directly under the spawn point (resting contact).
pub const KSC_PAD_DECK_ALT: f64 = 72.44806638965383;
/// Body-fixed Cartesian position of the pad deck centre (m), sim BodyFrame.
pub const KSC_PAD_DECK_BODY_FIXED: V3 = v3(159779.99675417927, -578408.3843433404, -1018.0800317083273);

/// Result of re-deriving the pad position.
#[derive(Clone, Copy, Debug)]
pub struct PadSite {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub spawn_alt: f64,
    pub deck_alt: f64,
    /// Body-fixed (BodyFrame) position of the spawn point and of the deck point under it.
    pub spawn_body_fixed: V3,
    pub deck_body_fixed: V3,
    /// Local "up" (surface normal at the pad, = KSC PQSCity up after Orientate) in BodyFrame.
    pub pad_up_body_fixed: V3,
}

/// Unity body-local position of a point given in launch-pad level-prefab coordinates.
fn level_local_to_body_unity(p: V3) -> V3 {
    let n = KSC_REPOSITION_RADIAL.normalized();
    let ksc_pos = n * (KERBIN_PQS_RADIUS + KSC_REPOSITION_RADIUS_OFFSET);
    let ksc_rot = Quat::from_to(KSC_REORIENT_INITIAL_UP, ksc_pos.normalized())
        * Quat::angle_axis(KSC_REORIENT_FINAL_ANGLE_DEG, v3(0.0, 1.0, 0.0));
    // Facility instance (identity pos/rot, prefab scale) -> LaunchPad -> SpaceCenter (identity) -> KSC
    let p_fac = p.mul_elem(LAUNCHPAD_LEVEL_SCALE);
    let p_lp = LAUNCHPAD_LOCAL_ROT.rotate(p_fac) + LAUNCHPAD_LOCAL_POS;
    ksc_rot.rotate(p_lp) + ksc_pos
}

/// `CelestialBody.GetLatLonAlt` on a BodyFrame vector.
pub fn lat_lon_alt(r: V3, radius: f64) -> (f64, f64, f64) {
    let m = r.norm();
    let u = r / m;
    (u.z.asin().to_degrees(), u.y.atan2(u.x).to_degrees(), m - radius)
}

/// Inverse: `CelestialBody.GetRelSurfacePosition(lat, lon, alt)` in BodyFrame.
pub fn body_fixed_from_lat_lon_alt(lat_deg: f64, lon_deg: f64, alt: f64, radius: f64) -> V3 {
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    v3(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin()) * (radius + alt)
}

pub fn derive_pad(level: PadLevel) -> PadSite {
    let s = level.spawn_local();
    let spawn = level_local_to_body_unity(s).xzy();
    let deck = level_local_to_body_unity(v3(s.x, level.deck_local_y(), s.z)).xzy();
    let (lat, lon, spawn_alt) = lat_lon_alt(spawn, KERBIN_RADIUS);
    let (_, _, deck_alt) = lat_lon_alt(deck, KERBIN_RADIUS);
    let n = KSC_REPOSITION_RADIAL.normalized();
    let ksc_rot = Quat::from_to(KSC_REORIENT_INITIAL_UP, n)
        * Quat::angle_axis(KSC_REORIENT_FINAL_ANGLE_DEG, v3(0.0, 1.0, 0.0));
    let up = ksc_rot.rotate(v3(0.0, 1.0, 0.0)).xzy();
    PadSite {
        lat_deg: lat,
        lon_deg: lon,
        spawn_alt,
        deck_alt,
        spawn_body_fixed: spawn,
        deck_body_fixed: deck,
        pad_up_body_fixed: up,
    }
}

/// Kerbin rotation angle (deg) at universal time `ut`: `CelestialBody.CBUpdate`.
pub fn kerbin_rotation_angle_deg(ut: f64) -> f64 {
    (KERBIN_INITIAL_ROTATION_DEG + 360.0 * (1.0 / KERBIN_SIDEREAL_DAY) * ut) % 360.0
}

/// Body-fixed -> body-centred inertial (non-rotating, same z axis) at time `ut`.
/// Longitude in KSP's inertial frame = body-fixed longitude + rotationAngle.
pub fn kerbin_fixed_to_inertial(p: V3, ut: f64) -> V3 {
    let a = kerbin_rotation_angle_deg(ut).to_radians();
    let (s, c) = a.sin_cos();
    v3(c * p.x - s * p.y, s * p.x + c * p.y, p.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn named_constants_match_derivation() {
        let p = derive_pad(PadLevel::Level3);
        println!("{p:#?}");
        assert!((p.lat_deg - KSC_PAD_LAT_DEG).abs() < 1e-12);
        assert!((p.lon_deg - KSC_PAD_LON_DEG).abs() < 1e-12);
        assert!((p.spawn_alt - KSC_PAD_SPAWN_ALT).abs() < 1e-8);
        assert!((p.deck_alt - KSC_PAD_DECK_ALT).abs() < 1e-8);
        assert!((p.deck_body_fixed - KSC_PAD_DECK_BODY_FIXED).norm() < 1e-6);
        // body-fixed point round-trips through lat/lon/alt
        let q = body_fixed_from_lat_lon_alt(KSC_PAD_LAT_DEG, KSC_PAD_LON_DEG, KSC_PAD_DECK_ALT, KERBIN_RADIUS);
        // the deck point is 0.12 m below the spawn point along the pad's local up, which is
        // 0.04 deg off the true radial -> ~0.1 mm sideways from the spawn lat/lon
        assert!((q - KSC_PAD_DECK_BODY_FIXED).norm() < 1e-3);
    }

    /// In-game ground truth: stock training scenarios shipped with KSP 1.12.5
    /// (saves/training/B_*.sfs), vessels `sit = PRELAUNCH` on the fully upgraded pad.
    /// alt - hgt is the height ASL of the surface the game's terrain raycast hits.
    #[test]
    fn matches_in_game_saves() {
        let p = derive_pad(PadLevel::Level3);
        // (lat, lon (0..360), alt, hgt) from B_flightBasics, B_fsuborbital, B_goForOrbit
        let saves = [
            (-0.0972077881900498, 285.44232753977, 74.5446779712802, 2.09127),
            (-0.097207773909019, 285.442327246855, 77.781072224956, 5.327667),
            (-0.0972074914371331, 285.442326492292, 82.4201027405215, 9.966702),
        ];
        for (lat, lon360, alt, hgt) in saves {
            let lon = lon360 - 360.0;
            // horizontal: settled craft root within a few cm of the spawn point
            let d_lat_m = (lat - p.lat_deg).to_radians() * KERBIN_RADIUS;
            let d_lon_m = (lon - p.lon_deg).to_radians() * KERBIN_RADIUS * p.lat_deg.to_radians().cos();
            assert!(d_lat_m.abs() < 0.05, "lat off by {d_lat_m} m");
            assert!(d_lon_m.abs() < 0.05, "lon off by {d_lon_m} m");
            // deck height ASL from the game's own terrain raycast: within 1 cm
            assert!(((alt - hgt) - p.deck_alt).abs() < 0.01, "deck {} vs {}", alt - hgt, p.deck_alt);
        }
    }

    #[test]
    fn kerbin_rotation() {
        // sidereal from the solar day exactly as CelestialBody.Start does it
        let mu_sun = 261600000.0f64 * 261600000.0 * (1.7468465610013681 * 9.80665) / 6.67408e-11 * 6.67408e-11;
        let a: f64 = 13599840256.0;
        let t_orb = core::f64::consts::TAU * (a.abs().powi(3) / mu_sun).sqrt();
        let sid = KERBIN_SOLAR_DAY * t_orb / (t_orb + KERBIN_SOLAR_DAY);
        assert!((sid - KERBIN_SIDEREAL_DAY).abs() < 1e-9, "{sid}");
        assert_eq!(kerbin_rotation_angle_deg(0.0), 90.0);
        // after one sidereal day the pad is back where it started in the inertial frame
        let p0 = kerbin_fixed_to_inertial(KSC_PAD_DECK_BODY_FIXED, 0.0);
        let p1 = kerbin_fixed_to_inertial(KSC_PAD_DECK_BODY_FIXED, KERBIN_SIDEREAL_DAY);
        assert!((p0 - p1).norm() < 1e-6);
        // pad surface speed = omega * r * cos(lat) = 174.9637 m/s eastward
        let r = KSC_PAD_DECK_BODY_FIXED;
        let v = KERBIN_OMEGA * (r.x * r.x + r.y * r.y).sqrt();
        assert!((v - 174.9637).abs() < 1e-3, "{v}");
    }

    #[test]
    fn spawn_above_deck_all_levels() {
        for l in [PadLevel::Level1, PadLevel::Level2, PadLevel::Level3] {
            let p = derive_pad(l);
            let gap = p.spawn_alt - p.deck_alt;
            assert!(gap > 0.05 && gap < 0.25, "{l:?} gap {gap}");
            // all within ~10 m of each other horizontally around KSC (lat -0.0972, lon -74.557)
            assert!((p.lat_deg + 0.0972).abs() < 1e-4 && (p.lon_deg + 74.5577).abs() < 1e-3);
        }
    }

    #[test]
    fn pad_up_is_radial() {
        let p = derive_pad(PadLevel::Level3);
        // PQSCity reorients KSC 'up' to the sphere radial through the KSC origin; ~0.04 deg from the pad's own radial.
        let ang = p.pad_up_body_fixed.dot(p.deck_body_fixed.normalized()).acos().to_degrees();
        assert!(ang < 0.1, "{ang}");
    }
}
