//! Single-environment physics tick: a pure function of (spec, params, state, control).
//! The batched stepper (batch.rs) gathers SoA columns into an `EnvState`, runs `tick` a few
//! times, and scatters back, exactly the shape a CUDA kernel (one thread per env) takes.
//!
//! Per-tick order follows KSP's FixedUpdate (see the frames/integration notes in DESIGN.md):
//! 1. mass stats (FlightIntegrator.UpdateMassStats) and vessel CoM / velocity
//!    (VesselPrecalculate.CalculatePhysicsStats);
//! 2. velocity smoothing (FlightIntegrator.SmoothVelocity), altitude, atmosphere
//!    (CalculatePressure, CalculateConstantsAtmosphere);
//! 3. part modules: engines (ModuleEngines), gimbal (ModuleGimbal), reaction wheel;
//! 4. gravity + Coriolis + centrifugal at the CoM (VesselPrecalculate.CalculateGravity) applied
//!    as a uniform acceleration (FlightIntegrator.Integrate);
//! 5. per-part aero (FlightIntegrator.UpdateAerodynamics) and angular drag;
//! 6. ground contact (approximation of ModuleWheelSuspension / PhysX contacts);
//! 7. PhysX semi-implicit Euler (v += a dt; x += v dt), or in vacuum the orbit drift
//!    compensation (VesselPrecalculate, ORBIT_DRIFT_COMPENSATION) that pins the CoM to the
//!    Kepler conic and adds only non-gravitational dv;
//! 8. rotating <-> inertial frame switch at inverseRotThresholdAltitude
//!    (OrbitPhysicsManager.checkReferenceFrame / setRotatingFrame).

use crate::aero::{mach_terms, BODY_LIFT_MULTIPLIER, DRAG_CUBE_MULTIPLIER, DRAG_MULTIPLIER};
use crate::atmo::{self, KPA_TO_ATM};
use crate::body::{RawBody, KERBIN};
use crate::engine;
use crate::ksc;
use crate::math::{v3, Quat, V3};
use crate::orbit;
use crate::parts::N_RES;
use crate::vessel::{VesselSpec, MAX_ENGINES, MAX_LEGS};
use crate::wheel::{self, LegState};

/// Flight physics step: `Time.fixedDeltaTime` = 0.02 s (TimeWarp.updateRate, 1x physics warp).
pub const FIXED_DT: f64 = 0.02;
/// FlightIntegrator.VelSmoothLen.
pub const VEL_SMOOTH_LEN: usize = 10;
/// FlightIntegrator.maxVelDeltaSqr (spike rejection threshold, (m/s^2)^2).
pub const MAX_VEL_DELTA_SQR: f32 = 1e6;
/// Physics.cfg maxAngularVelocity (rad/s).
pub const MAX_ANGULAR_VELOCITY: f64 = 50.0;
/// Ticks between refreshes of the sun/latitude temperature offset (sun moves 0.0003 deg/tick).
pub const TEMP_OFFSET_TICKS: u32 = 25;
/// `PSystemManager.MaximumSurfaceGeeASL`: max GeeASL over stock bodies with a solid surface (Eve).
pub const MAX_SURFACE_GEE_ASL: f64 = 1.7005807760832998;
/// Physics.cfg orbitDriftFramesToWait.
pub const ORBIT_DRIFT_FRAMES_TO_WAIT: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Frame {
    /// Kerbin-fixed (KSP's rotating "inverseRotation" world frame)
    #[default]
    Rotating = 0,
    /// Kerbin-centred inertial (planetarium axes)
    Inertial = 1,
}

pub const STATUS_FLYING: u8 = 0;
pub const STATUS_CONTACT: u8 = 1;
pub const STATUS_LANDED: u8 = 2;
pub const STATUS_CRASHED: u8 = 3;

/// Complete per-env dynamic state.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct EnvState {
    pub ut: f64,
    pub frame: Frame,
    /// root part origin position / velocity, rotation (root-local -> frame), angular velocity
    pub x: V3,
    pub v: V3,
    pub q: Quat,
    pub w: V3,
    pub res: [f64; N_RES],
    pub thr: [f64; MAX_ENGINES],
    pub gimbal: [[f64; 2]; MAX_ENGINES],
    pub flameout: u8,
    pub rw_in: V3,
    pub smooth_buf: [[f32; 3]; VEL_SMOOTH_LEN],
    pub smooth_idx: u8,
    pub vel_spiking: bool,
    pub last_vel_provisional: bool,
    pub last_vel: [f32; 3],
    pub last_vel_idx: u8,
    pub drift_frames: u8,
    pub status: u8,
    pub settled_ticks: u16,
    pub ticks: u32,
    /// max normal impact speed seen this step (diagnostics)
    pub impact_speed: f32,
    /// cached GetAtmoThermalStats temperature offset (K); NaN = recompute
    pub toff: f32,
    pub legs: [LegState; MAX_LEGS],
    /// Translation (body frame, m) of the scenery colliders from their nominal (surveyed)
    /// placement. KSP rebuilds the PQS / KSC float transforms from the double body position at
    /// every FloatingOrigin shift (vessel > 500 m from the origin), so the pad surface moves by
    /// float rounding at ~600 km scale; see Scenery and DESIGN.md "Scenery placement".
    pub ground_shift: V3,
    /// per-session radial offset of the KSC colliders (m, see Scenery)
    pub ksc_dz: f64,
    /// hull points in ground contact after the last tick (bit k = hull point k): a contact that
    /// was not there before is a PhysX OnCollisionEnter (Part.HandleCollision crash check)
    pub hull_touch: u32,
}

/// Pilot input, as `FlightCtrlState` (mainThrottle, pitch, yaw, roll) in [0,1] / [-1,1].
#[derive(Clone, Copy, Debug, Default)]
pub struct Control {
    pub throttle: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
}

/// World / model parameters shared by all envs.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub body: &'static RawBody,
    /// terrain modelled as the sphere R + ground_alt (m); default = pad deck altitude
    pub ground_alt: f64,
    pub friction: f64,
    /// include the GetAtmoThermalStats latitude/sun offset in temperature (density)
    pub sun_temperature: bool,
    pub drift_compensation: bool,
    pub aero: bool,
    pub ground: bool,
    /// use the surveyed KSC collision heightfield (terrain.rs) when available; else the sphere
    pub terrain: bool,
    /// destroy the vessel on impacts above a part's crashTolerance
    pub crash: bool,
    /// also end the episode when the estimated load through a part joint exceeds its breakForce.
    /// Off by default: in the in-game crash series (11 vertical pad drops, 9.7..39.7 m/s) no joint
    /// ever broke, although the estimate passes 4400 kN from ~22 m/s; the game's only failure
    /// (25.8 m/s) was a crashTolerance explosion. See VALIDATION_INGAME.md.
    pub joint_break: bool,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            body: KERBIN,
            ground_alt: ksc::KSC_PAD_DECK_ALT_GAME,
            friction: 1.0,
            sun_temperature: true,
            drift_compensation: true,
            aero: true,
            ground: true,
            terrain: true,
            crash: true,
            joint_break: false,
        }
    }
}

/// Placement of the scenery colliders relative to the surveyed ground (see DESIGN.md "Scenery
/// placement"). KSP builds the PQS / KSC float transforms from double body positions:
/// - `shift`: translation of all scenery (PQS terrain and KSC) by the PQS transform's rounding
///   error, re-rounded at every FloatingOrigin shift (each tick while radar altitude > ~150 m);
///   radial part sigma 9 mm, within +-2.2 cm (2589 in-game samples).
/// - `ksc_dz`: extra radial offset of the KSC colliders (pad, apron, buildings) fixed per game
///   session (scene load): 0 in 7 of 9 measured sessions, +0.037 m and -0.013 m in the others.
#[derive(Clone, Copy, Debug, Default)]
pub struct Scenery {
    pub shift: V3,
    pub ksc_dz: f64,
}

/// Ground ray `a - s u`, s in [0, len]: (s, hit point, outward normal).
#[inline]
pub fn ground_ray(prm: &Params, a: V3, u: V3, len: f64) -> Option<(f64, V3, V3)> {
    ground_ray_sc(prm, &Scenery::default(), a, u, len)
}

/// Ground ray against the scenery as placed by `sc`.
#[inline]
pub fn ground_ray_sc(prm: &Params, sc: &Scenery, a: V3, u: V3, len: f64) -> Option<(f64, V3, V3)> {
    let a = a - sc.shift;
    if prm.terrain {
        if let Some(t) = crate::terrain::ksc() {
            return t.ray_k(a, u, len, sc.ksc_dz).map(|(s, p, n)| (s, p + sc.shift, n));
        }
    }
    let rg = prm.body.radius + prm.ground_alt + sc.ksc_dz;
    let au = a.dot(u);
    let disc = au * au - (a.norm2() - rg * rg);
    if disc < 0.0 {
        return None;
    }
    let s = au - disc.sqrt();
    if s < 0.0 || s > len {
        return None;
    }
    let p = a - u * s;
    Some((s, p + sc.shift, p.normalized()))
}

/// Distance from the body centre above which no ground exists (any scenery placement within
/// |shift| and ksc_dz <= 0 is added by the caller): early-out for contact tests.
#[inline]
pub fn ground_ceiling(prm: &Params) -> f64 {
    if prm.terrain {
        if let Some(t) = crate::terrain::ksc() {
            return t.radius + t.max_alt;
        }
    }
    prm.body.radius + prm.ground_alt
}

/// Height of body-fixed point `p` above the ground (negative = below).
#[inline]
pub fn ground_height(prm: &Params, p: V3) -> f64 {
    ground_height_sc(prm, &Scenery::default(), p)
}

#[inline]
pub fn ground_height_sc(prm: &Params, sc: &Scenery, p: V3) -> f64 {
    let p = p - sc.shift;
    if prm.terrain {
        if let Some(t) = crate::terrain::ksc() {
            return t.height_above_k(p, sc.ksc_dz);
        }
    }
    p.norm() - prm.body.radius - prm.ground_alt - sc.ksc_dz
}

#[inline(always)]
fn spin(b: &RawBody) -> V3 {
    v3(0.0, 0.0, b.angular_velocity())
}

/// Body rotation angle (rad) at `ut`: `CelestialBody.CBUpdate`, (initialRotation + 360 ut/P) % 360.
#[inline]
pub fn rotation_angle(b: &RawBody, ut: f64) -> f64 {
    if !b.rotates {
        return b.initial_rotation_deg.to_radians();
    }
    ((b.initial_rotation_deg + 360.0 * (1.0 / b.sidereal_period()) * ut) % 360.0).to_radians()
}

#[inline(always)]
fn rot_z(p: V3, a: f64) -> V3 {
    let (s, c) = a.sin_cos();
    v3(c * p.x - s * p.y, s * p.x + c * p.y, p.z)
}

/// Body-fixed -> inertial (position, velocity) at `ut`.
#[inline]
pub fn fixed_to_inertial(b: &RawBody, ut: f64, x: V3, v: V3) -> (V3, V3) {
    let a = rotation_angle(b, ut);
    (rot_z(x, a), rot_z(v + spin(b).cross(x), a))
}

#[inline]
pub fn inertial_to_fixed(b: &RawBody, ut: f64, x: V3, v: V3) -> (V3, V3) {
    let a = rotation_angle(b, ut);
    let xf = rot_z(x, -a);
    (xf, rot_z(v, -a) - spin(b).cross(xf))
}

/// Unit vector from the body to the sun, body-fixed frame (for the temperature offset), plus the
/// body's true anomaly (deg). Bodies orbiting the Sun directly only (Kerbin).
pub fn sun_dir_fixed(b: &RawBody, ut: f64) -> (V3, f64) {
    let Some(sun) = b.parent() else { return (v3(1.0, 0.0, 0.0), 0.0) };
    let el = orbit::Elements {
        sma: b.orbit_sma,
        ecc: b.orbit_eccentricity,
        inc_deg: b.orbit_inclination_deg,
        lan_deg: b.orbit_lan_deg,
        arg_pe_deg: b.orbit_arg_pe_deg,
        mean_anomaly_at_epoch: b.orbit_mean_anomaly_at_epoch,
        epoch: b.orbit_epoch,
    };
    let (r, _) = el.state_at(sun.gm(), ut);
    let d = -r.normalized();
    // true anomaly: angle from periapsis direction (LAN + argPe for these equatorial orbits)
    let ta = r.y.atan2(r.x) - (el.lan_deg + el.arg_pe_deg).to_radians();
    (rot_z(d, -rotation_angle(b, ut)), ta.to_degrees())
}

/// `FlightIntegrator.SmoothVelocity` (float ring buffer with one-tick spike hold).
#[inline]
fn smooth_velocity(st: &mut EnvState, vel: V3, dt: f64) -> V3 {
    let vf = [vel.x as f32, vel.y as f32, vel.z as f32];
    let prev_i = if st.smooth_idx == 0 { VEL_SMOOTH_LEN - 1 } else { st.smooth_idx as usize - 1 };
    let prev = st.smooth_buf[prev_i];
    let d2 = (vf[0] - prev[0]).powi(2) + (vf[1] - prev[1]).powi(2) + (vf[2] - prev[2]).powi(2);
    let n2 = |a: [f32; 3]| a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    let idx = st.smooth_idx as usize;
    if (d2 as f64) > MAX_VEL_DELTA_SQR as f64 * dt * dt && n2(vf) > n2(prev) {
        if st.vel_spiking {
            st.smooth_buf[idx] = vf;
        } else if st.last_vel_provisional {
            st.smooth_buf[idx] = vf;
            st.smooth_buf[st.last_vel_idx as usize] = st.last_vel;
            st.vel_spiking = true;
        } else {
            st.last_vel_provisional = true;
            st.last_vel = vf;
            st.last_vel_idx = idx as u8;
            st.smooth_buf[idx] = prev;
        }
    } else {
        st.smooth_buf[idx] = vf;
        st.vel_spiking = false;
        st.last_vel_provisional = false;
    }
    st.smooth_idx = ((idx + 1) % VEL_SMOOTH_LEN) as u8;
    let mut s = [0.0f64; 3];
    for b in &st.smooth_buf {
        s[0] += b[0] as f64;
        s[1] += b[1] as f64;
        s[2] += b[2] as f64;
    }
    let r = 1.0 / VEL_SMOOTH_LEN as f64;
    v3(s[0] * r, s[1] * r, s[2] * r)
}

/// Fill the smoother as if the vessel had flown at `vel` for 10+ ticks (warm start on reset).
pub fn prime_smoother(st: &mut EnvState, vel: V3) {
    for b in st.smooth_buf.iter_mut() {
        *b = [vel.x as f32, vel.y as f32, vel.z as f32];
    }
    st.smooth_idx = 0;
    st.vel_spiking = false;
    st.last_vel_provisional = false;
}

#[inline(always)]
fn mat_vec(m: &[[f64; 3]; 3], v: V3) -> V3 {
    v3(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

fn inv3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let [a, b, c] = m[0];
    let [d, e, f] = m[1];
    let [g, h, i] = m[2];
    let co = [e * i - f * h, -(d * i - f * g), d * h - e * g];
    let det = a * co[0] + b * co[1] + c * co[2];
    let r = 1.0 / det;
    [
        [co[0] * r, -(b * i - c * h) * r, (b * f - c * e) * r],
        [co[1] * r, (a * i - c * g) * r, -(a * f - c * d) * r],
        [co[2] * r, -(a * h - b * g) * r, (a * e - b * d) * r],
    ]
}

const MAX_CONTACTS: usize = 24;
/// Fraction of the penetration-recovery velocity pen/dt that PhysX's contact solve (50 position
/// iterations, 1 velocity iteration) leaves in the body. Fitted to the in-game hard stop of trace
/// HopperSuicide v2 (Swivel bell 12.4 cm into the pad deck -> +3.3 m/s contact rebound).
pub const DEPEN_VELOCITY_FRACTION: f64 = 0.55;

#[derive(Clone, Copy, Debug, Default)]
struct Contact {
    /// contact point relative to the CoM, ground normal, signed gap (m, positive = above ground)
    r: V3,
    n: V3,
    gap: f64,
    crash_tol: f64,
    mu: f64,
    /// hull point index, owning part, and whether the point is on the part's primary collider
    hull: u8,
    part: u8,
    crash: bool,
}

/// Part-collider friction coefficient on the ground at `p` (see terrain.rs).
#[inline]
pub fn ground_friction(prm: &Params, p: V3) -> f64 {
    match (prm.terrain, crate::terrain::ksc()) {
        (true, Some(t)) => t.friction_at(p),
        _ => 0.6,
    }
}

/// Outward ground normal at body-fixed point `p`.
#[inline]
pub fn ground_normal(prm: &Params, p: V3) -> V3 {
    ground_normal_sc(prm, &Scenery::default(), p)
}

#[inline]
pub fn ground_normal_sc(prm: &Params, sc: &Scenery, p: V3) -> V3 {
    let p = p - sc.shift;
    match (prm.terrain, crate::terrain::ksc()) {
        (true, Some(t)) => t.normal_at_k(p, sc.ksc_dz),
        _ => p.normalized(),
    }
}

/// Per-part aerodynamic result of `FlightIntegrator.UpdateAerodynamics` (cube model).
#[derive(Clone, Copy, Debug, Default)]
pub struct PartAero {
    /// part.dragScalar (kN) along -dir, dir = unit part velocity relative to the air
    pub drag: f64,
    pub dir: V3,
    /// body lift force (world, kN), already projected perpendicular to dir
    pub lift: V3,
    /// part dynamic pressure (kPa)
    pub q: f64,
}

/// Drag and body lift of part `i` moving at `vpart` (air-relative, world) with root attitude `q`.
#[inline]
pub fn part_aero(spec: &VesselSpec, i: usize, q: Quat, vpart: V3, atm: &atmo::AtmState, mt: &crate::aero::MachTerms) -> PartAero {
    let s2 = vpart.norm2();
    if s2 <= 0.0 {
        return PartAero::default();
    }
    let qp = atm.rho * 0.0005 * s2;
    let vhat = vpart / s2.sqrt();
    let prot = q * spec.part_rot[i];
    let d = prot.conj().rotate(vhat);
    // DragCubeList.AddSurfaceDragDirection runs in float; the face normals are the +-axes, so each
    // face dot product is a signed component of the local flow direction
    let f = &spec.part_faces[i];
    let lcd = &spec.part_face_ln_cd32[i];
    let comp = [d.x as f32, d.y as f32, d.z as f32];
    let (tip, surf, tail, mult, cdp) = (mt.tip as f32, mt.surf as f32, mt.tail as f32, mt.mult as f32, mt.cd_pow as f32);
    let body_lift = spec.part_body_lift[i] != 0.0;
    let (mut area_drag, mut area) = (0.0f32, 0.0f32);
    let mut lift_l = [0.0f32; 3];
    for j in 0..6 {
        let dot = if j & 1 == 0 { comp[j >> 1] } else { -comp[j >> 1] };
        let dot01 = (dot + 1.0) * 0.5;
        let sc = if dot01 <= 0.5 { (tail + (surf - tail) * (2.0 * dot01)) * mult } else { (surf + (tip - surf) * (2.0 * dot01 - 1.0)) * mult };
        let fa = f.area[j] as f32;
        let a = fa * sc;
        area += a;
        let l = lcd[j];
        let cd = if l.is_nan() { f.drag[j] as f32 } else { (cdp * l).exp() };
        area_drag += a * cd;
        if dot > 0.0 && body_lift {
            let lv = dot * fa * f.drag[j] as f32 * crate::aero::body_lift_curve32(dot);
            let sgn = if j & 1 == 0 { -1.0 } else { 1.0 };
            lift_l[j >> 1] += sgn * lv;
        }
    }
    if area <= 0.0 {
        area_drag = 0.0;
    }
    let drag = qp * area_drag as f64 * DRAG_CUBE_MULTIPLIER * atm.pseudo_re_drag_mult * DRAG_MULTIPLIER;
    let mut lift = V3::ZERO;
    if body_lift {
        let bl = qp * spec.part_body_lift[i] * BODY_LIFT_MULTIPLIER * mt.body_lift_mach;
        let l = prot.rotate(v3(lift_l[0] as f64, lift_l[1] as f64, lift_l[2] as f64) * bl);
        lift = l - vhat * l.dot(vhat);
    }
    PartAero { drag, dir: vhat, lift, q: qp }
}

/// Diagnostic outputs of one tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct TickInfo {
    pub alt: f64,
    pub atm: atmo::AtmState,
    pub thrust: f64,
    pub mass: f64,
    pub legs_in_contact: u32,
    pub v_srf: V3,
}

/// Advance one env by one fixed physics tick.
pub fn tick(spec: &VesselSpec, prm: &Params, st: &mut EnvState, ctl: &Control) -> TickInfo {
    let dt = FIXED_DT;
    let b = prm.body;
    let omega = spin(b);
    let rot_frame = st.frame == Frame::Rotating;
    let mut info = TickInfo::default();
    if st.status == STATUS_CRASHED {
        return info;
    }

    // 1. mass stats / CoM
    let (m, com_l, i_l, i_own_l, m_parts) = spec.mass_props_parts(&st.res);
    info.mass = m;
    let r_com = st.q.rotate(com_l);
    let x_com = st.x + r_com;
    let v_com = st.v + st.w.cross(r_com);
    // surface-relative velocity
    let v_srf = if rot_frame { v_com } else { v_com - omega.cross(x_com) };
    info.v_srf = v_srf;

    // 2. smoothing, altitude, atmosphere (vessel-level, at the CoM)
    let v_smooth = smooth_velocity(st, v_com, dt);
    let spd = v_smooth.norm();
    let alt = x_com.norm() - b.radius;
    info.alt = alt;
    // FlightIntegrator.CalculateSunBodyFlux runs from Update() (once per rendered frame), not per
    // FixedUpdate; we refresh it every TEMP_OFFSET_TICKS ticks (0.5 s), far below its rate of change.
    if prm.sun_temperature && b.atmosphere && alt < b.atmosphere_depth {
        if st.ticks % TEMP_OFFSET_TICKS == 0 || st.toff.is_nan() {
            let (sun, ta) = sun_dir_fixed(b, st.ut);
            let up_f = if rot_frame { x_com } else { rot_z(x_com, -rotation_angle(b, st.ut)) }.normalized();
            st.toff = atmo::temperature_offset(b, sun, up_f, v3(0.0, 0.0, 1.0), ta, None) as f32;
        }
    } else {
        st.toff = 0.0;
    }
    let toff = if prm.sun_temperature { st.toff as f64 } else { 0.0 };
    let atm = atmo::atm_state(b, alt, spd, toff);
    info.atm = atm;

    let mut force = V3::ZERO; // kN, frame coords (non-gravitational)
    let mut torque = V3::ZERO; // kN m about CoM

    // 3a. engines + gimbal
    for k in 0..spec.n_engines as usize {
        let e = &spec.engines[k];
        let req = ctl.throttle.clamp(0.0, 1.0);
        let mut cur = engine::update_throttle(e, st.thr[k], req, dt);
        let pos_w = st.q.rotate(e.pos);
        let alt_e = (st.x + pos_w).norm() - b.radius;
        let p_atm = atmo::pressure_kpa(b, alt_e) * KPA_TO_ATM;
        let (thrust, out, c2) = engine::burn(e, cur, p_atm, &mut st.res, dt);
        cur = c2;
        st.thr[k] = cur;
        if out {
            st.flameout |= 1 << k;
        } else {
            st.flameout &= !(1 << k);
        }
        // gimbal (ModuleGimbal.GimbalRotation): pitch / yaw deflection = input * range, sign
        // flipped when the engine sits above the CoM (`s`); roll only for off-axis engines.
        let sgn = if com_l.y > e.pos.y { 1.0 } else { -1.0 };
        let tgt = [sgn * ctl.pitch.clamp(-1.0, 1.0) * e.gimbal_range_deg, sgn * ctl.yaw.clamp(-1.0, 1.0) * e.gimbal_range_deg];
        for a in 0..2 {
            st.gimbal[k][a] = if e.gimbal_response_speed > 0.0 {
                let t = (e.gimbal_response_speed * dt).clamp(0.0, 1.0);
                st.gimbal[k][a] + (tgt[a] - st.gimbal[k][a]) * t
            } else {
                tgt[a]
            };
        }
        // measured in game (trace Ascent80): ModuleGimbal.actuation = (pitch, roll, yaw); pitch
        // tilts the thrust toward root +Z by actuation.x * range, yaw toward the sim's root +X
        // (Unity -X) by actuation.z * range.
        let gq = Quat::angle_axis(st.gimbal[k][0], v3(1.0, 0.0, 0.0)) * Quat::angle_axis(-st.gimbal[k][1], v3(0.0, 0.0, 1.0));
        let dir_w = st.q.rotate(gq.rotate(e.dir));
        // the thrust transform is a child of the gimbal transform: it swings about the pivot
        // (measured: torque ratio 0.867 vs a fixed application point, trace HopperPadLand v6)
        let tp_w = st.q.rotate(e.pivot + gq.rotate(e.pos - e.pivot));
        let f = dir_w * thrust;
        force += f;
        torque += (tp_w - r_com).cross(f);
        info.thrust += thrust;
    }

    // 3b. reaction wheel (ModuleReactionWheel.ActiveUpdate, pilot mode)
    if spec.rw_ec_rate > 0.0 || spec.rw_torque.norm2() > 0.0 {
        let inp = v3(ctl.pitch.clamp(-1.0, 1.0), ctl.roll.clamp(-1.0, 1.0), ctl.yaw.clamp(-1.0, 1.0));
        let sum = inp.x.abs() + inp.y.abs() + inp.z.abs();
        let tq = v3(inp.x * spec.rw_torque.x, inp.y * spec.rw_torque.y, inp.z * spec.rw_torque.z);
        let t = (spec.rw_response * dt).clamp(0.0, 1.0);
        st.rw_in = st.rw_in + (tq - st.rw_in) * t;
        let ec_req = spec.rw_ec_rate * sum * dt;
        let ec = &mut st.res[crate::parts::Res::ElectricCharge as usize];
        let got = ec_req.min(*ec);
        *ec -= got;
        let operational = sum > 0.0 && got >= 0.9 * ec_req;
        if operational {
            // rw_in is Unity's inputVector (pitch, roll, yaw) on the part's Unity-local (x, y, z);
            // the game applies torque = rotation * -inputVector in that left-handed frame. Torque
            // is a pseudovector, so in the sim's x-mirrored frame it becomes (-x, +y, +z).
            torque += st.q.rotate(v3(-st.rw_in.x, st.rw_in.y, st.rw_in.z));
        }
    }

    // 4. gravity + frame accelerations at the CoM (uniform acceleration, no torque)
    let r2 = x_com.norm2();
    let g = x_com * (-b.gm() / (r2 * r2.sqrt()));
    let a_frame = if rot_frame { -(omega.cross(v_com) * 2.0) - omega.cross(omega.cross(x_com)) } else { V3::ZERO };

    // 5. aerodynamics per part + angular drag
    // PhysX damps each part rigidbody's angular velocity (rb.angularDrag); for the rigid composite
    // that removes sum_i c_i I_own_i w of angular momentum per unit time (root-frame matrix)
    let mut damp_l = [[0.0f64; 3]; 3];
    if prm.aero && atm.rho > 0.0 {
        let mt = mach_terms(atm.mach);
        for i in 0..spec.n_parts as usize {
            let pc = st.q.rotate(spec.part_com[i]);
            let vpart = v_com + st.w.cross(pc - r_com);
            let vpart = if rot_frame { vpart } else { vpart - omega.cross(st.x + pc) };
            let s2 = vpart.norm2();
            if s2 <= 0.0 {
                continue;
            }
            let qp = atm.rho * 0.0005 * s2; // part dynamic pressure, kPa
            // FlightIntegrator.UpdateAerodynamics: rb.angularDrag = angularDrag * q_atm * multiplier
            let ci = (spec.part_angular_drag[i] * (qp * KPA_TO_ATM) * crate::aero::ANGULAR_DRAG_MULTIPLIER).max(0.0);
            let mi = m_parts[i];
            for a in 0..3 {
                for bb in 0..3 {
                    damp_l[a][bb] += ci * mi * spec.part_inertia_root[i][a][bb];
                }
            }
            let pa = part_aero(spec, i, st.q, vpart, &atm, &mt);
            let fd = pa.dir * (-pa.drag);
            force += fd + pa.lift;
            // ApplyAeroDrag at TransformPoint(CoPOffset), ApplyAeroLift at TransformPoint(CoLOffset)
            torque += (st.q.rotate(spec.part_cop[i]) - r_com).cross(fd);
            torque += (st.q.rotate(spec.part_col[i]) - r_com).cross(pa.lift);
        }
    }

    // inverse inertia applied in the root frame (cheaper than building world-frame tensors)
    let q0 = st.q;
    let il_inv = inv3(&i_l);
    let iinv_w = |v: V3| q0.rotate(mat_vec(&il_inv, q0.conj().rotate(v)));


    // 6. ground contact (body-fixed ground; only meaningful in the rotating frame)
    let mut legs = 0u32;
    let mut contact_any = false;
    let mut contacts = [Contact::default(); MAX_CONTACTS];
    let mut nc = 0usize;
    // per-leg ground force this tick (suspension + tire), for the joint-load estimate
    let mut leg_force = [V3::ZERO; MAX_LEGS];
    // exact early-out: every contact point lies within bound_radius of the root origin, and the
    // surveyed relief within that radius is < 5 m, so nothing can touch when the root is higher
    let sc = Scenery { shift: st.ground_shift, ksc_dz: st.ksc_dz };
    let margin = spec.bound_radius + 10.0 + v_com.norm() * dt;
    // exact early-out: above the highest ground point (plus any scenery offset) nothing is near
    let above_all = st.x.norm() - sc.shift.norm() - sc.ksc_dz.max(0.0) - ground_ceiling(prm) >= margin;
    let near_ground = prm.ground && rot_frame && !above_all && ground_height_sc(prm, &sc, st.x) < margin;
    if !near_ground {
        st.hull_touch = 0;
        for ls in st.legs[..spec.n_legs as usize].iter_mut() {
            ls.grounded = false;
            ls.x = 0.0;
            ls.hit_force = 0.0;
            ls.tire = [0.0, 0.0];
        }
    }
    if near_ground {
        let g_mag = g.norm();
        let n_grounded = (0..spec.n_legs as usize).filter(|&k| st.legs[k].grounded).count() as u32;
        let inp = wheel::SuspInputs {
            vessel_mass: m,
            landed: n_grounded > 0,
            n_grounded,
            gee_asl: b.gee_asl,
            max_surface_gee_asl: MAX_SURFACE_GEE_ASL,
            g_mag,
            dt,
        };
        for k in 0..spec.n_legs as usize {
            let l = &spec.legs[k];
            let ls = &mut st.legs[k];
            // ModuleWheelSuspension / VPWheelCollider (from last step's WheelHit)
            wheel::update_spring(l, ls, &inp);
            // Unity WheelCollider ray (PhysX step) from the current pose
            let a = st.x + st.q.rotate(l.anchor);
            let u = st.q.rotate(l.up);
            let len = l.travel + l.radius;
            if let Some((s_hit, p, n)) = ground_ray_sc(prm, &sc, a, u, len) {
                let ndu = n.dot(u);
                // VPP tire force from the previous step's load, at the current contact
                if ls.grounded {
                    let vf = v_com + st.w.cross(p - x_com);
                    let vt = vf - n * vf.dot(n);
                    let fw = st.q.rotate(l.fwd);
                    let fwd = (fw - n * fw.dot(n)).normalized();
                    let side = n.cross(fwd);
                    let vloc = [vt.dot(side) as f32, vt.dot(fwd) as f32];
                    let ca = (u.dot(side)).atan2(ndu);
                    let load = ((ls.hit_force as f64) * ca.cos()).max(l.min_down_force) as f32;
                    let mfac = (ls.hit_force.max(0.0) as f64 / g_mag) as f32;
                    let fext = [-0.5 * mfac * vloc[0] / dt as f32, -0.5 * mfac * vloc[1] / dt as f32];
                    let f = wheel::tire_force(vloc, fext, load);
                    let sm = (10.0 * dt as f32).clamp(0.0, 1.0);
                    ls.tire = [ls.tire[0] + (f[0] - ls.tire[0]) * sm, ls.tire[1] + (f[1] - ls.tire[1]) * sm];
                    let ft = side * ls.tire[0] as f64 + fwd * ls.tire[1] as f64;
                    force += ft;
                    torque += (p - x_com).cross(ft);
                    leg_force[k] += ft;
                } else {
                    ls.tire = [0.0, 0.0];
                }
                // native suspension force (PhysX): jounce from full extension
                let x = len - s_hit;
                let x_prev = if ls.grounded { ls.x as f64 } else { 0.0 };
                let fs = ls.k as f64 * (x - ls.target as f64 * l.travel) + ls.c as f64 * (x - x_prev) / dt;
                let reported = fs * ndu;
                let fc = n * reported;
                force += fc;
                torque += (p - x_com).cross(fc);
                leg_force[k] += fc;
                let vn = (v_com + st.w.cross(p - x_com)).dot(n);
                st.impact_speed = st.impact_speed.max((-vn) as f32);
                ls.grounded = true;
                ls.x = x as f32;
                ls.hit_force = reported as f32;
                legs += 1;
                contact_any = true;
            } else {
                ls.grounded = false;
                ls.x = 0.0;
                ls.hit_force = 0.0;
                ls.tire = [0.0, 0.0];
            }
        }
        // rigid contacts (part colliders) for the PhysX solver below
        // PhysX generates contacts for shapes within contactOffset (Unity default 0.01 m) at the
        // start of the step; deeper penetration is recovered over later steps
        let reach = 0.01;
        for k in 0..spec.n_hull as usize {
            let p = st.x + st.q.rotate(spec.hull[k]);
            let gap = ground_height_sc(prm, &sc, p);
            if gap < reach && nc < MAX_CONTACTS {
                contacts[nc] = Contact {
                    r: p - x_com,
                    n: ground_normal_sc(prm, &sc, p),
                    gap,
                    crash_tol: spec.hull_tolerance[k],
                    mu: ground_friction(prm, p - sc.shift),
                    hull: k as u8,
                    part: spec.hull_part[k],
                    crash: spec.hull_crash[k],
                };
                nc += 1;
            }
        }
    }
    info.legs_in_contact = legs;

    // 7. integrate
    let a_ng = force / m;
    let vacuum = atm.p_kpa <= 0.0 && (!b.atmosphere || alt > b.atmosphere_depth) && !contact_any;
    if vacuum {
        st.drift_frames = st.drift_frames.saturating_add(1);
    } else {
        st.drift_frames = 0;
    }
    let (x_com_n, v_com_n) = if prm.drift_compensation && vacuum && st.drift_frames > ORBIT_DRIFT_FRAMES_TO_WAIT {
        // CoM follows the Kepler conic; non-gravity forces added on top through Euler
        let (xi, vi) = if rot_frame { fixed_to_inertial(b, st.ut, x_com, v_com) } else { (x_com, v_com) };
        let el = orbit::elements_from_state(xi, vi, b.gm(), st.ut);
        let (xr, vr) = el.state_at(b.gm(), st.ut + dt);
        let (xr, vr) = if rot_frame { inertial_to_fixed(b, st.ut + dt, xr, vr) } else { (xr, vr) };
        (xr + a_ng * (dt * dt), vr + a_ng * dt)
    } else {
        let a = a_ng + g + a_frame;
        let vn = v_com + a * dt;
        (x_com + vn * dt, vn)
    };
    // rotation: rigid composite of PhysX bodies. PhysX integrates each body without its own
    // gyroscopic term, but the joints enforce the parallel-axis inertia, whose gyroscopic torque
    // -w x (I_orb w) does act; then per-body angular damping, clamp, orientation.
    let mut i_orb_l = i_l;
    for a in 0..3 {
        for c in 0..3 {
            i_orb_l[a][c] -= i_own_l[a][c];
        }
    }
    let wl = q0.conj().rotate(st.w);
    let gyro_l = wl.cross(mat_vec(&i_orb_l, wl));
    let alpha_l = mat_vec(&il_inv, q0.conj().rotate(torque) - gyro_l);
    let mut wl2 = wl + alpha_l * dt;
    wl2 = wl2 - mat_vec(&il_inv, mat_vec(&damp_l, wl2)) * dt;
    let mut w = q0.rotate(wl2);
    // PhysX contact solve (velocity level, sequential impulses). Measured in game (SimHarness
    // collider dump): part colliders and KSC colliders carry no PhysicMaterial (Unity default:
    // friction 0.6, bounciness 0, Average), PQS terrain "Ground" (0.9, combine Maximum);
    // bounceThreshold 0.2, contactOffset 0.01, maxDepenetrationVelocity unlimited (part rbs).
    // Contacts are speculative (v_n >= -gap/dt, gap < 0 = penetration); penetration recovery
    // leaves DEPEN_VELOCITY_FRACTION * pen/dt in the body (maxDepenetrationVelocity is unlimited
    // for part rigidbodies). A part breaks off (vessel lost) when the contact impulse per tick
    // exceeds the breakForce of its joint (dumped: 4400 kN engine/tank, 4000 kN legs).
    let (v_com_n, x_com_n) = if nc > 0 {
        let mut v = v_com_n;
        // Part.OnCollisionEnter -> CheckCollision -> HandleCollision: a new contact on a part's
        // primary collider destroys the part if |relativeVelocity| (the part rigidbody's
        // velocity against the static ground, before the solve) exceeds its crashTolerance
        let mut touch = 0u32;
        for c in &contacts[..nc] {
            if c.gap <= 0.01 {
                contact_any = true;
                let vn = (v + w.cross(c.r)).dot(c.n);
                st.impact_speed = st.impact_speed.max((-vn) as f32);
                touch |= 1 << c.hull;
                if prm.crash && c.crash && st.hull_touch & (1 << c.hull) == 0 {
                    let rp = st.q.rotate(spec.part_com[c.part as usize] - com_l);
                    if (v + w.cross(rp)).norm() > c.crash_tol {
                        st.status = STATUS_CRASHED;
                    }
                }
            }
        }
        st.hull_touch = touch;
        let mut lam_n = [0.0f64; MAX_CONTACTS];
        let mut lam_t = [V3::ZERO; MAX_CONTACTS];
        // patch warm start: contacts of one part with (nearly) the same normal form a patch
        // (PhysX groups them per shape pair); solve one constraint at the patch centroid and
        // share its impulse equally, so a flat, evenly loaded patch takes an even impulse instead
        // of the order-dependent split Gauss-Seidel finds for redundant coplanar contacts
        let mut done = [false; MAX_CONTACTS];
        for a in 0..nc {
            if done[a] {
                continue;
            }
            let mut idx = [0usize; MAX_CONTACTS];
            let mut cnt = 0;
            for b in a..nc {
                if !done[b] && contacts[b].part == contacts[a].part && contacts[b].n.dot(contacts[a].n) > 0.999 {
                    idx[cnt] = b;
                    cnt += 1;
                    done[b] = true;
                }
            }
            if cnt < 2 {
                continue;
            }
            let inv = 1.0 / cnt as f64;
            let (mut rb, mut nb, mut tb) = (V3::ZERO, V3::ZERO, 0.0);
            for &b in &idx[..cnt] {
                let c = &contacts[b];
                rb += c.r * inv;
                nb += c.n * inv;
                tb += inv * if c.gap >= 0.0 { -c.gap / dt } else { -c.gap / dt * DEPEN_VELOCITY_FRACTION };
            }
            let nb = nb.normalized();
            let vn = (v + w.cross(rb)).dot(nb);
            let rn = rb.cross(nb);
            let kn = 1.0 / m + rn.dot(iinv_w(rn));
            let lam = ((tb - vn) / kn).max(0.0);
            for &b in &idx[..cnt] {
                let c = &contacts[b];
                let dl = lam * inv;
                lam_n[b] += dl;
                v += c.n * (dl / m);
                w += iinv_w(c.r.cross(c.n)) * dl;
            }
        }
        for _ in 0..8 {
            for (ci, c) in contacts[..nc].iter().enumerate() {
                let target = if c.gap >= 0.0 { -c.gap / dt } else { -c.gap / dt * DEPEN_VELOCITY_FRACTION };
                let vp = v + w.cross(c.r);
                let vn = vp.dot(c.n);
                let rn = c.r.cross(c.n);
                let k_n = 1.0 / m + rn.dot(iinv_w(rn));
                let dl = (target - vn) / k_n;
                let new = (lam_n[ci] + dl).max(0.0);
                let dj = new - lam_n[ci];
                lam_n[ci] = new;
                v += c.n * (dj / m);
                w += iinv_w(rn * dj);
                let vp = v + w.cross(c.r);
                let vt = vp - c.n * vp.dot(c.n);
                let vtn = vt.norm();
                if vtn > 1e-12 {
                    let tdir = vt / vtn;
                    let rt = c.r.cross(tdir);
                    let k_t = 1.0 / m + rt.dot(iinv_w(rt));
                    let want = lam_t[ci] - tdir * (vtn / k_t);
                    let cap = c.mu * lam_n[ci];
                    let lt = if want.norm() > cap { want.normalized() * cap } else { want };
                    let dj = lt - lam_t[ci];
                    lam_t[ci] = lt;
                    v += dj / m;
                    w += iinv_w(c.r.cross(dj));
                }
            }
        }
        // joint breakForce: the load through the joint of each non-root part P, from the rigid
        // composite: J_joint = (m_S / m) J_total - J_S, S = P's subtree, J = this tick's
        // contact impulses + leg ground forces * dt (thrust and aero are spread by mass, i.e.
        // add no joint load; error <= (m_S/m) * thrust * dt). Breaks when |J_joint| / dt >
        // breakForce (dumped: 4400 kN engine / tank).
        if prm.crash && prm.joint_break {
            let mut j_part = [V3::ZERO; crate::vessel::MAX_PARTS];
            for (ci, c) in contacts[..nc].iter().enumerate() {
                j_part[c.part as usize] += c.n * lam_n[ci] + lam_t[ci];
            }
            for k in 0..spec.n_legs as usize {
                j_part[spec.leg_part[k] as usize] += leg_force[k] * dt;
            }
            let j_tot = j_part.iter().fold(V3::ZERO, |a, &b| a + b);
            for p in 1..spec.n_parts as usize {
                let sub = spec.part_subtree[p];
                let (mut m_s, mut j_s) = (0.0, V3::ZERO);
                for i in 0..spec.n_parts as usize {
                    if sub & (1 << i) != 0 {
                        m_s += m_parts[i];
                        j_s += j_part[i];
                    }
                }
                if (j_tot * (m_s / m) - j_s).norm() / dt > spec.part_break_force[p] {
                    st.status = STATUS_CRASHED;
                }
            }
        }
        let dv = v - v_com_n;
        (v, x_com_n + dv * dt)
    } else {
        st.hull_touch = 0;
        (v_com_n, x_com_n)
    };
    let wn = w.norm();
    if wn > MAX_ANGULAR_VELOCITY {
        w = w * (MAX_ANGULAR_VELOCITY / wn);
    }
    let q = st.q.integrate_world(w, dt);
    let r_com_n = q.rotate(com_l);
    st.x = x_com_n - r_com_n;
    st.v = v_com_n - w.cross(r_com_n);
    st.q = q;
    st.w = w;
    st.ut += dt;
    st.ticks += 1;

    // contact / landed bookkeeping
    if st.status != STATUS_CRASHED {
        if legs == spec.n_legs && legs > 0 {
            let still = v_srf.norm() < 0.3 && st.w.norm() < 0.1;
            st.settled_ticks = if still { st.settled_ticks.saturating_add(1) } else { 0 };
            st.status = if st.settled_ticks >= 25 { STATUS_LANDED } else { STATUS_CONTACT };
        } else {
            st.settled_ticks = 0;
            st.status = if contact_any { STATUS_CONTACT } else { STATUS_FLYING };
        }
    }

    // 8. frame switch at inverseRotThresholdAltitude (clamped to >= atmosphereDepth)
    let thresh = if b.clamp_inverse_rot_threshold { b.inverse_rot_threshold_altitude.max(b.atmosphere_depth) } else { b.inverse_rot_threshold_altitude };
    let alt_n = (st.x + r_com_n).norm() - b.radius;
    if rot_frame && alt_n > thresh {
        // setRotatingFrame(false): v += w x r; positions untouched in the (momentarily coincident) frame
        let a = rotation_angle(b, st.ut);
        st.v = rot_z(st.v + omega.cross(st.x), a);
        st.x = rot_z(st.x, a);
        st.q = Quat::angle_axis(a.to_degrees(), v3(0.0, 0.0, 1.0)) * st.q;
        st.w = rot_z(st.w, a);
        st.frame = Frame::Inertial;
        rebase_smoother(st, omega, a, true);
    } else if !rot_frame && alt_n < thresh {
        let a = rotation_angle(b, st.ut);
        st.x = rot_z(st.x, -a);
        st.v = rot_z(st.v, -a) - omega.cross(st.x);
        st.q = Quat::angle_axis(-a.to_degrees(), v3(0.0, 0.0, 1.0)) * st.q;
        st.w = rot_z(st.w, -a);
        st.frame = Frame::Rotating;
        rebase_smoother(st, omega, a, false);
    }
    info
}

/// KSP keeps the smoothing buffer in world coordinates across a frame switch, which is a
/// velocity jump the spike filter then rides out; we re-express it so the switch is seamless
/// for the smoothed speed (a deliberate, tiny deviation: it only affects drag above 70 km = 0).
fn rebase_smoother(st: &mut EnvState, _omega: V3, a: f64, to_inertial: bool) {
    let ang = if to_inertial { a } else { -a };
    for b in st.smooth_buf.iter_mut() {
        let r = rot_z(v3(b[0] as f64, b[1] as f64, b[2] as f64), ang);
        *b = [r.x as f32, r.y as f32, r.z as f32];
    }
}

// ---------------------------------------------------------------------------------------------
// Pad-local frame helpers
// ---------------------------------------------------------------------------------------------

/// East / north / up unit vectors at the pad (body-fixed frame).
pub fn pad_enu() -> (V3, V3, V3) {
    let up = ksc::KSC_PAD_DECK_BODY_FIXED.normalized();
    let east = v3(0.0, 0.0, 1.0).cross(up).normalized();
    let north = up.cross(east);
    (east, north, up)
}

/// Rotation taking root-local axes to an attitude whose +Y is `up_dir` (body-fixed), with +Z
/// pointing as close to `fwd_hint` as possible.
pub fn attitude(up_dir: V3, fwd_hint: V3) -> Quat {
    let y = up_dir.normalized();
    let z = (fwd_hint - y * fwd_hint.dot(y)).normalized();
    let x = y.cross(z);
    // rotation matrix columns x, y, z -> quaternion
    let (m00, m11, m22) = (x.x, y.y, z.z);
    let tr = m00 + m11 + m22;
    let (m01, m02, m10, m12, m20, m21) = (y.x, z.x, x.y, z.y, x.z, y.z);
    if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        Quat::new((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quat::new(0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quat::new((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quat::new((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    }
    .normalized()
}

/// Initial state with the vessel's CoM at pad-ENU offset `enu` (m, up measured from the deck),
/// surface velocity `v_enu`, attitude `q` (root-local -> body-fixed), full tanks.
pub fn state_at_pad(spec: &VesselSpec, enu: V3, v_enu: V3, q: Quat, w: V3, ut: f64) -> EnvState {
    let (e, n, u) = pad_enu();
    let x_com = ksc::KSC_PAD_DECK_BODY_FIXED + e * enu.x + n * enu.y + u * enu.z;
    let v_com = e * v_enu.x + n * v_enu.y + u * v_enu.z;
    let res = spec.full_resources();
    let (_, com_l, _) = spec.mass_props(&res);
    let r = q.rotate(com_l);
    let mut st = EnvState { ut, x: x_com - r, v: v_com - w.cross(r), q, w, res, toff: f32::NAN, legs: [LegState::fresh(); MAX_LEGS], ..Default::default() };
    prime_smoother(&mut st, v_com);
    st
}

/// A state resting on the pad with legs at their static compression (for tests / hover starts).
pub fn state_landed(spec: &VesselSpec, ut: f64) -> EnvState {
    let (e, n, u) = pad_enu();
    let q = attitude(u, n);
    let res = spec.full_resources();
    let (_, com_l, _) = spec.mass_props(&res);
    // feet just touching; the legs then settle to their static compression
    let h = com_l.y - spec.bottom_y - 0.02;
    let _ = e;
    let mut st = state_at_pad(spec, v3(0.0, 0.0, h), V3::ZERO, q, V3::ZERO, ut);
    st.status = STATUS_CONTACT;
    let _ = n;
    st
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vessel::Craft;

    fn hopper() -> VesselSpec {
        VesselSpec::compile(&Craft::hopper())
    }

    #[test]
    fn free_fall_in_vacuum_matches_kepler() {
        // above the atmosphere, no thrust: drift compensation pins the CoM to the conic
        let spec = hopper();
        let prm = Params::default();
        let r0 = KERBIN.radius + 300_000.0;
        let vc = (KERBIN.gm() / r0).sqrt();
        let mut st = EnvState { frame: Frame::Inertial, x: v3(r0, 0.0, 0.0), v: v3(0.0, vc, 0.0), q: Quat::IDENTITY, res: spec.full_resources(), ..Default::default() };
        let (_, com_l, _) = spec.mass_props(&st.res);
        st.x = st.x - com_l;
        let el = orbit::elements_from_state(st.x + com_l, st.v, KERBIN.gm(), 0.0);
        let ctl = Control::default();
        for _ in 0..5000 {
            tick(&spec, &prm, &mut st, &ctl);
        }
        let (xr, _) = el.state_at(KERBIN.gm(), st.ut);
        let err = (st.x + st.q.rotate(com_l) - xr).norm();
        assert!(err < 0.5, "drift {err} m after 100 s");
        assert_eq!(st.frame, Frame::Inertial);
    }

    #[test]
    fn sits_on_pad() {
        let spec = hopper();
        let prm = Params::default();
        let mut st = state_landed(&spec, 0.0);
        let ctl = Control::default();
        for _ in 0..500 {
            tick(&spec, &prm, &mut st, &ctl);
        }
        assert_eq!(st.status, STATUS_LANDED, "{st:?}");
        let (e, n, u) = pad_enu();
        let d = st.x - ksc::KSC_PAD_DECK_BODY_FIXED;
        assert!(d.dot(e).abs() < 0.05 && d.dot(n).abs() < 0.05, "slid {} {}", d.dot(e), d.dot(n));
        assert!(st.v.norm() < 0.05);
        let _ = u;
    }

    #[test]
    fn hover_needs_thrust_equal_weight() {
        // throttle at weight/thrust_asl keeps a hover roughly level
        let spec = hopper();
        let prm = Params { aero: true, ..Default::default() };
        let (_, _, u) = pad_enu();
        let q = attitude(u, v3(0.0, 0.0, 1.0));
        let mut st = state_at_pad(&spec, v3(0.0, 0.0, 200.0), V3::ZERO, q, V3::ZERO, 0.0);
        let (m, _, _) = spec.mass_props(&st.res);
        let g = KERBIN.gm() / (KERBIN.radius + 270.0f64).powi(2);
        let thr_asl = 215.0 * engine::isp(&spec.engines[0], 0.99) / 320.0;
        let ctl = Control { throttle: m * g / thr_asl, ..Default::default() };
        let h0 = (st.x.norm() - KERBIN.radius) as f64;
        for _ in 0..50 {
            tick(&spec, &prm, &mut st, &ctl);
        }
        let h1 = st.x.norm() - KERBIN.radius;
        assert!((h1 - h0).abs() < 0.5, "{h0} -> {h1}");
    }

    #[test]
    fn frame_switch_preserves_inertial_state() {
        let b = KERBIN;
        let x = v3(400_000.0, 500_000.0, 10_000.0);
        let v = v3(-100.0, 50.0, 3.0);
        let (xi, vi) = fixed_to_inertial(b, 1234.5, x, v);
        let (xf, vf) = inertial_to_fixed(b, 1234.5, xi, vi);
        assert!((xf - x).norm() < 1e-6 && (vf - v).norm() < 1e-9);
    }

    #[test]
    fn coriolis_deflects_drop() {
        // rotating-frame physics == inertial physics: a dropped body in the body-fixed frame lands
        // east of the release point (classic Coriolis), check vs an inertial integration
        let prm = Params { aero: false, ground: false, ..Default::default() };
        let spec = hopper();
        let (_, _, u) = pad_enu();
        let q = attitude(u, v3(0.0, 0.0, 1.0));
        let mut st = state_at_pad(&spec, v3(0.0, 0.0, 5000.0), V3::ZERO, q, V3::ZERO, 0.0);
        let (_, com_l, _) = spec.mass_props(&st.res);
        let x0 = st.x + st.q.rotate(com_l);
        let (mut xi, mut vi) = fixed_to_inertial(KERBIN, 0.0, x0, V3::ZERO);
        let ctl = Control::default();
        let mut t = 0.0;
        for _ in 0..1000 {
            tick(&spec, &prm, &mut st, &ctl);
            let a = xi * (-KERBIN.gm() / xi.norm().powi(3));
            vi += a * FIXED_DT;
            xi += vi * FIXED_DT;
            t += FIXED_DT;
        }
        let xr = st.x + st.q.rotate(com_l);
        let (xf, _) = inertial_to_fixed(KERBIN, t, xi, vi);
        assert!((xr - xf).norm() < 0.05, "{}", (xr - xf).norm());
    }
}
