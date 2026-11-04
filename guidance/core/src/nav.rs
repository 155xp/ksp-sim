//! Navigation state and the craft model the guidance reads every tick.
//!
//! * `Nav`: CoM position / surface velocity / attitude in Kerbin's rotating (body-fixed) frame and
//!   the pad East-North-Up frame, whatever frame the sim state is in.
//! * `aero_force`: ksp-core's per-part drag-cube + body-lift force and torque (sim.rs step 5),
//!   re-evaluated for an arbitrary attitude / velocity so the guidance can predict and feed forward
//!   exactly what the sim will apply. `tests::aero_matches_sim_tick` checks it against a real tick.
//! * `AeroTables`: that model tabulated per craft over (mach, angle of attack) for the engine-first
//!   descent attitude, used by the point-mass predictor.
//! * `Propulsion`: thrust, Isp and mass flow at a given static pressure from the engine specs.

use ksp_core::aero::{mach_terms, BODY_LIFT_CURVE, BODY_LIFT_MULTIPLIER, DRAG_CUBE_MULTIPLIER, DRAG_MULTIPLIER, FACE_DIRS};
use ksp_core::atmo::{self, AtmState, KPA_TO_ATM};
use ksp_core::body::KERBIN;
use ksp_core::curve::eval;
use ksp_core::engine::{self, ENGINE_G0};
use ksp_core::ksc;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::{Res, N_RES, RES_DENSITY};
use ksp_core::sim::{self, EnvState, Frame};
use ksp_core::vessel::VesselSpec;

pub const OMEGA: V3 = v3(0.0, 0.0, ksc::KERBIN_OMEGA);

#[inline]
pub fn rot_z(p: V3, a: f64) -> V3 {
    let (s, c) = a.sin_cos();
    v3(c * p.x - s * p.y, s * p.x + c * p.y, p.z)
}

/// Pad frame: deck centre and East / North / Up unit vectors (body-fixed).
#[derive(Clone, Copy, Debug)]
pub struct Pad {
    pub origin: V3,
    pub e: V3,
    pub n: V3,
    pub u: V3,
}

impl Pad {
    pub fn ksc() -> Pad {
        let (e, n, u) = sim::pad_enu();
        Pad { origin: ksc::KSC_PAD_DECK_BODY_FIXED, e, n, u }
    }
    pub fn to_enu(&self, x: V3) -> V3 {
        let d = x - self.origin;
        v3(d.dot(self.e), d.dot(self.n), d.dot(self.u))
    }
    pub fn vec_to_enu(&self, v: V3) -> V3 {
        v3(v.dot(self.e), v.dot(self.n), v.dot(self.u))
    }
    pub fn vec_from_enu(&self, v: V3) -> V3 {
        self.e * v.x + self.n * v.y + self.u * v.z
    }
    /// Horizontal (great-circle-ish, local tangent) offset of body-fixed point `x` from the pad:
    /// project onto the pad tangent plane along the local vertical of `x`.
    pub fn ground_offset(&self, x: V3) -> (f64, f64) {
        // angular offset * radius, robust for long ranges
        let r = self.origin.norm();
        let xh = x.normalized();
        let oh = self.origin.normalized();
        let ang = xh.dot(oh).clamp(-1.0, 1.0).acos();
        let t = xh - oh * xh.dot(oh);
        let tn = t.norm();
        if tn < 1e-12 {
            return (0.0, 0.0);
        }
        let d = t / tn * (ang * r);
        (d.dot(self.e), d.dot(self.n))
    }
}

/// CoM kinematics and attitude in the body-fixed frame.
#[derive(Clone, Copy, Debug)]
pub struct Nav {
    pub x: V3,
    pub v: V3,
    /// root-local -> body-fixed
    pub q: Quat,
    /// angular velocity, root-local axes
    pub w_l: V3,
    pub up: V3,
    pub alt: f64,
    pub m: f64,
    pub com_l: V3,
    pub inertia_l: [[f64; 3]; 3],
    /// rotation angle (rad) from the state's frame to body-fixed (0 in the rotating frame)
    pub frame_angle: f64,
}

pub fn nav(spec: &VesselSpec, st: &EnvState) -> Nav {
    let (m, com_l, it) = spec.mass_props(&st.res);
    let r = st.q.rotate(com_l);
    let (x, v, q, a) = if st.frame == Frame::Rotating {
        (st.x + r, st.v + st.w.cross(r), st.q, 0.0)
    } else {
        let (x, v) = sim::inertial_to_fixed(KERBIN, st.ut, st.x + r, st.v + st.w.cross(r));
        let a = sim::rotation_angle(KERBIN, st.ut);
        (x, v, Quat::angle_axis(-a.to_degrees(), v3(0.0, 0.0, 1.0)) * st.q, -a)
    };
    Nav {
        x,
        v,
        q,
        w_l: st.q.conj().rotate(st.w),
        up: q.rotate(v3(0.0, 1.0, 0.0)),
        alt: x.norm() - KERBIN.radius,
        m,
        com_l,
        inertia_l: it,
        frame_angle: a,
    }
}

impl Nav {
    /// Body-fixed direction -> the sim state's frame.
    pub fn to_state_frame(&self, d: V3) -> V3 {
        rot_z(d, -self.frame_angle)
    }
}

/// Atmosphere at the CoM as the sim computes it (sun offset taken from the state's cache).
pub fn atm_at(alt: f64, spd: f64, toff: f64) -> AtmState {
    atmo::atm_state(KERBIN, alt, spd, toff)
}

/// Aerodynamic force (kN, frame of `q`) and torque about the CoM (kN m) on the vessel with
/// attitude `q`, CoM velocity `v` relative to the air (same frame), angular velocity `w` (same
/// frame), airbrake deflection `defl` (deg). Calls ksp-core sim.rs step 5's own per-part
/// (`sim::part_aero`: cube drag + body lift) and aero-surface (`sim::aero_surfaces`: airbrakes)
/// functions, so it is the sim's force by construction.
pub fn aero_force(spec: &VesselSpec, com_l: V3, q: Quat, v: V3, w: V3, atm: &AtmState, defl: f32) -> (V3, V3) {
    let mut force = V3::ZERO;
    let mut torque = V3::ZERO;
    if atm.rho <= 0.0 {
        return (force, torque);
    }
    let r_com = q.rotate(com_l);
    let mt = mach_terms(atm.mach);
    for i in 0..spec.n_parts as usize {
        if spec.part_no_cube_drag & (1 << i) != 0 {
            continue;
        }
        let pc = q.rotate(spec.part_com[i]);
        let pa = sim::part_aero(spec, i, q, v + w.cross(pc - r_com), atm, &mt);
        let fd = pa.dir * (-pa.drag);
        force += fd + pa.lift;
        torque += (q.rotate(spec.part_cop[i]) - r_com).cross(fd);
        torque += (q.rotate(spec.part_col[i]) - r_com).cross(pa.lift);
    }
    if spec.n_aero_surf > 0 {
        let st = EnvState { q, w, brake_deflection: defl, ..Default::default() };
        let (f, t) = sim::aero_surfaces(spec, &st, v, r_com, None, atm);
        force += f;
        torque += t;
    }
    (force, torque)
}

/// Fully deployed airbrake deflection of a spec (0 without airbrakes): the descent attitude the
/// predictor's tables assume (brakes go out at the end of the boostback).
pub fn deployed_deflection(spec: &VesselSpec) -> f32 {
    if spec.n_aero_surf == 0 {
        return 0.0;
    }
    let a = &spec.aero_surf[0];
    a.deploy_angle.clamp(-1.5 * a.ctrl_surface_range, 1.5 * a.ctrl_surface_range) as f32
}

/// Per-craft aero tables for the engine-first attitude tilted by angle of attack `alpha`
/// (between the vessel's up axis and -v, the airflow axis) toward body azimuth `phi` (tilt
/// direction in the root x-z plane, deg from +X toward +Z):
/// `cd` = drag / (q * pseudoRe) (kN/kPa), `cl` = side force along the tilt direction / q,
/// `cs` = side force perpendicular to the tilt plane / q (along airflow-axis x tilt direction;
/// non-zero between the box faces' symmetry azimuths),
/// `cm` = aero torque about the tilt axis / q (kN m / kPa, positive = increases alpha).
/// Drag cubes are boxes, so lift at a given alpha differs by ~25 % between body-axis and diagonal
/// tilts; `phi` captures that (and leg asymmetry).
#[derive(Clone, Debug)]
pub struct AeroTables {
    pub mach_step: f64,
    pub alpha_step_deg: f64,
    pub n_mach: usize,
    pub n_alpha: usize,
    pub n_phi: usize,
    pub cd: Vec<f64>,
    pub cl: Vec<f64>,
    pub cs: Vec<f64>,
    pub cm: Vec<f64>,
    /// axial drag at zero angle of attack on a fine mach grid (the transonic rise is steep)
    pub cd0: Vec<f64>,
}

pub const CD0_MACH_STEP: f64 = 0.005;
pub const TABLE_MACH_MAX: f64 = 12.0;
pub const TABLE_ALPHA_MAX_DEG: f64 = 30.0;
const N_PHI: usize = 24;

/// Tilt azimuth (deg) of direction `t` in the vessel's body x-z plane, using the body X axis
/// `body_x` re-orthogonalised against the thrust axis `axis`.
pub fn tilt_azimuth(body_x: V3, axis: V3, t: V3) -> f64 {
    let x = (body_x - axis * body_x.dot(axis)).normalized();
    let z = x.cross(axis);
    t.dot(z).atan2(t.dot(x)).to_degrees()
}

impl AeroTables {
    /// Tabulate from the aero model at a reference fuel state (CoM matters for the moment only).
    pub fn build(spec: &VesselSpec, res: &[f64; N_RES]) -> AeroTables {
        let (_, com_l, _) = spec.mass_props(res);
        let mach_step = 0.1;
        let alpha_step_deg = 2.0;
        let n_mach = (TABLE_MACH_MAX / mach_step) as usize + 1;
        let n_alpha = (TABLE_ALPHA_MAX_DEG / alpha_step_deg) as usize + 1;
        let sz = n_mach * n_alpha * N_PHI;
        let mut t = AeroTables { mach_step, alpha_step_deg, n_mach, n_alpha, n_phi: N_PHI, cd: vec![0.0; sz], cl: vec![0.0; sz], cs: vec![0.0; sz], cm: vec![0.0; sz], cd0: vec![] };
        let n0 = (TABLE_MACH_MAX / CD0_MACH_STEP) as usize + 1;
        for im in 0..n0 {
            let mach = (im as f64 * CD0_MACH_STEP).max(1e-3);
            let spd = mach * 300.0;
            let q_kpa = 0.0005 * spd * spd;
            let atm = AtmState { p_kpa: 50.0, t: 250.0, rho: 1.0, c: 300.0, mach, q_kpa, pseudo_re_drag_mult: 1.0 };
            let (f, _) = aero_force(spec, com_l, Quat::IDENTITY, v3(0.0, -spd, 0.0), V3::ZERO, &atm, deployed_deflection(spec));
            t.cd0.push(f.y / q_kpa);
        }
        let rho = 1.0;
        let c = 300.0;
        for im in 0..n_mach {
            let mach = (im as f64 * mach_step).max(1e-3);
            let spd = mach * c;
            let q_kpa = 0.0005 * rho * spd * spd;
            let atm = AtmState { p_kpa: 50.0, t: 250.0, rho, c, mach, q_kpa, pseudo_re_drag_mult: 1.0 };
            for ia in 0..n_alpha {
                let a = ia as f64 * alpha_step_deg;
                for ip in 0..N_PHI {
                    let phi = (ip as f64 * 360.0 / N_PHI as f64).to_radians();
                    let tb = v3(phi.cos(), 0.0, phi.sin());
                    // rotating +Y about (Y x t) by +a tilts it toward t; flow along -world Y
                    let axis = v3(0.0, 1.0, 0.0).cross(tb);
                    let q = Quat::angle_axis(a, axis);
                    let tdir = q.rotate(tb);
                    let tdir = (tdir - v3(0.0, tdir.y, 0.0)).normalized();
                    let (f, tq) = aero_force(spec, com_l, q, v3(0.0, -spd, 0.0), V3::ZERO, &atm, deployed_deflection(spec));
                    let k = (im * n_alpha + ia) * N_PHI + ip;
                    t.cd[k] = f.y / q_kpa; // flow along -Y: drag points +Y
                    t.cl[k] = f.dot(tdir) / q_kpa;
                    t.cs[k] = f.dot(v3(0.0, 1.0, 0.0).cross(tdir)) / q_kpa;
                    t.cm[k] = tq.dot(axis) / q_kpa;
                }
            }
        }
        t
    }

    #[inline]
    fn interp(&self, tab: &[f64], mach: f64, alpha_deg: f64, phi_deg: Option<f64>) -> f64 {
        let fm = (mach / self.mach_step).clamp(0.0, (self.n_mach - 1) as f64 - 1e-9);
        let fa = (alpha_deg.abs() / self.alpha_step_deg).clamp(0.0, (self.n_alpha - 1) as f64 - 1e-9);
        let (im, ia) = (fm as usize, fa as usize);
        let (tm, ta) = (fm - im as f64, fa - ia as f64);
        let np = self.n_phi;
        let at_phi = |m: usize, a: usize| -> f64 {
            let base = (m * self.n_alpha + a) * np;
            match phi_deg {
                None => tab[base..base + np].iter().sum::<f64>() / np as f64,
                Some(p) => {
                    let fp = p.rem_euclid(360.0) / (360.0 / np as f64);
                    let ip = (fp as usize) % np;
                    let tp = fp - fp.floor();
                    tab[base + ip] + (tab[base + (ip + 1) % np] - tab[base + ip]) * tp
                }
            }
        };
        let a0 = at_phi(im, ia) + (at_phi(im, ia + 1) - at_phi(im, ia)) * ta;
        let a1 = at_phi(im + 1, ia) + (at_phi(im + 1, ia + 1) - at_phi(im + 1, ia)) * ta;
        a0 + (a1 - a0) * tm
    }
    fn cd0(&self, mach: f64) -> f64 {
        let f = (mach / CD0_MACH_STEP).clamp(0.0, (self.cd0.len() - 1) as f64 - 1e-9);
        let i = f as usize;
        self.cd0[i] + (self.cd0[i + 1] - self.cd0[i]) * (f - i as f64)
    }
    /// Roll-averaged lookups (exact at alpha = 0).
    pub fn cd(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.cd_at(mach, alpha_deg, None)
    }
    pub fn cl(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.interp(&self.cl, mach, alpha_deg, None)
    }
    pub fn cm(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.interp(&self.cm, mach, alpha_deg, None)
    }
    /// Lookups at tilt azimuth `phi` (deg); `None` = roll average.
    pub fn cd_at(&self, mach: f64, alpha_deg: f64, phi: Option<f64>) -> f64 {
        if alpha_deg == 0.0 {
            return self.cd0(mach);
        }
        self.cd0(mach) + self.interp(&self.cd, mach, alpha_deg, phi) - self.interp(&self.cd, mach, 0.0, phi)
    }
    pub fn cl_at(&self, mach: f64, alpha_deg: f64, phi: Option<f64>) -> f64 {
        self.interp(&self.cl, mach, alpha_deg, phi)
    }
    pub fn cs_at(&self, mach: f64, alpha_deg: f64, phi: Option<f64>) -> f64 {
        self.interp(&self.cs, mach, alpha_deg, phi)
    }
    pub fn cm_at(&self, mach: f64, alpha_deg: f64, phi: Option<f64>) -> f64 {
        self.interp(&self.cm, mach, alpha_deg, phi)
    }
    /// Largest angle of attack (deg, <= 20) at tilt azimuth `phi` whose aero moment at dynamic
    /// pressure q stays within `torque` (kN m), interpolated between table rows.
    pub fn alpha_limit(&self, mach: f64, q_kpa: f64, torque: f64, phi: Option<f64>) -> f64 {
        let mut prev = 0.0;
        let mut a = self.alpha_step_deg;
        while a <= 20.0 + 1e-9 {
            let m = self.cm_at(mach, a, phi).abs() * q_kpa;
            if m > torque {
                let mp = self.cm_at(mach, prev, phi).abs() * q_kpa;
                return prev + (a - prev) * ((torque - mp) / (m - mp).max(1e-12)).clamp(0.0, 1.0);
            }
            prev = a;
            a += self.alpha_step_deg;
        }
        20.0
    }
}

/// Engine performance summed over the craft's engines (all share one throttle in ksp-core).
#[derive(Clone, Copy, Debug)]
pub struct Propulsion {
    /// max thrust (kN) and mass flow (t/s) at full throttle
    pub thrust: f64,
    pub mdot: f64,
    pub isp: f64,
    /// lowest non-zero throttle the engines can hold (minThrust / maxThrust)
    pub throttle_min: f64,
}

pub fn propulsion(spec: &VesselSpec, p_atm: f64) -> Propulsion {
    let (mut t, mut md, mut tmin) = (0.0, 0.0, 0.0f64);
    for k in 0..spec.n_engines as usize {
        let e = &spec.engines[k];
        let isp = engine::isp(e, p_atm);
        md += e.max_fuel_flow;
        t += e.max_fuel_flow * isp * ENGINE_G0;
        if e.max_fuel_flow > 0.0 {
            tmin = tmin.max(e.min_fuel_flow / e.max_fuel_flow);
        }
    }
    Propulsion { thrust: t, mdot: md, isp: if md > 0.0 { t / (md * ENGINE_G0) } else { 0.0 }, throttle_min: tmin }
}

/// Pressure (atm) at altitude.
pub fn p_atm(alt: f64) -> f64 {
    atmo::pressure_kpa(KERBIN, alt) * KPA_TO_ATM
}

/// Propellant mass (t) in the pool and the craft's propellant capacity (t).
pub fn propellant(spec: &VesselSpec, res: &[f64; N_RES]) -> (f64, f64) {
    let (lf, ox) = (Res::LiquidFuel as usize, Res::Oxidizer as usize);
    (res[lf] * RES_DENSITY[lf] + res[ox] * RES_DENSITY[ox], spec.res_cap_total[lf] * RES_DENSITY[lf] + spec.res_cap_total[ox] * RES_DENSITY[ox])
}

/// Usable propellant: the engines stop when either propellant runs out (mixture ratio).
pub fn usable_propellant(spec: &VesselSpec, res: &[f64; N_RES]) -> f64 {
    if spec.n_engines == 0 {
        return 0.0;
    }
    let e = &spec.engines[0];
    let mut t = f64::INFINITY;
    for r in 0..N_RES {
        if e.units_per_ton[r] > 0.0 {
            t = t.min(res[r] / e.units_per_ton[r]);
        }
    }
    if t.is_finite() { t } else { 0.0 }
}

/// CoM height above the ground at leg touchdown (upright, legs fully extended).
pub fn touchdown_height(spec: &VesselSpec, com_l: V3) -> f64 {
    com_l.y - spec.bottom_y
}

#[cfg(test)]
mod tests {
    use super::*;
    use ksp_core::sim::{Control, Params};
    use ksp_core::vessel::Craft;

    /// The replicated aero force equals the velocity change the sim applies (aero on - off).
    #[test]
    fn aero_matches_sim_tick() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let pad = Pad::ksc();
        let prm = Params { ground: false, sun_temperature: false, ..Params::default() };
        let tilt = (pad.u * 0.9 + pad.e * 0.3).normalized();
        let q = sim::attitude(tilt, pad.n);
        for vel in [v3(10.0, -5.0, -150.0), v3(-80.0, 30.0, -400.0)] {
            let mut a = sim::state_at_pad(&spec, v3(0.0, 0.0, 2000.0), vel, q, V3::ZERO, 0.0);
            let mut b = a;
            let n0 = nav(&spec, &a);
            let atm = atm_at(n0.alt, n0.v.norm(), 0.0);
            let (f, _) = aero_force(&spec, n0.com_l, n0.q, n0.v, V3::ZERO, &atm, a.brake_deflection);
            sim::tick(&spec, &prm, &mut a, &Control::default());
            sim::tick(&spec, &Params { aero: false, ..prm }, &mut b, &Control::default());
            let dv = (nav(&spec, &a).v - nav(&spec, &b).v) / sim::FIXED_DT * n0.m;
            assert!((dv - f).norm() < 1e-6 * f.norm().max(1.0), "{dv:?} vs {f:?}");
        }
    }

    #[test]
    fn tables_hopper_axial_drag() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let t = AeroTables::build(&spec, &spec.full_resources());
        // engine-first axial CdA at mach 0.3 ~ 0.72 m^2 (VALIDATION.md, /DRAG_PSEUDOREYNOLDS ~1)
        let cda = t.cd(0.295, 0.0);
        assert!((cda - 0.72).abs() < 0.05, "{cda}");
        // tilting produces a side force
        assert!(t.cl(0.5, 10.0).abs() > 1e-3);
    }
}

#[cfg(test)]
mod table_vs_direct {
    use super::*;
    use ksp_core::vessel::Craft;

    /// Table model force == direct cube force for arbitrary attitudes (roll azimuth included).
    #[test]
    fn table_matches_direct_force() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let res = spec.full_resources();
        let t = AeroTables::build(&spec, &res);
        let (_, com_l, _) = spec.mass_props(&res);
        let v = v3(0.3, -1.0, 0.2).normalized() * 340.0;
        let vh = v.normalized();
        let atm = AtmState { p_kpa: 50.0, t: 250.0, rho: 0.6, c: 300.0, mach: 340.0 / 300.0, q_kpa: 0.0005 * 0.6 * 340.0 * 340.0, pseudo_re_drag_mult: 1.0 };
        let mut worst: f64 = 0.0;
        for k in 0..12 {
            let az = k as f64 * 30.0 + 7.0;
            let a = 8.0f64;
            // a tilt direction perpendicular to the flow axis, then the attitude
            let ref1 = v3(1.0, 0.0, 0.0);
            let e1 = (ref1 - (-vh) * ref1.dot(-vh)).normalized();
            let e2 = (-vh).cross(e1);
            let tp = e1 * az.to_radians().cos() + e2 * az.to_radians().sin();
            let up = -vh * a.to_radians().cos() + tp * a.to_radians().sin();
            let q = ksp_core::sim::attitude(up, v3(0.0, 0.0, 1.0));
            let (f, _) = aero_force(&spec, com_l, q, v, V3::ZERO, &atm, deployed_deflection(&spec));
            let phi = tilt_azimuth(q.rotate(v3(1.0, 0.0, 0.0)), -vh, tp);
            let fm = -vh * (atm.q_kpa * t.cd_at(atm.mach, a, Some(phi))) + tp * (atm.q_kpa * t.cl_at(atm.mach, a, Some(phi))) + (-vh).cross(tp) * (atm.q_kpa * t.cs_at(atm.mach, a, Some(phi)));
            let lat_d = f - vh * f.dot(vh);
            let lat_m = fm - vh * fm.dot(vh);
            let e = (lat_d - lat_m).norm() / lat_d.norm().max(1e-9);
            println!("az {az:5.1} phi {phi:7.1}: lateral direct ({:.2} along t, {:.2} side) model ({:.2}, {:.2})  rel err {e:.3}", lat_d.dot(tp), lat_d.dot((-vh).cross(tp)), lat_m.dot(tp), lat_m.dot((-vh).cross(tp)));
            worst = worst.max(e);
        }
        assert!(worst < 0.05, "{worst}");
    }
}
