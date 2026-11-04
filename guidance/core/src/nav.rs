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
/// frame). Mirrors ksp-core sim.rs step 5 (per-part cube drag + body lift) term by term.
pub fn aero_force(spec: &VesselSpec, com_l: V3, q: Quat, v: V3, w: V3, atm: &AtmState) -> (V3, V3) {
    let mut force = V3::ZERO;
    let mut torque = V3::ZERO;
    if atm.rho <= 0.0 {
        return (force, torque);
    }
    let r_com = q.rotate(com_l);
    let mt = mach_terms(atm.mach);
    for i in 0..spec.n_parts as usize {
        let pc = q.rotate(spec.part_com[i]);
        let vpart = v + w.cross(pc - r_com);
        let s2 = vpart.norm2();
        if s2 <= 0.0 {
            continue;
        }
        let qp = atm.rho * 0.0005 * s2;
        let vhat = vpart / s2.sqrt();
        let prot = q * spec.part_rot[i];
        let d = prot.conj().rotate(vhat);
        let f = &spec.part_faces[i];
        let (mut area_drag, mut area, mut lift_l) = (0.0, 0.0, V3::ZERO);
        for j in 0..6 {
            let dot = d.dot(FACE_DIRS[j]);
            let dot01 = (dot + 1.0) * 0.5;
            let s = if dot01 <= 0.5 {
                (mt.tail + (mt.surf - mt.tail) * (2.0 * dot01)) * mt.mult
            } else {
                (mt.surf + (mt.tip - mt.surf) * (2.0 * dot01 - 1.0)) * mt.mult
            };
            let a = f.area[j] * s;
            area += a;
            if a == 0.0 {
                continue;
            }
            let lcd = spec.part_face_ln_cd[i][j];
            let cd = if lcd.is_nan() { f.drag[j] } else { (mt.cd_pow * lcd).exp() };
            area_drag += a * cd;
            if dot > 0.0 && spec.part_body_lift[i] != 0.0 {
                let l = dot * f.area[j] * f.drag[j] * (eval(BODY_LIFT_CURVE, dot as f32 as f64) as f32 as f64);
                lift_l -= FACE_DIRS[j] * l;
            }
        }
        if area <= 0.0 {
            area_drag = 0.0;
        }
        let drag = qp * area_drag * DRAG_CUBE_MULTIPLIER * atm.pseudo_re_drag_mult * DRAG_MULTIPLIER;
        let mut fp = vhat * (-drag);
        if spec.part_body_lift[i] != 0.0 {
            let bl = qp * spec.part_body_lift[i] * BODY_LIFT_MULTIPLIER * mt.body_lift_mach;
            let l = prot.rotate(lift_l * bl);
            fp += l - vhat * l.dot(vhat);
        }
        force += fp;
        let at = q.rotate(spec.part_cop[i]);
        torque += (at - r_com).cross(fp);
    }
    (force, torque)
}

/// Per-craft aero tables for the engine-first attitude tilted by angle of attack `alpha`
/// (between the vessel's -up axis and the airflow-relative velocity):
/// `cd[m][a]` = drag / (q * pseudoRe) (kN/kPa), `cl[m][a]` = side force along the tilt direction / q,
/// `cm[m][a]` = aero torque about the tilt axis / q (kN m / kPa, positive = increases alpha).
#[derive(Clone, Debug)]
pub struct AeroTables {
    pub mach_step: f64,
    pub alpha_step_deg: f64,
    pub n_mach: usize,
    pub n_alpha: usize,
    pub cd: Vec<f64>,
    pub cl: Vec<f64>,
    pub cm: Vec<f64>,
}

pub const TABLE_MACH_MAX: f64 = 12.0;
pub const TABLE_ALPHA_MAX_DEG: f64 = 30.0;

impl AeroTables {
    /// Tabulate from the aero model at a reference fuel state (CoM matters for the moment only).
    pub fn build(spec: &VesselSpec, res: &[f64; N_RES]) -> AeroTables {
        let (_, com_l, _) = spec.mass_props(res);
        let mach_step = 0.05;
        let alpha_step_deg = 2.0;
        let n_mach = (TABLE_MACH_MAX / mach_step) as usize + 1;
        let n_alpha = (TABLE_ALPHA_MAX_DEG / alpha_step_deg) as usize + 1;
        let mut t = AeroTables { mach_step, alpha_step_deg, n_mach, n_alpha, cd: vec![0.0; n_mach * n_alpha], cl: vec![0.0; n_mach * n_alpha], cm: vec![0.0; n_mach * n_alpha] };
        // reference flow: velocity along -Y in a frame where the vessel is rotated by alpha about
        // root X (tilt direction = root +Z) and root Z (tilt direction = root -X); averaged over
        // both roll orientations so 3- and 4-leg layouts read the same
        let rho = 1.0;
        let c = 300.0;
        for im in 0..n_mach {
            let mach = (im as f64 * mach_step).max(1e-3);
            let spd = mach * c;
            let q_kpa = 0.0005 * rho * spd * spd;
            let atm = AtmState { p_kpa: 50.0, t: 250.0, rho, c, mach, q_kpa, pseudo_re_drag_mult: 1.0 };
            for ia in 0..n_alpha {
                let a = (ia as f64 * alpha_step_deg).to_radians();
                let mut acc = (0.0, 0.0, 0.0);
                for (axis, tilt) in [(v3(1.0, 0.0, 0.0), v3(0.0, 0.0, 1.0)), (v3(0.0, 0.0, 1.0), v3(-1.0, 0.0, 0.0))] {
                    // vessel up = (0,1,0) rotated by a about `axis`; flow along -world Y.
                    let q = Quat::angle_axis(a.to_degrees(), axis);
                    let up = q.rotate(v3(0.0, 1.0, 0.0));
                    let v = v3(0.0, -spd, 0.0);
                    let (f, tq) = aero_force(spec, com_l, q, v, V3::ZERO, &atm);
                    let tdir = (up - v3(0.0, 1.0, 0.0) * up.y).normalized();
                    let tdir = if a == 0.0 { q.rotate(tilt) } else { tdir };
                    let drag = f.y; // flow along -Y: drag points +Y
                    let side = f.dot(tdir);
                    // torque about the tilt rotation axis (positive = increases alpha)
                    let mom = tq.dot(axis);
                    acc.0 += drag / q_kpa;
                    acc.1 += side / q_kpa;
                    acc.2 += mom / q_kpa;
                }
                let k = im * n_alpha + ia;
                t.cd[k] = acc.0 * 0.5;
                t.cl[k] = acc.1 * 0.5;
                t.cm[k] = acc.2 * 0.5;
            }
        }
        t
    }

    #[inline]
    fn interp(&self, tab: &[f64], mach: f64, alpha_deg: f64) -> f64 {
        let fm = (mach / self.mach_step).clamp(0.0, (self.n_mach - 1) as f64 - 1e-9);
        let fa = (alpha_deg.abs() / self.alpha_step_deg).clamp(0.0, (self.n_alpha - 1) as f64 - 1e-9);
        let (im, ia) = (fm as usize, fa as usize);
        let (tm, ta) = (fm - im as f64, fa - ia as f64);
        let at = |m: usize, a: usize| tab[m * self.n_alpha + a];
        let a0 = at(im, ia) + (at(im, ia + 1) - at(im, ia)) * ta;
        let a1 = at(im + 1, ia) + (at(im + 1, ia + 1) - at(im + 1, ia)) * ta;
        a0 + (a1 - a0) * tm
    }
    pub fn cd(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.interp(&self.cd, mach, alpha_deg)
    }
    pub fn cl(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.interp(&self.cl, mach, alpha_deg)
    }
    pub fn cm(&self, mach: f64, alpha_deg: f64) -> f64 {
        self.interp(&self.cm, mach, alpha_deg)
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
            let (f, _) = aero_force(&spec, n0.com_l, n0.q, n0.v, V3::ZERO, &atm);
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
