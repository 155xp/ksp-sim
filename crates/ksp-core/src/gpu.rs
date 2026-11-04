//! Flat packing of the vessel spec, the world constants and the per-env state for the CUDA tick
//! kernel (python/ksp_sim/cuda_tick.cu). The kernel reads everything through index macros
//! generated here (`header()`), so the Rust layout is the single source of truth.

use crate::aero;
use crate::body::{RawBody, IDEAL_GAS_CONSTANT};
use crate::curve::Key;
use crate::sim::{EnvState, Params, MAX_SURFACE_GEE_ASL};
use crate::vessel::{VesselSpec, MAX_ENGINES, MAX_HULL, MAX_ISP_KEYS, MAX_LEGS, MAX_PARTS};
use crate::wheel::LegState;
use std::fmt::Write;

/// Per-leg state fields (all stored as f64 in the state buffer).
pub const LEG_FIELDS: usize = 18;
/// State vector layout (field index; the device buffer is field-major [field][env]).
pub mod st {
    pub const UT: usize = 0;
    pub const X: usize = 1;
    pub const V: usize = 4;
    pub const Q: usize = 7;
    pub const W: usize = 11;
    pub const RES: usize = 14;
    pub const THR: usize = 18; // MAX_ENGINES
    pub const GIMBAL: usize = 22; // MAX_ENGINES x 2
    pub const FLAMEOUT: usize = 30;
    pub const RW: usize = 31;
    pub const SMOOTH: usize = 34; // 30
    pub const SMOOTH_IDX: usize = 64;
    pub const SPIKING: usize = 65;
    pub const PROVISIONAL: usize = 66;
    pub const LAST_VEL: usize = 67;
    pub const LAST_VEL_IDX: usize = 70;
    pub const DRIFT: usize = 71;
    pub const STATUS: usize = 72;
    pub const SETTLED: usize = 73;
    pub const TICKS: usize = 74;
    pub const IMPACT: usize = 75;
    pub const TOFF: usize = 76;
    pub const FRAME: usize = 77;
    pub const LEGS: usize = 78;
    pub const GSHIFT: usize = LEGS + super::MAX_LEGS * super::LEG_FIELDS;
    pub const KSCDZ: usize = GSHIFT + 3;
}
pub const STATE_FIELDS: usize = st::GSHIFT + 4;

pub fn pack_state(s: &EnvState, out: &mut [f64]) {
    out[st::UT] = s.ut;
    out[st::X..st::X + 3].copy_from_slice(&[s.x.x, s.x.y, s.x.z]);
    out[st::V..st::V + 3].copy_from_slice(&[s.v.x, s.v.y, s.v.z]);
    out[st::Q..st::Q + 4].copy_from_slice(&[s.q.x, s.q.y, s.q.z, s.q.w]);
    out[st::W..st::W + 3].copy_from_slice(&[s.w.x, s.w.y, s.w.z]);
    out[st::RES..st::RES + 4].copy_from_slice(&s.res);
    for k in 0..MAX_ENGINES {
        out[st::THR + k] = s.thr[k];
        out[st::GIMBAL + 2 * k] = s.gimbal[k][0];
        out[st::GIMBAL + 2 * k + 1] = s.gimbal[k][1];
    }
    out[st::FLAMEOUT] = s.flameout as f64;
    out[st::RW..st::RW + 3].copy_from_slice(&[s.rw_in.x, s.rw_in.y, s.rw_in.z]);
    for j in 0..10 {
        for c in 0..3 {
            out[st::SMOOTH + 3 * j + c] = s.smooth_buf[j][c] as f64;
        }
    }
    out[st::SMOOTH_IDX] = s.smooth_idx as f64;
    out[st::SPIKING] = s.vel_spiking as u8 as f64;
    out[st::PROVISIONAL] = s.last_vel_provisional as u8 as f64;
    for c in 0..3 {
        out[st::LAST_VEL + c] = s.last_vel[c] as f64;
    }
    out[st::LAST_VEL_IDX] = s.last_vel_idx as f64;
    out[st::DRIFT] = s.drift_frames as f64;
    out[st::STATUS] = s.status as f64;
    out[st::SETTLED] = s.settled_ticks as f64;
    out[st::TICKS] = s.ticks as f64;
    out[st::IMPACT] = s.impact_speed as f64;
    out[st::TOFF] = s.toff as f64;
    out[st::FRAME] = s.frame as u8 as f64;
    for k in 0..MAX_LEGS {
        let l = &s.legs[k];
        let b = st::LEGS + k * LEG_FIELDS;
        let v = [
            l.grounded as u8 as f32, l.x, l.hit_force, l.k, l.c, l.target, l.m_spring, l.m_damper, l.stw, l.dtw,
            l.auto_boost, l.auto_set as u8 as f32, l.rate, l.drate, l.rt, l.rtd, l.tire[0], l.tire[1],
        ];
        for (j, x) in v.iter().enumerate() {
            out[b + j] = *x as f64;
        }
    }
    out[st::GSHIFT..st::GSHIFT + 3].copy_from_slice(&[s.ground_shift.x, s.ground_shift.y, s.ground_shift.z]);
    out[st::KSCDZ] = s.ksc_dz;
}

pub fn unpack_state(a: &[f64]) -> EnvState {
    use crate::math::{v3, Quat};
    let mut s = EnvState {
        ut: a[st::UT],
        x: v3(a[st::X], a[st::X + 1], a[st::X + 2]),
        v: v3(a[st::V], a[st::V + 1], a[st::V + 2]),
        q: Quat::new(a[st::Q], a[st::Q + 1], a[st::Q + 2], a[st::Q + 3]),
        w: v3(a[st::W], a[st::W + 1], a[st::W + 2]),
        ..Default::default()
    };
    s.res.copy_from_slice(&a[st::RES..st::RES + 4]);
    for k in 0..MAX_ENGINES {
        s.thr[k] = a[st::THR + k];
        s.gimbal[k] = [a[st::GIMBAL + 2 * k], a[st::GIMBAL + 2 * k + 1]];
    }
    s.flameout = a[st::FLAMEOUT] as u8;
    s.rw_in = v3(a[st::RW], a[st::RW + 1], a[st::RW + 2]);
    for j in 0..10 {
        for c in 0..3 {
            s.smooth_buf[j][c] = a[st::SMOOTH + 3 * j + c] as f32;
        }
    }
    s.smooth_idx = a[st::SMOOTH_IDX] as u8;
    s.vel_spiking = a[st::SPIKING] != 0.0;
    s.last_vel_provisional = a[st::PROVISIONAL] != 0.0;
    for c in 0..3 {
        s.last_vel[c] = a[st::LAST_VEL + c] as f32;
    }
    s.last_vel_idx = a[st::LAST_VEL_IDX] as u8;
    s.drift_frames = a[st::DRIFT] as u8;
    s.status = a[st::STATUS] as u8;
    s.settled_ticks = a[st::SETTLED] as u16;
    s.ticks = a[st::TICKS] as u32;
    s.impact_speed = a[st::IMPACT] as f32;
    s.toff = a[st::TOFF] as f32;
    s.frame = if a[st::FRAME] != 0.0 { crate::sim::Frame::Inertial } else { crate::sim::Frame::Rotating };
    for k in 0..MAX_LEGS {
        let b = st::LEGS + k * LEG_FIELDS;
        let g = |j: usize| a[b + j] as f32;
        s.legs[k] = LegState {
            grounded: g(0) != 0.0, x: g(1), hit_force: g(2), k: g(3), c: g(4), target: g(5), m_spring: g(6), m_damper: g(7),
            stw: g(8), dtw: g(9), auto_boost: g(10), auto_set: g(11) != 0.0, rate: g(12), drate: g(13), rt: g(14), rtd: g(15),
            tire: [g(16), g(17)],
        };
    }
    s.ground_shift = v3(a[st::GSHIFT], a[st::GSHIFT + 1], a[st::GSHIFT + 2]);
    s.ksc_dz = a[st::KSCDZ];
    s
}

/// Packed constants: one f64 array + the C header naming every offset.
pub struct Pack {
    pub data: Vec<f64>,
    pub header: String,
}

struct Packer {
    data: Vec<f64>,
    header: String,
}

impl Packer {
    fn scalar(&mut self, name: &str, v: f64) {
        writeln!(self.header, "#define P_{name} {}", self.data.len()).unwrap();
        self.data.push(v);
    }
    fn array(&mut self, name: &str, v: &[f64]) {
        writeln!(self.header, "#define P_{name} {}", self.data.len()).unwrap();
        self.data.extend_from_slice(v);
    }
    fn curve(&mut self, name: &str, keys: &[Key]) {
        writeln!(self.header, "#define P_{name}_N {}", keys.len()).unwrap();
        let v: Vec<f64> = keys.iter().flat_map(|k| [k.t, k.v, k.tin, k.tout]).collect();
        self.array(name, &v);
    }
}

/// Pack spec + world constants. Ground terrain grids are passed separately (`terrain_arrays`).
const MB_STRIDE: usize = 22;

pub fn pack(spec: &VesselSpec, prm: &Params) -> Pack {
    let b: &RawBody = prm.body;
    let mut p = Packer { data: vec![], header: String::new() };
    writeln!(p.header, "#define MAX_PARTS {MAX_PARTS}\n#define MAX_ENGINES {MAX_ENGINES}\n#define MAX_LEGS {MAX_LEGS}\n#define MAX_HULL {MAX_HULL}\n#define MAX_ISP_KEYS {MAX_ISP_KEYS}\n#define LEG_FIELDS {LEG_FIELDS}\n#define STATE_FIELDS {STATE_FIELDS}").unwrap();
    for (n, v) in [("UT", st::UT), ("X", st::X), ("V", st::V), ("Q", st::Q), ("W", st::W), ("RES", st::RES), ("THR", st::THR), ("GIMBAL", st::GIMBAL),
        ("FLAMEOUT", st::FLAMEOUT), ("RW", st::RW), ("SMOOTH", st::SMOOTH), ("SMOOTH_IDX", st::SMOOTH_IDX), ("SPIKING", st::SPIKING),
        ("PROVISIONAL", st::PROVISIONAL), ("LAST_VEL", st::LAST_VEL), ("LAST_VEL_IDX", st::LAST_VEL_IDX), ("DRIFT", st::DRIFT),
        ("STATUS", st::STATUS), ("SETTLED", st::SETTLED), ("TICKS", st::TICKS), ("IMPACT", st::IMPACT), ("TOFF", st::TOFF), ("FRAME", st::FRAME), ("LEGS", st::LEGS), ("GSHIFT", st::GSHIFT), ("KSCDZ", st::KSCDZ)] {
        writeln!(p.header, "#define S_{n} {v}").unwrap();
    }
    // world
    p.scalar("GM", b.gm());
    p.scalar("RADIUS", b.radius);
    p.scalar("OMEGA", b.angular_velocity());
    p.scalar("GEE_ASL", b.gee_asl);
    p.scalar("MAX_SURF_GEE", MAX_SURFACE_GEE_ASL);
    p.scalar("ATM_DEPTH", b.atmosphere_depth);
    p.scalar("MOLAR", b.atm_molar_mass);
    p.scalar("GAMMA", b.atm_adiabatic_index);
    p.scalar("RGAS", IDEAL_GAS_CONSTANT);
    p.scalar("INIT_ROT_DEG", b.initial_rotation_deg);
    p.scalar("SIDEREAL", b.sidereal_period());
    p.scalar("ORB_SMA", b.orbit_sma);
    p.scalar("ORB_MA0", b.orbit_mean_anomaly_at_epoch);
    p.scalar("ORB_EPOCH", b.orbit_epoch);
    p.scalar("ORB_PE_LON_DEG", b.orbit_lan_deg + b.orbit_arg_pe_deg);
    p.scalar("ORB_N", (b.parent().map(|s| s.gm()).unwrap_or(1.0) / b.orbit_sma.powi(3)).sqrt());
    p.scalar("GROUND_ALT", prm.ground_alt);
    p.scalar("SUN_TEMP", prm.sun_temperature as u8 as f64);
    p.scalar("AERO", prm.aero as u8 as f64);
    p.scalar("GROUND", prm.ground as u8 as f64);
    p.scalar("CRASH", prm.crash as u8 as f64);
    p.curve("PRESSURE", b.pressure_curve);
    p.curve("TEMPERATURE", b.temperature_curve);
    p.curve("SUNMULT", b.atmosphere_temperature_sun_mult_curve);
    p.curve("LATBIAS", b.latitude_temperature_bias_curve);
    p.curve("LATSUN", b.latitude_temperature_sun_mult_curve);
    p.curve("AXBIAS", b.axial_temperature_sun_bias_curve);
    p.curve("AXMULT", b.axial_temperature_sun_mult_curve);
    p.curve("TIP", aero::DRAG_TIP);
    p.curve("SURF", aero::DRAG_SURFACE);
    p.curve("TAIL", aero::DRAG_TAIL);
    p.curve("MULT", aero::DRAG_MACH_MULT);
    p.curve("CDPOW", aero::DRAG_CD_POWER);
    p.curve("PRE", aero::DRAG_PSEUDOREYNOLDS);
    p.curve("BLIFT", aero::BODY_LIFT_CURVE);
    p.curve("BLMACH", aero::BODY_LIFT_MACH);
    // vessel
    let s = spec;
    p.scalar("N_PARTS", s.n_parts as f64);
    let flat3 = |v: &[crate::math::V3]| -> Vec<f64> { v.iter().flat_map(|x| [x.x, x.y, x.z]).collect() };
    p.array("DRY", &s.part_dry_mass);
    p.array("RESCAP", &s.part_res_cap.iter().flatten().cloned().collect::<Vec<_>>());
    p.array("PCOM", &flat3(&s.part_com));
    p.array("PCOP", &flat3(&s.part_cop));
    p.array("PCOL", &flat3(&s.part_col));
    p.array("PROT", &s.part_rot.iter().flat_map(|q| [q.x, q.y, q.z, q.w]).collect::<Vec<_>>());
    p.array("PINERT", &s.part_inertia_root.iter().flat_map(|m| m.iter().flatten().cloned().collect::<Vec<_>>()).collect::<Vec<_>>());
    p.array("FAREA", &s.part_faces.iter().flat_map(|f| f.area).collect::<Vec<_>>());
    p.array("FDRAG", &s.part_faces.iter().flat_map(|f| f.drag).collect::<Vec<_>>());
    p.array("FLNCD", &s.part_face_ln_cd32.iter().flat_map(|f| f.map(|x| x as f64)).collect::<Vec<_>>());
    p.array("BODYLIFT", &s.part_body_lift);
    p.array("ANGDRAG", &s.part_angular_drag);
    p.array("RESCAPTOT", &s.res_cap_total);
    p.array("RESDENS", &crate::parts::RES_DENSITY);
    // Mass properties are linear in the resource amounts: basis 0 = dry parts, basis 1+r = per unit
    // of resource r. Each basis holds [mass, first moment (3), second moment about the root origin
    // (9, own inertia + parallel axis), own inertia only (9)], so the kernel builds m, CoM and the
    // inertia from 5 small sums instead of a per-part loop (same algebra, rounding-level change).
    let mut mb = vec![0.0; 5 * MB_STRIDE];
    for i in 0..s.n_parts as usize {
        let pc = s.part_com[i];
        let pv = [pc.x, pc.y, pc.z];
        for k in 0..5 {
            let w = if k == 0 {
                s.part_dry_mass[i]
            } else if s.res_cap_total[k - 1] > 0.0 {
                s.part_res_cap[i][k - 1] / s.res_cap_total[k - 1] * crate::parts::RES_DENSITY[k - 1]
            } else {
                0.0
            };
            let o = &mut mb[k * MB_STRIDE..(k + 1) * MB_STRIDE];
            o[0] += w;
            for a in 0..3 {
                o[1 + a] += w * pv[a];
            }
            let d2 = pv[0] * pv[0] + pv[1] * pv[1] + pv[2] * pv[2];
            for a in 0..3 {
                for b in 0..3 {
                    let ir = s.part_inertia_root[i][a][b];
                    o[4 + 3 * a + b] += w * ir + w * ((if a == b { d2 } else { 0.0 }) - pv[a] * pv[b]);
                    o[13 + 3 * a + b] += w * ir;
                }
            }
        }
    }
    p.array("MBASIS", &mb);
    writeln!(p.header, "#define MB_STRIDE {MB_STRIDE}").unwrap();
    p.scalar("N_ENG", s.n_engines as f64);
    for k in 0..MAX_ENGINES {
        let e = &s.engines[k];
        let mut v = vec![e.max_fuel_flow, e.min_fuel_flow, e.n_isp_keys as f64, e.ignition_threshold, e.accel_speed, e.decel_speed,
            e.use_response as u8 as f64, e.gimbal_range_deg, e.gimbal_response_speed, e.pos.x, e.pos.y, e.pos.z, e.dir.x, e.dir.y, e.dir.z,
            e.pivot.x, e.pivot.y, e.pivot.z];
        v.extend_from_slice(&e.units_per_ton);
        for kk in e.isp_keys {
            v.extend_from_slice(&[kk.t, kk.v, kk.tin, kk.tout]);
        }
        p.array(&format!("ENG{k}"), &v);
    }
    writeln!(p.header, "#define ENG_STRIDE {}", 22 + 4 * MAX_ISP_KEYS).unwrap();
    p.scalar("N_LEGS", s.n_legs as f64);
    for k in 0..MAX_LEGS {
        let l = &s.legs[k];
        p.array(&format!("LEG{k}"), &[l.anchor.x, l.anchor.y, l.anchor.z, l.up.x, l.up.y, l.up.z, l.fwd.x, l.fwd.y, l.fwd.z, l.radius, l.travel,
            l.travel_cfg, l.spring_ratio, l.damper_ratio, l.boost_ratio, l.maximum_load, l.sprung_mass, l.crash_tolerance, l.min_down_force]);
    }
    writeln!(p.header, "#define LEG_STRIDE 19").unwrap();
    p.scalar("N_HULL", s.n_hull as f64);
    p.array("HULL", &flat3(&s.hull));
    p.array("HULLTOL", &s.hull_tolerance);
    p.array("HULLBF", &s.hull_break_force.map(|x| if x.is_finite() { x } else { 1e300 }));
    p.array("RWTORQUE", &[s.rw_torque.x, s.rw_torque.y, s.rw_torque.z]);
    p.scalar("RWEC", s.rw_ec_rate);
    p.scalar("RWRESP", s.rw_response);
    p.scalar("BOUND", s.bound_radius);
    p.scalar("GROUND_CEIL", crate::sim::ground_ceiling(prm));
    Pack { data: p.data, header: p.header }
}

/// Terrain grids for the device: (meta [origin, east, north, up, radius, fallback, ngrids, (step, half, n, offset)...], alt, ground).
pub fn terrain_arrays() -> Option<(Vec<f64>, Vec<f32>, Vec<u8>)> {
    let t = crate::terrain::ksc()?;
    let mut meta = vec![t.origin.x, t.origin.y, t.origin.z, t.east.x, t.east.y, t.east.z, t.north.x, t.north.y, t.north.z, t.up.x, t.up.y, t.up.z, t.radius, t.fallback_alt, t.grids.len() as f64];
    let (mut alt, mut gr) = (vec![], vec![]);
    for g in &t.grids {
        meta.extend_from_slice(&[g.step, g.half, g.n as f64, alt.len() as f64]);
        alt.extend_from_slice(&g.alt);
        gr.extend_from_slice(&g.ground);
    }
    Some((meta, alt, gr))
}
