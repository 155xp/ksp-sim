//! A craft compiled into the flat, fixed-size spec the batched kernel reads.
//!
//! Modelling choice (documented deviation): KSP simulates every part as its own PhysX rigidbody
//! joined by joints; a stiff, well-strutted lander behaves as one rigid body, so the sim treats
//! the vessel as a single rigid body while still computing every force per part (aero per part
//! at its CoP, thrust at each thrust transform, ground contact per leg). Mass and CoM are
//! recomputed every tick from dry + resource masses (FlightIntegrator.UpdateMassStats).
//! Per-part inertia is approximated as a uniform box of the part's drag-cube size (Unity derives
//! it from the colliders, which live in the model files).

use crate::aero::{facing_area, occlude, AeroFaces};
use crate::curve::Key;
use crate::math::{v3, Quat, V3};
use crate::parts::{PartDef, N_RES, RES_DENSITY};

pub const MAX_PARTS: usize = 16;
pub const MAX_ENGINES: usize = 4;
pub const MAX_LEGS: usize = 6;
pub const MAX_HULL: usize = 12;
pub const MAX_ISP_KEYS: usize = 8;

/// A part placed in the craft's root frame (root part origin, root orientation).
#[derive(Clone, Debug)]
pub struct Placed {
    pub def: PartDef,
    pub pos: V3,
    pub rot: Quat,
    /// stack attachments: (index of neighbour, this part's node direction, part-local)
    pub stack: Vec<(usize, V3)>,
}

#[derive(Clone, Debug, Default)]
pub struct Craft {
    pub parts: Vec<Placed>,
}

impl Craft {
    /// Stack `def` below part `parent` (parent bottom node <-> child top node).
    pub fn stack_below(&mut self, parent: usize, def: PartDef) -> usize {
        let p = &self.parts[parent];
        let (pb, pbd) = p.def.node_bottom.expect("parent has no bottom node");
        let (ct, ctd) = def.node_top.expect("child has no top node");
        let pos = p.pos + p.rot.rotate(pb) - ct;
        let i = self.parts.len();
        let prot = p.rot;
        self.parts[parent].stack.push((i, pbd));
        self.parts.push(Placed { def, pos, rot: prot, stack: vec![(parent, ctd)] });
        i
    }

    /// Surface-attach a leg-like part to `parent` at azimuth `az_deg` (about +Y), radius `r`,
    /// height `y` (root frame). Part +Z points into the parent, +Y along root +Y.
    pub fn surface_attach(&mut self, parent: usize, def: PartDef, az_deg: f64, r: f64, y: f64) -> usize {
        let a = az_deg.to_radians();
        let out = v3(a.cos(), 0.0, a.sin());
        let pos = self.parts[parent].pos + v3(0.0, y, 0.0) + out * r;
        // rotation taking +Z to -out, keeping +Y: angle about Y
        let rot = Quat::from_to(v3(0.0, 0.0, 1.0), -out);
        self.parts.push(Placed { def, pos, rot, stack: vec![] });
        self.parts.len() - 1
    }

    /// The default lander: Mk1 pod / FL-T400 / LV-T45 Swivel, three LT-2 legs on the tank.
    pub fn hopper() -> Craft {
        use crate::parts::*;
        let mut c = Craft { parts: vec![Placed { def: mk1pod_v2(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![] }] };
        let tank = c.stack_below(0, fuel_tank_t400());
        let _eng = c.stack_below(tank, swivel());
        for k in 0..3 {
            c.surface_attach(tank, lt2_leg(), 90.0 + 120.0 * k as f64, 0.625, -0.6);
        }
        c
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct EngineSpec {
    pub part: u32,
    pub max_fuel_flow: f64,
    pub min_fuel_flow: f64,
    pub isp_keys: [Key; MAX_ISP_KEYS],
    pub n_isp_keys: u32,
    /// unit LF/Ox/... draw per unit of propellant mass: ratio_i / mixtureDensity
    pub units_per_ton: [f64; N_RES],
    pub ignition_threshold: f64,
    pub accel_speed: f64,
    pub decel_speed: f64,
    pub use_response: bool,
    pub gimbal_range_deg: f64,
    pub gimbal_response_speed: f64,
    /// root-frame thrust point and nominal thrust direction
    pub pos: V3,
    pub dir: V3,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct LegSpec {
    /// root-frame anchor (foot at full extension) and the suspension axis (unit, root frame, "down")
    pub foot: V3,
    pub axis: V3,
    pub travel: f64,
    pub spring_ratio: f64,
    pub damper_ratio: f64,
    pub crash_tolerance: f64,
}

/// Flat, `Copy` vessel description (mirrors 1:1 into a CUDA constant buffer).
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct VesselSpec {
    pub n_parts: u32,
    pub part_dry_mass: [f64; MAX_PARTS],
    /// per-part resource capacity (units)
    pub part_res_cap: [[f64; N_RES]; MAX_PARTS],
    /// root-frame positions: part CoM (origin + CoMOffset) and CoP (origin + CoPOffset)
    pub part_com: [V3; MAX_PARTS],
    pub part_cop: [V3; MAX_PARTS],
    pub part_rot: [Quat; MAX_PARTS],
    /// diagonal box inertia per tonne (part-local), from cube size
    pub part_inertia_per_t: [V3; MAX_PARTS],
    pub part_faces: [AeroFaces; MAX_PARTS],
    /// DRAG_CD(face Cd) is mach-independent: precomputed per face (NaN marks Cd >= 1)
    pub part_face_cd_curve: [[f64; 6]; MAX_PARTS],
    pub part_body_lift: [f64; MAX_PARTS],
    pub part_angular_drag: [f64; MAX_PARTS],
    pub res_cap_total: [f64; N_RES],
    pub n_engines: u32,
    pub engines: [EngineSpec; MAX_ENGINES],
    pub n_legs: u32,
    pub legs: [LegSpec; MAX_LEGS],
    /// hull points (root frame) whose ground contact means a crash, with tolerance (m/s)
    pub n_hull: u32,
    pub hull: [V3; MAX_HULL],
    pub hull_tolerance: [f64; MAX_HULL],
    pub rw_torque: V3,
    pub rw_ec_rate: f64,
    pub rw_response: f64,
    /// lowest point (legs fully extended) below the root origin, along root -Y
    pub bottom_y: f64,
}

impl VesselSpec {
    pub fn compile(c: &Craft) -> VesselSpec {
        assert!(c.parts.len() <= MAX_PARTS);
        let mut s = VesselSpec {
            n_parts: c.parts.len() as u32,
            part_dry_mass: [0.0; MAX_PARTS],
            part_res_cap: [[0.0; N_RES]; MAX_PARTS],
            part_com: [V3::ZERO; MAX_PARTS],
            part_cop: [V3::ZERO; MAX_PARTS],
            part_rot: [Quat::IDENTITY; MAX_PARTS],
            part_inertia_per_t: [V3::ZERO; MAX_PARTS],
            part_faces: [AeroFaces::default(); MAX_PARTS],
            part_face_cd_curve: [[f64::NAN; 6]; MAX_PARTS],
            part_body_lift: [0.0; MAX_PARTS],
            part_angular_drag: [0.0; MAX_PARTS],
            res_cap_total: [0.0; N_RES],
            n_engines: 0,
            engines: [EngineSpec::default(); MAX_ENGINES],
            n_legs: 0,
            legs: [LegSpec::default(); MAX_LEGS],
            n_hull: 0,
            hull: [V3::ZERO; MAX_HULL],
            hull_tolerance: [0.0; MAX_HULL],
            rw_torque: V3::ZERO,
            rw_ec_rate: 0.0,
            rw_response: 30.0,
            bottom_y: 0.0,
        };
        for (i, p) in c.parts.iter().enumerate() {
            let d = &p.def;
            s.part_dry_mass[i] = d.mass;
            for (r, _amt, max) in &d.resources {
                s.part_res_cap[i][*r as usize] += max;
                s.res_cap_total[*r as usize] += max;
            }
            s.part_com[i] = p.pos + p.rot.rotate(d.com_offset);
            s.part_cop[i] = p.pos + p.rot.rotate(d.cop_offset);
            s.part_rot[i] = p.rot;
            let z = d.cube.size;
            s.part_inertia_per_t[i] = v3(z.y * z.y + z.z * z.z, z.x * z.x + z.z * z.z, z.x * z.x + z.y * z.y) / 12.0;
            // occlusion by stack neighbours (DragCubeList.SetPartOcclusion)
            let mut f = AeroFaces::from(&d.cube);
            for &(j, ndir) in &p.stack {
                let q = &c.parts[j];
                let v_in_neigh = q.rot.conj().rotate(p.rot.rotate(ndir));
                occlude(&mut f, &d.cube, ndir, facing_area(&q.def.cube, v_in_neigh));
            }
            s.part_faces[i] = f;
            for k in 0..6 {
                if f.drag[k] < 1.0 {
                    s.part_face_cd_curve[i][k] = crate::curve::eval(crate::aero::DRAG_CD, f.drag[k] as f32 as f64) as f32 as f64;
                }
            }
            s.part_body_lift[i] = if d.body_lift { 1.0 } else { 0.0 };
            s.part_angular_drag[i] = d.angular_drag;
            if let Some(e) = &d.engine {
                let k = s.n_engines as usize;
                assert!(k < MAX_ENGINES && e.atmosphere_curve.keys.len() <= MAX_ISP_KEYS);
                let g = crate::engine::ENGINE_G0;
                let isp0 = e.atmosphere_curve.eval(0.0);
                let mut es = EngineSpec {
                    part: i as u32,
                    max_fuel_flow: e.max_thrust / (isp0 * g),
                    min_fuel_flow: e.min_thrust / (isp0 * g),
                    n_isp_keys: e.atmosphere_curve.keys.len() as u32,
                    ignition_threshold: e.ignition_threshold,
                    accel_speed: e.response.map_or(0.0, |r| r.0),
                    decel_speed: e.response.map_or(0.0, |r| r.1),
                    use_response: e.response.is_some(),
                    gimbal_range_deg: e.gimbal_range_deg,
                    gimbal_response_speed: e.gimbal_response_speed.unwrap_or(0.0),
                    pos: p.pos + p.rot.rotate(e.thrust_pos),
                    dir: p.rot.rotate(e.thrust_dir).normalized(),
                    ..Default::default()
                };
                for (j, key) in e.atmosphere_curve.keys.iter().enumerate() {
                    es.isp_keys[j] = *key;
                }
                // ModuleEngines.SetupPropellant: mixtureDensity = sum density_i * ratio_i
                let mix: f64 = e.propellants.iter().map(|(r, ratio)| RES_DENSITY[*r as usize] * ratio).sum();
                for (r, ratio) in &e.propellants {
                    es.units_per_ton[*r as usize] = ratio / mix;
                }
                s.engines[k] = es;
                s.n_engines += 1;
                // the nozzle tip is a hull point
                s.add_hull(es.pos, d.crash_tolerance);
            }
            if let Some(w) = &d.reaction_wheel {
                s.rw_torque += v3(w.pitch_torque, w.roll_torque, w.yaw_torque);
                s.rw_ec_rate += w.ec_rate;
                s.rw_response = w.torque_response_speed;
            }
            if let Some(l) = &d.leg {
                let k = s.n_legs as usize;
                assert!(k < MAX_LEGS);
                s.legs[k] = LegSpec {
                    foot: p.pos + p.rot.rotate(l.foot),
                    axis: p.rot.rotate(v3(0.0, -1.0, 0.0)),
                    travel: l.suspension_distance,
                    spring_ratio: l.spring_ratio,
                    damper_ratio: l.damper_ratio,
                    crash_tolerance: d.crash_tolerance,
                };
                s.n_legs += 1;
            } else if d.engine.is_none() {
                // stack parts: bottom ring corners + top centre as hull points
                let half = d.cube.size * 0.5;
                let c0 = p.pos + p.rot.rotate(d.cube.center);
                for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                    s.add_hull(c0 + p.rot.rotate(v3(dx * half.x, -half.y, dz * half.z)), d.crash_tolerance);
                }
                s.add_hull(c0 + p.rot.rotate(v3(0.0, half.y, 0.0)), d.crash_tolerance);
            }
        }
        s.bottom_y = (0..s.n_legs as usize).map(|k| s.legs[k].foot.y).fold(f64::INFINITY, f64::min);
        for k in 0..s.n_hull as usize {
            s.bottom_y = s.bottom_y.min(s.hull[k].y);
        }
        s
    }

    fn add_hull(&mut self, p: V3, tol: f64) {
        if (self.n_hull as usize) < MAX_HULL {
            self.hull[self.n_hull as usize] = p;
            self.hull_tolerance[self.n_hull as usize] = tol;
            self.n_hull += 1;
        }
    }

    /// Total mass, root-frame CoM and root-frame inertia tensor (diagonal + off-diagonal) for a
    /// resource fill fraction per resource.
    #[inline]
    pub fn mass_props(&self, res: &[f64; N_RES]) -> (f64, V3, [[f64; 3]; 3]) {
        let mut m_parts = [0.0; MAX_PARTS];
        let mut m = 0.0;
        let mut mc = V3::ZERO;
        for i in 0..self.n_parts as usize {
            let mut mi = self.part_dry_mass[i];
            for r in 0..N_RES {
                if self.res_cap_total[r] > 0.0 {
                    mi += self.part_res_cap[i][r] * (res[r] / self.res_cap_total[r]) * RES_DENSITY[r];
                }
            }
            m_parts[i] = mi;
            m += mi;
            mc += self.part_com[i] * mi;
        }
        let com = mc / m;
        let mut it = [[0.0; 3]; 3];
        for i in 0..self.n_parts as usize {
            let mi = m_parts[i];
            let d = self.part_com[i] - com;
            // part box inertia rotated to root frame (parts here are axis-aligned about Y)
            let ib = self.part_inertia_per_t[i] * mi;
            let r = &self.part_rot[i];
            let ex = r.rotate(v3(1.0, 0.0, 0.0));
            let ey = r.rotate(v3(0.0, 1.0, 0.0));
            let ez = r.rotate(v3(0.0, 0.0, 1.0));
            let axes = [ex, ey, ez];
            let ibv = [ib.x, ib.y, ib.z];
            let dv = [d.x, d.y, d.z];
            let d2 = d.norm2();
            for a in 0..3 {
                for b in 0..3 {
                    let mut v = 0.0;
                    for k in 0..3 {
                        let ak = [axes[k].x, axes[k].y, axes[k].z];
                        v += ibv[k] * ak[a] * ak[b];
                    }
                    let delta = if a == b { 1.0 } else { 0.0 };
                    v += mi * (d2 * delta - dv[a] * dv[b]);
                    it[a][b] += v;
                }
            }
        }
        (m, com, it)
    }

    pub fn full_resources(&self) -> [f64; N_RES] {
        self.res_cap_total
    }

    pub fn dry_mass(&self) -> f64 {
        self.part_dry_mass[..self.n_parts as usize].iter().sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hopper_mass_and_geometry() {
        let s = VesselSpec::compile(&Craft::hopper());
        assert_eq!(s.n_parts, 6);
        assert_eq!(s.n_engines, 1);
        assert_eq!(s.n_legs, 3);
        let (m, com, it) = s.mass_props(&s.full_resources());
        // 0.8 + 0.25 + 1.5 + 3*0.1 dry, + 400 units * 0.005 + 10 mono * 0.004
        assert!((m - (2.85 + 2.0 + 0.04)).abs() < 1e-12, "{m}");
        assert!((s.dry_mass() - 2.85).abs() < 1e-12);
        assert!(com.x.abs() < 1e-9 && com.z.abs() < 1e-9);
        assert!(it[0][0] > 0.0 && it[1][1] > 0.0 && it[2][2] > 0.0);
        // tank top node meets pod bottom node
        let c = Craft::hopper();
        assert!((c.parts[1].pos.y - (-0.4050379 - 0.981725)).abs() < 1e-12);
        // pod bottom face and tank top face occluded away
        assert!(s.part_faces[0].area[3] < 1e-9);
        assert!(s.part_faces[1].area[2] < 1e-9);
        assert!(s.part_faces[1].area[3] < 1e-9 && s.part_faces[2].area[2] < 1e-9);
        // max fuel flow: 215 / (320 * 9.80665f)
        assert!((s.engines[0].max_fuel_flow - 0.068512).abs() < 1e-6, "{}", s.engines[0].max_fuel_flow);
        // the leg feet are the lowest points
        assert!(s.bottom_y < s.engines[0].pos.y);
    }
}
