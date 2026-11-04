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

/// 24: the in-game demo stack (10 stack parts, 4 legs, 4 airbrakes) needs 18.
pub const MAX_PARTS: usize = 24;
pub const MAX_ENGINES: usize = 4;
pub const MAX_LEGS: usize = 6;
pub const MAX_HULL: usize = 24;
pub const MAX_ISP_KEYS: usize = 8;
pub const MAX_AERO_SURFACES: usize = 8;

/// A part placed in the craft's root frame (root part origin, root orientation).
#[derive(Clone, Debug)]
pub struct Placed {
    pub def: PartDef,
    pub pos: V3,
    pub rot: Quat,
    /// stack attachments: (index of neighbour, this part's node direction, part-local)
    pub stack: Vec<(usize, V3)>,
    /// parent part index (None = root)
    pub parent: Option<usize>,
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
        self.parts.push(Placed { def, pos, rot: prot, stack: vec![(parent, ctd)], parent: Some(parent) });
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
        self.parts.push(Placed { def, pos, rot, stack: vec![], parent: Some(parent) });
        self.parts.len() - 1
    }

    /// Surface-attach with the VAB rule (EditorLogic.cs:10794), evaluated in Unity's left-handed
    /// coordinates and mirrored into the sim frame:
    /// rot = LookRotation(hitNormal, parentUp) * LookRotation(srfAttachNode.orientation, up),
    /// pos = hitPoint - rot * srfAttachNode.position (node position already * rescaleFactor).
    /// `az_deg` is the azimuth of the surface normal about +Y in Unity coordinates.
    pub fn surface_attach_editor(&mut self, parent: usize, def: PartDef, az_deg: f64, r: f64, y: f64, node_pos: V3, node_dir: V3) -> usize {
        let a = az_deg.to_radians();
        let n = v3(a.cos(), 0.0, a.sin());
        let ppos = self.parts[parent].pos;
        let hit_u = v3(-ppos.x, ppos.y + y, ppos.z) + n * r;
        let rot_u = unity_look_rotation(n, v3(0.0, 1.0, 0.0)) * unity_look_rotation(node_dir.normalized(), v3(0.0, 1.0, 0.0));
        let pos_u = hit_u - rot_u.rotate(node_pos);
        let pos = v3(-pos_u.x, pos_u.y, pos_u.z);
        let rot = Quat::new(rot_u.x, -rot_u.y, -rot_u.z, rot_u.w);
        self.parts.push(Placed { def, pos, rot, stack: vec![], parent: Some(parent) });
        self.parts.len() - 1
    }

    /// The default lander, identical to the in-game harness craft (harness/make_crafts.py):
    /// Mk1 pod / FL-T400 / LV-T45 Swivel, three LT-2 legs placed by the VAB surface-attach rule at
    /// the tank's lower edge (0.9 m below its centre).
    pub fn hopper() -> Craft {
        use crate::parts::*;
        let mut c = Craft { parts: vec![Placed { def: mk1pod_v2(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![], parent: None }] };
        let tank = c.stack_below(0, fuel_tank_t400());
        let _eng = c.stack_below(tank, swivel());
        // landingLeg1-2 node_attach (LT-2 cfg) * rescaleFactor 1.25
        let np = v3(0.0, 0.4271562, 0.0632596) * 1.25;
        let nd = v3(0.0, 0.5735765, 0.819152);
        for k in 0..3 {
            c.surface_attach_editor(tank, lt2_leg(), 90.0 + 120.0 * k as f64, 0.625, -0.9, np, nd);
        }
        c
    }
}

/// Unity `Quaternion.LookRotation(forward, up)` (+Z -> forward, +Y toward up; x = up x forward).
pub fn unity_look_rotation(fwd: V3, up: V3) -> Quat {
    let z = fwd.normalized();
    let x = up.cross(z).normalized();
    let y = z.cross(x);
    let (m00, m11, m22) = (x.x, y.y, z.z);
    let (m01, m02, m10, m12, m20, m21) = (y.x, z.x, x.y, z.y, x.z, y.z);
    let tr = m00 + m11 + m22;
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
    /// root-frame gimbal pivot: the thrust point swings about it with the deflection
    pub pivot: V3,
}

pub use crate::wheel::LegSpec;

/// One `ModuleAeroSurface` part (airbrake), root frame. All of a vessel's aero surfaces share the
/// deflection state `EnvState::brake_deflection` (the stock module ignores pitch / yaw / roll, so
/// every surface follows the Brakes action group with the same actuator rule).
#[derive(Clone, Copy, Debug)]
pub struct AeroSurfaceSpec {
    pub part: u32,
    /// baseTransform position (= part origin): point velocity, and (CoP/CoLOffset = 0) the force point
    pub pos: V3,
    pub cop: V3,
    pub col: V3,
    /// neutral lift vector and the direction it turns toward with positive deflection
    pub lift_vector: V3,
    pub lift_turn: V3,
    /// kN per kPa of dynamic pressure per unit curve value: deflectionLiftCoeff * multiplier * 1000
    pub drag_k: f64,
    pub lift_k: f64,
    pub ctrl_surface_area: f64,
    pub internal_drag: bool,
    pub has_lift: bool,
    pub curves: crate::aero::LiftingCurves,
    pub deploy_angle: f64,
    pub ctrl_surface_range: f64,
    pub actuator_speed: f64,
}

impl Default for AeroSurfaceSpec {
    fn default() -> Self {
        AeroSurfaceSpec {
            part: 0,
            pos: V3::ZERO,
            cop: V3::ZERO,
            col: V3::ZERO,
            lift_vector: V3::ZERO,
            lift_turn: V3::ZERO,
            drag_k: 0.0,
            lift_k: 0.0,
            ctrl_surface_area: 0.0,
            internal_drag: false,
            has_lift: false,
            curves: crate::aero::SPEED_BRAKE,
            deploy_angle: 0.0,
            ctrl_surface_range: 1.0,
            actuator_speed: 0.0,
        }
    }
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
    /// part CoL (origin + CoLOffset): body / surface lift application point
    pub part_col: [V3; MAX_PARTS],
    pub part_rot: [Quat; MAX_PARTS],
    /// box inertia per tonne (part-local diagonal), from cube size
    pub part_inertia_per_t: [V3; MAX_PARTS],
    /// the same, rotated into the root frame (3x3 per tonne)
    pub part_inertia_root: [[[f64; 3]; 3]; MAX_PARTS],
    pub part_faces: [AeroFaces; MAX_PARTS],
    /// ln(DRAG_CD(face Cd)) (mach-independent; Cd' = exp(pow(M) * this)); NaN marks Cd >= 1
    pub part_face_ln_cd: [[f64; 6]; MAX_PARTS],
    /// the same in f32 (the game evaluates the cube faces in float)
    pub part_face_ln_cd32: [[f32; 6]; MAX_PARTS],
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
    /// breakForce (kN) of the joint between the hull point's part and its parent (INF for the root)
    pub hull_break_force: [f64; MAX_HULL],
    /// part index owning each hull point (its rigidbody velocity is what Part.HandleCollision tests)
    pub hull_part: [u8; MAX_HULL],
    /// whether a contact at the hull point can destroy its part (on the part's primary collider)
    pub hull_crash: [bool; MAX_HULL],
    /// parent part index (-1 = root) and the mass of the subtree below each part's joint is
    /// computed from these (joint load estimate for breakForce)
    pub part_parent: [i8; MAX_PARTS],
    /// bitmask of the parts in each part's subtree (itself and everything below its joint)
    pub part_subtree: [u32; MAX_PARTS],
    /// breakForce of each part's joint to its parent (kN; INF for the root)
    pub part_break_force: [f64; MAX_PARTS],
    /// part index of each leg
    pub leg_part: [u8; MAX_LEGS],
    /// ModuleAeroSurface parts (not packed for the CUDA kernel: gpu.rs ignores them)
    pub n_aero_surf: u32,
    /// bit i: part i has no cube drag / body lift (dragModelType none), so the tick skips its
    /// drag-cube evaluation (angular drag still applies)
    pub part_no_cube_drag: u32,
    pub aero_surf: [AeroSurfaceSpec; MAX_AERO_SURFACES],
    pub rw_torque: V3,
    pub rw_ec_rate: f64,
    pub rw_response: f64,
    /// lowest point (legs fully extended) below the root origin, along root -Y
    pub bottom_y: f64,
    /// radius about the root origin enclosing every contact point (hull, leg rays)
    pub bound_radius: f64,
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
            part_col: [V3::ZERO; MAX_PARTS],
            part_rot: [Quat::IDENTITY; MAX_PARTS],
            part_inertia_per_t: [V3::ZERO; MAX_PARTS],
            part_faces: [AeroFaces::default(); MAX_PARTS],
            part_face_ln_cd: [[f64::NAN; 6]; MAX_PARTS],
            part_face_ln_cd32: [[f32::NAN; 6]; MAX_PARTS],
            part_inertia_root: [[[0.0; 3]; 3]; MAX_PARTS],
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
            hull_break_force: [f64::INFINITY; MAX_HULL],
            hull_part: [0; MAX_HULL],
            hull_crash: [true; MAX_HULL],
            part_parent: [-1; MAX_PARTS],
            part_subtree: [0; MAX_PARTS],
            part_break_force: [f64::INFINITY; MAX_PARTS],
            leg_part: [0; MAX_LEGS],
            n_aero_surf: 0,
            part_no_cube_drag: 0,
            aero_surf: [AeroSurfaceSpec::default(); MAX_AERO_SURFACES],
            rw_torque: V3::ZERO,
            rw_ec_rate: 0.0,
            rw_response: 30.0,
            bottom_y: 0.0,
            bound_radius: 0.0,
        };
        for (i, p) in c.parts.iter().enumerate() {
            s.part_parent[i] = p.parent.map_or(-1, |x| x as i8);
            s.part_break_force[i] = if p.parent.is_some() { p.def.joint_break_force } else { f64::INFINITY };
        }
        for i in 0..c.parts.len() {
            // walk up from every part, marking it in each ancestor's subtree
            let mut j = Some(i);
            while let Some(a) = j {
                s.part_subtree[a] |= 1 << i;
                j = c.parts[a].parent;
            }
        }
        for (i, p) in c.parts.iter().enumerate() {
            let d = &p.def;
            s.part_dry_mass[i] = d.mass;
            for (r, _amt, max) in &d.resources {
                s.part_res_cap[i][*r as usize] += max;
                s.res_cap_total[*r as usize] += max;
            }
            s.part_com[i] = p.pos + p.rot.rotate(d.com_offset);
            s.part_cop[i] = p.pos + p.rot.rotate(d.cop_offset);
            s.part_col[i] = p.pos + p.rot.rotate(d.col_offset);
            s.part_rot[i] = p.rot;
            let z = d.cube.size;
            s.part_inertia_per_t[i] = v3(z.y * z.y + z.z * z.z, z.x * z.x + z.z * z.z, z.x * z.x + z.y * z.y) / 12.0;
            let ib = s.part_inertia_per_t[i];
            let axes = [p.rot.rotate(v3(1.0, 0.0, 0.0)), p.rot.rotate(v3(0.0, 1.0, 0.0)), p.rot.rotate(v3(0.0, 0.0, 1.0))];
            let ibv = [ib.x, ib.y, ib.z];
            for a in 0..3 {
                for b in 0..3 {
                    s.part_inertia_root[i][a][b] = (0..3)
                        .map(|k| {
                            let ak = [axes[k].x, axes[k].y, axes[k].z];
                            ibv[k] * ak[a] * ak[b]
                        })
                        .sum();
                }
            }
            // occlusion by stack neighbours (DragCubeList.SetPartOcclusion)
            let mut f = AeroFaces::from(&d.cube);
            for &(j, ndir) in &p.stack {
                let q = &c.parts[j];
                let v_in_neigh = q.rot.conj().rotate(p.rot.rotate(ndir));
                occlude(&mut f, &d.cube, ndir, facing_area(&q.def.cube, v_in_neigh));
            }
            if let Some(a) = crate::parts::aero_surface(d.name) {
                // the cube weights follow the flap (AeroSurfaceDef::cube), but with dragModelType
                // none the cube carries no drag; a CUBE-model surface would need per-tick faces
                assert!(!a.cube_drag, "{}: aero surfaces with cube drag are not modelled", d.name);
                f = AeroFaces::default();
                s.part_no_cube_drag |= 1 << i;
                let k = s.n_aero_surf as usize;
                assert!(k < MAX_AERO_SURFACES);
                s.aero_surf[k] = AeroSurfaceSpec {
                    part: i as u32,
                    pos: p.pos,
                    cop: s.part_cop[i],
                    col: s.part_col[i],
                    lift_vector: p.rot.rotate(a.lift_vector),
                    lift_turn: p.rot.rotate(a.lift_turn),
                    drag_k: a.deflection_lift_coeff * crate::aero::LIFT_DRAG_MULTIPLIER * 1000.0,
                    lift_k: a.deflection_lift_coeff * crate::aero::LIFT_MULTIPLIER * 1000.0,
                    ctrl_surface_area: a.ctrl_surface_area,
                    internal_drag: a.internal_drag,
                    has_lift: !a.curves.no_lift(),
                    curves: a.curves,
                    deploy_angle: a.deploy_angle,
                    ctrl_surface_range: a.ctrl_surface_range,
                    actuator_speed: a.actuator_speed,
                };
                s.n_aero_surf += 1;
            }
            s.part_faces[i] = f;
            for k in 0..6 {
                if f.drag[k] < 1.0 {
                    s.part_face_ln_cd[i][k] = (crate::curve::eval(crate::aero::DRAG_CD, f.drag[k] as f32 as f64) as f32 as f64).ln();
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
                    pivot: p.pos + p.rot.rotate(e.gimbal_pivot),
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
                    anchor: p.pos + p.rot.rotate(l.anchor),
                    up: p.rot.rotate(v3(0.0, 1.0, 0.0)),
                    fwd: p.rot.rotate(v3(0.0, 0.0, 1.0)),
                    radius: l.radius,
                    travel: l.travel,
                    travel_cfg: l.travel_cfg,
                    spring_ratio: l.spring_ratio,
                    damper_ratio: l.damper_ratio,
                    boost_ratio: l.boost_ratio,
                    maximum_load: l.maximum_load,
                    sprung_mass: d.mass,
                    crash_tolerance: d.crash_tolerance,
                    min_down_force: 10.0,
                };
                s.leg_part[s.n_legs as usize] = i as u8;
                s.n_legs += 1;
            } else {
                // hull points: a ring at each stack node (bottom ring sized like the part, smaller
                // for an engine bell) and the top centre
                // collider rim radius at the bottom node, measured in game (SimHarness collider
                // extents): Swivel bell Col3 0.316, FL-T400 COL3 0.604, Mk1 pod COL 0.625
                let r = d.hull_radius.unwrap_or(if d.engine.is_some() { 0.316 } else { 0.5 * d.cube.size.x.min(d.cube.size.z) });
                let bf = if i == 0 { f64::INFINITY } else { d.joint_break_force };
                if let Some((nb, _)) = d.node_bottom {
                    for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                        s.add_hull(i, p.pos + p.rot.rotate(nb + v3(dx * r, 0.0, dz * r)), d.crash_tolerance, bf, d.hull_crash);
                    }
                }
                if let Some((nt, _)) = d.node_top {
                    s.add_hull(i, p.pos + p.rot.rotate(nt), d.crash_tolerance, bf, d.hull_crash);
                }
                for c in &d.crash_sensors {
                    s.add_hull(i, p.pos + p.rot.rotate(*c), d.crash_tolerance, bf, true);
                }
            }
        }
        s.bottom_y = (0..s.n_legs as usize).map(|k| s.legs[k].foot().y).fold(f64::INFINITY, f64::min);
        for k in 0..s.n_hull as usize {
            s.bottom_y = s.bottom_y.min(s.hull[k].y);
        }
        s.update_bounds();
        s.update_ln_cd32();
        s
    }

    pub fn update_ln_cd32(&mut self) {
        for i in 0..MAX_PARTS {
            for k in 0..6 {
                self.part_face_ln_cd32[i][k] = self.part_face_ln_cd[i][k] as f32;
            }
        }
    }

    /// Recompute `bound_radius` (call after editing hull / leg geometry).
    pub fn update_bounds(&mut self) {
        let mut r: f64 = 0.0;
        for k in 0..self.n_hull as usize {
            r = r.max(self.hull[k].norm());
        }
        for k in 0..self.n_legs as usize {
            r = r.max(self.legs[k].anchor.norm()).max(self.legs[k].foot().norm());
        }
        self.bound_radius = r;
    }

    fn add_hull(&mut self, part: usize, p: V3, tol: f64, break_force: f64, crash: bool) {
        if (self.n_hull as usize) < MAX_HULL {
            self.hull_part[self.n_hull as usize] = part as u8;
            self.hull_crash[self.n_hull as usize] = crash;
            self.hull[self.n_hull as usize] = p;
            self.hull_tolerance[self.n_hull as usize] = tol;
            self.hull_break_force[self.n_hull as usize] = break_force;
            self.n_hull += 1;
        }
    }

    /// Total mass, root-frame CoM and root-frame inertia tensor (diagonal + off-diagonal) for a
    /// resource fill fraction per resource.
    #[inline]
    pub fn mass_props(&self, res: &[f64; N_RES]) -> (f64, V3, [[f64; 3]; 3]) {
        let (m, c, it, _, _) = self.mass_props_parts(res);
        (m, c, it)
    }

    pub fn mass_props_full(&self, res: &[f64; N_RES]) -> (f64, V3, [[f64; 3]; 3], [[f64; 3]; 3]) {
        let (m, c, it, own, _) = self.mass_props_parts(res);
        (m, c, it, own)
    }

    /// Mass, CoM, total inertia about the CoM, and the sum of the parts' *own* inertias (all
    /// root frame). total - own = the parallel-axis ("orbital") part, which in the game comes from
    /// the joints between part rigidbodies.
    #[allow(clippy::type_complexity)]
    pub fn mass_props_parts(&self, res: &[f64; N_RES]) -> (f64, V3, [[f64; 3]; 3], [[f64; 3]; 3], [f64; MAX_PARTS]) {
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
        let mut own = [[0.0; 3]; 3];
        for i in 0..self.n_parts as usize {
            let mi = m_parts[i];
            let d = self.part_com[i] - com;
            let dv = [d.x, d.y, d.z];
            let d2 = d.norm2();
            let ir = &self.part_inertia_root[i];
            for a in 0..3 {
                for b in 0..3 {
                    let delta = if a == b { d2 } else { 0.0 };
                    own[a][b] += ir[a][b] * mi;
                    it[a][b] += ir[a][b] * mi + mi * (delta - dv[a] * dv[b]);
                }
            }
        }
        (m, com, it, own, m_parts)
    }

    /// Part mass for a resource state (dry + its share of each resource).
    #[inline]
    pub fn part_mass(&self, i: usize, res: &[f64; N_RES]) -> f64 {
        let mut mi = self.part_dry_mass[i];
        for r in 0..N_RES {
            if self.res_cap_total[r] > 0.0 {
                mi += self.part_res_cap[i][r] * (res[r] / self.res_cap_total[r]) * RES_DENSITY[r];
            }
        }
        mi
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
        // leg 0 placed exactly where the game put it (v2 harness craft, geometry dump):
        // Unity root-local (0, -2.7695, 0.86648), rot (-0.300706, 0, 0, 0.953717)
        let c = Craft::hopper();
        let l0 = &c.parts[3];
        assert!((l0.pos - v3(0.0, -2.7695012, 0.8664852)).norm() < 1e-5, "{:?}", l0.pos);
        assert!((l0.rot.x + 0.3007059).abs() < 1e-5 && (l0.rot.w - 0.9537171).abs() < 1e-5);
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
