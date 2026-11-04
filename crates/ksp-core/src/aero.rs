//! Stock drag-cube aerodynamics and body lift.
//!
//! Sources (decomp): `DragCube.Load` (DragCube.cs:106), `DragCubeList.SetDragWeights_List`
//! (:282), `DragCubeList.SetPartOcclusion` / `AreaToCubeOperation` (:401, :1579),
//! `DragCubeList.SetDrag` / `AddSurfaceDragDirection` (:634, :670),
//! `PhysicsGlobals.DragCurveValue` (PhysicsGlobals.cs:2808), `FlightIntegrator.UpdateAerodynamics`
//! (FlightIntegrator.cs:5981), `CalculateDragValue_Cube` (:6542), `ApplyAeroDrag` (:5899),
//! `ApplyAeroLift` (:5938). Curves and multipliers: Physics.cfg (values transcribed below, checked
//! against the install by `physics_cfg_matches_install`).

// Curve keys are Physics.cfg literals (sin(AoA) values to 7 digits), not approximated constants.
#![allow(clippy::approx_constant)]

use crate::curve::{eval, k, Key};
use crate::math::{v3, V3};

/// Physics.cfg `dragMultiplier`.
pub const DRAG_MULTIPLIER: f64 = 8.0;
/// Physics.cfg `dragCubeMultiplier`.
pub const DRAG_CUBE_MULTIPLIER: f64 = 0.1;
/// Physics.cfg `angularDragMultiplier`.
pub const ANGULAR_DRAG_MULTIPLIER: f64 = 2.0;
/// Physics.cfg `liftMultiplier`.
pub const LIFT_MULTIPLIER: f64 = 0.036;
/// Physics.cfg `liftDragMultiplier`.
pub const LIFT_DRAG_MULTIPLIER: f64 = 0.015;
/// Physics.cfg `bodyLiftMultiplier`.
pub const BODY_LIFT_MULTIPLIER: f64 = 18.0;

/// Physics.cfg DRAG_TIP (x = mach).
pub const DRAG_TIP: &[Key] = &[k(0.0, 1.0, 0.0, 0.0), k(0.85, 1.19, 0.6960422, 0.6960422), k(1.1, 2.83, 0.730473, 0.730473), k(5.0, 4.0, 0.0, 0.0)];
/// Physics.cfg DRAG_SURFACE.
pub const DRAG_SURFACE: &[Key] = &[
    k(0.0, 0.02, 0.0, 0.0), k(0.85, 0.02, 0.0, 0.0), k(0.9, 0.0152439, -0.07942077, -0.07942077),
    k(1.1, 0.0025, -0.005279571, -0.001936768), k(2.0, 0.002083333, -2.314833E-05, -2.314833E-05),
    k(5.0, 0.003333333, -0.000180556, -0.000180556), k(25.0, 0.001428571, -7.14286E-05, 0.0),
];
/// Physics.cfg DRAG_TAIL.
pub const DRAG_TAIL: &[Key] = &[
    k(0.0, 1.0, 0.0, 0.0), k(0.85, 1.0, 0.0, 0.0), k(1.1, 0.25, -0.02215106, -0.02487721),
    k(1.4, 0.22, -0.03391732, -0.03391732), k(5.0, 0.15, -0.001198566, -0.001198566), k(25.0, 0.14, 0.0, 0.0),
];
/// Physics.cfg DRAG_MULTIPLIER (overall mach multiplier).
pub const DRAG_MACH_MULT: &[Key] = &[
    k(0.0, 0.5, 0.0, 0.0), k(0.85, 0.5, 0.0, 0.0), k(1.1, 1.3, 0.0, -0.008100224), k(2.0, 0.7, -0.1104858, -0.1104858),
    k(5.0, 0.6, 0.0, 0.0), k(10.0, 0.85, 0.02198264, 0.02198264), k(14.0, 0.9, 0.007694946, 0.007694946), k(25.0, 0.95, 0.0, 0.0),
];
/// Physics.cfg DRAG_CD (cube Cd -> effective Cd).
pub const DRAG_CD: &[Key] = &[
    k(0.05, 0.0025, 0.15, 0.15), k(0.4, 0.15, 0.3963967, 0.3963967), k(0.7, 0.35, 0.9066986, 0.9066986),
    k(0.75, 0.45, 3.213604, 3.213604), k(0.8, 0.66, 3.49833, 3.49833), k(0.85, 0.8, 2.212924, 2.212924),
    k(0.9, 0.89, 1.1, 1.1), k(1.0, 1.0, 1.0, 1.0),
];
/// Physics.cfg DRAG_CD_POWER (x = mach).
pub const DRAG_CD_POWER: &[Key] = &[k(0.0, 1.0, 0.0, 0.00715953), k(0.85, 1.25, 0.7780356, 0.7780356), k(1.1, 2.5, 0.2492796, 0.2492796), k(5.0, 3.0, 0.0, 0.0)];
/// Physics.cfg DRAG_PSEUDOREYNOLDS (x = rho * v).
pub const DRAG_PSEUDOREYNOLDS: &[Key] = &[
    k(0.0, 4.0, 0.0, -2975.412), k(0.0001, 3.0, -251.1479, -251.1479), k(0.01, 2.0, -19.63584, -19.63584),
    k(0.1, 1.2, -0.7846036, -0.7846036), k(1.0, 1.0, 0.0, 0.0), k(100.0, 1.0, 0.0, 0.0), k(200.0, 0.82, 0.0, 0.0),
    k(500.0, 0.86, 0.0001932119, 0.0001932119), k(1000.0, 0.9, 1.54299E-05, 1.54299E-05), k(10000.0, 0.95, 0.0, 0.0),
];
/// Physics.cfg LIFTING_SURFACE "BodyLift" lift (x = dot).
pub const BODY_LIFT_CURVE: &[Key] = &[
    k(0.0, 0.0, 0.0, 1.975376), k(0.309017, 0.5877852, 1.565065, 1.565065), k(0.5877852, 0.9510565, 0.735902, 0.735902),
    k(0.7071068, 1.0, 0.0, 0.0), k(0.8910065, 0.809017, -2.70827, -2.70827), k(1.0, 0.0, -11.06124, 0.0),
];
/// Physics.cfg LIFTING_SURFACE "BodyLift" liftMach.
pub const BODY_LIFT_MACH: &[Key] = &[
    k(0.3, 0.167, 0.0, 0.0), k(0.8, 0.167, 0.0, -0.3904104), k(1.0, 0.125, -0.0005291355, -0.02625772),
    k(5.0, 0.0625, 0.0, 0.0), k(25.0, 0.05, 0.0, 0.0),
];

#[inline(always)]
fn fe(keys: &[Key], x: f64) -> f64 {
    eval(keys, x as f32 as f64) as f32 as f64
}

/// Face order of a cube line and `DragCubeList.faceDirections`: +X, -X, +Y, -Y, +Z, -Z
/// (part-local Unity axes).
pub const FACE_DIRS: [V3; 6] = [v3(1.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, -1.0, 0.0), v3(0.0, 0.0, 1.0), v3(0.0, 0.0, -1.0)];

/// One (weight-blended) drag cube.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DragCube {
    pub area: [f64; 6],
    pub drag: [f64; 6],
    pub depth: [f64; 6],
    pub center: V3,
    pub size: V3,
}

impl DragCube {
    /// Parse a `cube = name, a,d,depth x6, cx,cy,cz, sx,sy,sz` value (`DragCube.Load`).
    pub fn parse(value: &str) -> Option<(String, DragCube)> {
        let tok: Vec<&str> = value.split([',', ' ']).map(str::trim).filter(|s| !s.is_empty()).collect();
        if tok.len() != 25 {
            return None;
        }
        let f: Vec<f64> = tok[1..].iter().map(|s| s.parse::<f64>()).collect::<Result<_, _>>().ok()?;
        let mut c = DragCube::default();
        for i in 0..6 {
            c.area[i] = f[3 * i];
            c.drag[i] = f[3 * i + 1];
            c.depth[i] = f[3 * i + 2];
        }
        c.center = v3(f[18], f[19], f[20]);
        c.size = v3(f[21], f[22], f[23]);
        Some((tok[0].to_string(), c))
    }

    /// The same cube expressed in the sim's right-handed part frame, which is Unity's left-handed
    /// part frame mirrored in x (x -> -x): the +X and -X faces swap and the centre x flips.
    pub fn mirrored_x(&self) -> DragCube {
        let mut c = *self;
        c.area.swap(0, 1);
        c.drag.swap(0, 1);
        c.depth.swap(0, 1);
        c.center.x = -c.center.x;
        c
    }

    /// `SetDragWeights_List`: weight-normalised blend of several cubes.
    pub fn blend(cubes: &[(f64, DragCube)]) -> DragCube {
        let wsum: f64 = cubes.iter().map(|(w, _)| *w).sum();
        let mut o = DragCube::default();
        for (w, c) in cubes {
            let w = *w / wsum;
            for i in 0..6 {
                o.area[i] += w * c.area[i];
                o.drag[i] += w * c.drag[i];
                o.depth[i] += w * c.depth[i];
            }
            o.center += c.center * w;
            o.size += c.size * w;
        }
        o
    }
}

/// Post-occlusion per-face data used at run time (`areaOccluded`, `weightedDrag`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct AeroFaces {
    pub area: [f64; 6],
    pub drag: [f64; 6],
}

impl From<&DragCube> for AeroFaces {
    fn from(c: &DragCube) -> Self {
        AeroFaces { area: c.area, drag: c.drag }
    }
}

/// `DragCubeList.SetPartOcclusion` for one stack node: occlude the faces of `faces` (part-local
/// node direction `n`) by a neighbour whose facing area is `facing_area`.
/// `orig` is the un-occluded blended cube. Call once per occupied stack node, in node order.
pub fn occlude(faces: &mut AeroFaces, orig: &DragCube, n: V3, facing_area: f64) {
    let n = n.normalized();
    for i in 0..6 {
        let kf = n.dot(FACE_DIRS[i]).clamp(0.0, 1.0);
        if kf <= 0.0 {
            continue;
        }
        faces.area[i] = (faces.area[i] - facing_area * kf).max(0.0);
        faces.drag[i] = if faces.area[i] > 0.0 {
            ((orig.drag[i] * orig.area[i] - (orig.area[i] - faces.area[i])) / faces.area[i]).max(0.0)
        } else {
            1e-5
        };
    }
}

/// Facing area a neighbour presents along its own-frame direction `v` toward us:
/// sum_j area_j * clamp01(-v . faceDir_j) (`SetPartOcclusion`).
pub fn facing_area(neigh: &DragCube, v_in_neigh_frame: V3) -> f64 {
    (0..6).map(|j| neigh.area[j] * (-v_in_neigh_frame).dot(FACE_DIRS[j]).clamp(0.0, 1.0)).sum()
}

/// `PhysicsGlobals.DragCurveValue(dot01, mach)`.
#[inline]
pub fn drag_curve_value(dot01: f64, mach: f64) -> f64 {
    let m = fe(DRAG_MACH_MULT, mach);
    if dot01 <= 0.5 {
        let (a, b) = (fe(DRAG_TAIL, mach), fe(DRAG_SURFACE, mach));
        (a + (b - a) * (2.0 * dot01)) * m
    } else {
        let (a, b) = (fe(DRAG_SURFACE, mach), fe(DRAG_TIP, mach));
        (a + (b - a) * (2.0 * dot01 - 1.0)) * m
    }
}

/// Per-mach curve values reused across all faces/parts of a vessel in one tick.
#[derive(Clone, Copy, Debug)]
pub struct MachTerms {
    pub tip: f64,
    pub surf: f64,
    pub tail: f64,
    pub mult: f64,
    pub cd_pow: f64,
    pub body_lift_mach: f64,
}

#[inline]
pub fn mach_terms(mach: f64) -> MachTerms {
    MachTerms {
        tip: fe(DRAG_TIP, mach),
        surf: fe(DRAG_SURFACE, mach),
        tail: fe(DRAG_TAIL, mach),
        mult: fe(DRAG_MACH_MULT, mach),
        cd_pow: fe(DRAG_CD_POWER, mach),
        body_lift_mach: fe(BODY_LIFT_MACH, mach),
    }
}

/// Result of `DragCubeList.SetDrag`: `AreaDrag` and the part-local body-lift vector.
#[derive(Clone, Copy, Debug, Default)]
pub struct CubeDrag {
    pub area: f64,
    pub area_drag: f64,
    pub lift_local: V3,
}

/// `AddSurfaceDragDirection(d, mach)` where `d` is the part-local unit *velocity* direction
/// (FI passes -InverseTransformDirection(v) and SetDrag negates it back).
#[inline]
pub fn set_drag(f: &AeroFaces, d: V3, mt: &MachTerms) -> CubeDrag {
    let mut out = CubeDrag::default();
    for i in 0..6 {
        let dot = d.dot(FACE_DIRS[i]);
        let dot01 = (dot + 1.0) * 0.5;
        let s = if dot01 <= 0.5 {
            (mt.tail + (mt.surf - mt.tail) * (2.0 * dot01)) * mt.mult
        } else {
            (mt.surf + (mt.tip - mt.surf) * (2.0 * dot01 - 1.0)) * mt.mult
        };
        let a = f.area[i] * s;
        out.area += a;
        let cd = f.drag[i];
        let cdp = if cd < 1.0 { fe(DRAG_CD, cd).powf(mt.cd_pow) } else { cd };
        out.area_drag += a * cdp;
        if dot > 0.0 {
            let l = dot * f.area[i] * cd * fe(BODY_LIFT_CURVE, dot);
            out.lift_local -= FACE_DIRS[i] * l;
        }
    }
    if out.area <= 0.0 {
        out.area_drag = 0.0;
    }
    out
}

/// Per-part aero forces (world frame, kN), `FlightIntegrator.UpdateAerodynamics` cube branch.
/// `v_world`: part velocity relative to the air (m/s); `to_world`/`to_local`: part rotation.
/// `rho`: vessel density (at CoM), `pre_mult`: DRAG_PSEUDOREYNOLDS(rho*spd_vessel).
#[inline]
pub fn part_aero_force(
    f: &AeroFaces,
    v_world: V3,
    rot: crate::math::Quat,
    rho: f64,
    mt: &MachTerms,
    pre_mult: f64,
    body_lift_mult: f64,
) -> (V3, V3) {
    let spd2 = v_world.norm2();
    if rho <= 0.0 || spd2 <= 0.0 {
        return (V3::ZERO, V3::ZERO);
    }
    let vhat = v_world / spd2.sqrt();
    let d_local = rot.conj().rotate(vhat);
    let cd = set_drag(f, d_local, mt);
    let q = rho * 0.0005 * spd2; // kPa
    let drag_scalar = q * cd.area_drag * DRAG_CUBE_MULTIPLIER * pre_mult * DRAG_MULTIPLIER;
    let drag = vhat * (-drag_scalar);
    let lift = if body_lift_mult != 0.0 {
        let bl = q * body_lift_mult * BODY_LIFT_MULTIPLIER * mt.body_lift_mach;
        let l = rot.rotate(cd.lift_local * bl);
        l - vhat * l.dot(vhat) // ProjectOnPlane(l, -v)
    } else {
        V3::ZERO
    };
    (drag, lift)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_cube_line() {
        let (name, c) = DragCube::parse("Default, 1.1,0.7,0.5, 1.1,0.7,0.5, 2.0,0.9,0.3, 1.2,0.95,0.2, 1.1,0.7,0.5, 1.1,0.7,0.5, 0,0.1,0, 1.25,0.9,1.25").unwrap();
        assert_eq!(name, "Default");
        assert_eq!(c.area[2], 2.0);
        assert_eq!(c.drag[3], 0.95);
        assert_eq!(c.size, v3(1.25, 0.9, 1.25));
    }

    #[test]
    fn subsonic_head_on_flat_plate() {
        // a unit face with Cd = 1 straight into the wind at low mach: TIP(0)=1, MULT(0)=0.5
        let mut f = AeroFaces::default();
        f.area[2] = 1.0;
        f.drag[2] = 1.0;
        let mt = mach_terms(0.0);
        let r = set_drag(&f, v3(0.0, 1.0, 0.0), &mt);
        assert!((r.area - 0.5).abs() < 1e-12);
        assert!((r.area_drag - 0.5).abs() < 1e-12);
        // force: q * 0.5 * 0.1 * 8 = 0.4 q  -> effective CdA = 0.8 m^2 per m^2 face at Cd 1
        let (d, _) = part_aero_force(&f, v3(0.0, 100.0, 0.0), crate::math::Quat::IDENTITY, 1.0, &mt, 1.0, 1.0);
        assert!((d.y + 0.0005 * 1.0 * 1e4 * 0.4).abs() < 1e-9, "{d:?}");
    }

    #[test]
    fn occlusion_removes_area_and_keeps_drag_sane() {
        let (_, c) = DragCube::parse("A, 1,0.8,1, 1,0.8,1, 1.2,0.9,1, 1.2,0.9,1, 1,0.8,1, 1,0.8,1, 0,0,0, 1,1,1").unwrap();
        let mut f = AeroFaces::from(&c);
        occlude(&mut f, &c, v3(0.0, 1.0, 0.0), 1.2);
        assert_eq!(f.area[2], 0.0);
        assert_eq!(f.drag[2], 1e-5);
        assert_eq!(f.area[3], 1.2);
    }

    /// The transcribed curves equal the ones in the local install's Physics.cfg (skipped if no install).
    #[test]
    fn physics_cfg_matches_install() {
        let p = std::path::Path::new(&std::env::var("HOME").unwrap_or_default())
            .join(".steam/debian-installation/steamapps/common/Kerbal Space Program/Physics.cfg");
        let Ok(text) = std::fs::read_to_string(&p) else { return };
        let root = crate::cfg::parse(&text);
        let chk = |name: &str, keys: &[Key]| {
            let c = crate::cfg::curve(root.node(name).unwrap());
            assert_eq!(c.keys.as_slice(), keys, "{name}");
        };
        chk("DRAG_TIP", DRAG_TIP);
        chk("DRAG_SURFACE", DRAG_SURFACE);
        chk("DRAG_TAIL", DRAG_TAIL);
        chk("DRAG_MULTIPLIER", DRAG_MACH_MULT);
        chk("DRAG_CD", DRAG_CD);
        chk("DRAG_CD_POWER", DRAG_CD_POWER);
        chk("DRAG_PSEUDOREYNOLDS", DRAG_PSEUDOREYNOLDS);
        let ls = root.node("LIFTING_SURFACE_CURVES").unwrap();
        let bl = ls.nodes_named("LIFTING_SURFACE").find(|n| n.get("name") == Some("BodyLift")).unwrap();
        assert_eq!(crate::cfg::curve(bl.node("lift").unwrap()).keys.as_slice(), BODY_LIFT_CURVE);
        assert_eq!(crate::cfg::curve(bl.node("liftMach").unwrap()).keys.as_slice(), BODY_LIFT_MACH);
        for (k, v) in [("dragMultiplier", DRAG_MULTIPLIER), ("dragCubeMultiplier", DRAG_CUBE_MULTIPLIER),
                       ("angularDragMultiplier", ANGULAR_DRAG_MULTIPLIER), ("liftMultiplier", LIFT_MULTIPLIER),
                       ("liftDragMultiplier", LIFT_DRAG_MULTIPLIER), ("bodyLiftMultiplier", BODY_LIFT_MULTIPLIER)] {
            assert_eq!(root.get_f64(k), Some(v), "{k}");
        }
        let _ = drag_curve_value(0.5, 0.3);
    }
}
