//! Stock part definitions (subset), transcribed from the KSP 1.12.5 part cfgs
//! (GameData/Squad/Parts/...) and the generated drag cubes in PartDatabase.cfg. Every value
//! names the cfg it came from; `tests::matches_install` re-reads the local install and checks
//! the transcription where an install is present.
//!
//! Units follow KSP: tonnes, kN, metres, seconds, resource units.

use crate::aero::DragCube;
use crate::curve::Curve;
use crate::math::{v3, V3};

/// Resource densities (t/unit), GameData/Squad/Resources/ResourcesGeneric.cfg.
pub const DENSITY_LIQUID_FUEL: f64 = 0.005;
pub const DENSITY_OXIDIZER: f64 = 0.005;
pub const DENSITY_MONOPROP: f64 = 0.004;
pub const DENSITY_ELECTRIC: f64 = 0.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Res {
    LiquidFuel = 0,
    Oxidizer = 1,
    MonoPropellant = 2,
    ElectricCharge = 3,
}
pub const N_RES: usize = 4;
pub const RES_DENSITY: [f64; N_RES] = [DENSITY_LIQUID_FUEL, DENSITY_OXIDIZER, DENSITY_MONOPROP, DENSITY_ELECTRIC];

#[derive(Clone, Debug)]
pub struct EngineDef {
    pub max_thrust: f64,
    pub min_thrust: f64,
    pub atmosphere_curve: Curve,
    /// (resource, ratio) PROPELLANT entries
    pub propellants: Vec<(Res, f64)>,
    pub ignition_threshold: f64,
    /// `useEngineResponseTime` -> (engineAccelerationSpeed, engineDecelerationSpeed)
    pub response: Option<(f64, f64)>,
    pub gimbal_range_deg: f64,
    /// `useGimbalResponseSpeed` -> gimbalResponseSpeed
    pub gimbal_response_speed: Option<f64>,
    /// thrust transform position / direction of force, part-local
    pub thrust_pos: V3,
    pub thrust_dir: V3,
    /// gimbal transform position (pivot the thrust transform swings about), part-local
    pub gimbal_pivot: V3,
}

#[derive(Clone, Copy, Debug)]
pub struct ReactionWheelDef {
    pub pitch_torque: f64,
    pub yaw_torque: f64,
    pub roll_torque: f64,
    /// ElectricCharge per second at full input
    pub ec_rate: f64,
    pub torque_response_speed: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct LegDef {
    /// deployed wheel-collider transform position in the part frame (sim, x-mirrored); its up
    /// axis is the part's +Y (deployTgt has identity rotation)
    pub anchor: V3,
    /// WheelCollider radius and suspensionDistance, both already * rescaleFactor
    pub radius: f64,
    pub travel: f64,
    /// cfg suspensionDistance (unscaled; used by the auto springTweakable formula)
    pub travel_cfg: f64,
    pub spring_ratio: f64,
    pub damper_ratio: f64,
    pub boost_ratio: f64,
    pub maximum_load: f64,
}

#[derive(Clone, Debug)]
pub struct PartDef {
    pub name: &'static str,
    pub mass: f64,
    pub com_offset: V3,
    pub cop_offset: V3,
    /// CoLOffset (body-lift application point), part frame
    pub col_offset: V3,
    pub angular_drag: f64,
    pub crash_tolerance: f64,
    pub body_lift: bool,
    pub cube: DragCube,
    pub node_top: Option<(V3, V3)>,
    pub node_bottom: Option<(V3, V3)>,
    /// (resource, amount, maxAmount)
    pub resources: Vec<(Res, f64, f64)>,
    pub engine: Option<EngineDef>,
    pub reaction_wheel: Option<ReactionWheelDef>,
    pub leg: Option<LegDef>,
    /// collision-rim radius at the bottom stack node (measured collider extents), if known
    pub hull_radius: Option<f64>,
    /// breakForce of the joint to the parent as the game sets it (SimHarness dump, kN)
    pub joint_break_force: f64,
    /// whether the hull points above (stack-node rings, top centre) lie on the part's primary
    /// collider. Only contacts on `Part.collider` (PartLoader.GetPartCollider: the first
    /// non-trigger, non-airlock/ladder, non-wheel collider) reach Part.HandleCollision and can
    /// destroy the part; other colliders push but never crash it.
    pub hull_crash: bool,
    /// extra contact points on the primary collider when the ring points are not on it (part
    /// frame, sim x-mirrored; support points measured from the SimHarness collider dump)
    pub crash_sensors: Vec<V3>,
}

/// A PartDatabase cube line, converted to the sim's (x-mirrored) part frame.
fn cube(line: &str) -> DragCube {
    DragCube::parse(line).expect("bad cube").1.mirrored_x()
}

/// Mk1 Command Pod. Command/mk1pod_v2/mk1Pod_v2.cfg; cube PartDatabase.cfg (mk1pod_v2).
pub fn mk1pod_v2() -> PartDef {
    PartDef {
        name: "mk1pod_v2",
        mass: 0.8,
        com_offset: V3::ZERO,
        cop_offset: v3(0.0, 0.5, 0.0),
        col_offset: v3(0.0, -0.35, 0.0),
        angular_drag: 2.0,
        crash_tolerance: 14.0,
        // ModuleLiftingSurface "CapsuleBottom" has disableBodyLift = False, so body lift stays on
        body_lift: true,
        cube: cube("Default, 1.058,0.7167,0.7222, 1.058,0.7186,0.7222, 1.213,0.4635,1.166, 1.213,0.9675,0.1733, 1.063,0.7148,0.7222, 1.063,0.7065,0.7222, 0,0.09149,0, 1.25,1.139,1.25"),
        node_top: Some((v3(0.0, 0.6423756, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -0.4050379, 0.0), v3(0.0, -1.0, 0.0))),
        resources: vec![(Res::ElectricCharge, 50.0, 50.0), (Res::MonoPropellant, 10.0, 10.0)],
        engine: None,
        reaction_wheel: Some(ReactionWheelDef { pitch_torque: 5.0, yaw_torque: 5.0, roll_torque: 5.0, ec_rate: 0.24, torque_response_speed: 30.0 }),
        leg: None,
        hull_radius: Some(0.625),
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// FL-T400. FuelTank/Size1_Tanks/fuelTankT400.cfg; cube PartDatabase.cfg (fuelTank).
pub fn fuel_tank_t400() -> PartDef {
    PartDef {
        name: "fuelTank",
        mass: 0.25,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 6.0,
        body_lift: true,
        cube: cube("Default, 2.431,0.7695,0.7222, 2.431,0.7695,0.7222, 1.213,0.9103,0.177, 1.213,0.9103,0.177, 2.431,0.7695,0.7222, 2.431,0.7695,0.7222, 3.487E-06,0.03462,3.483E-06, 1.25,1.949,1.25"),
        node_top: Some((v3(0.0, 0.981725, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -0.9125, 0.0), v3(0.0, -1.0, 0.0))),
        resources: vec![(Res::LiquidFuel, 180.0, 180.0), (Res::Oxidizer, 220.0, 220.0)],
        engine: None,
        reaction_wheel: None,
        leg: None,
        hull_radius: Some(0.604),
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// FL-T800. FuelTank/Size1_Tanks/fuelTankT800.cfg; cube PartDatabase.cfg (fuelTank_long).
pub fn fuel_tank_t800() -> PartDef {
    PartDef {
        name: "fuelTank_long",
        mass: 0.5,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 6.0,
        body_lift: true,
        cube: cube("Default, 4.779,0.7737,0.7165, 4.779,0.7737,0.7165, 1.213,0.9103,0.1733, 1.213,0.9103,0.1733, 4.779,0.7737,0.7165, 4.779,0.7737,0.7165, 0,-0.006252,5.074E-07, 1.25,3.818,1.25"),
        node_top: Some((v3(0.0, 1.875, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -1.8875, 0.0), v3(0.0, -1.0, 0.0))),
        resources: vec![(Res::LiquidFuel, 360.0, 360.0), (Res::Oxidizer, 440.0, 440.0)],
        engine: None,
        reaction_wheel: None,
        leg: None,
        hull_radius: None,
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// LV-T45 "Swivel". Engine/liquidEngineLV-T45_v2/liquidEngineLV-T45_v2.cfg; "Clean" cube
/// (ModuleJettison fairing absent: nothing on the bottom node), PartDatabase.cfg (liquidEngine2_v2).
pub fn swivel() -> PartDef {
    PartDef {
        name: "liquidEngine2_v2",
        mass: 1.5,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 7.0,
        body_lift: true,
        cube: cube("Clean, 0.9307,0.7168,1.029, 0.9307,0.7145,0.7278, 1.213,0.9359,0.1939, 1.213,0.8416,1.609, 1.142,0.7373,0.7222, 1.142,0.7378,0.7278, 0,0.08432,5.96E-08, 1.25,1.631,1.25"),
        node_top: Some((v3(0.0, 0.0, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -1.63, 0.0), v3(0.0, -1.0, 0.0))),
        resources: vec![],
        engine: Some(EngineDef {
            max_thrust: 215.0,
            min_thrust: 0.0,
            atmosphere_curve: Curve::from_cfg_lines(["0 320", "1 250", "6 0.001"]),
            propellants: vec![(Res::LiquidFuel, 0.9), (Res::Oxidizer, 1.1)],
            ignition_threshold: 0.1,
            response: None,
            gimbal_range_deg: 3.0,
            gimbal_response_speed: None,
            // measured in game (SimHarness geometry dump): thrustTransform 1.15798 m and gimbal
            // transform 0.91589 m below the part origin, on the axis
            thrust_pos: v3(0.0, -1.157983, 0.0),
            thrust_dir: v3(0.0, 1.0, 0.0),
            gimbal_pivot: v3(0.0, -0.915888, 0.0),
        }),
        reaction_wheel: None,
        leg: None,
        hull_radius: Some(0.316),
        joint_break_force: 4400.0,
        // Part.collider is Col5 (engine body, y 0 .. -0.916); the bell (Col3, down to -1.63)
        // is a separate collider: it takes the ground contact but cannot destroy the engine
        // (in game: bell strikes at up to 21.8 m/s with crashTolerance 7, engine intact)
        hull_crash: false,
        crash_sensors: vec![
            v3(-0.067324, -0.915888, -0.116578),
            v3(-0.412290, -0.000722, 0.000019),
            v3(-0.413908, -0.512986, 0.000015),
            v3(0.413914, -0.512991, 0.000015),
            v3(0.000006, -0.840881, 0.277678),
            v3(0.000008, -0.000731, -0.316334),
        ],
    }
}

/// LV-909 "Terrier". Engine/liquidEngineLV-909_v2/liquidEngineLV-909_v2.cfg; "Clean" cube.
pub fn terrier() -> PartDef {
    PartDef {
        name: "liquidEngine3_v2",
        mass: 0.5,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 7.0,
        body_lift: true,
        cube: cube("Clean, 0.7073,0.7091,0.7316, 0.7073,0.7091,0.7316, 1.25,0.8731,0.857, 1.25,0.7881,0.6581, 0.7119,0.7102,0.8007, 0.7119,0.7116,0.7316, 0,-0.3757,4.768E-07, 1.269,0.7756,1.269"),
        node_top: Some((v3(0.0, 0.0, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -0.83, 0.0), v3(0.0, -1.0, 0.0))),
        resources: vec![],
        engine: Some(EngineDef {
            max_thrust: 60.0,
            min_thrust: 0.0,
            atmosphere_curve: Curve::from_cfg_lines(["0 345", "1 85", "3 0.001"]),
            propellants: vec![(Res::LiquidFuel, 0.9), (Res::Oxidizer, 1.1)],
            ignition_threshold: 0.1,
            response: None,
            gimbal_range_deg: 4.0,
            gimbal_response_speed: Some(30.0),
            // Terrier: thrust / gimbal transforms not measured in game (approximate)
            thrust_pos: v3(0.0, -0.83, 0.0),
            thrust_dir: v3(0.0, 1.0, 0.0),
            gimbal_pivot: v3(0.0, -0.6, 0.0),
        }),
        reaction_wheel: None,
        leg: None,
        hull_radius: None,
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// LT-1 Landing Struts, deployed. Utility/landingLegLT-1/landingLegLT-1.cfg
/// (ModuleWheelSuspension suspensionDistance 0.75, springRatio 30, damperRatio 2.2);
/// "Deployed" cube from PartDatabase.cfg (landingLeg1).
/// Part-local frame as surface-attached: +Y up along the vessel, +Z pointing into the parent.
/// The foot point is approximated from the deployed cube extent (wheel collider lives in the model).
pub fn lt1_leg() -> PartDef {
    PartDef {
        name: "landingLeg1",
        mass: 0.05,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 12.0,
        body_lift: true,
        cube: cube("Deployed, 0.3567,0.7883,0.249, 0.3567,0.7874,0.249, 0.1479,0.6561,2.255, 0.1479,0.6832,2.236, 0.358,0.7689,0.6407, 0.358,0.7964,0.6036, -1.49E-08,-0.8749,0.1349, 0.3,2.206,0.5893"),
        node_top: None,
        node_bottom: None,
        resources: vec![],
        engine: None,
        reaction_wheel: None,
        // wheel pose not measured in game for the LT-1 (approximate)
        leg: Some(LegDef { anchor: v3(0.0, -0.6, 0.1), radius: 0.15, travel: 0.75 * 1.25, travel_cfg: 0.75, spring_ratio: 30.0, damper_ratio: 2.2, boost_ratio: 0.7, maximum_load: 40.0 }),
        hull_radius: None,
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// LT-2 Landing Strut, deployed. Utility/landingLegLT-2/landingLegLT-2.cfg (CoMOffset 0,0,0.1;
/// ModuleWheelSuspension suspensionDistance 0.95, springRatio 33, damperRatio 2.2);
/// "Deployed" cube from PartDatabase.cfg (landingLeg1-2).
/// Wheel-collider geometry from the in-game dump; the part is placed with the VAB surface-attach
/// rule (vessel.rs / harness/make_crafts.py).
pub fn lt2_leg() -> PartDef {
    PartDef {
        name: "landingLeg1-2",
        mass: 0.1,
        com_offset: v3(0.0, 0.0, 0.1),
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 12.0,
        body_lift: true,
        cube: cube("Deployed, 0.5144,0.8194,0.6012, 0.5144,0.8194,1.093, 0.2288,0.6022,0.6196, 0.2288,0.7097,3.098, 0.8535,0.7444,0.6931, 0.8535,0.7363,0.5937, 0.0001921,-0.7179,0.1662, 1.017,3.025,0.6183"),
        node_top: None,
        node_bottom: None,
        resources: vec![],
        engine: None,
        reaction_wheel: None,
        // deployed WheelCollider measured in game (SimHarness geometry dump, ModuleWheelDeployment
        // deployTgt): part-local (0, -0.835, 0.104), radius 0.12*1.25, travel 0.95*1.25
        leg: Some(LegDef { anchor: v3(0.0, -0.835, 0.1039), radius: 0.15, travel: 1.1875, travel_cfg: 0.95, spring_ratio: 33.0, damper_ratio: 2.2, boost_ratio: 0.7, maximum_load: 40.0 }),
        hull_radius: None,
        joint_break_force: 4000.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

pub fn by_name(name: &str) -> Option<PartDef> {
    Some(match name {
        "mk1pod_v2" => mk1pod_v2(),
        "fuelTank" => fuel_tank_t400(),
        "fuelTank_long" => fuel_tank_t800(),
        "liquidEngine2_v2" => swivel(),
        "liquidEngine3_v2" => terrier(),
        "landingLeg1" => lt1_leg(),
        "landingLeg1-2" => lt2_leg(),
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cfg;

    fn install() -> Option<std::path::PathBuf> {
        let p = std::path::Path::new(&std::env::var("HOME").ok()?)
            .join(".steam/debian-installation/steamapps/common/Kerbal Space Program");
        p.exists().then_some(p)
    }

    fn part_node(path: &std::path::Path) -> cfg::Node {
        let root = cfg::parse(&std::fs::read_to_string(path).unwrap());
        root.node("PART").unwrap().clone()
    }

    fn vec3(s: &str) -> Vec<f64> {
        s.split(',').map(|t| t.trim().parse().unwrap()).collect()
    }

    /// Transcribed values == the local install's cfgs / PartDatabase.cfg (skipped without an install).
    #[test]
    fn matches_install() {
        let Some(k) = install() else { return };
        let parts = k.join("GameData/Squad/Parts");
        let pdb = cfg::parse(&std::fs::read_to_string(k.join("PartDatabase.cfg")).unwrap());
        let cases = [
            ("Command/mk1pod_v2/mk1Pod_v2.cfg", mk1pod_v2(), "Default"),
            ("FuelTank/Size1_Tanks/fuelTankT400.cfg", fuel_tank_t400(), "Default"),
            ("FuelTank/Size1_Tanks/fuelTankT800.cfg", fuel_tank_t800(), "Default"),
            ("Engine/liquidEngineLV-T45_v2/liquidEngineLV-T45_v2.cfg", swivel(), "Clean"),
            ("Engine/liquidEngineLV-909_v2/liquidEngineLV-909_v2.cfg", terrier(), "Clean"),
            ("Utility/landingLegLT-1/landingLegLT-1.cfg", lt1_leg(), "Deployed"),
            ("Utility/landingLegLT-2/landingLegLT-2.cfg", lt2_leg(), "Deployed"),
        ];
        for (rel, def, cube_name) in cases {
            let p = part_node(&parts.join(rel));
            assert_eq!(p.get("name"), Some(def.name));
            assert_eq!(p.get_f64("mass"), Some(def.mass), "{}", def.name);
            assert_eq!(p.get_f64("crashTolerance"), Some(def.crash_tolerance), "{}", def.name);
            if let Some((pos, _)) = def.node_top {
                let v = vec3(p.get("node_stack_top").unwrap());
                assert_eq!(&v[..3], &[pos.x, pos.y, pos.z], "{}", def.name);
            }
            if let Some((pos, _)) = def.node_bottom {
                let v = vec3(p.get("node_stack_bottom").unwrap());
                assert_eq!(&v[..3], &[pos.x, pos.y, pos.z], "{}", def.name);
            }
            for (r, amt, max) in &def.resources {
                let rn = ["LiquidFuel", "Oxidizer", "MonoPropellant", "ElectricCharge"][*r as usize];
                let n = p.nodes_named("RESOURCE").find(|n| n.get("name") == Some(rn)).unwrap();
                assert_eq!(n.get_f64("amount"), Some(*amt));
                assert_eq!(n.get_f64("maxAmount"), Some(*max));
            }
            if let Some(e) = &def.engine {
                let m = p.nodes_named("MODULE").find(|n| n.get("name").is_some_and(|s| s.starts_with("ModuleEngines"))).unwrap();
                assert_eq!(m.get_f64("maxThrust"), Some(e.max_thrust));
                assert_eq!(cfg::curve(m.node("atmosphereCurve").unwrap()), e.atmosphere_curve);
                let g = p.nodes_named("MODULE").find(|n| n.get("name") == Some("ModuleGimbal")).unwrap();
                assert_eq!(g.get_f64("gimbalRange"), Some(e.gimbal_range_deg));
            }
            if let Some(w) = &def.reaction_wheel {
                let m = p.nodes_named("MODULE").find(|n| n.get("name") == Some("ModuleReactionWheel")).unwrap();
                assert_eq!(m.get_f64("PitchTorque"), Some(w.pitch_torque));
                assert_eq!(m.node("RESOURCE").unwrap().get_f64("rate"), Some(w.ec_rate));
            }
            if let Some(l) = &def.leg {
                let m = p.nodes_named("MODULE").find(|n| n.get("name") == Some("ModuleWheelSuspension")).unwrap();
                assert_eq!(m.get_f64("suspensionDistance"), Some(l.travel_cfg));
                assert_eq!(m.get_f64("springRatio"), Some(l.spring_ratio));
                assert_eq!(m.get_f64("damperRatio"), Some(l.damper_ratio));
            }
            // drag cube: the PartDatabase DRAG_CUBE block of this part
            let dc = pdb.nodes_named("PART").find(|n| n.get("url").is_some_and(|u| u.ends_with(&format!("/{}", def.name)))
                || n.get("name") == Some(def.name)).and_then(|n| n.node("DRAG_CUBE").cloned());
            let dc = dc.expect(def.name);
            let found = dc.get_all("cube").filter_map(DragCube::parse).find(|(n, _)| n == cube_name).unwrap().1;
            assert_eq!(found.mirrored_x(), def.cube, "{}", def.name);
        }
    }
}
