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

/// LT-2 with the "Retracted" cube (legs stowed in flight); otherwise identical to `lt2_leg`.
pub fn lt2_leg_retracted() -> PartDef {
    PartDef {
        cube: cube("Retracted, 0.4164,0.8241,0.4329, 0.4164,0.8239,0.373, 0.4052,0.5802,1.287, 0.4052,0.5889,1.511, 0.5057,0.679,1.209, 0.5057,0.7584,1.134, 1.043E-07,0.7416,0.4329, 0.4942,1.433,1.152"),
        ..lt2_leg()
    }
}

/// A plain stack part (no modules the sim models besides resources / reaction wheel).
#[allow(clippy::too_many_arguments)]
fn stack_part(name: &'static str, mass: f64, angular_drag: f64, crash_tolerance: f64, half_height: f64, cube_line: &str, resources: Vec<(Res, f64, f64)>, reaction_wheel: Option<ReactionWheelDef>) -> PartDef {
    PartDef {
        name,
        mass,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag,
        crash_tolerance,
        body_lift: true,
        cube: cube(cube_line),
        node_top: Some((v3(0.0, half_height, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -half_height, 0.0), v3(0.0, -1.0, 0.0))),
        resources,
        engine: None,
        reaction_wheel,
        leg: None,
        hull_radius: None,
        // not measured for these parts; the value the game set on every stack joint dumped so far
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

// Cubes of the next four parts: PartDatabase.cfg of the demo install (~/ksp-demo/KSP, the copy
// that flew ingame/telemetry); the fidelity install's regenerated cubes differ by < 1 % on a few
// faces (renderer-dependent cube generation).

/// RC-001S probe core. Command/probeStackSmall/probeStackSmall.cfg (rescaleFactor 1).
pub fn probe_stack_small() -> PartDef {
    stack_part(
        "probeStackSmall", 0.1, 2.0, 9.0, 0.1,
        "Default, 0.2933,0.7008,0.758, 0.2933,0.7009,0.758, 1.398,0.9704,0.1499, 1.398,0.9739,0.1499, 0.2852,0.6613,0.7765, 0.2852,0.6622,0.7765, 0,0,0, 1.334,0.22,1.359",
        vec![(Res::ElectricCharge, 15.0, 15.0)],
        Some(ReactionWheelDef { pitch_torque: 0.5, yaw_torque: 0.5, roll_torque: 0.5, ec_rate: 0.03, torque_response_speed: 30.0 }),
    )
}

/// Z-1k battery bank. Electrical/z-1kBattery/z-1kBattery.cfg.
pub fn battery_bank() -> PartDef {
    stack_part(
        "batteryBank", 0.05, 1.0, 8.0, 0.1108553,
        "Default, 0.2808,0.7809,0.7051, 0.2808,0.7809,0.7051, 1.213,0.9695,0.14, 1.213,0.9695,0.14, 0.2808,0.7805,0.7051, 0.2808,0.7805,0.7051, 0,0,0, 1.25,0.22,1.25",
        vec![(Res::ElectricCharge, 1000.0, 1000.0)],
        None,
    )
}

/// Small inline reaction wheel. Command/inlineAdvancedStabilizer/inlineAdvancedStabilizer.cfg.
pub fn adv_sas_module() -> PartDef {
    stack_part(
        "advSasModule", 0.1, 2.0, 9.0, 0.1990267,
        "Default, 0.5389,0.7798,0.7665, 0.5389,0.7799,0.7665, 1.281,0.988,0.1504, 1.281,0.988,0.1504, 0.5389,0.7789,0.7665, 0.5389,0.7788,0.7665, 0,7.451E-09,-8.882E-16, 1.351,0.3993,1.351",
        vec![],
        Some(ReactionWheelDef { pitch_torque: 15.0, yaw_torque: 15.0, roll_torque: 15.0, ec_rate: 0.45, torque_response_speed: 30.0 }),
    )
}

/// TD-12 decoupler (inert: ModuleDecouple is not simulated). Coupling/Decoupler_1.cfg
/// (explosiveNodeID = top: after staging it stays on the part below it).
pub fn decoupler_td12() -> PartDef {
    stack_part(
        "Decoupler_1", 0.04, 2.0, 6.0, 0.05,
        "Default, 0.1988,0.7448,0.8757, 0.1988,0.7448,0.8757, 1.217,0.9952,0.1299, 1.217,0.9952,0.1299, 0.1988,0.7442,0.8871, 0.1988,0.7442,0.8871, 0,0,0, 1.25,0.16,1.25",
        vec![],
        None,
    )
}

/// `PartJoint.Create` breakForce (kN) of a joint between parts with cfg `breakingForce` `bf_a`,
/// `bf_b` (Part default 22) over an attach node of `size` (the smaller of the two nodes' sizes):
/// min(bf) * (size + 1) * (stackNodeFactor 2 | srfNodeFactor 0.8) * jointBreakForceFactor 50
/// (Physics.cfg). Nodes of size >= 2 get 3 internal ConfigurableJoints that split this force;
/// the value returned is their sum, the single-joint equivalent for a load shared evenly.
/// Reproduces the dumped 4400 (size-1 stack, both 22) and 4000 (LT-2 80 on FL-T400 50, srf).
pub fn joint_break_force(bf_a: f64, bf_b: f64, size: u32, surface: bool) -> f64 {
    bf_a.min(bf_b) * (size as f64 + 1.0) * if surface { 0.8 } else { 2.0 } * 50.0
}

/// `Part.breakingForce` default (no `breakingForce` in the cfg).
pub const DEFAULT_BREAKING_FORCE: f64 = 22.0;

// 3.75 m booster parts. Cubes: PartDatabase.cfg of the demo install, except where the part cfg
// carries its own DRAG_CUBE (Size3To2Adapter_v2, asasmodule1-2): PartLoader loads that into the
// prefab at full precision and only writes a rounded copy to PartDatabase (PartLoader
// CompileParts: SetDatabaseConfig when the prefab already has cubes), and flight parts clone the
// prefab's serialized DragCubeList. Hull radii are the bottom-face axis extents of each part's
// primary collider read from its .mu model (PartReader.ReadChild format), the quantity the
// measured FL-T400 0.604 / Mk1 pod 0.625 / Swivel 0.316 are; not yet confirmed by an in-game
// collider dump. Joint break forces: `joint_break_force` for the joint to the parent in the heavy
// booster (guidance ingame::heavy_craft: root S3-14400, adapter and the 2.5 m parts above it, the
// S3-7200 and Mammoth below); the pair-dependent value differs in other trees.

/// Joint to the part below over a size-2 node between two default-22 parts (3 joints, 6600 kN;
/// the Z-4K's to the adapter too: min(22, 200)).
const JBF_SIZE2: f64 = 22.0 * 3.0 * 2.0 * 50.0;

/// Protective Rocket Nose Cone Mk7 (2.5 m). Aero/cones/protectiveRocketNoseMk7_v2.cfg (two MODEL
/// nodes: orangeNoseCone, rocketNoseCone_v2; default variant White shows `nose` + `collider`).
pub fn nose_cone_mk7() -> PartDef {
    PartDef {
        name: "rocketNoseCone_v2",
        mass: 0.2,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 0.5,
        crash_tolerance: 10.0,
        body_lift: true,
        cube: cube("Default, 4.218,0.4841,1.345, 4.218,0.5846,1.345, 4.853,0.3728,1.976, 4.853,0.8912,0.3712, 4.218,0.483,1.345, 4.218,0.5811,1.345, 0,0.9061,0, 2.5,2.358,2.5"),
        node_top: None,
        node_bottom: Some((V3::ZERO, v3(0.0, -1.0, 0.0))),
        resources: vec![],
        engine: None,
        reaction_wheel: None,
        leg: None,
        // `collider` mesh: bottom face y -0.027, axis extents 1.25
        hull_radius: Some(1.25),
        joint_break_force: JBF_SIZE2,
        // PartLoader.GetPartCollider runs on the prefab with every MODEL loaded: the first collider
        // is orangeNoseCone's foamCol, which the White variant then deactivates (GAMEOBJECTS
        // foamCol = false). The active `collider` is not Part.collider, so it pushes but cannot
        // crash the cone. Decomp reading, not yet seen in game.
        hull_crash: false,
        crash_sensors: vec![],
    }
}

/// RC-L01 probe core. Command/probeStackLarge/probeStackLarge.cfg (nodes size 2; ModuleCommand's
/// 0.08 EC/s is not simulated).
pub fn probe_stack_large() -> PartDef {
    PartDef {
        hull_radius: Some(1.25),
        joint_break_force: JBF_SIZE2,
        ..stack_part(
            "probeStackLarge", 0.5, 2.0, 9.0, 0.19,
            "Default, 0.9721,0.7321,1.4, 0.9721,0.7326,1.4, 5.261,0.9806,0.1842, 5.261,0.9795,0.1842, 0.9997,0.7308,1.375, 0.9997,0.7297,1.42, -1.748E-09,2.384E-09,-0.02, 2.611,0.38,2.651",
            vec![(Res::ElectricCharge, 30.0, 30.0)],
            Some(ReactionWheelDef { pitch_torque: 1.5, yaw_torque: 1.5, roll_torque: 1.5, ec_rate: 0.15, torque_response_speed: 30.0 }),
        )
    }
}

/// Advanced Reaction Wheel Module, Large. Command/advancedSasModuleLarge/advSasModuleLarge.cfg
/// (cfg DRAG_CUBE).
pub fn adv_sas_module_large() -> PartDef {
    PartDef {
        hull_radius: Some(1.207407),
        joint_break_force: JBF_SIZE2,
        ..stack_part(
            "asasmodule1-2", 0.2, 2.0, 9.0, 0.25,
            "Default, 1.245117,0.7806778,1.344706, 1.245117,0.7806778,1.344706, 4.643318,0.9183766,0.5737258, 4.643318,0.9590334,0.5462748, 1.245117,0.774645,1.344706, 1.245117,0.774645,1.344706, 0,0,0, 2.5,0.5000004,2.5",
            vec![],
            Some(ReactionWheelDef { pitch_torque: 30.0, yaw_torque: 30.0, roll_torque: 30.0, ec_rate: 0.6, torque_response_speed: 30.0 }),
        )
    }
}

/// Z-4K battery bank. Electrical/z-4kBattery/z-4kBattery.cfg (nodes +0.11 / -0.142).
pub fn battery_bank_large() -> PartDef {
    PartDef {
        node_top: Some((v3(0.0, 0.11, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -0.142, 0.0), v3(0.0, -1.0, 0.0))),
        hull_radius: Some(1.25),
        joint_break_force: JBF_SIZE2,
        ..stack_part(
            "batteryBankLarge", 0.2, 1.0, 8.0, 0.0,
            "Default, 0.6287,0.7858,1.378, 0.6287,0.7857,1.378, 4.872,0.9759,0.2241, 4.872,0.9767,0.2259, 0.6221,0.7782,1.345, 0.6221,0.7731,1.345, 0,-0.01703,0, 2.633,0.25,2.5",
            vec![(Res::ElectricCharge, 4000.0, 4000.0)],
            None,
        )
    }
}

/// Kerbodyne ADTP-2-3 (3.75 m -> 2.5 m, carries fuel). Structural/Size3To2Adapter_v2/
/// Size2to3_v2.cfg (top node size 2, bottom size 3; breakingForce 200; cfg DRAG_CUBE).
pub fn adapter_s3_s2() -> PartDef {
    PartDef {
        node_top: Some((v3(0.0, 1.15017, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -1.1, 0.0), v3(0.0, -1.0, 0.0))),
        // AdapterCol bottom face y -1.1037: x extent 1.8751, z 1.8657 (polygon); the smaller
        hull_radius: Some(1.865699),
        // to the S3-14400 below (the root): size 3, min(200, 400)
        joint_break_force: joint_break_force(200.0, 400.0, 3, false),
        ..stack_part(
            "Size3To2Adapter_v2", 1.875, 2.0, 6.0, 0.0,
            "Default, 7.344658,0.7515181,1.980434, 7.344658,0.7516301,1.980434, 11.00077,0.4258759,2.35452, 11.00077,0.7470945,0.1906485, 7.3984,0.7432775,1.971122, 7.3984,0.7581861,1.971122, -1.907349E-06,0,0, 3.776463,2.301678,3.757764",
            vec![(Res::LiquidFuel, 1350.0, 1350.0), (Res::Oxidizer, 1650.0, 1650.0)],
            None,
        )
    }
}

/// Kerbodyne S3-14400. FuelTank/Size3Tanks/large.cfg (breakingForce 400; base variant
/// BlackAndWhite, useMultipleDragCubes = false).
pub fn s3_tank_14400() -> PartDef {
    PartDef {
        // COL_L: 24-gon, axis extents 1.8111
        hull_radius: Some(1.81111),
        // the heavy booster's root; this is the joint to a like part, min(400, 400) over size 3
        joint_break_force: joint_break_force(400.0, 400.0, 3, false),
        ..stack_part(
            "Size3LargeTank", 9.0, 2.0, 6.0, 3.74,
            "Default, 28.42,0.7567,2.051, 28.42,0.7555,2.051, 11,0.8538,0.516, 11,0.8535,0.516, 29.72,0.7815,1.936, 29.72,0.781,1.936, 0,-3.052E-05,0, 3.984,7.54,3.75",
            vec![(Res::LiquidFuel, 6480.0, 6480.0), (Res::Oxidizer, 7920.0, 7920.0)],
            None,
        )
    }
}

/// Kerbodyne S3-7200. FuelTank/Size3Tanks/medium.cfg (nodes +1.931 / -1.937; breakingForce 300).
pub fn s3_tank_7200() -> PartDef {
    PartDef {
        node_top: Some((v3(0.0, 1.931, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: Some((v3(0.0, -1.937, 0.0), v3(0.0, -1.0, 0.0))),
        hull_radius: Some(1.81111),
        // to the S3-14400 above: size 3, min(300, 400)
        joint_break_force: joint_break_force(300.0, 400.0, 3, false),
        ..stack_part(
            "Size3MediumTank", 4.5, 2.0, 6.0, 0.0,
            "Default, 14.7,0.7716,1.921, 14.7,0.7726,1.921, 10.92,0.8552,0.5019, 10.92,0.855,0.5019, 14.7,0.7714,1.936, 14.7,0.7714,1.936, 0,-0.002999,0, 3.75,3.928,3.75",
            vec![(Res::LiquidFuel, 3240.0, 3240.0), (Res::Oxidizer, 3960.0, 3960.0)],
            None,
        )
    }
}

/// The Mammoth's four `obj_gimbal` transforms (= its ModuleEnginesFX thrustVectorTransformName
/// and ModuleGimbal gimbalTransformName), part frame (sim, x-mirrored), from
/// Size3EngineCluster.mu: Unity (+-0.76, -1.14, +-1.38), forward -Y (exhaust down), right -X,
/// up -Z. Axis-aligned, so ModuleGimbal's per-axis clamp in the gimbal frame is a clamp of the
/// vessel-frame pitch / yaw channels.
pub const MAMMOTH_NOZZLES: [V3; 4] = [v3(0.76, -1.14, 1.38), v3(-0.76, -1.14, 1.38), v3(-0.76, -1.14, -1.38), v3(0.76, -1.14, -1.38)];

/// S3 KS-25x4 "Mammoth". Engine/Size3EngineCluster/part.cfg: mass 15, crashTolerance 20, only a
/// top stack node (size 3), ModuleEnginesFX (maxThrust 4000, no useEngineResponseTime), ModuleGimbal
/// (gimbalRange 2, no useGimbalResponseSpeed: instantaneous), ModuleAlternator (12 EC/s at full
/// thrust; not simulated). Single "Default" cube (no ModuleJettison).
///
/// Thrust transforms: four (`MAMMOTH_NOZZLES`, `thrust_transforms`), each taking
/// thrustTransformMultipliers = 1/4 (ModuleEngines.OnLoad default). The EngineDef carries their
/// centroid: with no roll input every nozzle deflects alike, and four equal parallel forces at
/// the nozzles equal one force at the centroid (same net force and torque, exactly). The gimbal
/// transform is the thrust transform, so the thrust point does not swing (pivot = thrust point).
/// Roll, which ModuleGimbal.GimbalRotation adds per nozzle from its offset, needs the nozzle
/// positions: `VesselSpec::compile` copies them into `EngineSpec::nozzles`.
pub fn mammoth() -> PartDef {
    PartDef {
        name: "Size3EngineCluster",
        mass: 15.0,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 20.0,
        body_lift: true,
        cube: cube("Default, 13.76,0.8137,1.976, 13.76,0.8137,1.976, 12.47,0.8767,3.067, 12.47,0.7577,2.59, 12.72,0.7565,2.1, 12.72,0.7576,2.1, 9.537E-07,-0.5307,0, 3.768,4.144,4.017"),
        node_top: Some((v3(0.0, 1.527248, 0.0), v3(0.0, 1.0, 0.0))),
        node_bottom: None,
        resources: vec![],
        engine: Some(EngineDef {
            max_thrust: 4000.0,
            min_thrust: 0.0,
            atmosphere_curve: Curve::from_cfg_lines(["0 315", "1 295", "12 0.001"]),
            propellants: vec![(Res::LiquidFuel, 0.9), (Res::Oxidizer, 1.1)],
            ignition_threshold: 0.1,
            response: None,
            gimbal_range_deg: 2.0,
            gimbal_response_speed: None,
            thrust_pos: v3(0.0, -1.14, 0.0),
            thrust_dir: v3(0.0, 1.0, 0.0),
            gimbal_pivot: v3(0.0, -1.14, 0.0),
        }),
        reaction_wheel: None,
        leg: None,
        hull_radius: None,
        // to the S3-7200 above: size 3, min(400, 300)
        joint_break_force: joint_break_force(400.0, 300.0, 3, false),
        // Part.collider is bodyCollider (mesh, y -1.566 .. 1.524, bottom-face radius 1.287); the
        // four engineCollider capsules (r 0.61, height 3.16, vertical, down to y -2.716) are its
        // children: they take the ground contact (`passive_contacts`) but cannot destroy the part
        hull_crash: true,
        crash_sensors: vec![
            v3(-1.287146, -1.565684, 0.0),
            v3(1.287146, -1.565684, 0.0),
            v3(0.0, -1.565684, 1.287146),
            v3(0.0, -1.565684, -1.287146),
        ],
    }
}

/// Thrust transforms of engines that have more than one (part frame, sim x-mirrored); each takes
/// an equal share of the thrust. Kept out of `EngineDef` so struct-literal users are unaffected.
pub fn thrust_transforms(name: &str) -> Option<&'static [V3]> {
    match name {
        "Size3EngineCluster" => Some(&MAMMOTH_NOZZLES),
        _ => None,
    }
}

/// Contact points on colliders other than `Part.collider` (part frame, sim x-mirrored): they
/// push on the ground but never crash the part. Mammoth: the lowest points of the four
/// engineCollider capsules (centre Unity (+-0.759, -1.136, +-1.39), half-height 1.58).
pub fn passive_contacts(name: &str) -> &'static [V3] {
    const MAMMOTH: [V3; 4] = [v3(0.759, -2.716, 1.39), v3(-0.759, -2.716, 1.39), v3(-0.759, -2.716, -1.39), v3(0.759, -2.716, -1.39)];
    match name {
        "Size3EngineCluster" => &MAMMOTH,
        _ => &[],
    }
}

/// `ModuleAeroSurface` (: ModuleControlSurface : ModuleLiftingSurface) as the game sets it up.
///
/// Frames are the sim's part frame (Unity part frame mirrored in x). Two decomp facts decide the
/// geometry:
/// - `ModuleControlSurface` re-declares `transformName` (`public new string transformName`), and
///   `BaseFieldList.ReflectedData` resolves every KSPField by name on the most derived type, so
///   the cfg's `transformName = Flap` only reaches the control-surface field (`ctrlSurface`,
///   the visual flap). `ModuleLiftingSurface.transformName` stays empty, so `baseTransform` is the
///   part transform: `liftVector` = part +Z (transformDir Z), the outward surface normal for a
///   surface attach with node_attach direction +Z.
/// - `ModuleControlSurface.FixedUpdate` turns that vector by `airflowIncidence =
///   AngleAxis(deflection, baseTransform.rotation * right)` whenever the flap's localRotation
///   differs from neutral (Unity `!=`, i.e. dot < 1 - 1e-6): lift vector = Z cos d - Y sin d.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AeroSurfaceDef {
    /// ModuleLiftingSurface.deflectionLiftCoeff (scales lift and drag curves)
    pub deflection_lift_coeff: f64,
    /// ModuleControlSurface.ctrlSurfaceRange (deg)
    pub ctrl_surface_range: f64,
    /// share of the force computed with the deflected lift vector (the rest uses the neutral one)
    pub ctrl_surface_area: f64,
    /// deg/s, Mathf.MoveTowards (useExponentialSpeed = false)
    pub actuator_speed: f64,
    /// ModuleAeroSurface.aeroDeployAngle (NaN in the cfg -> ctrlSurfaceRange at OnStart)
    pub deploy_angle: f64,
    /// ModuleLiftingSurface.useInternalDragModel
    pub internal_drag: bool,
    /// whether the part's drag-cube drag applies (dragModelType CUBE / DEFAULT); `none` makes
    /// FlightIntegrator.CalculateDragValue return 0 (the cubes then only feed thermal / area)
    pub cube_drag: bool,
    /// Physics.cfg LIFTING_SURFACE curves (`liftingSurfaceCurve`)
    pub curves: crate::aero::LiftingCurves,
    /// neutral lift vector and the direction it turns toward with positive deflection (part frame)
    pub lift_vector: V3,
    pub lift_turn: V3,
    /// IMultipleDragCube positions: "neutral" (= PartDef.cube), "fullDeflectionPos" / "Neg"
    /// (flap at +-1.5 * ctrlSurfaceRange, `AssumeDragCubePosition`)
    pub cube_pos: DragCube,
    pub cube_neg: DragCube,
}

impl AeroSurfaceDef {
    /// The blended cube for a flap deflection (deg), as `SetCubeWeight` + `SetDragWeights_List`.
    pub fn cube(&self, neutral: &DragCube, deflection: f64) -> DragCube {
        let w = crate::aero::control_surface_cube_weights(deflection, self.ctrl_surface_range);
        DragCube::blend(&[(w[0], *neutral), (w[1], self.cube_pos), (w[2], self.cube_neg)])
    }
}

/// A.I.R.B.R.A.K.E.S. Aero/airbrake/Airbrake.cfg (mass 0.05, crashTolerance 8, dragModelType
/// none, angularDrag 2, CoM/CoP/CoL offsets commented out = 0; node_attach 0,0,-0.025 dir +Z);
/// neutral cube PartDatabase.cfg (airbrake1). ModuleLiftingSurface.disableBodyLift defaults to
/// true (Part.hasLiftModule), so no body lift. The module data is `aero_surface("airbrake1")`.
pub fn airbrake() -> PartDef {
    PartDef {
        name: "airbrake1",
        mass: 0.05,
        com_offset: V3::ZERO,
        cop_offset: V3::ZERO,
        col_offset: V3::ZERO,
        angular_drag: 2.0,
        crash_tolerance: 8.0,
        body_lift: false,
        cube: cube("neutral, 0.3767,0.5233,0.525, 0.3767,0.527,0.525, 0.1066,0.3505,0.9374, 0.1066,0.4695,1.975, 1.453,0.9112,0.2996, 1.453,0.9087,0.2613, -5.96E-08,-0.6985,0.01376, 0.8542,2.343,0.2244"),
        node_top: None,
        node_bottom: None,
        resources: vec![],
        engine: None,
        reaction_wheel: None,
        leg: None,
        hull_radius: None,
        // not measured (surface-attach joint)
        joint_break_force: 4400.0,
        hull_crash: true,
        crash_sensors: vec![],
    }
}

/// Airbrake srfAttach node (Airbrake.cfg node_attach, rescaleFactor 1): position, direction.
pub const AIRBRAKE_NODE: (V3, V3) = (v3(0.0, 0.0, -0.025), v3(0.0, 0.0, 1.0));

/// `ModuleAeroSurface` data by part name (kept out of `PartDef` so struct-literal users of
/// `PartDef` are unaffected). Airbrake.cfg: dragCoeff 0.6 is not read by the module.
pub fn aero_surface(name: &str) -> Option<AeroSurfaceDef> {
    match name {
        "airbrake1" => Some(AeroSurfaceDef {
            deflection_lift_coeff: 0.38,
            ctrl_surface_range: 70.0,
            ctrl_surface_area: 1.0,
            actuator_speed: 20.0,
            deploy_angle: 70.0,
            internal_drag: true,
            cube_drag: false,
            curves: crate::aero::SPEED_BRAKE,
            lift_vector: v3(0.0, 0.0, 1.0),
            lift_turn: v3(0.0, -1.0, 0.0),
            cube_pos: cube("fullDeflectionPos, 0.4561,0.5978,0.525, 0.4561,0.5972,0.525, 1.338,0.8739,0.8227, 1.338,0.8747,1.589, 0.554,0.4864,2.113, 0.554,0.6162,2.122, 1.192E-07,-0.08187,0.9338, 0.8542,1.608,2.064"),
            cube_neg: cube("fullDeflectionNeg, 0.4447,0.5905,0.525, 0.4447,0.5898,0.525, 1.326,0.8441,0.8366, 1.326,0.8877,1.637, 0.5599,0.5775,2.105, 0.5599,0.5102,1.999, -8.941E-08,-0.07445,-0.8939, 0.8542,1.623,2.046"),
        }),
        _ => None,
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
        "probeStackSmall" => probe_stack_small(),
        "batteryBank" => battery_bank(),
        "advSasModule" => adv_sas_module(),
        "Decoupler_1" => decoupler_td12(),
        "airbrake1" => airbrake(),
        "Size3EngineCluster" => mammoth(),
        "Size3LargeTank" => s3_tank_14400(),
        "Size3MediumTank" => s3_tank_7200(),
        "Size3To2Adapter_v2" => adapter_s3_s2(),
        "probeStackLarge" => probe_stack_large(),
        "asasmodule1-2" => adv_sas_module_large(),
        "batteryBankLarge" => battery_bank_large(),
        "rocketNoseCone_v2" => nose_cone_mk7(),
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
            ("Utility/landingLegLT-2/landingLegLT-2.cfg", lt2_leg_retracted(), "Retracted"),
            ("Aero/airbrake/Airbrake.cfg", airbrake(), "neutral"),
        ];
        check_parts(&parts, &pdb, &cases);
        // ModuleAeroSurface fields and the deflected cubes
        let p = part_node(&parts.join("Aero/airbrake/Airbrake.cfg"));
        let a = aero_surface("airbrake1").unwrap();
        assert_eq!(p.get("dragModelType"), Some("none"));
        assert!(!a.cube_drag);
        assert_eq!(p.get_f64("angularDrag"), Some(airbrake().angular_drag));
        let m = p.nodes_named("MODULE").find(|n| n.get("name") == Some("ModuleAeroSurface")).unwrap();
        assert_eq!(m.get_f64("deflectionLiftCoeff"), Some(a.deflection_lift_coeff));
        assert_eq!(m.get_f64("ctrlSurfaceRange"), Some(a.ctrl_surface_range));
        assert_eq!(m.get_f64("ctrlSurfaceArea"), Some(a.ctrl_surface_area));
        assert_eq!(m.get_f64("actuatorSpeed"), Some(a.actuator_speed));
        assert_eq!(m.get("liftingSurfaceCurve"), Some("SpeedBrake"));
        assert_eq!(m.get("useInternalDragModel"), Some("True"));
        assert_eq!(m.get("ignorePitch"), Some("true"));
        assert_eq!(m.get("ignoreYaw"), Some("true"));
        // no aeroDeployAngle / transformDir / omnidirectional / perpendicularOnly overrides
        for k in ["aeroDeployAngle", "transformDir", "transformSign", "omnidirectional", "perpendicularOnly", "useExponentialSpeed", "displaceVelocity", "brakeDeployInvert"] {
            assert!(m.get(k).is_none(), "{k}");
        }
        let v = vec3(p.get("node_attach").unwrap());
        assert_eq!((v3(v[0], v[1], v[2]), v3(v[3], v[4], v[5])), AIRBRAKE_NODE);
        let dc = pdb.nodes_named("PART").find(|n| n.get("url").is_some_and(|u| u.ends_with("/airbrake1"))).and_then(|n| n.node("DRAG_CUBE").cloned()).unwrap();
        let cubes: Vec<(String, DragCube)> = dc.get_all("cube").filter_map(DragCube::parse).collect();
        let find = |n: &str| cubes.iter().find(|(c, _)| c == n).unwrap().1.mirrored_x();
        assert_eq!(find("fullDeflectionPos"), a.cube_pos);
        assert_eq!(find("fullDeflectionNeg"), a.cube_neg);
    }

    /// The parts of the in-game demo booster that only the demo install's PartDatabase was used
    /// for (~/ksp-demo/KSP or $KSP_DEMO; skipped when absent).
    #[test]
    fn matches_demo_install() {
        let k = std::env::var("KSP_DEMO").map(std::path::PathBuf::from).unwrap_or_else(|_| std::path::Path::new(&std::env::var("HOME").unwrap_or_default()).join("ksp-demo/KSP"));
        if !k.join("PartDatabase.cfg").exists() {
            return;
        }
        let parts = k.join("GameData/Squad/Parts");
        let pdb = cfg::parse(&std::fs::read_to_string(k.join("PartDatabase.cfg")).unwrap());
        let cases = [
            ("Command/probeStackSmall/probeStackSmall.cfg", probe_stack_small(), "Default"),
            ("Electrical/z-1kBattery/z-1kBattery.cfg", battery_bank(), "Default"),
            ("Command/inlineAdvancedStabilizer/inlineAdvancedStabilizer.cfg", adv_sas_module(), "Default"),
            ("Coupling/Decoupler_1.cfg", decoupler_td12(), "Default"),
            ("Aero/airbrake/Airbrake.cfg", airbrake(), "neutral"),
        ];
        check_parts(&parts, &pdb, &cases);
        for (rel, def, _) in &cases {
            let p = part_node(&parts.join(rel));
            assert_eq!(p.get_f64("angularDrag"), Some(def.angular_drag), "{}", def.name);
            assert_eq!(p.get_f64("rescaleFactor").unwrap_or(1.25), 1.0, "{}", def.name);
        }
    }

    fn check_parts(parts: &std::path::Path, pdb: &cfg::Node, cases: &[(&str, PartDef, &str)]) {
        for (rel, def, cube_name) in cases {
            let (def, cube_name) = (def.clone(), *cube_name);
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
            if let Some(own) = p.node("DRAG_CUBE") {
                // the part cfg's own cube is what the prefab (and so every flight part) carries;
                // PartDatabase holds a 4-significant-digit copy of it
                let c = own.get_all("cube").filter_map(DragCube::parse).find(|(n, _)| n == cube_name).unwrap().1;
                assert_eq!(c.mirrored_x(), def.cube, "{}", def.name);
                let flat = |d: &DragCube| [&d.area[..], &d.drag[..], &d.depth[..], &[d.center.x, d.center.y, d.center.z, d.size.x, d.size.y, d.size.z][..]].concat();
                for (x, y) in flat(&c).iter().zip(flat(&found)) {
                    assert!((x - y).abs() <= 5e-4 * x.abs() + 1e-9, "{}: cfg {x} vs PartDatabase {y}", def.name);
                }
            } else {
                assert_eq!(found.mirrored_x(), def.cube, "{}", def.name);
            }
        }
    }

    fn demo_install() -> Option<std::path::PathBuf> {
        let k = std::env::var("KSP_DEMO").map(std::path::PathBuf::from).unwrap_or_else(|_| std::path::Path::new(&std::env::var("HOME").unwrap_or_default()).join("ksp-demo/KSP"));
        k.join("PartDatabase.cfg").exists().then_some(k)
    }

    fn module<'a>(p: &'a cfg::Node, name: &str) -> Option<&'a cfg::Node> {
        p.nodes_named("MODULE").find(|n| n.get("name") == Some(name))
    }

    type PartFn = fn() -> PartDef;
    const HEAVY: [(&str, PartFn); 8] = [
        ("Engine/Size3EngineCluster/part.cfg", mammoth),
        ("FuelTank/Size3Tanks/large.cfg", s3_tank_14400),
        ("FuelTank/Size3Tanks/medium.cfg", s3_tank_7200),
        ("Structural/Size3To2Adapter_v2/Size2to3_v2.cfg", adapter_s3_s2),
        ("Command/probeStackLarge/probeStackLarge.cfg", probe_stack_large),
        ("Command/advancedSasModuleLarge/advSasModuleLarge.cfg", adv_sas_module_large),
        ("Electrical/z-4kBattery/z-4kBattery.cfg", battery_bank_large),
        ("Aero/cones/protectiveRocketNoseMk7_v2.cfg", nose_cone_mk7),
    ];

    /// The 3.75 m booster parts against the demo install's cfgs and PartDatabase (skipped without
    /// it), including the fields `check_parts` does not cover.
    #[test]
    fn heavy_parts_match_demo_install() {
        let Some(k) = demo_install() else { return };
        let parts = k.join("GameData/Squad/Parts");
        let pdb = cfg::parse(&std::fs::read_to_string(k.join("PartDatabase.cfg")).unwrap());
        let cases: Vec<(&str, PartDef, &str)> = HEAVY.iter().map(|(rel, f)| (*rel, f(), "Default")).collect();
        check_parts(&parts, &pdb, &cases);
        for (rel, def, _) in &cases {
            let p = part_node(&parts.join(rel));
            assert_eq!(by_name(def.name).unwrap().name, def.name);
            assert_eq!(p.get_f64("angularDrag"), Some(def.angular_drag), "{}", def.name);
            assert_eq!(p.get_f64("rescaleFactor"), Some(1.0), "{}", def.name);
            assert_eq!(p.get("node_stack_top").is_some(), def.node_top.is_some(), "{}", def.name);
            assert_eq!(p.get("node_stack_bottom").is_some(), def.node_bottom.is_some(), "{}", def.name);
            for k in ["CoMOffset", "CoPOffset", "CoLOffset"] {
                assert!(p.get(k).is_none(), "{} {k}", def.name);
            }
            let n_res = p.nodes_named("RESOURCE").count();
            assert_eq!(n_res, def.resources.len(), "{}", def.name);
            // one cube for every variant
            if let Some(m) = module(&p, "ModulePartVariants") {
                assert_eq!(m.get("useMultipleDragCubes"), Some("false"), "{}", def.name);
            }
            assert!(module(&p, "ModuleLiftingSurface").is_none());
            if let Some(w) = &def.reaction_wheel {
                let m = module(&p, "ModuleReactionWheel").unwrap();
                assert_eq!(m.get_f64("YawTorque"), Some(w.yaw_torque));
                assert_eq!(m.get_f64("RollTorque"), Some(w.roll_torque));
                assert_eq!(m.get_f64("torqueResponseSpeed").unwrap_or(30.0), w.torque_response_speed);
            } else {
                assert!(module(&p, "ModuleReactionWheel").is_none(), "{}", def.name);
            }
        }
        // joint break forces from the cfg breakingForce (Part default 22) and the node sizes
        let cfgp = |i: usize| part_node(&parts.join(HEAVY[i].0));
        let bf = |i: usize| cfgp(i).get_f64("breakingForce").unwrap_or(DEFAULT_BREAKING_FORCE);
        let size = |i: usize, key: &str| vec3(cfgp(i).get(key).unwrap()).get(6).map_or(1, |s| *s as u32);
        let jbf = |a: usize, ka: &str, b: usize, kb: &str| joint_break_force(bf(a), bf(b), size(a, ka).min(size(b, kb)), false);
        let (eng, large, med, adp, probe, sas, bat, nose) = (0, 1, 2, 3, 4, 5, 6, 7);
        assert_eq!(mammoth().joint_break_force, jbf(eng, "node_stack_top", med, "node_stack_bottom"));
        assert_eq!(s3_tank_7200().joint_break_force, jbf(med, "node_stack_top", large, "node_stack_bottom"));
        assert_eq!(s3_tank_14400().joint_break_force, jbf(large, "node_stack_top", large, "node_stack_bottom"));
        assert_eq!(adapter_s3_s2().joint_break_force, jbf(adp, "node_stack_bottom", large, "node_stack_top"));
        assert_eq!(battery_bank_large().joint_break_force, jbf(bat, "node_stack_bottom", adp, "node_stack_top"));
        assert_eq!(adv_sas_module_large().joint_break_force, jbf(sas, "node_stack_bottom", bat, "node_stack_top"));
        assert_eq!(probe_stack_large().joint_break_force, jbf(probe, "node_stack_bottom", sas, "node_stack_top"));
        assert_eq!(nose_cone_mk7().joint_break_force, jbf(nose, "node_stack_bottom", probe, "node_stack_top"));
        // Mammoth modules
        let p = cfgp(eng);
        let e = mammoth().engine.unwrap();
        let m = module(&p, "ModuleEnginesFX").unwrap();
        assert_eq!(m.get_f64("minThrust"), Some(e.min_thrust));
        assert_eq!(m.get_f64("ignitionThreshold"), Some(e.ignition_threshold));
        assert!(m.get("useEngineResponseTime").is_none() && m.node("transformMultipliers").is_none());
        let props: Vec<(&str, f64)> = m.nodes_named("PROPELLANT").map(|q| (q.get("name").unwrap(), q.get_f64("ratio").unwrap())).collect();
        assert_eq!(props, vec![("LiquidFuel", 0.9), ("Oxidizer", 1.1)]);
        let g = module(&p, "ModuleGimbal").unwrap();
        // the gimbal transforms are the thrust transforms (no swing about a separate pivot)
        assert_eq!(m.get("thrustVectorTransformName"), Some("obj_gimbal"));
        assert_eq!(g.get("gimbalTransformName"), Some("obj_gimbal"));
        for k in ["useGimbalResponseSpeed", "gimbalRangeXP", "gimbalRangeXN", "gimbalRangeYP", "gimbalRangeYN", "xMult", "yMult", "flipYZ", "enableRoll"] {
            assert!(g.get(k).is_none(), "{k}");
        }
        assert!(module(&p, "ModuleAlternator").is_some());
        assert!(module(&p, "ModuleJettison").is_none());
        // the model holds four obj_gimbal transforms (PartReader string table: names are stored
        // length-prefixed, so count the exact byte pattern)
        let mu = std::fs::read(parts.join("Engine/Size3EngineCluster/Size3EngineCluster.mu")).unwrap();
        let pat = b"\x0aobj_gimbal";
        assert_eq!(mu.windows(pat.len()).filter(|w| w == pat).count(), thrust_transforms("Size3EngineCluster").unwrap().len());
    }

    #[test]
    fn joint_break_force_reproduces_dumps() {
        // dumped by SimHarness: Mk1 pod (22) / FL-T400 (50) stack joint, LT-2 (80) on FL-T400
        assert_eq!(joint_break_force(DEFAULT_BREAKING_FORCE, 50.0, 1, false), 4400.0);
        assert_eq!(joint_break_force(80.0, 50.0, 1, true), lt2_leg().joint_break_force);
        assert_eq!(JBF_SIZE2, joint_break_force(DEFAULT_BREAKING_FORCE, DEFAULT_BREAKING_FORCE, 2, false));
    }

    use crate::math::Quat;
    use crate::sim::{self, Control, EnvState, Frame, Params};
    use crate::vessel::{Craft, Placed, VesselSpec, MAX_HULL};

    fn place_above(c: &mut Craft, below: usize, def: PartDef) -> usize {
        let p = &c.parts[below];
        let (pt, ptd) = p.def.node_top.unwrap();
        let (cb, cbd) = def.node_bottom.unwrap();
        let (pos, rot, i) = (p.pos + p.rot.rotate(pt) - cb, p.rot, c.parts.len());
        c.parts[below].stack.push((i, ptd));
        c.parts.push(Placed { def, pos, rot, stack: vec![(below, cbd)], parent: Some(below) });
        i
    }

    /// The heavy booster as guidance builds it (ingame::heavy_craft: root S3-14400, legs on the
    /// S3-7200 4.3 m below its centre, airbrakes 3 m above the S3-14400 centre).
    fn heavy_craft(legs: bool, brakes: bool) -> Craft {
        let mut c = Craft { parts: vec![Placed { def: s3_tank_14400(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![], parent: None }] };
        let med = c.stack_below(0, s3_tank_7200());
        c.stack_below(med, mammoth());
        let mut top = place_above(&mut c, 0, adapter_s3_s2());
        for d in [battery_bank_large(), adv_sas_module_large(), probe_stack_large(), nose_cone_mk7()] {
            top = place_above(&mut c, top, d);
        }
        let (np, nd) = (v3(0.0, 0.4271562 * 1.25, 0.0632596 * 1.25), v3(0.0, 0.5735765, 0.819152));
        for k in 0..6 {
            if legs {
                c.surface_attach_editor(med, lt2_leg(), 30.0 + 60.0 * k as f64, 1.875, -4.3, np, nd);
            }
        }
        for k in 0..8 {
            if brakes {
                c.surface_attach_editor(0, airbrake(), 45.0 * k as f64, 1.875, 3.0, AIRBRAKE_NODE.0, AIRBRAKE_NODE.1);
            }
        }
        c
    }

    /// The 22-part booster fits the VesselSpec limits without dropping anything.
    #[test]
    fn heavy_booster_fits_limits() {
        let c = heavy_craft(true, true);
        assert_eq!(c.parts.len(), 22);
        let s = VesselSpec::compile(&c);
        assert_eq!((s.n_parts, s.n_engines, s.n_legs, s.n_aero_surf), (22, 1, 6, 8));
        // rings + top centres: nose 4, five 2.5 m / adapter / tank parts 5 each, S3-14400 5,
        // Mammoth 1 top + 4 body sensors + 4 nozzle capsules (passive)
        let expect = 4 + 5 * 6 + 1 + 4 + 4;
        assert_eq!(s.n_hull as usize, expect);
        assert!(expect <= MAX_HULL);
        assert_eq!((0..s.n_hull as usize).filter(|&k| !s.hull_crash[k]).count(), 4 + 4, "nose ring + nozzles never crash");
        let e = &s.engines[0];
        assert_eq!(e.n_nozzles, 4);
        // mass and resources
        let dry: f64 = c.parts.iter().map(|p| p.def.mass).sum();
        assert!((s.dry_mass() - dry).abs() < 1e-9);
        assert!((dry - (15.0 + 9.0 + 4.5 + 1.875 + 0.2 + 0.2 + 0.5 + 0.2 + 6.0 * 0.1 + 8.0 * 0.05)).abs() < 1e-9);
        let lf = 6480.0 + 3240.0 + 1350.0;
        let ox = 7920.0 + 3960.0 + 1650.0;
        assert_eq!(s.res_cap_total[Res::LiquidFuel as usize], lf);
        assert_eq!(s.res_cap_total[Res::Oxidizer as usize], ox);
        assert_eq!(s.res_cap_total[Res::ElectricCharge as usize], 30.0 + 4000.0);
        let (m, _, _) = s.mass_props(&s.full_resources());
        assert!((m - (dry + (lf + ox) * DENSITY_LIQUID_FUEL)).abs() < 1e-9);
        // reaction wheels sum: RC-L01 1.5 + large wheel 30
        assert_eq!(s.rw_torque, v3(31.5, 31.5, 31.5));
        assert!((s.rw_ec_rate - 0.75).abs() < 1e-12);
    }

    /// Mammoth thrust at sea level and in vacuum (Isp 295 / 315 s, maxThrust 4000 at Isp(0)).
    #[test]
    fn mammoth_thrust_asl_vacuum() {
        let s = VesselSpec::compile(&heavy_craft(false, false));
        let e = &s.engines[0];
        let mut res = s.full_resources();
        let (t_vac, out, _) = crate::engine::burn(e, 1.0, 0.0, &mut res, 0.02);
        let (t_asl, _, _) = crate::engine::burn(e, 1.0, 1.0, &mut res, 0.02);
        assert!(!out);
        assert!((t_vac - 4000.0).abs() < 1e-3, "{t_vac}");
        assert!((t_asl - 4000.0 * 295.0 / 315.0).abs() < 1e-3, "{t_asl}");
        // propellant draw: maxFuelFlow = 4000 / (315 g0) t/s split LF:Ox 0.9:1.1 by volume
        let flow = 4000.0 / (315.0 * crate::engine::ENGINE_G0);
        assert!((e.max_fuel_flow - flow).abs() < 1e-12);
    }

    /// One tick in vacuum from rest (q = identity, w = 0): the angular velocity change is
    /// I^-1 * torque * dt, so torques compare through it.
    fn spin_up(s: &VesselSpec, ctl: &Control) -> V3 {
        let r0 = crate::body::KERBIN.radius + 400_000.0;
        let mut st = EnvState { frame: Frame::Inertial, x: v3(r0, 0.0, 0.0), v: v3(0.0, (crate::body::KERBIN.gm() / r0).sqrt(), 0.0), q: Quat::IDENTITY, res: s.full_resources(), ..Default::default() };
        sim::tick(s, &Params::default(), &mut st, ctl);
        st.w
    }

    fn torque_of(s: &VesselSpec, dw: V3) -> V3 {
        let (_, _, i) = s.mass_props(&s.full_resources());
        let r = [i[0][0] * dw.x + i[0][1] * dw.y + i[0][2] * dw.z, i[1][0] * dw.x + i[1][1] * dw.y + i[1][2] * dw.z, i[2][0] * dw.x + i[2][1] * dw.y + i[2][2] * dw.z];
        v3(r[0], r[1], r[2]) / sim::FIXED_DT
    }

    /// Four nozzles vs the single-nozzle equivalent at their centroid: identical pitch / yaw
    /// response (exact for equal deflections), the same signs as the reaction wheels, and the roll
    /// torque ModuleGimbal's per-nozzle roll mix makes (none for the single nozzle).
    #[test]
    fn mammoth_gimbal_four_nozzles() {
        let mut s4 = VesselSpec::compile(&heavy_craft(false, false));
        // gimbal only: no reaction wheels
        let rw = (s4.rw_torque, s4.rw_ec_rate);
        s4.rw_torque = V3::ZERO;
        s4.rw_ec_rate = 0.0;
        let mut s1 = s4;
        s1.engines[0].n_nozzles = 0;
        let mut s_rw = s4;
        (s_rw.rw_torque, s_rw.rw_ec_rate) = rw;
        let ctl = |throttle: f64, pitch: f64, yaw: f64, roll: f64| Control { throttle, pitch, yaw, roll };
        let base = spin_up(&s4, &ctl(1.0, 0.0, 0.0, 0.0));
        assert!(base.norm() < 1e-9, "straight thrust through the axis: {base:?}");
        for (p, y) in [(1.0, 0.0), (0.0, 1.0), (-0.6, 0.35), (0.3, -1.0)] {
            let w4 = spin_up(&s4, &ctl(1.0, p, y, 0.0));
            let w1 = spin_up(&s1, &ctl(1.0, p, y, 0.0));
            assert!((w4 - w1).norm() < 1e-9 * w1.norm().max(1e-12) + 1e-15, "{p} {y}: {w4:?} vs {w1:?}");
        }
        let (_, com, _) = s4.mass_props(&s4.full_resources());
        let h = s4.engines[0].pos.y - com.y;
        let t = 4000.0 * crate::engine::isp(&s4.engines[0], 0.0) / 315.0;
        let sin2 = 2f64.to_radians().sin();
        // pitch / yaw / roll: torque about root X / Z / Y, sign as the wheels' for the same input
        for (axis, c, expect) in [(0, ctl(1.0, 1.0, 0.0, 0.0), -h * t * sin2), (2, ctl(1.0, 0.0, 1.0, 0.0), -h * t * sin2), (1, ctl(1.0, 0.0, 0.0, 1.0), t * (0.76 + 1.38) * sin2)] {
            let tq = torque_of(&s4, spin_up(&s4, &c));
            let tq = [tq.x, tq.y, tq.z][axis];
            let wheel = spin_up(&s_rw, &Control { throttle: 0.0, ..c });
            let wheel = [wheel.x, wheel.y, wheel.z][axis];
            assert!(tq.signum() == wheel.signum() && wheel != 0.0, "axis {axis}: gimbal {tq} vs wheel {wheel}");
            assert!((tq.abs() - expect.abs()).abs() < 0.01 * expect.abs(), "axis {axis}: {tq} vs {expect}");
        }
        let w1 = spin_up(&s1, &ctl(1.0, 0.0, 0.0, 1.0));
        assert!(w1.norm() < 1e-9, "single nozzle has no roll authority: {w1:?}");
        // roll saturates the per-nozzle channel: with full pitch the nozzles on one side clamp
        let both = torque_of(&s4, spin_up(&s4, &ctl(1.0, 1.0, 0.0, 1.0)));
        let roll_only = torque_of(&s4, spin_up(&s4, &ctl(1.0, 0.0, 0.0, 1.0)));
        assert!(both.y.abs() < roll_only.y.abs());
    }

    /// Mammoth + S3 tanks + six LT-2 legs (the heavy booster): settles on the pad without a
    /// crash, then a 3 s full-throttle hop accelerates at thrust / mass - g.
    #[test]
    fn heavy_booster_pad_and_hop() {
        let s = VesselSpec::compile(&heavy_craft(true, true));
        let prm = Params::default();
        let mut st = sim::state_landed(&s, 0.0);
        let idle = Control::default();
        for _ in 0..250 {
            sim::tick(&s, &prm, &mut st, &idle);
        }
        assert_ne!(st.status, sim::STATUS_CRASHED);
        assert!(st.v.norm() < 0.05 && st.w.norm() < 0.01, "{:?} {:?}", st.v, st.w);
        let (_, _, up) = sim::pad_enu();
        let full = Control { throttle: 1.0, ..Default::default() };
        let mut v_prev = st.v.dot(up);
        let mut worst: f64 = 0.0;
        for i in 0..150 {
            let info = sim::tick(&s, &prm, &mut st, &full);
            assert_ne!(st.status, sim::STATUS_CRASHED);
            let v = st.v.dot(up);
            if i > 25 {
                // vertical: thrust / m - g (rotating frame: centrifugal ~0.03 m/s^2 at the pad)
                let r = st.x.norm();
                let g = crate::body::KERBIN.gm() / (r * r);
                let a_exp = info.thrust / info.mass - g;
                worst = worst.max(((v - v_prev) / sim::FIXED_DT - a_exp).abs() / a_exp);
            }
            v_prev = v;
        }
        // 155.475 t wet (32.475 dry + 123 propellant): sea-level TWR 3746 kN / 1525 kN = 2.456
        let m0 = s.mass_props(&s.full_resources()).0;
        assert!((m0 - 155.475).abs() < 1e-9, "{m0}");
        let twr = 4000.0 * 295.0 / 315.0 / (m0 * 9.81);
        assert!((twr - 2.456).abs() < 1e-3, "{twr}");
        assert!(worst < 0.02, "acceleration error {worst}");
        assert!(st.v.dot(up) > 30.0 && st.w.norm() < 0.01, "{:?} {:?}", st.v, st.w);
    }

    /// trace::spec_from_geometry with a dumped Mammoth: its four thrustTransform lines make one
    /// engine with four nozzles (Unity -> sim x mirror), and a `joints=3` count scales the dumped
    /// per-joint breakForce to the sum the catalog carries.
    #[test]
    fn geometry_dump_multi_nozzle_engine() {
        let mut g = String::from("test = synthetic\nPART {\n idx = 0\n name = Size3MediumTank\n parent = -1\n localPos = 0,0,0\n localRot = 0,0,0,1\n}\n");
        g += "PART {\n idx = 1\n name = Size3EngineCluster\n parent = 0\n localPos = 0,-3.464248,0\n localRot = 0,0,0,1\n";
        g += " joint = ConfigurableJoint breakForce=40000 joints=3 rigid=False\n";
        for (x, z) in [(-0.76, 1.38), (0.76, 1.38), (0.76, -1.38), (-0.76, -1.38)] {
            g += &format!(" thrustTransform = pos={x},-4.604248,{z} fwd=0,-1,0 maxThrust=4000 maxFuelFlow=1.2948 g=9.80665 useResponse=False\n");
            g += &format!(" gimbalTransform = pos={x},-4.604248,{z} rot=0,0.7071068,-0.7071068,0 initRot=0,0,-0.7071068,0.7071068 range=2 XP=2 XN=2 YP=2 YN=2 useSpeed=False speed=10\n");
        }
        g += "}\n";
        let (_, geom) = crate::trace::parse_geometry(&g);
        let s = crate::trace::spec_from_geometry(&geom);
        assert_eq!(s.n_engines, 1);
        let e = &s.engines[0];
        assert_eq!(e.n_nozzles, 4);
        assert!((e.pos - v3(0.0, -4.604248, 0.0)).norm() < 1e-12 && e.pivot == e.pos);
        assert!((e.dir - v3(0.0, 1.0, 0.0)).norm() < 1e-12);
        assert_eq!(e.max_fuel_flow, 1.2948);
        // Unity (-0.76, z 1.38) is sim (+0.76, 1.38): roll adds to pitch with -1 (Unity x < 0), to yaw with +1
        assert_eq!(e.nozzles[0], v3(0.76, -4.604248, 1.38));
        assert_eq!(e.nozzle_roll[0], [-1.0, 1.0]);
        // same nozzles as the catalog's, placed below the S3-7200
        let c = VesselSpec::compile(&{
            let mut c = Craft { parts: vec![Placed { def: s3_tank_7200(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![], parent: None }] };
            c.stack_below(0, mammoth());
            c
        });
        for j in 0..4 {
            assert!((c.engines[0].nozzles[j] - e.nozzles[j]).norm() < 1e-9, "{j}");
            assert_eq!(c.engines[0].nozzle_roll[j], e.nozzle_roll[j]);
        }
        assert_eq!(s.part_break_force[1], 120000.0);
        assert_eq!(s.part_break_force[1], mammoth().joint_break_force);
    }
}
