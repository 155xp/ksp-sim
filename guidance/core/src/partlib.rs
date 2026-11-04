//! Stock parts loaded at run time from the local KSP install (`GameData/Squad/Parts/**.cfg` and
//! `PartDatabase.cfg`), using ksp-core's ConfigNode parser. Nothing from the game is copied into
//! this repository; the loader reads the same fields ksp-core's hand transcriptions use.
//!
//! The hopper parts that ksp-core transcribes (pod, FL-T400, Swivel, Terrier, LT-1, LT-2) come
//! from ksp-core directly; `tests::matches_core_transcription` checks this loader reproduces them.
//! Landing legs always come from ksp-core: their foot point is a ksp-core approximation (the wheel
//! collider lives in the model file), not a cfg value.
//!
//! Model-file approximations shared with ksp-core: the thrust transform sits at the engine's bottom
//! stack node, thrusting along part +Y.

use ksp_core::aero::DragCube;
use ksp_core::cfg::{self, Node};
use ksp_core::math::{v3, V3};
use ksp_core::parts::{self, EngineDef, PartDef, ReactionWheelDef, Res};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// KSP install root: `$KSP_DIR`, else the Steam (Debian) default path.
pub fn install_dir() -> Option<PathBuf> {
    if let Ok(d) = std::env::var("KSP_DIR") {
        let p = PathBuf::from(d);
        return p.exists().then_some(p);
    }
    let p = Path::new(&std::env::var("HOME").ok()?).join(".steam/debian-installation/steamapps/common/Kerbal Space Program");
    p.exists().then_some(p)
}

struct Library {
    /// part name -> PART node
    parts: HashMap<String, Node>,
    /// part name -> DRAG_CUBE cubes (name, cube)
    cubes: HashMap<String, Vec<(String, DragCube)>>,
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            walk(&p, out);
        } else if p.extension().is_some_and(|x| x == "cfg") {
            out.push(p);
        }
    }
}

fn load_library() -> Result<Library, String> {
    let k = install_dir().ok_or("KSP install not found (set KSP_DIR)")?;
    let mut files = vec![];
    walk(&k.join("GameData/Squad/Parts"), &mut files);
    let mut parts = HashMap::new();
    for f in files {
        let Ok(t) = std::fs::read_to_string(&f) else { continue };
        for p in cfg::parse(&t).nodes_named("PART") {
            if let Some(n) = p.get("name") {
                parts.insert(n.replace('_', "."), p.clone());
                parts.insert(n.to_string(), p.clone());
            }
        }
    }
    let pdb = std::fs::read_to_string(k.join("PartDatabase.cfg")).map_err(|e| format!("PartDatabase.cfg: {e}"))?;
    let mut cubes = HashMap::new();
    for p in cfg::parse(&pdb).nodes_named("PART") {
        let Some(url) = p.get("url") else { continue };
        let name = url.rsplit('/').next().unwrap_or(url).to_string();
        if let Some(dc) = p.node("DRAG_CUBE") {
            let v: Vec<_> = dc.get_all("cube").filter_map(DragCube::parse).collect();
            cubes.insert(name, v);
        }
    }
    Ok(Library { parts, cubes })
}

fn library() -> Result<&'static Library, String> {
    static LIB: OnceLock<Result<Library, String>> = OnceLock::new();
    LIB.get_or_init(load_library).as_ref().map_err(|e| e.clone())
}

fn vec3(s: &str) -> Vec<f64> {
    s.split(',').filter_map(|t| t.trim().parse().ok()).collect()
}

fn get_v3(n: &Node, k: &str) -> Option<V3> {
    let v = vec3(n.get(k)?);
    (v.len() >= 3).then(|| v3(v[0], v[1], v[2]))
}

fn res_of(name: &str) -> Option<Res> {
    Some(match name {
        "LiquidFuel" => Res::LiquidFuel,
        "Oxidizer" => Res::Oxidizer,
        "MonoPropellant" => Res::MonoPropellant,
        "ElectricCharge" => Res::ElectricCharge,
        _ => return None,
    })
}

fn truthy(n: &Node, k: &str) -> bool {
    n.get(k).is_some_and(|v| v.trim().eq_ignore_ascii_case("true"))
}

/// Build a `PartDef` from the install's cfg + PartDatabase drag cube.
///
/// Cube choice: "Clean" (engines: nothing on the bottom node, as ksp-core's Swivel), else
/// "Default", else the first listed cube (variant 0).
pub fn load(name: &str) -> Result<PartDef, String> {
    match name {
        // ksp-core's transcriptions (incl. its leg foot approximations)
        "landingLeg1" => return Ok(parts::lt1_leg()),
        "landingLeg1-2" => return Ok(parts::lt2_leg()),
        _ => {}
    }
    let lib = library()?;
    let p = lib.parts.get(name).ok_or_else(|| format!("part {name} not in install"))?;
    // PartLoader: node positions scale with rescaleFactor (default 1.25)
    let scale = p.get_f64("rescaleFactor").unwrap_or(1.25);
    // `node_stack_bottom`, or the first numbered variant (`node_stack_bottom01`)
    let node = |k: &str| -> Option<(V3, V3)> {
        let raw = p.get(k).or_else(|| p.values.iter().find(|(n, _)| n.starts_with(k)).map(|(_, v)| v.as_str()))?;
        let v = vec3(raw);
        (v.len() >= 6).then(|| (v3(v[0], v[1], v[2]) * scale, v3(v[3], v[4], v[5]).normalized()))
    };
    let cl = lib.cubes.get(name).ok_or_else(|| format!("no drag cube for {name}"))?;
    let cube = cl
        .iter()
        .find(|(n, _)| n == "Clean")
        .or_else(|| cl.iter().find(|(n, _)| n == "Default"))
        .or_else(|| cl.first())
        .ok_or_else(|| format!("empty drag cube list for {name}"))?
        .1;
    let mut resources = vec![];
    for r in p.nodes_named("RESOURCE") {
        if let (Some(rn), Some(max)) = (r.get("name").and_then(res_of), r.get_f64("maxAmount")) {
            resources.push((rn, r.get_f64("amount").unwrap_or(max), max));
        }
    }
    let node_bottom = node("node_stack_bottom");
    let modules: Vec<&Node> = p.nodes_named("MODULE").collect();
    let module = |m: &str| modules.iter().copied().find(|n| n.get("name") == Some(m));
    let engine = modules
        .iter()
        .copied()
        .find(|n| n.get("name").is_some_and(|s| s == "ModuleEngines" || s == "ModuleEnginesFX"))
        .map(|m| {
            let props: Vec<(Res, f64)> = m.nodes_named("PROPELLANT").filter_map(|q| Some((res_of(q.get("name")?)?, q.get_f64("ratio")?))).collect();
            let g = module("ModuleGimbal");
            let resp = truthy(m, "useEngineResponseTime").then(|| (m.get_f64("engineAccelerationSpeed").unwrap_or(0.0), m.get_f64("engineDecelerationSpeed").unwrap_or(0.0)));
            let gimbal_speed = g.and_then(|g| truthy(g, "useGimbalResponseSpeed").then(|| g.get_f64("gimbalResponseSpeed").unwrap_or(10.0)));
            EngineDef {
                max_thrust: m.get_f64("maxThrust").unwrap_or(0.0),
                min_thrust: m.get_f64("minThrust").unwrap_or(0.0),
                atmosphere_curve: cfg::curve(m.node("atmosphereCurve").expect("engine without atmosphereCurve")),
                propellants: props,
                ignition_threshold: m.get_f64("ignitionThreshold").unwrap_or(0.1),
                response: resp,
                gimbal_range_deg: g.and_then(|g| g.get_f64("gimbalRange")).unwrap_or(0.0),
                gimbal_response_speed: gimbal_speed,
                thrust_pos: node_bottom.map_or(V3::ZERO, |n| n.0),
                thrust_dir: v3(0.0, 1.0, 0.0),
            }
        });
    let reaction_wheel = module("ModuleReactionWheel").map(|m| ReactionWheelDef {
        pitch_torque: m.get_f64("PitchTorque").unwrap_or(0.0),
        yaw_torque: m.get_f64("YawTorque").unwrap_or(0.0),
        roll_torque: m.get_f64("RollTorque").unwrap_or(0.0),
        ec_rate: m.node("RESOURCE").and_then(|r| r.get_f64("rate")).unwrap_or(0.0),
        torque_response_speed: m.get_f64("torqueResponseSpeed").unwrap_or(30.0),
    });
    // body lift is on unless a ModuleLiftingSurface disables it
    let body_lift = !modules.iter().any(|m| m.get("name") == Some("ModuleLiftingSurface") && truthy(m, "disableBodyLift"));
    Ok(PartDef {
        name: Box::leak(name.to_string().into_boxed_str()),
        mass: p.get_f64("mass").unwrap_or(0.0),
        com_offset: get_v3(p, "CoMOffset").unwrap_or(V3::ZERO),
        cop_offset: get_v3(p, "CoPOffset").unwrap_or(V3::ZERO),
        angular_drag: p.get_f64("angularDrag").unwrap_or(2.0),
        crash_tolerance: p.get_f64("crashTolerance").unwrap_or(9.0),
        body_lift,
        cube,
        node_top: node("node_stack_top"),
        node_bottom,
        resources,
        engine,
        reaction_wheel,
        leg: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn same(a: &PartDef, b: &PartDef) {
        assert_eq!(a.mass, b.mass, "{}", b.name);
        assert_eq!(a.crash_tolerance, b.crash_tolerance, "{}", b.name);
        assert_eq!(a.cube, b.cube, "{}", b.name);
        assert_eq!(a.node_top, b.node_top, "{}", b.name);
        assert_eq!(a.node_bottom, b.node_bottom, "{}", b.name);
        assert_eq!(a.resources, b.resources, "{}", b.name);
        assert_eq!(a.cop_offset, b.cop_offset, "{}", b.name);
        assert_eq!(a.engine.is_some(), b.engine.is_some());
        if let (Some(x), Some(y)) = (&a.engine, &b.engine) {
            assert_eq!(x.max_thrust, y.max_thrust);
            assert_eq!(x.atmosphere_curve, y.atmosphere_curve);
            assert_eq!(x.propellants, y.propellants);
            assert_eq!(x.gimbal_range_deg, y.gimbal_range_deg);
            assert_eq!(x.gimbal_response_speed, y.gimbal_response_speed);
            assert_eq!(x.thrust_pos, y.thrust_pos);
        }
        if let (Some(x), Some(y)) = (&a.reaction_wheel, &b.reaction_wheel) {
            assert_eq!(x.pitch_torque, y.pitch_torque);
            assert_eq!(x.ec_rate, y.ec_rate);
        }
    }

    /// The loader reproduces ksp-core's hand transcriptions (skipped without an install).
    #[test]
    fn matches_core_transcription() {
        if install_dir().is_none() {
            return;
        }
        same(&load("mk1pod_v2").unwrap(), &parts::mk1pod_v2());
        same(&load("fuelTank").unwrap(), &parts::fuel_tank_t400());
        same(&load("liquidEngine2_v2").unwrap(), &parts::swivel());
        same(&load("liquidEngine3_v2").unwrap(), &parts::terrier());
    }

    #[test]
    fn loads_booster_parts() {
        if install_dir().is_none() {
            return;
        }
        for n in crate::craft::ALL_PARTS {
            let p = load(n).unwrap_or_else(|e| panic!("{n}: {e}"));
            assert!(p.mass > 0.0, "{n}");
        }
        let m = load("liquidEngineMainsail_v2").unwrap();
        assert_eq!(m.engine.as_ref().unwrap().max_thrust, 1500.0);
    }
}
