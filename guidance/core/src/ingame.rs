//! The in-game demo craft (ingame/make_demo_craft.py writes the same geometry as a .craft):
//! booster = Mk1 pod (root, command + wheel) / FL-T800 / FL-T400 / LV-T45 Swivel with four LT-2
//! legs placed by the VAB surface-attach rule, all from ksp-core's in-game validated part catalog;
//! upper stage = TD-12 / Terrier / FL-T400 / nose cone on the pod (inert during ascent, its
//! propellant folded into dry mass, as `craft::build` does).

use crate::craft::{BoosterDesign, Built};
use crate::partlib;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::{self, PartDef, N_RES, RES_DENSITY};
use ksp_core::vessel::{Craft, Placed, VesselSpec};

/// LT-2 srfAttach node (cfg * rescaleFactor 1.25) and orientation
pub const LT2_NODE_POS: V3 = v3(0.0, 0.4271562 * 1.25, 0.0632596 * 1.25);
pub const LT2_NODE_DIR: V3 = v3(0.0, 0.5735765, 0.819152);
/// legs: on the FL-T400, 0.9 m below its centre (the pad-tested hopper placement), azimuths
pub const LEG_Y: f64 = -0.9;
pub const LEG_AZ: [f64; 4] = [45.0, 135.0, 225.0, 315.0];

fn place_above(c: &mut Craft, child: usize, def: PartDef) -> usize {
    let p = &c.parts[child];
    let (pt, ptd) = p.def.node_top.expect("part has no top node");
    let (cb, cbd) = def.node_bottom.expect("part above has no bottom node");
    let pos = p.pos + p.rot.rotate(pt) - cb;
    let i = c.parts.len();
    let rot = p.rot;
    c.parts[child].stack.push((i, ptd));
    c.parts.push(Placed { def, pos, rot, stack: vec![(child, cbd)], parent: Some(child) });
    i
}

/// Booster parts first (indices 0..n_booster), root = pod at the origin.
pub fn crafts() -> Result<(Craft, Craft, usize), String> {
    let mut c = Craft { parts: vec![Placed { def: parts::mk1pod_v2(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![], parent: None }] };
    let t800 = c.stack_below(0, parts::fuel_tank_t800());
    let t400 = c.stack_below(t800, parts::fuel_tank_t400());
    let _eng = c.stack_below(t400, parts::swivel());
    for az in LEG_AZ {
        c.surface_attach_editor(t400, parts::lt2_leg(), az, 0.625, LEG_Y, LT2_NODE_POS, LT2_NODE_DIR);
    }
    let n_booster = c.parts.len();
    let booster = c.clone();
    let dec = place_above(&mut c, 0, partlib::load("Decoupler_1")?);
    // the upper Terrier stays unlit: no engine module in the sim (ksp-core pools propellant vessel-wide)
    let mut terrier = parts::terrier();
    terrier.engine = None;
    let ter = place_above(&mut c, dec, terrier);
    let tank = place_above(&mut c, ter, parts::fuel_tank_t400());
    place_above(&mut c, tank, partlib::load("noseCone")?);
    Ok((c, booster, n_booster))
}

pub fn design() -> BoosterDesign {
    BoosterDesign {
        size: 1,
        tanks: vec!["fuelTank_long".into(), "fuelTank".into()],
        engine: "liquidEngine2_v2".into(),
        n_engines: 1,
        n_legs: 4,
        wheel: "mk1pod_v2".into(),
        n_wheels: 1,
        battery: "-".into(),
        upper_tanks: vec!["liquidEngine3_v2".into(), "fuelTank".into()],
        upper_fill: 1.0,
        nose: "noseCone".into(),
    }
}

pub fn built() -> Result<Built, String> {
    let (s, b, n_booster) = crafts()?;
    let booster = VesselSpec::compile(&b);
    let mut stack = VesselSpec::compile(&s);
    for i in n_booster..stack.n_parts as usize {
        for r in 0..N_RES {
            let cap = stack.part_res_cap[i][r];
            stack.part_dry_mass[i] += cap * RES_DENSITY[r];
            stack.res_cap_total[r] -= cap;
            stack.part_res_cap[i][r] = 0.0;
        }
    }
    Ok(Built { design: design(), stack, booster, n_booster })
}
