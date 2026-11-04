//! The in-game demo craft (ingame/make_demo_craft.py writes the same geometry as a .craft):
//! booster = Mk1 pod (root, command + wheel) / FL-T800 / FL-T400 / LV-T45 Swivel with four LT-2
//! legs placed by the VAB surface-attach rule, all from ksp-core's in-game validated part catalog;
//! TD-12 on the pod (stays with the booster); upper stage = Terrier / FL-T400 / nose cone (inert during ascent, its
//! propellant folded into dry mass, as `craft::build` does).

use crate::craft::{BoosterDesign, Built};
use crate::mission::Dispersion;
use crate::partlib;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::{self, PartDef, N_RES, RES_DENSITY};
use ksp_core::sim::{self, EnvState, Frame, STATUS_CONTACT, STATUS_FLYING, STATUS_LANDED};
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
    // TD-12 releases the part on its top node: the decoupler stays on the booster
    let dec = place_above(&mut c, 0, partlib::load("Decoupler_1")?);
    let n_booster = c.parts.len();
    let booster = c.clone();
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

/// Number of numbers in a bridge state line after the "S" tag (see ingame/plugin/Bridge.cs).
pub const BRIDGE_FIELDS: usize = 31;

/// Game state from a bridge line (Bridge.cs `State()`, SimHarness body-frame conventions), seeded
/// the way ksp-core trace.rs::state_at seeds a logged tick. `spec` = the configuration the game is
/// in at that tick.
pub fn state_from_bridge(x: &[f64], spec: &VesselSpec) -> EnvState {
    let vv = |i: usize| v3(x[i], x[i + 1], x[i + 2]);
    let (com, vsrf, up, fwd, wbf) = (vv(2), vv(5), vv(8), vv(11), vv(14));
    let res = [x[17], x[18], x[19], x[20]];
    let kb = vv(28);
    let q = sim::attitude(up.normalized(), fwd.normalized());
    let w = -wbf;
    let (_, com_l, _) = spec.mass_props(&res);
    let rc = q.rotate(com_l);
    let mut st = EnvState { ut: x[0], frame: if x[1] > 0.5 { Frame::Rotating } else { Frame::Inertial }, q, w, res, toff: f32::NAN, ..Default::default() };
    st.x = (com - kb * sim::FIXED_DT) - rc;
    st.v = vsrf - w.cross(rc);
    if spec.n_engines > 0 {
        st.thr[0] = x[21];
        let r = spec.engines[0].gimbal_range_deg;
        st.gimbal[0] = [x[22] * r, x[23] * r];
    }
    st.rw_in = vv(24);
    st.status = if x[27] > 0.5 { if vsrf.norm() < 0.3 { STATUS_LANDED } else { STATUS_CONTACT } } else { STATUS_FLYING };
    st
}

/// What Bridge.cs would send for a sim state (sim-side stand-in for the game, for testing the
/// bridge path in Monte Carlo).
pub fn bridge_from_state(st: &EnvState, spec: &VesselSpec) -> Vec<f64> {
    let (_, com_l, _) = spec.mass_props(&st.res);
    let rc = st.q.rotate(com_l);
    let com = st.x + rc;
    let vsrf = st.v + st.w.cross(rc);
    let up = st.q.rotate(v3(0.0, 1.0, 0.0));
    let fwd = st.q.rotate(v3(0.0, 0.0, 1.0));
    let wbf = -st.w;
    let r = if spec.n_engines > 0 { spec.engines[0].gimbal_range_deg.max(1e-9) } else { 1.0 };
    let landed = (st.status == STATUS_CONTACT || st.status == STATUS_LANDED) as u8 as f64;
    let mut x = vec![st.ut, (st.frame == Frame::Rotating) as u8 as f64];
    for v in [com, vsrf, up, fwd, wbf] {
        x.extend([v.x, v.y, v.z]);
    }
    x.extend(st.res);
    x.extend([st.thr[0], st.gimbal[0][0] / r, st.gimbal[0][1] / r, st.rw_in.x, st.rw_in.y, st.rw_in.z, landed, 0.0, 0.0, 0.0]);
    x
}

/// Nominal (dispersion-free) flight for the game: the middle of `sample_dispersion`'s ranges.
pub fn nominal_dispersion(ut: f64) -> Dispersion {
    Dispersion { ut, heading_deg: 90.0, kick_speed: 65.0, kick_deg: 5.0, turn_alt: 60_000.0, turn_exp: 0.525, meco_margin: 1.3, fuel_load: 1.0, ..Default::default() }
}
