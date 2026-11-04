//! Parametric reusable boosters built from stock parts, and the random design distribution the
//! Monte Carlo sweeps (1.25 m to 3.75 m, 1-4 engines, a range of TWRs and fuel fractions).
//!
//! Each design compiles to two `VesselSpec`s with identical booster geometry in the same root
//! frame: the *stack* (booster + inert upper stage, flown by the ascent script) and the *booster*
//! alone (after separation). The root part is the bottom tank, so engines and the lowest tank's
//! hull points come first in ksp-core's `MAX_HULL` budget.
//!
//! Known simplifications (no ksp-core change needed, all documented in the report):
//! * engine clusters hang directly under the bottom tank (no adapter / engine plate part);
//! * the upper stage is inert ballast during ascent: its propellant is folded into dry mass,
//!   because ksp-core pools resources vessel-wide (no decoupler crossfeed rules);
//! * legs are LT-2 struts placed so the extended feet sit `LEG_CLEARANCE` below the lowest nozzle.

use crate::partlib;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::{PartDef, N_RES, RES_DENSITY};
use ksp_core::vessel::{Craft, Placed, VesselSpec, MAX_ENGINES, MAX_LEGS, MAX_PARTS};

/// Every stock part a design may use (checked by `partlib` tests).
pub const ALL_PARTS: &[&str] = &[
    "fuelTank", "fuelTank_long", "Rockomax16_BW", "Rockomax32_BW", "Rockomax64_BW", "Size3MediumTank", "Size3LargeTank",
    "liquidEngine2_v2", "SSME", "engineLargeSkipper_v2", "liquidEngineMainsail_v2", "Size3AdvancedEngine", "Size3EngineCluster",
    "advSasModule", "asasmodule1-2", "batteryBank", "batteryBankLarge", "noseCone", "rocketNoseCone_v2", "rocketNoseConeSize3",
    "landingLeg1-2",
];

/// Extended leg feet below the lowest nozzle (m).
pub const LEG_CLEARANCE: f64 = 0.4;

#[derive(Clone, Debug, PartialEq)]
pub struct BoosterDesign {
    /// 1 = 1.25 m, 2 = 2.5 m, 3 = 3.75 m
    pub size: u8,
    /// booster tanks, top to bottom
    pub tanks: Vec<String>,
    pub engine: String,
    pub n_engines: usize,
    pub n_legs: usize,
    pub wheel: String,
    pub n_wheels: usize,
    pub battery: String,
    /// upper stage tanks (inert during ascent), top to bottom, and the fill fraction of their
    /// propellant folded into dry mass
    pub upper_tanks: Vec<String>,
    pub upper_fill: f64,
    pub nose: String,
}

impl BoosterDesign {
    pub fn label(&self) -> String {
        format!("{}x{} {}t", self.n_engines, short(&self.engine), self.tanks.len())
    }
}

pub fn short(engine: &str) -> &'static str {
    match engine {
        "liquidEngine2_v2" => "Swivel",
        "SSME" => "Vector",
        "engineLargeSkipper_v2" => "Skipper",
        "liquidEngineMainsail_v2" => "Mainsail",
        "Size3AdvancedEngine" => "Rhino",
        "Size3EngineCluster" => "Mammoth",
        _ => "?",
    }
}

/// Compiled design: the two specs plus the index bookkeeping.
#[derive(Clone)]
pub struct Built {
    pub design: BoosterDesign,
    pub stack: VesselSpec,
    pub booster: VesselSpec,
    /// number of booster parts (indices 0..n_booster in both crafts)
    pub n_booster: usize,
}

fn place_below(c: &mut Craft, parent: usize, def: PartDef, offset: V3) -> usize {
    let p = &c.parts[parent];
    let (pb, pbd) = p.def.node_bottom.expect("parent has no bottom node");
    let (ct, ctd) = def.node_top.expect("child has no top node");
    let pos = p.pos + p.rot.rotate(pb) - ct + offset;
    let i = c.parts.len();
    let rot = p.rot;
    c.parts[parent].stack.push((i, pbd));
    c.parts.push(Placed { def, pos, rot, stack: vec![(parent, ctd)] });
    i
}

fn place_above(c: &mut Craft, child: usize, def: PartDef) -> usize {
    let p = &c.parts[child];
    let (pt, ptd) = p.def.node_top.expect("part has no top node");
    let (cb, cbd) = def.node_bottom.expect("part above has no bottom node");
    let pos = p.pos + p.rot.rotate(pt) - cb;
    let i = c.parts.len();
    let rot = p.rot;
    c.parts[child].stack.push((i, ptd));
    c.parts.push(Placed { def, pos, rot, stack: vec![(child, cbd)] });
    i
}

fn cluster_offsets(n: usize, engine_width: f64) -> Vec<V3> {
    let w = engine_width;
    match n {
        1 => vec![V3::ZERO],
        2 => vec![v3(-0.5 * w, 0.0, 0.0), v3(0.5 * w, 0.0, 0.0)],
        3 => (0..3).map(|k| {
            let a = (90.0 + 120.0 * k as f64).to_radians();
            v3(a.cos(), 0.0, a.sin()) * (w / 3f64.sqrt())
        }).collect(),
        _ => (0..n).map(|k| {
            let a = (45.0 + 360.0 / n as f64 * k as f64).to_radians();
            v3(a.cos(), 0.0, a.sin()) * (w / 2f64.sqrt())
        }).collect(),
    }
}

/// Build both crafts. Errors if a part is missing or ksp-core's fixed limits are exceeded.
pub fn build(d: &BoosterDesign) -> Result<Built, String> {
    if d.tanks.is_empty() || d.n_engines == 0 || d.n_engines > MAX_ENGINES || d.n_legs > MAX_LEGS {
        return Err("design outside ksp-core limits".into());
    }
    let bottom = partlib::load(d.tanks.last().unwrap())?;
    let mut c = Craft { parts: vec![Placed { def: bottom, pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![] }] };
    // engines under the bottom tank
    let eng = partlib::load(&d.engine)?;
    let ew = eng.cube.size.x.max(eng.cube.size.z);
    let mut eng_bottom = f64::INFINITY;
    for off in cluster_offsets(d.n_engines, ew) {
        let i = place_below(&mut c, 0, eng.clone(), off);
        let e = eng.engine.as_ref().unwrap();
        let p = &c.parts[i];
        eng_bottom = eng_bottom.min(p.pos.y + e.thrust_pos.y).min(p.pos.y + p.def.cube.center.y - 0.5 * p.def.cube.size.y);
    }
    // legs on the bottom tank, feet LEG_CLEARANCE below the lowest nozzle
    let leg = partlib::load("landingLeg1-2")?;
    let r_tank = 0.5 * c.parts[0].def.cube.size.x;
    let foot_y = leg.leg.unwrap().foot.y;
    let attach_y = eng_bottom - LEG_CLEARANCE - foot_y;
    for k in 0..d.n_legs {
        let az = 90.0 + 360.0 / d.n_legs as f64 * k as f64;
        c.surface_attach(0, leg.clone(), az, r_tank, attach_y);
    }
    // remaining tanks upward, then battery and wheels
    let mut top = 0;
    for t in d.tanks.iter().rev().skip(1) {
        top = place_above(&mut c, top, partlib::load(t)?);
    }
    top = place_above(&mut c, top, partlib::load(&d.battery)?);
    for _ in 0..d.n_wheels {
        top = place_above(&mut c, top, partlib::load(&d.wheel)?);
    }
    let n_booster = c.parts.len();
    let booster_craft = c.clone();
    // inert upper stage
    let mut s = c;
    for t in &d.upper_tanks {
        top = place_above(&mut s, top, partlib::load(t)?);
    }
    place_above(&mut s, top, partlib::load(&d.nose)?);
    if s.parts.len() > MAX_PARTS {
        return Err(format!("{} parts > MAX_PARTS {MAX_PARTS}", s.parts.len()));
    }
    let booster = VesselSpec::compile(&booster_craft);
    let mut stack = VesselSpec::compile(&s);
    // fold the upper stage's resources into its dry mass (no crossfeed into the booster)
    for i in n_booster..stack.n_parts as usize {
        for r in 0..N_RES {
            let cap = stack.part_res_cap[i][r];
            let fill = if r < 2 { d.upper_fill } else { 1.0 };
            stack.part_dry_mass[i] += cap * fill * RES_DENSITY[r];
            stack.res_cap_total[r] -= cap;
            stack.part_res_cap[i][r] = 0.0;
        }
    }
    Ok(Built { design: d.clone(), stack, booster, n_booster })
}

/// Small deterministic RNG (splitmix64), shared by design and dispersion sampling.
#[derive(Clone, Copy, Debug)]
pub struct Rng(pub u64);
impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    pub fn unif(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * ((self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64))
    }
    pub fn pick<'a, T>(&mut self, xs: &'a [T]) -> &'a T {
        &xs[(self.next_u64() % xs.len() as u64) as usize]
    }
    pub fn normal(&mut self) -> f64 {
        let u1 = self.unif(1e-12, 1.0);
        let u2 = self.unif(0.0, 1.0);
        (-2.0 * u1.ln()).sqrt() * (core::f64::consts::TAU * u2).cos()
    }
}

struct Family {
    size: u8,
    tanks: &'static [&'static str],
    engines: &'static [(&'static str, usize, usize)],
    wheel: &'static str,
    battery: &'static str,
    upper: &'static [&'static str],
    nose: &'static str,
    max_tanks: usize,
}

const FAMILIES: [Family; 3] = [
    Family {
        size: 1,
        tanks: &["fuelTank_long", "fuelTank"],
        engines: &[("liquidEngine2_v2", 1, 4), ("SSME", 1, 1)],
        wheel: "advSasModule",
        battery: "batteryBank",
        upper: &["fuelTank", "fuelTank_long"],
        nose: "noseCone",
        max_tanks: 4,
    },
    Family {
        size: 2,
        tanks: &["Rockomax64_BW", "Rockomax32_BW", "Rockomax16_BW"],
        engines: &[("engineLargeSkipper_v2", 1, 4), ("liquidEngineMainsail_v2", 1, 2), ("SSME", 2, 4)],
        wheel: "asasmodule1-2",
        battery: "batteryBankLarge",
        upper: &["Rockomax16_BW", "Rockomax32_BW"],
        nose: "rocketNoseCone_v2",
        max_tanks: 3,
    },
    Family {
        size: 3,
        tanks: &["Size3LargeTank", "Size3MediumTank"],
        engines: &[("Size3AdvancedEngine", 1, 3), ("Size3EngineCluster", 1, 2), ("liquidEngineMainsail_v2", 3, 4)],
        wheel: "asasmodule1-2",
        battery: "batteryBankLarge",
        upper: &["Size3MediumTank", "Rockomax64_BW"],
        nose: "rocketNoseConeSize3",
        max_tanks: 3,
    },
];

fn part_mass_full(name: &str) -> f64 {
    let p = partlib::load(name).expect("part");
    p.mass + p.resources.iter().map(|(r, _, m)| m * RES_DENSITY[*r as usize]).sum::<f64>()
}

fn engine_asl_thrust(name: &str) -> f64 {
    let p = partlib::load(name).expect("engine");
    let e = p.engine.unwrap();
    e.max_thrust * e.atmosphere_curve.eval(1.0) / e.atmosphere_curve.eval(0.0)
}

/// Sample a random design: family (size) uniform unless `size` is given, engine type and count,
/// then the tank stack that brings the *stack* liftoff TWR closest to a sampled target in
/// [1.25, 2.1] with an upper stage of 15-35 % of the booster's wet mass.
pub fn sample_design(rng: &mut Rng, size: Option<u8>) -> BoosterDesign {
    loop {
        let fam = match size {
            Some(s) => &FAMILIES[(s.clamp(1, 3) - 1) as usize],
            None => rng.pick(&FAMILIES),
        };
        let &(engine, nmin, nmax) = rng.pick(fam.engines);
        let n_eng = nmin + (rng.next_u64() % (nmax - nmin + 1) as u64) as usize;
        let thrust = engine_asl_thrust(engine) * n_eng as f64;
        let twr = rng.unif(1.25, 2.1);
        let upper_frac = rng.unif(0.15, 0.35);
        let target_stack = thrust / (twr * 9.81);
        // greedy tank stack: largest tanks first until the booster mass target is met
        let eng_mass = part_mass_full(engine) * n_eng as f64;
        let target_booster = target_stack / (1.0 + upper_frac);
        let mut tanks: Vec<String> = vec![];
        let mut m = eng_mass + 1.0;
        for _ in 0..fam.max_tanks {
            let rem = target_booster - m;
            if rem <= 0.0 {
                break;
            }
            // the tank that best fills the remaining mass
            let best = fam.tanks.iter().min_by(|a, b| {
                let da = (rem - part_mass_full(a)).abs();
                let db = (rem - part_mass_full(b)).abs();
                da.partial_cmp(&db).unwrap()
            }).unwrap();
            if part_mass_full(best) > 1.6 * rem && !tanks.is_empty() {
                break;
            }
            tanks.push(best.to_string());
            m += part_mass_full(best);
        }
        if tanks.is_empty() {
            continue;
        }
        tanks.sort_by(|a, b| part_mass_full(a).partial_cmp(&part_mass_full(b)).unwrap());
        // upper stage: tank(s) filled to reach the sampled upper-stage fraction
        let upper_target = upper_frac * m;
        let mut upper = vec![];
        let mut mu = 0.0;
        for t in fam.upper.iter().rev() {
            if mu + 0.3 * part_mass_full(t) < upper_target && upper.len() < 2 {
                upper.push(t.to_string());
                mu += part_mass_full(t);
            }
        }
        if upper.is_empty() {
            upper.push(fam.upper[0].to_string());
        }
        let cap: f64 = upper.iter().map(|t| part_mass_full(t)).sum();
        let dry: f64 = upper.iter().map(|t| partlib::load(t).unwrap().mass).sum();
        let fill = ((upper_target - dry) / (cap - dry)).clamp(0.0, 1.0);
        let n_legs = if fam.size == 1 { 3 + (rng.next_u64() % 2) as usize } else { 4 };
        let n_wheels = if fam.size == 3 { 2 } else { 1 };
        let d = BoosterDesign {
            size: fam.size,
            tanks,
            engine: engine.to_string(),
            n_engines: n_eng,
            n_legs,
            wheel: fam.wheel.to_string(),
            n_wheels,
            battery: fam.battery.to_string(),
            upper_tanks: upper,
            upper_fill: fill,
            nose: fam.nose.to_string(),
        };
        if build(&d).is_ok() {
            return d;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn designs_build_and_cover_sizes() {
        if partlib::install_dir().is_none() {
            return;
        }
        let mut rng = Rng(7);
        let mut sizes = [0; 3];
        for _ in 0..60 {
            let d = sample_design(&mut rng, None);
            let b = build(&d).unwrap();
            sizes[(d.size - 1) as usize] += 1;
            let (mb, _, _) = b.booster.mass_props(&b.booster.full_resources());
            let (ms, _, _) = b.stack.mass_props(&b.stack.full_resources());
            assert!(ms > mb, "{d:?}");
            assert_eq!(b.booster.n_engines as usize, d.n_engines);
            assert_eq!(b.stack.res_cap_total[0], b.booster.res_cap_total[0]);
            // legs are the lowest points
            assert!(b.booster.bottom_y < b.booster.engines[0].pos.y);
        }
        assert!(sizes.iter().all(|&n| n > 5), "{sizes:?}");
    }
}
