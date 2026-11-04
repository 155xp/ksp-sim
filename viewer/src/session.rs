//! One viewer session: a single guided flight in the real sim, ticked at the requested multiple
//! of real time (or as fast as the CPU allows), with the state after every 0.02 s physics tick
//! sent to the browser as packed little-endian f32 rows of `STRIDE` values.

use ksp_core::atmo;
use ksp_core::body::KERBIN;
use ksp_core::ksc::KSC_PAD_DECK_ALT;
use ksp_core::math::{v3, V3};
use ksp_core::sim::{self, FIXED_DT};
use ksp_core::parts::Res;
use ksp_guidance::craft::{build, sample_design, short, Built, Rng};
use ksp_guidance::partlib;
use ksp_guidance::fleet::make_fleet;
use ksp_guidance::mission::{sample_dispersion, Flight, GuidanceConfig, LandingMode, Outcome, Phase, Shared, MAX_LANDING_G};
use ksp_guidance::nav::{nav, p_atm, propellant, propulsion, touchdown_height, Pad};
use ksp_guidance::predict::{predict, LandingBurnModel, PredCraft, PredOpts};
use serde::Deserialize;
use serde_json::json;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Row layout (keep in sync with `static/app.js`):
/// t, pos ENU (3), surface velocity ENU (3), vessel axis ENU (3), vessel +X ENU (3), throttle,
/// phase, height of the leg feet (CoM minus touchdown height) above the pad deck, propellant (t), mass (t), predicted touchdown (east, north),
/// ground offset from the pad (east, north), vertical speed, horizontal speed.
pub const STRIDE: usize = 24;
/// Predicted touchdown is re-solved every this many ticks (0.5 s of flight).
const PRED_EVERY: u64 = 25;

#[derive(Deserialize, Debug)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum Cmd {
    Start { seed: u64, size: u8, landing: String, start: Start, speed: f64 },
    Pause,
    Resume,
    /// multiple of real time; <= 0 means as fast as possible
    Speed { value: f64 },
    Reset,
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Start {
    /// the Monte Carlo flight for this seed: ascent from the pad, MECO, separation
    Random,
    /// booster-only flight starting at separation: altitude above the pad (m), downrange east of
    /// the pad (m), horizontal and vertical surface speed (m/s), propellant fraction
    Set { alt: f64, downrange: f64, vh: f64, vz: f64, fuel: f64 },
}

pub enum Out {
    Text(String),
    Ticks(Vec<u8>),
}

struct Live {
    f: Flight,
    ticks: u64,
    pred: (f64, f64),
    rows: Vec<f32>,
}

fn landing_mode(s: &str) -> LandingMode {
    if s == "gfold" { LandingMode::Gfold } else { LandingMode::Baseline }
}

fn outcome_name(o: Outcome) -> &'static str {
    match o {
        Outcome::Running => "running",
        Outcome::Landed => "landed",
        Outcome::Crashed => "crashed",
        Outcome::TippedOver => "tipped_over",
        Outcome::Timeout => "timeout",
        Outcome::NoReturn => "no_return",
    }
}

/// Part layout for drawing, in the booster's root frame relative to its full-tank CoM (m):
/// every stack part's drag-cube box (the booster's parts first, same indices in both specs),
/// engine thrust points and directions, and the leg feet.
fn geometry(b: &Built) -> serde_json::Value {
    let d = &b.design;
    let com = b.booster.mass_props(&b.booster.full_resources()).1;
    let ne = d.n_engines;
    let nl = d.n_legs;
    // part order as `craft::build` places them
    let mut names: Vec<(&str, &str)> = vec![(d.tanks.last().unwrap().as_str(), "tank")];
    names.extend(std::iter::repeat_n((d.engine.as_str(), "engine"), ne));
    names.extend(std::iter::repeat_n(("landingLeg1-2", "leg"), nl));
    names.extend(d.tanks.iter().rev().skip(1).map(|t| (t.as_str(), "tank")));
    names.push((d.battery.as_str(), "part"));
    names.extend(std::iter::repeat_n((d.wheel.as_str(), "part"), d.n_wheels));
    names.extend(d.upper_tanks.iter().map(|t| (t.as_str(), "upper")));
    names.push((d.nose.as_str(), "nose"));
    let s = &b.stack;
    let mut parts = vec![];
    for (i, (name, kind)) in names.iter().enumerate().take(s.n_parts as usize) {
        if *kind == "leg" || *kind == "engine" {
            continue;
        }
        let Ok(def) = partlib::load(name) else { continue };
        let c = s.part_com[i] + s.part_rot[i].rotate(def.cube.center) - com;
        let z = def.cube.size;
        parts.push(json!({ "kind": kind, "c": [c.x, c.y, c.z], "s": [z.x, z.y, z.z] }));
    }
    let bs = &b.booster;
    let engines: Vec<_> = (0..bs.n_engines as usize).map(|k| {
        let e = &bs.engines[k];
        let p = e.pos - com;
        let size = partlib::load(&d.engine).map(|def| def.cube.size).unwrap_or(v3(1.0, 1.0, 1.0));
        json!({ "p": [p.x, p.y, p.z], "d": [e.dir.x, e.dir.y, e.dir.z], "s": [size.x, size.y, size.z],
                "top": (s.part_com[1 + k] + s.part_rot[1 + k].rotate(partlib::load(&d.engine).map(|def| def.cube.center).unwrap_or(V3::ZERO)) - com).y + 0.5 * size.y })
    }).collect();
    let legs: Vec<_> = (0..bs.n_legs as usize).map(|k| {
        let f = bs.legs[k].foot - com;
        let a = bs.part_com[1 + ne + k] - com;
        json!({ "foot": [f.x, f.y, f.z], "attach": [a.x, a.y, a.z] })
    }).collect();
    json!({ "parts": parts, "engines": engines, "legs": legs })
}

fn new_flight(seed: u64, size: u8, landing: &str, start: Start) -> Result<Flight, String> {
    let cfg = GuidanceConfig { landing: landing_mode(landing), ..Default::default() };
    let sz = (size > 0).then_some(size.min(3));
    match start {
        // exactly the flight `Fleet(seed, n_designs=1, per_design=1, size)` / `mc` flies
        Start::Random => Ok(make_fleet(seed, 1, 1, sz, cfg, false).remove(0)),
        Start::Set { alt, downrange, vh, vz, fuel } => {
            let mut rng = Rng(seed);
            let d = sample_design(&mut rng, sz);
            let sh = Arc::new(Shared::new(build(&d)?));
            let disp = sample_dispersion(&mut rng, false);
            let mut f = Flight::new(sh.clone(), disp, cfg);
            let b = &sh.built.booster;
            let pad = Pad::ksc();
            let rg = pad.origin.norm();
            // a point `downrange` east of the pad along the ground sphere, `alt` above it
            let th = downrange / rg;
            let up = pad.u * th.cos() + pad.e * th.sin();
            let east = pad.e * th.cos() - pad.u * th.sin();
            let x = up * (rg + alt.max(10.0));
            let v = east * vh + up * vz;
            let dir = if v.norm() > 1e-6 { v.normalized() } else { up };
            let q = sim::attitude(dir, pad.n);
            let mut st = sim::state_at_pad(b, pad.to_enu(x), pad.vec_to_enu(v), q, V3::ZERO, disp.ut);
            let com_full = b.mass_props(&st.res).1;
            let fuel = fuel.clamp(0.0, 1.0);
            st.res[Res::LiquidFuel as usize] *= fuel;
            st.res[Res::Oxidizer as usize] *= fuel;
            // keep the CoM where it was asked for after the propellant change
            let com = b.mass_props(&st.res).1;
            st.x += q.rotate(com_full) - q.rotate(com);
            // the guidance reads the atmosphere's temperature offset before the first sim tick
            // fills it in; compute it as `sim::tick` does (rotating frame, CoM position)
            let (sun, ta) = sim::sun_dir_fixed(KERBIN, st.ut);
            st.toff = if alt < KERBIN.atmosphere_depth { atmo::temperature_offset(KERBIN, sun, x.normalized(), v3(0.0, 0.0, 1.0), ta, None) as f32 } else { 0.0 };
            f.st = st;
            f.phase = Phase::Flip;
            let s = &mut f.stats;
            s.sep_alt = alt + KSC_PAD_DECK_ALT;
            s.sep_speed = v.norm();
            s.sep_vz = vz;
            s.sep_vh = vh.abs();
            s.sep_downrange = downrange.abs();
            s.sep_mass = b.mass_props(&f.st.res).0;
            let (p, cap) = propellant(b, &f.st.res);
            s.sep_prop_frac = p / cap;
            Ok(f)
        }
    }
}

fn init_msg(f: &Flight, seed: u64, size: u8, landing: &str, start: Start) -> String {
    let b = &f.sh.built;
    let d = &b.design;
    let (_, cap) = propellant(&b.booster, &b.booster.full_resources());
    let diameter = [0.0, 1.25, 2.5, 3.75][d.size as usize];
    json!({
        "type": "init",
        "seed": seed, "size": d.size, "size_req": size, "landing": landing,
        "start": match start { Start::Random => "random", Start::Set { .. } => "set" },
        "design": {
            "label": d.label(), "engine": short(&d.engine), "n_engines": d.n_engines, "n_legs": d.n_legs,
            "diameter": diameter,
            "booster_wet_t": b.booster.mass_props(&b.booster.full_resources()).0,
            "booster_dry_t": f.sh.m_dry, "prop_cap_t": cap,
        },
        "geom": geometry(b),
        "ground_r": Pad::ksc().origin.norm(),
        "stride": STRIDE, "dt": FIXED_DT,
    })
    .to_string()
}

fn done_msg(l: &Live) -> String {
    let s = &l.f.stats;
    json!({
        "type": "done",
        "outcome": outcome_name(l.f.outcome),
        "miss": s.final_e.hypot(s.final_n),
        "final_e": s.final_e, "final_n": s.final_n,
        "td_miss": s.td_e.hypot(s.td_n),
        "td_vz": s.td_vz, "td_vh": s.td_vh, "td_speed": s.td_vz.hypot(s.td_vh), "td_t": s.td_t,
        "td_tilt_deg": s.td_tilt_deg, "final_tilt_deg": s.final_tilt_deg,
        "prop_left_t": s.prop_left, "prop_cap_t": s.prop_cap,
        "sep": { "t": s.sep_t, "alt": s.sep_alt - KSC_PAD_DECK_ALT, "speed": s.sep_speed, "vh": s.sep_vh, "vz": s.sep_vz,
                 "downrange": s.sep_downrange, "prop_frac": s.sep_prop_frac },
        "ticks": l.ticks,
    })
    .to_string()
}

/// Guidance's own touchdown predictor (landing-burn model), from the current state.
fn predicted_touchdown(f: &Flight) -> (f64, f64) {
    if matches!(f.phase, Phase::Ascent | Phase::Touchdown | Phase::Done) {
        return (f64::NAN, f64::NAN);
    }
    let b = &f.sh.built.booster;
    let n = nav(b, &f.st);
    let c = PredCraft { spec: b, tables: &f.sh.tables, m_dry: f.sh.m_dry, h_td: touchdown_height(b, n.com_l), toff: f.st.toff as f64, body_x: n.q.rotate(v3(1.0, 0.0, 0.0)) };
    let t_pad = propulsion(b, p_atm(KSC_PAD_DECK_ALT)).thrust;
    let burn = LandingBurnModel { f_ign: f.cfg.f_ign, max_g: MAX_LANDING_G, t_pad, v_td: f.cfg.v_td };
    let r = predict(&c, &PredOpts { burn: Some(burn), sim_offset: true, ..Default::default() }, n.x, n.v, n.m);
    if r.ok { Pad::ksc().ground_offset(r.x) } else { (f64::NAN, f64::NAN) }
}

fn push_row(l: &mut Live) {
    let f = &l.f;
    let pad = Pad::ksc();
    let spec = f.spec();
    let n = nav(spec, &f.st);
    let p = pad.to_enu(n.x);
    let v = pad.vec_to_enu(n.v);
    let ax = pad.vec_to_enu(n.up);
    let bx = pad.vec_to_enu(n.q.rotate(v3(1.0, 0.0, 0.0)));
    let ne = spec.n_engines as usize;
    let lit = (0..ne).filter(|k| f.st.flameout & (1 << k) == 0);
    let thr = lit.map(|k| f.st.thr[k]).sum::<f64>() / ne.max(1) as f64;
    let (prop, _) = propellant(spec, &f.st.res);
    let up = n.x.normalized();
    let vz = n.v.dot(up);
    let vh = (n.v - up * vz).norm();
    let (ge, gn) = pad.ground_offset(n.x);
    let row: [f64; STRIDE] = [
        f.st.ut - f.disp.ut, p.x, p.y, p.z, v.x, v.y, v.z, ax.x, ax.y, ax.z, bx.x, bx.y, bx.z,
        thr, f.phase as u8 as f64, n.alt - KSC_PAD_DECK_ALT - touchdown_height(spec, n.com_l), prop, n.m, l.pred.0, l.pred.1, ge, gn, vz, vh,
    ];
    l.rows.extend(row.iter().map(|&x| x as f32));
}

fn step(l: &mut Live) {
    l.f.tick();
    l.ticks += 1;
    if l.ticks % PRED_EVERY == 0 || l.f.done() {
        l.pred = predicted_touchdown(&l.f);
    }
    push_row(l);
}

fn flush(l: &mut Live) -> Option<Out> {
    if l.rows.is_empty() {
        return None;
    }
    let bytes = l.rows.iter().flat_map(|x| x.to_le_bytes()).collect();
    l.rows.clear();
    Some(Out::Ticks(bytes))
}

pub fn run(rx: Receiver<Cmd>, tx: tokio::sync::mpsc::Sender<Out>) {
    let send = |o: Out| tx.blocking_send(o).is_ok();
    let mut live: Option<Live> = None;
    let mut running = false;
    let mut speed = 1.0f64;
    let mut anchor = (Instant::now(), 0u64);
    let mut rate = (Instant::now(), 0u64);
    loop {
        let active = running && live.as_ref().is_some_and(|l| !l.f.done());
        // idle: block on the next command; flying: take whatever arrived
        let mut cmds = vec![];
        if !active {
            match rx.recv() {
                Ok(c) => cmds.push(c),
                Err(_) => return,
            }
        }
        loop {
            match rx.try_recv() {
                Ok(c) => cmds.push(c),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return,
            }
        }
        for c in cmds {
            match c {
                Cmd::Start { seed, size, landing, start, speed: s } => {
                    match new_flight(seed, size, &landing, start) {
                        Ok(f) => {
                            if !send(Out::Text(init_msg(&f, seed, size, &landing, start))) {
                                return;
                            }
                            let mut l = Live { f, ticks: 0, pred: (f64::NAN, f64::NAN), rows: vec![] };
                            push_row(&mut l);
                            live = Some(l);
                            running = true;
                            speed = s;
                        }
                        Err(e) => {
                            if !send(Out::Text(json!({"type": "error", "message": e}).to_string())) {
                                return;
                            }
                        }
                    }
                }
                Cmd::Pause => running = false,
                Cmd::Resume => running = live.is_some(),
                Cmd::Speed { value } => speed = value,
                Cmd::Reset => {
                    live = None;
                    running = false;
                    if !send(Out::Text(json!({"type": "reset"}).to_string())) {
                        return;
                    }
                }
            }
            if let Some(l) = &live {
                anchor = (Instant::now(), l.ticks);
                rate = anchor;
            }
        }
        let Some(l) = live.as_mut() else { continue };
        if !running || l.f.done() {
            continue;
        }
        let frame = Instant::now();
        let budget = frame + Duration::from_millis(30);
        let max = speed <= 0.0;
        let target = if max { u64::MAX } else { anchor.1 + (frame.duration_since(anchor.0).as_secs_f64() * speed / FIXED_DT) as u64 };
        while l.ticks < target && !l.f.done() && Instant::now() < budget {
            step(l);
        }
        // CPU-bound: run as fast as it can instead of racing to catch up later
        if !max && l.ticks < target && Instant::now() >= budget {
            anchor = (Instant::now(), l.ticks);
        }
        if let Some(o) = flush(l) {
            if !send(o) {
                return;
            }
        }
        let el = rate.0.elapsed().as_secs_f64();
        if el >= 1.0 || l.f.done() {
            let x = (l.ticks - rate.1) as f64 * FIXED_DT / el.max(1e-6);
            if !send(Out::Text(json!({"type": "rate", "x": x}).to_string())) {
                return;
            }
            rate = (Instant::now(), l.ticks);
        }
        if l.f.done() {
            running = false;
            if !send(Out::Text(done_msg(l))) {
                return;
            }
            continue;
        }
        if !max {
            std::thread::sleep(Duration::from_millis(16).saturating_sub(frame.elapsed()));
        }
    }
}
