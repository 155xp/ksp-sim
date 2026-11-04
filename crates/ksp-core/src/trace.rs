//! In-game ground-truth traces (SimHarness plugin, harness/plugin/SimHarness.cs): parsing,
//! a `VesselSpec` built from the game's exact geometry dump, state seeding from a logged tick,
//! and open-loop replay of the logged pilot input for tick-by-tick comparison.
//!
//! Conventions: logged vectors are in the sim body-fixed frame (BodyFrame.WorldToLocal(v.xzy)).
//! That map is a reflection of Unity's world axes, so *pseudo*-vectors (angular velocity) flip
//! sign. Root-local geometry is in Unity's left-handed part frame; the sim's right-handed root
//! frame is the same with x negated (positions/directions x -> -x, quaternions (x,y,z,w) ->
//! (x,-y,-z,w), drag-cube faces XP <-> XN).

use crate::aero::AeroFaces;
use crate::math::{v3, Quat, V3};
use crate::parts::{self, N_RES};
use crate::sim::{self, Control, EnvState, Frame, VEL_SMOOTH_LEN};
use crate::vessel::{Craft, Placed, VesselSpec};
use std::collections::HashMap;
use std::path::Path;

#[derive(Clone, Debug, Default)]
pub struct GeomPart {
    pub name: String,
    pub parent: i32,
    pub kv: Vec<(String, String)>,
}

impl GeomPart {
    pub fn get(&self, k: &str) -> Option<&str> {
        self.kv.iter().find(|(a, _)| a == k).map(|(_, b)| b.as_str())
    }
    pub fn all<'a>(&'a self, k: &'a str) -> impl Iterator<Item = &'a str> + 'a {
        self.kv.iter().filter(move |(a, _)| a == k).map(|(_, b)| b.as_str())
    }
    pub fn vec(&self, k: &str) -> Option<Vec<f64>> {
        self.get(k).map(nums)
    }
}

/// "a=1,2 b=3" style fields -> map.
pub fn fields(s: &str) -> HashMap<String, String> {
    s.split_whitespace().filter_map(|t| t.split_once('=')).map(|(a, b)| (a.to_string(), b.to_string())).collect()
}

pub fn nums(s: &str) -> Vec<f64> {
    s.split(',').map(|x| x.trim().parse().unwrap_or(f64::NAN)).collect()
}

pub fn parse_geometry(text: &str) -> (HashMap<String, String>, Vec<GeomPart>) {
    let mut header = HashMap::new();
    let mut parts = vec![];
    let mut cur: Option<GeomPart> = None;
    for ln in text.lines() {
        if ln.starts_with("PART {") {
            cur = Some(GeomPart::default());
        } else if ln.starts_with('}') {
            if let Some(mut p) = cur.take() {
                p.name = p.get("name").unwrap_or("").to_string();
                p.parent = p.get("parent").and_then(|s| s.parse().ok()).unwrap_or(-1);
                parts.push(p);
            }
        } else if let Some((k, v)) = ln.split_once(" = ") {
            match cur.as_mut() {
                Some(p) => p.kv.push((k.trim().to_string(), v.trim().to_string())),
                None => {
                    header.insert(k.trim().to_string(), v.trim().to_string());
                }
            }
        }
    }
    (header, parts)
}

/// CSV with a header row; non-numeric cells become NaN (strings kept separately).
pub struct Csv {
    pub cols: HashMap<String, Vec<f64>>,
    pub strs: HashMap<String, Vec<String>>,
    pub n: usize,
}

impl Csv {
    pub fn parse(text: &str) -> Csv {
        let mut lines = text.lines();
        let head: Vec<String> = lines.next().unwrap_or("").split(',').map(|s| s.to_string()).collect();
        let mut cols: Vec<Vec<f64>> = vec![vec![]; head.len()];
        let mut strs: Vec<Vec<String>> = vec![vec![]; head.len()];
        let mut n = 0;
        for ln in lines {
            let cells: Vec<&str> = ln.split(',').collect();
            if cells.len() != head.len() {
                continue;
            }
            for (j, c) in cells.iter().enumerate() {
                cols[j].push(c.parse().unwrap_or(f64::NAN));
                strs[j].push(c.to_string());
            }
            n += 1;
        }
        Csv {
            cols: head.iter().cloned().zip(cols).collect(),
            strs: head.into_iter().zip(strs).collect(),
            n,
        }
    }
    pub fn c(&self, k: &str) -> &[f64] {
        self.cols.get(k).unwrap_or_else(|| panic!("missing column {k}"))
    }
    pub fn has(&self, k: &str) -> bool {
        self.cols.contains_key(k)
    }
    pub fn v(&self, p: &str, i: usize) -> V3 {
        if p == "com" || p == "root" {
            return self.raw_v(p, i) - self.kb_offset(i);
        }
        self.raw_v(p, i)
    }
    pub fn raw_v(&self, p: &str, i: usize) -> V3 {
        v3(self.c(&format!("{p}_x"))[i], self.c(&format!("{p}_y"))[i], self.c(&format!("{p}_z"))[i])
    }
    /// Position correction for Krakensbane: while it is engaged the logger sees bodies already
    /// shifted by -FrameVel*dt for the coming PhysX step, so logged positions lead by kb*dt.
    /// Uses the body-frame frame velocity when logged, else the (equal while engaged) direction of
    /// the logged vessel velocity scaled to |kb|.
    pub fn kb_offset(&self, i: usize) -> V3 {
        let dt = self.c("dt")[i];
        if self.has("kbbf_x") {
            return self.raw_v("kbbf", i) * dt;
        }
        let kbm = self.raw_v("kb", i).norm();
        if kbm == 0.0 {
            return V3::ZERO;
        }
        self.raw_v("vsrf", i).normalized() * (kbm * dt)
    }
}

pub struct Trace {
    pub name: String,
    pub header: HashMap<String, String>,
    pub geom: Vec<GeomPart>,
    pub geom_end: Option<Vec<GeomPart>>,
    pub csv: Csv,
    /// per-row scenery translation (see ground_shift_at)
    pub gshift: Vec<V3>,
    /// per-session KSC collider offset (sim::Scenery::ksc_dz), from the ground probe; 0 when
    /// not logged (older traces fold everything into gshift)
    pub ksc_dz: f64,
}

impl Trace {
    pub fn load(dir: &Path, name: &str) -> std::io::Result<Trace> {
        let (header, geom) = parse_geometry(&std::fs::read_to_string(dir.join(format!("{name}.geometry.cfg")))?);
        let geom_end = std::fs::read_to_string(dir.join(format!("{name}.geometry.end.cfg"))).ok().map(|t| parse_geometry(&t).1);
        let csv = Csv::parse(&std::fs::read_to_string(dir.join(format!("{name}.csv")))?);
        let gshift = ground_shift_series(&csv);
        let ksc_dz = ksc_dz_from_probe(&csv, &gshift);
        Ok(Trace { name: name.to_string(), header, geom, geom_end, csv, gshift, ksc_dz })
    }
    /// VesselSpec from this trace's geometry (wheel colliders from the end dump if needed).
    pub fn spec(&self) -> VesselSpec {
        spec_from_geometry_wheels(&self.geom, self.geom_end.as_deref())
    }
}

// --- Unity root frame -> sim root frame -------------------------------------------------------

pub fn mpos(v: &[f64]) -> V3 {
    v3(-v[0], v[1], v[2])
}
pub fn mquat(q: &[f64]) -> Quat {
    Quat::new(q[0], -q[1], -q[2], q[3]).normalized()
}
fn mat_of(q: Quat) -> [[f64; 3]; 3] {
    let ex = q.rotate(v3(1.0, 0.0, 0.0));
    let ey = q.rotate(v3(0.0, 1.0, 0.0));
    let ez = q.rotate(v3(0.0, 0.0, 1.0));
    [[ex.x, ey.x, ez.x], [ex.y, ey.y, ez.y], [ex.z, ey.z, ez.z]]
}

/// A VesselSpec from the game's geometry dump: stock part definitions supply the module data
/// (engine curves, wheel torque, resources), the dump supplies every geometric / inertial value.
/// Airbrakes (`airbrake1`) load like any catalog part; their flap deflection is state
/// (`EnvState::brakes` / `brake_deflection`), which `state_at` leaves stowed: set it from the trace.
pub fn spec_from_geometry(geom: &[GeomPart]) -> VesselSpec {
    spec_from_geometry_wheels(geom, None)
}

/// As `spec_from_geometry`, with wheel colliders taken from `wheels` (e.g. the end-of-test dump,
/// where the legs are deployed) when the main dump has none.
pub fn spec_from_geometry_wheels(geom: &[GeomPart], wheels: Option<&[GeomPart]>) -> VesselSpec {
    let mut craft = Craft::default();
    for g in geom {
        let sim_name = g.name.replace('.', "_");
        let def = parts::by_name(&sim_name).unwrap_or_else(|| panic!("no sim part def for {}", g.name));
        let pos = mpos(&g.vec("localPos").unwrap());
        let rot = mquat(&g.vec("localRot").unwrap());
        craft.parts.push(Placed { def, pos, rot, stack: vec![], parent: (g.parent >= 0).then_some(g.parent as usize) });
    }
    let mut s = VesselSpec::compile(&craft);
    let mut e_idx = 0;
    // joint breakForce per part from the dump (INF for the root); hull points take their part's
    for (i, g) in geom.iter().enumerate() {
        let bf = g.get("joint").and_then(|j| fields(j).get("breakForce").map(|b| nums(b)[0])).unwrap_or(f64::INFINITY);
        s.part_break_force[i] = if g.parent >= 0 { bf } else { f64::INFINITY };
    }
    for k in 0..s.n_hull as usize {
        s.hull_break_force[k] = s.part_break_force[s.hull_part[k] as usize];
    }
    for (i, g) in geom.iter().enumerate() {
        // rigidbody CoM, inertia (Unity: diag inertiaTensor in the frame inertiaTensorRotation
        // relative to the part transform), all per tonne of the dump-time rb.mass
        let m = g.vec("rb.mass").map(|v| v[0]).unwrap_or(1.0);
        if let Some(c) = g.vec("rb.worldCoM_rootLocal") {
            s.part_com[i] = mpos(&c);
        }
        if let (Some(it), Some(itr)) = (g.vec("rb.inertiaTensor"), g.vec("rb.inertiaTensorRotation")) {
            // Unity-frame tensor in root coordinates, then mirror (x -> -x) into the sim frame
            let qr = Quat::new(itr[0], itr[1], itr[2], itr[3]);
            let lr = g.vec("localRot").unwrap();
            let qp = Quat::new(lr[0], lr[1], lr[2], lr[3]);
            let r = mat_of(qp * qr);
            let d = [it[0], it[1], it[2]];
            let mut t = [[0.0; 3]; 3];
            for a in 0..3 {
                for b in 0..3 {
                    t[a][b] = (0..3).map(|k| r[a][k] * d[k] * r[b][k]).sum::<f64>() / m;
                    if (a == 0) != (b == 0) {
                        t[a][b] = -t[a][b];
                    }
                }
            }
            s.part_inertia_root[i] = t;
        }
        // dragModelType none (airbrake): the game evaluates the cube but applies no cube drag
        let cube_drag = parts::aero_surface(&g.name.replace('.', "_")).is_none_or(|a| a.cube_drag);
        if let (true, Some(ao), Some(wd)) = (cube_drag, g.vec("areaOccluded"), g.vec("weightedDrag")) {
            let mut f = AeroFaces { area: [ao[0], ao[1], ao[2], ao[3], ao[4], ao[5]], drag: [wd[0], wd[1], wd[2], wd[3], wd[4], wd[5]] };
            f.area.swap(0, 1);
            f.drag.swap(0, 1);
            s.part_faces[i] = f;
            for k in 0..6 {
                s.part_face_ln_cd[i][k] = if f.drag[k] < 1.0 {
                    (crate::curve::eval(crate::aero::DRAG_CD, f.drag[k] as f32 as f64) as f32 as f64).ln()
                } else {
                    f64::NAN
                };
            }
        }
        let wsrc = if g.get("wheelCollider").is_some() { Some(g) } else { wheels.and_then(|w| w.get(i)) };
        if let Some(wg) = wsrc {
            if let Some(wc) = wg.get("wheelCollider") {
                let f = fields(wc);
                let li = (0..s.n_legs as usize).position(|k| s.legs[k].sprung_mass > 0.0 && {
                    // legs are compiled in part order: count legs before part i
                    let before = geom[..i].iter().filter(|q| q.name.starts_with("landingLeg")).count();
                    k == before
                });
                if let Some(k) = li {
                    s.legs[k].anchor = mpos(&nums(&f["pos"]));
                    s.legs[k].up = mpos(&nums(&f["up"])).normalized();
                    s.legs[k].radius = nums(&f["radius"])[0];
                    s.legs[k].travel = nums(&f["suspDist"])[0];
                }
            }
        }
        // thrust transforms: the settled (end-of-test) pose when available -- after spawn the engine
        // joint sags by 1e-6..2e-4 rad in a random direction (7 flights), constant thereafter. A
        // larger change means the end pose is a deformed joint (tip-over / crash): use the start.
        let start: Vec<&str> = g.all("thrustTransform").collect();
        let tsrc: Vec<&str> = match wheels.and_then(|w| w.get(i)) {
            Some(we) if we.get("thrustTransform").is_some() => {
                let end: Vec<&str> = we.all("thrustTransform").collect();
                let settled = end.len() == start.len()
                    && end.iter().zip(&start).all(|(a, b)| {
                        let (fa, fb) = (mpos(&nums(&fields(a)["fwd"])).normalized(), mpos(&nums(&fields(b)["fwd"])).normalized());
                        fa.dot(fb).clamp(-1.0, 1.0).acos() < 1e-3
                    });
                if settled { end } else { start }
            }
            _ => start,
        };
        for tt in tsrc {
            let f = fields(tt);
            let p = mpos(&nums(&f["pos"]));
            let fw = mpos(&nums(&f["fwd"]));
            if e_idx < s.n_engines as usize {
                if let Some(gt) = g.get("gimbalTransform") {
                    s.engines[e_idx].pivot = mpos(&nums(&fields(gt)["pos"]));
                }
                s.engines[e_idx].pos = p;
                s.engines[e_idx].dir = -fw.normalized();
                if let Some(mf) = f.get("maxFuelFlow") {
                    s.engines[e_idx].max_fuel_flow = nums(mf)[0];
                }
                e_idx += 1;
            }
        }
    }
    s.update_bounds();
    s.update_ln_cd32();
    s
}

/// Pilot input of tick `i` (FlightCtrlState incl. trims, as the game applies them).
pub fn control(t: &Trace, i: usize) -> Control {
    let c = &t.csv;
    Control {
        throttle: c.c("c_thr")[i],
        pitch: (c.c("c_pitch")[i] + c.c("c_pitchTrim")[i]).clamp(-1.0, 1.0),
        yaw: (c.c("c_yaw")[i] + c.c("c_yawTrim")[i]).clamp(-1.0, 1.0),
        roll: (c.c("c_roll")[i] + c.c("c_rollTrim")[i]).clamp(-1.0, 1.0),
    }
}

/// Scenery translation in effect at row `i` (EnvState::ground_shift), measured from the trace:
/// the logged PQS placement offset (`pqs_*`) when present, else the
/// mean height of the logged WheelHit points above the sim ground at the nearest row with legs
/// on the ground (forward first; origin shifts only happen while > 500 m away, so the landed
/// rows carry the placement the touchdown sees). Zero when nothing is grounded.
pub fn ground_shift_at(t: &Trace, i: usize) -> V3 {
    t.gshift[i]
}

/// KSC placement offset of the session: ground probe (pad centre) minus the surveyed deck there,
/// minus the radial part of the PQS translation (median over the logged rows).
fn ksc_dz_from_probe(c: &Csv, gshift: &[V3]) -> f64 {
    if !c.has("probe_alt") {
        return 0.0;
    }
    let prm = sim::Params::default();
    let p0 = crate::ksc::KSC_PAD_DECK_BODY_FIXED;
    let up = p0.normalized();
    let nominal = p0.norm() - prm.body.radius - sim::ground_height(&prm, p0);
    let mut v: Vec<f64> = (0..c.n).map(|i| c.c("probe_alt")[i] - nominal - gshift[i].dot(up)).filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

fn ground_shift_series(c: &Csv) -> Vec<V3> {
    let prm = sim::Params::default();
    let up = crate::ksc::KSC_PAD_DECK_BODY_FIXED.normalized();
    // SimHarness >= v7 logs the PQS transform's float-rounding offset from the double body
    // position (body frame): the whole scenery is translated by exactly that vector (checked
    // against the ground probe to 3e-5 m). The survey was taken at zero offset.
    if c.has("pqs_x") {
        return (0..c.n).map(|i| v3(c.c("pqs_x")[i], c.c("pqs_y")[i], c.c("pqs_z")[i])).collect();
    }
    let nw = (0..8).filter(|k| c.has(&format!("wc{k}_hx"))).count();
    let cols: Vec<[&[f64]; 4]> = (0..nw)
        .map(|k| [c.c(&format!("wc{k}_grounded")), c.c(&format!("wc{k}_hx")), c.c(&format!("wc{k}_hy")), c.c(&format!("wc{k}_hz"))])
        .collect();
    let at: Vec<Option<V3>> = (0..c.n)
        .map(|j| {
            let (mut s, mut n) = (0.0, 0);
            for col in &cols {
                if col[0][j] > 0.5 && col[1][j].is_finite() {
                    s += sim::ground_height(&prm, v3(col[1][j], col[2][j], col[3][j]));
                    n += 1;
                }
            }
            (n > 0).then(|| up * (s / n as f64))
        })
        .collect();
    // nearest grounded row forward, else backward, else zero
    let mut out = vec![None; c.n];
    let mut next = None;
    for j in (0..c.n).rev() {
        if at[j].is_some() {
            next = at[j];
        }
        out[j] = next;
    }
    let mut prev = None;
    out.iter()
        .enumerate()
        .map(|(j, o)| {
            if at[j].is_some() {
                prev = at[j];
            }
            o.or(prev).unwrap_or(V3::ZERO)
        })
        .collect()
}

/// Sim state equal to the logged state at tick `i` (needs i >= 10 for the smoother history).
pub fn state_at(t: &Trace, spec: &VesselSpec, i: usize) -> EnvState {
    let c = &t.csv;
    let up = c.v("up", i).normalized();
    let fw = c.v("fwd", i).normalized();
    let q = sim::attitude(up, fw);
    let root = c.v("root", i);
    let mut st = EnvState {
        ut: c.c("ut")[i],
        frame: if c.c("rotframe")[i] > 0.5 { Frame::Rotating } else { Frame::Inertial },
        x: root,
        q,
        w: -c.v("w", i),
        toff: f32::NAN,
        ground_shift: ground_shift_at(t, i),
        ksc_dz: t.ksc_dz,
        ..Default::default()
    };
    st.res = [c.c("lf")[i], c.c("ox")[i], c.c("mono")[i], c.c("ec")[i]];
    // seed from the vessel-level (mass-weighted) CoM position / velocity, which is what the game
    // integrates as a whole and what we compare against; the root part's own rb state carries
    // the small joint jitter of the individual parts
    let (_, com_l, _) = spec.mass_props(&st.res);
    let rc = q.rotate(com_l);
    st.x = c.v("com", i) - rc;
    st.v = c.v("vsrf", i) - st.w.cross(rc);
    let _ = root;
    for k in 0..spec.n_engines as usize {
        st.thr[k] = c.c(&format!("e{k}_thr"))[i];
        // ModuleGimbal.actuation is the normalised (-1..1) deflection; the sim stores degrees
        if c.has(&format!("g{k}_ax")) {
            let r = spec.engines[k].gimbal_range_deg;
            st.gimbal[k] = [c.c(&format!("g{k}_ax"))[i] * r, c.c(&format!("g{k}_az"))[i] * r];
        }
    }
    // ModuleReactionWheel.inputVector logged at row i is already tick i's lerp; tick i starts
    // from row i-1
    if c.has("rw0_x") {
        st.rw_in = c.raw_v("rw0", i.saturating_sub(1));
    }
    // landing legs: WheelHit of the last PhysX step (row i) + the hidden suspension state as it
    // was before tick i's module updates (row i-1)
    let j = i.saturating_sub(1);
    for k in 0..spec.n_legs as usize {
        let l = &spec.legs[k];
        let mut ls = crate::wheel::LegState::fresh();
        let w = |n: &str, r: usize| c.cols.get(&format!("{n}")).map(|v| v[r] as f32);
        if let Some(g) = w(&format!("wc{k}_grounded"), i) {
            ls.grounded = g > 0.5;
            if ls.grounded {
                // VPP's contactDepth (hit point + rb.velocity*dt against the current transform)
                // equals the jounce PhysX computed in the last step; older traces lack it
                ls.x = match w(&format!("ws{k}_cd"), i) {
                    Some(cd) if cd.is_finite() => cd,
                    _ => (l.travel - c.c(&format!("wc{k}_compr"))[i]) as f32,
                };
                ls.hit_force = c.c(&format!("wc{k}_force"))[i] as f32;
            }
            ls.k = w(&format!("wc{k}_spring"), j).unwrap_or(0.0);
            ls.c = w(&format!("wc{k}_damper"), j).unwrap_or(0.0);
            ls.target = w(&format!("wc{k}_target"), j).unwrap_or(f32::NAN);
        }
        if let Some(stw) = w(&format!("ws{k}_stw"), j) {
            ls.stw = stw;
            ls.dtw = w(&format!("ws{k}_dtw"), j).unwrap();
            ls.auto_boost = w(&format!("ws{k}_ab"), j).unwrap();
            ls.auto_set = w(&format!("ws{k}_set"), j).unwrap() > 0.5;
            ls.m_spring = w(&format!("ws{k}_spring"), j).unwrap();
            ls.m_damper = w(&format!("ws{k}_damper"), j).unwrap();
            ls.rate = w(&format!("ws{k}_vp_k"), j).unwrap();
            ls.drate = w(&format!("ws{k}_vp_c"), j).unwrap();
            ls.rt = w(&format!("ws{k}_vp_rtk"), j).unwrap();
            ls.rtd = w(&format!("ws{k}_vp_rtc"), j).unwrap();
            ls.tire = [w(&format!("ws{k}_ltf_x"), j).unwrap(), w(&format!("ws{k}_ltf_y"), j).unwrap()];
        } else if ls.k > 0.0 {
            // older traces: only the native values are known
            ls.rt = ls.k;
            ls.rtd = ls.c;
            ls.rate = ls.k;
            ls.drate = ls.c;
        }
        st.legs[k] = ls;
    }
    let n = VEL_SMOOTH_LEN;
    for j in 0..n {
        let r = i.saturating_sub(n - j);
        let vs = c.v("vsrf", r);
        st.smooth_buf[j] = [vs.x as f32, vs.y as f32, vs.z as f32];
    }
    st.smooth_idx = 0;
    st
}

/// Per-tick comparison of a sim replay against the trace.
#[derive(Clone, Debug, Default)]
pub struct Errors {
    pub ticks: Vec<usize>,
    pub pos: Vec<f64>,
    pub vel: Vec<f64>,
    pub att_deg: Vec<f64>,
    pub fuel: Vec<f64>,
}

impl Errors {
    pub fn max(v: &[f64]) -> f64 {
        v.iter().cloned().fold(0.0, f64::max)
    }
}

/// Replay ticks [start, end) open-loop with the logged pilot input; compare CoM position,
/// surface velocity, attitude (root up axis) and liquid fuel after every tick.
pub fn replay(t: &Trace, spec: &VesselSpec, prm: &sim::Params, start: usize, end: usize) -> (Errors, EnvState) {
    let mut st = state_at(t, spec, start);
    let c = &t.csv;
    let mut e = Errors::default();
    for i in start..end.min(c.n - 1) {
        sim::tick(spec, prm, &mut st, &control(t, i));
        let k = i + 1;
        let (_, com_l, _) = spec.mass_props(&st.res);
        let rc = st.q.rotate(com_l);
        let x_com = st.x + rc;
        let v_com = st.v + st.w.cross(rc);
        e.ticks.push(k);
        e.pos.push((x_com - c.v("com", k)).norm());
        e.vel.push((v_com - c.v("vsrf", k)).norm());
        let up = st.q.rotate(v3(0.0, 1.0, 0.0));
        e.att_deg.push(up.dot(c.v("up", k).normalized()).clamp(-1.0, 1.0).acos().to_degrees());
        e.fuel.push(st.res[0] - c.c("lf")[k]);
    }
    let _ = N_RES;
    (e, st)
}

/// CoM position / velocity of a sim state.
pub fn com_state(spec: &VesselSpec, st: &EnvState) -> (V3, V3) {
    let (_, com_l, _) = spec.mass_props(&st.res);
    let rc = st.q.rotate(com_l);
    (st.x + rc, st.v + st.w.cross(rc))
}

/// One-step (teacher-forced) errors: seed from the log at every tick i in [start, end), take one
/// sim tick, compare with the log at i+1. Returns per-tick |accel error| (m/s^2) and |angular
/// accel error| (rad/s^2).
pub fn one_step(t: &Trace, spec: &VesselSpec, prm: &sim::Params, start: usize, end: usize) -> (Vec<usize>, Vec<f64>, Vec<f64>) {
    let c = &t.csv;
    let (mut ks, mut ea, mut ew) = (vec![], vec![], vec![]);
    for i in start.max(VEL_SMOOTH_LEN)..end.min(c.n - 1) {
        let mut st = state_at(t, spec, i);
        sim::tick(spec, prm, &mut st, &control(t, i));
        let (_, v) = com_state(spec, &st);
        ks.push(i);
        ea.push((v - c.v("vsrf", i + 1)).norm() / sim::FIXED_DT);
        ew.push((st.w - (-c.v("w", i + 1))).norm() / sim::FIXED_DT);
    }
    (ks, ea, ew)
}

/// Windowed open-loop replay: re-seed from the log every `win` ticks; per-window end errors
/// (position m, velocity m/s, attitude deg).
pub fn windowed(t: &Trace, spec: &VesselSpec, prm: &sim::Params, start: usize, end: usize, win: usize) -> Vec<(usize, f64, f64, f64)> {
    let mut out = vec![];
    let mut s = start.max(VEL_SMOOTH_LEN);
    while s + win < end.min(t.csv.n - 1) {
        let (e, _) = replay(t, spec, prm, s, s + win);
        if let (Some(p), Some(v), Some(a)) = (e.pos.last(), e.vel.last(), e.att_deg.last()) {
            out.push((s, *p, *v, *a));
        }
        s += win;
    }
    out
}

/// Translation-only replay: the attitude and angular velocity follow the log every tick (so the
/// closed-loop kOS steering is reproduced exactly), translation and mass integrate freely.
pub fn replay_attitude_locked(t: &Trace, spec: &VesselSpec, prm: &sim::Params, start: usize, end: usize) -> Errors {
    let c = &t.csv;
    let mut st = state_at(t, spec, start);
    let mut e = Errors::default();
    for i in start..end.min(c.n - 1) {
        // impose the logged attitude about the current CoM (keeps CoM position / velocity)
        let (xc, vc) = com_state(spec, &st);
        let lg = state_at(t, spec, i);
        st.q = lg.q;
        st.w = lg.w;
        let (_, com_l, _) = spec.mass_props(&st.res);
        let rc = st.q.rotate(com_l);
        st.x = xc - rc;
        st.v = vc - st.w.cross(rc);
        sim::tick(spec, prm, &mut st, &control(t, i));
        let k = i + 1;
        let (xc, vc) = com_state(spec, &st);
        e.ticks.push(k);
        e.pos.push((xc - c.v("com", k)).norm());
        e.vel.push((vc - c.v("vsrf", k)).norm());
        e.att_deg.push(0.0);
        e.fuel.push(st.res[0] - c.c("lf")[k]);
    }
    e
}

/// Touchdown comparison: open-loop replay from `pre` ticks before the first wheel contact (after
/// at least 25 airborne ticks) to `post` ticks after it.
#[derive(Clone, Debug, Default)]
pub struct Touchdown {
    pub game_tick: usize,
    pub sim_tick: usize,
    /// vertical (radial) speed at first contact, m/s (negative = down)
    pub game_vz: f64,
    pub sim_vz: f64,
    /// CoM position error at the game's touchdown tick and at the end of the window (m)
    pub pos_err_td: f64,
    pub pos_err_end: f64,
    /// horizontal CoM error at the end (m) and max velocity error over the window (m/s)
    pub horiz_err_end: f64,
    pub vel_err_max: f64,
    pub att_err_end_deg: f64,
}

pub fn touchdowns(t: &Trace, spec: &VesselSpec, prm: &sim::Params, pre: usize, post: usize) -> Vec<Touchdown> {
    let c = &t.csv;
    let nw = spec.n_legs as usize;
    if nw == 0 || !c.has("wc0_grounded") {
        return vec![];
    }
    let any = |i: usize| (0..nw).any(|k| c.c(&format!("wc{k}_grounded"))[i] > 0.5);
    let mut out = vec![];
    let mut air = 0;
    for i in 1..c.n {
        if !any(i) {
            air += 1;
            continue;
        }
        if air >= 25 && i > pre + VEL_SMOOTH_LEN {
            let start = i - pre;
            let end = (i + post).min(c.n - 1);
            let mut st = state_at(t, spec, start);
            let mut td = Touchdown { game_tick: i, ..Default::default() };
            let up = c.v("com", i).normalized();
            td.game_vz = c.v("vsrf", i).dot(up);
            for j in start..end {
                sim::tick(spec, prm, &mut st, &control(t, j));
                let k = j + 1;
                let (xc, vc) = com_state(spec, &st);
                if td.sim_tick == 0 && st.legs[..nw].iter().any(|l| l.grounded) {
                    td.sim_tick = k;
                    td.sim_vz = vc.dot(xc.normalized());
                }
                if k == i {
                    td.pos_err_td = (xc - c.v("com", k)).norm();
                }
                td.vel_err_max = td.vel_err_max.max((vc - c.v("vsrf", k)).norm());
                if k == end {
                    let d = xc - c.v("com", k);
                    td.pos_err_end = d.norm();
                    let u = c.v("com", k).normalized();
                    td.horiz_err_end = (d - u * d.dot(u)).norm();
                    let upv = st.q.rotate(v3(0.0, 1.0, 0.0));
                    td.att_err_end_deg = upv.dot(c.v("up", k).normalized()).clamp(-1.0, 1.0).acos().to_degrees();
                }
            }
            out.push(td);
        }
        air = 0;
    }
    out
}
