//! Airbrake (ModuleAeroSurface) model vs the in-game RTLS demo flights.
//!
//! Data: ingame/telemetry/flight{1,2}_kos.csv (kOS, ~4 Hz; t, phase, alt ASL, vz, vh surface,
//! tilt of the booster axis from up, mass, q, lat, lng). Unpowered descent with the 4 airbrakes
//! deployed: flight 1 from apogee, flight 2 from the end of boostback, to the landing burn.
//!
//! Two comparisons, both on ksp-core's own `sim::tick`:
//! 1. drag deceleration at every logged state: measured = -(dv/dt - gravity - frame terms) . v_hat
//!    from the logged velocities (1 s windows); model = (v after one tick with aero - without) / dt
//!    from the logged state, for the booster with brakes deployed, stowed, and with no airbrakes.
//! 2. open-loop integration of each descent segment from its first state, attitude pinned to the
//!    logged tilt (kOS steering holds it; aero torque is not compared).
//!
//! Unknowns (not logged): the azimuth of the tilt (taken in the plane of the horizontal velocity,
//! both senses reported: +1 = top leaning along the horizontal velocity, -1 = against it), roll
//! (the brakes' azimuth relative to that plane; 4-fold symmetric), and UT (sun temperature
//! offset; the demo save starts at UT 0, flight 2 launched near UT 67).
//!
//! usage: cargo run --release -p ksp-core --example airbrake_validate [telemetry dir]
use ksp_core::body::KERBIN;
use ksp_core::ksc;
use ksp_core::math::{v3, Quat, V3};
use ksp_core::parts::{self, Res, AIRBRAKE_NODE};
use ksp_core::sim::{self, Control, EnvState, Frame, Params};
use ksp_core::trace::Csv;
use ksp_core::vessel::{Craft, Placed, VesselSpec};
use std::path::PathBuf;

const UT0: f64 = 67.0;

/// The booster after staging (ingame/make_demo_craft.py geometry): TD-12 (stays on the booster,
/// explosiveNodeID = top) / RC-001S / Z-1k / small reaction wheel / FL-T800 / FL-T400 / Swivel,
/// 4 LT-2 (retracted cube) 0.9 m below the FL-T400 centre at 45 + 90k deg, 4 airbrakes 1.45 m
/// above the FL-T800 centre at 90k deg.
fn booster(brakes: bool) -> Craft {
    let mut c = Craft { parts: vec![Placed { def: parts::decoupler_td12(), pos: V3::ZERO, rot: Quat::IDENTITY, stack: vec![], parent: None }] };
    let p = c.stack_below(0, parts::probe_stack_small());
    let b = c.stack_below(p, parts::battery_bank());
    let w = c.stack_below(b, parts::adv_sas_module());
    let t8 = c.stack_below(w, parts::fuel_tank_t800());
    let t4 = c.stack_below(t8, parts::fuel_tank_t400());
    c.stack_below(t4, parts::swivel());
    let (np, nd) = (v3(0.0, 0.4271562, 0.0632596) * 1.25, v3(0.0, 0.5735765, 0.819152));
    for k in 0..4 {
        c.surface_attach_editor(t4, parts::lt2_leg_retracted(), 45.0 + 90.0 * k as f64, 0.625, -0.9, np, nd);
    }
    if brakes {
        for k in 0..4 {
            c.surface_attach_editor(t8, parts::airbrake(), 90.0 * k as f64, 0.625, 1.45, AIRBRAKE_NODE.0, AIRBRAKE_NODE.1);
        }
    }
    c
}

/// Resources for a logged total mass (propellant LF:Ox 0.9:1.1 by units, EC full).
fn resources(spec: &VesselSpec, mass: f64, dry_ref: f64) -> [f64; parts::N_RES] {
    let prop = (mass - dry_ref).max(0.0) / parts::DENSITY_LIQUID_FUEL;
    let mut r = spec.full_resources();
    r[Res::LiquidFuel as usize] = prop * 0.45;
    r[Res::Oxidizer as usize] = prop * 0.55;
    r
}

fn enu(x: V3) -> (V3, V3, V3) {
    let up = x.normalized();
    let east = v3(0.0, 0.0, 1.0).cross(up).normalized();
    (east, up.cross(east), up)
}

/// Root attitude with +Y tilted from up by `tilt_deg` toward `sense * hdir` (horizontal unit).
fn attitude_at(x: V3, hdir: V3, tilt_deg: f64, sense: f64) -> Quat {
    let (_, n, u) = enu(x);
    let t = tilt_deg.to_radians();
    let h = if hdir.norm2() > 0.0 { hdir } else { n };
    let axis = u * t.cos() + h * (sense * t.sin());
    sim::attitude(axis, n)
}

struct Row {
    t: f64,
    phase: String,
    x: V3,
    v: V3,
    hdir: V3,
    tilt: f64,
    mass: f64,
    kd: f64,
}

fn load(path: &PathBuf) -> Vec<Row> {
    let c = Csv::parse(&std::fs::read_to_string(path).unwrap());
    let n = c.n;
    let pos: Vec<V3> = (0..n).map(|i| ksc::body_fixed_from_lat_lon_alt(c.c("lat")[i], c.c("lng")[i], c.c("alt")[i], KERBIN.radius)).collect();
    (0..n)
        .map(|i| {
            // horizontal direction of motion from the ground track (+-2 rows)
            let (a, b) = (i.saturating_sub(2), (i + 2).min(n - 1));
            let (_, _, u) = enu(pos[i]);
            let d = pos[b] - pos[a];
            let dh = d - u * d.dot(u);
            let hdir = if dh.norm() > 1e-3 { dh.normalized() } else { V3::ZERO };
            Row {
                t: c.c("t")[i],
                phase: c.strs["phase"][i].clone(),
                x: pos[i],
                v: u * c.c("vz")[i] + hdir * c.c("vh")[i],
                hdir,
                tilt: c.c("tilt")[i],
                mass: c.c("mass")[i],
                kd: c.c("kd_e6")[i],
            }
        })
        .collect()
}

fn frame_accel(x: V3, v: V3) -> V3 {
    let om = v3(0.0, 0.0, KERBIN.angular_velocity());
    x * (-KERBIN.gm() / x.norm().powi(3)) - om.cross(v) * 2.0 - om.cross(om.cross(x))
}

/// EnvState with the CoM at `x`, CoM velocity `v` (body-fixed), attitude `q`.
fn state(spec: &VesselSpec, res: [f64; parts::N_RES], x: V3, v: V3, q: Quat, ut: f64, defl: f32) -> EnvState {
    let (_, com_l, _) = spec.mass_props(&res);
    let mut st = EnvState { ut, frame: Frame::Rotating, x: x - q.rotate(com_l), v, q, res, toff: f32::NAN, brakes: defl > 0.0, brake_deflection: defl, ..Default::default() };
    sim::prime_smoother(&mut st, v);
    st
}

fn com_vel(spec: &VesselSpec, st: &EnvState) -> V3 {
    let (_, com_l, _) = spec.mass_props(&st.res);
    st.v + st.w.cross(st.q.rotate(com_l))
}

/// Aero acceleration (body-fixed, m/s^2) of `spec` at the state, from one tick with and without aero.
fn aero_accel(spec: &VesselSpec, st: &EnvState) -> V3 {
    let on = Params { ground: false, ..Default::default() };
    let off = Params { aero: false, ..on };
    let (mut a, mut b) = (*st, *st);
    sim::tick(spec, &on, &mut a, &Control::default());
    sim::tick(spec, &off, &mut b, &Control::default());
    (com_vel(spec, &a) - com_vel(spec, &b)) / sim::FIXED_DT
}

/// Rejected readings of the decomp, for contrast: (a) lift vector = the Flap transform's forward
/// (part +Y, if `ModuleLiftingSurface.transformName` received "Flap"), turned by the flap rotation
/// plus `airflowIncidence` (2 x deflection); (b) the cube drag of the blended airbrake cube applied
/// on top (dragModelType ignored).
fn hypotheses(spec: &VesselSpec) -> (VesselSpec, VesselSpec) {
    let mut a = *spec;
    for k in 0..a.n_aero_surf as usize {
        let s = &mut a.aero_surf[k];
        let i = s.part as usize;
        let up = spec.part_rot[i].rotate(v3(0.0, 1.0, 0.0));
        s.lift_turn = -s.lift_vector;
        s.lift_vector = up;
        // deflection doubled through deploy angle 140 (range widened so the clamp allows it)
        s.deploy_angle = 140.0;
        s.ctrl_surface_range = 100.0;
        s.actuator_speed = 40.0;
    }
    let mut b = *spec;
    let ab = parts::aero_surface("airbrake1").unwrap();
    let cube = ab.cube(&parts::airbrake().cube, 70.0);
    for k in 0..b.n_aero_surf as usize {
        let i = b.aero_surf[k].part as usize;
        b.part_faces[i] = ksp_core::aero::AeroFaces::from(&cube);
        for f in 0..6 {
            b.part_face_ln_cd[i][f] = if cube.drag[f] < 1.0 { (ksp_core::aero::eval32(ksp_core::aero::DRAG_CD, cube.drag[f]) as f32 as f64).ln() } else { f64::NAN };
        }
    }
    b.part_no_cube_drag = 0;
    b.update_ln_cd32();
    (a, b)
}

fn deflection_at(t: f64, t_on: f64) -> f32 {
    // MoveTowards at 20 deg/s from the brakes-on row
    ((t - t_on).max(0.0) * 20.0).min(70.0) as f32
}

fn main() {
    let dir = std::env::args().nth(1).map(PathBuf::from).unwrap_or_else(|| PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../ingame/telemetry")));
    let with = VesselSpec::compile(&booster(true));
    let without = VesselSpec::compile(&booster(false));
    let dry = with.dry_mass();
    let (hyp_flap, hyp_cube) = hypotheses(&with);
    // airbrake drag per unit dynamic pressure (m^2 = kN/kPa) when deployed in axial flow
    let ab = &with.aero_surf[0];
    let cd70 = ksp_core::aero::eval32(ksp_core::aero::SPEED_BRAKE_DRAG, 70f64.to_radians().sin());
    print!("one deployed airbrake, axial flow: drag / q =");
    for m in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.5, 2.0] {
        print!(" M{m}: {:.2}", ab.drag_k * cd70 * ksp_core::aero::eval32(ksp_core::aero::SPEED_BRAKE_DRAG_MACH, m));
    }
    println!(" m^2 (stowed: x {:.4})", ksp_core::aero::eval32(ksp_core::aero::SPEED_BRAKE_DRAG, 0.0) / cd70);
    println!("booster: {} parts, dry {:.3} t (log: mass - prop = 3.140 t), {} aero surfaces", with.n_parts, dry, with.n_aero_surf);
    let mut all = vec![];
    for (fname, brakes_after) in [("flight1_kos.csv", "apogee"), ("flight2_kos.csv", "boostback")] {
        let rows = load(&dir.join(fname));
        // brakes on: flight 1 at the first descending row after boostback, flight 2 at the end of boostback
        let i_bb = rows.iter().rposition(|r| r.phase == "boostback").unwrap();
        let i_on = if brakes_after == "apogee" { (i_bb..rows.len()).find(|&i| rows[i].v.dot(enu(rows[i].x).2) < 0.0).unwrap() } else { i_bb + 1 };
        let i_end = (i_on..rows.len()).find(|&i| rows[i].phase == "burn").unwrap();
        let t_on = rows[i_on].t;
        println!("\n== {fname}: brakes on at t = {t_on:.2} (alt {:.0} m), unpowered to t = {:.2} (phases {:?})", rows[i_on].x.norm() - KERBIN.radius, rows[i_end - 1].t,
            { let mut p: Vec<&str> = rows[i_on..i_end].iter().map(|r| r.phase.as_str()).collect(); p.dedup(); p });

        // --- 1. drag deceleration at logged states ---------------------------------------------
        println!("     t     alt    mach   q_kPa  tilt | a_drag meas | model +1  model -1 | stowed  no-brk | lift meas  +1    -1  | kd_e6 log  meas  model");
        let mut i = i_on;
        let mut rec = vec![];
        while i + 4 < i_end {
            let j = i + 4;
            let (r0, r1) = (&rows[i], &rows[j]);
            let dt = r1.t - r0.t;
            let xm = (r0.x + r1.x) * 0.5;
            let vm = (r0.v + r1.v) * 0.5;
            let a = (r1.v - r0.v) / dt - frame_accel(xm, vm);
            let vh = vm.normalized();
            let drag_meas = -a.dot(vh);
            let lift_meas = (a - vh * a.dot(vh)).norm();
            let tm = 0.5 * (r0.t + r1.t);
            let tilt = 0.5 * (r0.tilt + r1.tilt);
            let hd = (r0.hdir + r1.hdir).normalized();
            let defl = deflection_at(tm, t_on);
            let mass = 0.5 * (r0.mass + r1.mass);
            let res = resources(&with, mass, dry);
            let mut model = [0.0; 2];
            let mut lift = [0.0; 2];
            for (k, sense) in [1.0, -1.0].into_iter().enumerate() {
                let q = attitude_at(xm, hd, tilt, sense);
                let aa = aero_accel(&with, &state(&with, res, xm, vm, q, UT0 + tm, defl));
                model[k] = -aa.dot(vh);
                lift[k] = (aa - vh * aa.dot(vh)).norm();
            }
            let q = attitude_at(xm, hd, tilt, -1.0);
            let stowed = -aero_accel(&with, &state(&with, res, xm, vm, q, UT0 + tm, 0.0)).dot(vh);
            let res_nb = resources(&without, mass, without.dry_mass() + 0.2);
            // as the same force on the 0.2 t heavier vessel
            let m_ratio = without.mass_props(&res_nb).0 / with.mass_props(&res).0;
            let nobrk = -aero_accel(&without, &state(&without, res_nb, xm, vm, q, UT0 + tm, 0.0)).dot(vh) * m_ratio;
            let hyp = [
                -aero_accel(&hyp_flap, &state(&hyp_flap, res, xm, vm, q, UT0 + tm, 2.0 * defl)).dot(vh),
                -aero_accel(&hyp_cube, &state(&hyp_cube, res, xm, vm, q, UT0 + tm, defl)).dot(vh),
            ];
            let st = state(&with, res, xm, vm, q, UT0 + tm, defl);
            let alt = xm.norm() - KERBIN.radius;
            let atm = ksp_core::atmo::atm_state(KERBIN, alt, vm.norm(), { let mut s = st; sim::tick(&with, &Params { ground: false, ..Default::default() }, &mut s, &Control::default()); s.toff as f64 });
            let mm = ksp_core::aero::eval32(ksp_core::aero::DRAG_MACH_MULT, atm.mach);
            let kd = |a: f64| a / (mm * atm.rho * vm.norm2()) * 1e6;
            if (rec.len()) % 3 == 0 {
                println!("{tm:7.2} {alt:7.0} {:6.3} {:6.2} {tilt:5.1} | {drag_meas:9.3}   | {:8.3} {:8.3} | {stowed:6.3} {nobrk:6.3} | {lift_meas:7.3} {:6.3} {:6.3} | {:7.0} {:6.0} {:6.0}",
                    atm.mach, atm.q_kpa, model[0], model[1], lift[0], lift[1], r1.kd, kd(drag_meas), kd(model[1]));
            }
            rec.push((tm, alt, atm.mach, atm.q_kpa, drag_meas, model, stowed, nobrk, lift_meas, lift, hyp));
            i += 2;
        }
        let good: Vec<_> = rec.iter().filter(|r| r.3 > 0.5 && r.0 > t_on + 4.0).collect();
        for k in 0..2 {
            let rel: Vec<f64> = good.iter().map(|r| r.5[k] / r.4 - 1.0).collect();
            let mean = rel.iter().sum::<f64>() / rel.len() as f64;
            let rms = (rel.iter().map(|x| x * x).sum::<f64>() / rel.len() as f64).sqrt();
            println!("drag model/meas - 1 (sense {:+}): mean {:+.2} %, rms {:.2} % over {} windows with q > 0.5 kPa", [1, -1][k], 100.0 * mean, 100.0 * rms, rel.len());
        }
        for (lo, hi) in [(0.0, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 0.7), (0.7, 0.9)] {
            let b: Vec<_> = good.iter().filter(|r| r.2 >= lo && r.2 < hi).collect();
            if b.is_empty() {
                continue;
            }
            let n = b.len() as f64;
            let m = b.iter().map(|r| r.4).sum::<f64>() / n;
            let mo = b.iter().map(|r| r.5[1]).sum::<f64>() / n;
            let sw = b.iter().map(|r| r.6).sum::<f64>() / n;
            let nb = b.iter().map(|r| r.7).sum::<f64>() / n;
            let alt = b.iter().map(|r| r.1).sum::<f64>() / n;
            println!("  mach {lo:.1}-{hi:.1} (n {:2}, mean alt {alt:6.0} m): drag meas {m:6.3}  model {mo:6.3} ({:+.1} %)  stowed {sw:6.3}  no brakes {nb:6.3} m/s^2 -> deployed brakes add {:.2}x the bare booster's drag (measured {:.2}x)",
                b.len(), 100.0 * (mo / m - 1.0), mo / nb - 1.0, m / nb - 1.0);
        }
        for (name, k) in [("lift vector = Flap forward, 2 x deflection", 0), ("cube drag on as well", 1)] {
            let rel: Vec<f64> = good.iter().map(|r| r.10[k] / r.4 - 1.0).collect();
            let mean = rel.iter().sum::<f64>() / rel.len() as f64;
            println!("  rejected reading ({name}): drag model/meas - 1 mean {:+.1} %", 100.0 * mean);
        }
        let lr: Vec<(f64, f64)> = good.iter().filter(|r| r.8 > 0.2).map(|r| (r.8, r.9[1])).collect();
        println!("  lift (perpendicular aero accel), sense -1, windows with > 0.2 m/s^2: meas {:.3} model {:.3} m/s^2 (n {})",
            lr.iter().map(|x| x.0).sum::<f64>() / lr.len().max(1) as f64, lr.iter().map(|x| x.1).sum::<f64>() / lr.len().max(1) as f64, lr.len());
        all.extend(good.iter().map(|r| (r.5[0] / r.4 - 1.0, r.5[1] / r.4 - 1.0)));

        // --- 2. open-loop integration ---------------------------------------------------------
        for sense in [1.0, -1.0] {
            let r0 = &rows[i_on];
            let res = resources(&with, r0.mass, dry);
            let mut st = state(&with, res, r0.x, r0.v, attitude_at(r0.x, r0.hdir, r0.tilt, sense), UT0 + r0.t, 0.0);
            st.brakes = true;
            let prm = Params { ground: false, ..Default::default() };
            let (_, com_l, _) = with.mass_props(&st.res);
            let mut t = r0.t;
            let mut k = i_on + 1;
            let (mut e_vz, mut e_vh, mut e_alt, mut n) = (0.0, 0.0, 0.0, 0.0);
            let mut out = String::new();
            while k < i_end {
                // pin the attitude to the logged tilt, about the CoM (kOS steering holds it)
                let xc = st.x + st.q.rotate(com_l);
                let vc = st.v + st.w.cross(st.q.rotate(com_l));
                let (_, _, u) = enu(xc);
                let vhz = vc - u * vc.dot(u);
                let f = ((t - rows[k - 1].t) / (rows[k].t - rows[k - 1].t)).clamp(0.0, 1.0);
                let tilt = rows[k - 1].tilt + (rows[k].tilt - rows[k - 1].tilt) * f;
                let q = attitude_at(xc, if vhz.norm() > 0.5 { vhz.normalized() } else { rows[k].hdir }, tilt, sense);
                st.q = q;
                st.w = V3::ZERO;
                st.x = xc - q.rotate(com_l);
                st.v = vc;
                sim::tick(&with, &prm, &mut st, &Control::default());
                t += sim::FIXED_DT;
                if t >= rows[k].t - 1e-9 {
                    let r = &rows[k];
                    let xc = st.x + st.q.rotate(com_l);
                    let vc = com_vel(&with, &st);
                    let (_, _, u) = enu(xc);
                    let (vz, vh) = (vc.dot(u), (vc - u * vc.dot(u)).norm());
                    let (lvz, lvh) = (r.v.dot(enu(r.x).2), (r.v - enu(r.x).2 * r.v.dot(enu(r.x).2)).norm());
                    let alt = xc.norm() - KERBIN.radius;
                    let lalt = r.x.norm() - KERBIN.radius;
                    e_vz += (vz - lvz).powi(2);
                    e_vh += (vh - lvh).powi(2);
                    e_alt += (alt - lalt).powi(2);
                    n += 1.0;
                    if k % 24 == 0 || k + 1 == i_end {
                        out += &format!("    t {:7.2}  alt {lalt:7.0} ({:+6.1})  vz {lvz:8.2} ({:+6.2})  vh {lvh:6.2} ({:+6.2})\n", r.t, alt - lalt, vz - lvz, vh - lvh);
                    }
                    k += 1;
                }
            }
            println!("integration, tilt sense {sense:+}: rms residual vz {:.2} m/s, vh {:.2} m/s, alt {:.1} m over {} rows (log value (sim - log))", (e_vz / n).sqrt(), (e_vh / n).sqrt(), (e_alt / n).sqrt(), n);
            print!("{out}");
        }
    }
    let n = all.len() as f64;
    for k in 0..2 {
        let mean = all.iter().map(|r| if k == 0 { r.0 } else { r.1 }).sum::<f64>() / n;
        let rms = (all.iter().map(|r| (if k == 0 { r.0 } else { r.1 }).powi(2)).sum::<f64>() / n).sqrt();
        println!("both flights, drag model/meas - 1 (sense {:+}): mean {:+.2} %, rms {:.2} %, n {}", [1, -1][k], 100.0 * mean, 100.0 * rms, n);
    }
    // UT sensitivity of the density (sun temperature offset)
    let (_, _, u) = sim::pad_enu();
    let x = ksc::KSC_PAD_DECK_BODY_FIXED + u * 5000.0;
    let toff = |ut: f64| {
        let (sun, ta) = sim::sun_dir_fixed(KERBIN, ut);
        ksp_core::atmo::temperature_offset(KERBIN, sun, x.normalized(), v3(0.0, 0.0, 1.0), ta, None)
    };
    let rho = |ut: f64| ksp_core::atmo::atm_state(KERBIN, 5000.0, 100.0, toff(ut)).rho;
    println!("density at 5 km: UT 0 {:.5}, UT {UT0} {:.5}, UT 600 {:.5} kg/m^3", rho(0.0), rho(UT0), rho(600.0));
}
