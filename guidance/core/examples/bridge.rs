//! In-game bridge server: flies the demo booster in KSP with this guidance, one game tick at a
//! time, over TCP 127.0.0.1:7777 (client: ingame/plugin/Bridge.cs).
//!
//! client -> "S ut rotframe com(3) vsrf(3) up(3) fwd(3) w(3) lf ox mono ec thr gimbal_x gimbal_z
//!            rw(3) landed kb(3)"   (body frame, SimHarness conventions; resources of the booster only)
//! server -> "C throttle pitch yaw roll brakes decouple phase done"
//!
//! Each tick the game state is seeded into the guidance exactly as trace.rs::state_at seeds a
//! logged tick, and the guidance's command for that tick is returned. A CSV of what the guidance
//! saw and commanded goes to `$BRIDGE_LOG` (default /tmp/bridge_guidance.csv).
//! usage: cargo run --release --example bridge
use ksp_core::math::v3;
use ksp_guidance::ingame;
use ksp_guidance::mission::{Flight, GuidanceConfig, Shared};
use ksp_guidance::nav::{nav, Pad};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::Arc;

fn main() {
    let sh = Arc::new(Shared::new(ingame::built().expect("demo craft builds")));
    let listener = TcpListener::bind("127.0.0.1:7777").expect("bind 7777");
    eprintln!("bridge: listening on 127.0.0.1:7777");
    let log_path = std::env::var("BRIDGE_LOG").unwrap_or_else(|_| "/tmp/bridge_guidance.csv".into());
    for conn in listener.incoming() {
        let mut s = match conn {
            Ok(s) => s,
            Err(e) => {
                eprintln!("bridge: accept failed {e}");
                continue;
            }
        };
        s.set_nodelay(true).ok();
        eprintln!("bridge: client connected");
        let mut log = std::io::BufWriter::new(std::fs::File::create(&log_path).expect("log file"));
        writeln!(log, "tick,ut,phase,e,n,u,ve,vn,vu,tilt_deg,thr,pitch,yaw,roll,brakes,decouple,prop,status").ok();
        let mut rd = BufReader::new(s.try_clone().unwrap());
        let mut flight: Option<Flight> = None;
        let mut line = String::new();
        let mut tick = 0u64;
        let mut hop_state = HopState::default();
        let pad = Pad::ksc();
        loop {
            line.clear();
            if rd.read_line(&mut line).unwrap_or(0) == 0 {
                break;
            }
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.first() == Some(&"Q") {
                break;
            }
            if f.first() != Some(&"S") || f.len() < 32 {
                eprintln!("bridge: bad line {line:?}");
                continue;
            }
            let x: Vec<f64> = f[1..].iter().map(|t| t.parse().unwrap_or(f64::NAN)).collect();
            let ut = x[0];
            let fl = flight.get_or_insert_with(|| {
                let mut fl = Flight::new(sh.clone(), ingame::nominal_dispersion(ut), GuidanceConfig::default());
                fl.external = true;
                fl
            });
            let spec = *fl.spec();
            let st = ingame::state_from_bridge(&x, &spec);
            let thrust_est = 0.0;
            let hold = std::env::var("BRIDGE_HOLD").is_ok();
            let hop = std::env::var("BRIDGE_HOP").is_ok();
            let ctl = if hold {
                // pad-only plumbing check: never light the engine, just log what the guidance sees
                fl.st = st;
                ksp_core::sim::Control::default()
            } else if hop {
                // validation hop (INGAME_FLIGHT_NOTES.md "HeavyPadHop"): settle, climb ~15 m,
                // hover with pitch / yaw / roll steps, slow descent, engine off on contact
                fl.st = st;
                hop_ctl(&mut hop_state, &spec, &fl.st, tick)
            } else {
                fl.step_external(st, thrust_est)
            };
            let brakes = fl.st.brakes;
            let done = fl.done() || (hold && tick >= 1000) || (hop && hop_state.done_tick.is_some_and(|d| tick > d + 250));
            let reply = format!(
                "C {:.6} {:.6} {:.6} {:.6} {} {} {} {}\n",
                ctl.throttle, ctl.pitch, ctl.yaw, ctl.roll, brakes as u8, fl.ext_decouple as u8, fl.phase as u8, done as u8
            );
            s.write_all(reply.as_bytes()).ok();
            tick += 1;
            let n = nav(fl.spec(), &fl.st);
            let (e, nn) = pad.ground_offset(n.x);
            let venu = v3(n.v.dot(pad.e), n.v.dot(pad.n), n.v.dot(pad.u));
            let tilt = n.up.dot(n.x.normalized()).clamp(-1.0, 1.0).acos().to_degrees();
            let (p, _) = ksp_guidance::nav::propellant(fl.spec(), &fl.st.res);
            writeln!(
                log,
                "{tick},{ut:.3},{},{e:.3},{nn:.3},{:.3},{:.3},{:.3},{:.3},{tilt:.2},{:.4},{:.4},{:.4},{:.4},{},{},{p:.4},{}",
                fl.phase as u8, n.alt, venu.x, venu.y, venu.z, ctl.throttle, ctl.pitch, ctl.yaw, ctl.roll, brakes as u8, fl.ext_decouple as u8, fl.st.status
            )
            .ok();
            if done && tick % 50 == 0 {
                eprintln!("bridge: done {:?} {:?}", fl.outcome, fl.stats);
            }
        }
        log.flush().ok();
        if let Some(fl) = &flight {
            eprintln!("bridge: client gone; outcome {:?}\n{:#?}", fl.outcome, fl.stats);
        }
    }
}

#[derive(Default)]
struct HopState {
    h0: Option<f64>,
    lit: bool,
    landed_from: Option<u64>,
    done_tick: Option<u64>,
}

/// Scripted validation hop (ticks at 50 Hz): 0-8 s settle; climb at 3 m/s to 15 m; hover with
/// 0.5 s steps of pitch +0.4 (t 16 s), yaw +0.4 (20 s), roll +0.6 (24 s) on top of the attitude
/// hold; from 30 s descend at 1.5 m/s; engine off on ground contact.
fn hop_ctl(hs: &mut HopState, spec: &ksp_core::vessel::VesselSpec, st: &ksp_core::sim::EnvState, tick: u64) -> ksp_core::sim::Control {
    use ksp_core::sim::Control;
    let n = ksp_guidance::nav::nav(spec, st);
    let up = n.x.normalized();
    let h0 = *hs.h0.get_or_insert(n.alt);
    let h = n.alt - h0;
    let vz = n.v.dot(up);
    let t = tick as f64 * 0.02;
    if t < 8.0 || hs.done_tick.is_some() {
        return Control::default();
    }
    let pr = ksp_guidance::nav::propulsion(spec, ksp_guidance::nav::p_atm(n.alt));
    let g = 9.81;
    let vz_t = if t < 30.0 { if h < 15.0 { 3.0f64.min(0.6 * (15.0 - h) + 0.3) } else { 0.0 } } else { -1.5 };
    let thr = if pr.thrust > 0.0 { (n.m * (g + 1.5 * (vz_t - vz)) / pr.thrust).clamp(0.02, 1.0) } else { 0.0 };
    if t > 31.0 && st.status != ksp_core::sim::STATUS_FLYING {
        let lf = *hs.landed_from.get_or_insert(tick);
        if tick > lf + 5 {
            hs.done_tick = Some(tick);
            return Control::default();
        }
    }
    hs.lit = true;
    let o = ksp_guidance::attitude::point(spec, st, &n, up, thr, false);
    let mut c = o.ctl;
    c.throttle = thr;
    let step = |t0: f64| t >= t0 && t < t0 + 0.5;
    if step(16.0) { c.pitch = 0.4; }
    if step(20.0) { c.yaw = 0.4; }
    if step(24.0) { c.roll = 0.6; }
    c
}
