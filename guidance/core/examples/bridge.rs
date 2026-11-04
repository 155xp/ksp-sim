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
use ksp_guidance::mission::{Flight, GuidanceConfig, Phase, Shared};
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
            let ctl = if hold {
                // pad-only plumbing check: never light the engine, just log what the guidance sees
                fl.st = st;
                ksp_core::sim::Control::default()
            } else {
                fl.step_external(st, thrust_est)
            };
            let brakes = matches!(fl.phase, Phase::Coast | Phase::Descent | Phase::LandingBurn | Phase::Touchdown | Phase::Done);
            let done = fl.done() || (hold && tick >= 1000);
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
