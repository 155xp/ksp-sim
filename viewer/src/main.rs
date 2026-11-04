//! ksp-viewer: serves the web viewer and flies one guided RTLS flight per WebSocket connection
//! in the real sim (ksp-core + ksp-guidance), streaming the state after every physics tick.
//!
//! `ksp-viewer [--port 8787] [--static DIR]`

mod session;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use axum::routing::get;
use axum::Router;
use std::net::SocketAddr;
use tower_http::services::ServeDir;

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let port: u16 = arg("--port").and_then(|p| p.parse().ok()).unwrap_or(8787);
    let dir = arg("--static").unwrap_or_else(|| concat!(env!("CARGO_MANIFEST_DIR"), "/static").to_string());

    let app = Router::new().route("/ws", get(ws)).fallback_service(ServeDir::new(dir));
    let addr = SocketAddr::from(([0, 0, 0, 0], port));
    let listener = tokio::net::TcpListener::bind(addr).await.expect("bind");
    eprintln!("ksp-viewer on http://{addr}");
    axum::serve(listener, app).await.unwrap();
}

async fn ws(up: WebSocketUpgrade) -> Response {
    up.on_upgrade(handle)
}

async fn handle(mut sock: WebSocket) {
    let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<session::Cmd>();
    // bounded: when the socket is slower than the sim at max speed, the sim waits
    let (out_tx, mut out_rx) = tokio::sync::mpsc::channel::<session::Out>(64);
    let worker = std::thread::spawn(move || session::run(cmd_rx, out_tx));
    loop {
        tokio::select! {
            m = sock.recv() => match m {
                Some(Ok(Message::Text(t))) => match serde_json::from_str::<session::Cmd>(&t) {
                    Ok(c) => { let _ = cmd_tx.send(c); }
                    Err(e) => {
                        let msg = serde_json::json!({"type": "error", "message": e.to_string()}).to_string();
                        if sock.send(Message::Text(msg.into())).await.is_err() { break; }
                    }
                },
                Some(Ok(_)) => {}
                _ => break,
            },
            o = out_rx.recv() => match o {
                Some(session::Out::Text(t)) => if sock.send(Message::Text(t.into())).await.is_err() { break },
                Some(session::Out::Ticks(b)) => if sock.send(Message::Binary(b.into())).await.is_err() { break },
                None => break,
            },
        }
    }
    drop(cmd_tx);
    drop(out_rx);
    let _ = worker.join();
}
