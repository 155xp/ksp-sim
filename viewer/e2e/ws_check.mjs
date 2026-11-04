// Fly one flight over the WebSocket at max speed and print the result (protocol smoke test).
// node ws_check.mjs [seed] [size] [port]
import WebSocket from "ws";
const [seed = "7", size = "0", port = "8787"] = process.argv.slice(2);
const ws = new WebSocket(`ws://127.0.0.1:${port}/ws`);
let rows = 0, stride = 24, t0 = Date.now(), last = null;
ws.on("open", () => ws.send(JSON.stringify({ cmd: "start", seed: +seed, size: +size, landing: "baseline", start: { kind: "random" }, speed: -1 })));
ws.on("message", (d, bin) => {
  if (bin) { const f = new Float32Array(Uint8Array.from(d).buffer); rows += f.length / stride; last = f.slice(f.length - stride); return; }
  const m = JSON.parse(d.toString());
  if (m.type === "init") { stride = m.stride; console.log("init", JSON.stringify(m.design), JSON.stringify(m.geom).slice(0,1500)); }
  if (m.type === "done") { console.log("done", JSON.stringify(m)); console.log("rows", rows, "wall", (Date.now() - t0) / 1000, "last", Array.from(last).map(x => +x.toFixed(2)).join(" ")); ws.close(); }
  if (m.type === "error") { console.log(m); ws.close(); }
});
