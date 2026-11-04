import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

// Row layout streamed by the server (src/session.rs STRIDE): one row per 0.02 s physics tick.
const C = { t: 0, e: 1, n: 2, u: 3, ve: 4, vn: 5, vu: 6, ax: 7, bx: 10, thr: 13, phase: 14, alt: 15,
  prop: 16, mass: 17, pe: 18, pn: 19, ge: 20, gn: 21, vz: 22, vh: 23 };
const STRIDE = 24;
const MAX_ROWS = 2500 / 0.02 + 10;

const PHASES = [
  { name: "Ascent", h3: "#f1f0ef", h11: "#63635e", line: "#8d8d86" },
  { name: "Flip", h3: "#fff7c2", h11: "#ab6400", line: "#ffc53d" },
  { name: "Boostback", h3: "#ffefd6", h11: "#cc4e00", line: "#f76b15" },
  { name: "Coast", h3: "#e7f9fb", h11: "#107d98", line: "#00a2c7" },
  { name: "Entry", h3: "#edf2fe", h11: "#3a5bc7", line: "#3e63dd" },
  { name: "Landing burn", h3: "#fbebfb", h11: "#953ea3", line: "#ab4aba" },
  { name: "Touchdown", h3: "#f1f0ef", h11: "#63635e", line: "#63635e" },
  { name: "Done", h3: "#f1f0ef", h11: "#63635e", line: "#63635e" },
];
const OUTCOMES = {
  landed: ["Landed", "#e9f6e9", "#218358"], crashed: ["Crashed", "#feebec", "#ce2c31"],
  tipped_over: ["Tipped over", "#feebec", "#ce2c31"], timeout: ["Timed out", "#fff7c2", "#ab6400"],
  no_return: ["No return", "#feebec", "#ce2c31"],
};
const SPEEDS = [0.25, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, -1];

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------------------------
const rows = new Float32Array(MAX_ROWS * STRIDE);
let n = 0;               // rows received
let init = null;         // init message (design, geometry)
let result = null;       // done message
let mode = "idle";       // idle | live | paused | replay | finished
let tDisp = 0;           // flight time shown
let speedIdx = 5;
let achieved = null;
let camMode = "follow";
let ws = null;

const sel = { size: "0", start: "random", landing: "baseline", camera: "follow" };

// ---------------------------------------------------------------------------------------------
// three.js scene, pad ENU (east, north, up) mapped to three (x = e, y = u, z = -n)
// ---------------------------------------------------------------------------------------------
const enu = (e, nn, u, v = new THREE.Vector3()) => v.set(e, u, -nn);

const canvas = $("scene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
const sky = new THREE.Color();
const camera = new THREE.PerspectiveCamera(45, 1, 0.3, 6e6);
camera.position.set(-60, 40, 60);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.minDistance = 4;
controls.maxDistance = 3e6;
controls.enabled = false;

scene.add(new THREE.HemisphereLight(0xf4f6f8, 0x8a8f80, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.position.set(-0.4, 1, 0.5);
scene.add(sun);

let groundR = 600072.45;

// Ground: a polar mesh around the pad on the true ground sphere (the sim's pad-deck sphere),
// exact at every distance because height depends only on arc distance from the pad.
function buildGround(R) {
  const rings = [0];
  for (let s = 2; s < 0.995 * Math.PI * R; s *= 1.06) rings.push(s);
  const seg = 192;
  const pos = [], en = [], idx = [];
  for (let i = 0; i < rings.length; i++) {
    const th = rings[i] / R;
    for (let j = 0; j <= seg; j++) {
      const ph = (j / seg) * Math.PI * 2;
      const ce = Math.cos(ph), cn = Math.sin(ph);
      const r = R * Math.sin(th), up = R * Math.cos(th) - R;
      pos.push(r * ce, up, -r * cn);
      en.push(rings[i] * ce, rings[i] * cn);
    }
  }
  for (let i = 0; i < rings.length - 1; i++) {
    for (let j = 0; j < seg; j++) {
      const a = i * (seg + 1) + j, b = a + seg + 1;
      idx.push(a, b, a + 1, a + 1, b, b + 1);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute("en", new THREE.Float32BufferAttribute(en, 2));
  g.setIndex(idx);
  const m = new THREE.ShaderMaterial({
    vertexShader: `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      attribute vec2 en;
      varying vec2 vEn;
      varying float vDist;
      void main() {
        vEn = en;
        vec4 mv = modelViewMatrix * vec4(position, 1.0);
        vDist = length(mv.xyz);
        gl_Position = projectionMatrix * mv;
        #include <logdepthbuf_vertex>
      }`,
    fragmentShader: `
      #include <logdepthbuf_pars_fragment>
      varying vec2 vEn;
      varying float vDist;
      float ring(float d, float r) {
        float w = fwidth(d);
        return 1.0 - smoothstep(0.5 * w, 1.5 * w, abs(d - r));
      }
      void main() {
        #include <logdepthbuf_fragment>
        float e = vEn.x, nn = vEn.y;
        // scenery: the shore east of KSC (the sim's ground is one sphere at pad-deck height)
        float coast = 2200.0 + 700.0 * sin(nn / 4100.0) + 260.0 * sin(nn / 1300.0 + 1.3) + 9000.0 * smoothstep(-30000.0, -90000.0, nn);
        vec3 land = vec3(0.70, 0.74, 0.62);
        vec3 sea = vec3(0.62, 0.72, 0.78);
        float sh = smoothstep(-40.0, 40.0, e - coast);
        vec3 col = mix(land, sea, sh);
        float d = length(vEn);
        // pad: a 30 m deck with a cross
        float box = max(abs(e), abs(nn));
        col = mix(col, vec3(0.85, 0.84, 0.81), 1.0 - smoothstep(14.0, 15.0, box));
        float cross = (1.0 - smoothstep(0.25, 0.5, min(abs(e), abs(nn)))) * (1.0 - smoothstep(9.0, 9.5, box));
        col = mix(col, vec3(0.45, 0.44, 0.42), cross);
        // range rings: 100 m, 1 km, 10 km, 100 km
        float rr = max(max(ring(d, 100.0), ring(d, 1000.0)), max(ring(d, 10000.0), ring(d, 100000.0)));
        col = mix(col, vec3(0.30, 0.31, 0.28), 0.22 * rr);
        // haze with distance from the camera
        float haze = 1.0 - exp(-vDist / 160000.0);
        col = mix(col, vec3(0.86, 0.89, 0.92), 0.85 * haze);
        gl_FragColor = vec4(col, 1.0);
      }`,
  });
  const mesh = new THREE.Mesh(g, m);
  mesh.frustumCulled = false;
  return mesh;
}
let ground = buildGround(groundR);
scene.add(ground);

// screen-size markers (visible at any zoom)
function dotTexture(kind) {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  g.lineWidth = 7;
  if (kind === "ring") { g.strokeStyle = "#953ea3"; g.beginPath(); g.arc(32, 32, 24, 0, 7); g.stroke(); }
  if (kind === "pad") { g.strokeStyle = "#21201c"; g.lineWidth = 5; g.strokeRect(10, 10, 44, 44); }
  if (kind === "dot") { g.fillStyle = "#21201c"; g.beginPath(); g.arc(32, 32, 14, 0, 7); g.fill(); g.strokeStyle = "#fff"; g.lineWidth = 6; g.stroke(); }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
function marker(kind, px) {
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: dotTexture(kind), depthTest: false, sizeAttenuation: false, transparent: true }));
  s.scale.setScalar(px);
  s.renderOrder = 10;
  scene.add(s);
  return s;
}
const padMark = marker("pad", 0.022);
padMark.position.set(0, 0.5, 0);
const pipMark = marker("ring", 0.03);
pipMark.visible = false;
const boosterMark = marker("dot", 0.016);
boosterMark.visible = false;

// trail, one vertex per tick, coloured by phase
const trailPos = new Float32Array(MAX_ROWS * 3);
const trailCol = new Float32Array(MAX_ROWS * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
trailGeo.setAttribute("color", new THREE.BufferAttribute(trailCol, 3));
trailGeo.setDrawRange(0, 0);
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true }));
trail.frustumCulled = false;
scene.add(trail);
const phaseRGB = PHASES.map((p) => new THREE.Color(p.line));

// booster model, built from the server's part layout (root frame, relative to the CoM)
let booster = null, upper = [], flames = [];
const matTank = new THREE.MeshStandardMaterial({ color: 0xf2f1ee, roughness: 0.75 });
const matDark = new THREE.MeshStandardMaterial({ color: 0x3b3a37, roughness: 0.6, metalness: 0.2 });
const matBand = new THREE.MeshStandardMaterial({ color: 0x63635e, roughness: 0.7 });
const matFlame = new THREE.MeshBasicMaterial({ color: 0xffb547, transparent: true, opacity: 0.85, depthWrite: false });
const matCore = new THREE.MeshBasicMaterial({ color: 0xfff4dc, transparent: true, opacity: 0.95, depthWrite: false });

function buildBooster(geom) {
  if (booster) scene.remove(booster);
  booster = new THREE.Group();
  upper = []; flames = [];
  for (const p of geom.parts) {
    const r = p.s[0] / 2, h = p.s[1];
    let mesh;
    if (p.kind === "nose") {
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.08, r, h, 32), matTank);
    } else {
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 32), p.kind === "part" ? matBand : matTank);
    }
    mesh.position.set(p.c[0], p.c[1], p.c[2]);
    booster.add(mesh);
    if (p.kind === "upper" || p.kind === "nose") upper.push(mesh);
  }
  for (const e of geom.engines) {
    const w = e.s[0] * 0.5, len = Math.max(0.3, e.top - e.p[1]);
    const bell = new THREE.Mesh(new THREE.CylinderGeometry(w * 0.45, w * 0.9, len, 24), matDark);
    bell.position.set(e.p[0], e.p[1] + len / 2, e.p[2]);
    booster.add(bell);
    const fl = new THREE.Group();
    const outer = new THREE.Mesh(new THREE.ConeGeometry(w * 0.85, 1, 24, 1, true), matFlame);
    const core = new THREE.Mesh(new THREE.ConeGeometry(w * 0.45, 0.6, 20, 1, true), matCore);
    outer.rotation.x = Math.PI; core.rotation.x = Math.PI;
    outer.position.y = -0.5; core.position.y = -0.3;
    fl.add(outer, core);
    fl.position.set(e.p[0], e.p[1], e.p[2]);
    fl.userData.len = 7 * w;
    booster.add(fl);
    flames.push(fl);
  }
  for (const l of geom.legs) {
    const a = new THREE.Vector3(...l.attach), f = new THREE.Vector3(...l.foot);
    const d = f.clone().sub(a);
    const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, d.length(), 8), matDark);
    strut.position.copy(a).addScaledVector(d, 0.5);
    strut.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
    booster.add(strut);
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.06, 12), matDark);
    pad.position.copy(f);
    booster.add(pad);
  }
  scene.add(booster);
}

function boosterLength() {
  if (!init) return 15;
  const ys = init.geom.parts.filter((p) => p.kind !== "upper" && p.kind !== "nose").map((p) => p.c[1] + p.s[1] / 2);
  const lo = Math.min(...init.geom.legs.map((l) => l.foot[1]), ...init.geom.engines.map((e) => e.p[1]));
  return Math.max(...ys) - lo;
}

// ---------------------------------------------------------------------------------------------
// samples
// ---------------------------------------------------------------------------------------------
const R = (i, k) => rows[i * STRIDE + k];
const tEnd = () => (n ? R(n - 1, C.t) : 0);

function indexAt(t) {
  let lo = 0, hi = n - 1;
  if (hi < 0) return -1;
  if (t <= R(0, C.t)) return 0;
  if (t >= R(hi, C.t)) return hi;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (R(mid, C.t) <= t) lo = mid; else hi = mid;
  }
  return lo;
}

/** Linear interpolation between ticks i and i+1 (continuous values only). */
function sampleAt(t) {
  const i = indexAt(t);
  const j = Math.min(i + 1, n - 1);
  const t0 = R(i, C.t), t1 = R(j, C.t);
  const a = j > i && t1 > t0 ? Math.min(1, Math.max(0, (t - t0) / (t1 - t0))) : 0;
  const s = new Float32Array(STRIDE);
  for (let k = 0; k < STRIDE; k++) s[k] = R(i, k) + (R(j, k) - R(i, k)) * a;
  for (const k of [C.phase, C.pe, C.pn]) s[k] = R(i, k);
  return { s, i };
}

function appendRows(buf) {
  const f = new Float32Array(buf);
  const m = Math.min(f.length / STRIDE, MAX_ROWS - n);
  rows.set(f.subarray(0, m * STRIDE), n * STRIDE);
  const v = new THREE.Vector3();
  for (let i = n; i < n + m; i++) {
    enu(R(i, C.e), R(i, C.n), R(i, C.u), v);
    trailPos.set([v.x, v.y, v.z], i * 3);
    const c = phaseRGB[R(i, C.phase) | 0] || phaseRGB[0];
    trailCol.set([c.r, c.g, c.b], i * 3);
  }
  trailGeo.attributes.position.addUpdateRange(n * 3, m * 3);
  trailGeo.attributes.color.addUpdateRange(n * 3, m * 3);
  trailGeo.attributes.position.needsUpdate = true;
  trailGeo.attributes.color.needsUpdate = true;
  n += m;
}

// ---------------------------------------------------------------------------------------------
// server
// ---------------------------------------------------------------------------------------------
function connect() {
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.binaryType = "arraybuffer";
  ws.onmessage = (ev) => {
    if (typeof ev.data !== "string") { appendRows(ev.data); return; }
    const m = JSON.parse(ev.data);
    if (m.type === "init") onInit(m);
    else if (m.type === "done") result = m;
    else if (m.type === "rate") achieved = m.x;
    else if (m.type === "error") { $("design").textContent = m.message; setMode("idle"); }
  };
  ws.onclose = () => setTimeout(connect, 1000);
}
const send = (o) => ws && ws.readyState === 1 && ws.send(JSON.stringify(o));
connect();

function clearFlight() {
  n = 0; result = null; tDisp = 0; achieved = null;
  trailGeo.setDrawRange(0, 0);
  pipMark.visible = false; boosterMark.visible = false;
  if (booster) { scene.remove(booster); booster = null; }
  $("result").hidden = true;
}

function onInit(m) {
  clearFlight();
  init = m;
  if (Math.abs(m.ground_r - groundR) > 0.01) {
    groundR = m.ground_r;
    scene.remove(ground); ground = buildGround(groundR); scene.add(ground);
  }
  buildBooster(m.geom);
  const d = m.design;
  $("design").hidden = false;
  $("design").textContent = `${d.label} · ${d.diameter} m · ${d.booster_wet_t.toFixed(1)} t wet · seed ${m.seed}`;
  camOffsetDir = null;
  setMode("live");
}

function start() {
  const num = (id) => parseFloat($(id).value);
  const startSpec = sel.start === "set"
    ? { kind: "set", alt: num("sAlt") * 1000, downrange: num("sDown") * 1000, vh: num("sVh"), vz: num("sVz"), fuel: num("sFuel") / 100 }
    : { kind: "random" };
  clearFlight();
  send({ cmd: "start", seed: Math.max(0, parseInt($("seed").value, 10) || 0), size: parseInt(sel.size, 10), landing: sel.landing, start: startSpec, speed: SPEEDS[speedIdx] });
  $("design").hidden = false;
  $("design").textContent = "Building booster…";
}

// ---------------------------------------------------------------------------------------------
// controls
// ---------------------------------------------------------------------------------------------
for (const seg of document.querySelectorAll(".seg")) {
  seg.addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    for (const x of seg.querySelectorAll("button")) x.setAttribute("aria-pressed", String(x === b));
    sel[seg.dataset.name] = b.dataset.v;
    if (seg.dataset.name === "start") $("setFields").hidden = b.dataset.v !== "set";
    if (seg.dataset.name === "camera") setCamera(b.dataset.v);
  });
}

$("shuffle").onclick = () => { $("seed").value = Math.floor(Math.random() * 100000); };
$("run").onclick = start;

$("play").onclick = () => {
  if (mode === "live") { send({ cmd: "pause" }); setMode("paused"); }
  else if (mode === "paused") { send({ cmd: "resume" }); setMode("live"); }
  else if (mode === "replay") setMode("finished");
  else if (mode === "finished") { if (tDisp >= tEnd() - 1e-3) tDisp = 0; setMode("replay"); }
};
$("reset").onclick = () => { send({ cmd: "reset" }); clearFlight(); init = null; setMode("idle"); $("design").hidden = true; resetReadouts(); };

const scrub = $("scrub");
scrub.addEventListener("input", () => {
  tDisp = parseFloat(scrub.value);
  if (mode === "replay") setMode("finished");
});

const speed = $("speed");
function showSpeed() {
  const s = SPEEDS[speedIdx];
  let txt = s < 0 ? "Max" : `${s}×`;
  const running = mode === "live";
  if (running && achieved != null && (s < 0 || achieved < 0.9 * s)) txt += ` · ${achieved >= 10 ? Math.round(achieved) : achieved.toFixed(1)}× now`;
  $("speedVal").textContent = txt;
  speed.style.setProperty("--fill", `${(speedIdx / (SPEEDS.length - 1)) * 100}%`);
}
speed.addEventListener("input", () => {
  speedIdx = parseInt(speed.value, 10);
  achieved = null;
  send({ cmd: "speed", value: SPEEDS[speedIdx] });
  showSpeed();
});

function setMode(m) {
  mode = m;
  const flying = m === "live" || m === "paused";
  $("setup").classList.toggle("locked", flying);
  $("run").disabled = flying;
  $("play").disabled = m === "idle";
  const showPlay = m === "paused" || m === "finished" || m === "idle";
  $("play").classList.toggle("paused", showPlay);
  $("play").setAttribute("aria-label", showPlay ? "Play" : "Pause");
  $("reset").disabled = m === "idle";
  scrub.disabled = !(m === "paused" || m === "finished" || m === "replay");
}

let camOffsetDir = null;
function setCamera(v) {
  camMode = v;
  controls.enabled = v === "orbit";
}

// ---------------------------------------------------------------------------------------------
// readouts
// ---------------------------------------------------------------------------------------------
const fmtDist = (m) => (Math.abs(m) < 1000 ? `${m < 10 ? m.toFixed(2) : m < 100 ? m.toFixed(1) : Math.round(m)} m` : `${(m / 1000).toFixed(m < 1e4 ? 2 : 1)} km`);
const fmtSpeed = (v) => `${(Math.abs(v) < 0.05 ? 0 : v).toFixed(Math.abs(v) < 100 ? 1 : 0)} m/s`;
const fmtClock = (t) => { const m = Math.floor(t / 60); return `T+ ${m}:${(t - m * 60).toFixed(1).padStart(4, "0")}`; };

function setBadge(el, label, h3, h11) {
  el.textContent = label;
  el.classList.add("hued");
  el.style.setProperty("--h3", h3);
  el.style.setProperty("--h11", h11);
}

function updateReadouts(s) {
  const ph = PHASES[s[C.phase] | 0] || PHASES[0];
  setBadge($("tPhase"), ph.name, ph.h3, ph.h11);
  $("tAlt").textContent = fmtDist(Math.max(0, s[C.alt]));
  $("tVz").textContent = fmtSpeed(s[C.vz]);
  $("tVh").textContent = fmtSpeed(s[C.vh]);
  $("tThr").textContent = `${Math.round(s[C.thr] * 100)}%`;
  const cap = init ? init.design.prop_cap_t : 1;
  $("tFuel").innerHTML = `${Math.round((s[C.prop] / cap) * 100)}%<small>${s[C.prop].toFixed(2)} t</small>`;
  $("tMass").textContent = `${s[C.mass].toFixed(2)} t`;
  $("tPad").textContent = fmtDist(Math.hypot(s[C.ge], s[C.gn]));
  $("clock").textContent = fmtClock(s[C.t]);
}

function resetReadouts() {
  const ph = $("tPhase");
  ph.textContent = "–"; ph.classList.remove("hued");
  for (const id of ["tAlt", "tVz", "tVh", "tThr", "tFuel", "tMass", "tPad"]) $(id).textContent = "–";
  $("clock").textContent = fmtClock(0);
}

function showResult() {
  const r = result;
  const [label, h3, h11] = OUTCOMES[r.outcome] || [r.outcome, "#f1f0ef", "#63635e"];
  setBadge($("outcome"), label, h3, h11);
  $("rMiss").textContent = fmtDist(r.miss);
  $("rSpeed").textContent = `${r.td_speed.toFixed(r.td_speed < 10 ? 2 : 1)} m/s`;
  $("result").hidden = false;
}

// pad inset: top-down, pad at the centre, scale picked to fit the booster and the prediction
const inset = $("inset");
const ictx = inset.getContext("2d");
const IW = 264;
inset.width = inset.height = IW * Math.min(devicePixelRatio, 2);
let insetRange = 50;
const nice = (x) => { const p = 10 ** Math.floor(Math.log10(x)); const f = x / p; return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p; };

function drawInset(s, iNow) {
  const g = ictx, k = inset.width / IW;
  g.setTransform(k, 0, 0, k, 0, 0);
  g.clearRect(0, 0, IW, IW);
  const c = IW / 2;
  let need = 30;
  if (s) {
    need = Math.max(need, Math.hypot(s[C.ge], s[C.gn]));
    if (Number.isFinite(s[C.pe])) need = Math.max(need, Math.hypot(s[C.pe], s[C.pn]));
  }
  const target = nice(need * 1.15);
  insetRange = target;
  const px = (c - 14) / insetRange; // px per metre
  const P = (e, nn) => [c + e * px, c - nn * px];
  // rings at a quarter and half of the range
  g.strokeStyle = "#dad9d6"; g.lineWidth = 1;
  for (const f of [0.5, 1]) { g.beginPath(); g.arc(c, c, f * insetRange * px, 0, 7); g.stroke(); }
  // pad deck (30 m)
  const ps = Math.max(3, 15 * px);
  g.fillStyle = "#cfceca"; g.fillRect(c - ps, c - ps, ps * 2, ps * 2);
  g.strokeStyle = "#63635e"; g.lineWidth = 1;
  g.beginPath(); g.moveTo(c - ps * 0.6, c); g.lineTo(c + ps * 0.6, c); g.moveTo(c, c - ps * 0.6); g.lineTo(c, c + ps * 0.6); g.stroke();
  if (s) {
    // ground track
    g.lineWidth = 1.5;
    const step = Math.max(1, Math.floor(iNow / 1500));
    let prev = -1;
    for (let i = 0; i <= iNow; i += step) {
      const ph = R(i, C.phase) | 0;
      if (ph !== prev) { if (prev >= 0) g.stroke(); g.strokeStyle = PHASES[ph].line; g.beginPath(); const [x, y] = P(R(i, C.ge), R(i, C.gn)); g.moveTo(x, y); prev = ph; }
      else { const [x, y] = P(R(i, C.ge), R(i, C.gn)); g.lineTo(x, y); }
    }
    const [bx, by] = P(s[C.ge], s[C.gn]);
    g.lineTo(bx, by); g.stroke();
    if (Number.isFinite(s[C.pe])) {
      const [x, y] = P(s[C.pe], s[C.pn]);
      g.strokeStyle = "#953ea3"; g.lineWidth = 2; g.beginPath(); g.arc(x, y, 6, 0, 7); g.stroke();
    }
    g.fillStyle = "#21201c"; g.strokeStyle = "#fff"; g.lineWidth = 2;
    g.beginPath(); g.arc(bx, by, 4.5, 0, 7); g.fill(); g.stroke();
  }
  // scale bar
  const bar = nice(insetRange / 2.5) / (nice(insetRange / 2.5) > insetRange / 2 ? 2 : 1);
  const bw = bar * px;
  g.strokeStyle = "#21201c"; g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(12, IW - 14); g.lineTo(12 + bw, IW - 14); g.moveTo(12, IW - 18); g.lineTo(12, IW - 10); g.moveTo(12 + bw, IW - 18); g.lineTo(12 + bw, IW - 10); g.stroke();
  g.fillStyle = "#63635e"; g.font = '500 12px "Inter", system-ui'; g.textBaseline = "bottom";
  g.fillText(bar >= 1000 ? `${bar / 1000} km` : `${bar} m`, 18 + bw, IW - 7);
  g.textBaseline = "top"; g.textAlign = "right"; g.fillText("N", IW - 10, 8); g.textAlign = "left";
}

function updateMiss(s, atEnd) {
  if (atEnd && result) {
    $("missLabel").textContent = "Miss"; $("missVal").textContent = fmtDist(result.miss); return;
  }
  if (s && Number.isFinite(s[C.pe])) {
    $("missLabel").textContent = "Predicted miss"; $("missVal").textContent = fmtDist(Math.hypot(s[C.pe], s[C.pn]));
  } else if (s) {
    $("missLabel").textContent = "To pad"; $("missVal").textContent = fmtDist(Math.hypot(s[C.ge], s[C.gn]));
  } else { $("missLabel").textContent = "Predicted miss"; $("missVal").textContent = "–"; }
}

// ---------------------------------------------------------------------------------------------
// frame loop
// ---------------------------------------------------------------------------------------------
function resize() {
  const w = innerWidth, h = innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener("resize", resize);
resize();

/** On-screen size of one metre at world point p. */
function pxPerMetre(p) {
  const d = Math.max(1e-3, camera.position.distanceTo(p));
  return innerHeight / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) * d);
}

const tmpA = new THREE.Vector3(), tmpB = new THREE.Vector3(), tmpC = new THREE.Vector3();
const basis = new THREE.Matrix4();
const pos = new THREE.Vector3();
const lastTarget = new THREE.Vector3();
let last = performance.now();
let lastScrubSet = 0;

function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const sp = SPEEDS[speedIdx];
  if (n > 0) {
    if (mode === "live") tDisp = sp < 0 ? tEnd() : Math.min(tEnd(), tDisp + dt * sp);
    else if (mode === "replay") {
      tDisp = sp < 0 ? tEnd() : Math.min(tEnd(), tDisp + dt * sp);
      if (tDisp >= tEnd()) setMode("finished");
    }
    // the server finished and the display caught up
    if (mode === "live" && result && tDisp >= tEnd() - 1e-6) setMode("finished");
  }

  let s = null, iNow = 0;
  if (n > 0) {
    ({ s, i: iNow } = sampleAt(tDisp));
    updateReadouts(s);
    enu(s[C.e], s[C.n], s[C.u], pos);
    if (booster) {
      booster.visible = true;
      const ax = enu(s[C.ax], s[C.ax + 1], s[C.ax + 2], tmpA).normalize();
      const bx = enu(s[C.bx], s[C.bx + 1], s[C.bx + 2], tmpB);
      bx.addScaledVector(ax, -bx.dot(ax)).normalize();
      const bz = tmpC.crossVectors(bx, ax).normalize();
      basis.makeBasis(bx, ax, bz);
      booster.quaternion.setFromRotationMatrix(basis);
      booster.position.copy(pos);
      const thr = s[C.thr];
      for (const f of flames) { f.visible = thr > 0.005; f.scale.set(1, f.userData.len * (0.25 + thr), 1); }
      const ascent = (s[C.phase] | 0) === 0;
      for (const u of upper) u.visible = ascent;
    }
    // dot only when the booster itself is too small to see
    boosterMark.position.copy(pos);
    boosterMark.visible = pxPerMetre(pos) * boosterLength() < 10;
    pipMark.visible = Number.isFinite(s[C.pe]);
    if (pipMark.visible) {
      const d = Math.hypot(s[C.pe], s[C.pn]), th = d / groundR;
      const ce = d > 0 ? s[C.pe] / d : 0, cn = d > 0 ? s[C.pn] / d : 0;
      enu(groundR * Math.sin(th) * ce, groundR * Math.sin(th) * cn, groundR * Math.cos(th) - groundR + 0.3, pipMark.position);
    }
    trailGeo.setDrawRange(0, iNow + 1);
    if (now - lastScrubSet > 100 || mode !== "live") {
      scrub.max = String(Math.max(tEnd(), 0.02));
      if (document.activeElement !== scrub || mode !== "finished") scrub.value = String(tDisp);
      scrub.style.setProperty("--fill", `${(tDisp / Math.max(tEnd(), 0.02)) * 100}%`);
      lastScrubSet = now;
    }
  }
  padMark.visible = pxPerMetre(padMark.position) * 30 < 10;
  const atEnd = !!result && n > 0 && tDisp >= tEnd() - 1e-6;
  if (atEnd && $("result").hidden) showResult();
  if (!atEnd && !$("result").hidden) $("result").hidden = true;
  drawInset(s, iNow);
  updateMiss(s, atEnd);
  showSpeed();

  // camera
  const L = boosterLength();
  if (n > 0) {
    if (camMode === "follow") {
      // behind the booster as seen from the pad, raised: the pad stays in the background
      tmpA.set(s[C.ge], 0, -s[C.gn]);
      const far = tmpA.length() > 40;
      if (far) tmpA.normalize(); else tmpA.copy(camOffsetDir || new THREE.Vector3(-0.7, 0, 0.7));
      if (!camOffsetDir) camOffsetDir = tmpA.clone();
      camOffsetDir.lerp(tmpA, Math.min(1, dt * 1.5)).normalize();
      const D = 2.6 * L + 20;
      camera.position.copy(pos).addScaledVector(camOffsetDir, D).add(tmpB.set(0, 0.24 * D, 0));
      camera.lookAt(pos);
      controls.target.copy(pos);
      lastTarget.copy(pos);
    } else {
      // orbit: the target rides with the booster, the user's offset is kept
      tmpA.copy(pos).sub(lastTarget);
      camera.position.add(tmpA);
      controls.target.copy(pos);
      lastTarget.copy(pos);
      controls.update();
    }
  } else if (camMode === "orbit") controls.update();
  else { camera.position.set(-70, 35, 70); camera.lookAt(0, 8, 0); }

  // sky darkens with the camera's altitude (atmosphere top 70 km)
  const camAlt = Math.max(0, Math.hypot(camera.position.x, camera.position.y + groundR, camera.position.z) - groundR);
  const k = Math.min(1, camAlt / 70000) ** 0.6;
  sky.setRGB(0.874 + (0.035 - 0.874) * k, 0.902 + (0.05 - 0.902) * k, 0.925 + (0.11 - 0.925) * k, THREE.SRGBColorSpace);
  renderer.setClearColor(sky);

  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
setMode("idle");
speed.value = String(speedIdx);
showSpeed();
requestAnimationFrame(frame);

// for the end-to-end test
window.__viewer = { get mode() { return mode; }, get result() { return result; }, get rows() { return n; }, get tDisp() { return tDisp; }, get init() { return init; } };
