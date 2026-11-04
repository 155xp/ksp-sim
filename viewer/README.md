# viewer/ — web viewer for ksp-sim + guidance

A small Rust server that flies one guided return-to-launch-site flight per browser connection in
the real sim, and a three.js page that draws it. Every 0.02 s physics tick is streamed over a
WebSocket (packed f32 rows), so the page only ever shows sim output.

The sim and the guidance are linked read-only from this repo, pinned to `guidance` commit
`81b348f` (`Cargo.toml`); nothing outside `viewer/` changes.

```
src/main.rs      axum: static files + /ws
src/session.rs   one Flight per connection, real-time pacing / max speed, row layout, results
static/          index.html, style.css, app.js (three.js 0.170 and Inter vendored)
e2e/             landing.mjs (Playwright: flies a landing through the UI, screenshots),
                 ws_check.mjs (protocol smoke test)
docs/            screenshots from the e2e run
```

## Run

Needs the KSP 1.12.5 install for part data (`$KSP_DIR`, default the Steam/Debian path).

```bash
cd viewer
cargo build --release
./run.sh 8787            # nohup, nice 10, 0.0.0.0:8787, log in viewer.log, pid in viewer.pid
```

## What it flies

* **Random**: exactly the Monte Carlo flight for the seed (`make_fleet(seed, 1, 1, size)`, same as
  `Fleet(seed=…, n_designs=1, per_design=1, size=…)` and `examples/fly`): ascent from the pad,
  MECO, separation, flip, boostback, coast, entry, landing burn.
* **Set**: the seed's booster design, starting at separation from the given altitude above the
  pad, downrange east, horizontal / vertical surface speed and propellant fraction, pointing
  prograde.

The predicted touchdown is the guidance's own predictor (landing-burn model), re-solved every
0.5 s of flight. Miss and touchdown speed at the end are the flight's `Stats` (`final_e/final_n`,
`td_vz/td_vh`). The coastline is scenery: the sim's ground is one sphere at pad-deck height.

## Check

```bash
cd viewer/e2e && npm install
node landing.mjs http://127.0.0.1:8787/ 42 ../docs 6    # seed 42 at 20x
```
