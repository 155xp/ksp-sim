# In-game validation: ksp-sim vs KSP 1.12.5, tick by tick

The reference is the running game. A copy of the install (`~/ksp-harness/KSP`; the user's install
and saves are never touched) runs kOS plus the `SimHarness` plugin (`harness/plugin`). kOS flies
scripted tests (`harness/kos/harness/*.ks`). In the last FixedUpdate stage of every physics tick,
the plugin logs a CSV row with:

- vessel CoM and root state, attitude and angular velocity;
- pilot input, engine throttle, thrust and Isp, gimbal actuation, reaction-wheel state;
- per-part drag and lift vectors;
- every WheelCollider hit and the hidden suspension state;
- Krakensbane, and the scenery placement (PQS offset, ground probe).

It also writes geometry dumps (part transforms, colliders, inertia, wheel colliders, thrust
transforms) at the start and end of each test. The traces live in the git-ignored `data/traces/`
and are regression tests: `crates/ksp-core/tests/traces.rs` covers the CPU, and
`tests/test_cuda_parity.py::test_full_tick_vs_game_traces` covers both GPU kernels.

Regenerate the table: `cargo run --release -p ksp-core --example error_table`.

## Method

* **One-step error:** every logged tick is seeded into the sim:
  - the CoM state and attitude;
  - fuel and throttle state, gimbal actuation, reaction-wheel lerp state;
  - the smoothed-velocity history;
  - the wheel colliders' contact depth and hidden ModuleWheelSuspension / VPP state;
  - the scenery placement.

  It is then stepped once, and the CoM acceleration and the angular acceleration are compared
  with the game's next row.
* **1-s windows:** seeded every 50 ticks and run open loop for 50 ticks with the logged pilot
  input. Shown is the maximum CoM position / attitude error over all windows of the airborne
  part.
* **Touchdown:** seeded 2 s before the first wheel contact (0.5 s for the fast lateral hops),
  then run open loop through the contact. Errors are taken at the game's touchdown tick and
  2 s / 5 s later.

## Error table (current code)

| test | trace | airborne s | one-step accel err median / p95 (m/s²) | ang. accel err median (rad/s²) | 1-s windows: pos max (m), att max (deg) | touchdown tick (game / sim) | pos err at touchdown / +2 s / +5 s (m) |
|---|---|---|---|---|---|---|---|
| ascent to 80 km | v2/Ascent80 | 256.5 | 6.64e-5 / 2.36e-3 | 3.18e-4 | 0.0077, 0.055 | - | - |
| hover 30 m, land | v2/HopperHover | 45.5 | 9.99e-4 / 1.15e-3 | 9.06e-4 | 0.0065, 0.194 | 2318 / 2318 | 0.0020 / 0.0004 / 0.0008 |
| drop onto pad (9.2 m/s) | v2/HopperDrop2 | 2.4 | 6.70e-4 / 7.17e-4 | 4.27e-4 | 0.0059, 0.141 | 167 / 167 | 0.0005 / 0.1823 / 0.2000 |
| suicide burn, 16 m/s nozzle strike | v2/HopperSuicide | 40.8 | 3.47e-3 / 8.30e-3 | 5.28e-4 | 0.0073, 0.067 | 2078 / 2078 | 0.0077 / 0.4243 / 0.4568 |
| suicide burn to pad | v3/HopperSuicide | 42.2 | 3.20e-3 / 8.22e-3 | 4.39e-4 | 0.0101, 0.092 | 2147 / 2147 | 0.0041 / 0.0398 / 0.0494 |
| suicide burn to pad (placement logged) | v7/HopperSuicide | 42.2 | 3.75e-3 / 8.26e-3 | 3.76e-4 | 0.0120, 0.116 | 2147 / 2147 | 0.0057 / 0.0223 / 0.0297 |
| hop + pad landing | v6/HopperPadLand | 27.3 | 2.15e-3 / 1.34e-2 | 6.08e-4 | 0.0245, 0.814 | 1405 / 1405 | 0.0021 / 0.0296 / 0.0318 |
| hop + pad landing (placement logged) | v7/HopperPadLand | 27.4 | 2.18e-3 / 1.33e-2 | 6.66e-4 | 0.0267, 0.835 | 1406 / 1406 | 0.0042 / 0.0367 / 0.0441 |
| suborbital hop (flight) | v2/HopperHop | 34.1 | 5.79e-3 / 1.01e-2 | 2.99e-4 | 0.0241, 0.364 | 1741 / 1741 | 0.0021 / 1.0952 / 1.4461 |
| hop, lands on sloped grass, tips over | v7/HopperHop | 35.3 | 3.34e-3 / 8.38e-3 | 5.59e-4 | 0.0220, 0.327 | 1800 / 1800 | 0.0002 / 1.0250 / 0.9417 |

Ascent to 80 km, translation only (the logged attitude imposed each tick, everything else free
for 257 s): CoM position error ≤ 18 m, velocity ≤ 0.096 m/s.

One-step error **in contact** (legs on the pad; median / p95, m/s²):

| trace | v2/Hover | v2/Suicide | v2/Drop2 | v3/Suicide | v6/PadLand | v7/PadLand | v7/Suicide |
|---|---|---|---|---|---|---|---|
| median | 0.025 | 0.030 | 0.105 | 0.018 | 0.053 | 0.026 | 0.035 |
| p95 | 0.10 | 0.48 | 2.5 | 0.27 | 1.3 | 0.55 | 0.29 |

The p95 tails are the impact ticks themselves; see "contact solver" below.

Component checks (one-step, from the per-part columns):
- drag and body-lift vectors match per part to 4–5 significant figures;
- PhysX wheel spring, damper and target match exactly;
- the ModuleWheelSuspension spring matches exactly in 99.6–99.96% of ticks;
- touchdown happens on the same tick as in the game in every trace;
- rebound speed after the drop: 3.53 vs 3.46 m/s on the pad, 5.2 vs 4.84 m/s on terrain.

GPU: both kernels reproduce the CPU's one-step errors against the game to 3 significant figures
on every trace above, median and p95, airborne and in contact.

## Scenery placement (found and modelled in this round)

KSP keeps body positions in double but builds the scenery's Unity transforms in float:
`FloatingOrigin.setOffset` moves `CelestialBody.position`, and the PQS and KSC transforms are then
re-derived at ~600 km coordinate magnitude, where a float ulp is 1.5–6 cm. The harness logs a
downward raycast at the pad centre against the scenery layer, the PQS transform minus the double
body position, and the floating-origin offset. Over 3 flights and 9 game sessions:

* **The whole scenery moves by the PQS rounding vector.** The probe equals the deck height plus
  the radial part of that vector to 3e-5 m.
* **When it moves:** the origin shifts every tick while the vessel's radar altitude exceeds
  ~150 m, and the scenery is re-rounded each time. Below ~150 m it stops, so a landing sees the
  placement left by its last shift.
* **Size:** over 2589 shifts the radial part has mean 0, σ 9 mm and range ±2.2 cm.
* **Per-session KSC offset:** the KSC colliders (pad, apron, buildings) carry an extra radial
  offset fixed per game session. It is 0 in 7 of 9 sessions, +3.7 cm in one and −1.3 cm in one.
  PQS terrain does not.
* **Before this was modelled,** the one-step contact error on traces that had flown above 150 m
  was a constant 2.4–2.8 m/s²: the legs saw the ground 5.7 mm (7 mm along the leg) away. With the
  logged placement seeded, the game's wheel hits lie on the modelled pad to 0.0000 m in every
  row, and that error is 0.02–0.05 m/s².

The sim models this as `EnvState.ground_shift` (a translation of all scenery) and
`EnvState.ksc_dz` (KSC colliders only); see `sim::Scenery`. Traces are seeded from the logged
values; v2–v6 traces are seeded from the wheel-hit heights instead. Training resets can draw the
offsets with `ground_shift_max` / `ksc_dz_max`, which default to 0 (surveyed placement). For a
lander meant to fly in the real game, ±0.022 m / ±0.04 m covers everything measured.

## What is still not 1:1, and the bound

| difference | game | sim | bound / measured effect |
|---|---|---|---|
| vessel = one rigid body | each part is a PhysX rigidbody on joints | composite rigid body with KSP's gyroscopic split (parallel-axis part only) and per-part angular damping | in flight, joint flex shows up as thrust-axis sag of 1e-6..2e-4 rad after spawn (random per flight, constant after; measured in 7 flights): ≤ 0.006°, ≤ 0.003 m/s² lateral. Not modelled; treat as noise. A trace whose end dump shows a deformed joint (tip-over) uses the start pose |
| contact solver | PhysX PGS on 6 part bodies + joints, speculative contacts, contactOffset 0.01 | sequential impulses on the composite (8 iterations), same contactOffset, depenetration fraction 0.55 (fitted to the drop traces) | soft pad landings (≤ 3 m/s): ≤ 5 mm at touchdown, ≤ 5 cm after 5 s. Hard impacts: 9.2 m/s drop 0.2 m after 5 s; 16 m/s nozzle strike 0.45 m (post-impact sliding / rotation; the part-level joint response isn't there) |
| crash rule | joint breakForce (engine / tank 4400, legs 4000) and part crashTolerance per PhysX contact | same thresholds on the composite's contact impulse / post-solve speed | not yet validated with an in-game crash series. Every validation landing was below the thresholds in both |
| yaw/pitch response during fast gimbal ramps | gimbal transform hierarchy, joint compliance | rigid gimbal about the measured pivot (torque coefficient 0.867 measured) | ~7% of the commanded response during ramps; 1-s windows ≤ 0.84° in the gimbal-heavy pad landing, ≤ 0.12° in the suicide burns |
| terrain away from the pad | PQS quads (LOD mesh) + KSC colliders | in-game raycast survey: 0.2 m grid within ±40 m of the pad, 4 m grid within ±1.5 km, bilinear | pad: exact (hits on the modelled ground to 0.0000 m). Grass 310 m out: −8.4..−9.0 mm residual (4 m interpolation of the mesh, or LOD-dependent quad vertices). A lateral landing on that slope tips over in both, but diverges by ~1 m within 2 s |
| scenery placement | float-rounded per origin shift + per session | modelled (above); reset jitter opt-in | exact when seeded; otherwise ±2.2 cm (shift) + ±4 cm (session) of deck height |
| sun/temperature offset | recomputed per rendered frame | every 25 ticks | < 1e-4 K |
| GPU kernels | – | rotating (Kerbin-fixed) frame only, no vacuum orbit-drift compensation | irrelevant below 70 km; the CPU path has both |
| not implemented | thermal, lifting surfaces, RCS, parachutes, staging | – | not used by the pad lander |
