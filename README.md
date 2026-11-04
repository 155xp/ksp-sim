# ksp-sim

A headless, batched re-implementation of **stock Kerbal Space Program 1.12.5** flight physics
(build 3190) for RL / autonomous-landing research: many environments stepped per call from a
Rust core (struct-of-arrays state, rayon), with a gym-style Python API and a CUDA path.

Private repo: the physics was learned by mod-level reading of the decompiled game (kept local in the
git-ignored `decomp/`), and the sim is written from scratch, citing the KSP class/method each piece
re-implements. Game data needed at build time (body prefab fields, Physics.cfg curves, part stats,
drag cubes) is transcribed with tests that re-read the local install; see DESIGN.md.

## Layout

```
crates/ksp-core     Rust physics (no Python): curves, bodies, atmosphere, aero, engines, parts,
                    vessel compiler, per-env tick (sim.rs), batched SoA stepper (batch.rs), orbits
crates/ksp-py       PyO3 bindings (maturin): VecEnv with in-place numpy buffers, GIL released
python/ksp_sim      gym-style LanderVecEnv, CUDA point-mass path (cuda.py)
tools/              asset extractors (UnityPy) and code generator for bodies_data.rs
tests/              Python tests (API, determinism, scripted landing, GPU parity)
bench/              CPU (Python API) and GPU benchmarks
DESIGN.md           what KSP does each tick, frames, sources, approximations
VALIDATION.md       generated validation report
```

## Build / use

```bash
uv venv .venv && uv pip install -p .venv maturin numpy pytest cupy-cuda12x   # cupy optional
.venv/bin/maturin develop --release
cargo test -p ksp-core           # 35 Rust tests (some re-read the KSP install if present)
.venv/bin/python -m pytest tests # 6 Python tests
```

```python
import numpy as np, ksp_sim
env = ksp_sim.LanderVecEnv(num_envs=4096, seed=0, substeps=5)   # 5 x 0.02 s physics ticks per step
obs, info = env.reset()                                          # (4096, 22) float32, pad ENU frame
obs, rew, term, trunc, info = env.step(np.zeros((4096, 4), np.float32))  # throttle, pitch, yaw, roll
```

Same-step auto-reset; `info["outcome"]` (landed / crashed / out_of_bounds / timeout),
`final_dist`, `final_speed`, `episode_return` describe episodes that just ended. Observation layout
is in `python/ksp_sim/env.py`. The default craft is a "hopper": Mk1 pod + FL-T400 + LV-T45 Swivel
+ 3× LT-2 legs (4.89 t wet, 1650 m/s vac dV, sea-level TWR 3.5).

GPU point-mass path:
```python
from ksp_sim.cuda import PointMassBatch
b = PointMassBatch(1 << 20, backend="cuda", dtype=np.float32, substeps=10)
```

## Key constants (named + tested in `ksc.rs` / `body.rs`)

| constant | value | source |
|---|---|---|
| KSC pad latitude | −0.09720779075939527° | PQSCity.Orientate → LaunchPad prefab → `CelestialBody.GetLatLonAlt` (sharedassets9) |
| KSC pad longitude | −74.5576753437982° | same; 0.33 m from the kOS community pad `LATLNG(-0.0972092543643722, -74.557706433623)` |
| pad spawn-point altitude | 72.5667520618299 m ASL | `LaunchPad_spawn` transform (Level 3 pad) |
| pad deck surface altitude | 72.44806638965383 m ASL | MeshCollider ray cast; matches stock training saves' `alt − hgt` to < 1 cm |
| Kerbin equatorial radius | 600 000 m | `CelestialBody.Radius` |
| Kerbin GM | 3.5316 × 10¹² m³/s² | `GeeASL · 9.80665 · R²` (`CelestialBody.Awake`) |
| Kerbin sidereal rotation period | 21 549.425183 s | from the 21 600 s solar day (`CelestialBody.Start`) |
| Kerbin SOI | 84 159 286.48 m | `a · (M/M_sun)^0.4` |
| Kerbin atmosphere depth | 70 000 m | `atmosphereDepth` |

## Implemented

* Unity `AnimationCurve` / `FloatCurve` evaluation (Hermite, clamped, `AddKey` smooth tangents).
* All 17 stock bodies' serialized data; GM / SOI / sidereal day derivations; Kepler orbits
  (state ↔ elements, elliptic + hyperbolic propagation).
* Atmosphere: pressure / temperature curves, latitude + time-of-day temperature offset
  (`GetAtmoThermalStats`), density, speed of sound, mach, q, pseudo-Reynolds.
* Drag cubes (PartDatabase), stack-node occlusion, mach-dependent tip/surface/tail curves, Cd curve
  and power, pseudo-Reynolds multiplier, body lift, angular drag.
* Engines: Isp vs part static pressure, fuel flow, propellant ratios, deprivation / flameout,
  throttle response; gimbal; reaction wheel with ElectricCharge.
* Kerbin-fixed rotating frame with Coriolis + centrifugal, inertial frame above 100 km with KSP's
  switch, vacuum orbit drift compensation, PhysX-style semi-implicit Euler at 0.02 s.
* Rigid-body 6-DOF vessel from placed parts (mass / CoM / inertia updated as fuel burns), leg
  suspension contact, hull crash detection with part crash tolerances.
* Batched SoA stepper (rayon), pad-landing task (reset distribution, shaped reward, outcomes),
  PyO3 `VecEnv`, gym-style `LanderVecEnv`; CUDA point-mass + atmosphere kernel (float32/64).

## Not implemented / left

* Per-part joints (vessel is one rigid body), collider-accurate inertia, exact thrust-transform
  and gimbal-transform geometry from the `.mu` models.
* Exact `ModuleWheelSuspension` dynamics (boost, auto-spring, sprung-mass distribution), PQS
  terrain and KSC building colliders (ground is the pad-deck sphere).
* Thermal model, lifting surfaces / control surfaces in flight, RCS, parachutes, staging,
  multiple-tank flow priority, SOI transitions inside the physics tick, `.craft` file loading.
* Validation against recorded in-game trajectories (needs running KSP with a logger mod; the game
  was only read, never launched).
* CUDA port of the full 6-DOF kernel (the CPU kernel is already written gather → pure tick →
  scatter with `repr(C)` POD state, so it ports 1:1).

## Validation (full report: VALIDATION.md)

* Constants vs published values: radius, GM, SOI, sidereal day, orbital period, surface speed,
  escape velocity, Mun GM/SOI all agree to ≤ 1.5e-5 relative (limited by the published digits).
* Pad: 0.33 m from the kOS pad coordinate; deck height within 1 cm of the stock training saves.
* Atmosphere at sea level: P = 101.325 kPa (−3e-6), ρ = 1.22498 kg/m³ vs 1.225 (1.9e-5), c = 340.30 m/s.
* Engines: Swivel 215 / 167.969 kN and Terrier 60 / 14.783 kN vac / ASL match the part stats to ≤ 3e-5.
* Mk1 + FL-T400 + LV-909: 2755 m/s vac dV (community / in-game ≈ 2750).
* Dynamics: rotating-frame drop agrees with an inertial integration to 5 cm over 20 s; vacuum
  Kepler pinning holds to < 0.5 m per 100 s; GPU f32 within 3.4 cm of the f64 reference after
  20 s; point-mass model within 0.085 m/s of the 6-DOF core while the vessel stays axial.
* Not validated against the running game: drag magnitudes in flight (the per-face math follows
  the decompiled code exactly), leg touchdown dynamics.

## Performance (i7-13700KF, 24 threads, RTX 4060 Ti; other services running, ±20%)

Full 6-DOF hopper, CPU:

| envs | physics ticks/s (Rust) | env-steps/s via Python, 1 tick/step | via Python, 5 ticks/step |
|---|---|---|---|
| 1 | 1.0–1.2 M | 0.68 M | 0.23 M (1.17 M ticks/s) |
| 1 024 | 6.8–8.1 M | 6.5–7.7 M | 2.0–2.1 M (10–10.7 M ticks/s) |
| 65 536 | 11.6–14.2 M | 13.0–13.4 M | 3.1–3.3 M (15.7–16.6 M ticks/s) |

Point-mass + full atmosphere, GPU (`bench/bench_cuda.py`):

| dtype | envs | ticks/launch | physics ticks/s |
|---|---|---|---|
| float32 | 65 536 | 1 / 10 | 4.7 G / 8.0 G |
| float32 | 4 194 304 | 1 / 10 | 3.0 G / 9.5 G |
| float64 | 4 194 304 | 1 / 10 | 0.39 G / 0.42 G |
