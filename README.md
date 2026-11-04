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
python/ksp_sim      gym-style LanderVecEnv; CUDA full 6-DOF tick (cuda_full.py: cuda_tick32.cu mixed
                    precision, cuda_tick.cu f64 reference); CUDA point-mass path (cuda.py)
harness/            in-game ground truth: SimHarness KSP plugin (per-tick CSV logger, geometry dumps,
                    ground survey), kOS test scripts, craft generator, launch scripts
assets/             ksc_terrain.bin (in-game raycast survey of the pad / KSC / terrain)
tools/              asset extractors (UnityPy), body code generator, terrain packer
tests/              Python tests (API, determinism, scripted landing, GPU vs CPU vs game traces)
bench/              CPU (Python API) and GPU benchmarks
data/               (git-ignored) in-game traces used by the regression tests
DESIGN.md           what KSP does each tick, frames, scenery placement, sources, precision
VALIDATION.md       generated validation report (constants, atmosphere, engines, dV, drag)
VALIDATION_INGAME.md  tick-by-tick comparison with the running game, what is not 1:1 and its bound
BENCHMARKS.md       speed history (every change gated by the trace regression tests)
```

## Build / use

```bash
uv venv .venv && uv pip install -p .venv maturin numpy pytest cupy-cuda12x   # cupy optional
.venv/bin/maturin develop --release
cargo test --release -p ksp-core # 37 unit tests + 9 in-game trace regressions (skipped without data/)
.venv/bin/python -m pytest tests # 28 Python tests (GPU ones skipped without cupy)
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

Full 6-DOF tick on the GPU (same physics as the CPU tick; state stays on the device):
```python
from ksp_sim.cuda_full import GpuTick
g = GpuTick.from_reset(1 << 20, seed=0)        # precision="mixed" (default) or "f64"
g.step(actions, substeps=5)                     # (n, 4) float32 on host or device
states = g.states()                             # (n, STATE_FIELDS) float64, ksp_core::gpu layout
```

For a lander meant to fly in the real game, randomize the scenery placement the way KSP moves it
(see DESIGN.md "Scenery placement"):
`LanderVecEnv(..., ground_shift_max=0.022, ksc_dz_max=0.04)`.

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
| pad deck surface altitude (asset) | 72.44806638965383 m ASL | MeshCollider ray cast on the prefab |
| pad deck surface altitude (in game, under the spawn point) | 72.4203 m ASL | SimHarness Physics.Raycast survey; 2.8 cm below the asset value. Moves ±2.2 cm with floating-origin re-rounding, plus a per-session KSC offset (DESIGN.md "Scenery placement") |
| Kerbin equatorial radius | 600 000 m | `CelestialBody.Radius` |
| Kerbin GM | 3.5316 × 10¹² m³/s² | `GeeASL · 9.80665 · R²` (`CelestialBody.Awake`) |
| Kerbin sidereal rotation period | 21 549.425183 s | from the 21 600 s solar day (`CelestialBody.Start`) |
| Kerbin SOI | 84 159 286.48 m | `a · (M/M_sun)^0.4` |
| Kerbin atmosphere depth | 70 000 m | `atmosphereDepth` |

## Implemented

* **Curves and bodies:**
  - Unity `AnimationCurve` / `FloatCurve` evaluation (Hermite, clamped, `AddKey` smooth tangents
    recomputed from the final neighbours).
  - All 17 stock bodies' serialized data, GM / SOI / sidereal day derivations, Kepler orbits.
* **Atmosphere:** pressure and temperature curves, latitude + time-of-day temperature offset,
  density, speed of sound, mach, q, pseudo-Reynolds, velocity smoothing.
* **Aero:** drag cubes (mirrored into the sim frame), stack-node occlusion, mach-dependent
  tip/surface/tail curves, Cd curve and power, body lift at the CoL, per-part angular drag.
* **Engines:**
  - Isp vs part static pressure, fuel flow, deprivation / flameout, throttle response.
  - Gimbal about the measured pivot; reaction wheel with ElectricCharge.
* **Frames and integration:**
  - Kerbin-fixed rotating frame with Coriolis and centrifugal terms, inertial above 100 km,
    vacuum orbit drift compensation.
  - PhysX-style semi-implicit Euler at 0.02 s, composite gyroscopics and per-part damping.
* **Landing legs, exactly as the game:** `SuspensionLoadBalancer` → `ModuleWheelSuspension`
  (BoostCurve, autoBoost, tweakables, rounding, hysteresis) → VPP wheel collider (one-tick lag,
  target clamp and slew) → PhysX suspension law, plus the locked-wheel tire friction.
* **Ground:**
  - The in-game raycast survey around the pad (0.2 m / 4 m grids), with per-collider friction.
  - Scenery placement (floating-origin re-rounding and the per-session KSC offset) as state, with
    optional reset jitter.
  - Hull-point impulse contacts. Crashes follow the game's collision rule (primary colliders
    only, collision-enter speed against crashTolerance), checked by an in-game drop series.
* **Craft geometry** taken from the game's geometry dump: parts, colliders, inertia, wheel
  colliders, thrust and gimbal transforms.
* **Batching and APIs:**
  - Batched SoA stepper (rayon), pad-landing task, PyO3 `VecEnv`, gym-style `LanderVecEnv`.
  - Full-tick CUDA kernels (mixed precision and f64), CUDA point-mass path.
* **In-game harness:** a kOS + plugin logger in a copy of the install, 10 reference traces, and
  regression tests on CPU and GPU.

## Not implemented / left

* **Per-part joints:** the vessel is one rigid body. Joint flex is bounded (thrust-axis sag
  ≤ 2e-4 rad), and post-impact sliding on hard impacts is 0.2–0.45 m over 5 s.
* **Crash rule:** validated on vertical pad drops (9.7–39.7 m/s): 10 of 11 outcomes seeded from
  the game, 9 of 11 blind. Tilted and
  sideways impacts are not yet series-tested.
* **Off-pad terrain:** ~1 cm residual on the 4 m grid. Tip-over on slopes diverges within ~2 s.
* **Not modelled:** thermal, lifting/control surfaces, RCS, parachutes, staging, multi-tank flow
  priority, `.craft` loading.
* **GPU kernels:** no vacuum orbit-drift compensation (rotating frame below 100 km only).
* **GPU env wrapper:** observation / reward / auto-reset on the device. The kernel steps physics
  only; `LanderVecEnv` runs on the CPU.

## Validation

* **Against the running game** (VALIDATION_INGAME.md, 10 traces):
  - One-step CoM acceleration error, airborne: median 7e-5 m/s² (ascent to 80 km) to 6e-3 m/s²
    (gimbal-heavy hops).
  - 1-s open-loop windows: ≤ 1.2 cm and ≤ 0.12° in the ascent / hover / suicide burns;
    ≤ 2.7 cm / 0.84° in gimbal-heavy pad landings and hops.
  - Touchdown happens on the same tick as in the game in every trace, with 0.2–7.7 mm position
    error at touchdown.
  - Soft pad landings stay within 5 cm 5 s later; hard impacts drift 0.2–0.45 m.
  - Per-part drag and lift match the game to 4–5 significant figures; the PhysX wheel
    spring/damper/target match exactly.
* **Constants** (VALIDATION.md): radius, GM, SOI, sidereal day etc. to ≤ 1.5e-5 relative.
* **Pad:**
  - The game spawn point is within 2.7 cm horizontally of the derived pad.
  - The deck is at 72.4203 m by in-game raycast, ±2.2 cm from origin-shift re-rounding plus a
    per-session offset (0 in 7 of 9 sessions).
* **GPU:**
  - The mixed kernel's error against the game equals the CPU's to 3 significant figures on every
    trace.
  - Against the CPU: ≤ 1.3e-6 m after 5 ticks, ≤ 6e-6 m after 50.
  - The f64 kernel matches the CPU to 1e-9 m.

## Performance (i7-13700KF, RTX 4060 Ti; full history and method in BENCHMARKS.md)

Full 6-DOF hopper, CPU (rayon, measured on a quiet machine before the scenery change):
1.17 M / 8.4 M / 14.1 M env-ticks/s at 1 / 1 024 / 65 536 envs; single-thread tick 609 ns
airborne, 842 ns landed.

Full 6-DOF hopper, GPU (`bench/bench_cuda_full.py`, 5 ticks per launch):

| kernel | reset | 65 536 envs | 1 048 576 envs | vs CPU at 65 536 |
|---|---|---|---|---|
| mixed (default) | default (600–1500 m) | 246 M ticks/s | 283 M ticks/s | 17–20x |
| mixed | near ground (2–30 m) | 137 M ticks/s | 151 M ticks/s | 10x |
| f64 reference | default | 37 M ticks/s | 41 M ticks/s | 2.9x |
