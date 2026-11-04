# guidance/ — booster return-to-launch-site guidance for ksp-sim

Classic (no neural net) guidance that flies a reusable booster from the KSC pad, through an
ascent script and stage separation, back to the pad: boostback, aero-steered descent, and a
landing burn — for **any** booster `craft.rs` can build from stock parts. An RL hook (and a
trained residual policy) sits on the landing burn; the classic law stays the default.

This directory is a separate Cargo workspace and Python package. It links `ksp-core`
read-only (path dependency); nothing in the sim's crates or manifests is changed.

```
core/        ksp-guidance (Rust): all guidance, the craft library, the Monte Carlo fleet
  partlib.rs   stock parts read at run time from the local KSP install (cfg + PartDatabase.cfg)
  craft.rs     parametric boosters: 1.25 / 2.5 / 3.75 m, 1-4 engines, random design sampler
  nav.rs       frames, replicated drag-cube aero (exact vs a sim tick), per-craft aero tables
  predict.rs   impact-point predictor (RK4, rotating frame, sim atmosphere, landing-burn model)
  attitude.rs  torque-allocating attitude control from the live effectiveness matrix
  landing.rs   constant-throttle stop solver, lift-aware thrust allocation
  gfold.rs     convex powered descent (lossless convexification, Clarabel SOCP)
  mission.rs   the flight: phases, guidance laws, statistics, RL observation
  fleet.rs     parallel Monte Carlo (rayon)
  terminal.rs  batched terminal-phase RL environment (the hook)
py/          PyO3 bindings (module ksp_guidance._native)
python/ksp_guidance/  mc.py (runner), report.py (tables), render.py (figure), rl.py (PPO)
docs/        figure
runs/        Monte Carlo outputs (git-ignored)
```

## Build / run

Needs the KSP 1.12.5 install for part data (`$KSP_DIR`, default the Steam/Debian path).

```bash
cd guidance
cargo test --release -p ksp-guidance                     # unit + model-vs-sim tests
uv pip install -p ../.venv maturin numpy matplotlib torch
VIRTUAL_ENV=$PWD/../.venv ../.venv/bin/maturin develop --release
../.venv/bin/python -m ksp_guidance.mc --designs 400 --per 8 --landing baseline --out runs/baseline.npz
../.venv/bin/python -m ksp_guidance.report runs/baseline.npz
../.venv/bin/python -m ksp_guidance.render --flights 100 --out docs/ksp-landings.png
../.venv/bin/python -m ksp_guidance.rl --out runs/rl
cargo run --release --example mc -- 30 3 2 baseline 7   # quick Rust-only MC (size class 2)
MC_TRACE=13 KSPG_DEBUG=1 cargo run --release --example mc -- 30 3 2 baseline 7   # replay flight #13 with logs
```

```python
import ksp_guidance as kg
f = kg.Fleet(seed=0, n_designs=50, per_design=4, landing="gfold", record=True)
f.run()                              # GIL released, all cores
res, designs = f.results(), f.designs()
```

## The mission (`mission.rs`)

| phase | what the guidance does |
|---|---|
| ascent | full stack (booster + inert upper stage). Vertical, pitch kick, `90 (h/h_turn)^k` pitch program clamped near surface prograde in thick air. **MECO** when the booster's propellant only covers `margin x` what the predictor says the return needs (impulsive boostback solved by Newton, then the predicted landing burn), margin dispersed 1.15-1.45. |
| separation | switch to the booster-only spec at the same state, plus dispersed separation kicks. |
| flip | turn to the boostback direction. If the wheels can't do it in ~20 s against the live aero moment, burn at the throttle that gives the gimbal the authority. |
| boostback | Newton on the predicted touchdown error vs an impulsive dv (3 unknowns, 2 equations, minimum-norm member of the solution family), re-solved every 0.2 s from the actual state while burning; near the end the family member aligned with the thrust axis is used so trimming never needs another flip. Arcs that stay in the dense air aim deliberately long (5 % of downrange): an engine-first booster can always shorten its range, never extend it. |
| coast | point at the predicted entry retrograde. |
| aero descent | steer the touchdown point with angle of attack: the command is a fraction of the largest angle the craft's wheels can hold against the aero moment *along the predicted path*; Newton with a damped Jacobian of the predicted touchdown vs that command. If saturated and still > 30 m off, a short trim burn along the thrust axis (same solver). |
| landing burn | ignition from an energy balance shared by the predictor and the guidance; baseline or G-FOLD (below); a constant-rate final segment whose height comes from the craft's attitude agility; leg-footprint tilt limit. |

### Landing laws

* **Baseline** (`landing.rs`, `baseline_accel`): the vertical channel re-solves, every tick, the
  constant throttle that ends at the hold height at the touchdown rate (forward simulation of
  thrust/Isp at local pressure, mass flow, gravity, drag; bisection). Horizontal: ZEM/ZEV to rest
  over the pad, timed to finish one attitude swing before the final segment, with its effective
  time-to-go floored at 3 swing times of this craft (an outer loop faster than the attitude loop
  just oscillates). Planned deceleration is capped at 3 g above hover.
* **G-FOLD** (`gfold.rs`): fuel-optimal SOCP (r, v, ln m, thrust accel, slack per node, 24 nodes)
  with the lossless thrust bounds, mass bound, 35° pointing cone, 8° glide slope, linear
  Coriolis and drag along the previous solution; golden-section on the final time; re-planned every
  second, tracked with planned acceleration + PD; minimum-miss fallback if pinpoint is infeasible;
  same 3 g cap and terminal segment as the baseline.
* Both feed a **lift-aware allocator**: at high dynamic pressure a tilted booster's body lift
  can exceed the thrust's side component and reverse a divert, so the tilt and throttle are solved
  against the craft's own lift/drag tables (roll-resolved), capped at the angle of attack its
  wheels + gimbal can hold.

### Craft-agnostic by construction

Read from the spec / sim state every tick: mass, CoM, inertia tensor, per-engine thrust and Isp at
the local static pressure, min-throttle, gimbal geometry and range (torque Jacobian at the commanded
throttle), reaction-wheel torque and ElectricCharge, drag cubes (replicated per part), leg geometry.
Built once per design from the drag cubes: aero tables over mach x angle of attack x roll azimuth.
The constants in the code are dimensionless shape choices (margins, damping, "3 swing times", the
3 g cap), never values for a particular rocket.

### RL hook

`LandingMode::External { residual }` takes a 3-vector per flight: the requested non-gravitational
acceleration in pad ENU, in units of the craft's current max thrust acceleration — added to the
baseline request when `residual`. `TerminalEnv` (Python) batches episodes that start from real
ignition states; `Flight::terminal_obs` is scale-free and carries the craft parameters (TWR, mass,
angular authority, ballistic coefficient, fuel, touchdown height). `rl.py` trains a PPO residual
policy across the booster distribution and evaluates it against the baseline on held-out
boosters. The classic guidance remains the default and the fallback.

## Results

`docs/RESULTS.md`: version-by-version Monte Carlo tables per booster class (3,200 flights per
run), the verdict against the 3 m / 5 m bar, failure analysis; `docs/ksp-landings.png`: 100
real trajectories. Final (v6, baseline law): 99.0 % pad landings within 10 m, 99.9 % intact,
pad miss p50 0.09 m / p99 2.64 m; 1.25 m and 3.75 m classes p99 <= 0.54 m with and without model
errors; the 2.5 m class tail (Vector clusters, 3-3.7 m) and one 19 m outlier remain.
