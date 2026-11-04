# Benchmarks

Machine: i7-13700KF (8P+8E cores, 24 threads), RTX 4060 Ti, Linux. Another user process was
using ~3 cores during most of these runs (load average 12-15), so batch numbers carry roughly
+-15% noise; single-thread numbers are pinned to P-core 0 (`taskset -c 0`) and best-of-5.

Every change below keeps the in-game regression traces (`cargo test --release -p ksp-core --test
traces`) passing within their bounds; speed changes that alter physics are not accepted.

How to reproduce:
- single-thread tick ablation: `taskset -c 0 target/release/examples/profile_tick`
- batched stepping (Rust, 1 physics tick per env-step): `cargo run --release -p ksp-core --example bench`
- Python API: `.venv/bin/python bench/bench_py.py`; GPU point-mass: `bench/bench_cuda.py`; GPU full tick: `bench/bench_cuda_full.py`

## History

| # | change | 1-env tick, airborne / landed (ns) | env-ticks/s: n=1 / 1 024 / 65 536 |
|---|---|---|---|
| 0 | first version (sphere ground, approximate legs) | ~840 (unpinned) | 1.19 M / 8.1 M / 14.2 M |
| 1 | fidelity pass: exact leg suspension chain, surveyed terrain, impulse contacts, composite gyroscopics, per-part damping | 767 / 970 | 0.58 M / 4.7 M / 10.6 M (load 12.5) |
| 2 | exact ground early-out by bounding radius | 751 / 970 | - |
| 3 | drag-cube face loop in f32 (as the game: float `AddSurfaceDragDirection`, `Mathf.Pow`), axis-aligned face dots | 648 / 874 | - |
| 4 | angular update and contact effective masses in the root frame (no world-tensor builds) | 609 / 842 | 1.17 M / 8.4 M / 14.1 M |

Notes:
- Ablation at #4 (airborne): aerodynamics (6 parts) ~320 ns, everything else ~290 ns; landed adds
  ~230 ns for the wheel chain, terrain rays and the contact solve.
- 65 536 envs at 1 tick/step are memory-bound on the SoA gather/scatter (EnvState with leg and
  smoother state is ~0.9 KB); with the RL default of 5 ticks per step the throughput per tick is
  higher (see bench_py.py).

## GPU full tick (RTX 4060 Ti, `bench/bench_cuda_full.py`)

One thread per env, field-major state in device memory, the whole sim::tick per thread (engines,
gimbal, reaction wheel, drag cubes and body lift per part, gravity and rotating-frame terms, leg
suspension chain, terrain rays, hull impulse contacts, gyroscopics). Physics ticks/s, hover/landing
reset distribution, random actions. CPU reference: 14.1 M env-ticks/s at 65 536 envs (#4 above).

| # | change | ticks/s at 65 536 / 1 048 576 envs (5 ticks per launch) | vs CPU 64k |
|---|---|---|---|
| G1 | f64 port, per-thread local copy of the state | 31.8 M / 35.9 M | 2.5x |
| G2 | direct field-major global addressing (no local state copy) | 31.8 M / 35.9 M | 2.5x |
| G3 | mass properties from a linear-in-resources basis; per-part aero in the root frame (one rotation in/out) | 36.3 M / 41.0 M | 2.9x |
| G4 | mixed precision kernel (`precision="mixed"`, default): double where KSP keeps double (CoM position/velocity, gravity, altitude, resources, attitude accumulation), float where Unity/PhysX use float | 217 M / 235 M | 16.7x |
| G5 | scenery placement state (ground shift + KSC offset, 4 more state fields, flag-aware terrain sampling) | 204 M / 214 M | 15.2x |
| G6 | exact early-out above the highest ground point (176 m ASL) instead of a terrain lookup every tick | 264 M / 284 M | 20.1x |
| G7 | mixed kernel: terrain queries in float in a local frame at the pad, heights relative to the origin height (exact expansion of the radius, no 600 km / 72 m magnitudes in float) | 246 M / 283 M | 20.1x |

| G8 | crash rule from the game (primary-collider contacts only, collision-enter speed): 6 more hull points (Swivel Col5), per-env contact bitmask | 251 M / 278 M | 19.7x |

Near-ground start (2-30 m: leg rays, terrain, contact solve), mixed kernel, 65 536 / 1 048 576 envs:
G5 43 M / 46 M ticks/s -> G7 137 M / 151 M ticks/s (3.3x; the f64 sphere-ground path was 125 M)
-> G8 130 M / 142 M ticks/s (accuracy change: the extra crash-sensor points).

Notes:
- The 4060 Ti runs f64 at 1/64 of the f32 rate; register caps (64..168) and block sizes did not
  change G3's throughput, so it was f64-ALU bound. G4 is ~5.7x G3.
- Ablation at G4 (262 144 envs): no aero 338 M, no ground 287 M, neither 455 M ticks/s.
- CPU numbers were not re-measured after G5-G7: the guidance thread's Monte Carlo was using
  ~19 of the 24 hardware threads (load 24-35), which roughly halves single-thread timings. The
  CPU tick gained the same exact ground-ceiling early-out as G6.
- Accuracy gate for G4-G7 (tests/test_cuda_parity.py): every in-game trace row re-seeded as one env
  and stepped once on the GPU; the one-step error against the game equals the CPU's to 3
  significant figures (median and p95, airborne and in contact, 10 traces). Against the CPU: 5 ticks
  <= 1.3e-6 m, 50 ticks <= 6e-6 m (landings: 99% of envs <= 4e-6 m); the f64 kernel matches
  the CPU to 1e-9 m.
