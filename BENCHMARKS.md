# Benchmarks

Machine: i7-13700KF (8P+8E cores, 24 threads), RTX 4060 Ti, Linux. Another user process was
using ~3 cores during most of these runs (load average 12-15), so batch numbers carry roughly
+-15% noise; single-thread numbers are pinned to P-core 0 (`taskset -c 0`) and best-of-5.

Every change below keeps the in-game regression traces (`cargo test --release -p ksp-core --test
traces`) passing within their bounds; speed changes that alter physics are not accepted.

How to reproduce:
- single-thread tick ablation: `taskset -c 0 target/release/examples/profile_tick`
- batched stepping (Rust, 1 physics tick per env-step): `cargo run --release -p ksp-core --example bench`
- Python API: `.venv/bin/python bench/bench_py.py`; GPU: `bench/bench_cuda.py`

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
