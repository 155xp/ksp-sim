"""Point-mass + atmosphere physics throughput on the GPU (cupy RawKernel).

Reports kernel time (CUDA events) and synced wall-clock per step (what an RL loop sees).
"""
import time

import cupy as cp
import numpy as np

from ksp_sim.cuda import PointMassBatch


def bench(n, dtype, substeps, launches=50):
    b = PointMassBatch(n, backend="cuda", dtype=dtype, substeps=substeps)
    rng = np.random.default_rng(0)
    b.reset(np.c_[rng.uniform(-300, 300, (n, 2)), rng.uniform(5e4, 6e4, n)], np.c_[np.zeros((n, 2)), -np.ones(n)])
    a = cp.asarray(np.c_[np.full(n, 0.5), rng.normal(0, 0.05, (n, 2)), np.ones(n)].astype(dtype))
    b.step(a)
    cp.cuda.Device().synchronize()
    s, e = cp.cuda.Event(), cp.cuda.Event()
    s.record()
    for _ in range(launches):
        b.step(a)
    e.record()
    e.synchronize()
    kernel_ms = cp.cuda.get_elapsed_time(s, e) / launches
    t0 = time.perf_counter()
    for _ in range(launches):
        b.step(a)
        cp.cuda.Device().synchronize()
    wall_ms = (time.perf_counter() - t0) * 1e3 / launches
    assert int(b.done.sum()) == 0
    return kernel_ms, wall_ms


if __name__ == "__main__":
    print(cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
    print("| dtype | envs | ticks/launch | kernel ms | physics ticks/s (kernel) | env-steps/s (synced wall) |")
    print("|---|---|---|---|---|---|")
    for dtype in (np.float32, np.float64):
        for n in (65536, 1 << 20, 1 << 22):
            for sub in (1, 10):
                k, w = bench(n, dtype, sub)
                print(f"| {np.dtype(dtype).name} | {n:,} | {sub} | {k:.3f} | {n * sub / (k / 1e3):,.0f} | {n / (w / 1e3):,.0f} |")
