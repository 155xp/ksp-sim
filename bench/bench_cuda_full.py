"""Full 6-DOF tick on the GPU (cuda_full.GpuTick): physics ticks/s vs env count."""
import time

import cupy as cp
import numpy as np

from ksp_sim.cuda_full import GpuTick

if __name__ == "__main__":
    print(cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
    print("| kernel | envs | ticks/launch | kernel ms | physics ticks/s |")
    print("|---|---|---|---|---|")
    for prec, n in [(p, n) for p in ("f64", "mixed") for n in (65536, 262144, 1 << 20)]:
        g = GpuTick.from_reset(n, seed=1, precision=prec)
        rng = np.random.default_rng(0)
        act = cp.asarray(np.c_[rng.uniform(0.3, 0.8, n), rng.uniform(-0.2, 0.2, (n, 3))].astype(np.float32))
        for sub in (1, 5):
            g.step(act, sub)
            cp.cuda.Device().synchronize()
            s, e = cp.cuda.Event(), cp.cuda.Event()
            reps = 10
            s.record()
            for _ in range(reps):
                g.step(act, sub)
            e.record()
            e.synchronize()
            ms = cp.cuda.get_elapsed_time(s, e) / reps
            print(f"| {prec} | {n:,} | {sub} | {ms:.2f} | {n * sub / (ms / 1e3):,.0f} |")
