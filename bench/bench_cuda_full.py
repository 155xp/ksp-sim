"""Full 6-DOF tick on the GPU (cuda_full.GpuTick): physics ticks/s vs env count."""
import time

import cupy as cp
import numpy as np

from ksp_sim.cuda_full import GpuTick

if __name__ == "__main__":
    print(cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
    print("| kernel | reset | envs | ticks/launch | kernel ms | physics ticks/s |")
    print("|---|---|---|---|---|---|")
    # default reset (600-1500 m, mostly airborne) and a near-ground start (2-30 m: legs,
    # terrain rays and the contact solve every tick once down)
    low = dict(alt_min=2.0, alt_max=30.0, vdown_min=1.0, vdown_max=5.0)
    runs = [(p, n, "default") for p in ("f64", "mixed") for n in (65536, 262144, 1 << 20)]
    runs += [("mixed", n, "near-ground") for n in (65536, 1 << 20)]
    for prec, n, task in runs:
        g = GpuTick.from_reset(n, seed=1, precision=prec, **(low if task == "near-ground" else {}))
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
            print(f"| {prec} | {task} | {n:,} | {sub} | {ms:.2f} | {n * sub / (ms / 1e3):,.0f} |")
