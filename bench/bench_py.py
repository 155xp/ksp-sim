"""Steps/sec through the Python API (includes numpy buffer handling)."""
import os
import sys
import time

import numpy as np

import ksp_sim


def bench(n, substeps, seconds=3.0):
    env = ksp_sim.LanderVecEnv(n, seed=0, substeps=substeps, max_steps=10**9)
    env.reset()
    a = np.zeros((n, 4), np.float32)
    a[:, 0] = 0.6
    for _ in range(3):
        env.step(a)
    it = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        env.step(a)
        it += 1
    dt = time.perf_counter() - t0
    return it * n / dt


if __name__ == "__main__":
    print(f"threads: {os.environ.get('RAYON_NUM_THREADS', os.cpu_count())}")
    for n in [1, 1024, 65536]:
        for sub in [1, 5]:
            sps = bench(n, sub)
            print(f"n={n:>6} substeps={sub}: {sps:>12,.0f} env-steps/s  = {sps*sub:>12,.0f} physics ticks/s")
    sys.stdout.flush()
