"""GPU (float32 / float64) point-mass vs the float64 numpy reference, and vs the full 6-DOF core."""
import numpy as np
import pytest

import ksp_sim
from ksp_sim.cuda import PointMassBatch

cp = pytest.importorskip("cupy")


def _scenario(n, rng):
    pos = np.c_[rng.uniform(-300, 300, n), rng.uniform(-300, 300, n), rng.uniform(800, 3000, n)]
    vel = np.c_[rng.uniform(-20, 20, n), rng.uniform(-20, 20, n), rng.uniform(-150, -20, n)]
    act = np.c_[rng.uniform(0, 1, n), rng.normal(0, 0.1, (n, 2)), np.ones(n)]
    return pos, vel, act


def _run(backend, dtype, pos, vel, act, enu, steps):
    b = PointMassBatch(len(pos), backend=backend, dtype=dtype, substeps=5)
    b.reset(pos, vel, temp_offset=20.0)
    a = act.copy()
    a[:, 1:] = act[:, 1:] @ enu  # ENU thrust direction -> body-fixed
    for _ in range(steps):
        b.step(a)
    return b.enu_state()


def test_gpu_matches_numpy_reference():
    rng = np.random.default_rng(0)
    pos, vel, act = _scenario(2048, rng)
    enu = PointMassBatch(1, backend="numpy").enu
    P64, V64 = _run("numpy", np.float64, pos, vel, act, enu, 200)  # 20 s
    Pd, Vd = _run("cuda", np.float64, pos, vel, act, enu, 200)
    Pf, Vf = _run("cuda", np.float32, pos, vel, act, enu, 200)
    assert np.abs(Pd - P64).max() < 1e-6, np.abs(Pd - P64).max()
    errf = np.linalg.norm(Pf - P64, axis=1)
    print("float32 GPU vs float64 ref after 20 s: max", errf.max(), "m, median", np.median(errf))
    assert errf.max() < 0.5


def test_point_mass_matches_full_sim_vertical_drop():
    # engines off, vertical fall from 3 km: the reduced model's CdA(mach) table comes from the full sim
    t, alt, vup, mach, q = ksp_sim.hopper_drop(3000.0, -50.0, 0.0, 1000, True, False)
    b = PointMassBatch(1, backend="numpy", substeps=1)
    tbl = b.t
    com_h = 3000.0
    b.reset([[0, 0, com_h]], [[0, 0, -50.0]], temp_offset=0.0)
    vz = []
    for _ in range(1000):
        b.step(np.array([[0.0, 0, 0, 1]]))
        vz.append(b.enu_state()[1][0, 2])
    vz = np.array(vz)
    err = np.abs(vz - vup)
    # the uncontrolled 6-DOF hopper starts to tilt after ~8 s (Coriolis-induced airspeed, marginal
    # engine-first stability), changing its drag; the reduced model only matches while axial
    print("point-mass vs 6-DOF vertical speed: first 8 s max err", err[:400].max(),
          "m/s; full 20 s max err", err.max(), "m/s (6-DOF tilts ~7 deg)")
    assert err[:400].max() < 0.1
    del tbl
