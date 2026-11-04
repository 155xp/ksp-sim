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
    # first 4 s: the 6-DOF hopper (lift at CoL, composite gyroscopics) starts to rotate later on
    assert err[:200].max() < 0.1
    del tbl


# ---- full 6-DOF tick kernels (cuda_full.GpuTick) vs the CPU core and vs the in-game traces ----
import os
from pathlib import Path

from ksp_sim._ksp_sim import cpu_step_packed, gpu_reset_states


def _step_both(prec, n, seed, launches):
    from ksp_sim.cuda_full import GpuTick

    s0 = gpu_reset_states(n, seed)
    rng = np.random.default_rng(seed)
    act = np.c_[rng.uniform(0.2, 0.9, n), rng.uniform(-0.3, 0.3, (n, 3))].astype(np.float32)
    g = GpuTick(s0, precision=prec)
    sc = s0.copy()
    for _ in range(launches):
        g.step(act, 5)
        sc = cpu_step_packed(sc, act, 5)
    sg = g.states()
    return np.linalg.norm(sg[:, 1:4] - sc[:, 1:4], axis=1), np.linalg.norm(sg[:, 4:7] - sc[:, 4:7], axis=1)


@pytest.mark.parametrize("prec, tol5, tol50", [("f64", (1e-8, 1e-7), (1e-6, 1e-6)), ("mixed", (1e-5, 1e-5), (1e-4, 2e-4))])
def test_full_tick_matches_cpu(prec, tol5, tol50):
    # reset distribution (hover / landing starts incl. leg contact), 5 and 50 ticks
    for launches, (tp, tv) in ((1, tol5), (10, tol50)):
        dx, dv = _step_both(prec, 512, 3, launches)
        print(f"{prec}: {5 * launches} ticks, GPU vs CPU pos max {dx.max():.2e} m, vel max {dv.max():.2e} m/s")
        assert dx.max() < tp and dv.max() < tv


_TRACES = Path(os.environ.get("KSP_SIM_TRACES", Path(__file__).resolve().parents[1] / "data" / "traces"))
_TRACE_CASES = [("v2", "Ascent80"), ("v2", "HopperHover"), ("v2", "HopperSuicide"), ("v2", "HopperDrop2"), ("v2", "HopperHop"),
                ("v3", "HopperSuicide"), ("v6", "HopperPadLand")]


@pytest.mark.parametrize("prec", ["f64", "mixed"])
@pytest.mark.parametrize("batch, name", _TRACE_CASES)
def test_full_tick_vs_game_traces(prec, batch, name):
    """Every logged tick re-seeded as one env and stepped once on the GPU: the error against the
    game must equal the CPU tick's (the GPU precision is invisible next to the sim-vs-game error)."""
    from ksp_sim._ksp_sim import trace_gpu_cases, trace_one_step_errors
    from ksp_sim.cuda_full import GpuTick

    d = _TRACES / batch
    if not (d / f"{name}.csv").exists():
        pytest.skip("no trace data")
    data, hdr, st, act, rows = trace_gpu_cases(str(d), name)
    g = GpuTick(st, precision=prec, pack=(data, hdr))
    g.step(act, 1)
    ga, gw, ca, cw, gr = trace_one_step_errors(str(d), name, g.states(), rows)
    for m in (~gr, gr):
        if m.sum() < 50:
            continue
        for gpu, cpu in ((ga, ca), (gw, cw)):
            for q in (50, 95):
                a, b = np.percentile(gpu[m], q), np.percentile(cpu[m], q)
                assert a <= b * 1.02 + 1e-5, (name, q, a, b)
