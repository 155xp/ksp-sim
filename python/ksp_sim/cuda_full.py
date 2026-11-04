"""Full 6-DOF tick on the GPU: a CUDA port of ksp_core::sim::tick.

One thread per env, state field-major in device memory, `substeps` ticks per launch. Constants,
curve keys and the surveyed KSC terrain come from the Rust core (ksp_core::gpu), so the kernel
and the CPU code read identical data. Two kernels:

- precision="mixed" (default, cuda_tick32.cu): double where KSP keeps double (CoM position and
  velocity, gravity, altitude, resources, attitude accumulation), float where Unity/PhysX compute
  in float. Several times faster on consumer GPUs (f64 runs at 1/64 rate).
- precision="f64" (cuda_tick.cu): mirrors the CPU operation for operation; matches it to rounding.

tests/test_cuda_parity.py checks both against the CPU.

    from ksp_sim.cuda_full import GpuTick
    g = GpuTick.from_reset(n=1 << 20, seed=0)
    g.step(actions, substeps=5)          # actions: (n, 4) float32 throttle, pitch, yaw, roll
    states = g.states()                  # (n, STATE_FIELDS) float64, ksp_core::gpu layout
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np

if "CUDA_PATH" not in os.environ and os.path.exists("/usr/include/cuda_runtime.h"):
    os.environ["CUDA_PATH"] = "/usr"

from ._ksp_sim import gpu_pack, gpu_reset_states, gpu_terrain

_SRC = {"f64": Path(__file__).with_name("cuda_tick.cu").read_text(), "mixed": Path(__file__).with_name("cuda_tick32.cu").read_text()}


class GpuTick:
    def __init__(self, states: np.ndarray, block: int = 128, precision: str = "mixed", pack=None):
        import cupy as cp

        if precision not in _SRC:
            raise ValueError(f"precision must be one of {sorted(_SRC)}")
        self.cp = cp
        self.precision = precision
        self.n, self.nf = states.shape
        data, header = gpu_pack() if pack is None else pack  # pack: (data, header) for another vessel
        self.mod = cp.RawModule(code=header + "\n" + _SRC[precision], options=("-std=c++14",))
        self.kernel = self.mod.get_function("tick_kernel")
        self.P = cp.asarray(data)
        with np.errstate(over="ignore"):  # 1e300 "unbreakable" sentinels become inf
            self.Pf = cp.asarray(np.asarray(data, dtype=np.float32))
        terr = gpu_terrain()
        if terr is None:
            self.tmeta, self.talt, self.tgr, self.tok = cp.zeros(16), cp.zeros(1, cp.float32), cp.zeros(1, cp.uint8), 0
        else:
            m, a, g = terr
            self.tmeta, self.talt, self.tgr, self.tok = cp.asarray(m), cp.asarray(a), cp.asarray(g), 1
        self.block = block
        self.state = cp.asarray(np.ascontiguousarray(states.T))  # field-major [field][env]

    @classmethod
    def from_reset(cls, n: int, seed: int = 0, block: int = 128, precision: str = "mixed", **task):
        return cls(gpu_reset_states(n, seed, **task), block=block, precision=precision)

    def step(self, actions, substeps: int = 1):
        cp = self.cp
        a = cp.asarray(actions, dtype=cp.float32).reshape(self.n, 4)
        grid = ((self.n + self.block - 1) // self.block,)
        p = (self.P,) if self.precision == "f64" else (self.P, self.Pf)
        self.kernel(grid, (self.block,), p + (self.tmeta, self.talt, self.tgr, np.int32(self.tok), self.state, a,
                                              np.int32(self.n), np.int32(substeps)))

    def states(self) -> np.ndarray:
        return self.cp.asnumpy(self.state).T.copy()
