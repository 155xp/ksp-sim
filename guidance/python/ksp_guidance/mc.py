"""Monte Carlo runner: random boosters x separation dispersions through the full RTLS mission.

    python -m ksp_guidance.mc --designs 300 --per 10 --landing baseline --out runs/baseline.npz

Writes every per-flight result plus the design table (one row per flight) to an .npz.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from . import Fleet


def run(seed: int, designs: int, per: int, landing: str, size: int = 0, model_errors: bool = False,
        record: bool = False) -> tuple[dict, list, Fleet]:
    f = Fleet(seed=seed, n_designs=designs, per_design=per, size=size, landing=landing,
              model_errors=model_errors, record=record)
    f.run()
    return f.results(), f.designs(), f


def flatten(res: dict, designs: list) -> dict:
    """Per-flight arrays incl. the design columns broadcast to flights."""
    out = {k: np.asarray(v) for k, v in res.items()}
    idx = out["design"]
    for k in designs[0]:
        col = [d[k] for d in designs]
        out["d_" + k] = np.asarray(col)[idx]
    out["miss"] = np.hypot(out["final_e"], out["final_n"])
    out["td_miss"] = np.hypot(out["td_e"], out["td_n"])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--designs", type=int, default=100)
    ap.add_argument("--per", type=int, default=10)
    ap.add_argument("--landing", default="baseline")
    ap.add_argument("--size", type=int, default=0)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--model-errors", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    t0 = time.time()
    res, designs, _ = run(a.seed, a.designs, a.per, a.landing, a.size, a.model_errors)
    flat = flatten(res, designs)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    meta = dict(vars(a), wall_s=time.time() - t0, n=len(flat["outcome"]))
    np.savez_compressed(a.out, meta=json.dumps(meta), **flat)
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
