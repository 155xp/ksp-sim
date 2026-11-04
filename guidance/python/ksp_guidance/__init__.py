"""ksp_guidance: return-to-launch-site booster guidance for the ksp-sim KSP 1.12.5 simulator.

    >>> import ksp_guidance as kg
    >>> fleet = kg.Fleet(seed=0, n_designs=20, per_design=5, landing="baseline")
    >>> fleet.run()                       # ascent -> boostback -> descent -> landing, in parallel
    >>> res, designs = fleet.results(), fleet.designs()

Landing modes: "baseline" (stop-throttle suicide burn + ZEM/ZEV divert), "gfold" (convex
powered descent, re-planned), "external" / "residual" (RL hook, see `TerminalEnv`).
"""
from ._native import (  # noqa: F401
    ACT_DIM,
    OUTCOMES,
    PHASES,
    TERM_OBS_DIM,
    Fleet,
    TerminalEnv,
    num_threads,
    sample_designs,
)
