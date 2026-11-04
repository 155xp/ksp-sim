"""ksp_sim: fast, batched re-implementation of stock KSP 1.12.5 flight physics.

    >>> import numpy as np, ksp_sim
    >>> env = ksp_sim.LanderVecEnv(num_envs=1024, seed=0)
    >>> obs, info = env.reset()
    >>> obs, rew, term, trunc, info = env.step(np.zeros((1024, 4), np.float32))
"""
from ._ksp_sim import (  # noqa: F401
    ACT_DIM,
    FIXED_DT,
    KERBIN_ATMOSPHERE_DEPTH,
    KERBIN_GM,
    KERBIN_RADIUS,
    KERBIN_SIDEREAL_DAY,
    KERBIN_SOI,
    KSC_PAD_DECK_ALT,
    KSC_PAD_LAT_DEG,
    KSC_PAD_LON_DEG,
    KSC_PAD_SPAWN_ALT,
    OBS_DIM,
    OUTCOMES,
    VecEnv,
    hopper_drop,
    hopper_info,
    hopper_thrust,
    kerbin_atmosphere,
)
from .env import OBS_FIELDS, LanderVecEnv  # noqa: F401
