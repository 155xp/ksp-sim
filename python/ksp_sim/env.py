"""Gym-style vectorized wrapper around the Rust `VecEnv` (pad-landing task).

Observation (float32, OBS_DIM = 22), pad East/North/Up frame at the KSC launch pad deck:
    0-2   CoM position relative to the pad deck centre (m)
    3-5   surface-relative velocity (m/s)
    6-8   vessel up axis (root +Y), 9-11 vessel forward axis (root +Z)
    12-14 angular velocity, vessel-local (rad/s)
    15    lowest leg foot height above ground (m)
    16    liquid-fuel fraction, 17 engine throttle, 18 mass (t)
    19    fraction of legs touching, 20 mach, 21 dynamic pressure (kPa)
Action (float32, ACT_DIM = 4): throttle [0, 1], pitch, yaw, roll [-1, 1].

Episodes auto-reset inside `step` (same-step reset): the returned obs of a finished env is the
first observation of its next episode; `info` describes the episode that just ended.
"""
from __future__ import annotations

import numpy as np

from ._ksp_sim import ACT_DIM, OBS_DIM, OUTCOMES, VecEnv

OBS_FIELDS = (
    "x_e", "x_n", "x_u", "v_e", "v_n", "v_u", "up_e", "up_n", "up_u", "fw_e", "fw_n", "fw_u",
    "w_x", "w_y", "w_z", "foot_height", "fuel", "throttle", "mass", "legs_down", "mach", "q_kpa",
)

try:  # optional: expose gymnasium spaces when gymnasium is installed
    import gymnasium as _gym

    _OBS_SPACE = _gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
    _ACT_SPACE = _gym.spaces.Box(np.array([0, -1, -1, -1], np.float32), np.ones(4, np.float32))
except ImportError:  # pragma: no cover
    _OBS_SPACE = _ACT_SPACE = None


class LanderVecEnv:
    """N independent hopper landers stepped in one Rust call.

    kwargs: substeps (0.02 s physics ticks per step, default 5), max_steps, alt_min, alt_max,
    radius, vdown_min, vdown_max, vh_max, tilt_max_deg, w_max, fuel_min, fuel_max, ut_max,
    oob_radius, oob_alt, ground_alt, friction, sun_temperature, aero.
    """

    single_observation_space = _OBS_SPACE
    single_action_space = _ACT_SPACE
    obs_dim = OBS_DIM
    act_dim = ACT_DIM

    def __init__(self, num_envs: int, seed: int = 0, **kwargs):
        self.num_envs = num_envs
        self._seed = seed
        self._kwargs = kwargs
        self._env = VecEnv(num_envs, seed, **kwargs)

    def reset(self, seed: int | None = None):
        if seed is not None and seed != self._seed:
            self._seed = seed
            self._env = VecEnv(self.num_envs, seed, **self._kwargs)
        return self._env.reset(), {}

    def step(self, actions):
        a = np.ascontiguousarray(actions, dtype=np.float32).reshape(self.num_envs, ACT_DIM)
        obs, rew, term, trunc, outcome, fdist, fspeed, fret = self._env.step(a)
        info = {"outcome": outcome, "final_dist": fdist, "final_speed": fspeed, "episode_return": fret}
        return obs, rew, term.view(bool), trunc.view(bool), info

    def env_state(self, i: int) -> dict:
        return self._env.env_state(i)

    @staticmethod
    def outcome_names(codes) -> list[str]:
        return [OUTCOMES[c] for c in codes]
