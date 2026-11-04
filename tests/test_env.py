"""Gym-style API of the batched lander env."""
import numpy as np

import ksp_sim


def test_reset_step_shapes_and_autoreset():
    n = 256
    env = ksp_sim.LanderVecEnv(n, seed=3, max_steps=50)
    obs, info = env.reset()
    assert obs.shape == (n, ksp_sim.OBS_DIM) and obs.dtype == np.float32
    a = np.zeros((n, 4), np.float32)
    ended = 0
    for _ in range(60):
        obs, rew, term, trunc, info = env.step(a)
        assert rew.shape == (n,) and term.dtype == bool and np.isfinite(obs).all()
        ended += int((term | trunc).sum())
        assert ((info["outcome"] != 0) == (term | trunc)).all()
    assert ended >= n  # every env hit max_steps or crashed at least once (free fall)


def test_determinism():
    def roll(seed):
        env = ksp_sim.LanderVecEnv(64, seed=seed)
        env.reset()
        rng = np.random.default_rng(0)
        out = []
        for _ in range(100):
            a = rng.uniform([0, -1, -1, -1], [1, 1, 1, 1], (64, 4)).astype(np.float32)
            obs, *_ = env.step(a)
            out.append(obs.copy())
        return np.stack(out)

    assert np.array_equal(roll(5), roll(5))
    assert not np.array_equal(roll(5), roll(6))


def test_constants_and_pad():
    assert ksp_sim.KERBIN_RADIUS == 600_000.0
    assert abs(ksp_sim.KSC_PAD_LAT_DEG + 0.0972) < 1e-4
    assert abs(ksp_sim.KSC_PAD_LON_DEG + 74.5577) < 1e-3
    p, t, rho, c = ksp_sim.kerbin_atmosphere(np.array([0.0, 70_000.0]))
    assert abs(p[0] - 101.325) < 1e-4 and p[1] == 0.0


def attitude_hold(obs, kp=4.0, kd=6.0, target=None):
    """Reaction-wheel/gimbal command (pitch, roll, yaw) that rotates the vessel up axis to vertical.

    Vessel-local axes in ENU: y = up (obs 6:9), z = forward (obs 9:12), x = y cross z (the sim's
    right-handed root frame). As in the game, the wheel torque in that frame is
    (-pitch, +roll, +yaw) * 5 kN m (ModuleReactionWheel, Unity frame mirrored in x).
    """
    up, fw = obs[:, 6:9], obs[:, 9:12]
    x = np.cross(up, fw)
    if target is None:
        target = np.broadcast_to(np.array([0.0, 0.0, 1.0]), up.shape)
    err = np.cross(up, target)  # rotation axis taking up -> target
    w = obs[:, 12:15]
    tau = kp * np.stack([(err * x).sum(1), (err * up).sum(1), (err * fw).sum(1)], 1) - kd * w
    return np.clip(tau * np.array([-1.0, 1.0, 1.0]) / 5.0, -1, 1)


def test_hover_controller_lands():
    """A simple PD descent controller with attitude hold should touch down softly."""
    n = 512
    env = ksp_sim.LanderVecEnv(n, seed=1, radius=0.0, vh_max=0.0, tilt_max_deg=0.0, w_max=0.0, max_steps=3000)
    obs, _ = env.reset()
    landed = crashed = 0
    for _ in range(3000):
        h, vz, m = obs[:, 15], obs[:, 5], obs[:, 18]
        # target descent rate shrinking with height, thrust/weight from current mass
        v_tgt = -np.clip(0.12 * h + 1.0, 1.0, 60.0)
        acc = 9.81 + 2.0 * (v_tgt - vz)
        thr = np.clip(acc * m / 168.0, 0.0, 1.0)
        a = np.zeros((n, 4), np.float32)
        a[:, 0] = thr
        # horizontal guidance: tilt the thrust axis against horizontal offset / velocity (<= 10 deg)
        hcmd = -0.02 * obs[:, 0:2] - 0.12 * obs[:, 3:5]
        hn = np.linalg.norm(hcmd, axis=1, keepdims=True)
        hcmd = np.where(hn > 0.176, hcmd / np.maximum(hn, 1e-9) * 0.176, hcmd)
        tgt = np.c_[hcmd, np.ones(n)]
        tgt /= np.linalg.norm(tgt, axis=1, keepdims=True)
        cmd = attitude_hold(obs, target=tgt)
        a[:, 1], a[:, 3], a[:, 2] = cmd[:, 0], cmd[:, 1], cmd[:, 2]
        a[obs[:, 19] > 0] = 0.0  # engine and controls off once a leg touches
        obs, rew, term, trunc, info = env.step(a)
        landed += int((info["outcome"] == 1).sum())
        crashed += int((info["outcome"] == 2).sum())
        if landed + crashed >= n:
            break
    assert landed > 0.9 * n, (landed, crashed)
