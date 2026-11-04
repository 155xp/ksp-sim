"""Terminal-phase RL: PPO on the batched landing-burn environment, conditioned on craft params.

Episodes start at real landing-burn ignition states: random boosters flown from the pad by the
classic guidance and paused at ignition. The policy outputs a residual acceleration request
(pad ENU, units of the craft's max thrust acceleration, scaled by RESIDUAL_SCALE) on top of the
classic baseline, so a zero policy *is* the baseline and the comparison is exact. Training uses
boosters from one seed; evaluation flies ignition states of *different* boosters (another seed)
once each, with the baseline and the trained policy.

    python -m ksp_guidance.rl --train-designs 200 --updates 80 --out runs/rl

CPU torch is enough (the policy is a 2 x 128 MLP; the simulator is the cost).
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn

from . import ACT_DIM, TERM_OBS_DIM, Fleet, TerminalEnv

RESIDUAL_SCALE = 0.25


def ignition_pool(seed: int, designs: int, per: int) -> Fleet:
    f = Fleet(seed=seed, n_designs=designs, per_design=per, landing="baseline", stop_at_ignition=True)
    f.run()
    return f


class RunningNorm:
    def __init__(self, dim):
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4

    def update(self, x):
        b_mean, b_var, n = x.mean(0), x.var(0), x.shape[0]
        d = b_mean - self.mean
        tot = self.count + n
        self.mean = self.mean + d * n / tot
        self.var = (self.var * self.count + b_var * n + d * d * self.count * n / tot) / tot
        self.count = tot

    def __call__(self, x):
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8), -10, 10).astype(np.float32)

    def state(self):
        return {"mean": self.mean.tolist(), "var": self.var.tolist(), "count": self.count}


class Policy(nn.Module):
    def __init__(self, obs_dim=TERM_OBS_DIM, act_dim=ACT_DIM, hidden=128):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(obs_dim, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, act_dim))
        self.v = nn.Sequential(nn.Linear(obs_dim, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, 1))
        self.log_std = nn.Parameter(torch.full((act_dim,), -1.5))
        # start at the baseline: zero residual
        nn.init.zeros_(self.pi[-1].weight)
        nn.init.zeros_(self.pi[-1].bias)

    def dist(self, obs):
        return torch.distributions.Normal(self.pi(obs), self.log_std.exp())

    def act(self, obs, deterministic=False):
        with torch.no_grad():
            d = self.dist(obs)
            a = d.mean if deterministic else d.sample()
            return a, d.log_prob(a).sum(-1), self.v(obs).squeeze(-1)


def to_env(a: np.ndarray) -> np.ndarray:
    return (RESIDUAL_SCALE * np.tanh(a)).astype(np.float32)


def evaluate(pool: Fleet, policy: Policy | None, norm: RunningNorm | None) -> dict:
    """Fly every ignition state of `pool` once; policy None = classic baseline (zero residual)."""
    env = TerminalEnv(pool, residual=True, eval=True)
    n = env.num_envs
    obs = env.observe().copy()
    out = {k: np.full(n, np.nan) for k in ("outcome", "miss", "td_vz", "td_vh", "fuel")}
    while not env.all_finished:
        if policy is None:
            act = np.zeros((n, ACT_DIM), np.float32)
        else:
            a, _, _ = policy.act(torch.as_tensor(norm(obs)), deterministic=True)
            act = to_env(a.numpy())
        obs, rew, done, info = env.step(act)
        obs = obs.copy()
        d = done.astype(bool)
        for k in out:
            out[k][d] = info[k][d]
    return out


def summarize(e: dict, radius=10.0) -> dict:
    landed = e["outcome"] == 1
    ok = landed & (e["miss"] <= radius)
    m = e["miss"][landed]
    return dict(n=int(len(landed)), success=float(ok.mean()), intact=float(landed.mean()),
                miss_p50=float(np.percentile(m, 50)) if len(m) else np.nan,
                miss_p90=float(np.percentile(m, 90)) if len(m) else np.nan,
                miss_p99=float(np.percentile(m, 99)) if len(m) else np.nan,
                vz_p50=float(np.percentile(-e["td_vz"][landed], 50)) if len(m) else np.nan,
                vh_p50=float(np.percentile(e["td_vh"][landed], 50)) if len(m) else np.nan,
                vh_p99=float(np.percentile(e["td_vh"][landed], 99)) if len(m) else np.nan,
                fuel_used_p50=float(np.percentile(e["fuel"][landed], 50)) if len(m) else np.nan)


def train(a):
    torch.set_num_threads(4)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    train_pool = ignition_pool(a.seed, a.train_designs, a.per)
    eval_pool = ignition_pool(a.seed + 1000, a.eval_designs, a.per)
    env = TerminalEnv(train_pool, num_envs=a.envs, seed=a.seed, residual=True, ticks_per_step=a.ticks)
    print(f"pools: train {env.pool_size} ignition states, eval {TerminalEnv(eval_pool, eval=True).num_envs}; {time.time() - t0:.0f} s", flush=True)
    pol = Policy()
    opt = torch.optim.Adam(pol.parameters(), lr=a.lr)
    norm = RunningNorm(TERM_OBS_DIM)
    obs = env.reset().copy()
    norm.update(obs)
    T, N = a.horizon, env.num_envs
    log = []
    for upd in range(a.updates):
        buf_o = np.zeros((T, N, TERM_OBS_DIM), np.float32)
        buf_a = np.zeros((T, N, ACT_DIM), np.float32)
        buf_lp = np.zeros((T, N), np.float32)
        buf_v = np.zeros((T, N), np.float32)
        buf_r = np.zeros((T, N), np.float32)
        buf_d = np.zeros((T, N), np.float32)
        ep = {"success": [], "miss": [], "crash": 0, "n": 0}
        for t in range(T):
            on = norm(obs)
            act, lp, v = pol.act(torch.as_tensor(on))
            nobs, rew, done, info = env.step(to_env(act.numpy()))
            buf_o[t], buf_a[t], buf_lp[t], buf_v[t], buf_r[t], buf_d[t] = on, act.numpy(), lp.numpy(), v.numpy(), rew, done
            d = done.astype(bool)
            if d.any():
                oc = info["outcome"][d]
                ep["n"] += int(d.sum())
                ep["crash"] += int((oc != 1).sum())
                ep["success"] += list(((oc == 1) & (info["miss"][d] <= 10)).astype(float))
                ep["miss"] += list(info["miss"][d][oc == 1])
            obs = nobs.copy()
            norm.update(obs)
        with torch.no_grad():
            last_v = pol.v(torch.as_tensor(norm(obs))).squeeze(-1).numpy()
        adv = np.zeros((T, N), np.float32)
        gae = np.zeros(N, np.float32)
        for t in reversed(range(T)):
            nv = last_v if t == T - 1 else buf_v[t + 1]
            nonterm = 1.0 - buf_d[t]
            delta = buf_r[t] + a.gamma * nv * nonterm - buf_v[t]
            gae = delta + a.gamma * a.lam * nonterm * gae
            adv[t] = gae
        ret = adv + buf_v
        o = torch.as_tensor(buf_o.reshape(-1, TERM_OBS_DIM))
        ac = torch.as_tensor(buf_a.reshape(-1, ACT_DIM))
        olp = torch.as_tensor(buf_lp.reshape(-1))
        ad = torch.as_tensor(adv.reshape(-1))
        ad = (ad - ad.mean()) / (ad.std() + 1e-8)
        rt = torch.as_tensor(ret.reshape(-1))
        idx = np.arange(T * N)
        for _ in range(a.epochs):
            np.random.shuffle(idx)
            for s in range(0, len(idx), a.minibatch):
                b = torch.as_tensor(idx[s:s + a.minibatch])
                d = pol.dist(o[b])
                lp = d.log_prob(ac[b]).sum(-1)
                ratio = (lp - olp[b]).exp()
                pl = -torch.min(ratio * ad[b], ratio.clamp(1 - a.clip, 1 + a.clip) * ad[b]).mean()
                vl = 0.5 * ((pol.v(o[b]).squeeze(-1) - rt[b]) ** 2).mean()
                loss = pl + 0.5 * vl - a.ent * d.entropy().sum(-1).mean()
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(pol.parameters(), 0.5)
                opt.step()
        rec = dict(update=upd, steps=(upd + 1) * T * N, wall_s=time.time() - t0, episodes=ep["n"],
                   success=float(np.mean(ep["success"])) if ep["success"] else np.nan,
                   crash_rate=ep["crash"] / max(ep["n"], 1),
                   miss_p50=float(np.median(ep["miss"])) if ep["miss"] else np.nan,
                   mean_reward=float(buf_r.sum() / max(ep["n"], 1)), std=pol.log_std.exp().detach().numpy().round(3).tolist())
        log.append(rec)
        print(json.dumps(rec), flush=True)
        if (upd + 1) % 20 == 0 or upd == a.updates - 1:
            torch.save({"policy": pol.state_dict(), "norm": norm.state()}, out / "policy.pt")
    base = summarize(evaluate(eval_pool, None, None))
    rl = summarize(evaluate(eval_pool, pol, norm))
    base_tr = summarize(evaluate(train_pool, None, None))
    rl_tr = summarize(evaluate(train_pool, pol, norm))
    res = dict(eval_held_out=dict(baseline=base, rl_residual=rl), eval_train_states=dict(baseline=base_tr, rl_residual=rl_tr), log=log, args=vars(a))
    (out / "results.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res["eval_held_out"], indent=1))
    print(json.dumps(res["eval_train_states"], indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--train-designs", type=int, default=200)
    ap.add_argument("--eval-designs", type=int, default=100)
    ap.add_argument("--per", type=int, default=4)
    ap.add_argument("--envs", type=int, default=512)
    ap.add_argument("--ticks", type=int, default=5)
    ap.add_argument("--horizon", type=int, default=64)
    ap.add_argument("--updates", type=int, default=80)
    ap.add_argument("--epochs", type=int, default=6)
    ap.add_argument("--minibatch", type=int, default=4096)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.995)
    ap.add_argument("--lam", type=float, default=0.95)
    ap.add_argument("--clip", type=float, default=0.2)
    ap.add_argument("--ent", type=float, default=0.0)
    ap.add_argument("--out", default="runs/rl")
    train(ap.parse_args(argv))


if __name__ == "__main__":
    main()
