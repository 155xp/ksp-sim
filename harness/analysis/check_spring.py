"""Reproduce the logged native WheelCollider spring/damper/target from logged inputs using the
ModuleWheelSuspension + SuspensionLoadBalancer + VPWheelCollider logic (open loop)."""
import sys

import numpy as np

sys.path.insert(0, "python")
from ksp_sim.trace import load_csv  # noqa: E402

f32 = np.float32
GEE = 1.0003416049313476
MAXG = 1.7005807760832998  # Eve: PSystemManager.MaximumSurfaceGeeASL (max over solid bodies)


def rnd(x):  # Mathf.Round: banker's rounding
    return float(np.round(f32(x)))


def lerp(a, b, t):
    return a + (b - a) * min(max(t, 0.0), 1.0)


def invlerp(a, b, v):
    return 0.0 if a == b else min(max((v - a) / (b - a), 0.0), 1.0)


def boost_curve(b, p):
    b = min(max(b, 0.0), 2.0)
    p = min(max(p, 0.01), 1.0)
    return round(min(max(1.0 / abs(1.0 - 2.0 / b ** p), 0.01), 1e7) * 10.0) / 10.0


def run(path, order="slb_mws_vpwc", d_cfg=0.95, d=1.1875, r=0.15, ratio=33.0, dratio=2.2, maxload=40.0):
    t = load_csv(path)
    nwc = sum(1 for c in t if c.endswith("_grounded"))
    n = len(t["tick"])
    st = [dict(spring=0.0, damper=0.0, rate=0.0, drate=0.0, rt=0.0, rtd=0.0, tgt=None, ab=0.0, stw=1.0, dtw=1.0, set=False) for _ in range(nwc)]
    pred = np.full((n, nwc, 3), np.nan)
    for i in range(n):
        grounded = [t[f"wc{j}_grounded"][i] > 0.5 for j in range(nwc)]
        ng = max(sum(grounded), 1)
        M = t["mass"][i]
        T = M
        if t["landed"][i] > 0.5:
            T = round(T * GEE * 10.0) / 10.0
        S = max(T * (round(100.0 / ng) / 100.0), 0.5)
        for j in range(nwc):
            s = st[j]
            if grounded[j]:
                cd = d + r - (t[f"wc{j}_compr"][i] + r)
                c = cd / d
                num = M * c
                if c > 0.8:
                    s["ab"] = min(max(s["ab"] + 0.2 * 0.02, 0.85), 2.0)
                elif c < 0.4:
                    s["ab"] = max(0.0, s["ab"] - 0.2 * 0.02)
                boost = 1.0 + min(max(c * boost_curve(GEE, 1.0), -1.0), 0.5)
                if (not s["set"]) or num > maxload:
                    s["dtw"] = round(lerp(1.0, 2.0, invlerp(1.0, MAXG, GEE)), 2)
                    if s["spring"] > 0 and ((not s["set"]) or num > maxload):
                        v = d_cfg * s["spring"] / 9.80665
                        s["stw"] = round(lerp(1.0, 3.0, invlerp(0.0, maxload * 2.0, v)), 2)
                        s["set"] = True
                fudge = lerp(1.0, 15.0, invlerp(10.0, 100.0, num))
                clampmax = lerp(5000.0, 10000.0, invlerp(50.0, 200.0, M))
                spring = min(max(s["stw"] * ratio * S * boost_curve(boost, 0.7 + s["ab"]), 0.01), clampmax)
                damper = np.sqrt(min(max(spring * dratio * fudge, 0.01), clampmax)) * s["dtw"]
                s["damper"] = max(rnd(damper), 0.1)
                s["spring"] = max(rnd(spring), 0.1)
                if abs(s["spring"] - s["rate"]) > 3.0:
                    s["rate"] = s["spring"]
                if abs(s["damper"] - s["drate"]) > 1.05:
                    s["drate"] = s["damper"]
            # VPWheelCollider.UpdateWheelCollider with the runtime (previous) values, then copy
            W = 0.1 * 9.81
            k_nat = max(s["rt"], max(W / d, 0.1))
            c_nat = max(s["rtd"], 0.1)
            tgt = min(max(W / k_nat / d, 0.1), 1.0)
            if s["tgt"] is not None and abs(tgt - s["tgt"]) > 0.02:
                tgt = s["tgt"] - 0.02 if s["tgt"] > tgt else s["tgt"] + 0.02
            s["tgt"] = tgt
            pred[i, j] = (k_nat, c_nat, tgt)
            s["rt"], s["rtd"] = s["rate"], s["drate"]
    return t, pred, nwc


if __name__ == "__main__":
    for name in sys.argv[1:]:
        t, p, nwc = run(f"/home/eric/ksp-harness/KSP/SimHarnessLogs/{name}.csv")
        for j in range(nwc):
            k, c = t[f"wc{j}_spring"], t[f"wc{j}_damper"]
            ok = ~np.isnan(p[:, j, 0])
            dk = np.abs(p[ok, j, 0] - k[ok]); dc = np.abs(p[ok, j, 1] - c[ok])
            print(name, j, "spring exact %.1f%%  max|dk| %.1f   damper exact %.1f%% max|dc| %.1f" % (100 * np.mean(dk < 1e-6), dk.max(), 100 * np.mean(dc < 1e-6), dc.max()))
            bad = np.nonzero(dk > 1e-6)[0][:6]
            for b in bad:
                print("   tick", b, "pred", p[b, j], "log", k[b], c[b], t[f"wc{j}_target"][b], "grounded", t[f"wc{j}_grounded"][b])
