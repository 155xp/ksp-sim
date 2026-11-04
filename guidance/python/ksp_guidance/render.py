"""Render Monte Carlo RTLS trajectories converging on the KSC pad (real sim output only).

    python -m ksp_guidance.render --flights 100 --data runs/trajectories.npz --out docs/ksp-landings.png

Step 1 (`--flights`) flies the fleet with recording on and saves every trajectory sample;
step 2 draws from that file. Positions are the sim's CoM in the pad's East-North-Up tangent
frame (km); Kerbin's surface is drawn as the sphere the sim uses (pad-deck altitude), which
drops below the tangent plane as d^2 / 2R. Terrain is not modelled by the sim, so none is drawn.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

R_KERBIN = 600_000.0
# phase codes (mission.rs): 0 ascent, 1 flip, 2 boostback, 3 coast, 4 descent, 5 landing burn, 6 touchdown
GROUPS = [("ascent", (0,)), ("flip + boostback / trim", (1, 2)), ("coast + aero descent", (3, 4)), ("landing burn", (5, 6))]
# dark-surface categorical slots 1-4 of the dataviz reference palette (validated, adjacent pairs)
COLORS = ["#3987e5", "#d95926", "#199e70", "#c98500"]
SURFACE = "#1a1a19"
INK = "#f0efec"
INK2 = "#c3c2b7"
MUTED = "#6e6d68"


def fly(n: int, seed: int, out: str):
    from . import Fleet

    per = 2
    f = Fleet(seed=seed, n_designs=(n + per - 1) // per, per_design=per, landing="baseline", record=True)
    f.run()
    res = f.results()
    des = f.designs()
    tr = [f.trajectory(i) for i in range(len(f))][:n]
    lens = np.array([len(t) for t in tr])
    flat = np.concatenate(tr)
    keep = {k: np.asarray(v)[:n] for k, v in res.items()}
    sizes = np.array([des[i]["size"] for i in keep["design"]])
    labels = np.array([des[i]["label"] for i in keep["design"]])
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, traj=flat, lens=lens, size=sizes, label=labels, **keep)
    print(f"saved {len(tr)} trajectories ({flat.shape[0]} samples) to {out}")


def draw(data: str, out: str, width_px: int = 3000):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    z = np.load(data)
    flat, lens = z["traj"], z["lens"]
    starts = np.concatenate([[0], np.cumsum(lens)[:-1]])
    outcome, fe, fn = z["outcome"], z["final_e"], z["final_n"]
    sizes = z["size"]
    miss = np.hypot(fe, fn)
    landed = outcome == 1
    n = len(lens)

    dpi = 200
    fig = plt.figure(figsize=(width_px / dpi, width_px / dpi * 0.56), dpi=dpi, facecolor=SURFACE)
    ax = fig.add_axes([0.0, 0.0, 0.70, 0.93], projection="3d", facecolor=SURFACE)
    ax.set_proj_type("persp", focal_length=0.35)

    # extent from the data
    xs, ys, zs = flat[:, 1] / 1e3, flat[:, 2] / 1e3, flat[:, 3] / 1e3
    xmin, xmax = min(-5.0, xs.min() - 2), xs.max() + 2
    ymax = max(abs(ys).max() + 2, 0.25 * (xmax - xmin))
    ztop = zs.max() * 1.02

    # Kerbin's surface: the sim's sphere, as a recessive grid
    gx = np.linspace(xmin, xmax, 60)
    gy = np.linspace(-ymax, ymax, 30)
    GX, GY = np.meshgrid(gx, gy)
    GZ = -(GX ** 2 + GY ** 2) * 1e3 / (2 * R_KERBIN)
    ax.plot_surface(GX, GY, GZ, color="#22221f", linewidth=0, antialiased=True, shade=False, alpha=1.0, zorder=0)
    for xv in np.arange(np.ceil(xmin / 10) * 10, xmax + 1e-9, 10):
        yy = np.linspace(-ymax, ymax, 50)
        ax.plot(np.full_like(yy, xv), yy, -(xv ** 2 + yy ** 2) * 1e3 / (2 * R_KERBIN), color="#33332f", lw=0.5, zorder=1)
    for yv in np.arange(np.ceil(-ymax / 10) * 10, ymax + 1e-9, 10):
        xx = np.linspace(xmin, xmax, 80)
        ax.plot(xx, np.full_like(xx, yv), -(xx ** 2 + yv ** 2) * 1e3 / (2 * R_KERBIN), color="#33332f", lw=0.5, zorder=1)

    # trajectories, coloured by phase
    for i in range(n):
        t = flat[starts[i]:starts[i] + lens[i]]
        ph = t[:, 4].astype(int)
        for gi, (_, codes) in enumerate(GROUPS):
            m = np.isin(ph, codes)
            if not m.any():
                continue
            # contiguous runs, joined to the previous sample so phases connect
            idx = np.flatnonzero(m)
            breaks = np.flatnonzero(np.diff(idx) > 1)
            for seg in np.split(idx, breaks + 1):
                a = max(seg[0] - 1, 0)
                b = seg[-1] + 1
                ax.plot(t[a:b, 1] / 1e3, t[a:b, 2] / 1e3, t[a:b, 3] / 1e3, color=COLORS[gi], lw=0.55, alpha=0.85, solid_capstyle="round", zorder=3 + gi)
        # separation point
        sep = np.flatnonzero(ph >= 1)
        if len(sep):
            s = t[sep[0]]
            ax.scatter([s[1] / 1e3], [s[2] / 1e3], [s[3] / 1e3], s=6, color=INK, edgecolors="none", depthshade=False, zorder=8)
    ax.scatter([0], [0], [0], s=40, marker="o", facecolors="none", edgecolors=INK, linewidths=1.0, depthshade=False, zorder=9)
    ax.text(0, -ymax * 0.12, 0.6, "KSC pad", color=INK, fontsize=9, zorder=10)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(-ymax, ymax)
    ax.set_zlim(-1.0, ztop)
    ax.set_box_aspect((xmax - xmin, 2 * ymax, ztop + 1.0))
    ax.view_init(elev=16, azim=-118)
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.set_pane_color((0, 0, 0, 0))
        a.line.set_color(MUTED)
        a._axinfo["grid"]["color"] = (1, 1, 1, 0.05)
        a._axinfo["grid"]["linewidth"] = 0.4
    ax.tick_params(colors=INK2, labelsize=7, pad=1)
    ax.set_xlabel("east of pad (km)", color=INK2, fontsize=8, labelpad=6)
    ax.set_ylabel("north (km)", color=INK2, fontsize=8, labelpad=4)
    ax.set_zlabel("altitude above pad (km)", color=INK2, fontsize=8, labelpad=4)

    # title + stats
    ok = landed & (miss <= 10)
    lm = miss[landed]
    fig.text(0.03, 0.955, "Booster return-to-launch-site, KSP 1.12.5 (ksp-sim)", color=INK, fontsize=15, weight="bold", ha="left")
    fig.text(0.03, 0.925,
             f"{n} Monte Carlo flights, {len(set(z['label'].tolist()))} booster designs (1.25 / 2.5 / 3.75 m, 1-4 engines), "
             f"dispersed ascent and separation. Classic guidance, baseline landing burn.",
             color=INK2, fontsize=9, ha="left")
    handles = [Line2D([0], [0], color=c, lw=2) for c in COLORS] + [Line2D([0], [0], marker="o", color=SURFACE, markerfacecolor=INK, markersize=4, lw=0)]
    labs = [g[0] for g in GROUPS] + ["separation"]
    leg = fig.legend(handles, labs, loc="upper left", bbox_to_anchor=(0.03, 0.905), frameon=False, fontsize=8.5, labelcolor=INK2, ncol=5, handlelength=1.6, columnspacing=1.4)
    leg.set_zorder(20)

    # inset: touchdown points, top-down
    ia = fig.add_axes([0.71, 0.16, 0.27, 0.27 * width_px / (width_px * 0.56)], facecolor="#202020")
    lim = max(3.0, float(np.percentile(miss[landed], 98)) * 1.3) if landed.any() else 10.0
    lim = min(lim, 15.0)
    for r, ls in [(10.0, "-"), (1.0, ":")]:
        th = np.linspace(0, 2 * np.pi, 200)
        ia.plot(r * np.cos(th), r * np.sin(th), color=MUTED, lw=0.7, ls=ls)
    size_c = {1: COLORS[0], 2: COLORS[1], 3: COLORS[2]}
    for s in (1, 2, 3):
        m = landed & (sizes == s)
        ia.scatter(fe[m], fn[m], s=10, color=size_c[s], edgecolors=SURFACE, linewidths=0.4, zorder=3)
    bad = ~landed
    if bad.any():
        inside = bad & (np.abs(fe) < lim) & (np.abs(fn) < lim)
        ia.scatter(fe[inside], fn[inside], s=14, marker="x", color=INK, linewidths=0.8, zorder=4)
    ia.plot([0], [0], marker="+", color=INK, ms=8, mew=0.8)
    ia.set_xlim(-lim, lim)
    ia.set_ylim(-lim, lim)
    ia.set_aspect("equal")
    ia.set_xticks([])
    ia.set_yticks([])
    for sp in ia.spines.values():
        sp.set_color("#33332f")
    # scale bar
    sb = 1.0 if lim < 6 else 5.0
    x0, y0 = -lim * 0.9, -lim * 0.88
    ia.plot([x0, x0 + sb], [y0, y0], color=INK, lw=1.5, solid_capstyle="butt")
    ia.text(x0 + sb / 2, y0 + lim * 0.04, f"{sb:.0f} m", color=INK, fontsize=8, ha="center", va="bottom")
    ia.text(-lim * 0.95, lim * 0.95, "touchdown points, top-down\n(pad centre +, rings 1 m and 10 m)", color=INK2, fontsize=7.5, va="top")
    sl = [mpl_label for mpl_label in ("1.25 m", "2.5 m", "3.75 m")]
    ih = [Line2D([0], [0], marker="o", color="#202020", markerfacecolor=size_c[s], markersize=5, lw=0) for s in (1, 2, 3)]
    ia.legend(ih, sl, loc="lower right", frameon=False, fontsize=7.5, labelcolor=INK2, handletextpad=0.3)
    txt = (f"landed on pad (<= 10 m): {ok.sum()} / {n}\n"
           f"miss p50 {np.percentile(lm, 50):.2f} m, p90 {np.percentile(lm, 90):.2f} m, max {lm.max():.2f} m" if landed.any() else "")
    fig.text(0.71, 0.13, txt, color=INK2, fontsize=8.5, va="top")
    fig.text(0.71, 0.075, "Kerbin surface drawn as the sim's sphere at pad-deck altitude;\nterrain and buildings are not modelled.", color=MUTED, fontsize=7, va="top")

    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=dpi, facecolor=SURFACE)
    stats = dict(n=int(n), on_pad=int(ok.sum()), landed=int(landed.sum()),
                 miss_p50=float(np.percentile(lm, 50)), miss_p90=float(np.percentile(lm, 90)), miss_max=float(lm.max()))
    print(json.dumps(stats))
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--flights", type=int, default=0, help="fly and record this many flights first")
    ap.add_argument("--seed", type=int, default=77)
    ap.add_argument("--data", default="runs/trajectories.npz")
    ap.add_argument("--out", default="docs/ksp-landings.png")
    ap.add_argument("--width", type=int, default=3000)
    a = ap.parse_args(argv)
    if a.flights:
        fly(a.flights, a.seed, a.data)
    draw(a.data, a.out, a.width)


if __name__ == "__main__":
    main()
