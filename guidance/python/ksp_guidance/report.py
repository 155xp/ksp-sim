"""Monte Carlo report: success rate and touchdown error distributions per booster class.

    python -m ksp_guidance.report runs/baseline.npz [runs/gfold.npz ...] > table.md

Success = landed intact and settled upright (sim status LANDED, tilt < 45 deg) within
`PAD_RADIUS` of the pad centre.
"""
from __future__ import annotations

import json
import sys

import numpy as np

PAD_RADIUS = 10.0  # m: the flat centre of the KSC pad deck
TWR_BANDS = [(0, 6, "dry TWR < 6"), (6, 10, "dry TWR 6-10"), (10, 1e9, "dry TWR > 10")]


def load(path):
    z = np.load(path, allow_pickle=False)
    d = {k: z[k] for k in z.files if k != "meta"}
    return d, json.loads(str(z["meta"]))


def pct(x, p):
    return float(np.percentile(x, p)) if len(x) else float("nan")


def row(name, d, mask):
    n = int(mask.sum())
    if n == 0:
        return None
    landed = (d["outcome"] == 1) & mask
    ok = landed & (d["miss"] <= PAD_RADIUS)
    lm = d["miss"][landed]
    vz = -d["td_vz"][landed]
    vh = d["td_vh"][landed]
    fuel = (d["prop_left"] / np.maximum(d["prop_cap"], 1e-9))[landed]
    dv = d["dv_left"][landed]
    return dict(name=name, n=n, success=ok.sum() / n, intact=landed.sum() / n,
                miss_p50=pct(lm, 50), miss_p90=pct(lm, 90), miss_p99=pct(lm, 99),
                vz_p50=pct(vz, 50), vz_p99=pct(vz, 99), vh_p50=pct(vh, 50), vh_p99=pct(vh, 99),
                fuel_p50=pct(fuel, 50), fuel_p05=pct(fuel, 5), dv_p50=pct(dv, 50), dv_p05=pct(dv, 5))


def table(d):
    rows = [row("all", d, np.ones_like(d["outcome"], bool))]
    for s, lab in [(1, "1.25 m"), (2, "2.5 m"), (3, "3.75 m")]:
        rows.append(row(lab, d, d["d_size"] == s))
    for lo, hi, lab in TWR_BANDS:
        rows.append(row(lab, d, (d["d_twr_dry"] >= lo) & (d["d_twr_dry"] < hi)))
    for k in (1, 2, 3, 4):
        rows.append(row(f"{k} engine" + ("s" if k > 1 else ""), d, d["d_n_engines"] == k))
    for e in sorted(set(d["d_engine"].tolist())):
        rows.append(row(e, d, d["d_engine"] == e))
    return [r for r in rows if r]


def markdown(rows, title):
    h = ("| class | flights | success (<= 10 m) | intact | miss p50 / p90 / p99 (m) | v_down p50 / p99 (m/s) "
         "| v_horiz p50 / p99 (m/s) | fuel left p50 / p5 | dV left p50 / p5 (m/s) |")
    out = [f"### {title}", "", h, "|" + "---|" * 9]
    for r in rows:
        out.append(
            f"| {r['name']} | {r['n']} | {100 * r['success']:.1f} % | {100 * r['intact']:.1f} % "
            f"| {r['miss_p50']:.2f} / {r['miss_p90']:.2f} / {r['miss_p99']:.1f} "
            f"| {r['vz_p50']:.2f} / {r['vz_p99']:.2f} | {r['vh_p50']:.2f} / {r['vh_p99']:.2f} "
            f"| {100 * r['fuel_p50']:.1f} % / {100 * r['fuel_p05']:.1f} % | {r['dv_p50']:.0f} / {r['dv_p05']:.0f} |")
    return "\n".join(out)


def failures(d):
    """Failure breakdown by outcome and by where the error came from."""
    n = len(d["outcome"])
    bad = ~((d["outcome"] == 1) & (d["miss"] <= PAD_RADIUS))
    lines = [f"failures: {bad.sum()} / {n}"]
    names = ["running", "landed (off pad)", "crashed", "tipped over", "timeout", "no return"]
    for c in range(6):
        k = int((bad & (d["outcome"] == c)).sum())
        if k:
            lines.append(f"  {names[c]}: {k}")
    low = bad & (d["sep_alt"] < 13000)
    lines.append(f"  of which separated below 13 km: {int(low.sum())}")
    return "\n".join(lines)


def main(argv=None):
    argv = argv or sys.argv[1:]
    for p in argv:
        d, meta = load(p)
        print(markdown(table(d), f"{meta['landing']}{' + model errors' if meta.get('model_errors') else ''}: {meta['n']} flights ({p})"))
        print()
        print("```")
        print(failures(d))
        sep = d["sep_alt"]
        print(f"separation: alt {np.min(sep)/1e3:.1f}-{np.max(sep)/1e3:.1f} km, speed {np.min(d['sep_speed']):.0f}-{np.max(d['sep_speed']):.0f} m/s, "
              f"downrange {np.min(d['sep_downrange'])/1e3:.1f}-{np.max(d['sep_downrange'])/1e3:.1f} km, mass {np.min(d['sep_mass']):.1f}-{np.max(d['sep_mass']):.1f} t")
        print(f"wall time {meta['wall_s']:.0f} s")
        print("```")
        print()


if __name__ == "__main__":
    main()
