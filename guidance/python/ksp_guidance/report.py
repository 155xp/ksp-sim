"""Monte Carlo report: success rate and touchdown error distributions per booster class.

    python -m ksp_guidance.report runs/baseline.npz [runs/gfold.npz ...] > table.md

Success = landed intact and settled upright (sim status LANDED, tilt < 45 deg) within
`PAD_RADIUS` of the pad centre. Declared outcomes are counted separately: *abort* (the return
was infeasible at separation, the booster landed at its own impact point) and *divert* (the pad
was beyond the landing burn's reach at ignition, the booster landed at its predicted touchdown
point). Pad-landing miss statistics use intact landings that were aimed at the pad.
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


def declared(d):
    z = np.zeros(len(d["outcome"]), bool)
    ab = np.asarray(d["abort"] > 0, bool) if "abort" in d else z
    dv = np.asarray(d["divert"] > 0, bool) if "divert" in d else z.copy()
    return ab, dv


def row(name, d, mask):
    n = int(mask.sum())
    if n == 0:
        return None
    ab, dvt = declared(d)
    intact = (d["outcome"] == 1) & mask
    landed = intact & ~ab & ~dvt
    ok = landed & (d["miss"] <= PAD_RADIUS)
    lm = d["miss"][landed]
    vz = -d["td_vz"][landed]
    vh = d["td_vh"][landed]
    fuel = (d["prop_left"] / np.maximum(d["prop_cap"], 1e-9))[landed]
    dv = d["dv_left"][landed]
    return dict(name=name, n=n, success=ok.sum() / n, intact=intact.sum() / n, declared=int(((ab | dvt) & mask).sum()),
                miss_max=float(lm.max()) if len(lm) else float("nan"),
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
    h = ("| class | flights | success (<= 10 m) | intact | declared abort/divert | pad miss p50 / p90 / p99 / max (m) | v_down p50 / p99 (m/s) "
         "| v_horiz p50 / p99 (m/s) | fuel left p50 / p5 | dV left p50 / p5 (m/s) |")
    out = [f"### {title}", "", h, "|" + "---|" * 10]
    for r in rows:
        out.append(
            f"| {r['name']} | {r['n']} | {100 * r['success']:.1f} % | {100 * r['intact']:.1f} % | {r['declared']} "
            f"| {r['miss_p50']:.2f} / {r['miss_p90']:.2f} / {r['miss_p99']:.2f} / {r['miss_max']:.2f} "
            f"| {r['vz_p50']:.2f} / {r['vz_p99']:.2f} | {r['vh_p50']:.2f} / {r['vh_p99']:.2f} "
            f"| {100 * r['fuel_p50']:.1f} % / {100 * r['fuel_p05']:.1f} % | {r['dv_p50']:.0f} / {r['dv_p05']:.0f} |")
    return "\n".join(out)


def failures(d):
    """Failure breakdown by outcome and by where the error came from."""
    n = len(d["outcome"])
    ab, dv = declared(d)
    pad = (d["outcome"] == 1) & ~ab & ~dv
    bad = ~(pad & (d["miss"] <= PAD_RADIUS)) & ~((ab | dv) & (d["outcome"] == 1))
    m = d["miss"][pad]
    lines = [f"pad landings: {pad.sum()}, of which > 3 m: {(m > 3).sum()}, > 5 m: {(m > 5).sum()}; bar (p99 < 3 m, max < 5 m): "
             f"{'MET' if len(m) and np.percentile(m, 99) < 3 and m.max() < 5 else 'NOT MET'}",
             f"declared: abort {int((ab & (d['outcome'] == 1)).sum())} landed safely of {int(ab.sum())}, divert {int((dv & (d['outcome'] == 1)).sum())} landed safely of {int(dv.sum())}",
             f"failures (not intact, or pad-aimed and off the pad): {bad.sum()} / {n}"]
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
