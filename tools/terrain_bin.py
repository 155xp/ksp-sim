"""Pack SimHarness ground-survey CSVs into assets/ksc_terrain.bin (read by crates/ksp-core/src/terrain.rs).

Each survey ray starts 300 m above its grid node and points at the body centre, so the hit lies
on the radial line through the node: its horizontal offset is the node offset scaled by
(R + h) / (R + h_pad + 300). The effective grid step is scaled accordingly.

usage: python -I tools/terrain_bin.py <out.bin> <fine.csv> <coarse.csv>
"""
import re
import struct
import sys

import numpy as np

R = 600000.0


def read(path):
    with open(path) as f:
        head = f.readline()
        f.readline()
        meta = dict(re.findall(r"(\w+)=([^ \n]+)", head))
        rows = [ln.rstrip("\n").split(",") for ln in f]
    n = int(meta["n"])
    alt = np.full((n, n), np.nan, np.float32)
    ground = np.zeros((n, n), np.uint8)
    for r in rows:
        if r[4] == "1":
            alt[int(r[0]), int(r[1])] = float(r[5])
            # PQS terrain quads ("Kerbin Zn...") carry the "Ground" PhysicMaterial (friction 0.9,
            # combine Maximum); KSC colliders have none (Unity default 0.6, Average)
            ground[int(r[0]), int(r[1])] = 1 if r[10].startswith("Kerbin Zn") else 0
    vec = lambda k: np.array([float(x) for x in meta[k].split(",")])
    return float(meta["step"]), float(meta["half"]), n, alt, ground, vec("pad_bf"), vec("east_bf"), vec("north_bf"), vec("up_bf")


def main(out, *paths):
    grids = [read(p) for p in paths]
    _, _, _, _, _, pad, east, north, up = grids[0]
    hpad = np.linalg.norm(pad) - R
    blob = bytearray(b"KSCTERR2")
    blob += struct.pack("<d", R)
    blob += struct.pack("<12d", *pad, *east, *north, *up)
    blob += struct.pack("<d", 72.44806638965383)
    blob += struct.pack("<I", len(grids))
    for step, half, n, alt, ground, *_ in grids:
        scale = (R + np.nanmedian(alt)) / (R + hpad + 300.0)
        blob += struct.pack("<ddI", step * scale, half * scale, n)
        blob += alt.astype("<f4").tobytes()
        blob += ground.astype("u1").tobytes()
        print(f"grid step {step} m (eff {step*scale:.6f}), n {n}, hits {np.isfinite(alt).mean()*100:.1f}%, alt {np.nanmin(alt):.3f}..{np.nanmax(alt):.3f}")
    open(out, "wb").write(bytes(blob))


if __name__ == "__main__":
    main(sys.argv[1], *sys.argv[2:])
