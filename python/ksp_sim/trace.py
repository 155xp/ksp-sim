"""Load in-game ground-truth traces written by the SimHarness plugin (harness/plugin).

A trace is <test>.csv (one row per physics tick, state *before* that tick's PhysX step) plus
<test>.geometry.cfg (exact rigidbody / transform data of every part, dumped once).

Frames: vectors are in the sim's body-fixed frame (BodyFrame.WorldToLocal(world.xzy), the frame
CelestialBody.GetLatLonAlt uses). Root-local geometry is Unity's left-handed part frame; `to_sim`
mirrors x (x -> -x) into the sim's right-handed root frame.
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _vec(s):
    return np.array([float(x) for x in s.split(",")])


@dataclass
class GamePart:
    idx: int
    name: str
    parent: int
    local_pos: np.ndarray
    local_rot: np.ndarray  # x, y, z, w (Unity, root-local)
    mass: float
    resource_mass: float
    physics_mass: float
    rb: dict = field(default_factory=dict)
    lines: list = field(default_factory=list)

    def get(self, key):
        return [ln.split("=", 1)[1].strip() for ln in self.lines if ln.strip().startswith(key + " =")]

    def kv(self, key):
        """`key = a=1 b=2 ...` lines -> list of dicts."""
        out = []
        for s in self.get(key):
            d = {}
            for m in re.finditer(r"(\w+)=([^ ]+)", s):
                d[m.group(1)] = m.group(2)
            out.append(d)
        return out


@dataclass
class Geometry:
    header: dict
    parts: list


def load_geometry(path) -> Geometry:
    header, parts, cur = {}, [], None
    for ln in Path(path).read_text().splitlines():
        if ln.startswith("PART {"):
            cur = []
        elif ln.startswith("}") and cur is not None:
            d = {l.split("=", 1)[0].strip(): l.split("=", 1)[1].strip() for l in cur if "=" in l}
            m = [float(x) for x in d["mass"].split(",")]
            gp = GamePart(int(d["idx"]), d["name"], int(d["parent"]), _vec(d["localPos"]), _vec(d["localRot"]),
                          m[0], m[1], m[2], lines=cur)
            for k in ("rb.mass", "rb.centerOfMass", "rb.worldCoM_rootLocal", "rb.inertiaTensor", "rb.inertiaTensorRotation", "rb.drag"):
                if k in d:
                    gp.rb[k] = _vec(d[k])
            parts.append(gp)
            cur = None
        elif cur is not None:
            cur.append(ln)
        elif "=" in ln:
            k, v = ln.split("=", 1)
            header[k.strip()] = v.strip()
    return Geometry(header, parts)


def load_csv(path) -> dict:
    """Columns -> numpy arrays (strings where non-numeric)."""
    with open(path) as f:
        r = csv.reader(f)
        cols = next(r)
        rows = [row for row in r if len(row) == len(cols)]
    out = {}
    for j, c in enumerate(cols):
        vals = [row[j] for row in rows]
        try:
            out[c] = np.array([float(v) for v in vals])
        except ValueError:
            out[c] = np.array(vals)
    return out


def vec(tr, prefix):
    return np.stack([tr[prefix + "_x"], tr[prefix + "_y"], tr[prefix + "_z"]], 1)


# --- Unity left-handed root frame -> sim right-handed root frame (x mirrored) -----------------

def mirror_pos(p):
    p = np.array(p, float)
    p[..., 0] = -p[..., 0]
    return p


def mirror_quat(q):
    """Reflection x -> -x conjugates a rotation: (x, y, z, w) -> (x, -y, -z, w)."""
    q = np.array(q, float)
    q[..., 1] = -q[..., 1]
    q[..., 2] = -q[..., 2]
    return q


def load(log_dir, test):
    d = Path(log_dir)
    return load_csv(d / f"{test}.csv"), load_geometry(d / f"{test}.geometry.cfg")
