"""Write the harness .craft files (stock parts only) exactly as the VAB would place them.

Stack parts: child.pos = parent.pos + parent.node_bottom - child.node_top (rotation identity).
Surface-attached legs: EditorLogic attach rule
    rot = LookRotation(hit.normal, parent.up) * LookRotation(srfAttachNode.orientation, up)
    pos = hit.point - rot * srfAttachNode.position
with node positions scaled by the part's rescaleFactor (PartLoader).
usage: python -I harness/make_crafts.py <out_dir>
"""
import math
import sys

import numpy as np


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def q_rot(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def q_from_matrix(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)


def look_rotation(fwd, up):
    """Unity Quaternion.LookRotation: +Z -> fwd, +Y as close to up as possible (left-handed axes)."""
    z = np.asarray(fwd, float)
    z /= np.linalg.norm(z)
    x = np.cross(up, z)  # Unity: right = up x forward (component formula identical)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return q_from_matrix(np.column_stack([x, y, z]))


PODS = {
    # name: (node_top, node_bottom) (y only, rescaleFactor 1)
    "mk1pod_v2": (0.6423756, -0.4050379),
    "fuelTank": (0.981725, -0.9125),
    "fuelTank_long": (1.875, -1.8875),
    "liquidEngine2_v2": (0.0, -1.63),
}
LT2_NODE_POS = np.array([0.0, 0.4271562, 0.0632596]) * 1.25
LT2_NODE_DIR = np.array([0.0, 0.5735765, 0.819152])


def stack(names):
    parts, y = [], 10.0
    for i, n in enumerate(names):
        top, bot = PODS[n]
        if i > 0:
            y = y_prev_bottom - top
        parts.append({"name": n, "pos": np.array([0.0, y, 0.0]), "rot": (0.0, 0.0, 0.0, 1.0)})
        y_prev_bottom = y + bot
    return parts


def leg(parent, az_deg, y_off, radius=0.625, vertical=True):
    """vertical=False: the plain editor placement (suspension axis splayed 35 deg out, which on this
    craft leaves the Swivel nozzle on the ground at rest). vertical=True: the leg rotated in the
    editor so its suspension axis (part +Y) is parallel to the vessel axis, part +Z inward."""
    a = math.radians(az_deg)
    n = np.array([math.cos(a), 0.0, math.sin(a)])
    hit = parent["pos"] + np.array([0.0, y_off, 0.0]) + n * radius
    if vertical:
        rot = look_rotation(-n, np.array([0.0, 1.0, 0.0]))
    else:
        rot = q_mul(look_rotation(n, np.array([0.0, 1.0, 0.0])), look_rotation(LT2_NODE_DIR, np.array([0.0, 1.0, 0.0])))
    pos = hit - q_rot(rot, LT2_NODE_POS)
    return {"name": "landingLeg1-2", "pos": pos, "rot": rot, "srf": True}


def write(path, ship, parts, links, attn, srfn, stage_engine):
    ids = [f"{p['name'].replace('_', '.')}_{4294000000 + i}" for i, p in enumerate(parts)]
    lines = [f"ship = {ship}", "version = 1.12.5", "description = ksp-sim harness craft", "type = VAB",
             "size = 3,8,3", "persistentId = 1234567", "rot = 0,0,0,1", "missionFlag = Squad/Flags/default",
             "vesselType = Debris" if False else "vesselType = Lander"]
    for i, p in enumerate(parts):
        f = lambda v: ",".join(repr(float(x)) for x in v)
        lines += ["PART", "{", f"\tpart = {ids[i]}", "\tpartName = Part", "\tpersistentId = " + str(1000 + i),
                  f"\tpos = {f(p['pos'])}", "\tattPos = 0,0,0", f"\tattPos0 = {f(p['pos'])}",
                  f"\trot = {f(p['rot'])}", "\tattRot = 0,0,0,1", f"\tattRot0 = {f(p['rot'])}",
                  "\tmir = 1,1,1", "\tsymMethod = Radial", "\tautostrutMode = Grandparent", "\trigidAttachment = True",
                  f"\tistg = {0 if i == stage_engine else -1}", "\tresPri = 0", "\tdstg = 0", f"\tsidx = {0 if i == stage_engine else -1}",
                  "\tsqor = -1", "\tsepI = -1", f"\tattm = {1 if p.get('srf') else 0}", "\tmodCost = 0", "\tmodMass = 0", "\tmodSize = 0,0,0"]
        for c in links.get(i, []):
            lines.append(f"\tlink = {ids[c]}")
        for node, j in attn.get(i, []):
            lines.append(f"\tattN = {node},{ids[j]}")
        if i in srfn:
            lines.append(f"\tsrfN = srfAttach,{ids[srfn[i]]}")
        lines += ["}"]
    open(path, "w").write("\n".join(lines) + "\n")


def main(out):
    # hopper: pod / FL-T400 / Swivel + 3 LT-2 (editor placement) at the tank's lower edge, 0.9 m
    # below its centre (0.3 m lower than the v1 craft, whose Swivel nozzle touched the pad at rest)
    hp = stack(["mk1pod_v2", "fuelTank", "liquidEngine2_v2"])
    for k in range(3):
        hp.append(leg(hp[1], 90.0 + 120.0 * k, -0.9, vertical=False))
    links = {0: [1], 1: [2, 3, 4, 5]}
    attn = {0: [("bottom", 1)], 1: [("top", 0), ("bottom", 2)], 2: [("top", 1)]}
    srfn = {3: 1, 4: 1, 5: 1}
    for ship in ["HopperDrop", "HopperDrop2", "HopperHover", "HopperHop", "HopperSuicide"]:
        write(f"{out}/{ship}.craft", ship, hp, links, attn, srfn, 2)
    # ascent: pod / FL-T800 / FL-T800 / Swivel
    asc = stack(["mk1pod_v2", "fuelTank_long", "fuelTank_long", "liquidEngine2_v2"])
    write(f"{out}/Ascent80.craft", "Ascent80", asc, {0: [1], 1: [2], 2: [3]},
          {0: [("bottom", 1)], 1: [("top", 0), ("bottom", 2)], 2: [("top", 1), ("bottom", 3)], 3: [("top", 2)]}, {}, 3)
    for p in hp:
        print(p["name"], np.round(p["pos"], 5), np.round(p["rot"], 6))


if __name__ == "__main__":
    main(sys.argv[1])
