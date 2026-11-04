"""Write the in-game RTLS demo craft (stock parts + kOS via ModuleManager patch) as the VAB would place it.

Same placement rules as the fidelity branch harness/make_crafts.py (stack nodes; surface attach
rot = LookRotation(hit.normal, parent.up) * LookRotation(srfAttachNode.orientation, up),
pos = hit.point - rot * srfAttachNode.position, node positions scaled by rescaleFactor).
usage: python -I ingame/make_demo_craft.py <Ships/VAB dir>
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



# name: (node_top_y, node_bottom_y) after rescaleFactor / legacy scale
NODES = {
    "mk1pod_v2": (0.6423756, -0.4050379),
    "fuelTankSmall": (0.55525, -0.55525),          # FL-T200
    "liquidEngine3_v2": (0.0, -0.81781),           # Terrier, Shroud variant
    "Decoupler_1": (0.05, -0.05),                  # TD-12
    "probeStackSmall": (0.1, -0.1),                # RC-001S (+ kOS by patch)
    "batteryBank": (0.1108553, -0.1108553),        # Z-1k
    "advSasModule": (0.1990267, -0.1990267),       # small inline reaction wheel
    "fuelTank_long": (1.875, -1.8875),             # FL-T800
    "fuelTank": (0.981725, -0.9125),               # FL-T400
    "liquidEngine2_v2": (0.0, -1.63),              # Swivel
}
R_125 = 0.625
LT2_NODE_POS = np.array([0.0, 0.4271562, 0.0632596]) * 1.25
LT2_NODE_DIR = np.array([0.0, 0.5735765, 0.819152])
AB_NODE_POS = np.array([0.0, 0.0, -0.025])
AB_NODE_DIR = np.array([0.0, 0.0, 1.0])
UP = np.array([0.0, 1.0, 0.0])


def stack(names, y0=0.0):
    parts, yb = [], None
    for n in names:
        top, bot = NODES[n]
        y = y0 if yb is None else yb - top
        parts.append({"name": n, "pos": np.array([0.0, y, 0.0]), "rot": (0.0, 0.0, 0.0, 1.0)})
        yb = y + bot
    return parts


def surface(name, parent, az_deg, y_off, node_pos, node_dir, radius=R_125):
    a = math.radians(az_deg)
    n = np.array([math.cos(a), 0.0, math.sin(a)])
    hit = parent["pos"] + np.array([0.0, y_off, 0.0]) + n * radius
    rot = q_mul(look_rotation(n, UP), look_rotation(node_dir, UP))
    return {"name": name, "pos": hit - q_rot(rot, node_pos), "rot": rot, "srf": True}


def write(path, ship, parts, parent, stage_of):
    ids = [f"{p['name'].replace('_', '.')}_{4293000000 + i}" for i, p in enumerate(parts)]
    kids = {i: [j for j, p in parent.items() if p == i] for i in range(len(parts))}
    lines = [f"ship = {ship}", "version = 1.12.5", "description = ksp-sim in-game RTLS demo", "type = VAB",
             "size = 3,20,3", "persistentId = 2345678", "rot = 0,0,0,1", "missionFlag = Squad/Flags/default",
             "vesselType = Ship"]
    f = lambda v: ",".join(repr(float(x)) for x in v)
    for i, p in enumerate(parts):
        st = stage_of.get(i, -1)
        lines += ["PART", "{", f"\tpart = {ids[i]}", "\tpartName = Part", "\tpersistentId = " + str(2000 + i),
                  f"\tpos = {f(p['pos'])}", "\tattPos = 0,0,0", f"\tattPos0 = {f(p['pos'])}",
                  f"\trot = {f(p['rot'])}", "\tattRot = 0,0,0,1", f"\tattRot0 = {f(p['rot'])}",
                  "\tmir = 1,1,1", "\tsymMethod = Radial", "\tautostrutMode = Grandparent", "\trigidAttachment = True",
                  f"\tistg = {st}", "\tresPri = 0", "\tdstg = 0", f"\tsidx = {0 if st >= 0 else -1}",
                  "\tsqor = -1", "\tsepI = -1", f"\tattm = {1 if p.get('srf') else 0}", "\tmodCost = 0", "\tmodMass = 0", "\tmodSize = 0,0,0"]
        for c in kids[i]:
            lines.append(f"\tlink = {ids[c]}")
        if p.get("srf"):
            lines.append(f"\tsrfN = srfAttach,{ids[parent[i]]}")
        else:
            if i in parent:
                lines.append(f"\tattN = top,{ids[parent[i]]}")
            for c in kids[i]:
                if not parts[c].get("srf"):
                    lines.append(f"\tattN = bottom,{ids[c]}")
        lines += ["}"]
    open(path, "w").write("\n".join(lines) + "\n")


def main(out):
    names = ["mk1pod_v2", "fuelTankSmall", "liquidEngine3_v2", "Decoupler_1",
             "probeStackSmall", "batteryBank", "advSasModule", "fuelTank_long", "fuelTank", "liquidEngine2_v2"]
    parts = stack(names, y0=10.0)
    parent = {i: i - 1 for i in range(1, len(names))}
    i_t800, i_t400 = names.index("fuelTank_long"), names.index("fuelTank")
    # 4 LT-2 legs on the FL-T400, 0.9 m below its centre (the harness hopper's pad-tested placement)
    for k in range(4):
        parent[len(parts)] = i_t400
        parts.append(surface("landingLeg1-2", parts[i_t400], 45.0 + 90.0 * k, -0.9, LT2_NODE_POS, LT2_NODE_DIR))
    # 4 airbrakes near the top of the FL-T800, between the legs' azimuths (grid-fin stand-ins)
    for k in range(4):
        parent[len(parts)] = i_t800
        parts.append(surface("airbrake1", parts[i_t800], 90.0 * k, 1.45, AB_NODE_POS, AB_NODE_DIR))
    # staging: 2 = booster engine, 1 = decoupler, 0 = upper engine (kOS also drives them directly)
    stage_of = {names.index("liquidEngine2_v2"): 2, names.index("Decoupler_1"): 1, names.index("liquidEngine3_v2"): 0}
    write(f"{out}/RTLSDemo.craft", "RTLS Demo", parts, parent, stage_of)
    for p in parts:
        print(p["name"], np.round(p["pos"], 4), np.round(p["rot"], 4))


if __name__ == "__main__":
    main(sys.argv[1])
