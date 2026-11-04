"""Extract the KSC launch-pad geometry from a local KSP 1.12.x install.

Reads (never redistributes) KSP_x64_Data/sharedassets9.assets, which holds the
Kerbin system prefab: Kerbin's PQS, the KSC PQSCity, the KSC facility prefab
("SpaceCenter" nested prefab) and the three LaunchPad upgrade-level prefabs.
MonoBehaviour type trees are stripped in the build, so they are regenerated from
the install's own Managed/*.dll via TypeTreeGeneratorAPI.

Writes ksp_data_local/ksc.json (gitignored).  Every number carries the Unity
object it came from.  The pad lat/lon/alt is then derived the same way the game
does it (PQSCity.Orientate -> transform chain -> CelestialBody.GetLatLonAlt);
see crates/ksp-core/src/ksc.rs for the re-implementation and named constants.

usage (needs: pip install UnityPy TypeTreeGeneratorAPI numpy):
    python -I tools/extract_ksc.py "<KSP>/KSP_x64_Data" ksp_data_local/ksc.json
"""
import json
import math
import sys

import numpy as np
import UnityPy
from UnityPy.helpers.TypeTreeGenerator import TypeTreeGenerator

UNITY_VERSION = "2019.4.18f1"


def main(data_dir: str, out_path: str) -> None:
    data_dir = data_dir.rstrip("/\\") + "/"
    gen = TypeTreeGenerator(UNITY_VERSION)
    gen.load_local_dll_folder(data_dir + "Managed")
    env = UnityPy.load(data_dir + "sharedassets9.assets")
    gg = UnityPy.load(data_dir + "globalgamemanagers.assets")
    scripts = {o.path_id: o.read().m_ClassName for o in gg.objects if o.type.name == "MonoScript"}
    by_id = {o.path_id: o for o in env.objects}

    def comps(go):
        for c in go.m_Component:
            ptr = c.component if hasattr(c, "component") else c[1]
            yield ptr, ptr.deref()

    def transform_of(go):
        for ptr, o in comps(go):
            if o.type.name == "Transform":
                return ptr.path_id, o.read()
        raise KeyError(go.m_Name)

    def mono(go, cls, full=None):
        for ptr, o in comps(go):
            if o.type.name != "MonoBehaviour":
                continue
            raw = o.get_raw_data()
            # MonoBehaviour header: PPtr m_GameObject(12) + u8 m_Enabled (aligned to 4) + PPtr m_Script
            script_pid = int.from_bytes(raw[20:28], "little", signed=True)
            if scripts.get(script_pid) == cls:
                return ptr.path_id, o.read_typetree(gen.get_nodes_up("Assembly-CSharp.dll", full or cls))
        raise KeyError(cls)

    def tdict(pid, t):
        p, r, s = t.m_LocalPosition, t.m_LocalRotation, t.m_LocalScale
        name = t.m_GameObject.deref_parse_as_object().m_Name
        return {"name": name, "transform_path_id": pid, "localPosition": [p.x, p.y, p.z],
                "localRotation_xyzw": [r.x, r.y, r.z, r.w], "localScale": [s.x, s.y, s.z]}

    gos = {}
    for o in env.objects:
        if o.type.name == "GameObject":
            gos.setdefault(o.read().m_Name, []).append(o)

    # Kerbin PQS radius (the GameObject 'Kerbin' that carries a PQS component)
    pqs = None
    for o in gos["Kerbin"]:
        try:
            pid, f = mono(o.read(), "PQS")
            pqs = {"mono_path_id": pid, "radius": f["radius"]}
        except KeyError:
            pass
    # KSC PQSCity
    city = None
    for o in gos["KSC"]:
        try:
            pid, f = mono(o.read(), "PQSCity")
            keys = ["repositionToSphere", "repositionToSphereSurface", "repositionToSphereSurfaceAddHeight",
                    "repositionRadial", "repositionRadiusOffset", "reorientToSphere", "reorientInitialUp",
                    "reorientFinalAngle"]
            city = {"go_path_id": o.path_id, "mono_path_id": pid, **{k: f[k] for k in keys}}
            ksc_city_go = o.read()
        except KeyError:
            pass
    # SpaceCenter nested prefab placeholder under the PQSCity KSC
    _, ksc_t = transform_of(ksc_city_go)
    sc_placeholder = None
    for ch in ksc_t.m_Children:
        t = ch.deref_parse_as_object()
        if t.m_GameObject.deref_parse_as_object().m_Name == "SpaceCenter":
            sc_placeholder = tdict(ch.path_id, t)
            sc_go = t.m_GameObject.deref_parse_as_object()
    _, nps = mono(sc_go, "NestedPrefabSpawner")
    ksc_prefab_go = by_id[nps["Prefabs"][0]["prefab"]["m_PathID"]].read()
    # LaunchPad facility inside the KSC prefab
    _, kp_t = transform_of(ksc_prefab_go)
    lp = fac = None
    for ch in kp_t.m_Children:
        t = ch.deref_parse_as_object()
        if t.m_GameObject.deref_parse_as_object().m_Name == "LaunchPad":
            lp = tdict(ch.path_id, t)
            lp_go = t.m_GameObject.deref_parse_as_object()
    _, uf = mono(lp_go, "UpgradeableFacility", "Upgradeables.UpgradeableFacility")
    fac_t = by_id[uf["facilityTransform"]["m_PathID"]].read()
    fac = tdict(uf["facilityTransform"]["m_PathID"], fac_t)

    levels = []
    for i, lvl in enumerate(uf["upgradeLevels"]):
        prefab_go = by_id[lvl["facilityPrefab"]["m_PathID"]].read()
        root_pid, root_t = transform_of(prefab_go)
        spawn = None
        for ch in root_t.m_Children:
            t = ch.deref_parse_as_object()
            if t.m_GameObject.deref_parse_as_object().m_Name == "LaunchPad_spawn":
                spawn = tdict(ch.path_id, t)
        root = tdict(root_pid, root_t)
        deck_y, deck_name = raycast_down(by_id, root_pid, root_t, spawn["localPosition"])
        levels.append({"level_index": i, "prefab": prefab_go.m_Name, "prefab_root": root,
                       "spawn": spawn, "deck_collider_y_below_spawn": deck_y, "deck_collider": deck_name})

    out = {"_source": "KSP_x64_Data/sharedassets9.assets (+ globalgamemanagers.assets MonoScripts)",
           "unity_version": UNITY_VERSION,
           "kerbin_pqs": pqs, "ksc_pqscity": city, "spacecenter_placeholder": sc_placeholder,
           "ksc_prefab": ksc_prefab_go.m_Name, "launchpad_facility": lp, "facility_transform": fac,
           "launchpad_levels": levels}
    json.dump(out, open(out_path, "w"), indent=1)
    print(json.dumps(out, indent=1))


def _q2m(r):
    x, y, z, w = r.x, r.y, r.z, r.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def raycast_down(by_id, root_pid, root_t, spawn_local):
    """Highest MeshCollider triangle directly below/above the spawn point, in prefab-root-local
    coordinates (the root's own transform is replaced at runtime, see UpgradeableObject)."""
    hits = []

    def walk(t, M):
        go = t.m_GameObject.deref_parse_as_object()
        P, S = t.m_LocalPosition, t.m_LocalScale
        L = np.eye(4)
        L[:3, :3] = _q2m(t.m_LocalRotation) @ np.diag([S.x, S.y, S.z])
        L[:3, 3] = [P.x, P.y, P.z]
        W = M @ L
        for c in go.m_Component:
            ptr = c.component if hasattr(c, "component") else c[1]
            o = ptr.deref()
            if o.type.name == "MeshCollider":
                try:
                    mesh = o.read().m_Mesh.deref_parse_as_object()
                    V, F = _parse_obj(mesh.export())
                except Exception:
                    continue
                Vw = (W @ np.c_[V, np.ones(len(V))].T).T[:, :3]
                y = _ray_y(Vw, F, spawn_local)
                if y is not None:
                    hits.append((y, go.m_Name))
        for ch in t.m_Children:
            walk(ch.deref_parse_as_object(), W)

    # root-local: identity for the root itself, walk only children
    for ch in root_t.m_Children:
        walk(ch.deref_parse_as_object(), np.diag([root_t.m_LocalScale.x, root_t.m_LocalScale.y, root_t.m_LocalScale.z, 1.0]))
    # deck = highest hit not above the spawn point + 1 m
    cand = [h for h in hits if h[0] <= spawn_local[1] + 1e-6]
    if not cand:
        return None, None
    return max(cand)


def _parse_obj(txt):
    V, F = [], []
    for ln in txt.splitlines():
        if ln.startswith("v "):
            a = ln.split()
            V.append([-float(a[1]), float(a[2]), float(a[3])])  # UnityPy OBJ export mirrors X
        elif ln.startswith("f "):
            F.append([int(t.split("/")[0]) - 1 for t in ln.split()[1:4]])
    return np.array(V, dtype=float), np.array(F, dtype=int)


def _ray_y(V, F, p):
    o = np.array([p[0], p[1] + 50.0, p[2]])
    d = np.array([0.0, -1.0, 0.0])
    best = None
    for tri in F:
        a, b, c = V[tri]
        e1, e2 = b - a, c - a
        h = np.cross(d, e2)
        det = e1 @ h
        if abs(det) < 1e-12:
            continue
        f = 1.0 / det
        s = o - a
        u = f * (s @ h)
        if u < 0 or u > 1:
            continue
        q = np.cross(s, e1)
        v = f * (d @ q)
        if v < 0 or u + v > 1:
            continue
        t = f * (e2 @ q)
        if t > 0:
            y = o[1] - t
            if y <= p[1] + 1e-6 and (best is None or y > best):
                best = y
    return best


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
