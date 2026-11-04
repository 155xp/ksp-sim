"""Extract every CelestialBody's serialized physics fields from a local KSP 1.12.x install.

Reads (never redistributes) <KSP>/KSP_Data/sharedassets9.assets (the Kerbol-system prefab,
same file tools/extract_ksc.py reads). Type trees are regenerated from the install's own
Managed/*.dll. Writes ksp_data_local/bodies.json (gitignored); the sim's named constants in
crates/ksp-core/src/body.rs are transcribed from it and cite the field names used here.

usage (pip install UnityPy TypeTreeGeneratorAPI):
    python -I tools/extract_bodies.py "<KSP>/KSP_Data" ksp_data_local/bodies.json
"""
import json
import sys

import UnityPy
from UnityPy.helpers.TypeTreeGenerator import TypeTreeGenerator

UNITY_VERSION = "2019.4.18f1"

SCALARS = [
    "bodyName", "GeeASL", "Radius", "Mass", "gravParameter", "sphereOfInfluence", "hillSphere",
    "atmDensityASL", "atmPressureASL", "ocean", "atmosphere", "atmosphereContainsOxygen",
    "atmosphereDepth", "atmosphereTemperatureSeaLevel", "atmospherePressureSeaLevel",
    "atmosphereMolarMass", "atmosphereAdiabaticIndex", "atmosphereTemperatureLapseRate",
    "atmosphereGasMassLapseRate", "atmosphereUseTemperatureCurve",
    "atmosphereTemperatureCurveIsNormalized", "maxAxialDot", "albedo", "emissivity",
    "coreTemperatureOffset", "convectionMultiplier", "shockTemperatureMultiplier",
    "atmosphereUsePressureCurve", "atmospherePressureCurveIsNormalized", "radiusAtmoFactor",
    "hasSolidSurface", "isStar", "rotates", "rotationPeriod", "solarRotationPeriod",
    "initialRotation", "tidallyLocked", "clampInverseRotThreshold", "inverseRotThresholdAltitude",
    "timeWarpAltitudeLimits", "isHomeWorld", "minOrbitalDistance", "oceanDensity",
]
CURVES = [
    "atmosphereTemperatureCurve", "latitudeTemperatureBiasCurve", "latitudeTemperatureSunMultCurve",
    "axialTemperatureSunMultCurve", "axialTemperatureSunBiasCurve",
    "atmosphereTemperatureSunMultCurve", "eccentricityTemperatureBiasCurve",
    "atmospherePressureCurve",
]
ORBIT = ["inclination", "eccentricity", "semiMajorAxis", "LAN", "argumentOfPeriapsis",
         "meanAnomalyAtEpoch", "epoch"]


def curve(fc):
    ac = fc["fCurve"]
    keys = [{k: kf[k] for k in ("time", "value", "inSlope", "outSlope", "weightedMode",
                                "inWeight", "outWeight") if k in kf} for kf in ac["m_Curve"]]
    return {"keys": keys, "preInfinity": ac.get("m_PreInfinity"), "postInfinity": ac.get("m_PostInfinity"),
            "minTime": fc.get("_minTime"), "maxTime": fc.get("_maxTime")}


def main(data_dir, out_path):
    data_dir = data_dir.rstrip("/\\") + "/"
    gen = TypeTreeGenerator(UNITY_VERSION)
    gen.load_local_dll_folder(data_dir + "Managed")
    gg = UnityPy.load(data_dir + "globalgamemanagers.assets")
    scripts = {o.path_id: o.read().m_ClassName for o in gg.objects if o.type.name == "MonoScript"}
    env = UnityPy.load(data_dir + "sharedassets9.assets")
    nodes_cb = gen.get_nodes_up("Assembly-CSharp.dll", "CelestialBody")
    nodes_od = gen.get_nodes_up("Assembly-CSharp.dll", "OrbitDriver")
    bodies, drivers = {}, {}
    for o in env.objects:
        if o.type.name != "MonoBehaviour":
            continue
        raw = o.get_raw_data()
        cls = scripts.get(int.from_bytes(raw[20:28], "little", signed=True))
        if cls == "CelestialBody":
            f = o.read_typetree(nodes_cb)
            d = {k: f.get(k) for k in SCALARS}
            d.update({k: curve(f[k]) for k in CURVES if k in f})
            d["_mono_path_id"] = o.path_id
            d["_go_path_id"] = f["m_GameObject"]["m_PathID"]
            bodies[d["bodyName"]] = d
        elif cls == "OrbitDriver":
            f = o.read_typetree(nodes_od)
            drivers[f["m_GameObject"]["m_PathID"]] = {
                "orbit": {k: f["orbit"].get(k) for k in ORBIT},
                "referenceBody_path_id": f["orbit"].get("referenceBody", {}).get("m_PathID")
                if isinstance(f["orbit"].get("referenceBody"), dict) else None,
                "_mono_path_id": o.path_id}
    by_mono = {b["_mono_path_id"]: n for n, b in bodies.items()}
    for b in bodies.values():
        od = drivers.get(b["_go_path_id"])
        if od:
            b["orbitDriver"] = od
            b["orbitDriver"]["referenceBody"] = by_mono.get(od["referenceBody_path_id"])
    out = {"_source": "KSP_Data/sharedassets9.assets", "unity_version": UNITY_VERSION, "bodies": bodies}
    json.dump(out, open(out_path, "w"), indent=1, default=float)
    for n, b in bodies.items():
        print(n, b["Radius"], b["gravParameter"], b["GeeASL"], b["atmosphereDepth"],
              b.get("orbitDriver", {}).get("referenceBody"))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
