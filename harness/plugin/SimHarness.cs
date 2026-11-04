// SimHarness: KSP 1.12.5 test-automation + ground-truth logger for ksp-sim.
// Installed only into a COPY of the game (never the user's main install / saves).
//
// - Main menu: creates/loads sandbox save "simharness" and launches the next queued craft on the pad.
// - Flight: dumps exact vessel geometry once (rigidbody mass/CoM/inertia, part transforms, thrust
//   transforms, wheel colliders, drag cubes) and logs one CSV row per physics tick, written at the
//   last FixedUpdate stage, i.e. the state *before* this tick's PhysX step plus the forces/actuators
//   that step will integrate. kOS (boot script per craft) flies the test and turns AG10 on when done.
// - Queue file: GameData/SimHarness/queue.txt, one "craftFile testName maxSeconds" per line;
//   progress kept in GameData/SimHarness/progress.txt. Logs go to <KSP>/SimHarnessLogs/.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace SimHarness
{
    internal static class H
    {
        public static readonly CultureInfo C = CultureInfo.InvariantCulture;
        public static string Root => KSPUtil.ApplicationRootPath;
        public static string Dir => Path.Combine(Root, "GameData/SimHarness");
        public static string LogDir => Path.Combine(Root, "SimHarnessLogs");

        public static string[] Queue()
        {
            var p = Path.Combine(Dir, "queue.txt");
            return File.Exists(p) ? File.ReadAllLines(p).Where(l => l.Trim().Length > 0 && !l.StartsWith("#")).ToArray() : new string[0];
        }
        public static int Progress()
        {
            var p = Path.Combine(Dir, "progress.txt");
            return File.Exists(p) ? int.Parse(File.ReadAllText(p).Trim()) : 0;
        }
        public static void SetProgress(int i) => File.WriteAllText(Path.Combine(Dir, "progress.txt"), i.ToString());
        public static string R(double v) => v.ToString("R", C);
        public static string V(Vector3d v) => R(v.x) + "," + R(v.y) + "," + R(v.z);
        public static string V(Vector3 v) => R(v.x) + "," + R(v.y) + "," + R(v.z);
        public static string Q(Quaternion q) => R(q.x) + "," + R(q.y) + "," + R(q.z) + "," + R(q.w);
        public static void Log(string s) => Debug.Log("[SimHarness] " + s);

        // Launch queue entry i (or quit when the queue is exhausted).
        public static void LaunchNext()
        {
            var q = Queue();
            int i = Progress();
            if (i >= q.Length)
            {
                Log("queue done, quitting");
                File.WriteAllText(Path.Combine(LogDir, "ALL_DONE"), DateTime.Now.ToString("o"));
                Application.Quit();
                return;
            }
            var f = q[i].Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            string craft = Path.Combine(Root, "Ships/VAB", f[0]);
            Log("launching test " + i + ": " + q[i]);
            var node = ConfigNode.Load(craft);
            var manifest = VesselCrewManifest.FromConfigNode(node);
            foreach (var pcm in manifest.PartManifests)
                for (int s = 0; s < pcm.partCrew.Length; s++)
                    if (string.IsNullOrEmpty(pcm.partCrew[s]))
                        pcm.AddCrewToSeat(HighLogic.CurrentGame.CrewRoster.GetNextOrNewKerbal(), s);
            FlightDriver.StartWithNewLaunch(craft, HighLogic.CurrentGame.flagURL, "LaunchPad", manifest);
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class HarnessMenu : MonoBehaviour
    {
        IEnumerator Start()
        {
            Directory.CreateDirectory(H.LogDir);
            yield return new WaitForSecondsRealtime(3f);
            if (H.Queue().Length == 0) yield break;
            var pars = GameParameters.GetDefaultParameters(Game.Modes.SANDBOX, GameParameters.Preset.Normal);
            var g = GamePersistence.CreateNewGame("simharness", Game.Modes.SANDBOX, pars, "Squad/Flags/default", GameScenes.SPACECENTER, EditorFacility.VAB);
            HighLogic.CurrentGame = g;
            HighLogic.SaveFolder = "simharness";
            GamePersistence.SaveGame("persistent", "simharness", SaveMode.OVERWRITE);
            // the launch-pad facility only exists once the Space Center scene has spawned it
            HighLogic.LoadScene(GameScenes.SPACECENTER);
        }
    }

    [KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
    public class HarnessKSC : MonoBehaviour
    {
        IEnumerator Start()
        {
            if (H.Queue().Length == 0 || HighLogic.SaveFolder != "simharness") yield break;
            yield return new WaitForSecondsRealtime(4f);
            H.LaunchNext();
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class HarnessFlight : MonoBehaviour
    {
        StreamWriter w;
        Vessel v;
        string test;
        double maxSec;
        float t0;
        long tick;
        bool dumped, finishing;
        List<Part> parts;
        List<ModuleEngines> engines;
        List<ModuleGimbal> gimbals;
        List<ModuleReactionWheel> wheels;
        List<WheelCollider> wcs;
        List<ModuleWheels.ModuleWheelSuspension> susp;
        List<ModuleWheelBase> wbase;
        static readonly System.Reflection.BindingFlags BF_ALL = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic;
        static object Get(object o, string n)
        {
            if (o == null) return null;
            var t = o.GetType();
            while (t != null)
            {
                var f = t.GetField(n, BF_ALL); if (f != null) return f.GetValue(o);
                var p = t.GetProperty(n, BF_ALL); if (p != null) return p.GetValue(o, null);
                t = t.BaseType;
            }
            return null;
        }
        static string GS(object o, string n)
        {
            var v = Get(o, n);
            if (v == null) return "nan";
            if (v is bool) return (bool)v ? "1" : "0";
            if (v is Vector2) { var q = (Vector2)v; return H.R(q.x) + "," + H.R(q.y); }
            return H.R(Convert.ToDouble(v, H.C));
        }
        List<Part> wcParts;
        TimingManager.UpdateAction act;

        void Start()
        {
            var q = H.Queue();
            int i = H.Progress();
            if (i >= q.Length) { enabled = false; return; }
            var f = q[i].Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            test = f[1];
            maxSec = f.Length > 2 ? double.Parse(f[2], H.C) : 300;
            t0 = Time.realtimeSinceStartup;
            if (test.StartsWith("Survey")) { StartCoroutine(Survey()); return; }
            act = OnTick;
            TimingManager.FixedUpdateAdd(TimingManager.TimingStage.BetterLateThanNever, act);
        }

        bool probeSet;
        Vector3d probeBF;
        bool partDied;
        float diedAt;

        void Awake() { GameEvents.onPartDie.Add(OnPartDie); }
        void OnPartDie(Part p) { if (!partDied) { partDied = true; diedAt = Time.realtimeSinceStartup; H.Log("part died: " + p.partInfo.name); } }

        // Runs while the game is paused (e.g. the Flight Results dialog after a crash), where
        // FixedUpdate does not: real-time timeout, and end the test 3 s after any part died.
        void Update()
        {
            if (finishing || string.IsNullOrEmpty(test)) return;
            if (Time.realtimeSinceStartup - t0 > maxSec) StartCoroutine(Finish(false));
            else if (partDied && Time.realtimeSinceStartup - diedAt > 3f) StartCoroutine(Finish(false, "crashed"));
        }

        // Ground survey: downward Physics.Raycasts on a local East/North grid around the launch pad
        // spawn point against the scenery layer (15: PQS terrain + KSC colliders), recording the hit
        // height above the body radius and the surface normal in the body frame.
        IEnumerator Survey()
        {
            while (FlightGlobals.ActiveVessel == null || FlightGlobals.ActiveVessel.packed || !FlightGlobals.ready) yield return null;
            yield return new WaitForSecondsRealtime(8f);
            var vsl = FlightGlobals.ActiveVessel;
            var b = vsl.mainBody;
            var site = PSystemSetup.Instance.GetSpaceCenterFacility("LaunchPad").GetSpawnPoint("LaunchPad");
            Vector3d padW = site.GetSpawnPointTransform().position;
            Vector3d upW = (padW - b.position).normalized;
            Vector3d eastW = Vector3d.Cross(b.angularVelocity.normalized, upW).normalized;
            if (eastW.magnitude < 0.5) eastW = Vector3d.Cross(Vector3d.up, upW).normalized;
            Vector3d northW = Vector3d.Cross(upW, eastW).normalized;
            int mask = 1 << 15;
            var colInfo = new Dictionary<string, string>();
            foreach (var grid in new[] { new[] { 0.2, 40.0 }, new[] { 4.0, 1500.0 } })
            {
                double stp = grid[0], half = grid[1];
                int n = (int)Math.Round(2 * half / stp) + 1;
                var sb = new StringBuilder();
                sb.AppendLine("# step=" + H.R(stp) + " half=" + H.R(half) + " n=" + n + " pad_bf=" + H.V(BF(b, padW - b.position)) + " east_bf=" + H.V(BF(b, eastW)) + " north_bf=" + H.V(BF(b, northW)) + " up_bf=" + H.V(BF(b, upW)));
                sb.AppendLine("ie,in,e,n,hit,alt,nx,ny,nz,layer,collider");
                for (int ie = 0; ie < n; ie++)
                {
                    for (int jn = 0; jn < n; jn++)
                    {
                        double e = -half + ie * stp, nn = -half + jn * stp;
                        Vector3d top = padW + eastW * e + northW * nn + upW * 300.0;
                        Vector3d dirW = -(top - b.position).normalized;
                        RaycastHit hit;
                        if (Physics.Raycast((Vector3)top, (Vector3)dirW, out hit, 700f, mask))
                        {
                            if (!colInfo.ContainsKey(hit.collider.name))
                            {
                                var arb = hit.collider.attachedRigidbody;
                                var pm = hit.collider.sharedMaterial;
                                colInfo[hit.collider.name] = "layer=" + hit.collider.gameObject.layer + " rb=" + (arb != null) + (arb != null ? " rb.mass=" + H.R(arb.mass) + " kinematic=" + arb.isKinematic : "") +
                                    " material=" + (pm != null ? pm.name + " dyn=" + H.R(pm.dynamicFriction) + " stat=" + H.R(pm.staticFriction) + " bounce=" + H.R(pm.bounciness) + " fc=" + pm.frictionCombine + " bc=" + pm.bounceCombine : "none") +
                                    " PQ=" + (hit.collider.GetComponent<PQ>() != null) + " type=" + hit.collider.GetType().Name;
                            }
                            Vector3d hp = (Vector3d)hit.point - b.position;
                            sb.Append(ie).Append(',').Append(jn).Append(',').Append(H.R(e)).Append(',').Append(H.R(nn)).Append(",1,")
                              .Append(H.R(hp.magnitude - b.Radius)).Append(',').Append(H.V(BF(b, (Vector3d)hit.normal))).Append(',')
                              .Append(hit.collider.gameObject.layer).Append(',').Append(hit.collider.name.Replace(',', ' ')).Append('\n');
                        }
                        else
                            sb.Append(ie).Append(',').Append(jn).Append(',').Append(H.R(e)).Append(',').Append(H.R(nn)).Append(",0,nan,nan,nan,nan,-1,none\n");
                    }
                    if (ie % 50 == 0) yield return null;
                }
                File.WriteAllText(Path.Combine(H.LogDir, test + "_" + H.R(stp) + ".csv"), sb.ToString());
                H.Log("survey grid " + stp + " done");
            }
            var ci = new StringBuilder();
            foreach (var kv in colInfo) ci.AppendLine(kv.Key + " : " + kv.Value);
            // the vessel's own part colliders: physic materials
            foreach (var p in vsl.parts)
                foreach (var col in p.GetComponentsInChildren<Collider>(true))
                {
                    var pm = col.sharedMaterial;
                    ci.AppendLine("PART " + p.partInfo.name + " / " + col.name + " : layer=" + col.gameObject.layer + " material=" + (pm != null ? pm.name + " dyn=" + H.R(pm.dynamicFriction) + " stat=" + H.R(pm.staticFriction) + " bounce=" + H.R(pm.bounciness) + " fc=" + pm.frictionCombine + " bc=" + pm.bounceCombine : "none"));
                }
            ci.AppendLine("Physics: bounceThreshold=" + H.R(Physics.bounceThreshold) + " defaultContactOffset=" + H.R(Physics.defaultContactOffset) + " sleepThreshold=" + H.R(Physics.sleepThreshold) + " maxDepen(root)=" + H.R(vsl.rootPart.rb.maxDepenetrationVelocity) + " solverIter=" + Physics.defaultSolverIterations + "/" + Physics.defaultSolverVelocityIterations);
            for (int a = 0; a < 32; a++)
            {
                var row = new StringBuilder();
                for (int bb = 0; bb < 32; bb++) row.Append(Physics.GetIgnoreLayerCollision(a, bb) ? '.' : 'X');
                ci.AppendLine("layer " + a.ToString("00") + " " + LayerMask.LayerToName(a).PadRight(22) + " " + row);
            }
            File.WriteAllText(Path.Combine(H.LogDir, test + "_colliders.txt"), ci.ToString());
            StartCoroutine(Finish(true));
        }

        void OnDestroy()
        {
            GameEvents.onPartDie.Remove(OnPartDie);
            TimingManager.FixedUpdateRemove(TimingManager.TimingStage.BetterLateThanNever, act);
            if (w != null) { w.Flush(); w.Close(); w = null; }
        }

        static Vector3d BF(CelestialBody b, Vector3d worldVec) => b.BodyFrame.WorldToLocal(worldVec.xzy);

        void Dump(string suffix = "")
        {
            var b = v.mainBody;
            var root = v.rootPart;
            var sb = new StringBuilder();
            sb.AppendLine("test = " + test);
            sb.AppendLine("ut = " + H.R(Planetarium.GetUniversalTime()));
            sb.AppendLine("fixedDeltaTime = " + H.R(Time.fixedDeltaTime));
            sb.AppendLine("maxAngularVelocity_root = " + H.R(root.rb.maxAngularVelocity));
            sb.AppendLine("solverIterations = " + Physics.defaultSolverIterations + "," + Physics.defaultSolverVelocityIterations);
            sb.AppendLine("gravity = " + H.V(Physics.gravity));
            for (int i = 0; i < parts.Count; i++)
            {
                var p = parts[i];
                sb.AppendLine("PART {");
                sb.AppendLine(" idx = " + i);
                sb.AppendLine(" name = " + p.partInfo.name);
                sb.AppendLine(" parent = " + (p.parent == null ? -1 : parts.IndexOf(p.parent)));
                sb.AppendLine(" localPos = " + H.V(root.transform.InverseTransformPoint(p.transform.position)));
                sb.AppendLine(" localRot = " + H.Q(Quaternion.Inverse(root.transform.rotation) * p.transform.rotation));
                sb.AppendLine(" mass = " + H.R(p.mass) + "," + H.R(p.GetResourceMass()) + "," + H.R(p.physicsMass));
                sb.AppendLine(" physicalSignificance = " + p.physicalSignificance);
                sb.AppendLine(" CoMOffset = " + H.V(p.CoMOffset));
                sb.AppendLine(" CoPOffset = " + H.V(p.CoPOffset));
                sb.AppendLine(" CoLOffset = " + H.V(p.CoLOffset));
                sb.AppendLine(" angularDrag = " + H.R(p.angularDrag));
                sb.AppendLine(" crashTolerance = " + H.R(p.crashTolerance));
                if (p.rb != null)
                {
                    sb.AppendLine(" rb.mass = " + H.R(p.rb.mass));
                    sb.AppendLine(" rb.centerOfMass = " + H.V(p.rb.centerOfMass));
                    sb.AppendLine(" rb.worldCoM_rootLocal = " + H.V(root.transform.InverseTransformPoint(p.rb.worldCenterOfMass)));
                    sb.AppendLine(" rb.inertiaTensor = " + H.V(p.rb.inertiaTensor));
                    sb.AppendLine(" rb.inertiaTensorRotation = " + H.Q(p.rb.inertiaTensorRotation));
                    sb.AppendLine(" rb.drag = " + H.R(p.rb.drag) + "," + H.R(p.rb.angularDrag));
                    sb.AppendLine(" rb.maxAngularVelocity = " + H.R(p.rb.maxAngularVelocity));
                    sb.AppendLine(" rb.interpolation = " + p.rb.interpolation + " collisionDetection=" + p.rb.collisionDetectionMode);
                }
                if (p.attachJoint != null && p.attachJoint.Joint != null)
                {
                    var j = p.attachJoint.Joint;
                    sb.AppendLine(" joint = " + j.GetType().Name + " breakForce=" + H.R(j.breakForce) + " xDrive=" + H.R(j.angularXDrive.positionSpring) + "," + H.R(j.angularXDrive.positionDamper) + " yzDrive=" + H.R(j.angularYZDrive.positionSpring) + " linX=" + H.R(j.xDrive.positionSpring) + " motion=" + j.xMotion + "," + j.angularXMotion + " rigid=" + p.rigidAttachment);
                }
                var dc = p.DragCubes;
                if (dc != null && dc.Cubes != null)
                {
                    foreach (var c in dc.Cubes)
                        sb.AppendLine(" cube = " + c.Name + " w=" + H.R(c.Weight) + " area=" + string.Join(",", c.Area.Select(x => H.R(x))) + " drag=" + string.Join(",", c.Drag.Select(x => H.R(x))));
                    sb.AppendLine(" areaOccluded = " + string.Join(",", dc.AreaOccluded.Select(x => H.R(x))));
                    sb.AppendLine(" weightedDrag = " + string.Join(",", dc.WeightedDrag.Select(x => H.R(x))));
                }
                foreach (var col in p.GetComponentsInChildren<Collider>(true))
                {
                    string s = " collider = " + col.GetType().Name + " enabled=" + col.enabled + " trigger=" + col.isTrigger + " layer=" + col.gameObject.layer + " name=" + col.name;
                    var bc = col as BoxCollider; var sc = col as SphereCollider; var cc = col as CapsuleCollider;
                    if (bc != null) s += " center=" + H.V(bc.center) + " size=" + H.V(bc.size);
                    if (sc != null) s += " center=" + H.V(sc.center) + " r=" + H.R(sc.radius);
                    if (cc != null) s += " center=" + H.V(cc.center) + " r=" + H.R(cc.radius) + " h=" + H.R(cc.height) + " dir=" + cc.direction;
                    s += " pos=" + H.V(root.transform.InverseTransformPoint(col.transform.position)) + " rot=" + H.Q(Quaternion.Inverse(root.transform.rotation) * col.transform.rotation) + " lossyScale=" + H.V(col.transform.lossyScale);
                    // extreme points of the collider along the six root axes (ClosestPoint to a far
                    // point; exact for convex colliders), in root-local coordinates
                    try
                    {
                        if (!(col is WheelCollider) && (!(col is MeshCollider) || ((MeshCollider)col).convex))
                        {
                            var c0 = col.bounds.center;
                            foreach (var ax in new[] { -root.transform.up, root.transform.up, root.transform.right, -root.transform.right, root.transform.forward, -root.transform.forward })
                            {
                                var cp = col.ClosestPoint(c0 + ax * 1000f);
                                s += " ext=" + H.V(root.transform.InverseTransformPoint(cp));
                            }
                        }
                    }
                    catch (Exception) { }
                    sb.AppendLine(s);
                }
                foreach (var e in p.FindModulesImplementing<ModuleEngines>())
                    foreach (var t in e.thrustTransforms)
                        sb.AppendLine(" thrustTransform = pos=" + H.V(root.transform.InverseTransformPoint(t.position)) + " fwd=" + H.V(root.transform.InverseTransformDirection(t.forward)) + " maxThrust=" + H.R(e.maxThrust) + " maxFuelFlow=" + H.R(e.maxFuelFlow) + " g=" + H.R(e.g) + " useResponse=" + e.useEngineResponseTime);
                foreach (var g in p.FindModulesImplementing<ModuleGimbal>())
                    for (int k = 0; k < g.gimbalTransforms.Count; k++)
                    {
                        var t = g.gimbalTransforms[k];
                        sb.AppendLine(" gimbalTransform = pos=" + H.V(root.transform.InverseTransformPoint(t.position)) + " rot=" + H.Q(Quaternion.Inverse(root.transform.rotation) * t.rotation) + " initRot=" + H.Q(g.initRots[k]) + " range=" + H.R(g.gimbalRange) + " XP=" + H.R(g.gimbalRangeXP) + " XN=" + H.R(g.gimbalRangeXN) + " YP=" + H.R(g.gimbalRangeYP) + " YN=" + H.R(g.gimbalRangeYN) + " useSpeed=" + g.useGimbalResponseSpeed + " speed=" + H.R(g.gimbalResponseSpeed));
                    }
                foreach (var wc in p.GetComponentsInChildren<WheelCollider>(true))
                    sb.AppendLine(" wheelCollider = pos=" + H.V(root.transform.InverseTransformPoint(wc.transform.position)) + " up=" + H.V(root.transform.InverseTransformDirection(wc.transform.up)) + " center=" + H.V(wc.center) + " radius=" + H.R(wc.radius) + " suspDist=" + H.R(wc.suspensionDistance) + " spring=" + H.R(wc.suspensionSpring.spring) + " damper=" + H.R(wc.suspensionSpring.damper) + " target=" + H.R(wc.suspensionSpring.targetPosition) + " mass=" + H.R(wc.mass) + " forceAppPoint=" + H.R(wc.forceAppPointDistance) + " wheelDamping=" + H.R(wc.wheelDampingRate) + " enabled=" + wc.enabled + " lossyScale=" + H.V(wc.transform.lossyScale));
                sb.AppendLine("}");
            }
            File.WriteAllText(Path.Combine(H.LogDir, test + ".geometry" + suffix + ".cfg"), sb.ToString());
        }

        void Header()
        {
            var cols = new List<string> { "tick", "ut", "dt", "rotframe", "kb_x", "kb_y", "kb_z",
                "com_x", "com_y", "com_z", "vsrf_x", "vsrf_y", "vsrf_z", "vobt_x", "vobt_y", "vobt_z",
                "root_x", "root_y", "root_z", "rootv_x", "rootv_y", "rootv_z",
                "up_x", "up_y", "up_z", "fwd_x", "fwd_y", "fwd_z", "w_x", "w_y", "w_z",
                "mass", "situation", "landed", "nparts", "kbbf_x", "kbbf_y", "kbbf_z",
                "c_thr", "c_pitch", "c_yaw", "c_roll", "c_pitchTrim", "c_yawTrim", "c_rollTrim",
                "lf", "ox", "mono", "ec",
                "p_kpa", "rho", "mach", "q_kpa", "sos", "t_atm", "t_ext", "alt", "ralt", "lat", "lon" };
            for (int k = 0; k < engines.Count; k++)
                cols.AddRange(new[] { "e" + k + "_reqthr", "e" + k + "_thr", "e" + k + "_thrust", "e" + k + "_isp", "e" + k + "_flameout",
                    "e" + k + "_dir_x", "e" + k + "_dir_y", "e" + k + "_dir_z", "e" + k + "_pos_x", "e" + k + "_pos_y", "e" + k + "_pos_z" });
            for (int k = 0; k < gimbals.Count; k++)
                cols.AddRange(new[] { "g" + k + "_ax", "g" + k + "_ay", "g" + k + "_az" });
            for (int k = 0; k < wheels.Count; k++)
                cols.AddRange(new[] { "rw" + k + "_x", "rw" + k + "_y", "rw" + k + "_z", "rw" + k + "_op" });
            for (int k = 0; k < parts.Count; k++)
                cols.AddRange(new[] { "p" + k + "_drag", "p" + k + "_lift", "p" + k + "_dvx", "p" + k + "_dvy", "p" + k + "_dvz", "p" + k + "_spa", "p" + k + "_rx", "p" + k + "_ry", "p" + k + "_rz", "p" + k + "_blx", "p" + k + "_bly", "p" + k + "_blz" });
            for (int k = 0; k < wcs.Count; k++)
                cols.AddRange(new[] { "wc" + k + "_grounded", "wc" + k + "_force", "wc" + k + "_compr", "wc" + k + "_spring", "wc" + k + "_damper", "wc" + k + "_target", "wc" + k + "_sprung",
                    "wc" + k + "_hx", "wc" + k + "_hy", "wc" + k + "_hz", "wc" + k + "_fwdslip", "wc" + k + "_sideslip" });
            for (int k = 0; k < susp.Count; k++)
                foreach (var n in new[] { "stw", "dtw", "ab", "set", "spring", "damper", "smg", "sc", "vp_k", "vp_c", "vp_rtk", "vp_rtc", "cd", "tf_x", "tf_y", "ltf_x", "ltf_y", "lv_x", "lv_y", "angv", "down", "etf_x", "etf_y", "grounded2" })
                    cols.Add("ws" + k + "_" + n);
            // scenery placement probe (see OnTick)
            cols.AddRange(new[] { "probe_alt", "pqs_x", "pqs_y", "pqs_z", "fo_x", "fo_y", "fo_z" });
            w.WriteLine(string.Join(",", cols));
        }

        void OnTick()
        {
            if (finishing) return;
            v = FlightGlobals.ActiveVessel;
            if (v == null || v.packed || !FlightGlobals.ready) return;
            // the kOS script turns AG9 on once its setup (legs deployed, settled) is done
            if (w == null && !v.ActionGroups[KSPActionGroup.Custom09]) return;
            if (w == null)
            {
                parts = v.parts.ToList();
                engines = v.FindPartModulesImplementing<ModuleEngines>();
                gimbals = v.FindPartModulesImplementing<ModuleGimbal>();
                wheels = v.FindPartModulesImplementing<ModuleReactionWheel>();
                susp = v.FindPartModulesImplementing<ModuleWheels.ModuleWheelSuspension>();
                wbase = v.FindPartModulesImplementing<ModuleWheelBase>();
                wcs = new List<WheelCollider>(); wcParts = new List<Part>();
                foreach (var p in parts) foreach (var wc in p.GetComponentsInChildren<WheelCollider>(true)) { wcs.Add(wc); wcParts.Add(p); }
                w = new StreamWriter(Path.Combine(H.LogDir, test + ".csv"), false, new UTF8Encoding(false), 1 << 20);
                Header();
            }
            if (!dumped) { Dump(); dumped = true; }
            var b = v.mainBody;
            var root = v.rootPart;
            var sb = new StringBuilder(4096);
            Action<string> a = s => { sb.Append(s); sb.Append(','); };
            a(tick.ToString()); a(H.R(Planetarium.GetUniversalTime())); a(H.R(TimeWarp.fixedDeltaTime)); a(b.inverseRotation ? "1" : "0");
            a(H.V(Krakensbane.GetFrameVelocity()));
            a(H.V(BF(b, v.CoMD - b.position))); a(H.V(BF(b, v.srf_velocity))); a(H.V(BF(b, v.obt_velocity)));
            a(H.V(BF(b, (Vector3d)root.rb.position - b.position))); a(H.V(BF(b, (Vector3d)root.rb.velocity + Krakensbane.GetFrameVelocity())));
            a(H.V(BF(b, (Vector3d)root.transform.up))); a(H.V(BF(b, (Vector3d)root.transform.forward))); a(H.V(BF(b, (Vector3d)root.rb.angularVelocity)));
            a(H.R(v.totalMass)); a(v.situation.ToString()); a(v.Landed ? "1" : "0"); a(parts.Count(p => p != null && p.vessel == v).ToString());
            // Krakensbane frame velocity in the body frame: when active, FloatingOrigin has already
            // moved the bodies by -FrameVel*dt for the coming step, so com/root above lead by kb*dt
            a(H.V(BF(b, Krakensbane.GetFrameVelocity())));
            var cs = v.ctrlState;
            a(H.R(cs.mainThrottle)); a(H.R(cs.pitch)); a(H.R(cs.yaw)); a(H.R(cs.roll)); a(H.R(cs.pitchTrim)); a(H.R(cs.yawTrim)); a(H.R(cs.rollTrim));
            double lf = 0, ox = 0, mono = 0, ec = 0;
            foreach (var p in parts) if (p != null) foreach (var r in p.Resources)
                    {
                        if (r.resourceName == "LiquidFuel") lf += r.amount; else if (r.resourceName == "Oxidizer") ox += r.amount;
                        else if (r.resourceName == "MonoPropellant") mono += r.amount; else if (r.resourceName == "ElectricCharge") ec += r.amount;
                    }
            a(H.R(lf)); a(H.R(ox)); a(H.R(mono)); a(H.R(ec));
            a(H.R(v.staticPressurekPa)); a(H.R(v.atmDensity)); a(H.R(v.mach)); a(H.R(v.dynamicPressurekPa)); a(H.R(v.speedOfSound));
            a(H.R(v.atmosphericTemperature)); a(H.R(v.externalTemperature)); a(H.R(v.altitude)); a(H.R(v.radarAltitude)); a(H.R(v.latitude)); a(H.R(v.longitude));
            foreach (var e in engines)
            {
                var t = e.thrustTransforms[0];
                a(H.R(e.requestedThrottle)); a(H.R(e.currentThrottle)); a(H.R(e.finalThrust)); a(H.R(e.realIsp)); a(e.flameout ? "1" : "0");
                a(H.V(BF(b, -(Vector3d)t.forward))); a(H.V(BF(b, (Vector3d)t.position - v.CoMD)));
            }
            foreach (var g in gimbals) a(H.V(g.actuation));
            foreach (var rw in wheels) { a(H.V(rw.inputVector)); a(rw.operational ? "1" : "0"); }
            foreach (var p in parts)
            {
                bool ok = p != null && p.vessel == v;
                a(ok ? H.R(p.dragScalar) : "nan"); a(ok ? H.R(p.bodyLiftScalar) : "nan");
                a(ok ? H.V(BF(b, (Vector3d)p.dragVectorDir)) : "nan,nan,nan"); a(ok ? H.R(p.staticPressureAtm) : "nan");
                a(ok ? H.V(root.transform.InverseTransformPoint(p.transform.position)) : "nan,nan,nan");
                // body lift as applied: ProjectOnPlane(rotation * (bodyLiftScalar * bodyLiftLocalVector), -v)
                a(ok ? H.V(BF(b, (Vector3d)Vector3.ProjectOnPlane(p.transform.rotation * (p.bodyLiftScalar * p.bodyLiftLocalVector), -p.dragVectorDir))) : "nan,nan,nan");
            }
            for (int k = 0; k < wcs.Count; k++)
            {
                var wc = wcs[k];
                WheelHit hit = default(WheelHit);
                bool g = wc != null && wc.enabled && wc.GetGroundHit(out hit);
                if (!g) hit = default(WheelHit);
                double compr = double.NaN;
                if (g) compr = Vector3.Dot(wc.transform.position - hit.point, wc.transform.up) - wc.radius;
                a(g ? "1" : "0"); a(H.R(hit.force)); a(H.R(compr));
                a(wc != null ? H.R(wc.suspensionSpring.spring) : "nan"); a(wc != null ? H.R(wc.suspensionSpring.damper) : "nan"); a(wc != null ? H.R(wc.suspensionSpring.targetPosition) : "nan");
                a(wc != null ? H.R(wc.sprungMass) : "nan");
                a(g ? H.V(BF(b, (Vector3d)hit.point - b.position)) : "nan,nan,nan"); a(H.R(hit.forwardSlip)); a(H.R(hit.sidewaysSlip));
            }
            for (int k = 0; k < susp.Count; k++)
            {
                var m = susp[k];
                var wc = (k < wbase.Count) ? Get(wbase[k], "wheel") : null;
                var vp = wc != null ? Get(wc, "wheelCollider") : null;
                var cs2 = wc != null ? Get(wc, "currentState") : null;
                a(GS(m, "springTweakable")); a(GS(m, "damperTweakable")); a(GS(m, "autoBoost")); a(GS(m, "autoSpringDamperSet"));
                a(GS(m, "spring")); a(GS(m, "damper")); a(GS(m, "sprungMassGravity")); a(GS(m, "suspCompression"));
                a(GS(vp, "springRate")); a(GS(vp, "damperRate")); a(GS(vp, "runtimeSpringRate")); a(GS(vp, "runtimeDamperRate"));
                a(GS(cs2, "contactDepth")); a(GS(cs2, "tireForce")); a(GS(cs2, "lastTireForce")); a(GS(cs2, "localWheelVelocity"));
                a(GS(cs2, "angularVelocity")); a(GS(cs2, "downforce")); a(GS(cs2, "externalTireForce")); a(GS(cs2, "grounded"));
            }
            // Scenery placement probe: a downward raycast from a fixed body-fixed point above the pad
            // centre against the scenery layer, i.e. the deck height as the running game currently
            // has it. FloatingOrigin.setOffset moves the bodies (double) and the PQS / KSC float
            // transforms are rebuilt from them, so the collider surface can move by float rounding
            // at ~600 km scale at every origin shift. Also logged: the PQS transform minus the
            // double body position (body frame) and the current floating-origin offset.
            if (!probeSet)
            {
                var site = PSystemSetup.Instance.GetSpaceCenterFacility("LaunchPad").GetSpawnPoint("LaunchPad");
                probeBF = BF(b, (Vector3d)site.GetSpawnPointTransform().position - b.position);
                probeSet = true;
            }
            Vector3d pw = b.position + b.BodyFrame.LocalToWorld(probeBF).xzy;
            Vector3d puw = (pw - b.position).normalized;
            RaycastHit ph;
            bool pok = Physics.Raycast((Vector3)(pw + puw * 20.0), (Vector3)(-puw), out ph, 60f, 1 << 15);
            a(pok ? H.R(((Vector3d)ph.point - b.position).magnitude - b.Radius) : "nan");
            a(b.pqsController != null ? H.V(BF(b, (Vector3d)b.pqsController.transform.position - b.position)) : "nan,nan,nan");
            a(H.V(BF(b, FloatingOrigin.Offset)));
            sb.Length -= 1;
            w.WriteLine(sb.ToString());
            tick++;
            bool done = v.ActionGroups[KSPActionGroup.Custom10];
            if (done) StartCoroutine(Finish(true));
        }

        IEnumerator Finish(bool ok, string why = "timeout")
        {
            finishing = true;
            if (w != null) { w.Flush(); w.Close(); w = null; }
            try { if (v != null && parts != null) Dump(".end"); } catch (Exception e) { H.Log("end dump failed: " + e); }
            File.WriteAllText(Path.Combine(H.LogDir, test + ".status"), ok ? (partDied ? "ok_partdied" : "ok") : why);
            H.SetProgress(H.Progress() + 1);
            yield return new WaitForSecondsRealtime(1f);
            if (H.Progress() >= H.Queue().Length) H.LaunchNext();  // quits
            else HighLogic.LoadScene(GameScenes.SPACECENTER);      // HarnessKSC launches the next one
        }
    }
}
