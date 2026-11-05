// DemoBridge: flies the active vessel with the ksp-sim guidance (guidance/core/examples/bridge.rs)
// over a localhost socket, tick-synchronously. In OnFlyByWire (FlightInputHandler.FixedUpdate ->
// Vessel.FeedInputFeed, before the part modules read ctrlState this tick) it sends the vessel's
// navigation state in the sim's body frame (SimHarness conventions: BodyFrame.WorldToLocal(v.xzy))
// and applies the reply (throttle, pitch, yaw, roll, brakes, decouple, done) on the same tick.
// The guidance phase also drives the camera director. TraceLogger writes the full SimHarness trace.
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace DemoHarness
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class DemoBridge : MonoBehaviour
    {
        public static bool Logging;
        const int Port = 7777;
        TcpClient client;
        NetworkStream ns;
        StreamReader rd;
        Vessel v;
        Part decoupler;
        HashSet<Part> upper = new HashSet<Part>();
        ModuleEngines eng;
        ModuleGimbal gim;
        List<ModuleReactionWheel> rws = new List<ModuleReactionWheel>();
        DemoCamera cam;
        TraceLogger logger;
        bool active, engineOn, decoupled, done;
        int phase = -1;
        float doneAt;
        long tick;
        StreamWriter blog;
        string runName;

        IEnumerator Start()
        {
            Logging = false;
            if (HighLogic.SaveFolder != "demo") yield break;
            while (FlightGlobals.ActiveVessel == null || FlightGlobals.ActiveVessel.packed || !FlightGlobals.ready) yield return null;
            v = FlightGlobals.ActiveVessel;
            cam = FindObjectOfType<DemoCamera>();
            runName = "flight_" + DateTime.Now.ToString("yyyyMMdd_HHmmss");
            Directory.CreateDirectory(H.LogDir);
            // booster engine = the one with a gimbal; the decoupler and everything above it is the upper stage
            foreach (var p in v.parts)
            {
                var d = p.FindModuleImplementing<ModuleDecouple>();
                if (d != null && decoupler == null) decoupler = p;
            }
            // ModuleDecouple releases the part on its top node: the decoupler stays with the booster
            if (decoupler != null) foreach (var c in decoupler.children) Collect(c);
            foreach (var p in v.parts)
            {
                if (upper.Contains(p)) continue;
                var e = p.FindModuleImplementing<ModuleEngines>();
                if (e != null && eng == null) { eng = e; gim = p.FindModuleImplementing<ModuleGimbal>(); }
                rws.AddRange(p.FindModulesImplementing<ModuleReactionWheel>());
            }
            H.Log("bridge: engine " + (eng != null ? eng.part.name : "none") + " decoupler " + (decoupler != null) + " upper parts " + upper.Count);
            // legs down (the sim's legs are always deployed), settle on the pad
            v.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
            v.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            yield return new WaitForSeconds(6f);
            // connect to the guidance server
            for (int i = 0; i < 60 && client == null; i++)
            {
                try { client = new TcpClient("127.0.0.1", Port); client.NoDelay = true; }
                catch (Exception e) { H.Log("bridge: connect failed (" + e.Message + "), retrying"); client = null; }
                if (client == null) yield return new WaitForSecondsRealtime(1f);
            }
            if (client == null) { H.Log("bridge: no guidance server, giving up"); yield break; }
            ns = client.GetStream();
            ns.ReadTimeout = 20000;
            rd = new StreamReader(ns, Encoding.ASCII);
            blog = new StreamWriter(Path.Combine(H.LogDir, runName + ".bridge.csv"), false, new UTF8Encoding(false), 1 << 16);
            blog.WriteLine("tick,ut,state,reply");
            logger = gameObject.AddComponent<TraceLogger>();
            logger.testName = runName;
            Logging = true;
            v.OnFlyByWire += Fly;
            active = true;
            // clean frame: hide the KSP UI (what F2 does)
            GameEvents.onHideUI.Fire();
            H.Log("bridge: flying " + runName);
        }

        void Collect(Part p)
        {
            upper.Add(p);
            foreach (var c in p.children) Collect(c);
        }

        void Cam(string c) { if (cam != null) cam.Command(c); }

        static Vector3d BF(CelestialBody b, Vector3d worldVec) => b.BodyFrame.WorldToLocal(worldVec.xzy);

        string State()
        {
            var b = v.mainBody;
            var root = v.rootPart;
            // vessel CoM / velocity straight from the rigidbodies (fresh after the last PhysX step)
            double m = 0; Vector3d com = Vector3d.zero, vel = Vector3d.zero;
            foreach (var p in v.parts)
            {
                if (p.rb == null) continue;
                double pm = p.rb.mass;
                m += pm; com += (Vector3d)p.rb.worldCenterOfMass * pm; vel += (Vector3d)p.rb.velocity * pm;
            }
            com /= m; vel /= m;
            vel += Krakensbane.GetFrameVelocity();
            double lf = 0, ox = 0, mono = 0, ec = 0;
            foreach (var p in v.parts)
            {
                if (upper.Contains(p)) continue;
                foreach (var r in p.Resources)
                {
                    if (r.resourceName == "LiquidFuel") lf += r.amount; else if (r.resourceName == "Oxidizer") ox += r.amount;
                    else if (r.resourceName == "MonoPropellant") mono += r.amount; else if (r.resourceName == "ElectricCharge") ec += r.amount;
                }
            }
            var sb = new StringBuilder(512);
            Action<string> a = s => { sb.Append(s); sb.Append(' '); };
            a(H.R(Planetarium.GetUniversalTime())); a(b.inverseRotation ? "1" : "0");
            a(V(BF(b, com - b.position))); a(V(BF(b, vel)));
            a(V(BF(b, (Vector3d)root.transform.up))); a(V(BF(b, (Vector3d)root.transform.forward))); a(V(BF(b, (Vector3d)root.rb.angularVelocity)));
            a(H.R(lf)); a(H.R(ox)); a(H.R(mono)); a(H.R(ec));
            a(H.R(eng != null ? eng.currentThrottle : 0));
            a(gim != null ? H.R(gim.actuation.x) + " " + H.R(gim.actuation.z) : "0 0");
            // the sim lumps all wheels into one (torque = sum): send the summed input vectors
            Vector3 rwsum = Vector3.zero;
            foreach (var w in rws) if (w.part.vessel == v) rwsum += w.inputVector;
            a(V(rwsum));
            a(v.Landed ? "1" : "0");
            a(V(BF(b, Krakensbane.GetFrameVelocity())));
            return sb.ToString().TrimEnd();
        }

        static string V(Vector3d x) => H.R(x.x) + " " + H.R(x.y) + " " + H.R(x.z);
        static string V(Vector3 x) => H.R(x.x) + " " + H.R(x.y) + " " + H.R(x.z);

        void Fly(FlightCtrlState s)
        {
            if (!active || v == null || v.packed) return;
            string st = State(), reply;
            try
            {
                var bytes = Encoding.ASCII.GetBytes("S " + st + "\n");
                ns.Write(bytes, 0, bytes.Length);
                reply = rd.ReadLine();
            }
            catch (Exception e)
            {
                H.Log("bridge: socket error " + e.Message + ", cutting throttle");
                active = false;
                s.mainThrottle = 0;
                return;
            }
            tick++;
            if (blog != null) blog.WriteLine(tick + "," + H.R(Planetarium.GetUniversalTime()) + "," + st + "," + reply);
            // reply: C thr pitch yaw roll brakes decouple phase done
            var f = reply.Split(' ');
            if (f.Length < 9 || f[0] != "C") { H.Log("bridge: bad reply " + reply); return; }
            float thr = (float)H.D(f[1]);
            s.mainThrottle = thr;
            s.pitch = (float)H.D(f[2]); s.yaw = (float)H.D(f[3]); s.roll = (float)H.D(f[4]);
            s.pitchTrim = 0; s.yawTrim = 0; s.rollTrim = 0;
            s.X = 0; s.Y = 0; s.Z = 0;
            bool brakes = f[5] == "1";
            if (v.ActionGroups[KSPActionGroup.Brakes] != brakes) v.ActionGroups.SetGroup(KSPActionGroup.Brakes, brakes);
            if (!engineOn && thr > 0 && eng != null) { eng.Activate(); engineOn = true; }
            if (!decoupled && f[6] == "1" && decoupler != null)
            {
                decoupled = true;
                var d = decoupler.FindModuleImplementing<ModuleDecouple>();
                if (d != null) d.Decouple();
                H.Log("bridge: decoupled");
            }
            int ph = int.Parse(f[7]);
            if (ph != phase) OnPhase(ph);
            if (!done && f[8] == "1") { done = true; doneAt = Time.realtimeSinceStartup; H.Log("bridge: guidance done " + reply); }
        }

        // ---- camera director: phase + height driven, cinematic defaults ----------------------------
        // ground positions (lat, lon, alt ASL) around the pad (deck 72.45 m ASL)
        const string CamLiftoff = "ground -0.1155 -74.5637 84 0";   // ~190 m S, 65 m W, low
        const string CamApproach = "ground -0.0492 -74.6057 +3 0";  // ~700 m NW of the pad, 3 m above the terrain: side-on telephoto
        const string CamPad = "ground -0.1003 -74.5600 78 0";       // ~33 m S, 25 m W, deck level
        int stage = -1;
        int boostbacks;
        void Director()
        {
            if (v == null) return;
            double h = v.altitude - 72.45;
            int want = stage;
            if (phase == 0) want = h < 2500 ? 0 : 1;
            else if (phase == 1 || (phase == 2 && boostbacks <= 1)) want = 2;
            else if (phase >= 3 || phase == 2) want = 3;
            if (phase >= 3 || (phase == 2 && boostbacks > 1)) { if (h < 1500 && v.verticalSpeed < 0) want = 4; if (h < 120) want = 5; }
            if (stage == 5) want = 5;
            if (want == stage) return;
            if (want < stage && stage >= 4) return;
            stage = want;
            switch (stage)
            {
                case 0: Cam(CamLiftoff); Cam("zoom 90"); break;                    // tracking telephoto from the ground
                case 1: Cam("fov 55"); Cam("chase 205 -12 130 4"); break;          // chase from below-side, plume in frame
                case 2: Cam("fov 50"); Cam("chase 165 8 95 4 1.5"); break;         // side view: flip and boostback burn
                case 3: Cam("fov 50"); Cam("chase 250 50 90 5"); break;             // over the shoulder, ground below
                case 4: Cam(CamApproach); Cam("zoom 70"); break;                   // cut: telephoto from the ground
                case 5: Cam(CamPad); Cam("zoom 70"); break;                        // cut: close pad shot for touchdown
            }
            H.Log("bridge: camera stage " + stage + " at h " + h.ToString("F0"));
        }

        void OnPhase(int ph)
        {
            H.Log("bridge: phase " + phase + " -> " + ph);
            phase = ph;
            if (ph == 2) boostbacks++;
        }

        void Update()
        {
            if (active || done) Director();
            if (done && Time.realtimeSinceStartup - doneAt > 8f)
            {
                done = false;
                active = false;
                Finish();
            }
        }

        // minimal overlay: altitude above the pad and speed, small, lower left
        GUIStyle style, shadow;
        void OnGUI()
        {
            if (v == null || (!active && !done) || phase < 0) return;
            if (style == null)
            {
                style = new GUIStyle(GUI.skin.label) { fontSize = Mathf.RoundToInt(Screen.height / 45f), fontStyle = FontStyle.Normal };
                style.normal.textColor = new Color(1f, 1f, 1f, 0.92f);
                shadow = new GUIStyle(style);
                shadow.normal.textColor = new Color(0f, 0f, 0f, 0.55f);
            }
            double h = Math.Max(0, v.altitude - 72.45 - 5.0);
            string alt = h >= 1000 ? (h / 1000).ToString("F1") + " km" : h.ToString("F0") + " m";
            string txt = "ALT  " + alt + "     SPD  " + v.srfSpeed.ToString("F0") + " m/s";
            float x = Screen.width * 0.03f, y = Screen.height * 0.93f;
            var r = new Rect(x, y, Screen.width * 0.5f, style.fontSize * 2f);
            GUI.Label(new Rect(r.x + 1.5f, r.y + 1.5f, r.width, r.height), txt, shadow);
            GUI.Label(r, txt, style);
        }

        void Finish()
        {
            Logging = false;
            if (logger != null) logger.Close();
            if (blog != null) { blog.Flush(); blog.Close(); blog = null; }
            try { var bytes = Encoding.ASCII.GetBytes("Q\n"); ns.Write(bytes, 0, bytes.Length); } catch { }
            H.Log("bridge: finished, quitting");
            Application.Quit();
        }

        void OnDestroy()
        {
            if (v != null) v.OnFlyByWire -= Fly;
            if (blog != null) { blog.Flush(); blog.Close(); }
            if (client != null) client.Close();
        }
    }
}
