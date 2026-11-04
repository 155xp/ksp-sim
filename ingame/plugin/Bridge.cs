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
        ModuleReactionWheel rw;
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
                var w = p.FindModuleImplementing<ModuleReactionWheel>();
                if (w != null && rw == null) rw = w;
            }
            H.Log("bridge: engine " + (eng != null ? eng.part.name : "none") + " decoupler " + (decoupler != null) + " upper parts " + upper.Count);
            Cam("ground -0.1087 -74.5617 85 0");
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
            a(rw != null ? V(rw.inputVector) : "0 0 0");
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

        float t0 = -1;
        bool landCam;
        void OnPhase(int ph)
        {
            H.Log("bridge: phase " + phase + " -> " + ph);
            phase = ph;
            switch (ph)
            {
                case 1: Cam("chase 160 10 35 2"); break;           // flip
                case 2: Cam("chase 180 25 150 4 4"); break;        // boostback: wide, slow orbit
                case 3: Cam("chase 200 35 70 4 2"); break;         // coast
                case 4: Cam("chase 270 65 45 4"); break;           // aero descent
            }
        }

        void Update()
        {
            if (!active && !done) return;
            if (v != null && phase == 0 && t0 < 0) t0 = Time.time;
            if (phase == 0 && t0 >= 0 && Time.time - t0 > 12 && Time.time - t0 < 13) Cam("chase 200 -5 45 4");
            if (!landCam && phase >= 3 && v != null && v.altitude < 2600)
            {
                landCam = true;
                Cam("ground -0.1033 -74.5627 80.5 3");
            }
            if (done && Time.realtimeSinceStartup - doneAt > 10f)
            {
                done = false;
                active = false;
                Finish();
            }
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
