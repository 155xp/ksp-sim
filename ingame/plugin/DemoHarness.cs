// DemoHarness: launches one craft on the KSC pad in a COPY of KSP 1.12.5 and lets a kOS script
// direct the flight camera, for recording the in-game booster return demo.
// Installed only into the demo copy (~/ksp-demo/KSP), never the user's main install or saves.
//
// - Main menu: creates sandbox save "demo", goes to the Space Center, launches the craft named in
//   GameData/DemoHarness/launch.txt (consumed, so one launch per game start).
// - Flight: polls <KSP>/Ships/Script/demo/cam.txt (written by kOS) and applies one camera command:
//     chase <hdg> <pitch> <dist> [blendSec] [hdgRate]   camera at a fixed offset in the vessel's
//                                                        local east/north/up frame (deg, deg, m)
//     ground <lat> <lon> <altASL> [blendSec]             camera parked at a geo point, tracking the vessel
//     free                                               hand the camera back to KSP
//     map 1|0                                            enter / leave map view
//     quit                                               quit the game
using System;
using System.Collections;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace DemoHarness
{
    internal static class H
    {
        public static readonly CultureInfo C = CultureInfo.InvariantCulture;
        public static string Root => KSPUtil.ApplicationRootPath;
        public static string Dir => Path.Combine(Root, "GameData/DemoHarness");
        public static string LaunchFile => Path.Combine(Dir, "launch.txt");
        public static string CamFile => Path.Combine(Root, "Ships/Script/demo/cam.txt");
        public static void Log(string s) => Debug.Log("[DemoHarness] " + s);
        public static double D(string s) => double.Parse(s, C);
        public static string LogDir => Path.Combine(Root, "SimHarnessLogs");
        public static string R(double v) => v.ToString("R", C);
        public static string V(Vector3d v) => R(v.x) + "," + R(v.y) + "," + R(v.z);
        public static string V(Vector3 v) => R(v.x) + "," + R(v.y) + "," + R(v.z);
        public static string Q(Quaternion q) => R(q.x) + "," + R(q.y) + "," + R(q.z) + "," + R(q.w);
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class DemoMenu : MonoBehaviour
    {
        IEnumerator Start()
        {
            yield return new WaitForSecondsRealtime(3f);
            if (!File.Exists(H.LaunchFile)) yield break;
            GameSettings.SHOW_PWARP_WARNING = false;
            var pars = GameParameters.GetDefaultParameters(Game.Modes.SANDBOX, GameParameters.Preset.Normal);
            var g = GamePersistence.CreateNewGame("demo", Game.Modes.SANDBOX, pars, "Squad/Flags/default", GameScenes.SPACECENTER, EditorFacility.VAB);
            HighLogic.CurrentGame = g;
            HighLogic.SaveFolder = "demo";
            // kOS: pick PermitAll so its "select a connectivity manager" dialog never shows
            foreach (var la in AssemblyLoader.loadedAssemblies)
            {
                Type[] ts;
                try { ts = la.assembly.GetTypes(); } catch { continue; }
                foreach (var t in ts)
                {
                    if (t.Name != "kOSConnectivityParameters") continue;
                    var p = g.Parameters.CustomParams(t);
                    var f = t.GetField("connectivityHandler");
                    if (p != null && f != null) { f.SetValue(p, "PermitAllConnectivityManager"); H.Log("kOS connectivity -> PermitAll"); }
                }
            }
            GamePersistence.SaveGame("persistent", "demo", SaveMode.OVERWRITE);
            HighLogic.LoadScene(GameScenes.SPACECENTER);
        }
    }

    [KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
    public class DemoKSC : MonoBehaviour
    {
        IEnumerator Start()
        {
            if (!File.Exists(H.LaunchFile) || HighLogic.SaveFolder != "demo") yield break;
            yield return new WaitForSecondsRealtime(3f);
            string name = File.ReadAllText(H.LaunchFile).Trim();
            File.Delete(H.LaunchFile);
            string craft = Path.Combine(H.Root, "Ships/VAB", name);
            H.Log("launching " + craft);
            var node = ConfigNode.Load(craft);
            var manifest = VesselCrewManifest.FromConfigNode(node);
            foreach (var pcm in manifest.PartManifests)
                for (int s = 0; s < pcm.partCrew.Length; s++)
                    if (string.IsNullOrEmpty(pcm.partCrew[s]))
                        pcm.AddCrewToSeat(HighLogic.CurrentGame.CrewRoster.GetNextOrNewKerbal(), s);
            FlightDriver.StartWithNewLaunch(craft, HighLogic.CurrentGame.flagURL, "LaunchPad", manifest);
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class DemoCamera : MonoBehaviour
    {
        string lastCmd = "";
        float nextPoll;
        string mode = "free";
        // chase parameters (current, blended toward target)
        float hdg, pitch, dist, hdgRate;
        float tHdg, tPitch, tDist;
        double gLat, gLon, gAlt;
        // blend from the camera's world offset at command time
        Vector3d blendFrom;
        float blendT0, blendDur;

        void Update()
        {
            if (Time.unscaledTime >= nextPoll)
            {
                nextPoll = Time.unscaledTime + 0.1f;
                Poll();
            }
            Apply();
        }

        void Poll()
        {
            if (!File.Exists(H.CamFile)) return;
            string cmd;
            try { cmd = File.ReadAllText(H.CamFile).Trim(); } catch { return; }
            var lines = cmd.Split('\n');
            cmd = lines[lines.Length - 1].Trim();
            if (cmd.Length == 0 || cmd == lastCmd) return;
            lastCmd = cmd;
            Command(cmd);
        }

        public void Command(string cmd)
        {
            H.Log("cam: " + cmd);
            var f = cmd.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            var cam = FlightCamera.fetch;
            var v = FlightGlobals.ActiveVessel;
            try
            {
                switch (f[0])
                {
                    case "chase":
                        tHdg = (float)H.D(f[1]); tPitch = (float)H.D(f[2]); tDist = (float)H.D(f[3]);
                        blendDur = f.Length > 4 ? (float)H.D(f[4]) : 0f;
                        hdgRate = f.Length > 5 ? (float)H.D(f[5]) : 0f;
                        StartBlend(cam, v);
                        if (mode != "chase" || blendDur <= 0) { hdg = tHdg; pitch = tPitch; dist = tDist; }
                        mode = "chase";
                        break;
                    case "ground":
                        gLat = H.D(f[1]); gLon = H.D(f[2]); gAlt = H.D(f[3]);
                        blendDur = f.Length > 4 ? (float)H.D(f[4]) : 0f;
                        StartBlend(cam, v);
                        mode = "ground";
                        break;
                    case "free":
                        mode = "free";
                        break;
                    case "map":
                        if (f[1] == "1") MapView.EnterMapView(); else MapView.ExitMapView();
                        break;
                    case "quit":
                        H.Log("quit requested");
                        Application.Quit();
                        break;
                }
            }
            catch (Exception e) { H.Log("bad cam command '" + cmd + "': " + e.Message); }
        }

        void StartBlend(FlightCamera cam, Vessel v)
        {
            if (cam == null || v == null) return;
            blendFrom = (Vector3d)cam.transform.position - v.CoMD;
            blendT0 = Time.unscaledTime;
        }

        void Apply()
        {
            if (mode == "free" || MapView.MapIsEnabled) return;
            var v = FlightGlobals.ActiveVessel;
            var cam = FlightCamera.fetch;
            if (v == null || cam == null) return;
            Vector3d com = v.CoMD;
            Vector3d up = (com - v.mainBody.position).normalized;
            Vector3d north = Vector3d.Exclude(up, v.mainBody.transform.up).normalized;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            Vector3d off;
            float dt = Time.unscaledDeltaTime;
            if (mode == "chase")
            {
                float k = Mathf.Clamp01(dt * 0.8f);
                hdg = Mathf.LerpAngle(hdg, tHdg, k); pitch = Mathf.Lerp(pitch, tPitch, k); dist = Mathf.Lerp(dist, tDist, k);
                tHdg += hdgRate * dt; hdg += hdgRate * dt;
                double h = hdg * Mathf.Deg2Rad, p = pitch * Mathf.Deg2Rad;
                off = dist * (Math.Cos(p) * (Math.Cos(h) * north + Math.Sin(h) * east) + Math.Sin(p) * up);
            }
            else
            {
                off = v.mainBody.GetWorldSurfacePosition(gLat, gLon, gAlt) - com;
            }
            if (blendDur > 0)
            {
                float s = Mathf.Clamp01((Time.unscaledTime - blendT0) / blendDur);
                s = s * s * (3 - 2 * s);
                if (s < 1)
                {
                    // blend direction and log-distance separately so the camera swings rather than cuts through the craft
                    double d0 = blendFrom.magnitude, d1 = off.magnitude;
                    Vector3d dir = Vector3d.Slerp(blendFrom.normalized, off.normalized, s);
                    off = dir * Math.Exp(Math.Log(Math.Max(d0, 1)) * (1 - s) + Math.Log(Math.Max(d1, 1)) * s);
                }
            }
            float dd = (float)off.magnitude;
            if (dd > cam.maxDistance) { off *= cam.maxDistance / dd; dd = cam.maxDistance; }
            cam.SetCamCoordsFromPosition((Vector3)(com + off));
            cam.SetDistanceImmediate(dd);
        }
    }
}
