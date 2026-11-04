// Crash series: climb to drop_h above the pad, hover still, cut the engine, free-fall onto the
// pad at ~sqrt(2 g drop_h). The ship name CrashDrop<v> picks the impact speed v (m/s).
runoncepath("0:/harness/lib.ks").
prep().
lock steering to up.
global tgt is base_ralt + drop_h.
lock vs_tgt to max(-3, min(12, (tgt - alt:radar) * 0.8)).
lock throttle to thr_for_vs(vs_tgt, 2.5).
wait until alt:radar > tgt - 0.3.
local t1 is time:seconds.
wait until (abs(ship:verticalspeed) < 0.05 and abs(alt:radar - tgt) < 0.1) or time:seconds > t1 + 20.
lock throttle to 0.
finish(20).
