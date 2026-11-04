// Powered hover: climb to 30 m, hold for 30 s, descend at ~1.5 m/s and land.
runoncepath("0:/harness/lib.ks").
prep().
lock steering to up.
global tgt is base_ralt + 30.
lock vs_tgt to max(-5, min(5, (tgt - alt:radar) * 0.5)).
lock throttle to thr_for_vs(vs_tgt).
wait until alt:radar > tgt - 1.
local t1 is time:seconds.
wait until time:seconds > t1 + 30.
lock vs_tgt to max(-4, min(0, (base_ralt - alt:radar) * 0.3)) - 1.
wait until ship:status = "LANDED".
lock throttle to 0.
finish(30).
