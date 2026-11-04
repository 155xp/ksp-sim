// Precision pad landing: climb ~150 m, then descend under closed-loop guidance back to the
// launch point (horizontal velocity damping + position feedback), soft touchdown, engine off.
runoncepath("0:/harness/lib.ks").
prep().
global pad is ship:geoposition.
lock steering to heading(90, 88).
lock throttle to 0.9.
wait 4.
lock throttle to 0.
lock steering to up.
wait until ship:verticalspeed < 0.
lock hvel to vxcl(up:vector, ship:velocity:surface).
lock hoff to vxcl(up:vector, pad:position).
lock acc_cmd to up:vector * g0 + (hoff * 0.08 - hvel * 0.5).
lock steering to lookdirup(acc_cmd, ship:facing:topvector).
lock h to alt:radar - base_ralt.
lock amax to ship:availablethrust / ship:mass - g0.
wait until h < ship:verticalspeed ^ 2 / (2 * amax * 0.5) + 30.
lock vs_tgt to -max(1.2, min(25, sqrt(max(0, 2 * amax * 0.35 * max(0, h - 0.5))))).
lock throttle to thr_for_vs(vs_tgt, 2.0).
wait until ship:status = "LANDED".
lock throttle to 0.
unlock steering.
finish(30).
