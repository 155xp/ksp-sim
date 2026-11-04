// Ascent: full throttle, gentle gravity turn east, MECO at apoapsis 80 km, coast through apoapsis
// (vacuum, drift compensation) and back down to 65 km.
runoncepath("0:/harness/lib.ks").
prep(false).
lock pitch_cmd to 90 - min(45, max(0, (altitude - 1500) / 40000 * 45)).
lock steering to heading(90, pitch_cmd).
lock throttle to 1.
wait until ship:apoapsis > 80000.
lock throttle to 0.
wait until altitude > 70000.
wait until ship:verticalspeed < 0 and altitude < 65000.
unlock steering.
ag10 on.
wait until false.
