// Shared helpers for the harness tests. AG10 tells the SimHarness logger the test is over.
global g0 is body:mu / body:radius ^ 2.
global base_ralt is alt:radar.
global t_start is time:seconds.

function prep {
  parameter use_legs is true.
  sas off. rcs off.
  lock throttle to 0.
  if use_legs { legs on. wait 6. }
  set base_ralt to alt:radar.
  stage.
  wait 1.
  ag9 on.  // SimHarness starts logging here
  wait 0.5.
}
function settled { return ship:status = "LANDED" and ship:velocity:surface:mag < 0.05. }
function clamp01 { parameter x. return max(0, min(1, x)). }
function thr_for_vs {
  // throttle that tracks a vertical-speed target with a P term on the error
  parameter vs_tgt, kp is 1.5.
  if ship:availablethrust <= 0 { return 0. }
  return clamp01(ship:mass * (g0 + kp * (vs_tgt - ship:verticalspeed)) / ship:availablethrust).
}
function land_from_fall {
  // suicide burn: wait until the stopping distance at 80% of max decel reaches the height, then
  // track a sqrt-profile descent rate down to ~1 m/s at touchdown, engine off on contact
  lock h to alt:radar - base_ralt.
  lock amax to ship:availablethrust / ship:mass - g0.
  wait until ship:verticalspeed < -1 and h < ship:verticalspeed ^ 2 / (2 * amax * 0.8) + 8.
  lock vs_tgt to -min(80, sqrt(max(0, 2 * amax * 0.55 * max(0, h - 0.3)))) - 1.
  lock throttle to thr_for_vs(vs_tgt, 2.0).
  wait until ship:status = "LANDED".
  lock throttle to 0.
}
function finish {
  parameter timeout is 30.
  lock throttle to 0.
  local t1 is time:seconds.
  wait until settled() or time:seconds > t1 + timeout.
  unlock steering.
  wait 3.
  ag10 on.
  wait until false.
}
