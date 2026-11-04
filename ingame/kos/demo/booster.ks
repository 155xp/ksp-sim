@clobberbuiltins on.
// In-game booster return to the KSC pad (kOS port of the guidance branch's laws, simplified):
// ascent with a fixed pitch program, MECO on remaining booster propellant, separation, flip,
// boostback (Newton-free: burn along the predicted touchdown error, dead-reckoned between
// predictions), coast, angle-of-attack steering in the air, hoverslam with ZEM/ZEV divert and a
// short constant-rate terminal segment.
//
// Predictor: RK2 in Kerbin's rotating frame (gravity, Coriolis, centrifugal, drag
// kd * mach_mult(M) * rho * v^2 with kd learned online from the measured deceleration).
// It writes camera commands for the DemoHarness plugin to 0:/demo/cam.txt.

runoncepath("0:/demo/params.ks").

global PAD is latlng(-0.0972092543643722, -74.557706433623).
global R0 is body:radius.
global MU is body:mu.
global G0 is MU / R0 ^ 2.
global Wv is body:angularvel.
global LOGF is "0:/demo/flight.csv".
global RESF is "0:/demo/result.txt".
global phase is "pad".
global thr is 0.
global steer is ship:facing.
global t_last_log is 0.
global ALT0 is ship:altitude.

function cam { parameter c. log c to "0:/demo/cam.txt". }
function say { parameter s, d is 4. hudtext(s, d, 2, 28, white, false). }
function clamp { parameter x, a, b. return max(a, min(b, x)). }
function booster_prop {
  local m is 0.
  for p in ship:parts {
    if p:name = "fuelTank.long" or p:name = "fuelTank" {
      for r in p:resources { if r:name = "LiquidFuel" or r:name = "Oxidizer" set m to m + r:amount * r:density. }
    }
  }
  return m.
}
function set_warp { parameter n.
  if not WARP_OK set n to 0.
  if kuniverse:timewarp:mode <> "PHYSICS" set kuniverse:timewarp:mode to "PHYSICS".
  if kuniverse:timewarp:warp <> n set kuniverse:timewarp:warp to n.
}

// ---- atmosphere tables (500 m) -------------------------------------------------------------
global RHO is list().
global CS is list().
{
  local mm is body:atm:molarmass.
  local gam is body:atm:adiabaticindex.
  from { local i is 0. } until i > 142 step { set i to i + 1. } do {
    local hh is i * 500.
    local p is body:atm:altitudepressure(hh) * constant:atmtokpa * 1000.
    local tk is max(1, body:atm:alttemp(hh)).
    RHO:add(p * mm / (8.31446 * tk)).
    CS:add(sqrt(gam * 8.31446 * tk / mm)).
  }
}
function mmult { parameter m.
  if m < 0.85 return 0.5.
  if m < 1.1 return 0.5 + (m - 0.85) * 3.2.
  if m < 2 return 1.3 - (m - 1.1) * 0.6667.
  if m < 5 return 0.7 - (m - 2) * 0.0333.
  return 0.6.
}
function atm_at { parameter h.
  local x is clamp(h, 0, 70000) / 500.
  local i is floor(x).
  local f is x - i.
  return list(RHO[i] + (RHO[i + 1] - RHO[i]) * f, CS[i] + (CS[i + 1] - CS[i]) * f).
}

// ---- predictor ------------------------------------------------------------------------------
global kd is KD_PRIOR.
function acc { parameter r, v.
  local rm is r:mag.
  local a is r * (-MU / rm ^ 3) - 2 * vcrs(Wv, v) - vcrs(Wv, vcrs(Wv, r)).
  local h is rm - R0.
  if h < 69000 {
    local x is max(0, h) / 500.
    local i is floor(x).
    local f is x - i.
    local rho is RHO[i] + (RHO[i + 1] - RHO[i]) * f.
    local c is CS[i] + (CS[i + 1] - CS[i]) * f.
    local sp is v:mag.
    set a to a - v * (kd * mmult(sp / c) * rho * sp).
  }
  return a.
}
// returns list(touchdown position from body centre, time of flight, velocity there)
function predict { parameter tgt_alt.
  local r is ship:position - body:position.
  local v is ship:velocity:surface.
  local t is 0.
  local h is r:mag - R0.
  if h <= tgt_alt return list(r, 0, v).
  local rp is r.
  local hp is h.
  local dt is 1.
  until h < tgt_alt or t > 900 {
    set dt to choose 2 if h > 25000 else (choose 1 if h > 6000 else 0.5).
    local a1 is acc(r, v).
    local vm is v + a1 * (dt / 2).
    local a2 is acc(r + v * (dt / 2), vm).
    set rp to r.
    set hp to h.
    set r to r + vm * dt.
    set v to v + a2 * dt.
    set t to t + dt.
    set h to r:mag - R0.
  }
  local f is (hp - tgt_alt) / max(0.001, hp - h).
  local ri is rp + (r - rp) * f.
  set pred_geo to body:geopositionof(ri + body:position).
  set pred_ut to time:seconds + t - dt * (1 - f).
  return list(ri, t - dt * (1 - f), v).
}
global AIM_OFF is v(0, 0, 0).
function pad_r { return PAD:altitudeposition(ALT0) - body:position. }
// horizontal (pad-plane) vector from the aim point to the predicted touchdown point
function pad_err { parameter ri.
  local rp is pad_r().
  return vxcl(rp:normalized, ri - rp - AIM_OFF).
}

// ---- drag learning --------------------------------------------------------------------------
global lv is ship:velocity:surface.
global lt is time:seconds.
global kd_n is 0.
function learn {
  local t is time:seconds.
  local vv is ship:velocity:surface.
  local dt is t - lt.
  if dt < 0.1 return.
  if dt < 1 and thr < 0.001 and ship:altitude < 65000 {
    local r is ship:position - body:position.
    local vm is (vv + lv) / 2.
    local am is (vv - lv) / dt.
    local a0 is r * (-MU / r:mag ^ 3) - 2 * vcrs(Wv, vm) - vcrs(Wv, vcrs(Wv, r)).
    local ad is -vdot(am - a0, vm:normalized).
    local at is atm_at(ship:altitude).
    local sp is vm:mag.
    local den is mmult(sp / at[1]) * at[0] * sp ^ 2.
    if den * kd > 0.4 and ad > 0 {
      set kd to kd + 0.25 * (ad / den - kd).
      set kd_n to kd_n + 1.
    }
  }
  set lt to t.
  set lv to vv.
}

// ---- logging --------------------------------------------------------------------------------
global pred_geo is PAD.
global pred_ut is 0.
global ENG is 0.
function enu { parameter vec, nd.
  local u is up:vector.
  local n is north:vector.
  local e is vcrs(u, n).
  return round(vdot(vec, e), nd) + "," + round(vdot(vec, n), nd) + "," + round(vdot(vec, u), nd).
}
function eng_thrust { if ENG = 0 return 0. return ENG:thrust. }
global last_err is v(0, 0, 0).
global last_tof is 0.
function logrow {
  if time:seconds - t_last_log < 0.25 return.
  set t_last_log to time:seconds.
  local up_ is up:vector.
  local hv is vxcl(up_, ship:velocity:surface).
  global miss is vxcl(up_, PAD:position):mag.
  log round(time:seconds - T0, 2) + "," + phase + "," + round(ship:altitude, 1) + "," + round(ship:altitude - ALT0, 2) + ","
    + round(ship:verticalspeed, 2) + "," + round(hv:mag, 2) + "," + round(last_err:mag, 1) + "," + round(miss, 2) + ","
    + round(thr, 3) + "," + round(booster_prop(), 3) + "," + round(kd * 1e6, 2) + "," + round(last_tof, 1) + ","
    + round(ship:mass, 3) + "," + round(vang(ship:facing:vector, up_), 2) + "," + round(ship:q * constant:atmtokpa, 2) + ","
    + kuniverse:timewarp:rate + "," + round(ship:geoposition:lat, 7) + "," + round(ship:geoposition:lng, 7) + ","
    + enu(ship:velocity:surface, 3) + "," + enu(ship:facing:vector, 5) + "," + round(eng_thrust(), 2) + "," + brakes + ","
    + round(pred_geo:lat, 7) + "," + round(pred_geo:lng, 7) + "," + round(pred_ut, 2) + "," + round(time:seconds, 2) to LOGF.
}

// ================================ flight =====================================================
if exists(LOGF) deletepath(LOGF).
if exists(RESF) deletepath(RESF).
log "t,phase,alt,h,vz,vh,err,miss,thr,prop,kd_e6,tof,mass,tilt,q_kpa,warp,lat,lng,ve,vn,vu,fe,fn,fu,thrust_kn,brakes,pred_lat,pred_lng,pred_t,ut" to LOGF.
global T0 is time:seconds.
sas off.
rcs off.
brakes off.
lock throttle to thr.
cam(CAM_PAD).
legs on.
wait 5.
set ALT0 to ship:altitude.
// sign of the body rotation vector in kOS's raw frame: w x r must be the frame's velocity
{
  local r is ship:position - body:position.
  local vf is ship:velocity:orbit - ship:velocity:surface.
  if vdot(vcrs(Wv, r), vf) < 0 set Wv to -Wv.
  log "# ALT0 " + ALT0 + " wsign_check " + round(vdot(vcrs(Wv, r), vf) / max(0.001, vf:sqrmagnitude), 4) + " pad_miss0 " + vxcl(up:vector, PAD:position):mag to LOGF.
}
set ENG to ship:partsnamedpattern("liquidEngine2")[0].
set steer to lookdirup(up:vector, ship:facing:topvector).
lock steering to steer.
say("RTLS demo: liftoff", 3).
wait 1.

// ---- ascent ---------------------------------------------------------------------------------
set phase to "ascent".
set T0 to time:seconds.
ENG:activate().
set thr to 1.
global legs_up is false.
until booster_prop() <= MECO_PROP {
  local hh is ship:altitude - ALT0.
  local p is 90.
  if ship:velocity:surface:mag > PITCH_V0 {
    set p to 90 - (90 - PITCH_END) * min(1, max(0, hh - PITCH_H0) / PITCH_H1) ^ 0.7.
  }
  set steer to lookdirup(heading(90, p):vector, ship:facing:topvector).
  if not legs_up and hh > 120 { legs off. set legs_up to true. }
  if hh > 900 and hh < 1100 cam(CAM_ASCENT).
  if hh > WARP_ASCENT_H set_warp(1).
  logrow().
  wait 0.
}

// ---- MECO, separation ------------------------------------------------------------------------
set thr to 0.
set_warp(0).
set phase to "meco".
say("MECO", 2).
cam(CAM_SEP).
wait 1.
set steer to ship:facing.
for d in ship:partsnamedpattern("Decoupler") { d:getmodule("ModuleDecouple"):doevent("decouple"). }
wait 0.1.
kuniverse:forcesetactivevessel(ship).
say("Stage separation", 3).
set phase to "sep".
global tsep is time:seconds.
// coast up until the air is thin enough for the wheel + gimbal to hold a broadside attitude
until time:seconds > tsep + 2.5 and (ship:q * constant:atmtokpa < FLIP_Q or time:seconds > tsep + 30) { logrow(). wait 0. }

// ---- flip + boostback -----------------------------------------------------------------------
set phase to "flip".
global pr is predict(ALT0).
global PADUP is pad_r():normalized.
// aim point: AIM_LONG beyond the pad along the return direction
set AIM_OFF to vxcl(PADUP, PAD:position):normalized * AIM_LONG.
global e0 is pad_err(pr[0]).
global tof is pr[1].
global vref is ship:velocity:surface.
global einit is e0.
set last_err to e0. set last_tof to tof.
lock e_dr to e0 + vxcl(PADUP, ship:velocity:surface - vref) * tof.
global bdir is vxcl(up:vector, -e0):normalized.
set steer to lookdirup(bdir, up:vector).
cam(CAM_FLIP).
say("Flip for boostback", 3).
set steeringmanager:maxstoppingtime to 4.
global tflip is time:seconds.
until vang(ship:facing:vector, bdir) < 6 or time:seconds > tflip + 40 {
  logrow(). wait 0.
}
set phase to "boostback".
say("Boostback burn", 4).
global bb_done is false.
lock thr_bb to choose 0 if bb_done else clamp(e_dr:mag / (tof * ship:availablethrust / ship:mass * BB_TAU), 0.04, 1).
lock throttle to thr_bb.
lock steering to lookdirup(choose bdir if e_dr:mag < 300 else vxcl(up:vector, -e_dr):normalized, up:vector).
when e_dr:mag < BB_TOL or vdot(e_dr, einit) < 0 then { set bb_done to true. }
until bb_done {
  set pr to predict(ALT0).
  set vref to ship:velocity:surface.
  set e0 to pad_err(pr[0]).
  set tof to pr[1].
  if e0:mag > 300 set bdir to vxcl(up:vector, -e0):normalized.
  set last_err to e0. set last_tof to tof.
  set thr to thr_bb.
  logrow().
  wait 0.
}
set thr to 0.
lock throttle to thr.
set steer to lookdirup(-ship:velocity:surface, up:vector).
lock steering to steer.
say("Boostback complete", 3).
brakes on.
global brakes_set is true.

// ---- coast / entry / aero steering ----------------------------------------------------------
set phase to "coast".
cam(CAM_COAST).
global aero_off is v(0, 0, 0).
global corr is false.
lock corr_dir to up:vector * cos(CORR_TILT) - e_dr:normalized * sin(CORR_TILT).
lock steering to choose lookdirup(corr_dir, ship:facing:topvector) if corr else lookdirup(-ship:velocity:surface:normalized + aero_off, up:vector).
lock throttle to choose 0 if not corr or vang(ship:facing:vector, corr_dir) > 8 or e_dr:mag < CORR_OFF
  else clamp(e_dr:mag / (max(5, tof) * amax * sin(CORR_TILT) * CORR_TAU), 0.05, 1).
// ignition trigger evaluated every tick (the predictor blocks the main loop for a few ticks)
lock hgt to ship:altitude - ALT0.
lock amax to ship:availablethrust / ship:mass.
global burn is false.
when ship:verticalspeed < -20 and hgt < ship:verticalspeed ^ 2 / (2 * max(1, IGN_F * amax - G0)) + H_HOLD + abs(ship:verticalspeed) * IGN_LAG then {
  set burn to true.
}
until burn {
  local hh is ship:altitude - ALT0.
  if ship:verticalspeed < 0 and hh < 28000 and phase = "coast" { set phase to "aero". cam(CAM_AERO). say("Entry: steering with the airframe", 3). }
  if hh > WARP_COAST_H set_warp(3). else if hh > WARP_AERO_H set_warp(1). else set_warp(0).
  learn().
  set pr to predict(ALT0).
  set vref to ship:velocity:surface.
  set e0 to pad_err(pr[0]).
  set tof to pr[1].
  set last_err to e0. set last_tof to pr[1].
  if phase = "aero" and not corr and hh < CORR_H_MAX and hh > CORR_H_MIN and e0:mag > CORR_ON {
    set corr to true. set phase to "corr". say("Correction burn", 2).
  }
  if corr and (e0:mag < CORR_OFF or hh < CORR_H_MIN) { set corr to false. set phase to "aero". }
  set thr to choose 0.01 if corr else 0.
  if phase = "aero" or phase = "corr" {
    local vn is ship:velocity:surface:normalized.
    local tilt is min(AOA_MAX, e0:mag * AOA_K).
    set aero_off to vxcl(vn, e0):normalized * tan(tilt) * AERO_SIGN.
  }
  if hh < CAM_LAND_H and phase <> "final" { set corr to false. set phase to "final". cam(CAM_LAND). }
  logrow().
  wait 0.
}

// ---- landing burn ----------------------------------------------------------------------------
set_warp(0).
set phase to "burn".
say("Landing burn", 4).
if hgt > CAM_LAND_H { cam(CAM_LAND). }
global acmd is up:vector * G0.
lock steering to lookdirup(acmd, ship:facing:topvector).
lock throttle to clamp(vdot(acmd, ship:facing:vector) * ship:mass / max(0.001, ship:availablethrust), 0, 1).
global legs_dn is false.
global td is false.
global td_v is 0.
global td_vz is 0.
global td_vh is 0.
when ship:status = "LANDED" or hgt < 0.15 then {
  set td to true.
  set td_v to ship:velocity:surface:mag.
  set td_vz to ship:verticalspeed.
  set td_vh to vxcl(up:vector, ship:velocity:surface):mag.
}
until td {
  local upv is up:vector.
  local h is hgt.
  local vz is ship:verticalspeed.
  local rel is vxcl(upv, PAD:altitudeposition(ALT0)).
  local vh is vxcl(upv, ship:velocity:surface).
  local az is G0.
  local ah is v(0, 0, 0).
  if not legs_dn and h < LEGS_H { legs on. set legs_dn to true. }
  if h > H_HOLD + 0.5 and phase = "burn" {
    local vt is -V_HOLD.
    set az to (vz ^ 2 - vt ^ 2) / (2 * max(0.5, h - H_HOLD)) + G0.
    if vz > vt set az to G0 + 2 * (vt - vz).
    local tgo is max(TGO_MIN, 2 * (h - H_HOLD) / max(1, abs(vz) + abs(vt))).
    set ah to rel * (6 / tgo ^ 2) - vh * (4 / tgo).
    local hmax is tan(TILT_MAX) * az.
    if ah:mag > hmax set ah to ah:normalized * hmax.
  } else {
    set phase to "terminal".
    local vt is -max(V_TD, min(V_HOLD, 0.3 * h)).
    set az to G0 + 2.5 * (vt - vz).
    set ah to rel * TERM_KP - vh * TERM_KD.
    local hmax is tan(TILT_TERM) * max(0, az).
    if ah:mag > hmax set ah to ah:normalized * hmax.
  }
  set acmd to upv * az + ah.
  set last_err to rel.
  logrow().
  wait 0.
}
lock throttle to 0.
set thr to 0.
set phase to "landed".
set steer to lookdirup(up:vector, ship:facing:topvector).
lock steering to steer.
say("Touchdown", 4).
global ttd is time:seconds.
until (ship:velocity:surface:mag < 0.05 and time:seconds > ttd + 3) or time:seconds > ttd + 12 { logrow(). wait 0. }
unlock steering.
global miss is vxcl(up:vector, PAD:position):mag.
global res is "miss_m " + round(miss, 3) + " td_speed " + round(td_v, 3) + " td_vz " + round(td_vz, 3) + " td_vh " + round(td_vh, 3)
  + " lat " + ship:geoposition:lat + " lng " + ship:geoposition:lng + " alt " + round(ship:altitude, 3) + " alt0 " + round(ALT0, 3)
  + " status " + ship:status + " prop_left " + round(booster_prop(), 3) + " parts " + ship:parts:length + " kd_e6 " + round(kd * 1e6, 2)
  + " flight_s " + round(time:seconds - T0, 1).
log res to RESF.
log "# " + res to LOGF.
say("Landed " + round(miss, 2) + " m from pad centre", 8).
wait 8.
cam("quit").
wait until false.
