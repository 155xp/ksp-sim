# Design notes: how stock KSP 1.12.5 flies a vessel, and how ksp-sim reproduces it

Everything here was learned by reading the decompiled `Assembly-CSharp.dll` of KSP 1.12.5
(build 3190, Linux Steam build) with `ilspycmd` into the git-ignored `decomp/` folder, plus the
game's own data files. No decompiled code is in this repository; the sim is written from scratch
and cites the class/method it re-implements. Cross-checks: kOS (pad coordinate), MechJeb / KSP
wiki numbers (constants, engine stats).

## Data provenance

| data | source in the install | how it gets into the sim |
|---|---|---|
| CelestialBody fields (radius, GeeASL, atmosphere curves, rotation, orbits) | `KSP_Data/sharedassets9.assets` (Kerbol prefab) | `tools/extract_bodies.py` -> `ksp_data_local/bodies.json` (ignored) -> `tools/gen_bodies_rs.py` -> `crates/ksp-core/src/bodies_data.rs` |
| KSC / launch pad transforms, pad colliders | `sharedassets9.assets` | `tools/extract_ksc.py` (Windows thread) -> constants in `ksc.rs` |
| drag / lift curves, global multipliers | `Physics.cfg` | transcribed in `aero.rs`, checked against the install by a test |
| part stats, engine curves, attach nodes | `GameData/Squad/Parts/**.cfg` | transcribed in `parts.rs`, checked against the install by a test |
| drag cubes | `PartDatabase.cfg` | transcribed in `parts.rs`, checked against the install by a test |

## Frames

* **World frame below 100 km is Kerbin-fixed.** `OrbitPhysicsManager.checkReferenceFrame` turns on
  `inverseRotation` when the active vessel is below `inverseRotThresholdAltitude` (Kerbin: 100 km;
  `CelestialBody` clamps it to at least `atmosphereDepth`). In that mode the planet does not move
  in Unity coordinates and the rigidbodies hold surface-relative velocities.
* **The fictitious forces are applied.** `VesselPrecalculate.CalculateGravity` adds
  `FlightGlobals.getCoriolisAcc` (−2 Ω×v) and `getCentrifugalAcc` (−Ω×(Ω×r)) to gravity
  (`getGeeForceAtPosition`, −GM r/|r|³, point mass of the main body only), evaluated once at the
  vessel CoM and applied as the same acceleration to every part (`FlightIntegrator.Integrate`).
  So KSP's physics is frame-consistent; the sim integrates in the Kerbin-fixed frame with these
  terms (`sim.rs`), verified against an inertial integration (`coriolis_deflects_drop`).
* **Switch at 100 km.** `setRotatingFrame` adds/subtracts Ω×r to rigidbody velocities and leaves
  positions alone. The sim re-expresses state in the inertial frame at the switch.
* **Body frame convention** (sim): +x lat 0 / lon 0, +y lon 90E, +z north. Unity body-local
  vectors map to it as `.xzy`. Rotation angle at UT: `(initialRotation + 360·UT/P_sid) mod 360`
  (`CelestialBody.CBUpdate`), with the sidereal period derived from the serialized 21600 s solar
  day as `CelestialBody.Start` does.
* Krakensbane and FloatingOrigin are Galilean shifts for float precision. The sim uses f64 and
  doesn't need them for the vessel; they matter only in two places. The logger sees positions one
  tick ahead while Krakensbane is engaged (corrected in `trace.rs`). And the scenery is re-rounded
  at each origin shift (next section).

## Scenery placement

`FloatingOrigin.setOffset` subtracts the offset from every `CelestialBody.position` (double). The
PQS and KSC Unity transforms are float and are rebuilt from those positions, at ~600 km coordinate
magnitude where the float ulp is 1.5–6 cm. So the colliders of the pad, the KSC and the terrain
sit a few cm from where the double-precision body says they are. Measured in game (harness ground
probe, VALIDATION_INGAME.md):

* the whole scenery is translated by the PQS transform's rounding vector. It is re-rounded at
  every origin shift, which happens every tick while radar altitude is above ~150 m and not below
  it. Radial part: σ 9 mm, within ±2.2 cm;
* the KSC colliders carry an extra radial offset fixed per game session: 0 in 7 of 9 sessions,
  +3.7 cm and −1.3 cm in the other two. PQS terrain doesn't.

The sim models both as `EnvState.ground_shift` (translation of all scenery) and `EnvState.ksc_dz`
(KSC colliders, i.e. grid cells whose surveyed collider is not PQS terrain); `sim::Scenery`.
Traces seed them from the logged values. Training resets can randomize them (`ground_shift_max`,
`ksc_dz_max`; default 0 = the surveyed placement).

## One physics tick (dt = 0.02 s), in KSP's order

1. **Mass**: part mass = dry + Σ resource·density (`FlightIntegrator.UpdateMassStats`); vessel CoM
   and velocity are mass-weighted (`VesselPrecalculate.CalculatePhysicsStats`).
2. **Velocity smoothing** (`FlightIntegrator.SmoothVelocity`): 10-sample float ring buffer with a
   one-tick spike hold; the *smoothed* speed feeds vessel mach, dynamic pressure and the
   pseudo-Reynolds number.
3. **Atmosphere** at the CoM altitude: `GetPressure` (curve evaluated at `(float)alt`, 0 at and above
   70 km), `GetFullTemperature` = curve + `atmosphereTemperatureSunMultCurve(alt)` × offset, where
   the offset comes from `GetAtmoThermalStats` (latitude bias + latitude sun-mult × time-of-day
   factor; KSC gets +17..+26 K, so pad density is 1.124..1.157 kg/m³, not 1.225).
   ρ = P·1000·M/(R·T), R = 8.31447 (hard-coded in PhysicsGlobals), c = sqrt(γP/ρ).
4. **Engines** (`ModuleEngines`): throttle response (`UpdateThrottle`), Isp = atmosphereCurve at the
   *part's* static pressure in atm, mass flow = maxFuelFlow·throttle with maxFuelFlow =
   maxThrust/(Isp(0)·g), g = 9.80665**f** (a float field on ModuleEngines), thrust = flow·Isp·g;
   propellant draw with `ignitionThreshold` deprivation / flameout (`RequestPropellant`,
   `CheckDeprived`). Two-value cfg keys get Unity `AddKey` smooth tangents (only the inserted key's
   slope is set, so Isp between keys is *not* linear: Swivel at 0.5 atm = 293.75 s, not 285 s).
5. **Gimbal / reaction wheel.**
   - **Wheel torque** = rotation·−inputVector, with inputVector = (pitch·Pt, roll·Rt, yaw·Yt)
     lerped at 30·dt and gated by ElectricCharge (`ModuleReactionWheel`). Torque is a
     pseudovector, so in the sim's x-mirrored frame it is (−x, +y, +z).
   - **Gimbal:** actuation (pitch, roll, yaw) at 3° per unit. Pitch tilts toward root +Z, yaw
     toward sim +X. The thrust transform rotates about the measured gimbal pivot, so the thrust
     point swings with the deflection. Thrust transform and pivot positions come from the game's
     geometry dump.
6. **Aero per part** (`FlightIntegrator.UpdateAerodynamics`, cube model): per face i
   `S = DragCurveValue((d·n_i+1)/2, M)` (lerp TAIL→SURFACE→TIP × MULTIPLIER), area += A_i·S,
   areaDrag += A_i·S·Cd′ with Cd′ = DRAG_CD(Cd)^DRAG_CD_POWER(M) for Cd < 1; drag =
   q_part · areaDrag · 0.1 · DRAG_PSEUDOREYNOLDS(ρ·v) · 8 along −v, applied at the part's CoP.
   Faces covered by stack neighbours are occluded at build time exactly like
   `DragCubeList.SetPartOcclusion`. Body lift from windward faces through the BodyLift curve × 18,
   projected perpendicular to v and applied at the part's CoL. Angular drag:
   `rb.angularDrag = angularDrag·q_atm·2`, applied per part as c_i·I_own,i. Drag cubes are
   mirrored into the sim frame (XP ↔ XN).
7. **Ground contact.**
   - **Ground:** the in-game raycast survey of layer 15 (PQS terrain + KSC colliders) on a 0.2 m
     grid within ±40 m of the pad and a 4 m grid within ±1.5 km (`terrain.rs`), placed per
     `Scenery`. Friction: Unity default 0.6 / Average on KSC colliders, "Ground" 0.9 / Maximum on
     PQS terrain.
   - **Legs:** the whole chain, each step in float as in the game.
     1. `SuspensionLoadBalancer` gives the sprung mass share.
     2. `ModuleWheelSuspension` computes spring/damper: BoostCurve, autoBoost,
        springTweakable/damperTweakable, rounding, and the 3 / 1.05 hysteresis.
     3. The VPP wheel collider applies them one tick late, with target clamp 0.1 and slew 0.02.
     4. PhysX applies F = k(x − target·d) + c·dx/dt, projected on the ground normal.

     The ray runs along the leg's suspension axis. The tire is the locked-wheel friction model
     (Smooth curve (0.01, 0.1), (2, 1), (7, 1), with Fext = −0.5·(F/g)·V/dt lerped at 10·dt).
   - **Hull points:** the collider extremes from the geometry dump. They feed a sequential-impulse
     solve (8 iterations, inelastic, speculative within contactOffset 0.01 m, depenetration at
     0.55 of the gap per tick). The vessel crashes when a contact impulse exceeds the part joint's
     breakForce, or the post-solve normal speed exceeds `crashTolerance`.
   - **Early-out:** above the highest ground point (plus the scenery offset) none of this runs.
8. **Integrate** like PhysX: v += a·dt, x += v·dt. For rotation, the composite inertia about
   the CoM integrates ω += I⁻¹(τ − ω×(I_orb·ω))·dt, where I_orb is the parallel-axis part only:
   each PhysX body integrates its own spin, so only the orbital part shows up as a vessel-level
   gyroscopic term. Then per-part angular damping, |ω| ≤ 50 rad/s. Ground contacts are resolved
   after the velocity update.
   In vacuum (P = 0, above the atmosphere, not landed) KSP's **orbit drift compensation**
   (`VesselPrecalculate`, ORBIT_DRIFT_COMPENSATION) pins the CoM to the Kepler conic and adds only
   non-gravitational dv; the sim does the same after 5 vacuum frames.

## What is not 1:1

The current list, each item with its measured bound against the running game, is in
VALIDATION_INGAME.md ("What is still not 1:1"). In short:
- the vessel is one rigid body (joint flex, i.e. thrust-axis sag ≤ 2e-4 rad, is noise);
- the contact solver is sequential impulses on the composite (post-impact sliding 0.2–0.45 m
  over 5 s on hard impacts, ≤ 5 cm on soft pad landings);
- the crash rule is not yet validated by a crash series;
- yaw response during fast gimbal ramps is off by ~7%;
- terrain off the pad is good to ~1 cm;
- the GPU kernels have no vacuum orbit-drift compensation.

## Precision

The CPU tick is f64 except where the game uses float: drag-cube faces, the wheel/suspension
chain, curve arguments and results. The GPU `mixed` kernel goes further. It keeps double only
where KSP keeps double (CoM position and velocity, gravity, altitude, resource amounts, attitude
accumulation) and computes everything Unity/PhysX computes in float in float, including the
terrain queries, which run in a local frame at the pad. Its error against the game equals the
CPU's to 3 significant figures on every trace. The GPU `f64` kernel matches the CPU to 1e-9 m.
