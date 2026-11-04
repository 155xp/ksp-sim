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
* Krakensbane and FloatingOrigin are pure Galilean shifts for float precision; the sim uses f64 and
  ignores them.

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
5. **Gimbal / reaction wheel**: wheel torque = rotation·−(pitch·Pt, roll·Rt, yaw·Yt) lerped at
   30·dt, gated by ElectricCharge (`ModuleReactionWheel`); gimbal deflection = input × range.
6. **Aero per part** (`FlightIntegrator.UpdateAerodynamics`, cube model): per face i
   `S = DragCurveValue((d·n_i+1)/2, M)` (lerp TAIL→SURFACE→TIP × MULTIPLIER), area += A_i·S,
   areaDrag += A_i·S·Cd′ with Cd′ = DRAG_CD(Cd)^DRAG_CD_POWER(M) for Cd < 1; drag =
   q_part · areaDrag · 0.1 · DRAG_PSEUDOREYNOLDS(ρ·v) · 8 along −v, applied at the part's CoP.
   Faces covered by stack neighbours are occluded at build time exactly like
   `DragCubeList.SetPartOcclusion`. Body lift from windward faces through the BodyLift curve × 18,
   projected perpendicular to v. Angular drag: `rb.angularDrag = angularDrag·q_atm·2`.
7. **Ground contact**: legs as wheel-collider-like springs, `spring = springRatio × sprung weight`,
   `damper = sqrt(spring·damperRatio)` (`ModuleWheelSuspension.SuspensionSpringUpdate`), Coulomb
   friction; hull points (tank rims, nozzle, pod top) as stiff contacts that destroy the vessel above
   the part's `crashTolerance`.
8. **Integrate** like PhysX: v += a·dt, x += v·dt; ω += I⁻¹τ·dt, angular damping, |ω| ≤ 50 rad/s.
   In vacuum (P = 0, above the atmosphere, not landed) KSP's **orbit drift compensation**
   (`VesselPrecalculate`, ORBIT_DRIFT_COMPENSATION) pins the CoM to the Kepler conic and adds only
   non-gravitational dv; the sim does the same after 5 vacuum frames.

## Deliberate approximations (and why)

| approximation | KSP | impact for the pad lander |
|---|---|---|
| vessel = one rigid body | every part is a PhysX rigidbody with joints | negligible for a short, stiff lander; no joint flex/wobble |
| part inertia = box of drag-cube size | inertia from colliders (model files) | rotational response within tens of percent |
| thrust transform at the engine bottom node, along part +Y | `thrustTransform` in the .mu model | correct line of action for on-axis engines |
| gimbal = rotation about root X/Z by input × range | `ModuleGimbal.GimbalRotation` per transform | same authority and sign as the wheel; transform hierarchy not modelled |
| leg spring/damper without boost / auto-spring terms; foot point from cube extents | `ModuleWheelSuspension` + VehiclePhysics wheel collider | touchdown dynamics are plausible but not 1:1 |
| terrain = sphere at pad-deck altitude | PQS terrain + KSC colliders | exact at the pad; wrong away from it |
| temperature offset refreshed every 25 ticks | per rendered frame | < 1e-4 K |
| thermal model, buoyancy, parachutes, lifting surfaces, RCS: not implemented | | not needed for powered landing on the pad |
| only the main body's gravity; SOI changes not handled in the physics tick | same (no n-body) | none near Kerbin |
| FloatCurve evaluated in f64 with f32 inputs/outputs | Unity float math | ~1e-7 relative |
