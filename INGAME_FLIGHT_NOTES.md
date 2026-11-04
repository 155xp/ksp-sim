# In-game demo flights: notes

Findings from the in-game RTLS demo flights (`ingame/`, kOS logs in `ingame/telemetry/`).
Each section is self-contained.

## Airbrakes (A.I.R.B.R.A.K.E.S, `airbrake1`) in ksp-core

The booster carries 4 stock airbrakes at azimuths 0/90/180/270 deg, 1.45 m above the FL-T800
centre. ksp-core now models them from the game's code and cfg, not from a fit. The model was
validated against both flights' unpowered, brakes-deployed descents. Reproduce with:

    cargo run --release -p ksp-core --example airbrake_validate

### What the game does (decomp + Airbrake.cfg + Physics.cfg)

- **No cube drag, no body lift.** `dragModelType = none` gives `Part.DragModel.NONE`, so
  `FlightIntegrator.CalculateDragValue` returns 0. The drag cubes are still evaluated and blended
  by deflection (`ModuleControlSurface.FixedUpdate` → `SetCubeWeight`):
  - neutral weight = 1.5 − |d/70|;
  - fullDeflectionPos weight = clamp01(d/70);
  - fullDeflectionNeg weight = clamp01(−d/70).

  These cubes only feed thermal and area code. `ModuleLiftingSurface.disableBodyLift` defaults to
  true, which sets `Part.hasLiftModule`, so the part has no body lift.
- **All of its force is the module's internal model**, applied at the part origin
  (CoP/CoL offsets are 0):
  - drag = −nVel · SpeedBrake.drag(|dot|) · SpeedBrake.dragMach(M) · 0.38 · Q[Pa] · 0.015,
    which is 5.7 kN per kPa per unit curve value;
  - lift: the SpeedBrake lift curve is identically 0, so the airbrake makes no lift.
  - `ctrlSurfaceArea = 1`, so the whole force uses the deflected lift vector.
- **The lift vector is the part's +Z axis**, which is the outward wall normal for this surface
  attach. `ModuleControlSurface` re-declares `transformName` with `new`, and
  `BaseFieldList.ReflectedData` resolves fields by name on the most derived type. So the cfg's
  `transformName = Flap` only reaches the control-surface field (the visual flap).
  `ModuleLiftingSurface.baseTransform` stays the part transform.
- **Deflection turns the vector once** (`airflowIncidence`): lift vector = Z·cos d − Y·sin d.
  In axial flow (nose-first or engine-first) this gives |dot| = sin d.
  - Deployed (d = 70): drag curve value 2.30.
  - Stowed (d = 0): |dot| = 0 and the curve value is 0.01, so stowed drag is 0.43 % of deployed.
- **Actuator**: deflection moves toward `aeroDeployAngle` with `Mathf.MoveTowards` at
  actuatorSpeed 20 deg/s, i.e. 0.4 deg per tick, about 3.5 s from 0 to 70 deg.
  - `aeroDeployAngle` is NaN in the cfg, so OnStart sets it to ctrlSurfaceRange = 70.
  - `ignorePitch` / `ignoreYaw` are set and roll is forced off, so control input never moves the
    flap.
  - The flap also moves in vacuum.
  - A flap within Unity's quaternion tolerance of neutral (|d| < 0.162 deg) counts as neutral.

Drag of one deployed airbrake in axial flow, per unit dynamic pressure (m², i.e. kN/kPa):

| M   | 0.2  | 0.3  | 0.4  | 0.5  | 0.6  | 0.7  | 0.8  | 0.9  | 1.0  | 1.1  | 1.5  | 2.0  |
|-----|------|------|------|------|------|------|------|------|------|------|------|------|
| m²  | 1.64 | 1.68 | 1.76 | 1.91 | 2.15 | 2.50 | 2.98 | 3.61 | 6.91 | 9.85 | 4.86 | 3.94 |

### API (ksp-core)

**State.** Brakes are action-group state, as in the game.
- `EnvState::brakes: bool` holds the Brakes action group (default false). It stays set until
  cleared.
- `EnvState::brake_deflection: f32` is the flap angle in degrees, shared by all of a vessel's
  airbrakes.
- `Control` is unchanged. A `Control.brakes` field would have broken the exhaustive `Control { .. }`
  literals in `guidance/core/src/attitude.rs` and `crates/ksp-py`, which this change could not
  edit.

**Parts.**
- `parts::airbrake()` (in `by_name("airbrake1")`), plus `AIRBRAKE_NODE` for
  `Craft::surface_attach_editor`.
- `parts::aero_surface(name) -> Option<AeroSurfaceDef>` holds the module data, kept out of
  `PartDef` so struct literals of `PartDef` keep compiling. `AeroSurfaceDef::cube(neutral, d)`
  gives the blended cube.
- New catalog parts: `probe_stack_small()`, `battery_bank()`, `adv_sas_module()`,
  `decoupler_td12()`, `lt2_leg_retracted()`.

**Vessel spec and sim.**
- `vessel::AeroSurfaceSpec`, with `VesselSpec::{n_aero_surf, aero_surf, part_no_cube_drag}`.
- `vessel::MAX_PARTS` is now 24 (was 16); the full demo stack needs 18.
- `sim::aero_surface_force`, `sim::AeroSurfaceFlap`, `sim::move_towards`.
- Curves: `aero::SPEED_BRAKE*`, `aero::LiftingCurves`, `aero::control_surface_cube_weights`.

**Traces.** `trace::spec_from_geometry` loads `airbrake1` and ignores the dumped cube faces for it.

**Cost.** Hot path: about +50 ns per airbrake per tick (hopper 622 ns, hopper + 4 airbrakes
818 ns; min of 5 runs, `examples/profile_tick`). Vessels without airbrakes pay nothing measurable.

### Validation against flights 1 and 2

**Segments.**
- Flight 1: brakes on from apogee (t = 89.7 s, 17.5 km) to the landing burn (t = 210.9 s).
- Flight 2: brakes on from the end of boostback (t = 98.1 s, 18.9 km) to t = 218.6 s.

The engine is off throughout. The booster is rebuilt from `ingame/make_demo_craft.py` geometry:
TD-12 (stays on the booster: `explosiveNodeID = top`) / RC-001S / Z-1k / small reaction wheel /
FL-T800 / FL-T400 / Swivel, 4 LT-2 legs (retracted cube) and 4 airbrakes. Dry mass is 3.140 t,
equal to the log's mass − prop. Propellant comes from the logged mass.

**Measured drag deceleration:** −(Δv/Δt − gravity − Coriolis − centrifugal) · v̂, over 1 s
windows of the logged vz/vh. The horizontal direction comes from the lat/lng track.

**Model:** ksp-core's `sim::tick` at the logged state, with aero on minus aero off. The attitude
uses the logged tilt. The tilt's azimuth is not logged; it is taken in the plane of the horizontal
velocity.
- The lift data picks the sense: with the top leaning *against* the horizontal velocity, modelled
  lift is 0.31 m/s² against 0.35 measured.
- The other sense gives 0.97 m/s² of lift, which the data rules out.

Windows with q > 0.5 kPa, drag model/measured − 1 (identified sense):

| flight | windows | mean    | rms    |
|--------|---------|---------|--------|
| 1      | 126     | +0.43 % | 1.50 % |
| 2      | 111     | −0.37 % | 0.94 % |
| both   | 237     | +0.05 % | 1.27 % |

By Mach (flight 1 / flight 2). "Total/bare" is the drag with brakes deployed divided by the
booster without airbrakes (stowed brakes add ~1.7 %), model and measured:

| Mach    | mean alt (m) | drag meas (m/s²) | model vs meas | total / bare, model | total / bare, measured |
|---------|--------------|------------------|---------------|---------------------|------------------------|
| 0.0–0.4 | 2500 / 2700  | 10.47 / 10.48    | +1.6 / +0.2 % | 3.91 / 4.05         | 3.85 / 4.04            |
| 0.4–0.5 | 7400 / 6500  | 9.62 / 10.47     | +0.5 / −0.2 % | 4.25 / 4.28         | 4.22 / 4.28            |
| 0.5–0.6 | 11100 / 10700 | 8.32 / 9.06     | −0.6 / −2.3 % | 4.76 / 5.13         | 4.79 / 5.25            |
| 0.6–0.7 | 13000 / 12700 | 8.25 / 9.06     | −1.1 / −1.4 % | 5.40 / 5.71         | 5.46 / 5.80            |
| 0.7–0.9 | 13300 / 14100 | 8.94 / 9.00     | −1.4 / −1.0 % | 5.61 / 6.50         | 5.68 / 6.57            |

The 4 deployed airbrakes carry 74 % (M 0.3) to 84 % (M 0.8) of the descent drag. Their share rises
with Mach because SpeedBrake.dragMach climbs from 0.13 to 0.23 while the tanks' cube drag changes
little.

**Open-loop integration** from the first brakes-on state, attitude pinned to the logged tilt
(identified sense). The sim steps at 0.02 s; the logs were at 2x physics warp (0.04 s).
- Flight 1 (121 s, 272 rows): rms vz 0.36 m/s, vh 0.14 m/s, alt 11 m. At the burn start
  (452 m): vz +0.27 m/s, alt +0.3 m.
- Flight 2 (120 s, 230 rows): rms vz 0.59 m/s, vh 1.35 m/s, alt 33 m. At the burn start: vz
  +0.50 m/s, alt −30 m.
  - The largest vh error (+2.6 m/s) comes in the `corr` phase, where the horizontal velocity
    passes through zero and reverses. There the "tilt in the velocity plane" assumption cannot
    hold.

**Rejected readings of the code, for contrast** (same windows):
- Lift vector = the Flap transform's forward, turned by 2 × deflection: drag −13.8 % / −14.6 %.
  This reading would also give stowed airbrakes |dot| = 1 in axial flow, i.e. curve value 2.4.
  Estimated: ~500 kN of drag at the 26 kPa ascent max-q against 215 kN of thrust, while the
  rocket was accelerating at ~15 m/s² along its path.
- Blended-cube drag applied on top (dragModelType ignored): +12.7 % / +11.7 %.

**kd (the kOS fit).** The log's `kd_e6` (1800–2240) uses the kOS density table. With ksp-core's
density, the same definition gives 1900–2860 measured and the model matches it to within the
residuals above. The prior of 120 was never representative of this booster: even with stowed
brakes the model gives about 400–500.

**UT** is not logged. The density sensitivity is +0.02 % for UT 67 vs 0 and +0.18 % for UT 600
vs 0, which is negligible.

### Approximated / not modelled

- **Probe core, battery, reaction wheel, decoupler.** Transcribed cfg mass, nodes, resources and
  wheel torque, with the demo install's PartDatabase cubes. On a few faces these differ < 1 % from
  the fidelity install's regenerated cubes. Inertia is the usual cube-sized box. ModuleCommand,
  SAS and the decoupler module are not simulated.
- **Airbrake details.**
  - Joint breakForce is not measured; 4400 kN is assumed.
  - Heating is not modelled: the skinMaxTemp switch between cased and uncased does not matter.
  - The flap's own inertia is not modelled.
  - The cube blend exists (`AeroSurfaceDef::cube`) but feeds nothing; the game uses it only for
    thermal.
- **One deflection per vessel.** The stock module ignores control input, so all airbrakes move
  together. Brakes set to respond to pitch/yaw in the PAW (per-part deflection) are not modelled.
- **No GPU support.** The CUDA kernel (gpu.rs packing) ignores aero surfaces; vessels with
  airbrakes run on the CPU tick only.

### What remains unvalidated, and the in-game test that would validate it

The kOS logs fix the **drag along the flight path** at |AoA| ≲ 10 deg and M 0.3–0.77, plus the
**magnitude** of the perpendicular force. They do not separate per-part lift from the tanks' body
lift, and they say nothing about the aero torque (kOS steering held the attitude).

**Unvalidated:**
- per-airbrake force direction at AoA;
- the asymmetric drag of opposite brakes, and the restoring torque it makes;
- the deploy transient (the drag dips toward the stowed value near d = 45 deg and rises again);
- transonic and supersonic dragMach (M ≥ 0.8; the curve peaks at M 1.1);
- nose-first flow in game (the model is symmetric by construction).

**SimHarness test that would close these.** A tick-level trace with per-part aero vectors:
- **Craft:** probe / FL-T800 with 4 airbrakes at 0/90/180/270 (VAB placement as above) / FL-T400 /
  Swivel, launched to ~10 km.
- **Brakes:** toggled through kOS `BRAKES` at known ticks, so the transient is logged at 0.4 deg
  per tick at 1x warp.
- **AoA sweeps:** kOS holds the booster at AoA 0, 5, 10, 20 and 30 deg in a fixed plane at
  roll 0 and at roll 45 deg (brakes in and out of the AoA plane). Do this in both engine-first
  (falling) and nose-first (climbing, engine at low thrust) flow, through M 0.3–1.4.
- **Extra per-airbrake columns on top of the usual SimHarness ones:**
  - `ModuleAeroSurface.deflection` (protected; read by reflection);
  - `dragForce` and `liftForce` (public `ModuleLiftingSurface` fields), `Qdrag`, `nVel`;
  - `part.machNumber` and `part.dynamicPressurekPa`;
  - the part transform's forward / up.
- **Comparison:** seed each tick with `trace::state_at` plus `brakes` / `brake_deflection` from
  the log. Compare each airbrake's `aero_surface_force` with the logged `dragForce` per tick, the
  way `examples/part_aero_check.rs` does for cube drag. Compare the angular acceleration of
  unsteered stretches (SAS off) with the sim's torque.
