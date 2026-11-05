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

## 3.75 m heavy booster parts in ksp-core

These are the parts of the heavy booster (`guidance` `ingame::heavy_craft`): Mk7 nose /
RC-L01 / large reaction wheel / Z-4K / ADTP-2-3 / S3-14400 (root) / S3-7200 / Mammoth, six LT-2
legs and eight airbrakes, 22 parts. They are transcribed from the demo install
(`~/ksp-demo/KSP`) the same way as the earlier catalog parts. `parts::by_name` serves them, so
`guidance` `partlib::load` now returns these definitions for all eight names.

### What was transcribed

| name | fn | cfg | mass t | crash m/s | nodes (size) | resources | modules simulated |
|---|---|---|---|---|---|---|---|
| `rocketNoseCone_v2` | `nose_cone_mk7` | Aero/cones/protectiveRocketNoseMk7_v2.cfg | 0.2 | 10 | bottom 0 (2) | – | – (angularDrag 0.5) |
| `probeStackLarge` | `probe_stack_large` | Command/probeStackLarge | 0.5 | 9 | ±0.19 (2) | EC 30 | wheel 1.5/1.5/1.5, 0.15 EC/s |
| `asasmodule1-2` | `adv_sas_module_large` | Command/advancedSasModuleLarge | 0.2 | 9 | ±0.25 (2) | – | wheel 30/30/30, 0.6 EC/s |
| `batteryBankLarge` | `battery_bank_large` | Electrical/z-4kBattery | 0.2 | 8 | +0.11 / −0.142 (2) | EC 4000 | – (angularDrag 1) |
| `Size3To2Adapter_v2` | `adapter_s3_s2` | Structural/Size3To2Adapter_v2 | 1.875 | 6 | +1.15017 (2) / −1.1 (3) | LF 1350 Ox 1650 | – |
| `Size3LargeTank` | `s3_tank_14400` | FuelTank/Size3Tanks/large.cfg | 9 | 6 | ±3.74 (3) | LF 6480 Ox 7920 | – |
| `Size3MediumTank` | `s3_tank_7200` | FuelTank/Size3Tanks/medium.cfg | 4.5 | 6 | +1.931 / −1.937 (3) | LF 3240 Ox 3960 | – |
| `Size3EngineCluster` | `mammoth` | Engine/Size3EngineCluster/part.cfg | 15 | 20 | top +1.527248 (3) only | – | ModuleEnginesFX, ModuleGimbal |

None of these cfgs sets CoM, CoP or CoL offsets. All are rescaleFactor 1.

**Drag cubes.** Where `ModulePartVariants` is present it has `useMultipleDragCubes = false`, so
each part uses one "Default" cube from PartDatabase.cfg. Two parts carry their own `DRAG_CUBE` in
the part cfg: the ADTP-2-3 and asasmodule1-2. For those, PartLoader puts the cfg cube into the
prefab at full precision and only writes a rounded copy to PartDatabase. Flight parts clone the
prefab's serialized `DragCubeList`, so ksp-core uses the cfg value. The test checks the
PartDatabase copy against it to 4 significant digits.

**Mammoth engine.** maxThrust 4000, minThrust 0, Isp 315 (vacuum) / 295 (1 atm) / 0.001 at
12 atm, LF:Ox 0.9:1.1, ignitionThreshold 0.1. No `useEngineResponseTime`, so throttle is
instantaneous. ModuleGimbal has gimbalRange 2, no per-axis ranges and no `useGimbalResponseSpeed`,
so the gimbal is instantaneous. ModuleAlternator (12 EC/s at full thrust) is not simulated.

**Thrust check.** Vacuum 4000 kN; sea level 3746.0 kN. The wet booster is 155.475 t
(32.475 t dry + 123 t propellant), so sea-level TWR is 2.456.

### How the Mammoth's four nozzles are modelled

**What the game does:**
- Both `thrustVectorTransformName` and `gimbalTransformName` are `obj_gimbal`, and the model has
  four of them. Their Unity positions are (±0.76, −1.14, ±1.38) in the part frame. Forward is −Y
  (exhaust down), right is −X, up is −Z.
- So each nozzle's thrust transform *is* its gimbal transform. The thrust point never swings
  about a separate pivot.
- `ModuleEngines.OnLoad` gives each transform `thrustTransformMultipliers` = 1/4 (there is no
  `transformMultipliers` node). `ThrustUpdate` applies `finalThrust / 4` at each transform.
- `ModuleGimbal.FixedUpdate` runs `GimbalRotation` per transform:
  - pitch and yaw are sign-flipped when the CoM is below the transform;
  - when the transform sits more than `minRollOffset` = 0.1 m off the reference transform's
    axis, roll is added to pitch (offset in x) and to yaw (offset in z);
  - each channel is clamped to ±1 in the gimbal frame and multiplied by 2°.
- The gimbal frames are axis-aligned, so that clamp is a clamp of the vessel-frame channels.

**What ksp-core does:**
- `EngineDef` holds the nozzle centroid (0, −1.14, 0), with pivot = thrust point.
- `parts::thrust_transforms("Size3EngineCluster")` returns the four positions.
- `VesselSpec::compile` copies them into `EngineSpec::{n_nozzles, nozzles, nozzle_roll}` through
  `EngineSpec::set_nozzles`.
- `sim::tick` loops over the nozzles when `n_nozzles > 0`, using the rule above, with the root
  origin as the reference transform (the existing gimbal model's assumption).
- The single-nozzle path is unchanged. All 9 trace regressions pass unchanged against
  `data/traces`.

**What the tests show (`parts::tests::mammoth_gimbal_four_nozzles`):**
- With no roll input every nozzle deflects alike. Four equal parallel forces equal one at the
  centroid, so the result matches a single centroid nozzle exactly (to 1e-9) for any pitch/yaw.
- Pitch, yaw and roll torques have the same sign as the reaction wheels' for the same input.
- Roll torque is T · (0.76 + 1.38) · sin 2°, about 299 kN·m at 4000 kN. That is about 10 times
  the two wheels together (31.5 kN·m). The single-nozzle equivalent has no roll authority.
- A roll command saturates the channel on one side when pitch is also full.

**Guidance gap.** `attitude.rs` models gimbal effectiveness from `pos` and `pivot` only, so it
does not know about this roll authority. `EngineSpec::nozzles` and `nozzle_roll` have what it
needs.

**Trace replay.** `trace::spec_from_geometry` now builds one engine per part from all of its
`thrustTransform` lines:
- one line: same behaviour as before;
- several lines: `set_nozzles` with the dumped positions, and direction = mean of −forward.

### Hull points, contacts and limits

- **Hull radii.** These are the bottom-face axis extents of each part's primary collider, read
  offline from the `.mu` files (`PartReader` format). This is the same quantity as the dumped
  FL-T400 0.604 / Mk1 0.625 / Swivel 0.316; the offline reader reproduces those three exactly.
  Values: S3 tanks 1.81111, ADTP-2-3 1.865699, RC-L01 / Z-4K / nose 1.25, asasmodule1-2 1.207407.
- **Mammoth contacts.** `Part.collider` is `bodyCollider` (mesh, y −1.566 … 1.524).
  - Four crash sensors sit on its bottom face (radius 1.287146 at y −1.565684).
  - The four `engineCollider` capsules (r 0.61, height 3.16, vertical, centres
    (±0.759, −1.136, ±1.39)) are separate colliders. Like the Swivel's bell, they push on the
    ground but never crash the engine. Their lowest points, y −2.716, come from
    `parts::passive_contacts`, which `VesselSpec::compile` adds with `hull_crash = false`.
- **Nose cone.** `PartLoader.GetPartCollider` runs on the prefab with both MODELs loaded. It
  picks orangeNoseCone's `foamCol`, which the default White variant then deactivates. So the
  cone's ring is `hull_crash = false`: it pushes but cannot crash the cone.
- **Limits.**
  - `MAX_HULL` 24 → 48, and `EnvState::hull_touch` u32 → u64. The heavy booster needs 43 hull
    points: nose 4, six stack parts × 5, Mammoth 1 + 4 + 4.
  - New `MAX_NOZZLES` = 4.
  - `MAX_PARTS` 24, `MAX_ENGINES` 4, `MAX_LEGS` 6 and `MAX_AERO_SURFACES` 8 are unchanged; this
    craft uses 22 / 1 / 6 / 8 of them.
  - `gpu::pack` now asserts `n_hull <= 32` (cuda_tick.cu keeps the contact mask in an
    `unsigned int`) and no multi-nozzle engines. The CUDA kernel therefore cannot run this craft.
- **Sim pad and hop (`parts::tests::heavy_booster_pad_and_hop`).**
  - The booster settles to LANDED on the six legs (0.21 m static compression, no hull contact).
  - A 3 s full-throttle hop then tracks thrust/mass − g within 0.35 % per tick.

### Joint break forces

`parts::joint_break_force` implements `PartJoint.Create` /
`SetBreakingForces`:

min(breakingForce) · (size + 1) · (stack 2 | srf 0.8) · jointBreakForceFactor 50

- Part default `breakingForce` is 22. Node size is the smaller of the two nodes.
- Nodes of size ≥ 2 get 3 internal joints that split this force. ksp-core stores the sum.
- The formula reproduces both dumped values: 4400 (size-1 stack) and 4000 (LT-2 on the FL-T400).
- For this tree: 2.5 m joints 6600 kN, ADTP-2-3 → S3-14400 80000, S3-7200 → S3-14400 120000,
  Mammoth → S3-7200 120000. These values are pair-dependent.
- The TraceLogger dump logs only `PartJoint.Joint` = `joints[0]`. `trace.rs` multiplies by an
  optional `joints=` field, so adding `" joints=" + p.attachJoint.joints.Count` to the `joint`
  line in TraceLogger.cs makes replays match.

### Not measured in game (from cfg / decomp / .mu only)

1. Mammoth nozzle positions (.mu) and the per-nozzle roll mix (decomp). No trace shows the
   Mammoth's gimbal torque yet; the Swivel's measured torque coefficient was 0.867.
2. All hull radii, the Mammoth crash sensors and nozzle-capsule contacts (.mu colliders, not the
   SimHarness collider dump), and the nose cone's `hull_crash = false` (decomp reading).
3. Joint break forces 6600 / 80000 / 120000 (formula). Only size-1 nodes have been dumped.
4. Drag cubes. They come from the demo install's PartDatabase, and cube generation depends on
   the renderer.
5. Inertia. The sim uses its usual box-from-cube approximation; the game derives it from the
   colliders (`rb.inertiaTensor` is in the dump).
6. Not simulated: ModuleAlternator; ModuleCommand's 0.08 EC/s on the RC-L01.
7. Two catalog joint values are wrong for this craft:
   - `lt2_leg()` carries the FL-T400's 4000 kN; on the S3-7200 the game gives
     min(80, 300) · 2 · 0.8 · 50 = 6400.
   - `airbrake()` has 4400; the game gives min(22, 400) · 2 · 0.8 · 50 = 1760.

   Both only matter for the opt-in joint-break check.

### The short in-game test that would measure them (`HeavyPadHop`)

Fly the heavy craft (`ingame/make_demo_craft.py --heavy`) on the pad with TraceLogger running
(SimHarness CSV + geometry dumps at start and end). Run it at 1x warp, SAS off, with raw
kOS `SHIP:CONTROL` inputs.

1. **Pad settle, 10 s at throttle 0.** The start and end geometry dumps give:
   - `collider … ext=` support points: hull radii, Mammoth `bodyCollider` / `engineCollider`
     extents, the nose cone's active collider and which one is `Part.collider`;
   - four `thrustTransform` and four `gimbalTransform` lines;
   - joint `breakForce` (with `joints=` added), `rb.inertiaTensor`, deployed `wheelCollider`.

   Compare with `trace::spec_from_geometry` against `VesselSpec::compile`. Also compare the
   static leg compression (`wc*_compr`) and the CoM height with the sim's
   `heavy_booster_pad_and_hop` settle.
2. **Hop, 3 s at throttle 0.5** (TWR about 1.23, 2.2 m/s², about 10 m). Compare `e0_thrust`,
   `e0_isp`, mass flow (`lf` / `ox`) and vertical acceleration per tick, as in the
   `powered_hover` trace test.
3. **Gimbal steps** in a kOS vertical-speed hold at about 15 m, with both reaction wheels'
   torque toggled off (`ModuleReactionWheel` "Toggle Torque"). Input steps of 1 s each, 1 s
   apart:
   - pitch +0.5,
   - yaw +0.5,
   - roll +0.5,
   - roll −0.5,
   - pitch +1 with roll +1 (the clamp case).

   Compare the logged `g0_ax/ay/az` and the angular acceleration from `w_*` with the sim torque
   in a `trace::state_at` replay. Roll is the new physics. TraceLogger only logs
   `thrustTransforms[0]` per tick (`e0_dir` / `e0_pos`), so per-nozzle `localRotation` columns
   would make each nozzle's deflection directly checkable.
4. **Descent** at −2 m/s to touchdown, then the end-of-test dump.

## Heavy booster: sim Monte Carlo, in-game validation hop, in-game flights (ingame-demo)

Craft (`guidance/core/src/ingame.rs::heavy_craft`, `ingame/make_demo_craft.py --heavy`): Mk7 nose /
RC-L01 / large reaction wheel / Z-4K / ADTP-2-3 / S3-14400 / S3-7200 / Mammoth, six LT-2 legs on
the S3-7200 offset 2.4 m below it beside the Mammoth (feet 0.43 m below the nozzle exits), eight
airbrakes 3.0 m above the S3-14400 centre. 155.5 t full, 32.5 t dry; flown at 60 % propellant
(106.3 t at liftoff).

Guidance profile (MC-tuned, booster only): ascent capped at TWR 1.8, pitch-over scale 90 km,
MECO when the propellant left is 2x the predicted return need (budget planned at the 0.7 ignition
fraction), long-aim 2 % of downrange, airbrakes out at boostback cutoff, landing burn ignition at
0.3 of the stop throttle (earlier and gentler: time to divert), touchdown 1.5 m/s; if still > 40 m
off the pad at 100 m above it, descend vertically and land where it is. Without the TWR cap and
margin the light booster flew to 45 km / 1950 m/s / 370 km apogee (unmodelled reentry heating).

Sim (nominal ascent; thrust error ~2 %, drag error ~5 %, separation kicks, time of day varied):

| run | landed | miss p50 / p90 / p99 / max (m) | touchdown vz p99 / max (m/s) |
|---|---|---|---|
| 2000 flights | 2000 | 0.75 / 0.99 / 1.27 / 1.58 | 1.65 / 1.78 |
| 1000 through the bridge mapping (`MC_BRIDGE`) | 1000 | 0.72 / 0.94 / 1.20 / 1.43 | 1.65 / 1.70 |
| 1000, drag error x2 | 1000 | 0.76 / 1.14 / 1.43 / 1.59 | 1.65 / 1.72 |

The ~0.7 m median is a systematic offset along the approach (west): the final ZEM/ZEV segment is
deliberately slow for this slow-turning craft (time to go floored at 3 attitude swings).

In-game validation hop (`BRIDGE_HOP=1`, trace flight_20261104_184826; `guidance/core/examples/trace_check.rs`):
settle, climb to 15 m, hover with pitch / yaw / roll steps, 1.5 m/s descent. Game vs ksp-core,
airborne 33 s: one-step CoM acceleration error median 1.5e-4 / p95 2.1e-3 m/s^2, angular 8e-5 /
2e-3 rad/s^2; 1-s open-loop windows max 1.8 mm, 0.065 deg; touchdown on the same tick, 2.4 mm.
Mass and CoM identical; inertia diag game (2359, 182, 2353) vs repo craft (2402, 261, 2408) t m^2
(roll axis 43 % high in the transcription: open item, no effect on the windows).

In-game flights through the bridge (three takes, identical to the cm): separation 21.1 km at
~455 m/s vertical, boostback, apogee ~38 km, two burns, landing burn from ~2 km; touchdown 1.27-1.34
m/s vertical, tilt 0.08 deg, CoM at rest 0.70 m from the pad centre (e -0.69, n -0.05), as the sim
predicts. Not validated in game yet: Mach > 0.8 airbrake drag (the descent peaks ~Mach 1.9 at 20 km).
