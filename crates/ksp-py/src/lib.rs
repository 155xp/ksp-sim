//! Python bindings: a batched env whose numpy buffers are written in place by Rust
//! (no per-step allocation, GIL released while stepping), plus physics helpers for validation.

use ksp_core::batch::{Batch, StepOut, TaskConfig, ACT_DIM, OBS_DIM};
use ksp_core::sim::{self, Params};
use ksp_core::vessel::{Craft, VesselSpec};
use ksp_core::{atmo, body, engine, ksc};
use numpy::{PyArray1, PyArray2, PyArrayMethods, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::prelude::*;
use pyo3::types::PyDict;

#[pyclass(module = "ksp_sim._ksp_sim")]
struct VecEnv {
    batch: Batch,
    obs: Py<PyArray2<f32>>,
    reward: Py<PyArray1<f32>>,
    terminated: Py<PyArray1<u8>>,
    truncated: Py<PyArray1<u8>>,
    outcome: Py<PyArray1<u8>>,
    final_dist: Py<PyArray1<f32>>,
    final_speed: Py<PyArray1<f32>>,
    final_return: Py<PyArray1<f32>>,
}

fn task_from_kwargs(kw: Option<&Bound<'_, PyDict>>) -> PyResult<(TaskConfig, Params)> {
    let mut c = TaskConfig::default();
    let mut p = Params::default();
    if let Some(kw) = kw {
        for (k, v) in kw.iter() {
            let k: String = k.extract()?;
            match k.as_str() {
                "substeps" => c.substeps = v.extract()?,
                "max_steps" => c.max_steps = v.extract()?,
                "alt_min" => c.alt_min = v.extract()?,
                "alt_max" => c.alt_max = v.extract()?,
                "radius" => c.radius = v.extract()?,
                "vdown_min" => c.vdown_min = v.extract()?,
                "vdown_max" => c.vdown_max = v.extract()?,
                "vh_max" => c.vh_max = v.extract()?,
                "tilt_max_deg" => c.tilt_max_deg = v.extract()?,
                "w_max" => c.w_max = v.extract()?,
                "fuel_min" => c.fuel_min = v.extract()?,
                "fuel_max" => c.fuel_max = v.extract()?,
                "ut_max" => c.ut_max = v.extract()?,
                "oob_radius" => c.oob_radius = v.extract()?,
                "oob_alt" => c.oob_alt = v.extract()?,
                "ground_alt" => p.ground_alt = v.extract()?,
                "friction" => p.friction = v.extract()?,
                "sun_temperature" => p.sun_temperature = v.extract()?,
                "aero" => p.aero = v.extract()?,
                _ => return Err(pyo3::exceptions::PyTypeError::new_err(format!("unknown option {k}"))),
            }
        }
    }
    Ok((c, p))
}

#[pymethods]
impl VecEnv {
    #[new]
    #[pyo3(signature = (num_envs, seed = 0, **kwargs))]
    fn new(py: Python<'_>, num_envs: usize, seed: u64, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Self> {
        let (cfg, prm) = task_from_kwargs(kwargs)?;
        let spec = VesselSpec::compile(&Craft::hopper());
        let n = num_envs;
        Ok(VecEnv {
            batch: Batch::new(spec, prm, cfg, n, seed),
            obs: PyArray2::zeros(py, [n, OBS_DIM], false).unbind(),
            reward: PyArray1::zeros(py, n, false).unbind(),
            terminated: PyArray1::zeros(py, n, false).unbind(),
            truncated: PyArray1::zeros(py, n, false).unbind(),
            outcome: PyArray1::zeros(py, n, false).unbind(),
            final_dist: PyArray1::zeros(py, n, false).unbind(),
            final_speed: PyArray1::zeros(py, n, false).unbind(),
            final_return: PyArray1::zeros(py, n, false).unbind(),
        })
    }

    #[getter]
    fn num_envs(&self) -> usize {
        self.batch.len()
    }

    /// Reset all envs. Returns the (shared, in-place updated) observation buffer.
    fn reset<'py>(&mut self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let obs = self.obs.bind(py);
        let mut o = unsafe { obs.as_array_mut() };
        let s = o.as_slice_mut().unwrap();
        self.batch.reset(s);
        obs.clone()
    }

    /// Step with actions (N, 4) float32: throttle [0,1], pitch, yaw, roll [-1,1].
    /// Returns (obs, reward, terminated, truncated, outcome, final_dist, final_speed, final_return);
    /// all are buffers owned by the env and overwritten by the next call.
    #[allow(clippy::type_complexity)]
    fn step<'py>(&mut self, py: Python<'py>, actions: PyReadonlyArray2<'py, f32>) -> PyResult<PyObject> {
        let n = self.batch.len();
        let a = actions.as_array();
        if a.shape() != [n, ACT_DIM] {
            return Err(pyo3::exceptions::PyValueError::new_err(format!("actions must be ({n}, {ACT_DIM}), got {:?}", a.shape())));
        }
        let a = a.as_standard_layout();
        let act = a.as_slice().unwrap();
        macro_rules! buf {
            ($f:ident) => {
                unsafe { self.$f.bind(py).as_array_mut() }
            };
        }
        let (mut o, mut r, mut te, mut tr, mut oc, mut fd, mut fs, mut fr) =
            (buf!(obs), buf!(reward), buf!(terminated), buf!(truncated), buf!(outcome), buf!(final_dist), buf!(final_speed), buf!(final_return));
        let out = StepOut {
            obs: o.as_slice_mut().unwrap(),
            reward: r.as_slice_mut().unwrap(),
            terminated: te.as_slice_mut().unwrap(),
            truncated: tr.as_slice_mut().unwrap(),
            outcome: oc.as_slice_mut().unwrap(),
            final_dist: fd.as_slice_mut().unwrap(),
            final_speed: fs.as_slice_mut().unwrap(),
            final_return: fr.as_slice_mut().unwrap(),
        };
        let batch = &mut self.batch;
        py.allow_threads(|| batch.step(act, out));
        Ok((
            self.obs.clone_ref(py),
            self.reward.clone_ref(py),
            self.terminated.clone_ref(py),
            self.truncated.clone_ref(py),
            self.outcome.clone_ref(py),
            self.final_dist.clone_ref(py),
            self.final_speed.clone_ref(py),
            self.final_return.clone_ref(py),
        )
            .into_pyobject(py)?
            .into_any()
            .unbind())
    }

    /// Raw physics state of env i (body-fixed frame unless frame == 1).
    fn env_state<'py>(&self, py: Python<'py>, i: usize) -> PyResult<Bound<'py, PyDict>> {
        let s = self.batch.soa.get(i);
        let d = PyDict::new(py);
        d.set_item("ut", s.ut)?;
        d.set_item("frame", s.frame as u8)?;
        d.set_item("x", [s.x.x, s.x.y, s.x.z])?;
        d.set_item("v", [s.v.x, s.v.y, s.v.z])?;
        d.set_item("q", [s.q.x, s.q.y, s.q.z, s.q.w])?;
        d.set_item("w", [s.w.x, s.w.y, s.w.z])?;
        d.set_item("res", s.res.to_vec())?;
        d.set_item("throttle", s.thr[0])?;
        d.set_item("status", s.status)?;
        Ok(d)
    }
}

/// Kerbin atmosphere at altitudes (m): (pressure kPa, temperature K, density kg/m^3, speed of sound m/s).
/// `temp_offset` is the GetAtmoThermalStats offset (K) applied through atmosphereTemperatureSunMultCurve.
#[pyfunction]
#[pyo3(signature = (alts, temp_offset = 0.0))]
fn kerbin_atmosphere<'py>(py: Python<'py>, alts: PyReadonlyArray1<'py, f64>, temp_offset: f64) -> PyResult<PyObject> {
    let a = alts.as_array();
    let b = body::KERBIN;
    let (mut p, mut t, mut r, mut c) = (vec![], vec![], vec![], vec![]);
    for &h in a.iter() {
        let pk = atmo::pressure_kpa(b, h);
        let tk = if pk > 0.0 { atmo::full_temperature(b, h, temp_offset) } else { atmo::SPACE_TEMPERATURE };
        let rho = atmo::density(b, pk, tk);
        p.push(pk);
        t.push(tk);
        r.push(rho);
        c.push(atmo::speed_of_sound(b, pk, rho));
    }
    Ok((PyArray1::from_vec(py, p), PyArray1::from_vec(py, t), PyArray1::from_vec(py, r), PyArray1::from_vec(py, c)).into_pyobject(py)?.into_any().unbind())
}

/// Hopper engine thrust (kN) at static pressure p (atm), full throttle.
#[pyfunction]
fn hopper_thrust(p_atm: f64) -> f64 {
    let s = VesselSpec::compile(&Craft::hopper());
    let mut res = [1e9; ksp_core::parts::N_RES];
    engine::burn(&s.engines[0], 1.0, p_atm, &mut res, 0.02).0
}

/// Vessel summary of the default hopper: wet/dry mass, vacuum dV, sea-level TWR, etc.
#[pyfunction]
fn hopper_info(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let s = VesselSpec::compile(&Craft::hopper());
    let (wet, _, _) = s.mass_props(&s.full_resources());
    let mut fuel_out = s.full_resources();
    fuel_out[0] = 0.0;
    fuel_out[1] = 0.0;
    let (dry, _, _) = s.mass_props(&fuel_out);
    let e = &s.engines[0];
    let g0 = engine::ENGINE_G0;
    let isp_vac = engine::isp(e, 0.0);
    let isp_asl = engine::isp(e, 1.0);
    let d = PyDict::new(py);
    d.set_item("wet_mass_t", wet)?;
    d.set_item("dry_mass_t", dry)?;
    d.set_item("isp_vac", isp_vac)?;
    d.set_item("isp_asl", isp_asl)?;
    d.set_item("dv_vac", isp_vac * g0 * (wet / dry).ln())?;
    d.set_item("dv_asl", isp_asl * g0 * (wet / dry).ln())?;
    d.set_item("thrust_vac_kN", hopper_thrust(0.0))?;
    d.set_item("thrust_asl_kN", hopper_thrust(1.0))?;
    d.set_item("twr_asl", hopper_thrust(1.0) / (wet * body::G0))?;
    Ok(d)
}

/// Run the full sim for a single hopper with a fixed control for `ticks` 0.02 s ticks from a
/// given pad-ENU position / velocity (vertical attitude). Returns arrays (t, alt, v_up, mach, q).
#[pyfunction]
#[pyo3(signature = (alt, v_up, throttle, ticks, aero = true, sun_temperature = true))]
fn hopper_drop<'py>(py: Python<'py>, alt: f64, v_up: f64, throttle: f64, ticks: usize, aero: bool, sun_temperature: bool) -> PyResult<PyObject> {
    let spec = VesselSpec::compile(&Craft::hopper());
    let prm = Params { aero, ground: false, sun_temperature, ..Params::default() };
    let (e, _n, u) = sim::pad_enu();
    let q = sim::attitude(u, e);
    let mut st = sim::state_at_pad(&spec, ksp_core::math::v3(0.0, 0.0, alt), ksp_core::math::v3(0.0, 0.0, v_up), q, ksp_core::math::V3::ZERO, 0.0);
    let ctl = sim::Control { throttle, ..Default::default() };
    let (mut ts, mut hs, mut vs, mut ms, mut qs) = (vec![], vec![], vec![], vec![], vec![]);
    for _ in 0..ticks {
        let info = sim::tick(&spec, &prm, &mut st, &ctl);
        ts.push(st.ut);
        hs.push(info.alt);
        // velocity after the tick (CoM, surface-relative)
        let (_, com_l, _) = spec.mass_props(&st.res);
        vs.push((st.v + st.w.cross(st.q.rotate(com_l))).dot(u));
        ms.push(info.atm.mach);
        qs.push(info.atm.q_kpa);
    }
    Ok((PyArray1::from_vec(py, ts), PyArray1::from_vec(py, hs), PyArray1::from_vec(py, vs), PyArray1::from_vec(py, ms), PyArray1::from_vec(py, qs)).into_pyobject(py)?.into_any().unbind())
}


fn keys_array<'py>(py: Python<'py>, keys: &[ksp_core::curve::Key]) -> Bound<'py, PyArray2<f64>> {
    let rows: Vec<Vec<f64>> = keys.iter().map(|k| vec![k.t, k.v, k.tin, k.tout]).collect();
    PyArray2::from_vec2(py, &rows).unwrap()
}

/// Everything the reduced point-mass model (CUDA path) needs: curve keys, body/pad constants,
/// hopper engine and mass data, and the hopper's axial CdA(mach) measured from the full sim.
#[pyfunction]
fn point_mass_tables<'py>(py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
    let b = body::KERBIN;
    let spec = VesselSpec::compile(&Craft::hopper());
    let d = PyDict::new(py);
    d.set_item("pressure_keys", keys_array(py, b.pressure_curve))?;
    d.set_item("temperature_keys", keys_array(py, b.temperature_curve))?;
    d.set_item("temp_sun_mult_keys", keys_array(py, b.atmosphere_temperature_sun_mult_curve))?;
    let e = &spec.engines[0];
    d.set_item("isp_keys", keys_array(py, &e.isp_keys[..e.n_isp_keys as usize]))?;
    d.set_item("radius", b.radius)?;
    d.set_item("gm", b.gm())?;
    d.set_item("omega", b.angular_velocity())?;
    d.set_item("atmosphere_depth", b.atmosphere_depth)?;
    d.set_item("molar_mass", b.atm_molar_mass)?;
    d.set_item("adiabatic_index", b.atm_adiabatic_index)?;
    d.set_item("gas_constant", body::IDEAL_GAS_CONSTANT)?;
    d.set_item("g0", engine::ENGINE_G0)?;
    d.set_item("max_fuel_flow", e.max_fuel_flow)?;
    let full = spec.full_resources();
    let mut empty = full;
    empty[0] = 0.0;
    empty[1] = 0.0;
    d.set_item("wet_mass", spec.mass_props(&full).0)?;
    d.set_item("dry_mass", spec.mass_props(&empty).0)?;
    let pad = ksc::KSC_PAD_DECK_BODY_FIXED;
    d.set_item("pad", [pad.x, pad.y, pad.z])?;
    let (ee, nn, uu) = sim::pad_enu();
    d.set_item("enu", [[ee.x, ee.y, ee.z], [nn.x, nn.y, nn.z], [uu.x, uu.y, uu.z]])?;
    d.set_item("pad_deck_alt", ksc::KSC_PAD_DECK_ALT)?;
    // CoM height above the ground when the legs just touch (landing reference)
    let (_, com_l, _) = spec.mass_props(&full);
    d.set_item("com_height_at_touchdown", com_l.y - spec.bottom_y)?;
    // CdA_base(mach) from the full 6-DOF sim (engine-first axial flow at 100 m, sun offset off)
    let prm = Params { ground: false, sun_temperature: false, ..Params::default() };
    let q = sim::attitude(uu, ee);
    let mut machs = vec![];
    let mut cdas = vec![];
    for k in 0..=60 {
        let v = 10.0 + k as f64 * 15.0;
        let mut st = sim::state_at_pad(&spec, ksp_core::math::v3(0.0, 0.0, 100.0), ksp_core::math::v3(0.0, 0.0, -v), q, ksp_core::math::V3::ZERO, 0.0);
        let mut st0 = st;
        let info = sim::tick(&spec, &prm, &mut st, &sim::Control::default());
        sim::tick(&spec, &Params { aero: false, ..prm }, &mut st0, &sim::Control::default());
        let drag = (st.v - st0.v).dot(uu) / sim::FIXED_DT * info.mass;
        machs.push(info.atm.mach);
        // divide out DRAG_PSEUDOREYNOLDS(rho v) so the table is a function of mach only
        cdas.push(drag / info.atm.q_kpa / info.atm.pseudo_re_drag_mult);
    }
    d.set_item("pseudo_re_keys", keys_array(py, ksp_core::aero::DRAG_PSEUDOREYNOLDS))?;
    d.set_item("cda_mach", PyArray1::from_vec(py, machs))?;
    d.set_item("cda", PyArray1::from_vec(py, cdas))?;
    Ok(d)
}

/// CUDA tick inputs: (packed constants f64, C header with index macros).
#[pyfunction]
fn gpu_pack<'py>(py: Python<'py>) -> PyResult<PyObject> {
    let spec = VesselSpec::compile(&Craft::hopper());
    let p = ksp_core::gpu::pack(&spec, &Params::default());
    Ok((PyArray1::from_vec(py, p.data), p.header).into_pyobject(py)?.into_any().unbind())
}

/// Terrain heightfield for the device: (meta f64, alt f32, ground u8) or None.
#[pyfunction]
fn gpu_terrain<'py>(py: Python<'py>) -> PyResult<PyObject> {
    match ksp_core::gpu::terrain_arrays() {
        Some((m, a, g)) => Ok((PyArray1::from_vec(py, m), PyArray1::from_vec(py, a), PyArray1::from_vec(py, g)).into_pyobject(py)?.into_any().unbind()),
        None => Ok(py.None()),
    }
}

/// Packed initial states (N, STATE_FIELDS) from the task reset distribution (seeded).
#[pyfunction]
#[pyo3(signature = (n, seed = 0, **kwargs))]
fn gpu_reset_states<'py>(py: Python<'py>, n: usize, seed: u64, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let (cfg, _) = task_from_kwargs(kwargs)?;
    let spec = VesselSpec::compile(&Craft::hopper());
    let nf = ksp_core::gpu::STATE_FIELDS;
    let mut out = vec![0.0; n * nf];
    for i in 0..n {
        let mut rng = seed ^ (i as u64).wrapping_mul(0xA24BAED4963EE407);
        let st = ksp_core::batch::sample_reset(&spec, &cfg, &mut rng);
        ksp_core::gpu::pack_state(&st, &mut out[i * nf..(i + 1) * nf]);
    }
    Ok(PyArray1::from_vec(py, out).reshape([n, nf])?)
}

/// Step packed states (N, STATE_FIELDS) on the CPU with sim::tick (reference for GPU parity).
#[pyfunction]
fn cpu_step_packed<'py>(py: Python<'py>, states: PyReadonlyArray2<'py, f64>, actions: PyReadonlyArray2<'py, f32>, substeps: usize) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let spec = VesselSpec::compile(&Craft::hopper());
    let prm = Params::default();
    let a = states.as_array();
    let u = actions.as_array();
    let (n, nf) = (a.shape()[0], a.shape()[1]);
    let mut out = vec![0.0; n * nf];
    for i in 0..n {
        let row: Vec<f64> = a.row(i).to_vec();
        let mut st = ksp_core::gpu::unpack_state(&row);
        let ctl = sim::Control { throttle: u[[i, 0]] as f64, pitch: u[[i, 1]] as f64, yaw: u[[i, 2]] as f64, roll: u[[i, 3]] as f64 };
        for _ in 0..substeps {
            if st.status == sim::STATUS_CRASHED {
                break;
            }
            sim::tick(&spec, &prm, &mut st, &ctl);
        }
        ksp_core::gpu::pack_state(&st, &mut out[i * nf..(i + 1) * nf]);
    }
    Ok(PyArray1::from_vec(py, out).reshape([n, nf])?)
}

/// GPU replay of an in-game trace (regression test of the GPU kernels against the game).
/// Returns (pack data, pack header, states (R, STATE_FIELDS) seeded from rows i, actions (R, 4),
/// rows i) for every row with a successor; the pack is built from the trace's own vessel.
#[pyfunction]
fn trace_gpu_cases<'py>(py: Python<'py>, dir: &str, name: &str) -> PyResult<PyObject> {
    let t = ksp_core::trace::Trace::load(std::path::Path::new(dir), name).map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
    let spec = t.spec();
    let p = ksp_core::gpu::pack(&spec, &Params::default());
    let nf = ksp_core::gpu::STATE_FIELDS;
    let rows: Vec<usize> = (sim::VEL_SMOOTH_LEN..t.csv.n - 1).collect();
    let mut st_out = vec![0.0; rows.len() * nf];
    let mut act = vec![0f32; rows.len() * 4];
    for (j, &i) in rows.iter().enumerate() {
        let st = ksp_core::trace::state_at(&t, &spec, i);
        ksp_core::gpu::pack_state(&st, &mut st_out[j * nf..(j + 1) * nf]);
        let c = ksp_core::trace::control(&t, i);
        act[4 * j..4 * j + 4].copy_from_slice(&[c.throttle as f32, c.pitch as f32, c.yaw as f32, c.roll as f32]);
    }
    let r = rows.len();
    let out = (
        PyArray1::from_vec(py, p.data),
        p.header,
        PyArray1::from_vec(py, st_out).reshape([r, nf])?,
        PyArray1::from_vec(py, act).reshape([r, 4])?,
        PyArray1::from_vec(py, rows.iter().map(|&i| i as u64).collect::<Vec<_>>()),
    );
    Ok(out.into_pyobject(py)?.into_any().unbind())
}

/// One-step errors against the game for states stepped one tick from trace rows `rows`:
/// (CoM acceleration error m/s^2, angular acceleration error rad/s^2, any-wheel-grounded flag),
/// for the given stepped states and for the CPU tick from the same seeds.
#[pyfunction]
fn trace_one_step_errors<'py>(py: Python<'py>, dir: &str, name: &str, stepped: PyReadonlyArray2<'py, f64>, rows: PyReadonlyArray1<'py, u64>) -> PyResult<PyObject> {
    let t = ksp_core::trace::Trace::load(std::path::Path::new(dir), name).map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
    let spec = t.spec();
    let prm = Params::default();
    let c = &t.csv;
    let a = stepped.as_array();
    let rows = rows.as_slice()?;
    let nw = (0..8).filter(|k| c.has(&format!("wc{k}_grounded"))).count();
    let (mut ga, mut gw, mut ca, mut cw, mut gr) = (vec![], vec![], vec![], vec![], vec![]);
    let dt = sim::FIXED_DT;
    for (j, &i) in rows.iter().enumerate() {
        let i = i as usize;
        let vg = c.v("vsrf", i + 1);
        let wg = -c.v("w", i + 1);
        let st = ksp_core::gpu::unpack_state(&a.row(j).to_vec());
        let (_, v) = ksp_core::trace::com_state(&spec, &st);
        ga.push((v - vg).norm() / dt);
        gw.push((st.w - wg).norm() / dt);
        let mut sc = ksp_core::trace::state_at(&t, &spec, i);
        sim::tick(&spec, &prm, &mut sc, &ksp_core::trace::control(&t, i));
        let (_, v) = ksp_core::trace::com_state(&spec, &sc);
        ca.push((v - vg).norm() / dt);
        cw.push((sc.w - wg).norm() / dt);
        gr.push((0..nw).any(|k| c.c(&format!("wc{k}_grounded"))[i] > 0.5 || c.c(&format!("wc{k}_grounded"))[i + 1] > 0.5));
    }
    let out = (
        PyArray1::from_vec(py, ga),
        PyArray1::from_vec(py, gw),
        PyArray1::from_vec(py, ca),
        PyArray1::from_vec(py, cw),
        PyArray1::from_vec(py, gr),
    );
    Ok(out.into_pyobject(py)?.into_any().unbind())
}

#[pymodule]
fn _ksp_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VecEnv>()?;
    m.add_function(wrap_pyfunction!(kerbin_atmosphere, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_thrust, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_info, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_drop, m)?)?;
    m.add_function(wrap_pyfunction!(point_mass_tables, m)?)?;
    m.add_function(wrap_pyfunction!(gpu_pack, m)?)?;
    m.add_function(wrap_pyfunction!(gpu_terrain, m)?)?;
    m.add_function(wrap_pyfunction!(gpu_reset_states, m)?)?;
    m.add_function(wrap_pyfunction!(cpu_step_packed, m)?)?;
    m.add_function(wrap_pyfunction!(trace_gpu_cases, m)?)?;
    m.add_function(wrap_pyfunction!(trace_one_step_errors, m)?)?;
    m.add("OBS_DIM", OBS_DIM)?;
    m.add("ACT_DIM", ACT_DIM)?;
    m.add("FIXED_DT", sim::FIXED_DT)?;
    m.add("OUTCOMES", ["none", "landed", "crashed", "out_of_bounds", "timeout"])?;
    m.add("KERBIN_RADIUS", body::KERBIN_RADIUS)?;
    m.add("KERBIN_GM", body::KERBIN_GM)?;
    m.add("KERBIN_SIDEREAL_DAY", body::KERBIN_SIDEREAL_DAY)?;
    m.add("KERBIN_SOI", body::KERBIN_SOI)?;
    m.add("KERBIN_ATMOSPHERE_DEPTH", body::KERBIN_ATMOSPHERE_DEPTH)?;
    m.add("KSC_PAD_LAT_DEG", ksc::KSC_PAD_LAT_DEG)?;
    m.add("KSC_PAD_LON_DEG", ksc::KSC_PAD_LON_DEG)?;
    m.add("KSC_PAD_SPAWN_ALT", ksc::KSC_PAD_SPAWN_ALT)?;
    m.add("KSC_PAD_DECK_ALT", ksc::KSC_PAD_DECK_ALT)?;
    Ok(())
}
