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

#[pymodule]
fn _ksp_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VecEnv>()?;
    m.add_function(wrap_pyfunction!(kerbin_atmosphere, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_thrust, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_info, m)?)?;
    m.add_function(wrap_pyfunction!(hopper_drop, m)?)?;
    m.add_function(wrap_pyfunction!(point_mass_tables, m)?)?;
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
