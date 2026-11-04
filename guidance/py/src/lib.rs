//! Python bindings for ksp-guidance: a batched Monte Carlo fleet and the terminal-phase RL env.
//! All heavy work runs in Rust with the GIL released (rayon across flights).

use ksp_guidance::craft::{short, BoosterDesign};
use ksp_guidance::fleet::{make_fleet, run_all};
use ksp_guidance::mission::{Flight, GuidanceConfig, LandingMode, Phase, Stats, TERM_OBS_DIM};
use ksp_guidance::nav::{propellant, propulsion, p_atm};
use ksp_guidance::terminal::{TermOut, TerminalBatch, ACT_DIM};
use numpy::{PyArray1, PyArray2, PyArrayMethods, PyReadonlyArray2};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

fn landing_mode(s: &str) -> PyResult<LandingMode> {
    Ok(match s {
        "baseline" => LandingMode::Baseline,
        "gfold" => LandingMode::Gfold,
        "external" => LandingMode::External { residual: false },
        "residual" => LandingMode::External { residual: true },
        _ => return Err(pyo3::exceptions::PyValueError::new_err(format!("unknown landing mode {s}"))),
    })
}

/// Static description of a design (what the Monte Carlo groups by).
fn design_dict<'py>(py: Python<'py>, d: &BoosterDesign, f: &Flight) -> PyResult<Bound<'py, PyDict>> {
    let b = &f.sh.built;
    let dd = PyDict::new(py);
    let (mw, _, _) = b.booster.mass_props(&b.booster.full_resources());
    let (ms, _, _) = b.stack.mass_props(&b.stack.full_resources());
    let t_asl = propulsion(&b.booster, p_atm(0.0)).thrust;
    dd.set_item("label", d.label())?;
    dd.set_item("size", d.size)?;
    dd.set_item("diameter_m", [0.0, 1.25, 2.5, 3.75][d.size as usize])?;
    dd.set_item("engine", short(&d.engine))?;
    dd.set_item("n_engines", d.n_engines)?;
    dd.set_item("n_tanks", d.tanks.len())?;
    dd.set_item("n_legs", d.n_legs)?;
    dd.set_item("booster_wet_t", mw)?;
    dd.set_item("booster_dry_t", f.sh.m_dry)?;
    dd.set_item("stack_t", ms)?;
    dd.set_item("twr_liftoff", t_asl / (ms * 9.81))?;
    dd.set_item("twr_dry", t_asl / (f.sh.m_dry * 9.81))?;
    dd.set_item("fuel_fraction", 1.0 - f.sh.m_dry / mw)?;
    Ok(dd)
}

const STAT_FIELDS: &[&str] = &[
    "sep_t", "sep_alt", "sep_speed", "sep_vh", "sep_vz", "sep_downrange", "sep_mass", "sep_prop_frac", "bb_dv_plan", "bb_dv_used",
    "bb_end_err", "apo_alt", "entry_err", "ign_alt", "ign_speed", "ign_err", "ign_t", "td_t", "td_e", "td_n", "td_vz", "td_vh",
    "td_tilt_deg", "final_e", "final_n", "final_tilt_deg", "prop_left", "prop_cap", "dv_left", "landing_dv", "max_q_descent",
    "gfold_solves", "gfold_fail", "trim_burns", "trim_dv",
];

fn stat(s: &Stats, k: &str) -> f64 {
    match k {
        "sep_t" => s.sep_t, "sep_alt" => s.sep_alt, "sep_speed" => s.sep_speed, "sep_vh" => s.sep_vh, "sep_vz" => s.sep_vz,
        "sep_downrange" => s.sep_downrange, "sep_mass" => s.sep_mass, "sep_prop_frac" => s.sep_prop_frac, "bb_dv_plan" => s.bb_dv_plan,
        "bb_dv_used" => s.bb_dv_used, "bb_end_err" => s.bb_end_err, "apo_alt" => s.apo_alt, "entry_err" => s.entry_err,
        "ign_alt" => s.ign_alt, "ign_speed" => s.ign_speed, "ign_err" => s.ign_err, "ign_t" => s.ign_t, "td_t" => s.td_t,
        "td_e" => s.td_e, "td_n" => s.td_n, "td_vz" => s.td_vz, "td_vh" => s.td_vh, "td_tilt_deg" => s.td_tilt_deg,
        "final_e" => s.final_e, "final_n" => s.final_n, "final_tilt_deg" => s.final_tilt_deg, "prop_left" => s.prop_left,
        "prop_cap" => s.prop_cap, "dv_left" => s.dv_left, "landing_dv" => s.landing_dv, "max_q_descent" => s.max_q_descent,
        "gfold_solves" => s.gfold_solves as f64, "gfold_fail" => s.gfold_fail as f64, "trim_burns" => s.trim_burns as f64,
        "trim_dv" => s.trim_dv,
        _ => f64::NAN,
    }
}

/// A Monte Carlo fleet: `n_designs` random boosters (optionally one size class: 1, 2, 3) each
/// flown `per_design` times with independent dispersions, from the pad through landing.
#[pyclass(module = "ksp_guidance._native")]
struct Fleet {
    flights: Vec<Flight>,
}

#[pymethods]
impl Fleet {
    #[new]
    #[pyo3(signature = (seed = 0, n_designs = 10, per_design = 10, size = 0, landing = "baseline", model_errors = false, record = false, f_ign = 0.7, v_td = 1.5, stop_at_ignition = false))]
    #[allow(clippy::too_many_arguments)]
    fn new(py: Python<'_>, seed: u64, n_designs: usize, per_design: usize, size: u8, landing: &str, model_errors: bool, record: bool, f_ign: f64, v_td: f64, stop_at_ignition: bool) -> PyResult<Self> {
        let cfg = GuidanceConfig { landing: landing_mode(landing)?, f_ign, v_td, record, stop_at_ignition };
        let sz = (size > 0).then_some(size);
        let flights = py.allow_threads(|| make_fleet(seed, n_designs, per_design, sz, cfg, model_errors));
        Ok(Fleet { flights })
    }

    fn __len__(&self) -> usize {
        self.flights.len()
    }

    /// Fly every flight to completion (or to ignition with stop_at_ignition), in parallel.
    fn run(&mut self, py: Python<'_>) {
        let f = &mut self.flights;
        py.allow_threads(|| run_all(f));
    }

    /// Change the landing mode of every flight (e.g. compare baseline / gfold from the same states).
    fn set_landing(&mut self, landing: &str) -> PyResult<()> {
        let m = landing_mode(landing)?;
        for f in self.flights.iter_mut() {
            f.cfg.landing = m;
        }
        Ok(())
    }

    /// Resume flights paused at ignition (stop_at_ignition) with `landing` and fly them out.
    fn resume(&mut self, py: Python<'_>, landing: &str) -> PyResult<()> {
        let m = landing_mode(landing)?;
        for f in self.flights.iter_mut() {
            f.cfg.landing = m;
            f.cfg.stop_at_ignition = false;
            f.paused = false;
        }
        let fl = &mut self.flights;
        py.allow_threads(|| run_all(fl));
        Ok(())
    }

    /// A copy of this fleet (e.g. to fly the same ignition states with two landing laws).
    fn copy(&self) -> Fleet {
        Fleet { flights: self.flights.clone() }
    }

    /// Per-flight results as numpy arrays (outcome code: 1 landed, 2 crashed, 3 tipped over,
    /// 4 timeout, 5 no return; `phase` is the phase the flight ended in).
    fn results<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let d = PyDict::new(py);
        let outc: Vec<u8> = self.flights.iter().map(|f| f.outcome as u8).collect();
        let phase: Vec<u8> = self.flights.iter().map(|f| f.phase as u8).collect();
        let design: Vec<i64> = self.flights.iter().map(|f| std::sync::Arc::as_ptr(&f.sh) as i64).collect();
        // design index: order of first appearance
        let mut ids = std::collections::HashMap::new();
        let didx: Vec<i64> = design.iter().map(|p| { let n = ids.len() as i64; *ids.entry(*p).or_insert(n) }).collect();
        d.set_item("outcome", PyArray1::from_vec(py, outc))?;
        d.set_item("phase", PyArray1::from_vec(py, phase))?;
        d.set_item("design", PyArray1::from_vec(py, didx))?;
        for k in STAT_FIELDS {
            let v: Vec<f64> = self.flights.iter().map(|f| stat(&f.stats, k)).collect();
            d.set_item(*k, PyArray1::from_vec(py, v))?;
        }
        let up: Vec<f64> = self.flights.iter().map(|f| propellant(&f.sh.built.booster, &f.st.res).0).collect();
        d.set_item("prop_now", PyArray1::from_vec(py, up))?;
        Ok(d)
    }

    /// Design descriptions, one dict per distinct design, in `results()["design"]` order.
    fn designs<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let l = PyList::empty(py);
        let mut seen = std::collections::HashSet::new();
        for f in &self.flights {
            let p = std::sync::Arc::as_ptr(&f.sh) as usize;
            if seen.insert(p) {
                l.append(design_dict(py, &f.sh.built.design, f)?)?;
            }
        }
        Ok(l)
    }

    /// Recorded trajectory of flight i: (k, 6) array of t, pad-ENU x, y, z (tangent frame at the
    /// pad), phase code, commanded throttle (needs record=True).
    fn trajectory<'py>(&self, py: Python<'py>, i: usize) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let f = self.flights.get(i).ok_or_else(|| pyo3::exceptions::PyIndexError::new_err("flight index"))?;
        let rows: Vec<Vec<f32>> = f.rec.iter().map(|r| r.to_vec()).collect();
        if rows.is_empty() {
            return Ok(PyArray2::zeros(py, [0, 6], false));
        }
        Ok(PyArray2::from_vec2(py, &rows).unwrap())
    }
}

/// Terminal-phase RL environment over ignition states drawn from `fleet` (which must have been
/// run with stop_at_ignition=True). Observations (N, TERM_OBS_DIM) float32; actions (N, 3) in
/// [-1, 1]: requested non-gravitational acceleration in pad ENU, in units of the craft's max
/// thrust acceleration, added to the classic baseline when residual=True.
#[pyclass(module = "ksp_guidance._native")]
struct TerminalEnv {
    batch: TerminalBatch,
    obs: Py<PyArray2<f32>>,
}

#[pymethods]
impl TerminalEnv {
    #[new]
    #[pyo3(signature = (fleet, num_envs = 256, seed = 0, residual = true, ticks_per_step = 5))]
    fn new(py: Python<'_>, fleet: &Fleet, num_envs: usize, seed: u64, residual: bool, ticks_per_step: u32) -> PyResult<Self> {
        let pool: Vec<Flight> = fleet.flights.iter().filter(|f| f.paused && f.phase == Phase::LandingBurn).cloned().collect();
        if pool.is_empty() {
            return Err(pyo3::exceptions::PyValueError::new_err("fleet has no flights paused at ignition (run with stop_at_ignition=True)"));
        }
        let batch = py.allow_threads(|| TerminalBatch::new(pool, num_envs, seed, residual, ticks_per_step));
        Ok(TerminalEnv { obs: PyArray2::zeros(py, [num_envs, TERM_OBS_DIM], false).unbind(), batch })
    }

    #[getter]
    fn num_envs(&self) -> usize {
        self.batch.len()
    }

    #[getter]
    fn pool_size(&self) -> usize {
        self.batch.pool.len()
    }

    fn reset<'py>(&mut self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let o = self.obs.bind(py);
        let mut a = unsafe { o.as_array_mut() };
        let s = a.as_slice_mut().unwrap();
        let b = &mut self.batch;
        py.allow_threads(|| b.reset(s));
        o.clone()
    }

    /// Returns (obs, reward, done, info) with info = dict of outcome, miss, td_vz, td_vh, fuel
    /// for episodes that ended this step (auto-reset).
    fn step<'py>(&mut self, py: Python<'py>, actions: PyReadonlyArray2<'py, f32>) -> PyResult<PyObject> {
        let n = self.batch.len();
        let a = actions.as_array();
        if a.shape() != [n, ACT_DIM] {
            return Err(pyo3::exceptions::PyValueError::new_err(format!("actions must be ({n}, {ACT_DIM})")));
        }
        let a = a.as_standard_layout().to_owned();
        let act = a.as_slice().unwrap().to_vec();
        let (mut rew, mut done, mut oc, mut miss, mut vz, mut vh, mut fu) = (vec![0f32; n], vec![0u8; n], vec![0u8; n], vec![0f32; n], vec![0f32; n], vec![0f32; n], vec![0f32; n]);
        {
            let o = self.obs.bind(py);
            let mut ob = unsafe { o.as_array_mut() };
            let obs = ob.as_slice_mut().unwrap();
            let b = &mut self.batch;
            py.allow_threads(|| {
                b.step(&act, TermOut { obs, reward: &mut rew, done: &mut done, outcome: &mut oc, miss: &mut miss, td_vz: &mut vz, td_vh: &mut vh, fuel: &mut fu })
            });
        }
        let info = PyDict::new(py);
        info.set_item("outcome", PyArray1::from_vec(py, oc))?;
        info.set_item("miss", PyArray1::from_vec(py, miss))?;
        info.set_item("td_vz", PyArray1::from_vec(py, vz))?;
        info.set_item("td_vh", PyArray1::from_vec(py, vh))?;
        info.set_item("fuel", PyArray1::from_vec(py, fu))?;
        Ok((self.obs.clone_ref(py), PyArray1::from_vec(py, rew), PyArray1::from_vec(py, done), info).into_pyobject(py)?.into_any().unbind())
    }
}

/// Sample `n` random booster designs and return their descriptions (no flights).
#[pyfunction]
#[pyo3(signature = (n, seed = 0, size = 0))]
fn sample_designs<'py>(py: Python<'py>, n: usize, seed: u64, size: u8) -> PyResult<Bound<'py, PyList>> {
    let fl = make_fleet(seed, n, 1, (size > 0).then_some(size), GuidanceConfig::default(), false);
    let l = PyList::empty(py);
    for f in &fl {
        l.append(design_dict(py, &f.sh.built.design, f)?)?;
    }
    Ok(l)
}

/// Number of logical CPUs rayon uses.
#[pyfunction]
fn num_threads() -> usize {
    rayon::current_num_threads()
}

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Fleet>()?;
    m.add_class::<TerminalEnv>()?;
    m.add_function(wrap_pyfunction!(sample_designs, m)?)?;
    m.add_function(wrap_pyfunction!(num_threads, m)?)?;
    m.add("TERM_OBS_DIM", TERM_OBS_DIM)?;
    m.add("ACT_DIM", ACT_DIM)?;
    m.add("OUTCOMES", ["running", "landed", "crashed", "tipped_over", "timeout", "no_return"])?;
    m.add("PHASES", ["ascent", "flip", "boostback", "coast", "descent", "landing_burn", "touchdown", "done"])?;
    Ok(())
}
