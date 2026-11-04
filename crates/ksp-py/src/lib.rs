use pyo3::prelude::*;

#[pymodule]
fn _ksp_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("KERBIN_RADIUS", ksp_core::body::KERBIN_RADIUS)?;
    Ok(())
}
