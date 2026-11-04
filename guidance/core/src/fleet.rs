//! Batched Monte Carlo: many flights (each its own booster design and dispersions) stepped in
//! parallel with rayon. Flights are independent, so this is a plain parallel map.

use crate::craft::{build, sample_design, BoosterDesign, Rng};
use crate::mission::{run, sample_dispersion, Flight, GuidanceConfig, Shared};
use rayon::prelude::*;
use std::sync::Arc;

/// Sample `n_designs` designs (optionally one size class), each flown `per_design` times.
pub fn make_fleet(seed: u64, n_designs: usize, per_design: usize, size: Option<u8>, cfg: GuidanceConfig, model_errors: bool) -> Vec<Flight> {
    let mut rng = Rng(seed);
    let designs: Vec<BoosterDesign> = (0..n_designs).map(|_| sample_design(&mut rng, size)).collect();
    let shared: Vec<Arc<Shared>> = designs.par_iter().map(|d| Arc::new(Shared::new(build(d).expect("design builds")))).collect();
    let mut flights = vec![];
    for sh in shared {
        for _ in 0..per_design {
            let disp = sample_dispersion(&mut rng, model_errors);
            flights.push(Flight::new(sh.clone(), disp, cfg));
        }
    }
    flights
}

/// Fly every flight to completion in parallel.
pub fn run_all(flights: &mut [Flight]) {
    flights.par_iter_mut().for_each(run);
}
