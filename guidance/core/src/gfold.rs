//! G-FOLD-style powered-descent guidance: the lossless-convexification fuel-optimal landing
//! problem (Acikmese & Ploen 2007; Blackmore, Acikmese & Scharf 2010) solved as an SOCP with
//! Clarabel, with free final time by golden-section search.
//!
//! Variables per node k = 0..N (pad ENU, relative to the touchdown point): r, v, z = ln m, the
//! thrust acceleration u = T/m and its slack sigma >= |u|. Dynamics are exact for a
//! piecewise-linear u under constant gravity; Coriolis enters linearly (-2 Omega x v) and the
//! drag acceleration along the previous solution is a known per-node term (one re-linearisation
//! on the first solve, carried between re-plans). Constraints: |u| <= sigma, the linearised
//! thrust upper bound in z, the mass lower bound, a thrust pointing cone, a glide-slope cone and
//! r_z >= 0. Objective: maximise final mass. If the pinpoint problem is infeasible, the
//! minimum-miss problem (Blackmore's problem 3) is solved instead.

use crate::nav::{atm_at, AeroTables};
use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettingsBuilder, DefaultSolver, IPSolver, SolverStatus, SupportedConeT};
use ksp_core::ksc;
use ksp_core::math::{v3, V3};

/// Thrust the planner may use (the rest is tracking margin).
pub const THRUST_MARGIN: f64 = 0.85;
pub const REPLAN_DT: f64 = 1.0;
/// Below this time-to-go the terminal (baseline) law takes over.
pub const TERMINAL_TGO: f64 = 1.5;
pub const N_NODES: usize = 24;
/// Max thrust tilt from vertical and glide-slope elevation (deg).
pub const POINTING_DEG: f64 = 35.0;
pub const GLIDE_DEG: f64 = 8.0;

pub struct Problem<'a> {
    pub r0: V3,
    pub v0: V3,
    pub m0: f64,
    pub m_dry: f64,
    /// thrust bounds (kN) and mass flow per kN (t/s/kN = 1 / (Isp g0))
    pub t_max: f64,
    pub t_min: f64,
    pub alpha: f64,
    /// gravity + centrifugal (pad ENU, m/s^2)
    pub g: V3,
    pub v_td: f64,
    /// the plan ends this high above the touchdown point (descending at v_td); the terminal
    /// law flies the rest
    pub h_end: f64,
    /// terminal vertical descent: horizontal position and velocity are zero for the last
    /// `t_vert` seconds of the plan (attitude lag of the craft; capped at a third of the plan)
    pub t_vert: f64,
    pub tables: &'a AeroTables,
    pub toff: f64,
    pub omega_enu: V3,
}

#[derive(Clone, Debug)]
pub struct Plan {
    pub t0: f64,
    pub tf: f64,
    pub r: Vec<V3>,
    pub v: Vec<V3>,
    pub u: Vec<V3>,
    pub d: Vec<V3>,
    pub z: Vec<f64>,
    pub g: V3,
    pub omega_enu: V3,
    pub pinpoint: bool,
}

impl Plan {
    pub fn t_go(&self, t: f64) -> f64 {
        self.tf - (t - self.t0)
    }
    /// Planned non-thrust acceleration at velocity v with planned drag d.
    pub fn natural(&self, v: V3, d: V3) -> V3 {
        self.g + d - self.omega_enu.cross(v) * 2.0
    }
    /// (r, v, u, drag) interpolated at absolute time t.
    pub fn at(&self, t: f64) -> (V3, V3, V3, V3) {
        let n = self.r.len() - 1;
        let s = ((t - self.t0) / self.tf * n as f64).clamp(0.0, n as f64 - 1e-9);
        let k = s as usize;
        let f = s - k as f64;
        let l = |a: &Vec<V3>| a[k] + (a[k + 1] - a[k]) * f;
        (l(&self.r), l(&self.v), l(&self.u), l(&self.d))
    }
}

const NV: usize = 11;
#[inline]
fn ix(k: usize, j: usize) -> usize {
    NV * k + j
}
// node layout: r 0..3, v 3..6, z 6, u 7..10, s 10

struct Build {
    rows: Vec<usize>,
    cols: Vec<usize>,
    vals: Vec<f64>,
    b: Vec<f64>,
    m: usize,
}
impl Build {
    fn new() -> Self {
        Build { rows: vec![], cols: vec![], vals: vec![], b: vec![], m: 0 }
    }
    fn row(&mut self, entries: &[(usize, f64)], rhs: f64) {
        for &(c, v) in entries {
            if v != 0.0 {
                self.rows.push(self.m);
                self.cols.push(c);
                self.vals.push(v);
            }
        }
        self.b.push(rhs);
        self.m += 1;
    }
}

/// Drag acceleration (pad ENU) at node state (r, v, u, m) from the craft tables.
fn drag_at(p: &Problem, r: V3, v: V3, u: V3, m: f64, h_ref: f64) -> V3 {
    let spd = v.norm();
    if spd < 1e-3 {
        return V3::ZERO;
    }
    let alt = ksc::KSC_PAD_DECK_ALT + h_ref + r.z.max(0.0);
    let atm = atm_at(alt, spd, p.toff);
    if atm.rho <= 0.0 {
        return V3::ZERO;
    }
    let vh = v / spd;
    let alpha = if u.norm() > 1e-6 { (-vh).dot(u.normalized()).clamp(-1.0, 1.0).acos().to_degrees() } else { 0.0 };
    vh * (-(atm.q_kpa * p.tables.cd(atm.mach, alpha.min(30.0)) * atm.pseudo_re_drag_mult) / m)
}

/// Solve the fixed-final-time problem. Returns (plan, final mass) or None if infeasible.
fn solve_fixed(p: &Problem, tf: f64, d: &[V3], pinpoint: bool) -> Option<Plan> {
    let n = N_NODES;
    let dt = tf / n as f64;
    let nx = NV * (n + 1) + usize::from(!pinpoint);
    let ev = NV * (n + 1); // miss epigraph variable (min-miss problem)
    let mut eq = Build::new();
    let mut ineq = Build::new();
    let mut socs: Vec<(Build, usize)> = vec![];
    // initial state
    for j in 0..3 {
        eq.row(&[(ix(0, j), 1.0)], [p.r0.x, p.r0.y, p.r0.z][j]);
        eq.row(&[(ix(0, 3 + j), 1.0)], [p.v0.x, p.v0.y, p.v0.z][j]);
    }
    eq.row(&[(ix(0, 6), 1.0)], p.m0.ln());
    // Coriolis matrix C v = -2 omega x v
    let w = p.omega_enu * 2.0;
    let cmat = [[0.0, w.z, -w.y], [-w.z, 0.0, w.x], [w.y, -w.x, 0.0]];
    for k in 0..n {
        let gk = p.g + d[k];
        let gv = [gk.x, gk.y, gk.z];
        for j in 0..3 {
            // v_{k+1} - v_k - dt/2 (u_k + u_{k+1}) - dt/2 C (v_k + v_{k+1}) = dt g_k
            let mut e = vec![(ix(k + 1, 3 + j), 1.0), (ix(k, 3 + j), -1.0), (ix(k, 7 + j), -0.5 * dt), (ix(k + 1, 7 + j), -0.5 * dt)];
            for c in 0..3 {
                if cmat[j][c] != 0.0 {
                    e.push((ix(k, 3 + c), -0.5 * dt * cmat[j][c]));
                    e.push((ix(k + 1, 3 + c), -0.5 * dt * cmat[j][c]));
                }
            }
            // merge duplicate column entries (diagonal of C is zero, so none here)
            eq.row(&e, dt * gv[j]);
            // r_{k+1} - r_k - dt/2 (v_k + v_{k+1}) - dt^2/12 (u_k - u_{k+1}) = 0
            eq.row(&[(ix(k + 1, j), 1.0), (ix(k, j), -1.0), (ix(k, 3 + j), -0.5 * dt), (ix(k + 1, 3 + j), -0.5 * dt), (ix(k, 7 + j), -dt * dt / 12.0), (ix(k + 1, 7 + j), dt * dt / 12.0)], 0.0);
        }
        // z_{k+1} - z_k + alpha dt/2 (s_k + s_{k+1}) = 0
        eq.row(&[(ix(k + 1, 6), 1.0), (ix(k, 6), -1.0), (ix(k, 10), 0.5 * p.alpha * dt), (ix(k + 1, 10), 0.5 * p.alpha * dt)], 0.0);
    }
    // terminal vertical descent
    if pinpoint {
        let kv = n - ((p.t_vert.min(tf / 3.0) / dt).round() as usize).min(n);
        for k in kv..n {
            eq.row(&[(ix(k, 0), 1.0)], 0.0);
            eq.row(&[(ix(k, 1), 1.0)], 0.0);
            eq.row(&[(ix(k, 3), 1.0)], 0.0);
            eq.row(&[(ix(k, 4), 1.0)], 0.0);
        }
    }
    // final state
    if pinpoint {
        eq.row(&[(ix(n, 0), 1.0)], 0.0);
        eq.row(&[(ix(n, 1), 1.0)], 0.0);
    }
    eq.row(&[(ix(n, 2), 1.0)], p.h_end);
    eq.row(&[(ix(n, 3), 1.0)], 0.0);
    eq.row(&[(ix(n, 4), 1.0)], 0.0);
    eq.row(&[(ix(n, 5), 1.0)], -p.v_td);
    let cos_p = POINTING_DEG.to_radians().cos();
    let tan_g = GLIDE_DEG.to_radians().tan();
    for k in 0..=n {
        let t = k as f64 * dt;
        let m_lo = (p.m0 - p.alpha * p.t_max * t).max(p.m_dry * 0.5);
        let z0 = m_lo.ln();
        let ez = (-z0).exp();
        // sigma <= T_max e^-z0 (1 - (z - z0))
        ineq.row(&[(ix(k, 10), 1.0), (ix(k, 6), p.t_max * ez)], p.t_max * ez * (1.0 + z0));
        // sigma >= T_min e^-z0 (1 - (z - z0))   (tangent of the convex bound)
        if p.t_min > 0.0 {
            ineq.row(&[(ix(k, 10), -1.0), (ix(k, 6), -p.t_min * ez)], -p.t_min * ez * (1.0 + z0));
        }
        // z >= z0(t) (mass can't drop faster than at full thrust), z_N >= ln m_dry
        ineq.row(&[(ix(k, 6), -1.0)], -z0);
        // pointing: u_up >= sigma cos(theta)
        ineq.row(&[(ix(k, 9), -1.0), (ix(k, 10), cos_p)], 0.0);
        // above the end height
        if k > 0 {
            ineq.row(&[(ix(k, 2), -1.0)], -p.h_end.min(p.r0.z));
        }
        // |u| <= sigma
        let mut s = Build::new();
        s.row(&[(ix(k, 10), -1.0)], 0.0);
        for j in 0..3 {
            s.row(&[(ix(k, 7 + j), -1.0)], 0.0);
        }
        socs.push((s, 4));
        // glide slope (skip the first nodes: the start may sit outside the cone)
        if k >= n / 3 {
            let mut s = Build::new();
            s.row(&[(ix(k, 2), -1.0 / tan_g)], -p.h_end / tan_g);
            s.row(&[(ix(k, 0), -1.0)], 0.0);
            s.row(&[(ix(k, 1), -1.0)], 0.0);
            socs.push((s, 3));
        }
    }
    ineq.row(&[(ix(n, 6), -1.0)], -p.m_dry.ln());
    if !pinpoint {
        let mut s = Build::new();
        s.row(&[(ev, -1.0)], 0.0);
        s.row(&[(ix(n, 0), -1.0)], 0.0);
        s.row(&[(ix(n, 1), -1.0)], 0.0);
        socs.push((s, 3));
    }
    // assemble A = [eq; ineq; socs...]
    let (mut rows, mut cols, mut vals, mut b) = (vec![], vec![], vec![], vec![]);
    let mut off = 0;
    let mut cones = vec![];
    let mut push = |blk: &Build, off: &mut usize| {
        for i in 0..blk.vals.len() {
            rows.push(blk.rows[i] + *off);
            cols.push(blk.cols[i]);
            vals.push(blk.vals[i]);
        }
        b.extend_from_slice(&blk.b);
        *off += blk.m;
    };
    push(&eq, &mut off);
    cones.push(SupportedConeT::ZeroConeT(eq.m));
    push(&ineq, &mut off);
    cones.push(SupportedConeT::NonnegativeConeT(ineq.m));
    for (s, dim) in &socs {
        push(s, &mut off);
        cones.push(SupportedConeT::SecondOrderConeT(*dim));
    }
    let a = CscMatrix::new_from_triplets(off, nx, rows, cols, vals);
    let mut q = vec![0.0; nx];
    q[ix(n, 6)] = -1.0;
    if !pinpoint {
        q[ix(n, 6)] = -1e-3;
        q[ev] = 1.0;
    }
    let pm = CscMatrix::zeros((nx, nx));
    let settings = DefaultSettingsBuilder::default().verbose(false).max_iter(60).build().ok()?;
    let mut solver = DefaultSolver::new(&pm, &q, &a, &b, &cones, settings).ok()?;
    solver.solve();
    let st = solver.solution.status;
    if !matches!(st, SolverStatus::Solved | SolverStatus::AlmostSolved) {
        return None;
    }
    let x = &solver.solution.x;
    let g3 = |k: usize, o: usize| v3(x[ix(k, o)], x[ix(k, o + 1)], x[ix(k, o + 2)]);
    Some(Plan {
        t0: 0.0,
        tf,
        r: (0..=n).map(|k| g3(k, 0)).collect(),
        v: (0..=n).map(|k| g3(k, 3)).collect(),
        u: (0..=n).map(|k| g3(k, 7)).collect(),
        d: d.to_vec(),
        z: (0..=n).map(|k| x[ix(k, 6)]).collect(),
        g: p.g,
        omega_enu: p.omega_enu,
        pinpoint,
    })
}

fn drag_profile(p: &Problem, prev: Option<&Plan>, tf: f64, h_ref: f64) -> Vec<V3> {
    let n = N_NODES;
    (0..=n)
        .map(|k| {
            let t = k as f64 * tf / n as f64;
            match prev {
                Some(pl) => {
                    let (r, v, u, _) = pl.at(pl.t0 + t * pl.tf / tf);
                    let m = pl.z[(k * (pl.z.len() - 1)) / n].exp();
                    drag_at(p, r, v, u, m, h_ref)
                }
                None => {
                    // straight-line guess from the initial state to rest
                    let f = t / tf;
                    let r = p.r0 * (1.0 - f) * (1.0 - f);
                    let v = p.v0 * (1.0 - f);
                    drag_at(p, r, v, v3(0.0, 0.0, 1.0), p.m0, h_ref)
                }
            }
        })
        .collect()
}

fn fuel_cost(p: &Problem, tf: f64, prev: Option<&Plan>, h_ref: f64) -> (f64, Option<Plan>) {
    let d = drag_profile(p, prev, tf, h_ref);
    match solve_fixed(p, tf, &d, true) {
        Some(pl) => (-pl.z[N_NODES], Some(pl)),
        None => (1e6 - tf, None),
    }
}

/// Plan from the problem; `warm_tf` is the previous plan's remaining time (narrows the search).
pub fn plan(p: &Problem, warm_tf: Option<f64>) -> Option<Plan> {
    let h_ref = 0.0;
    let vz = p.v0.z.min(-0.5);
    let tgo = 2.0 * (p.r0.z - p.h_end).max(1.0) / (-vz + p.v_td);
    let (mut lo, mut hi) = match warm_tf {
        Some(t) => (0.8 * t, 1.25 * t),
        None => (0.5 * tgo, 2.5 * tgo),
    };
    let iters = if warm_tf.is_some() { 5 } else { 9 };
    let gr = 0.5 * (5f64.sqrt() - 1.0);
    let mut c = hi - gr * (hi - lo);
    let mut d = lo + gr * (hi - lo);
    let (mut fc, mut pc) = fuel_cost(p, c, None, h_ref);
    let (mut fd, mut pd) = fuel_cost(p, d, None, h_ref);
    for _ in 0..iters {
        if fc < fd {
            hi = d;
            d = c;
            fd = fc;
            pd = pc;
            c = hi - gr * (hi - lo);
            (fc, pc) = fuel_cost(p, c, None, h_ref);
        } else {
            lo = c;
            c = d;
            fc = fd;
            pc = pd;
            d = lo + gr * (hi - lo);
            (fd, pd) = fuel_cost(p, d, None, h_ref);
        }
    }
    let (best_t, best) = if fc < fd { (c, pc) } else { (d, pd) };
    match best {
        Some(pl) => {
            // one drag re-linearisation along the solution
            let dd = drag_profile(p, Some(&pl), best_t, h_ref);
            solve_fixed(p, best_t, &dd, true).or(Some(pl))
        }
        None => {
            // pinpoint infeasible over the bracket: minimum-miss landing at the upper time
            let d0 = drag_profile(p, None, hi, h_ref);
            solve_fixed(p, hi, &d0, false)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ksp_core::vessel::{Craft, VesselSpec};

    #[test]
    fn plans_a_divert() {
        let spec = VesselSpec::compile(&Craft::hopper());
        let tables = AeroTables::build(&spec, &spec.full_resources());
        let p = Problem {
            r0: v3(150.0, -80.0, 1500.0),
            v0: v3(-10.0, 5.0, -120.0),
            m0: 4.0,
            m_dry: 2.9,
            t_max: 167.0 * THRUST_MARGIN,
            t_min: 0.0,
            alpha: 1.0 / (250.0 * 9.80665),
            g: v3(0.0, 0.0, -9.79),
            v_td: 1.5,
            h_end: 0.0,
            t_vert: 2.0,
            tables: &tables,
            toff: 0.0,
            omega_enu: v3(0.0, 0.0, 0.0),
        };
        let t = std::time::Instant::now();
        let pl = plan(&p, None).expect("plan");
        println!("tf {:.2} s, m_f {:.4} t, pinpoint {}, {:.1} ms", pl.tf, pl.z[N_NODES].exp(), pl.pinpoint, t.elapsed().as_secs_f64() * 1e3);
        assert!(pl.pinpoint);
        assert!(pl.r[N_NODES].norm() < 1e-3);
        assert!(pl.z[N_NODES].exp() > 2.9);
    }
}
