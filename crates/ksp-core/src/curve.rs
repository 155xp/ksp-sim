//! `FloatCurve` (KSP) == Unity `AnimationCurve` evaluation.
//!
//! Source: `FloatCurve.Evaluate` (decomp FloatCurve.cs:86) forwards to `AnimationCurve.Evaluate`.
//! Unity evaluates non-weighted keyframes with a cubic Hermite segment whose tangents are scaled
//! by the segment width, and clamps outside the key range (serialized m_PreInfinity /
//! m_PostInfinity = 2 = clamp for every stock curve; checked by tools/gen_bodies_rs.py).
//! An infinite tangent on either side makes the segment a step (holds the left value).
//!
//! Keys loaded from a cfg `key = t v` line (two values) go through `FloatCurve.Add(t, v)` ->
//! `AnimationCurve.AddKey(time, value)` (FloatCurve.cs:67), which computes "smooth" tangents for
//! the *newly inserted key only* (Unity `RecalculateSplineSlope(curve, index, bias = 0)`): first
//! key -> slope to its right neighbour, last key -> slope from its left neighbour, interior ->
//! mean of the two secant slopes. Previously inserted keys keep the slope they got when they were
//! inserted. `key = t v in out` (four values) uses the explicit tangents (FloatCurve.cs:74).
//!
//! Unity evaluates in f32; we evaluate in f64 (relative difference ~1e-7).

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Key {
    pub t: f64,
    pub v: f64,
    pub tin: f64,
    pub tout: f64,
}

pub const fn k(t: f64, v: f64, tin: f64, tout: f64) -> Key {
    Key { t, v, tin, tout }
}

/// Evaluate a curve given as a sorted key slice.
#[inline]
pub fn eval(keys: &[Key], x: f64) -> f64 {
    let n = keys.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 || x <= keys[0].t {
        return keys[0].v;
    }
    if x >= keys[n - 1].t {
        return keys[n - 1].v;
    }
    // binary search for the segment [i, i+1] containing x
    let (mut lo, mut hi) = (0usize, n - 1);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if keys[mid].t <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hermite(&keys[lo], &keys[hi], x)
}

#[inline(always)]
fn hermite(a: &Key, b: &Key, x: f64) -> f64 {
    if !a.tout.is_finite() || !b.tin.is_finite() {
        return a.v;
    }
    let dx = b.t - a.t;
    let t = (x - a.t) / dx;
    let m0 = a.tout * dx;
    let m1 = b.tin * dx;
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    h00 * a.v + h10 * m0 + h01 * b.v + h11 * m1
}

/// Owned curve built the way `FloatCurve.Load` builds one from a ConfigNode.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Curve {
    pub keys: Vec<Key>,
}

impl Curve {
    pub fn from_keys(keys: &[Key]) -> Self {
        Curve { keys: keys.to_vec() }
    }

    /// `FloatCurve.Add(t, v)` -> `AnimationCurve.AddKey(t, v)` (smooth tangent on the new key).
    pub fn add_smooth(&mut self, t: f64, v: f64) {
        let i = self.insert(k(t, v, 0.0, 0.0));
        let n = self.keys.len();
        if n < 2 {
            return;
        }
        let ks = &self.keys;
        let slope = |a: &Key, b: &Key| {
            let dx = b.t - a.t;
            if dx.abs() > 1e-12 { (b.v - a.v) / dx } else { 0.0 }
        };
        let m = if i == 0 {
            slope(&ks[0], &ks[1])
        } else if i == n - 1 {
            slope(&ks[n - 2], &ks[n - 1])
        } else {
            0.5 * slope(&ks[i - 1], &ks[i]) + 0.5 * slope(&ks[i], &ks[i + 1])
        };
        self.keys[i].tin = m;
        self.keys[i].tout = m;
    }

    /// `FloatCurve.Add(t, v, in, out)`.
    pub fn add(&mut self, t: f64, v: f64, tin: f64, tout: f64) {
        self.insert(k(t, v, tin, tout));
    }

    fn insert(&mut self, key: Key) -> usize {
        // AnimationCurve.AddKey refuses a key at an existing time; KSP cfgs never do that.
        let i = self.keys.partition_point(|q| q.t < key.t);
        self.keys.insert(i, key);
        i
    }

    /// Parse the `key = ...` lines of a cfg curve block (as `FloatCurve.Load` does: split on
    /// space/comma/semicolon/tab; 4 values -> explicit tangents, otherwise smooth).
    pub fn from_cfg_lines<'a>(lines: impl IntoIterator<Item = &'a str>) -> Self {
        let mut c = Curve::default();
        for ln in lines {
            let a: Vec<f64> = ln
                .split([' ', ',', ';', '\t'])
                .filter(|s| !s.is_empty())
                .map(|s| s.parse().expect("bad float in curve key"))
                .collect();
            if a.len() == 4 {
                c.add(a[0], a[1], a[2], a[3]);
            } else if a.len() >= 2 {
                c.add_smooth(a[0], a[1]);
            }
        }
        c
    }

    #[inline]
    pub fn eval(&self, x: f64) -> f64 {
        eval(&self.keys, x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hits_keys_and_clamps() {
        let c = Curve::from_cfg_lines(["0 1 0 0", "0.85 1.19 0.6960422 0.6960422", "1.1 2.83 0.730473 0.730473", "5 4 0 0"]);
        assert_eq!(c.eval(-1.0), 1.0);
        assert_eq!(c.eval(0.0), 1.0);
        assert!((c.eval(0.85) - 1.19).abs() < 1e-12);
        assert!((c.eval(1.1) - 2.83).abs() < 1e-12);
        assert_eq!(c.eval(50.0), 4.0);
    }

    #[test]
    fn hermite_reproduces_cubic() {
        // a single Hermite segment with exact tangents reproduces any cubic exactly
        let f = |x: f64| 0.3 * x * x * x - x * x + 2.0 * x + 1.0;
        let df = |x: f64| 0.9 * x * x - 2.0 * x + 2.0;
        let keys = [k(0.5, f(0.5), df(0.5), df(0.5)), k(2.5, f(2.5), df(2.5), df(2.5))];
        for i in 0..=20 {
            let x = 0.5 + 0.1 * i as f64;
            assert!((eval(&keys, x) - f(x)).abs() < 1e-12);
        }
    }

    #[test]
    fn smooth_tangents_are_insertion_order_dependent() {
        // typical stock atmosphereCurve: "key = 0 345", "key = 1 290", "key = 9 0.001"
        let c = Curve::from_cfg_lines(["0 345", "1 290", "9 0.001"]);
        // first key was alone when inserted -> slope 0; second -> secant from first; third -> secant from second
        assert_eq!(c.keys[0].tout, 0.0);
        assert!((c.keys[1].tin - (-55.0)).abs() < 1e-12);
        assert!((c.keys[2].tin - (0.001 - 290.0) / 8.0).abs() < 1e-12);
        let mid = c.eval(0.5);
        // h00*345 + h01*290 + h11*(-55): t=0.5 -> 0.5*345 + 0.5*290 - 0.125*(-55)
        assert!((mid - (0.5 * 345.0 + 0.5 * 290.0 + 0.125 * 55.0)).abs() < 1e-9);
    }

    #[test]
    fn infinite_tangent_steps() {
        let keys = [k(0.0, 1.0, 0.0, f64::INFINITY), k(1.0, 5.0, 0.0, 0.0)];
        assert_eq!(eval(&keys, 0.7), 1.0);
    }
}
