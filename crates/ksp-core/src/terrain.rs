//! Ground model around the KSC launch pad, sampled from the game's actual collision geometry.
//!
//! The SimHarness plugin (harness/plugin, `Survey` test) Physics.Raycasts straight down on two
//! East/North grids centred on the LaunchPad spawn point against layer 15 (PQS terrain + KSC
//! colliders: pad deck, apron, buildings): 0.2 m spacing within +-40 m, 4 m within +-1.5 km.
//! `tools/terrain_bin.py` packs the hit altitudes into `assets/ksc_terrain.bin`.
//! Lookup is bilinear in the grid; outside both grids the ground is the sphere at `fallback_alt`.

use crate::math::{v3, V3};
use std::sync::OnceLock;

pub struct Grid {
    pub step: f64,
    pub half: f64,
    pub n: usize,
    /// altitude above the body radius, row-major [ie * n + in]; NaN = no hit
    pub alt: Vec<f32>,
    /// 1 where the hit collider is PQS terrain ("Ground" PhysicMaterial), 0 for KSC colliders
    pub ground: Vec<u8>,
}

pub struct Terrain {
    pub radius: f64,
    pub origin: V3,
    pub east: V3,
    pub north: V3,
    pub up: V3,
    pub grids: Vec<Grid>,
    pub fallback_alt: f64,
}

impl Grid {
    #[inline]
    fn sample(&self, e: f64, n: f64) -> Option<f64> {
        let fe = (e + self.half) / self.step;
        let fnn = (n + self.half) / self.step;
        if fe < 0.0 || fnn < 0.0 {
            return None;
        }
        let (ie, inn) = (fe.floor() as usize, fnn.floor() as usize);
        if ie + 1 >= self.n || inn + 1 >= self.n {
            return None;
        }
        let (te, tn) = (fe - ie as f64, fnn - inn as f64);
        let a = |i: usize, j: usize| self.alt[i * self.n + j] as f64;
        let (h00, h10, h01, h11) = (a(ie, inn), a(ie + 1, inn), a(ie, inn + 1), a(ie + 1, inn + 1));
        if !(h00.is_finite() && h10.is_finite() && h01.is_finite() && h11.is_finite()) {
            return None;
        }
        Some((h00 * (1.0 - te) + h10 * te) * (1.0 - tn) + (h01 * (1.0 - te) + h11 * te) * tn)
    }
}

impl Terrain {
    /// Ground altitude (m above the body radius) under body-fixed point `p`.
    #[inline]
    pub fn alt_at(&self, p: V3) -> f64 {
        let d = p - self.origin;
        let (e, n) = (d.dot(self.east), d.dot(self.north));
        for g in &self.grids {
            if let Some(h) = g.sample(e, n) {
                return h;
            }
        }
        self.fallback_alt
    }

    /// Contact friction for a part collider (no PhysicMaterial = Unity default 0.6, Average) on the
    /// ground at `p`: PQS terrain ("Ground", 0.9, Maximum wins) -> 0.9, KSC colliders -> 0.6.
    pub fn friction_at(&self, p: V3) -> f64 {
        let d = p - self.origin;
        let (e, n) = (d.dot(self.east), d.dot(self.north));
        for g in &self.grids {
            let fe = ((e + g.half) / g.step).round();
            let fnn = ((n + g.half) / g.step).round();
            if fe >= 0.0 && fnn >= 0.0 && (fe as usize) < g.n && (fnn as usize) < g.n {
                return if g.ground[fe as usize * g.n + fnn as usize] == 1 { 0.9 } else { 0.6 };
            }
        }
        0.9
    }

    /// Height of `p` above the ground (negative = below).
    #[inline]
    pub fn height_above(&self, p: V3) -> f64 {
        p.norm() - self.radius - self.alt_at(p)
    }

    /// Outward surface normal at `p` (finite differences of the heightfield).
    pub fn normal_at(&self, p: V3) -> V3 {
        let r = p.normalized();
        let d = p - self.origin;
        let (e, n) = (d.dot(self.east), d.dot(self.north));
        let step = self.grids.iter().find(|g| e.abs() < g.half - g.step && n.abs() < g.half - g.step).map_or(1.0, |g| g.step);
        let h = |de: f64, dn: f64| self.alt_at(p + self.east * de + self.north * dn);
        let dhde = (h(step, 0.0) - h(-step, 0.0)) / (2.0 * step);
        let dhdn = (h(0.0, step) - h(0.0, -step)) / (2.0 * step);
        (r - self.east * dhde - self.north * dhdn).normalized()
    }

    /// Ray `a - s u` (s >= 0) against the ground: returns (s, hit point, normal) if s <= len.
    pub fn ray(&self, a: V3, u: V3, len: f64) -> Option<(f64, V3, V3)> {
        // start from the hit with the sphere at the local ground altitude, then re-evaluate the
        // altitude at the hit point a few times (piecewise-linear ground)
        let mut s = 0.0;
        for _ in 0..4 {
            let p = a - u * s;
            let rg = self.radius + self.alt_at(p);
            let au = a.dot(u);
            let disc = au * au - (a.norm2() - rg * rg);
            if disc < 0.0 {
                return None;
            }
            s = au - disc.sqrt();
        }
        if s < 0.0 || s > len {
            return None;
        }
        let p = a - u * s;
        Some((s, p, self.normal_at(p)))
    }

    pub fn load(bytes: &[u8]) -> Option<Terrain> {
        let mut o = 0;
        let mut rd = |nb: usize| {
            let s = &bytes[o..o + nb];
            o += nb;
            s
        };
        let magic = rd(8);
        let v2 = magic == b"KSCTERR2";
        if magic != b"KSCTERR1" && !v2 {
            return None;
        }
        let f64s = |b: &[u8]| f64::from_le_bytes(b.try_into().unwrap());
        let radius = f64s(rd(8));
        let mut v = [0.0; 12];
        for x in v.iter_mut() {
            *x = f64s(rd(8));
        }
        let fallback_alt = f64s(rd(8));
        let ng = u32::from_le_bytes(rd(4).try_into().unwrap()) as usize;
        let mut grids = vec![];
        for _ in 0..ng {
            let step = f64s(rd(8));
            let half = f64s(rd(8));
            let n = u32::from_le_bytes(rd(4).try_into().unwrap()) as usize;
            let raw = rd(n * n * 4);
            let alt = raw.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
            let ground = if v2 { rd(n * n).to_vec() } else { vec![0; n * n] };
            grids.push(Grid { step, half, n, alt, ground });
        }
        Some(Terrain {
            radius,
            origin: v3(v[0], v[1], v[2]),
            east: v3(v[3], v[4], v[5]),
            north: v3(v[6], v[7], v[8]),
            up: v3(v[9], v[10], v[11]),
            grids,
            fallback_alt,
        })
    }
}

static KSC: OnceLock<Option<Terrain>> = OnceLock::new();

/// The surveyed KSC terrain: `$KSP_SIM_TERRAIN`, else `assets/ksc_terrain.bin` next to the crate.
pub fn ksc() -> Option<&'static Terrain> {
    KSC.get_or_init(|| {
        let path = std::env::var("KSP_SIM_TERRAIN")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/ksc_terrain.bin"));
        std::fs::read(path).ok().and_then(|b| Terrain::load(&b))
    })
    .as_ref()
}
