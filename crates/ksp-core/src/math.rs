//! Minimal f64 vector / quaternion math. Plain `#[repr(C)]` PODs so the same layout can be
//! mirrored 1:1 in CUDA kernels later.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[inline(always)]
pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    #[inline(always)]
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline(always)]
    pub fn cross(self, o: V3) -> V3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    #[inline(always)]
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }
    #[inline(always)]
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }
    #[inline(always)]
    pub fn normalized(self) -> V3 {
        let n = self.norm();
        if n > 0.0 {
            self / n
        } else {
            V3::ZERO
        }
    }
    /// KSP's `.xzy` swizzle (Unity left-handed <-> KSP "Zup" planetarium frame).
    #[inline(always)]
    pub fn xzy(self) -> V3 {
        v3(self.x, self.z, self.y)
    }
    #[inline(always)]
    pub fn mul_elem(self, o: V3) -> V3 {
        v3(self.x * o.x, self.y * o.y, self.z * o.z)
    }
}

impl Add for V3 {
    type Output = V3;
    #[inline(always)]
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl AddAssign for V3 {
    #[inline(always)]
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}
impl Sub for V3 {
    type Output = V3;
    #[inline(always)]
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl SubAssign for V3 {
    #[inline(always)]
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    #[inline(always)]
    fn mul(self, s: f64) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<V3> for f64 {
    type Output = V3;
    #[inline(always)]
    fn mul(self, v: V3) -> V3 {
        v * self
    }
}
impl Div<f64> for V3 {
    type Output = V3;
    #[inline(always)]
    fn div(self, s: f64) -> V3 {
        v3(self.x / s, self.y / s, self.z / s)
    }
}
impl Neg for V3 {
    type Output = V3;
    #[inline(always)]
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

/// Unit quaternion (x, y, z, w), Hamilton convention, same component layout as Unity.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Quat {
        Quat { x, y, z, w }
    }
    /// Unity `Quaternion.AngleAxis(deg, axis)`.
    pub fn angle_axis(deg: f64, axis: V3) -> Quat {
        let a = axis.normalized();
        let h = deg.to_radians() * 0.5;
        let s = h.sin();
        Quat::new(a.x * s, a.y * s, a.z * s, h.cos())
    }
    /// Unity `Quaternion.FromToRotation(from, to)` (shortest arc).
    pub fn from_to(from: V3, to: V3) -> Quat {
        let a = from.normalized();
        let b = to.normalized();
        let c = a.cross(b);
        let d = a.dot(b);
        if d < -1.0 + 1e-15 {
            // 180 deg: any orthogonal axis
            let ortho = if a.x.abs() < 0.9 { v3(1.0, 0.0, 0.0) } else { v3(0.0, 1.0, 0.0) };
            return Quat::angle_axis(180.0, a.cross(ortho));
        }
        Quat::new(c.x, c.y, c.z, 1.0 + d).normalized()
    }
    pub fn normalized(self) -> Quat {
        let n = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        Quat::new(self.x / n, self.y / n, self.z / n, self.w / n)
    }
    pub fn conj(self) -> Quat {
        Quat::new(-self.x, -self.y, -self.z, self.w)
    }
    #[inline(always)]
    pub fn rotate(self, v: V3) -> V3 {
        let u = v3(self.x, self.y, self.z);
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }
    /// Integrate body angular velocity `w` (world frame, rad/s) over `dt`.
    #[inline(always)]
    pub fn integrate_world(self, w: V3, dt: f64) -> Quat {
        let ang = w.norm() * dt;
        if ang < 1e-300 {
            return self;
        }
        let dq = Quat::angle_axis(ang.to_degrees(), w);
        (dq * self).normalized()
    }
}

impl Mul for Quat {
    type Output = Quat;
    #[inline(always)]
    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat::new(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unity_angle_axis_convention() {
        // Unity: Quaternion.AngleAxis(90, Vector3.up) * Vector3.forward == Vector3.right
        let r = Quat::angle_axis(90.0, v3(0.0, 1.0, 0.0)).rotate(v3(0.0, 0.0, 1.0));
        assert!((r - v3(1.0, 0.0, 0.0)).norm() < 1e-12);
    }
    #[test]
    fn from_to_maps() {
        let a = v3(0.0, 1.0, 0.0);
        let b = v3(0.3, -0.2, 0.9).normalized();
        assert!((Quat::from_to(a, b).rotate(a) - b).norm() < 1e-12);
    }
}
