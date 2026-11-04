"""GPU path: reduced point-mass lander (3-DOF) with the full KSP atmosphere, on CUDA via cupy.

Same physics as the Rust core for a CoM-only vessel:
  * Kerbin pressure / temperature FloatCurves (Unity Hermite, clamped), sun-mult temperature offset
    (per-env constant), density P*M/(R*T), speed of sound sqrt(gamma*P/rho)    [atmo.rs]
  * Swivel Isp(p) curve, thrust = maxFuelFlow * throttle * Isp * g0, propellant mass flow    [engine.rs]
  * gravity -GM r/|r|^3 + Coriolis -2 W x v + centrifugal -W x (W x r), body-fixed frame       [sim.rs]
  * drag = q * CdA(mach) * DRAG_PSEUDOREYNOLDS(rho v), CdA(mach) of the hopper measured from the
    full 6-DOF sim (axial flow) with the pseudo-Reynolds factor divided out
  * PhysX-style semi-implicit Euler at dt = 0.02 s
What it drops vs the 6-DOF core: attitude dynamics (the thrust direction is commanded directly),
per-part aero / body lift, legs and contacts (touchdown = CoM reaching leg height), velocity smoothing.

Positions are stored relative to the pad deck point so float32 keeps ~mm resolution near the pad;
altitude is formed without cancellation from precomputed (|pad|^2 - R^2).
"""
from __future__ import annotations

import os

import numpy as np

if "CUDA_PATH" not in os.environ and os.path.exists("/usr/include/cuda_runtime.h"):
    os.environ["CUDA_PATH"] = "/usr"

from ._ksp_sim import point_mass_tables

_SRC = r"""
typedef REAL real;
struct Key { real t, v, tin, tout; };

__device__ __forceinline__ real hermite_eval(const Key* k, int n, real x) {
    if (x <= k[0].t) return k[0].v;
    if (x >= k[n-1].t) return k[n-1].v;
    int i = 0;
    while (i < n - 2 && x >= k[i+1].t) ++i;
    real dx = k[i+1].t - k[i].t;
    real t = (x - k[i].t) / dx;
    real t2 = t*t, t3 = t2*t;
    return (2*t3 - 3*t2 + 1) * k[i].v + (t3 - 2*t2 + t) * k[i].tout * dx
         + (-2*t3 + 3*t2) * k[i+1].v + (t3 - t2) * k[i+1].tin * dx;
}

__device__ __forceinline__ real interp(const real* xs, const real* ys, int n, real x) {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n-1]) return ys[n-1];
    int i = 0;
    while (i < n - 2 && x >= xs[i+1]) ++i;
    real f = (x - xs[i]) / (xs[i+1] - xs[i]);
    return ys[i] + f * (ys[i+1] - ys[i]);
}

struct Consts {
    real pad[3];        // pad deck position (body-fixed), only used through pad_dot / pad2_minus_r2
    real up[3];         // pad up (for diagnostics)
    real pad2_minus_r2; // |pad|^2 - R^2
    real radius, gm, omega, depth, molar, gamma, rgas, g0, mdot_max, dry_mass, ground_h;
    int np, nt, ns, ni, nc, nr;
};

extern "C" __global__ void step_kernel(
    const Consts* __restrict__ cc, const Key* __restrict__ pk, const Key* __restrict__ tk,
    const Key* __restrict__ sk, const Key* __restrict__ ik, const Key* __restrict__ rk, const real* __restrict__ cm,
    const real* __restrict__ ca,
    real* __restrict__ px, real* __restrict__ py, real* __restrict__ pz,
    real* __restrict__ vx, real* __restrict__ vy, real* __restrict__ vz,
    real* __restrict__ fuel, const real* __restrict__ toff, unsigned char* __restrict__ done,
    const real* __restrict__ act, int n, int substeps, real dt)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    if (done[i]) return;
    const Consts c = *cc;
    real p[3] = {px[i], py[i], pz[i]};
    real v[3] = {vx[i], vy[i], vz[i]};
    real f = fuel[i];
    real thr = act[4*i]; thr = thr < 0 ? 0 : (thr > 1 ? 1 : thr);
    real d[3] = {act[4*i+1], act[4*i+2], act[4*i+3]};
    real dn = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    if (dn > 0) { d[0] /= dn; d[1] /= dn; d[2] /= dn; }
    real to = toff[i];
    unsigned char dn_flag = 0;
    for (int s = 0; s < substeps; ++s) {
        // r = pad + p ; alt = |r| - R without cancellation
        real r[3] = {c.pad[0] + p[0], c.pad[1] + p[1], c.pad[2] + p[2]};
        real pp = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];
        real padp = c.pad[0]*p[0] + c.pad[1]*p[1] + c.pad[2]*p[2];
        real r2 = (c.radius*c.radius + c.pad2_minus_r2) + 2*padp + pp;
        real rn = sqrt(r2);
        real alt = (c.pad2_minus_r2 + 2*padp + pp) / (rn + c.radius);
        // atmosphere (CelestialBody.GetPressure / GetFullTemperature / GetDensity / GetSpeedOfSound)
        real P = 0, rho = 0, snd = 0;
        if (alt < c.depth) {
            P = hermite_eval(pk, c.np, alt);
            real T = hermite_eval(tk, c.nt, alt) + hermite_eval(sk, c.ns, alt) * to;
            rho = P * 1000 * c.molar / (c.rgas * T);
            snd = sqrt(c.gamma * P * 1000 / rho);
        }
        real spd2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
        real spd = sqrt(spd2);
        real m = c.dry_mass + f;
        // engine
        real thrust = 0;
        if (f > 0 && thr > 0) {
            real isp = hermite_eval(ik, c.ni, P * (real)0.009869232667160128);
            real mdot = c.mdot_max * thr;
            real dm = mdot * dt;
            real frac = dm <= f ? 1 : f / dm;
            thrust = mdot * isp * c.g0 * frac;
            f -= dm * frac;
        }
        // drag: q (kPa) * CdA(mach) (kN), opposite velocity
        real drag = 0;
        if (rho > 0 && spd > 0) {
            real q = (real)0.0005 * rho * spd2;
            drag = q * interp(cm, ca, c.nc, spd / snd) * hermite_eval(rk, c.nr, rho * spd);
        }
        real ir3 = -c.gm / (r2 * rn);
        real a[3];
        for (int k = 0; k < 3; ++k) {
            a[k] = (thrust * d[k] - (spd > 0 ? drag * v[k] / spd : 0)) / m + ir3 * r[k];
        }
        // Coriolis -2 W x v and centrifugal -W x (W x r), W = (0, 0, omega)
        real w = c.omega;
        a[0] += 2*w*v[1] + w*w*r[0];
        a[1] += -2*w*v[0] + w*w*r[1];
        for (int k = 0; k < 3; ++k) { v[k] += a[k] * dt; p[k] += v[k] * dt; }
        if (alt <= c.ground_h) { dn_flag = 1; break; }
    }
    px[i] = p[0]; py[i] = p[1]; pz[i] = p[2];
    vx[i] = v[0]; vy[i] = v[1]; vz[i] = v[2];
    fuel[i] = f;
    done[i] = dn_flag;
}
"""


def _np_hermite(keys, x):
    t, v, tin, tout = keys.T
    x = np.asarray(x, dtype=np.float64)
    i = np.clip(np.searchsorted(t, x, side="right") - 1, 0, len(t) - 2)
    dx = t[i + 1] - t[i]
    u = (x - t[i]) / dx
    u2, u3 = u * u, u * u * u
    y = (2 * u3 - 3 * u2 + 1) * v[i] + (u3 - 2 * u2 + u) * tout[i] * dx + (-2 * u3 + 3 * u2) * v[i + 1] + (u3 - u2) * tin[i + 1] * dx
    return np.where(x <= t[0], v[0], np.where(x >= t[-1], v[-1], y))


class PointMassBatch:
    """N point-mass landers stepped on the GPU (cupy) or with numpy (reference, float64).

    State is pad-relative body-fixed position p (m), body-fixed velocity v (m/s), propellant mass (t).
    Action (N, 4): throttle in [0, 1] and a thrust direction 3-vector (body-fixed, normalised inside).
    """

    def __init__(self, n: int, backend: str = "cuda", dtype=np.float32, substeps: int = 1, block: int = 256):
        self.n, self.backend, self.substeps, self.block = n, backend, substeps, block
        self.dtype = np.dtype(dtype)
        t = point_mass_tables()
        self.t = t
        pad = np.array(t["pad"])
        self.enu = np.array(t["enu"])
        self.ground_h = t["pad_deck_alt"] + t["com_height_at_touchdown"]
        self.consts = dict(pad=pad, up=self.enu[2], pad2_minus_r2=float(pad @ pad - t["radius"] ** 2))
        if backend == "cuda":
            import cupy as cp

            self.xp = cp
            real = "float" if self.dtype == np.float32 else "double"
            self._mod = cp.RawModule(code=_SRC.replace("REAL", real), options=("--use_fast_math",) if real == "float" else ())
            self._k = self._mod.get_function("step_kernel")
            vals = list(pad) + list(self.enu[2]) + [self.consts["pad2_minus_r2"], t["radius"], t["gm"], t["omega"],
                    t["atmosphere_depth"], t["molar_mass"], t["adiabatic_index"], t["gas_constant"], t["g0"],
                    t["max_fuel_flow"], t["dry_mass"], self.ground_h]
            ints = [len(t["pressure_keys"]), len(t["temperature_keys"]), len(t["temp_sun_mult_keys"]), len(t["isp_keys"]), len(t["cda"]), len(t["pseudo_re_keys"])]
            reals = np.array(vals, dtype=self.dtype).tobytes()
            pad_to = (len(reals) + 3) // 4 * 4
            blob = reals + b"\0" * (pad_to - len(reals)) + np.array(ints, dtype=np.int32).tobytes()
            self._cc = cp.asarray(np.frombuffer(blob, dtype=np.uint8))
            self._keys = [cp.asarray(np.ascontiguousarray(t[k], dtype=self.dtype)) for k in
                          ("pressure_keys", "temperature_keys", "temp_sun_mult_keys", "isp_keys", "pseudo_re_keys")]
            self._cm = cp.asarray(t["cda_mach"].astype(self.dtype))
            self._ca = cp.asarray(t["cda"].astype(self.dtype))
        else:
            self.xp = np
            self.dtype = np.dtype(np.float64)
        xp = self.xp
        self.p = [xp.zeros(n, self.dtype) for _ in range(3)]
        self.v = [xp.zeros(n, self.dtype) for _ in range(3)]
        self.fuel = xp.zeros(n, self.dtype)
        self.toff = xp.zeros(n, self.dtype)
        self.done = xp.zeros(n, np.uint8)

    def reset(self, enu_pos, enu_vel, fuel=None, temp_offset=0.0):
        """Set state from pad-ENU positions / velocities, arrays of shape (n, 3)."""
        xp = self.xp
        P = np.asarray(enu_pos, np.float64) @ self.enu
        V = np.asarray(enu_vel, np.float64) @ self.enu
        for k in range(3):
            self.p[k][...] = xp.asarray(P[:, k].astype(self.dtype))
            self.v[k][...] = xp.asarray(V[:, k].astype(self.dtype))
        wet, dry = self.t["wet_mass"], self.t["dry_mass"]
        f = np.full(self.n, wet - dry) if fuel is None else np.asarray(fuel, np.float64)
        self.fuel[...] = xp.asarray(f.astype(self.dtype))
        self.toff[...] = xp.asarray(np.broadcast_to(np.asarray(temp_offset, np.float64), (self.n,)).astype(self.dtype))
        self.done[...] = 0

    def step(self, actions):
        if self.backend == "cuda":
            a = self.xp.asarray(actions, dtype=self.dtype)
            grid = ((self.n + self.block - 1) // self.block,)
            self._k(grid, (self.block,), (self._cc, *self._keys, self._cm, self._ca, *self.p, *self.v,
                                          self.fuel, self.toff, self.done, a, np.int32(self.n),
                                          np.int32(self.substeps), self.dtype.type(0.02)))
        else:
            self._step_numpy(np.asarray(actions, np.float64))

    def _step_numpy(self, a):
        t = self.t
        pad = self.consts["pad"]
        thr = np.clip(a[:, 0], 0, 1)
        d = a[:, 1:4]
        dn = np.linalg.norm(d, axis=1, keepdims=True)
        d = np.where(dn > 0, d / np.where(dn > 0, dn, 1), 0)
        live = self.done == 0
        p = np.stack(self.p, 1)
        v = np.stack(self.v, 1)
        f = self.fuel.copy()
        for _ in range(self.substeps):
            r = pad + p
            pp = (p * p).sum(1)
            padp = p @ pad
            r2 = t["radius"] ** 2 + self.consts["pad2_minus_r2"] + 2 * padp + pp
            rn = np.sqrt(r2)
            alt = (self.consts["pad2_minus_r2"] + 2 * padp + pp) / (rn + t["radius"])
            inatm = alt < t["atmosphere_depth"]
            P = np.where(inatm, _np_hermite(t["pressure_keys"], alt), 0.0)
            T = _np_hermite(t["temperature_keys"], alt) + _np_hermite(t["temp_sun_mult_keys"], alt) * self.toff
            rho = np.where(inatm, P * 1000 * t["molar_mass"] / (t["gas_constant"] * T), 0.0)
            snd = np.where(inatm, np.sqrt(t["adiabatic_index"] * P * 1000 / np.where(rho > 0, rho, 1)), 1.0)
            spd2 = (v * v).sum(1)
            spd = np.sqrt(spd2)
            m = t["dry_mass"] + f
            isp = _np_hermite(t["isp_keys"], P * 0.009869232667160128)
            mdot = t["max_fuel_flow"] * thr
            dm = mdot * 0.02
            frac = np.where(f > 0, np.where(dm <= f, 1.0, f / np.where(dm > 0, dm, 1)), 0.0)
            thrust = np.where((f > 0) & (thr > 0), mdot * isp * t["g0"] * frac, 0.0)
            f = f - np.where((f > 0) & (thr > 0), dm * frac, 0.0)
            cda = np.interp(spd / snd, t["cda_mach"], t["cda"]) * _np_hermite(t["pseudo_re_keys"], rho * spd)
            drag = np.where((rho > 0) & (spd > 0), 0.0005 * rho * spd2 * cda, 0.0)
            acc = (thrust[:, None] * d - np.where(spd[:, None] > 0, drag[:, None] * v / np.where(spd > 0, spd, 1)[:, None], 0)) / m[:, None]
            acc += (-t["gm"] / (r2 * rn))[:, None] * r
            w = t["omega"]
            acc[:, 0] += 2 * w * v[:, 1] + w * w * r[:, 0]
            acc[:, 1] += -2 * w * v[:, 0] + w * w * r[:, 1]
            vn = v + acc * 0.02
            pn = p + vn * 0.02
            v = np.where(live[:, None], vn, v)
            p = np.where(live[:, None], pn, p)
            f = np.where(live, f, self.fuel)
            live = live & ~(alt <= self.ground_h)
        for k in range(3):
            self.p[k][...] = p[:, k]
            self.v[k][...] = v[:, k]
        self.fuel[...] = f
        self.done[...] = (~live).astype(np.uint8)

    def enu_state(self):
        """(pos, vel) in pad ENU as float64 numpy arrays (n, 3)."""
        get = (lambda a: a.get()) if self.backend == "cuda" else (lambda a: a)
        P = np.stack([get(x).astype(np.float64) for x in self.p], 1) @ self.enu.T
        V = np.stack([get(x).astype(np.float64) for x in self.v], 1) @ self.enu.T
        return P, V
