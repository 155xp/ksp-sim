// Mixed-precision CUDA port of ksp_core::sim::tick (the fast GPU path; cuda_tick.cu is the
// all-f64 reference that matches the CPU to rounding). Precision follows the game rather than the
// CPU reference: double where KSP/Unity keep double (world position and velocity of the CoM,
// gravity, altitude, resource amounts, the accumulated attitude quaternion), float where Unity /
// PhysX compute in float (part forces, drag cubes, FloatCurve evaluation, gimbal and wheel math,
// inertia, the contact solve, the per-tick rotation increment). On the RTX 4060 Ti f64 runs at
// 1/64 of the f32 rate, so this split is what makes the kernel fast. Validated against the
// in-game traces and against the CPU in tests/test_cuda_parity.py.
// Scope as cuda_tick.cu: rotating (Kerbin-fixed) frame below the 100 km switch, no vacuum orbit
// drift compensation. P (double) and Pf (the same array as float) share the P_* indices.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
struct D3 { double x, y, z; };
struct F3 { float x, y, z; };
__device__ __forceinline__ D3 d3(double x, double y, double z) { D3 r; r.x = x; r.y = y; r.z = z; return r; }
__device__ __forceinline__ F3 f3(float x, float y, float z) { F3 r; r.x = x; r.y = y; r.z = z; return r; }
__device__ __forceinline__ D3 operator+(D3 a, D3 b) { return d3(a.x + b.x, a.y + b.y, a.z + b.z); }
__device__ __forceinline__ D3 operator-(D3 a, D3 b) { return d3(a.x - b.x, a.y - b.y, a.z - b.z); }
__device__ __forceinline__ D3 operator-(D3 a) { return d3(-a.x, -a.y, -a.z); }
__device__ __forceinline__ D3 operator*(D3 a, double s) { return d3(a.x * s, a.y * s, a.z * s); }
__device__ __forceinline__ F3 operator+(F3 a, F3 b) { return f3(a.x + b.x, a.y + b.y, a.z + b.z); }
__device__ __forceinline__ F3 operator-(F3 a, F3 b) { return f3(a.x - b.x, a.y - b.y, a.z - b.z); }
__device__ __forceinline__ F3 operator-(F3 a) { return f3(-a.x, -a.y, -a.z); }
__device__ __forceinline__ F3 operator*(F3 a, float s) { return f3(a.x * s, a.y * s, a.z * s); }
__device__ __forceinline__ F3 operator/(F3 a, float s) { return f3(a.x / s, a.y / s, a.z / s); }
__device__ __forceinline__ D3 dd(F3 a) { return d3(a.x, a.y, a.z); }
__device__ __forceinline__ F3 ff(D3 a) { return f3((float)a.x, (float)a.y, (float)a.z); }
__device__ __forceinline__ double dotd(D3 a, D3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__device__ __forceinline__ float dot(F3 a, F3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__device__ __forceinline__ F3 cross(F3 a, F3 b) { return f3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
__device__ __forceinline__ D3 crossd(D3 a, D3 b) { return d3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
__device__ __forceinline__ float norm(F3 a) { return sqrtf(dot(a, a)); }
__device__ __forceinline__ double normd(D3 a) { return sqrt(dotd(a, a)); }
__device__ __forceinline__ F3 normalized(F3 a) { float n = norm(a); return n > 0.f ? a / n : f3(0, 0, 0); }
__device__ __forceinline__ D3 normalizedd(D3 a) { double n = normd(a); return n > 0.0 ? a * (1.0 / n) : d3(0, 0, 0); }

struct Qf { float x, y, z, w; };
struct Qd { double x, y, z, w; };
__device__ __forceinline__ Qf qf(float x, float y, float z, float w) { Qf q; q.x = x; q.y = y; q.z = z; q.w = w; return q; }
__device__ __forceinline__ Qf qmul(Qf a, Qf b) {
    return qf(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
              a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}
__device__ __forceinline__ Qf qconj(Qf q) { return qf(-q.x, -q.y, -q.z, q.w); }
__device__ __forceinline__ F3 qrot(Qf q, F3 v) {
    F3 u = f3(q.x, q.y, q.z);
    F3 t = cross(u, v) * 2.f;
    return v + t * q.w + cross(u, t);
}

// ---- curves: Unity AnimationCurve (Hermite, clamped), evaluated in float as FloatCurve does ----
__device__ float curve32(const float* k, int n, float x) {
    if (n == 0) return 0.f;
    if (n == 1 || x <= k[0]) return k[1];
    if (x >= k[4 * (n - 1)]) return k[4 * (n - 1) + 1];
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) { int mid = (lo + hi) >> 1; if (k[4 * mid] <= x) lo = mid; else hi = mid; }
    const float* a = k + 4 * lo; const float* b = k + 4 * hi;
    if (!isfinite(a[3]) || !isfinite(b[2])) return a[1];
    float dx = b[0] - a[0];
    float t = (x - a[0]) / dx;
    float m0 = a[3] * dx, m1 = b[2] * dx;
    float t2 = t * t, t3 = t2 * t;
    return (2.f * t3 - 3.f * t2 + 1.f) * a[1] + (t3 - 2.f * t2 + t) * m0 + (-2.f * t3 + 3.f * t2) * b[1] + (t3 - t2) * m1;
}
#define CURVE(name, x) curve32(Pf + P_##name, P_##name##_N, (float)(x))
// f64 evaluation for the (every 25 ticks) temperature-offset block, as in cuda_tick.cu
__device__ double eval_curve(const double* k, int n, double x) {
    if (n == 0) return 0.0;
    if (n == 1 || x <= k[0]) return k[1];
    if (x >= k[4 * (n - 1)]) return k[4 * (n - 1) + 1];
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) { int mid = (lo + hi) / 2; if (k[4 * mid] <= x) lo = mid; else hi = mid; }
    const double* a = k + 4 * lo; const double* b = k + 4 * hi;
    if (!isfinite(a[3]) || !isfinite(b[2])) return a[1];
    double dx = b[0] - a[0];
    double t = (x - a[0]) / dx;
    double m0 = a[3] * dx, m1 = b[2] * dx;
    double t2 = t * t, t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * a[1] + (t3 - 2.0 * t2 + t) * m0 + (-2.0 * t3 + 3.0 * t2) * b[1] + (t3 - t2) * m1;
}
#define CURVED(name, x) ((double)(float)eval_curve(P + P_##name, P_##name##_N, (double)(float)(x)))

__device__ float body_lift_curve32(const float* Pf, float x) {
    const float* k = Pf + P_BLIFT; int n = P_BLIFT_N;
    if (x <= k[0]) return k[1];
    if (x >= k[4 * (n - 1)]) return k[4 * (n - 1) + 1];
    int i = 0;
    while (i < n - 2 && x >= k[4 * (i + 1)]) i++;
    const float* a = k + 4 * i; const float* b = k + 4 * (i + 1);
    float dx = b[0] - a[0];
    float t = (x - a[0]) / dx;
    float t2 = t * t, t3 = t2 * t;
    return (2.f * t3 - 3.f * t2 + 1.f) * a[1] + (t3 - 2.f * t2 + t) * a[3] * dx + (-2.f * t3 + 3.f * t2) * b[1] + (t3 - t2) * b[2] * dx;
}

// ---- terrain heightfield (double: world positions are ~600 km from the body centre) ----------
struct Terrain { const double* meta; const float* alt; const float* altr; const unsigned char* gr; int ok; };
__device__ bool grid_sample(const Terrain& T, int gi, double e, double n, double kdz, double* out) {
    const double* g = T.meta + 15 + 4 * gi;
    double step = g[0], half = g[1]; int N = (int)g[2]; int off = (int)g[3];
    double fe = (e + half) / step, fn = (n + half) / step;
    if (fe < 0.0 || fn < 0.0) return false;
    int ie = (int)floor(fe), in = (int)floor(fn);
    if (ie + 1 >= N || in + 1 >= N) return false;
    double te = fe - ie, tn = fn - in;
    const float* a = T.alt + off;
    const unsigned char* gf = T.gr + off;  // KSC cells (flag 0) raised by kdz
    double h00 = a[ie * N + in] + (gf[ie * N + in] ? 0.0 : kdz), h10 = a[(ie + 1) * N + in] + (gf[(ie + 1) * N + in] ? 0.0 : kdz),
           h01 = a[ie * N + in + 1] + (gf[ie * N + in + 1] ? 0.0 : kdz), h11 = a[(ie + 1) * N + in + 1] + (gf[(ie + 1) * N + in + 1] ? 0.0 : kdz);
    if (!(isfinite(h00) && isfinite(h10) && isfinite(h01) && isfinite(h11))) return false;
    *out = (h00 * (1.0 - te) + h10 * te) * (1.0 - tn) + (h01 * (1.0 - te) + h11 * te) * tn;
    return true;
}
__device__ double terrain_alt(const Terrain& T, D3 p, double kdz) {
    D3 o = d3(T.meta[0], T.meta[1], T.meta[2]), E = d3(T.meta[3], T.meta[4], T.meta[5]), N = d3(T.meta[6], T.meta[7], T.meta[8]);
    D3 d = p - o; double e = dotd(d, E), n = dotd(d, N);
    int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) { double h; if (grid_sample(T, g, e, n, kdz, &h)) return h; }
    return T.meta[13];
}
__device__ double ground_height(const double* P, const Terrain& T, D3 p, double kdz) {
    if (T.ok) return normd(p) - T.meta[12] - terrain_alt(T, p, kdz);
    return normd(p) - P[P_RADIUS] - P[P_GROUND_ALT] - kdz;
}
__device__ D3 ground_normal(const double* P, const Terrain& T, D3 p, double kdz) {
    if (!T.ok) return normalizedd(p);
    D3 r = normalizedd(p);
    D3 o = d3(T.meta[0], T.meta[1], T.meta[2]), E = d3(T.meta[3], T.meta[4], T.meta[5]), N = d3(T.meta[6], T.meta[7], T.meta[8]);
    D3 d = p - o; double e = dotd(d, E), n = dotd(d, N);
    double step = 1.0; int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) { const double* gg = T.meta + 15 + 4 * g; if (fabs(e) < gg[1] - gg[0] && fabs(n) < gg[1] - gg[0]) { step = gg[0]; break; } }
    double h1 = terrain_alt(T, p + E * step, kdz), h2 = terrain_alt(T, p - E * step, kdz), h3 = terrain_alt(T, p + N * step, kdz), h4 = terrain_alt(T, p - N * step, kdz);
    double dhde = (h1 - h2) / (2.0 * step), dhdn = (h3 - h4) / (2.0 * step);
    return normalizedd(r - E * dhde - N * dhdn);
}
__device__ float ground_friction(const Terrain& T, D3 p) {
    if (!T.ok) return 0.6f;
    D3 o = d3(T.meta[0], T.meta[1], T.meta[2]), E = d3(T.meta[3], T.meta[4], T.meta[5]), N = d3(T.meta[6], T.meta[7], T.meta[8]);
    D3 d = p - o; double e = dotd(d, E), n = dotd(d, N);
    int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) {
        const double* gg = T.meta + 15 + 4 * g;
        double fe = round((e + gg[1]) / gg[0]), fn = round((n + gg[1]) / gg[0]); int Nn = (int)gg[2];
        if (fe >= 0.0 && fn >= 0.0 && (int)fe < Nn && (int)fn < Nn) return T.gr[(int)gg[3] + (int)fe * Nn + (int)fn] == 1 ? 0.9f : 0.6f;
    }
    return 0.9f;
}
// ray a - s u, s in [0, len]
__device__ bool ground_ray(const double* P, const Terrain& T, D3 a, D3 u, double len, double kdz, double* s_out, D3* p_out, D3* n_out) {
    double s = 0.0;
    if (T.ok) {
        for (int it = 0; it < 4; it++) {
            D3 p = a - u * s;
            double rg = T.meta[12] + terrain_alt(T, p, kdz);
            double au = dotd(a, u);
            double disc = au * au - (dotd(a, a) - rg * rg);
            if (disc < 0.0) return false;
            s = au - sqrt(disc);
        }
    } else {
        double rg = P[P_RADIUS] + P[P_GROUND_ALT] + kdz;
        double au = dotd(a, u);
        double disc = au * au - (dotd(a, a) - rg * rg);
        if (disc < 0.0) return false;
        s = au - sqrt(disc);
    }
    if (s < 0.0 || s > len) return false;
    *s_out = s; *p_out = a - u * s; *n_out = ground_normal(P, T, *p_out, kdz);
    return true;
}

// ---- terrain in float, local frame at the terrain origin o (the pad) --------------------------
// Query points are d = p - o in float (|d| <= 1.5 km: ulp <= 1.2e-4 m, <= 4e-6 m on the fine pad
// grid). Heights are relative to the origin's height h0 = |o| - R: the grid altitudes arrive as
// talt_rel = talt - h0 (exact in float near the pad), and |o + d| - |o| = (2u + w/|o|) /
// (sqrt(1 + eps) + 1), u = o_hat.d, w = |d|^2, eps = (2u + w/|o|)/|o|, so no float value near the
// contact carries the 72 m (or 600 km) magnitude. Same algorithms as the double versions above
// (bilinear grid, finite-difference normal, 4-step ray iteration); only rounding differs.
struct LT { F3 oh, E, N; float inv_o, two_o; };
__device__ __forceinline__ float lt_alt(const Terrain& T, const float* M, float e, float n, float kdz) {
    int ng = (int)M[14];
    for (int gi = 0; gi < ng; gi++) {
        const float* g = M + 15 + 4 * gi;
        float step = g[0], half = g[1]; int N = (int)g[2]; int off = (int)g[3];
        float fe = (e + half) / step, fn = (n + half) / step;
        if (fe < 0.f || fn < 0.f) continue;
        int ie = (int)floorf(fe), in = (int)floorf(fn);
        if (ie + 1 >= N || in + 1 >= N) continue;
        float te = fe - ie, tn = fn - in;
        const float* a = T.altr + off; const unsigned char* gf = T.gr + off;
        float h00 = a[ie * N + in] + (gf[ie * N + in] ? 0.f : kdz), h10 = a[(ie + 1) * N + in] + (gf[(ie + 1) * N + in] ? 0.f : kdz),
              h01 = a[ie * N + in + 1] + (gf[ie * N + in + 1] ? 0.f : kdz), h11 = a[(ie + 1) * N + in + 1] + (gf[(ie + 1) * N + in + 1] ? 0.f : kdz);
        if (!(isfinite(h00) && isfinite(h10) && isfinite(h01) && isfinite(h11))) continue;
        return (h00 * (1.f - te) + h10 * te) * (1.f - tn) + (h01 * (1.f - te) + h11 * te) * tn;
    }
    return M[13];
}
// |o + d| - |o|
__device__ __forceinline__ float lt_rad(const LT& L, F3 d) {
    float u = dot(L.oh, d), w = dot(d, d);
    float num = 2.f * u + w * L.inv_o;
    return num / (sqrtf(1.f + num * L.inv_o) + 1.f);
}
__device__ __forceinline__ float lt_height(const Terrain& T, const float* M, const LT& L, F3 d, float kdz) {
    return lt_rad(L, d) - lt_alt(T, M, dot(d, L.E), dot(d, L.N), kdz);
}
__device__ F3 lt_normal(const Terrain& T, const float* M, const LT& L, F3 d, float kdz) {
    F3 r = normalized(L.oh + d * L.inv_o);
    float e = dot(d, L.E), n = dot(d, L.N);
    float step = 1.f; int ng = (int)M[14];
    for (int g = 0; g < ng; g++) { const float* gg = M + 15 + 4 * g; if (fabsf(e) < gg[1] - gg[0] && fabsf(n) < gg[1] - gg[0]) { step = gg[0]; break; } }
    F3 dE = L.E * step, dN = L.N * step;
    F3 p1 = d + dE, p2 = d - dE, p3 = d + dN, p4 = d - dN;
    float h1 = lt_alt(T, M, dot(p1, L.E), dot(p1, L.N), kdz), h2 = lt_alt(T, M, dot(p2, L.E), dot(p2, L.N), kdz);
    float h3 = lt_alt(T, M, dot(p3, L.E), dot(p3, L.N), kdz), h4 = lt_alt(T, M, dot(p4, L.E), dot(p4, L.N), kdz);
    float dhde = (h1 - h2) / (2.f * step), dhdn = (h3 - h4) / (2.f * step);
    return normalized(r - L.E * dhde - L.N * dhdn);
}
__device__ float lt_friction(const Terrain& T, const float* M, const LT& L, F3 d) {
    float e = dot(d, L.E), n = dot(d, L.N);
    int ng = (int)M[14];
    for (int g = 0; g < ng; g++) {
        const float* gg = M + 15 + 4 * g;
        float fe = rintf((e + gg[1]) / gg[0]), fn = rintf((n + gg[1]) / gg[0]); int Nn = (int)gg[2];
        if (fe >= 0.f && fn >= 0.f && (int)fe < Nn && (int)fn < Nn) return T.gr[(int)gg[3] + (int)fe * Nn + (int)fn] == 1 ? 0.9f : 0.6f;
    }
    return 0.9f;
}
// ray a - s u from local point da: the double ground_ray's iteration, s = k / (au + sqrt(au^2 - k))
// with k = |a|^2 - rg^2 = (|a| - rg)(|a| + rg)
__device__ bool lt_ray(const Terrain& T, const float* M, const LT& L, F3 da, F3 u, float len, float kdz, float* s_out, F3* d_out, F3* n_out) {
    float ra = lt_rad(L, da);                       // |a| - |o|
    float au = dot(normalized(L.oh + da * L.inv_o), u) * (L.two_o * 0.5f + ra);
    if (au <= 0.f) return false;                     // s = au - sqrt(disc) < 0
    float s = 0.f;
    for (int it = 0; it < 4; it++) {
        F3 d = da - u * s;
        float alt = lt_alt(T, M, dot(d, L.E), dot(d, L.N), kdz);
        float k = (ra - alt) * (L.two_o + ra + alt);  // (|a| - rg)(|a| + rg), alt relative to h0
        float disc = au * au - k;
        if (disc < 0.f) return false;
        s = k / (au + sqrtf(disc));
    }
    if (s < 0.f || s > len) return false;
    *s_out = s; *d_out = da - u * s; *n_out = lt_normal(T, M, L, *d_out, kdz);
    return true;
}

// ---- helpers ---------------------------------------------------------------------------------
__device__ __forceinline__ float roundhe(float x) { return rintf(x); }  // Mathf.Round: to even
__device__ __forceinline__ float lerpf(float a, float b, float t) { t = fminf(fmaxf(t, 0.f), 1.f); return a + (b - a) * t; }
__device__ __forceinline__ float invlerpf(float a, float b, float v) { return a == b ? 0.f : fminf(fmaxf((v - a) / (b - a), 0.f), 1.f); }
__device__ __forceinline__ float round2f(float x) { return (float)((double)rintf((float)((double)x * 100.0)) / 100.0); }
__device__ float boost_curve(float b, float p) {
    b = fminf(fmaxf(b, 0.f), 2.f); p = fminf(fmaxf(p, 0.01f), 1.f);
    float v = fminf(fmaxf(1.f / fabsf(1.f - 2.f / powf(b, p)), 0.01f), 1e7f);
    return roundhe(v * 10.f) / 10.f;
}
__device__ float mu_curve(float s) {
    if (s <= 0.01f) return 0.1f;
    if (s < 2.f) { float t = (s - 0.01f) / (2.f - 0.01f); float t2 = t * t, t3 = t2 * t; return 0.1f * (2.f * t3 - 3.f * t2 + 1.f) + 1.f * (-2.f * t3 + 3.f * t2); }
    return 1.f;
}
__device__ __forceinline__ F3 mat_vec(const float m[9], F3 v) {
    return f3(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z);
}
__device__ void inv3(const float m[9], float o[9]) {
    float a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
    float c0 = e * i - f * h, c1 = -(d * i - f * g), c2 = d * h - e * g;
    float r = 1.f / (a * c0 + b * c1 + c * c2);
    o[0] = c0 * r; o[1] = -(b * i - c * h) * r; o[2] = (b * f - c * e) * r;
    o[3] = c1 * r; o[4] = (a * i - c * g) * r; o[5] = -(a * f - c * d) * r;
    o[6] = c2 * r; o[7] = -(a * h - b * g) * r; o[8] = (a * e - b * d) * r;
}
__device__ __forceinline__ D3 rot_z(D3 p, double a) { double s = sin(a), c = cos(a); return d3(c * p.x - s * p.y, s * p.x + c * p.y, p.z); }

#define KPA_TO_ATM 0.009869232667160128
#define FIXED_DT 0.02
#define G0F 9.80665f

// ---- the tick --------------------------------------------------------------------------------
#define SV(f) state[(size_t)(f) * n + idx]
#define PF3(base, i) f3(Pf[(base) + 3 * (i)], Pf[(base) + 3 * (i) + 1], Pf[(base) + 3 * (i) + 2])
extern "C" __global__ void tick_kernel(const double* __restrict__ P, const float* __restrict__ Pf, const double* __restrict__ tmeta, const float* __restrict__ tmf,
                                       const float* __restrict__ talt, const float* __restrict__ taltr, const unsigned char* __restrict__ tgr, int tok,
                                       double* __restrict__ state, const float* __restrict__ act, int n, int substeps)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;
    Terrain T; T.meta = tmeta; T.alt = talt; T.altr = taltr; T.gr = tgr; T.ok = tok;

    const float throttle = fminf(fmaxf(act[4 * idx], 0.f), 1.f);
    const float pitch = fminf(fmaxf(act[4 * idx + 1], -1.f), 1.f);
    const float yaw = fminf(fmaxf(act[4 * idx + 2], -1.f), 1.f);
    const float roll = fminf(fmaxf(act[4 * idx + 3], -1.f), 1.f);
    const double dt = FIXED_DT;
    const float dtf = (float)FIXED_DT;
    const double om = P[P_OMEGA];
    const int np = (int)P[P_N_PARTS], nl = (int)P[P_N_LEGS], nh = (int)P[P_N_HULL], ne = (int)P[P_N_ENG];

    for (int sub = 0; sub < substeps; sub++) {
        if (SV(S_STATUS) == 3.0) break;  // crashed
        D3 X = d3(SV(S_X), SV(S_X + 1), SV(S_X + 2)), Vv = d3(SV(S_V), SV(S_V + 1), SV(S_V + 2));
        F3 Wv = f3((float)SV(S_W), (float)SV(S_W + 1), (float)SV(S_W + 2));
        Qd qd; qd.x = SV(S_Q); qd.y = SV(S_Q + 1); qd.z = SV(S_Q + 2); qd.w = SV(S_Q + 3);
        Qf qq = qf((float)qd.x, (float)qd.y, (float)qd.z, (float)qd.w);
        double res[4] = { SV(S_RES), SV(S_RES + 1), SV(S_RES + 2), SV(S_RES + 3) };
        // 1. mass props: linear in the resource amounts (P_MBASIS, see gpu.rs)
        float m = Pf[P_MBASIS], mc[3], il[9], own[9];
        for (int a = 0; a < 3; a++) mc[a] = Pf[P_MBASIS + 1 + a];
        for (int a = 0; a < 9; a++) { il[a] = Pf[P_MBASIS + 4 + a]; own[a] = Pf[P_MBASIS + 13 + a]; }
        float resf[4];
        for (int r = 0; r < 4; r++) {
            resf[r] = (float)res[r];
            if (P[P_RESCAPTOT + r] <= 0.0) continue;
            const float* B = Pf + P_MBASIS + (1 + r) * MB_STRIDE;
            float w = resf[r];
            m += w * B[0];
            for (int a = 0; a < 3; a++) mc[a] += w * B[1 + a];
            for (int a = 0; a < 9; a++) { il[a] += w * B[4 + a]; own[a] += w * B[13 + a]; }
        }
        F3 com_l = f3(mc[0], mc[1], mc[2]) / m;
        {
            float cv[3] = { com_l.x, com_l.y, com_l.z }; float c2 = dot(com_l, com_l);
            for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) il[3 * a + b] -= m * ((a == b ? c2 : 0.f) - cv[a] * cv[b]);
        }
        F3 r_com = qrot(qq, com_l);
        D3 x_com = X + dd(r_com);
        D3 v_com = Vv + dd(cross(Wv, r_com));
        F3 v_comf = ff(v_com);
        // 2. SmoothVelocity
        {
            float vf[3] = { v_comf.x, v_comf.y, v_comf.z };
            int si = (int)SV(S_SMOOTH_IDX);
            int pi = si == 0 ? 9 : si - 1;
            float pv[3] = { (float)SV(S_SMOOTH + 3 * pi), (float)SV(S_SMOOTH + 3 * pi + 1), (float)SV(S_SMOOTH + 3 * pi + 2) };
            float d2 = (vf[0] - pv[0]) * (vf[0] - pv[0]) + (vf[1] - pv[1]) * (vf[1] - pv[1]) + (vf[2] - pv[2]) * (vf[2] - pv[2]);
            float n2v = vf[0] * vf[0] + vf[1] * vf[1] + vf[2] * vf[2], n2p = pv[0] * pv[0] + pv[1] * pv[1] + pv[2] * pv[2];
            if ((double)d2 > 1e6 * dt * dt && n2v > n2p) {
                if (SV(S_SPIKING) != 0.0) { for (int c = 0; c < 3; c++) SV(S_SMOOTH + 3 * si + c) = vf[c]; }
                else if (SV(S_PROVISIONAL) != 0.0) {
                    for (int c = 0; c < 3; c++) SV(S_SMOOTH + 3 * si + c) = vf[c];
                    int li = (int)SV(S_LAST_VEL_IDX);
                    for (int c = 0; c < 3; c++) SV(S_SMOOTH + 3 * li + c) = SV(S_LAST_VEL + c);
                    SV(S_SPIKING) = 1.0;
                } else {
                    SV(S_PROVISIONAL) = 1.0;
                    for (int c = 0; c < 3; c++) SV(S_LAST_VEL + c) = vf[c];
                    SV(S_LAST_VEL_IDX) = si;
                    for (int c = 0; c < 3; c++) SV(S_SMOOTH + 3 * si + c) = pv[c];
                }
            } else {
                for (int c = 0; c < 3; c++) SV(S_SMOOTH + 3 * si + c) = vf[c];
                SV(S_SPIKING) = 0.0; SV(S_PROVISIONAL) = 0.0;
            }
            SV(S_SMOOTH_IDX) = (si + 1) % 10;
        }
        float sm[3] = { 0.f, 0.f, 0.f };
        for (int j = 0; j < 10; j++) for (int c = 0; c < 3; c++) sm[c] += (float)SV(S_SMOOTH + 3 * j + c);
        float spd = sqrtf(sm[0] * sm[0] + sm[1] * sm[1] + sm[2] * sm[2]) * 0.1f;
        double alt = normd(x_com) - P[P_RADIUS];
        // sun / latitude temperature offset (every 25 ticks), GetAtmoThermalStats: f64 as the CPU
        if (P[P_SUN_TEMP] != 0.0 && alt < P[P_ATM_DEPTH]) {
            if (((unsigned int)SV(S_TICKS)) % 25u == 0u || !isfinite(SV(S_TOFF))) {
                double ut = SV(S_UT);
                double ang = P[P_ORB_MA0] + P[P_ORB_N] * (ut - P[P_ORB_EPOCH]);  // circular, equatorial
                D3 rk = d3(cos(ang), sin(ang), 0.0);
                double rotang = fmod(P[P_INIT_ROT_DEG] + 360.0 * (1.0 / P[P_SIDEREAL]) * ut, 360.0) * (M_PI / 180.0);
                D3 sun = rot_z(-rk, -rotang);
                double ta = atan2(rk.y, rk.x) - P[P_ORB_PE_LON_DEG] * (M_PI / 180.0);
                D3 up = normalizedd(x_com);
                double d1 = sun.z, d2 = up.z;  // spin axis (0, 0, 1)
                double colat0 = acos(d2); if (isnan(colat0)) colat0 = d2 > 0 ? 0.0 : M_PI;
                double sunp = acos(d1); if (isnan(sunp)) sunp = d1 > 0 ? 0.0 : M_PI;
                double maxf = (1.0 + cos(sunp - colat0)) * 0.5, minf = (1.0 + cos(sunp + colat0)) * 0.5;
                double colat = colat0 > M_PI / 2 ? M_PI - colat0 : colat0;
                double latd = (M_PI / 2 - colat) * 57.295780181884766;
                double s45 = sin(45.0 * (M_PI / 180.0) * 0.5), c45 = cos(45.0 * (M_PI / 180.0) * 0.5);
                // Quaternion.AngleAxis(45, spin) * up: rotation about z
                double ca = c45 * c45 - s45 * s45, sa = 2.0 * s45 * c45;
                D3 up45 = d3(ca * up.x - sa * up.y, sa * up.x + ca * up.y, up.z);
                double todf = (1.0 + dotd(sun, up45)) * 0.5;
                double diff = maxf - minf;
                double sunf;
                if (diff > 0.001) { sunf = (todf - minf) / diff; if (isnan(sunf)) sunf = todf > 0.5 ? 1.0 : 0.0; } else sunf = minf + diff * 0.5;
                double tad = fmod(ta * (180.0 / M_PI) + 360.0, 360.0);
                double off = CURVED(LATBIAS, latd) + CURVED(LATSUN, latd) * sunf + CURVED(AXBIAS, tad) * CURVED(AXMULT, latd);
                SV(S_TOFF) = (float)off;
            }
        } else SV(S_TOFF) = 0.0;
        float toff = P[P_SUN_TEMP] != 0.0 ? (float)SV(S_TOFF) : 0.f;
        // atmosphere (CelestialBody.GetPressure / GetFullTemperature / GetDensity / GetSpeedOfSound)
        float altf = (float)alt;
        float p_kpa = (alt < P[P_ATM_DEPTH]) ? CURVE(PRESSURE, altf) : 0.f;
        float rho = 0.f, mach = 0.f, prem = 1.f;
        if (p_kpa > 0.f) {
            float tk = CURVE(TEMPERATURE, altf) + CURVE(SUNMULT, altf) * toff;
            rho = (tk > 0.f) ? p_kpa * 1000.f * Pf[P_MOLAR] / (Pf[P_RGAS] * tk) : 0.f;
            float csnd = rho > 0.f ? sqrtf(Pf[P_GAMMA] * p_kpa * 1000.f / rho) : 0.f;
            mach = csnd > 0.f ? spd / csnd : 0.f;
            prem = CURVE(PRE, rho * spd);
        }
        F3 force = f3(0, 0, 0), torque = f3(0, 0, 0);
        // 3a. engines + gimbal
        for (int k = 0; k < ne; k++) {
            const float* E = Pf + P_ENG0 + k * ENG_STRIDE;
            float cur = (float)SV(S_THR + k);
            float tmin = E[0] > 0.f ? E[1] / E[0] : 0.f;
            float req = tmin != 0.f ? tmin + (1.f - tmin) * throttle : throttle;
            if (E[6] == 0.f) cur = req;
            else if (fabsf(cur - req) <= 1e-4f) cur = req;
            else if (cur < req) cur = cur + (req - cur) * fminf(fmaxf(E[4] * dtf, 0.f), 1.f);
            else cur = cur + (req - cur) * fminf(fmaxf(E[5] * dtf, 0.f), 1.f);
            F3 epos = f3(E[9], E[10], E[11]), edir = f3(E[12], E[13], E[14]), epiv = f3(E[15], E[16], E[17]);
            F3 pos_w = qrot(qq, epos);
            double alt_e = normd(X + dd(pos_w)) - P[P_RADIUS];
            float p_atm = ((alt_e < P[P_ATM_DEPTH]) ? CURVE(PRESSURE, alt_e) : 0.f) * (float)KPA_TO_ATM;
            float thrust = 0.f; bool out = false;
            if (cur > 0.f) {
                float isp = curve32(E + 22, (int)E[2], p_atm);
                float mflow = E[0] * cur;
                float rthr = mflow * isp * G0F;
                double mass = (double)mflow * dt;
                float nprop = 0.f;
                const double* Ed = P + P_ENG0 + k * ENG_STRIDE;
                for (int r = 0; r < 4; r++) { double u = Ed[18 + r]; if (u > 0.0) { if (res[r] < mass * u * Ed[3]) out = true; nprop += 1.f; } }
                if (out) { cur = 0.f; }
                else {
                    float fs = 0.f;
                    for (int r = 0; r < 4; r++) { double u = Ed[18 + r]; if (u > 0.0) { double rq = mass * u; double got = fmax(fmin(rq, res[r]), 0.0); res[r] -= got; fs += (float)got / (float)rq; } }
                    thrust = rthr * fminf(fmaxf(fs / nprop, 0.f), 1.f);
                }
            }
            SV(S_THR + k) = cur;
            float sgn = com_l.y > epos.y ? 1.f : -1.f;
            float tg[2] = { sgn * pitch * E[7], sgn * yaw * E[7] };
            float gv[2];
            for (int a = 0; a < 2; a++) {
                float g0 = (float)SV(S_GIMBAL + 2 * k + a);
                gv[a] = E[8] > 0.f ? g0 + (tg[a] - g0) * fminf(fmaxf(E[8] * dtf, 0.f), 1.f) : tg[a];
                SV(S_GIMBAL + 2 * k + a) = gv[a];
            }
            // Quaternion.AngleAxis(g0, right) * Quaternion.AngleAxis(-g1, forward)
            float s0, c0, s1, c1;
            sincosf(gv[0] * (float)(M_PI / 180.0) * 0.5f, &s0, &c0);
            sincosf(-gv[1] * (float)(M_PI / 180.0) * 0.5f, &s1, &c1);
            Qf gq = qmul(qf(s0, 0.f, 0.f, c0), qf(0.f, 0.f, s1, c1));
            F3 dir_w = qrot(qq, qrot(gq, edir));
            F3 tp_w = qrot(qq, epiv + qrot(gq, epos - epiv));
            F3 f = dir_w * thrust;
            force = force + f; torque = torque + cross(tp_w - r_com, f);
        }
        // 3b. reaction wheel
        {
            F3 tq = f3(pitch * Pf[P_RWTORQUE], roll * Pf[P_RWTORQUE + 1], yaw * Pf[P_RWTORQUE + 2]);
            float sum = fabsf(pitch) + fabsf(roll) + fabsf(yaw);
            float t = fminf(fmaxf(Pf[P_RWRESP] * dtf, 0.f), 1.f);
            F3 rw = f3((float)SV(S_RW), (float)SV(S_RW + 1), (float)SV(S_RW + 2));
            rw = rw + (tq - rw) * t;
            SV(S_RW) = rw.x; SV(S_RW + 1) = rw.y; SV(S_RW + 2) = rw.z;
            double ecr = P[P_RWEC] * (double)sum * dt;
            double got = fmin(ecr, res[3]);
            res[3] -= got;
            if (sum > 0.f && got >= 0.9 * ecr) torque = torque + qrot(qq, f3(-rw.x, rw.y, rw.z));
        }
        // 4. gravity + frame accelerations (double: they act on the double CoM state)
        double r2 = dotd(x_com, x_com);
        D3 g = x_com * (-P[P_GM] / (r2 * sqrt(r2)));
        // omega = (0, 0, om): -2 omega x v - omega x (omega x x)
        D3 a_frame = d3(2.0 * om * v_com.y + om * om * x_com.x, -2.0 * om * v_com.x + om * om * x_com.y, 0.0);
        // 5. aero per part (root frame) + angular damping
        float damp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        if (P[P_AERO] != 0.0 && rho > 0.f) {
            float tip = CURVE(TIP, mach), surf = CURVE(SURF, mach), tail = CURVE(TAIL, mach), mult = CURVE(MULT, mach), cdp = CURVE(CDPOW, mach);
            float blm = CURVE(BLMACH, mach);
            F3 vl = qrot(qconj(qq), v_comf), wlc = qrot(qconj(qq), Wv);
            F3 fl = f3(0, 0, 0), tl = f3(0, 0, 0);
            for (int i = 0; i < np; i++) {
                float mpi = Pf[P_DRY + i];
                for (int r = 0; r < 4; r++) if (Pf[P_RESCAPTOT + r] > 0.f) mpi += Pf[P_RESCAP + 4 * i + r] * (resf[r] / Pf[P_RESCAPTOT + r]) * Pf[P_RESDENS + r];
                F3 pr = PF3(P_PCOM, i) - com_l;
                F3 vp = vl + cross(wlc, pr);
                float s2 = dot(vp, vp);
                if (s2 <= 0.f) continue;
                float qp = rho * 0.0005f * s2;
                float ci = fmaxf(Pf[P_ANGDRAG + i] * (qp * (float)KPA_TO_ATM) * 2.f, 0.f);
                for (int a = 0; a < 9; a++) damp[a] += ci * mpi * Pf[P_PINERT + 9 * i + a];
                F3 vh = vp * rsqrtf(s2);
                Qf prot = qf(Pf[P_PROT + 4 * i], Pf[P_PROT + 4 * i + 1], Pf[P_PROT + 4 * i + 2], Pf[P_PROT + 4 * i + 3]);
                F3 d = qrot(qconj(prot), vh);
                float comp[3] = { d.x, d.y, d.z };
                bool bl = Pf[P_BODYLIFT + i] != 0.f;
                float ad = 0.f, ar = 0.f; float ll[3] = { 0.f, 0.f, 0.f };
                for (int j = 0; j < 6; j++) {
                    float dt_ = (j & 1) == 0 ? comp[j >> 1] : -comp[j >> 1];
                    float d01 = (dt_ + 1.f) * 0.5f;
                    float sc = d01 <= 0.5f ? (tail + (surf - tail) * (2.f * d01)) * mult : (surf + (tip - surf) * (2.f * d01 - 1.f)) * mult;
                    float fa = Pf[P_FAREA + 6 * i + j];
                    float a = fa * sc;
                    ar += a;
                    float l = Pf[P_FLNCD + 6 * i + j];
                    float cd = isnan(l) ? Pf[P_FDRAG + 6 * i + j] : expf(cdp * l);
                    ad += a * cd;
                    if (dt_ > 0.f && bl) {
                        float lv = dt_ * fa * Pf[P_FDRAG + 6 * i + j] * body_lift_curve32(Pf, dt_);
                        ll[j >> 1] += ((j & 1) == 0 ? -1.f : 1.f) * lv;
                    }
                }
                if (ar <= 0.f) ad = 0.f;
                float drag = qp * ad * 0.1f * prem * 8.f;
                F3 fd = vh * (-drag);
                fl = fl + fd;
                tl = tl + cross(PF3(P_PCOP, i) - com_l, fd);
                if (bl) {
                    float blv = qp * Pf[P_BODYLIFT + i] * 18.f * blm;
                    F3 l = qrot(prot, f3(ll[0], ll[1], ll[2]) * blv);
                    F3 lift = l - vh * dot(l, vh);
                    fl = fl + lift;
                    tl = tl + cross(PF3(P_PCOL, i) - com_l, lift);
                }
            }
            force = force + qrot(qq, fl); torque = torque + qrot(qq, tl);
        }
        float ilinv[9]; inv3(il, ilinv);
        // 6. ground contact (scenery translated by the per-env ground shift: query at p - GS)
        const D3 GS = d3(SV(S_GSHIFT), SV(S_GSHIFT + 1), SV(S_GSHIFT + 2));
        const double KDZ = SV(S_KSCDZ);
        const float kdzf = (float)KDZ;
        // local terrain frame: dX = X - GS - o (double subtraction, then float)
        LT Lt; F3 dX = f3(0, 0, 0);
        if (T.ok) {
            D3 o = d3(tmeta[0], tmeta[1], tmeta[2]);
            double on = normd(o);
            Lt.oh = ff(o * (1.0 / on)); Lt.E = f3(tmf[3], tmf[4], tmf[5]); Lt.N = f3(tmf[6], tmf[7], tmf[8]);
            Lt.inv_o = (float)(1.0 / on); Lt.two_o = (float)(2.0 * on);
            dX = ff(X - GS - o);
        }
        int legs = 0; bool contact_any = false;
        F3 cr[MAX_HULL], cn[MAX_HULL]; float cgap[MAX_HULL], ctol[MAX_HULL], cmu[MAX_HULL]; int chull[MAX_HULL]; int nc = 0;
        const double margin = P[P_BOUND] + 10.0 + normd(v_com) * dt;
        // exact early-out above the highest ground point (sim::ground_ceiling)
        const bool above_all = normd(X) - normd(GS) - fmax(KDZ, 0.0) - P[P_GROUND_CEIL] >= margin;
        bool near_ground = P[P_GROUND] != 0.0 && !above_all && (T.ok ? (double)lt_height(T, tmf, Lt, dX, kdzf) : ground_height(P, T, X - GS, KDZ)) < margin;
        if (!near_ground) {
            for (int k = 0; k < nl; k++) { int lb = S_LEGS + k * LEG_FIELDS; SV(lb) = 0; SV(lb + 1) = 0; SV(lb + 2) = 0; SV(lb + 16) = 0; SV(lb + 17) = 0; }
        } else {
            float gm = (float)normd(g);
            int ng = 0; for (int k = 0; k < nl; k++) if (SV(S_LEGS + k * LEG_FIELDS) != 0.0) ng++;
            for (int k = 0; k < nl; k++) {
                const double* Ls = P + P_LEG0 + k * LEG_STRIDE;
                const float* Lf = Pf + P_LEG0 + k * LEG_STRIDE;
                int lb = S_LEGS + k * LEG_FIELDS;
#define L(j) SV(lb + (j))
                // wheel::update_spring (float, as the game)
                bool grounded = L(0) != 0.0;
                float lx = (float)L(1), lhf = (float)L(2), lk = (float)L(3), lc = (float)L(4), ltg = (float)L(5);
                float msp = (float)L(6), mdm = (float)L(7), stw = (float)L(8), dtw = (float)L(9), ab = (float)L(10);
                bool aset = L(11) != 0.0;
                float rate = (float)L(12), drate = (float)L(13), rt = (float)L(14), rtd = (float)L(15);
                float tire0 = (float)L(16), tire1 = (float)L(17);
                if (grounded) {
                    double tt = (double)m;
                    if (ng > 0) tt = round(tt * P[P_GEE_ASL] * 10.0) / 10.0;
                    float share = roundhe(100.f / (float)(ng > 0 ? ng : 1)) / 100.f;
                    float smg = fmaxf((float)(tt * (double)share), 0.5f);
                    float mm = m;
                    float sc = (float)((double)lx / Ls[10]);
                    float num = mm * sc;
                    float cmax = lerpf(5000.f, 10000.f, invlerpf(50.f, 200.f, mm));
                    if (sc > 0.8f) ab = fminf(fmaxf(ab + 0.2f * dtf, 0.85f), 2.f);
                    else if (sc < 0.4f) ab = fmaxf(ab - 0.2f * dtf, 0.f);
                    float boost = 1.f + fminf(fmaxf(sc * boost_curve(Pf[P_GEE_ASL], 1.f), -1.f), 0.5f);
                    bool rearm = !aset || (Lf[15] > 0.f && num > Lf[15]);
                    if (rearm) {
                        dtw = P[P_GEE_ASL] < 1.0 ? 1.1f : round2f(lerpf(1.f, 2.f, invlerpf(1.f, Pf[P_MAX_SURF_GEE], Pf[P_GEE_ASL])));
                        if (msp > 0.f) { float v = Lf[11] * msp / 9.80665f; stw = round2f(lerpf(1.f, 3.f, invlerpf(0.f, Lf[15] * 2.f, v))); aset = true; }
                    }
                    float fudge = lerpf(1.f, 15.f, invlerpf(10.f, 100.f, num));
                    float spring = fminf(fmaxf(stw * Lf[12] * smg * boost_curve(boost, Lf[14] + ab), 0.01f), cmax);
                    float damper = sqrtf(fminf(fmaxf(spring * Lf[13] * fudge, 0.01f), cmax)) * dtw;
                    if (P[P_GEE_ASL] > 1.5) { spring *= 3.f; damper *= 1.2f; }
                    mdm = fmaxf(roundhe(damper), 0.1f); msp = fmaxf(roundhe(spring), 0.1f);
                    if (fabsf(msp - rate) > 3.f) rate = msp;
                    if (fabsf(mdm - drate) > 1.05f) drate = mdm;
                }
                float wgt = (float)(Ls[16] * (double)gm); float d = Lf[10];
                lk = fmaxf(rt, fmaxf(wgt / d, 0.1f)); lc = fmaxf(rtd, 0.1f);
                float tgt = fminf(fmaxf(wgt / lk / d, 0.1f), 1.f);
                ltg = (isfinite(ltg) && fabsf(tgt - ltg) > 0.02f) ? (ltg > tgt ? ltg - 0.02f : ltg + 0.02f) : tgt;
                rt = rate; rtd = drate;
                // wheel collider ray (double world positions)
                F3 ra = qrot(qq, PF3(P_LEG0 + k * LEG_STRIDE, 0));
                F3 uf = qrot(qq, PF3(P_LEG0 + k * LEG_STRIDE, 1));
                double len = Ls[10] + Ls[9];
                double s_hit; F3 nn; bool hit;
                if (T.ok) {
                    float sf; F3 dl;
                    hit = lt_ray(T, tmf, Lt, dX + ra, uf, (float)len, kdzf, &sf, &dl, &nn);
                    s_hit = sf;
                } else {
                    D3 a = X + dd(ra), u = dd(uf), p, nnd;
                    hit = ground_ray(P, T, a - GS, u, len, KDZ, &s_hit, &p, &nnd);
                    nn = ff(nnd);
                }
                if (hit) {
                    F3 rel = ra - r_com - uf * (float)s_hit;  // p - x_com in float
                    float ndu = dot(nn, uf);
                    if (grounded) {
                        F3 vf = v_comf + cross(Wv, rel);
                        F3 vt = vf - nn * dot(vf, nn);
                        F3 fw = qrot(qq, PF3(P_LEG0 + k * LEG_STRIDE, 2));
                        F3 fwd = normalized(fw - nn * dot(fw, nn));
                        F3 side = cross(nn, fwd);
                        float vl0 = dot(vt, side), vl1 = dot(vt, fwd);
                        float ca = atan2f(dot(uf, side), ndu);
                        float load = fmaxf(lhf * cosf(ca), Lf[18]);
                        float mf = fmaxf(lhf, 0.f) / gm;
                        float fe0 = -0.5f * mf * vl0 / dtf, fe1 = -0.5f * mf * vl1 / dtf;
                        float smag = sqrtf(vl0 * vl0 + vl1 * vl1);
                        float cap0, cap1;
                        if (smag < 1e-6f) { cap0 = 0.f; cap1 = 0.f; }
                        else if (smag < 0.01f) { float kk = mu_curve(0.01f) * load / 0.01f; cap0 = kk * fabsf(vl0); cap1 = kk * sqrtf(fmaxf(0.01f * 0.01f - vl0 * vl0, 0.f)); }
                        else { float fq = mu_curve(smag) * load; cap0 = fq * fabsf(vl0) / smag; cap1 = fq * fabsf(vl1) / smag; }
                        float f0 = fminf(fmaxf(fe0, -cap0), cap0), f1 = fminf(fmaxf(fe1, -cap1), cap1);
                        float smo = fminf(fmaxf(10.f * dtf, 0.f), 1.f);
                        tire0 = tire0 + (f0 - tire0) * smo; tire1 = tire1 + (f1 - tire1) * smo;
                        F3 ft = side * tire0 + fwd * tire1;
                        force = force + ft; torque = torque + cross(rel, ft);
                    } else { tire0 = 0.f; tire1 = 0.f; }
                    float x = (float)(len - s_hit);
                    float xp = grounded ? lx : 0.f;
                    float fs = lk * (x - ltg * d) + lc * (x - xp) / dtf;
                    float rep = fs * ndu;
                    F3 fc = nn * rep;
                    force = force + fc; torque = torque + cross(rel, fc);
                    float vn = dot(v_comf + cross(Wv, rel), nn);
                    SV(S_IMPACT) = fmax(SV(S_IMPACT), (double)(-vn));
                    grounded = true; lx = x; lhf = rep;
                    legs++; contact_any = true;
                } else { grounded = false; lx = 0.f; lhf = 0.f; tire0 = 0.f; tire1 = 0.f; }
                L(0) = grounded ? 1.0 : 0.0; L(1) = lx; L(2) = lhf; L(3) = lk; L(4) = lc; L(5) = ltg; L(6) = msp; L(7) = mdm; L(8) = stw; L(9) = dtw;
                L(10) = ab; L(11) = aset ? 1.0 : 0.0; L(12) = rate; L(13) = drate; L(14) = rt; L(15) = rtd; L(16) = tire0; L(17) = tire1;
#undef L
            }
            for (int k = 0; k < nh; k++) {
                F3 rh = qrot(qq, PF3(P_HULL, k));
                F3 dh = dX + rh;
                D3 p = X + dd(rh);
                double gap = T.ok ? (double)lt_height(T, tmf, Lt, dh, kdzf) : ground_height(P, T, p - GS, KDZ);
                if (gap < 0.01 && nc < MAX_HULL) {
                    cr[nc] = rh - r_com; cgap[nc] = (float)gap; ctol[nc] = Pf[P_HULLTOL + k]; chull[nc] = k;
                    if (T.ok) { cn[nc] = lt_normal(T, tmf, Lt, dh, kdzf); cmu[nc] = lt_friction(T, tmf, Lt, dh); }
                    else { cn[nc] = ff(ground_normal(P, T, p - GS, KDZ)); cmu[nc] = ground_friction(T, p - GS); }
                    nc++;
                }
            }
        }
        // 7. integrate (Euler; the CPU's vacuum drift-compensation branch is not ported)
        F3 a_ng = force / m;
        D3 v_com_n = v_com + (dd(a_ng) + g + a_frame) * dt;
        D3 x_com_n = x_com + v_com_n * dt;
        float iorb[9];
        for (int a = 0; a < 9; a++) iorb[a] = il[a] - own[a];
        F3 wl = qrot(qconj(qq), Wv);
        F3 gyro = cross(wl, mat_vec(iorb, wl));
        F3 al = mat_vec(ilinv, qrot(qconj(qq), torque) - gyro);
        F3 wl2 = wl + al * dtf;
        F3 dw = mat_vec(ilinv, mat_vec(damp, wl2));
        wl2 = wl2 - dw * dtf;
        F3 w = qrot(qq, wl2);
        if (nc == 0) SV(S_HTOUCH) = 0.0;
        if (nc > 0) {
            F3 vb = ff(v_com_n), dV = f3(0, 0, 0);
            // Part.OnCollisionEnter -> HandleCollision (see cuda_tick.cu / sim.rs)
            unsigned int touch = 0u, prev = (unsigned int)SV(S_HTOUCH);
            for (int c = 0; c < nc; c++) if (cgap[c] <= 0.01f) {
                contact_any = true; float vn = dot(vb + cross(w, cr[c]), cn[c]); SV(S_IMPACT) = fmax(SV(S_IMPACT), (double)(-vn));
                int k = chull[c]; touch |= 1u << k;
                if (P[P_CRASH] != 0.0 && Pf[P_HULLCRASH + k] != 0.f && !(prev & (1u << k))) {
                    int pi = (int)Pf[P_HULLPART + k];
                    F3 rp = qrot(qq, PF3(P_PCOM, pi) - com_l);
                    if (norm(vb + cross(w, rp)) > ctol[c]) SV(S_STATUS) = 3.0;
                }
            }
            SV(S_HTOUCH) = (double)touch;
            float ln[MAX_HULL]; F3 lt[MAX_HULL];
            for (int c = 0; c < nc; c++) { ln[c] = 0.f; lt[c] = f3(0, 0, 0); }
            const float minv = 1.f / m;
            for (int it = 0; it < 8; it++) {
                for (int c = 0; c < nc; c++) {
                    float target = cgap[c] >= 0.f ? -cgap[c] / dtf : -cgap[c] / dtf * 0.55f;
                    F3 vp = vb + dV + cross(w, cr[c]);
                    float vn = dot(vp, cn[c]);
                    F3 rn = cross(cr[c], cn[c]);
                    F3 irn = qrot(qq, mat_vec(ilinv, qrot(qconj(qq), rn)));
                    float kn = minv + dot(rn, irn);
                    float nw = fmaxf(ln[c] + (target - vn) / kn, 0.f);
                    float dj = nw - ln[c]; ln[c] = nw;
                    dV = dV + cn[c] * (dj * minv); w = w + irn * dj;
                    vp = vb + dV + cross(w, cr[c]);
                    F3 vt = vp - cn[c] * dot(vp, cn[c]);
                    float vtn = norm(vt);
                    if (vtn > 1e-12f) {
                        F3 td = vt / vtn; F3 rtt = cross(cr[c], td);
                        F3 irt = qrot(qq, mat_vec(ilinv, qrot(qconj(qq), rtt)));
                        float kt = minv + dot(rtt, irt);
                        F3 want = lt[c] - td * (vtn / kt);
                        float cap = cmu[c] * ln[c];
                        F3 ltn = norm(want) > cap ? normalized(want) * cap : want;
                        F3 djv = ltn - lt[c]; lt[c] = ltn;
                        dV = dV + djv * minv;
                        w = w + qrot(qq, mat_vec(ilinv, qrot(qconj(qq), cross(cr[c], djv))));
                    }
                }
            }
            x_com_n = x_com_n + dd(dV) * dt; v_com_n = v_com_n + dd(dV);
        }
        float wn = norm(w);
        if (wn > 50.f) w = w * (50.f / wn);
        // attitude: float rotation increment (as PhysX), accumulated into the double quaternion
        Qd qn = qd;
        float ang = norm(w) * dtf;
        if (ang > 0.f) {
            float s, c; sincosf(ang * 0.5f, &s, &c);
            F3 ax = w / norm(w);
            double dx = ax.x * s, dy = ax.y * s, dz = ax.z * s, dw_ = c;
            Qd r;
            r.x = dw_ * qd.x + dx * qd.w + dy * qd.z - dz * qd.y;
            r.y = dw_ * qd.y - dx * qd.z + dy * qd.w + dz * qd.x;
            r.z = dw_ * qd.z + dx * qd.y - dy * qd.x + dz * qd.w;
            r.w = dw_ * qd.w - dx * qd.x - dy * qd.y - dz * qd.z;
            double inv = rsqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
            qn.x = r.x * inv; qn.y = r.y * inv; qn.z = r.z * inv; qn.w = r.w * inv;
        }
        F3 rcn = qrot(qf((float)qn.x, (float)qn.y, (float)qn.z, (float)qn.w), com_l);
        X = x_com_n - dd(rcn); Vv = v_com_n - dd(cross(w, rcn));
        SV(S_X) = X.x; SV(S_X + 1) = X.y; SV(S_X + 2) = X.z;
        SV(S_V) = Vv.x; SV(S_V + 1) = Vv.y; SV(S_V + 2) = Vv.z;
        SV(S_Q) = qn.x; SV(S_Q + 1) = qn.y; SV(S_Q + 2) = qn.z; SV(S_Q + 3) = qn.w;
        SV(S_W) = w.x; SV(S_W + 1) = w.y; SV(S_W + 2) = w.z;
        for (int r = 0; r < 4; r++) SV(S_RES + r) = res[r];
        SV(S_UT) = SV(S_UT) + dt;
        SV(S_TICKS) = SV(S_TICKS) + 1.0;
        if (SV(S_STATUS) != 3.0) {
            if (legs == nl && legs > 0) {
                bool still = norm(v_comf) < 0.3f && norm(w) < 0.1f;
                SV(S_SETTLED) = still ? fmin(SV(S_SETTLED) + 1.0, 65535.0) : 0.0;
                SV(S_STATUS) = SV(S_SETTLED) >= 25.0 ? 2.0 : 1.0;
            } else { SV(S_SETTLED) = 0.0; SV(S_STATUS) = contact_any ? 1.0 : 0.0; }
        }
    }
}
