// CUDA port of ksp_core::sim::tick (crates/ksp-core/src/sim.rs): one thread per env, the env's
// state gathered from the field-major SoA buffer, `substeps` physics ticks, scattered back.
// Mirrors the CPU code operation for operation (f64 where the CPU uses f64, f32 where the CPU -
// and the game - uses float). Index macros P_* / S_* come from ksp_core::gpu::pack (prepended).
// Scope: rotating (Kerbin-fixed) frame below the 100 km switch; the vacuum orbit-drift
// compensation branch of the CPU tick is not ported (plain Euler is used there).

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
struct V3 { double x, y, z; };
__device__ __forceinline__ V3 v3(double x, double y, double z) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
__device__ __forceinline__ V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
__device__ __forceinline__ V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
__device__ __forceinline__ V3 operator-(V3 a) { return v3(-a.x, -a.y, -a.z); }
__device__ __forceinline__ V3 operator*(V3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
__device__ __forceinline__ V3 operator/(V3 a, double s) { return v3(a.x / s, a.y / s, a.z / s); }
__device__ __forceinline__ double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__device__ __forceinline__ V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
__device__ __forceinline__ double norm2(V3 a) { return dot(a, a); }
__device__ __forceinline__ double norm(V3 a) { return sqrt(dot(a, a)); }
__device__ __forceinline__ V3 normalized(V3 a) { double n = norm(a); return n > 0.0 ? a / n : v3(0, 0, 0); }

struct Q { double x, y, z, w; };
__device__ __forceinline__ Q qmk(double x, double y, double z, double w) { Q q; q.x = x; q.y = y; q.z = z; q.w = w; return q; }
__device__ __forceinline__ Q qmul(Q a, Q b) {
    return qmk(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
               a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}
__device__ __forceinline__ Q qconj(Q q) { return qmk(-q.x, -q.y, -q.z, q.w); }
__device__ __forceinline__ V3 qrot(Q q, V3 v) {
    V3 u = v3(q.x, q.y, q.z);
    V3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}
__device__ __forceinline__ Q qnorm(Q q) { double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w); return qmk(q.x / n, q.y / n, q.z / n, q.w / n); }
__device__ __forceinline__ Q angle_axis(double deg, V3 axis) {
    V3 a = normalized(axis);
    double h = deg * (M_PI / 180.0) * 0.5;
    double s = sin(h), c = cos(h);
    return qmk(a.x * s, a.y * s, a.z * s, c);
}
__device__ __forceinline__ Q integrate_world(Q q, V3 w, double dt) {
    double ang = norm(w) * dt;
    if (ang < 1e-300) return q;
    Q dq = angle_axis(ang * (180.0 / M_PI), w);
    return qnorm(qmul(dq, q));
}

// ---- curves: Unity AnimationCurve (Hermite, clamped); keys (t, v, in, out) -----------------
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
// FloatCurve.Evaluate(float): f32 argument and result
__device__ __forceinline__ double feval(const double* k, int n, double x) { return (double)(float)eval_curve(k, n, (double)(float)x); }
#define CURVE(P, name, x) feval(P + P_##name, P_##name##_N, x)

__device__ float body_lift_curve32(const double* P, float x) {
    const double* k = P + P_BLIFT; int n = P_BLIFT_N;
    if (x <= (float)k[0]) return (float)k[1];
    if (x >= (float)k[4 * (n - 1)]) return (float)k[4 * (n - 1) + 1];
    int i = 0;
    while (i < n - 2 && x >= (float)k[4 * (i + 1)]) i++;
    const double* a = k + 4 * i; const double* b = k + 4 * (i + 1);
    float dx = (float)(b[0] - a[0]);
    float t = (x - (float)a[0]) / dx;
    float t2 = t * t, t3 = t2 * t;
    return (2.f * t3 - 3.f * t2 + 1.f) * (float)a[1] + (t3 - 2.f * t2 + t) * (float)a[3] * dx + (-2.f * t3 + 3.f * t2) * (float)b[1] + (t3 - t2) * (float)b[2] * dx;
}

// ---- terrain heightfield ---------------------------------------------------------------------
struct Terrain { const double* meta; const float* alt; const unsigned char* gr; int ok; };
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
__device__ double terrain_alt(const Terrain& T, V3 p, double kdz) {
    V3 o = v3(T.meta[0], T.meta[1], T.meta[2]), E = v3(T.meta[3], T.meta[4], T.meta[5]), N = v3(T.meta[6], T.meta[7], T.meta[8]);
    V3 d = p - o; double e = dot(d, E), n = dot(d, N);
    int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) { double h; if (grid_sample(T, g, e, n, kdz, &h)) return h; }
    return T.meta[13];
}
__device__ double ground_height(const double* P, const Terrain& T, V3 p, double kdz) {
    if (T.ok) return norm(p) - T.meta[12] - terrain_alt(T, p, kdz);
    return norm(p) - P[P_RADIUS] - P[P_GROUND_ALT] - kdz;
}
__device__ V3 ground_normal(const double* P, const Terrain& T, V3 p, double kdz) {
    if (!T.ok) return normalized(p);
    V3 r = normalized(p);
    V3 o = v3(T.meta[0], T.meta[1], T.meta[2]), E = v3(T.meta[3], T.meta[4], T.meta[5]), N = v3(T.meta[6], T.meta[7], T.meta[8]);
    V3 d = p - o; double e = dot(d, E), n = dot(d, N);
    double step = 1.0; int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) { const double* gg = T.meta + 15 + 4 * g; if (fabs(e) < gg[1] - gg[0] && fabs(n) < gg[1] - gg[0]) { step = gg[0]; break; } }
    double h1 = terrain_alt(T, p + E * step, kdz), h2 = terrain_alt(T, p - E * step, kdz), h3 = terrain_alt(T, p + N * step, kdz), h4 = terrain_alt(T, p - N * step, kdz);
    double dhde = (h1 - h2) / (2.0 * step), dhdn = (h3 - h4) / (2.0 * step);
    return normalized(r - E * dhde - N * dhdn);
}
__device__ double ground_friction(const Terrain& T, V3 p) {
    if (!T.ok) return 0.6;
    V3 o = v3(T.meta[0], T.meta[1], T.meta[2]), E = v3(T.meta[3], T.meta[4], T.meta[5]), N = v3(T.meta[6], T.meta[7], T.meta[8]);
    V3 d = p - o; double e = dot(d, E), n = dot(d, N);
    int ng = (int)T.meta[14];
    for (int g = 0; g < ng; g++) {
        const double* gg = T.meta + 15 + 4 * g;
        double fe = round((e + gg[1]) / gg[0]), fn = round((n + gg[1]) / gg[0]); int Nn = (int)gg[2];
        if (fe >= 0.0 && fn >= 0.0 && (int)fe < Nn && (int)fn < Nn) return T.gr[(int)gg[3] + (int)fe * Nn + (int)fn] == 1 ? 0.9 : 0.6;
    }
    return 0.9;
}
// ray a - s u, s in [0, len]
__device__ bool ground_ray(const double* P, const Terrain& T, V3 a, V3 u, double len, double kdz, double* s_out, V3* p_out, V3* n_out) {
    double s = 0.0;
    if (T.ok) {
        for (int it = 0; it < 4; it++) {
            V3 p = a - u * s;
            double rg = T.meta[12] + terrain_alt(T, p, kdz);
            double au = dot(a, u);
            double disc = au * au - (norm2(a) - rg * rg);
            if (disc < 0.0) return false;
            s = au - sqrt(disc);
        }
    } else {
        double rg = P[P_RADIUS] + P[P_GROUND_ALT] + kdz;
        double au = dot(a, u);
        double disc = au * au - (norm2(a) - rg * rg);
        if (disc < 0.0) return false;
        s = au - sqrt(disc);
    }
    if (s < 0.0 || s > len) return false;
    *s_out = s; *p_out = a - u * s; *n_out = ground_normal(P, T, *p_out, kdz);
    return true;
}

// ---- helpers ---------------------------------------------------------------------------------
__device__ __forceinline__ float roundhe(float x) { float r = rintf(x); return r; }  // Mathf.Round: to even
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
__device__ void mat_vec(const double m[3][3], V3 v, V3* o) {
    *o = v3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}
__device__ void inv3(const double m[3][3], double o[3][3]) {
    double a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
    double c0 = e * i - f * h, c1 = -(d * i - f * g), c2 = d * h - e * g;
    double r = 1.0 / (a * c0 + b * c1 + c * c2);
    o[0][0] = c0 * r; o[0][1] = -(b * i - c * h) * r; o[0][2] = (b * f - c * e) * r;
    o[1][0] = c1 * r; o[1][1] = (a * i - c * g) * r; o[1][2] = -(a * f - c * d) * r;
    o[2][0] = c2 * r; o[2][1] = -(a * h - b * g) * r; o[2][2] = (a * e - b * d) * r;
}
__device__ __forceinline__ V3 rot_z(V3 p, double a) { double s = sin(a), c = cos(a); return v3(c * p.x - s * p.y, s * p.x + c * p.y, p.z); }

#define KPA_TO_ATM 0.009869232667160128
#define FIXED_DT 0.02
#define G0F ((double)9.80665f)

// ---- the tick --------------------------------------------------------------------------------
#define SV(f) state[(size_t)(f) * n + idx]
extern "C" __global__ void tick_kernel(const double* __restrict__ P, const double* __restrict__ tmeta, const float* __restrict__ talt,
                                       const unsigned char* __restrict__ tgr, int tok, double* __restrict__ state, const float* __restrict__ act,
                                       int n, int substeps)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;
    Terrain T; T.meta = tmeta; T.alt = talt; T.gr = tgr; T.ok = tok;

    const double throttle = fmin(fmax((double)act[4 * idx], 0.0), 1.0);
    const double pitch = fmin(fmax((double)act[4 * idx + 1], -1.0), 1.0);
    const double yaw = fmin(fmax((double)act[4 * idx + 2], -1.0), 1.0);
    const double roll = fmin(fmax((double)act[4 * idx + 3], -1.0), 1.0);
    const double dt = FIXED_DT;
    const V3 omega = v3(0, 0, P[P_OMEGA]);
    const int np = (int)P[P_N_PARTS], nl = (int)P[P_N_LEGS], nh = (int)P[P_N_HULL], ne = (int)P[P_N_ENG];

    for (int sub = 0; sub < substeps; sub++) {
        if (SV(S_STATUS) == 3.0) break;  // crashed
        V3 X = v3(SV(S_X), SV(S_X + 1), SV(S_X + 2)), Vv = v3(SV(S_V), SV(S_V + 1), SV(S_V + 2)), Wv = v3(SV(S_W), SV(S_W + 1), SV(S_W + 2));
        Q qq = qmk(SV(S_Q), SV(S_Q + 1), SV(S_Q + 2), SV(S_Q + 3));
        double res[4] = { SV(S_RES), SV(S_RES + 1), SV(S_RES + 2), SV(S_RES + 3) };
        // 1. mass props: linear in the resource amounts (P_MBASIS, see gpu.rs)
        double res0[4];
        for (int r = 0; r < 4; r++) res0[r] = res[r];
        double m = P[P_MBASIS], mc[3], jj[9], own9[9];
        for (int a = 0; a < 3; a++) mc[a] = P[P_MBASIS + 1 + a];
        for (int a = 0; a < 9; a++) { jj[a] = P[P_MBASIS + 4 + a]; own9[a] = P[P_MBASIS + 13 + a]; }
        for (int r = 0; r < 4; r++) {
            const double* B = P + P_MBASIS + (1 + r) * MB_STRIDE;
            if (P[P_RESCAPTOT + r] <= 0.0) continue;
            double w = res0[r];
            m += w * B[0];
            for (int a = 0; a < 3; a++) mc[a] += w * B[1 + a];
            for (int a = 0; a < 9; a++) { jj[a] += w * B[4 + a]; own9[a] += w * B[13 + a]; }
        }
        V3 com_l = v3(mc[0], mc[1], mc[2]) / m;
        double il[3][3], own[3][3];
        {
            double cv[3] = { com_l.x, com_l.y, com_l.z }; double c2 = norm2(com_l);
            for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) {
                il[a][b] = jj[3 * a + b] - m * ((a == b ? c2 : 0.0) - cv[a] * cv[b]);
                own[a][b] = own9[3 * a + b];
            }
        }
        V3 r_com = qrot(qq, com_l);
        V3 x_com = X + r_com;
        V3 v_com = Vv + cross(Wv, r_com);
        V3 v_srf = v_com;
        // 2. SmoothVelocity
        {
            float vf[3] = { (float)v_com.x, (float)v_com.y, (float)v_com.z };
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
        double sm[3] = { 0, 0, 0 };
        for (int j = 0; j < 10; j++) for (int c = 0; c < 3; c++) sm[c] += (double)(float)SV(S_SMOOTH + 3 * j + c);
        double spd = sqrt(sm[0] * sm[0] + sm[1] * sm[1] + sm[2] * sm[2]) * 0.1;
        double alt = norm(x_com) - P[P_RADIUS];
        // sun / latitude temperature offset (every 25 ticks), GetAtmoThermalStats
        if (P[P_SUN_TEMP] != 0.0 && alt < P[P_ATM_DEPTH]) {
            if (((unsigned int)SV(S_TICKS)) % 25u == 0u || !isfinite(SV(S_TOFF))) {
                double ut = SV(S_UT);
                double ang = P[P_ORB_MA0] + P[P_ORB_N] * (ut - P[P_ORB_EPOCH]);  // circular, equatorial
                V3 rk = v3(cos(ang), sin(ang), 0.0);
                double rotang = fmod(P[P_INIT_ROT_DEG] + 360.0 * (1.0 / P[P_SIDEREAL]) * ut, 360.0) * (M_PI / 180.0);
                V3 sun = rot_z(-rk, -rotang);
                double ta = atan2(rk.y, rk.x) - P[P_ORB_PE_LON_DEG] * (M_PI / 180.0);
                V3 up = normalized(x_com);
                V3 spin = v3(0, 0, 1);
                double d1 = dot(sun, spin), d2 = dot(spin, up);
                double colat0 = acos(d2); if (isnan(colat0)) colat0 = d2 > 0 ? 0.0 : M_PI;
                double sunp = acos(d1); if (isnan(sunp)) sunp = d1 > 0 ? 0.0 : M_PI;
                double maxf = (1.0 + cos(sunp - colat0)) * 0.5, minf = (1.0 + cos(sunp + colat0)) * 0.5;
                double colat = colat0 > M_PI / 2 ? M_PI - colat0 : colat0;
                double latd = (M_PI / 2 - colat) * 57.295780181884766;
                V3 up45 = qrot(angle_axis(45.0, spin), up);
                double todf = (1.0 + dot(sun, up45)) * 0.5;
                double diff = maxf - minf;
                double sunf;
                if (diff > 0.001) { sunf = (todf - minf) / diff; if (isnan(sunf)) sunf = todf > 0.5 ? 1.0 : 0.0; } else sunf = minf + diff * 0.5;
                double tad = fmod(ta * (180.0 / M_PI) + 360.0, 360.0);
                double off = CURVE(P, LATBIAS, latd) + CURVE(P, LATSUN, latd) * sunf + CURVE(P, AXBIAS, tad) * CURVE(P, AXMULT, latd);
                SV(S_TOFF) = (float)off;
            }
        } else SV(S_TOFF) = 0.0;
        double toff = P[P_SUN_TEMP] != 0.0 ? (double)(float)SV(S_TOFF) : 0.0;
        // atmosphere (CelestialBody.GetPressure / GetFullTemperature / GetDensity / GetSpeedOfSound)
        double p_kpa = (alt <= P[P_ATM_DEPTH] && alt < P[P_ATM_DEPTH]) ? CURVE(P, PRESSURE, alt) : 0.0;
        double rho = 0.0, csnd = 0.0, mach = 0.0, prem = 1.0;
        if (p_kpa > 0.0) {
            double tk = CURVE(P, TEMPERATURE, alt) + CURVE(P, SUNMULT, alt) * toff;
            rho = (tk > 0.0) ? p_kpa * 1000.0 * P[P_MOLAR] / (P[P_RGAS] * tk) : 0.0;
            csnd = rho > 0.0 ? sqrt(P[P_GAMMA] * p_kpa * 1000.0 / rho) : 0.0;
            mach = csnd > 0.0 ? spd / csnd : 0.0;
            prem = CURVE(P, PRE, rho * spd);
        }
        V3 force = v3(0, 0, 0), torque = v3(0, 0, 0);
        // 3a. engines + gimbal
        for (int k = 0; k < ne; k++) {
            const double* E = P + P_ENG0 + k * ENG_STRIDE;
            double cur = SV(S_THR + k);
            double tmin = E[0] > 0.0 ? E[1] / E[0] : 0.0;
            double req = tmin != 0.0 ? tmin + (1.0 - tmin) * throttle : throttle;
            if (E[6] == 0.0) cur = req;
            else if (fabs(cur - req) <= 1e-4) cur = req;
            else if (cur < req) cur = cur + (req - cur) * fmin(fmax(E[4] * dt, 0.0), 1.0);
            else cur = cur + (req - cur) * fmin(fmax(E[5] * dt, 0.0), 1.0);
            V3 epos = v3(E[9], E[10], E[11]), edir = v3(E[12], E[13], E[14]), epiv = v3(E[15], E[16], E[17]);
            V3 pos_w = qrot(qq, epos);
            double alt_e = norm(X + pos_w) - P[P_RADIUS];
            double p_atm = ((alt_e < P[P_ATM_DEPTH]) ? CURVE(P, PRESSURE, alt_e) : 0.0) * KPA_TO_ATM;
            double thrust = 0.0; bool out = false;
            if (cur > 0.0) {
                double isp = feval(E + 22, (int)E[2], p_atm);
                double mflow = E[0] * cur;
                double rthr = mflow * isp * G0F;
                double mass = mflow * dt;
                double nprop = 0.0;
                for (int r = 0; r < 4; r++) { double u = E[18 + r]; if (u > 0.0) { if (res[r] < mass * u * E[3]) out = true; nprop += 1.0; } }
                if (out) { cur = 0.0; }
                else {
                    double fs = 0.0;
                    for (int r = 0; r < 4; r++) { double u = E[18 + r]; if (u > 0.0) { double rq = mass * u; double got = fmax(fmin(rq, res[r]), 0.0); res[r] -= got; fs += got / rq; } }
                    thrust = rthr * fmin(fmax(fs / nprop, 0.0), 1.0);
                }
            }
            SV(S_THR + k) = cur;
            double sgn = com_l.y > epos.y ? 1.0 : -1.0;
            double tg[2] = { sgn * pitch * E[7], sgn * yaw * E[7] };
            for (int a = 0; a < 2; a++) {
                double gv = SV(S_GIMBAL + 2 * k + a);
                SV(S_GIMBAL + 2 * k + a) = E[8] > 0.0 ? gv + (tg[a] - gv) * fmin(fmax(E[8] * dt, 0.0), 1.0) : tg[a];
            }
            Q gq = qmul(angle_axis(SV(S_GIMBAL + 2 * k), v3(1, 0, 0)), angle_axis(-SV(S_GIMBAL + 2 * k + 1), v3(0, 0, 1)));
            V3 dir_w = qrot(qq, qrot(gq, edir));
            V3 tp_w = qrot(qq, epiv + qrot(gq, epos - epiv));
            V3 f = dir_w * thrust;
            force = force + f; torque = torque + cross(tp_w - r_com, f);
        }
        // 3b. reaction wheel
        {
            V3 tq = v3(pitch * P[P_RWTORQUE], roll * P[P_RWTORQUE + 1], yaw * P[P_RWTORQUE + 2]);
            double sum = fabs(pitch) + fabs(roll) + fabs(yaw);
            double t = fmin(fmax(P[P_RWRESP] * dt, 0.0), 1.0);
            V3 rw = v3(SV(S_RW), SV(S_RW + 1), SV(S_RW + 2));
            rw = rw + (tq - rw) * t;
            SV(S_RW) = rw.x; SV(S_RW + 1) = rw.y; SV(S_RW + 2) = rw.z;
            double ecr = P[P_RWEC] * sum * dt;
            double got = fmin(ecr, res[3]);
            res[3] -= got;
            if (sum > 0.0 && got >= 0.9 * ecr) torque = torque + qrot(qq, v3(-rw.x, rw.y, rw.z));
        }
        // 4. gravity + frame accelerations
        double r2 = norm2(x_com);
        V3 g = x_com * (-P[P_GM] / (r2 * sqrt(r2)));
        V3 a_frame = -(cross(omega, v_com) * 2.0) - cross(omega, cross(omega, x_com));
        // 5. aero per part + angular damping
        double damp[3][3] = {{0}};
        if (P[P_AERO] != 0.0 && rho > 0.0) {
            float tip = (float)CURVE(P, TIP, mach), surf = (float)CURVE(P, SURF, mach), tail = (float)CURVE(P, TAIL, mach), mult = (float)CURVE(P, MULT, mach), cdp = (float)CURVE(P, CDPOW, mach);
            double blm = CURVE(P, BLMACH, mach);
            // per-part loop in the root (vessel-local) frame: one rotation in, one out
            V3 vl = qrot(qconj(qq), v_com), wlc = qrot(qconj(qq), Wv);
            V3 fl = v3(0, 0, 0), tl = v3(0, 0, 0);
            for (int i = 0; i < np; i++) {
                double mpi = P[P_DRY + i];
                for (int r = 0; r < 4; r++) if (P[P_RESCAPTOT + r] > 0.0) mpi += P[P_RESCAP + 4 * i + r] * (res0[r] / P[P_RESCAPTOT + r]) * P[P_RESDENS + r];
                V3 pr = v3(P[P_PCOM + 3 * i], P[P_PCOM + 3 * i + 1], P[P_PCOM + 3 * i + 2]) - com_l;
                V3 vp = vl + cross(wlc, pr);
                double s2 = norm2(vp);
                if (s2 <= 0.0) continue;
                double qp = rho * 0.0005 * s2;
                double ci = fmax(P[P_ANGDRAG + i] * (qp * KPA_TO_ATM) * 2.0, 0.0);
                for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) damp[a][b] += ci * mpi * P[P_PINERT + 9 * i + 3 * a + b];
                V3 vh = vp / sqrt(s2);
                Q prot = qmk(P[P_PROT + 4 * i], P[P_PROT + 4 * i + 1], P[P_PROT + 4 * i + 2], P[P_PROT + 4 * i + 3]);
                V3 d = qrot(qconj(prot), vh);
                float comp[3] = { (float)d.x, (float)d.y, (float)d.z };
                bool bl = P[P_BODYLIFT + i] != 0.0;
                float ad = 0.f, ar = 0.f; float ll[3] = { 0.f, 0.f, 0.f };
                for (int j = 0; j < 6; j++) {
                    float dt_ = (j & 1) == 0 ? comp[j >> 1] : -comp[j >> 1];
                    float d01 = (dt_ + 1.f) * 0.5f;
                    float sc = d01 <= 0.5f ? (tail + (surf - tail) * (2.f * d01)) * mult : (surf + (tip - surf) * (2.f * d01 - 1.f)) * mult;
                    float fa = (float)P[P_FAREA + 6 * i + j];
                    float a = fa * sc;
                    ar += a;
                    float l = (float)P[P_FLNCD + 6 * i + j];
                    float cd = isnan(l) ? (float)P[P_FDRAG + 6 * i + j] : expf(cdp * l);
                    ad += a * cd;
                    if (dt_ > 0.f && bl) {
                        float lv = dt_ * fa * (float)P[P_FDRAG + 6 * i + j] * body_lift_curve32(P, dt_);
                        ll[j >> 1] += ((j & 1) == 0 ? -1.f : 1.f) * lv;
                    }
                }
                if (ar <= 0.f) ad = 0.f;
                double drag = qp * (double)ad * 0.1 * prem * 8.0;
                V3 fd = vh * (-drag);
                fl = fl + fd;
                tl = tl + cross(v3(P[P_PCOP + 3 * i], P[P_PCOP + 3 * i + 1], P[P_PCOP + 3 * i + 2]) - com_l, fd);
                if (bl) {
                    double blv = qp * P[P_BODYLIFT + i] * 18.0 * blm;
                    V3 l = qrot(prot, v3((double)ll[0], (double)ll[1], (double)ll[2]) * blv);
                    V3 lift = l - vh * dot(l, vh);
                    fl = fl + lift;
                    tl = tl + cross(v3(P[P_PCOL + 3 * i], P[P_PCOL + 3 * i + 1], P[P_PCOL + 3 * i + 2]) - com_l, lift);
                }
            }
            force = force + qrot(qq, fl); torque = torque + qrot(qq, tl);
        }
        double ilinv[3][3]; inv3(il, ilinv);
        // 6. ground contact (scenery translated by the per-env ground shift: query at p - GS)
        const V3 GS = v3(SV(S_GSHIFT), SV(S_GSHIFT + 1), SV(S_GSHIFT + 2));
        const double KDZ = SV(S_KSCDZ);
        int legs = 0; bool contact_any = false;
        V3 cr[MAX_HULL], cn[MAX_HULL]; double cgap[MAX_HULL], ctol[MAX_HULL], cmu[MAX_HULL]; int chull[MAX_HULL]; int nc = 0;
        const double margin = P[P_BOUND] + 10.0 + norm(v_com) * dt;
        // exact early-out above the highest ground point (sim::ground_ceiling)
        const bool above_all = norm(X) - norm(GS) - fmax(KDZ, 0.0) - P[P_GROUND_CEIL] >= margin;
        bool near_ground = P[P_GROUND] != 0.0 && !above_all && ground_height(P, T, X - GS, KDZ) < margin;
        if (!near_ground) {
            for (int k = 0; k < nl; k++) { int lb = S_LEGS + k * LEG_FIELDS; SV(lb) = 0; SV(lb + 1) = 0; SV(lb + 2) = 0; SV(lb + 16) = 0; SV(lb + 17) = 0; }
        } else {
            double gm = norm(g);
            int ng = 0; for (int k = 0; k < nl; k++) if (SV(S_LEGS + k * LEG_FIELDS) != 0.0) ng++;
            for (int k = 0; k < nl; k++) {
                const double* Ls = P + P_LEG0 + k * LEG_STRIDE;
                int lb = S_LEGS + k * LEG_FIELDS;  // leg fields are field-major too
#define L(j) SV(lb + (j))
                // wheel::update_spring (float, as the game)
                bool grounded = L(0) != 0.0;
                float lx = (float)L(1), lhf = (float)L(2), lk = (float)L(3), lc = (float)L(4), ltg = (float)L(5);
                float msp = (float)L(6), mdm = (float)L(7), stw = (float)L(8), dtw = (float)L(9), ab = (float)L(10);
                bool aset = L(11) != 0.0;
                float rate = (float)L(12), drate = (float)L(13), rt = (float)L(14), rtd = (float)L(15);
                float tire0 = (float)L(16), tire1 = (float)L(17);
                if (grounded) {
                    double tt = m;
                    if (ng > 0) tt = round(tt * P[P_GEE_ASL] * 10.0) / 10.0;
                    float share = roundhe(100.f / (float)(ng > 0 ? ng : 1)) / 100.f;
                    float smg = fmaxf((float)(tt * (double)share), 0.5f);
                    float mm = (float)m;
                    float sc = (float)((double)lx / Ls[10]);
                    float num = mm * sc;
                    float cmax = lerpf(5000.f, 10000.f, invlerpf(50.f, 200.f, mm));
                    if (sc > 0.8f) ab = fminf(fmaxf(ab + 0.2f * (float)dt, 0.85f), 2.f);
                    else if (sc < 0.4f) ab = fmaxf(ab - 0.2f * (float)dt, 0.f);
                    float boost = 1.f + fminf(fmaxf(sc * boost_curve((float)P[P_GEE_ASL], 1.f), -1.f), 0.5f);
                    bool rearm = !aset || (Ls[15] > 0.0 && num > (float)Ls[15]);
                    if (rearm) {
                        dtw = P[P_GEE_ASL] < 1.0 ? 1.1f : round2f(lerpf(1.f, 2.f, invlerpf(1.f, (float)P[P_MAX_SURF_GEE], (float)P[P_GEE_ASL])));
                        if (msp > 0.f) { float v = (float)Ls[11] * msp / 9.80665f; stw = round2f(lerpf(1.f, 3.f, invlerpf(0.f, (float)Ls[15] * 2.f, v))); aset = true; }
                    }
                    float fudge = lerpf(1.f, 15.f, invlerpf(10.f, 100.f, num));
                    float spring = fminf(fmaxf(stw * (float)Ls[12] * smg * boost_curve(boost, (float)Ls[14] + ab), 0.01f), cmax);
                    float damper = sqrtf(fminf(fmaxf(spring * (float)Ls[13] * fudge, 0.01f), cmax)) * dtw;
                    if (P[P_GEE_ASL] > 1.5) { spring *= 3.f; damper *= 1.2f; }
                    mdm = fmaxf(roundhe(damper), 0.1f); msp = fmaxf(roundhe(spring), 0.1f);
                    if (fabsf(msp - rate) > 3.f) rate = msp;
                    if (fabsf(mdm - drate) > 1.05f) drate = mdm;
                }
                float wgt = (float)(Ls[16] * gm); float d = (float)Ls[10];
                lk = fmaxf(rt, fmaxf(wgt / d, 0.1f)); lc = fmaxf(rtd, 0.1f);
                float tgt = fminf(fmaxf(wgt / lk / d, 0.1f), 1.f);
                ltg = (isfinite(ltg) && fabsf(tgt - ltg) > 0.02f) ? (ltg > tgt ? ltg - 0.02f : ltg + 0.02f) : tgt;
                rt = rate; rtd = drate;
                // wheel collider ray
                V3 a = X + qrot(qq, v3(Ls[0], Ls[1], Ls[2]));
                V3 u = qrot(qq, v3(Ls[3], Ls[4], Ls[5]));
                double len = Ls[10] + Ls[9];
                double s_hit; V3 p, nn;
                if (ground_ray(P, T, a - GS, u, len, KDZ, &s_hit, &p, &nn)) {
                    p = p + GS;
                    double ndu = dot(nn, u);
                    if (grounded) {
                        V3 vf = v_com + cross(Wv, p - x_com);
                        V3 vt = vf - nn * dot(vf, nn);
                        V3 fw = qrot(qq, v3(Ls[6], Ls[7], Ls[8]));
                        V3 fwd = normalized(fw - nn * dot(fw, nn));
                        V3 side = cross(nn, fwd);
                        float vl0 = (float)dot(vt, side), vl1 = (float)dot(vt, fwd);
                        double ca = atan2(dot(u, side), ndu);
                        float load = (float)fmax((double)lhf * cos(ca), Ls[18]);
                        float mf = (float)((double)fmaxf(lhf, 0.f) / gm);
                        float fe0 = -0.5f * mf * vl0 / (float)dt, fe1 = -0.5f * mf * vl1 / (float)dt;
                        float smag = sqrtf(vl0 * vl0 + vl1 * vl1);
                        float cap0, cap1;
                        if (smag < 1e-6f) { cap0 = 0.f; cap1 = 0.f; }
                        else if (smag < 0.01f) { float kk = mu_curve(0.01f) * load / 0.01f; cap0 = kk * fabsf(vl0); cap1 = kk * sqrtf(fmaxf(0.01f * 0.01f - vl0 * vl0, 0.f)); }
                        else { float ff = mu_curve(smag) * load; cap0 = ff * fabsf(vl0) / smag; cap1 = ff * fabsf(vl1) / smag; }
                        float f0 = fminf(fmaxf(fe0, -cap0), cap0), f1 = fminf(fmaxf(fe1, -cap1), cap1);
                        float smo = fminf(fmaxf(10.f * (float)dt, 0.f), 1.f);
                        tire0 = tire0 + (f0 - tire0) * smo; tire1 = tire1 + (f1 - tire1) * smo;
                        V3 ft = side * (double)tire0 + fwd * (double)tire1;
                        force = force + ft; torque = torque + cross(p - x_com, ft);
                    } else { tire0 = 0.f; tire1 = 0.f; }
                    double x = len - s_hit;
                    double xp = grounded ? (double)lx : 0.0;
                    double fs = (double)lk * (x - (double)ltg * Ls[10]) + (double)lc * (x - xp) / dt;
                    double rep = fs * ndu;
                    V3 fc = nn * rep;
                    force = force + fc; torque = torque + cross(p - x_com, fc);
                    double vn = dot(v_com + cross(Wv, p - x_com), nn);
                    SV(S_IMPACT) = fmax(SV(S_IMPACT), (double)(float)(-vn));
                    grounded = true; lx = (float)x; lhf = (float)rep;
                    legs++; contact_any = true;
                } else { grounded = false; lx = 0.f; lhf = 0.f; tire0 = 0.f; tire1 = 0.f; }
                L(0) = grounded ? 1.0 : 0.0; L(1) = lx; L(2) = lhf; L(3) = lk; L(4) = lc; L(5) = ltg; L(6) = msp; L(7) = mdm; L(8) = stw; L(9) = dtw;
                L(10) = ab; L(11) = aset ? 1.0 : 0.0; L(12) = rate; L(13) = drate; L(14) = rt; L(15) = rtd; L(16) = tire0; L(17) = tire1;
#undef L
            }
            for (int k = 0; k < nh; k++) {
                V3 p = X + qrot(qq, v3(P[P_HULL + 3 * k], P[P_HULL + 3 * k + 1], P[P_HULL + 3 * k + 2]));
                double gap = ground_height(P, T, p - GS, KDZ);
                if (gap < 0.01 && nc < MAX_HULL) {
                    cr[nc] = p - x_com; cn[nc] = ground_normal(P, T, p - GS, KDZ); cgap[nc] = gap; ctol[nc] = P[P_HULLTOL + k]; chull[nc] = k; cmu[nc] = ground_friction(T, p - GS);
                    nc++;
                }
            }
        }
        // 7. integrate (Euler; the CPU's vacuum drift-compensation branch is not ported)
        V3 a_ng = force / m;
        V3 v_com_n = v_com + (a_ng + g + a_frame) * dt;
        V3 x_com_n = x_com + v_com_n * dt;
        double iorb[3][3];
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) iorb[a][b] = il[a][b] - own[a][b];
        V3 wl = qrot(qconj(qq), Wv);
        V3 tmp; mat_vec(iorb, wl, &tmp);
        V3 gyro = cross(wl, tmp);
        V3 al; mat_vec(ilinv, qrot(qconj(qq), torque) - gyro, &al);
        V3 wl2 = wl + al * dt;
        V3 dw; mat_vec(damp, wl2, &tmp); mat_vec(ilinv, tmp, &dw);
        wl2 = wl2 - dw * dt;
        V3 w = qrot(qq, wl2);
        if (nc == 0) SV(S_HTOUCH) = 0.0;
        if (nc > 0) {
            V3 vv = v_com_n;
            // Part.OnCollisionEnter -> HandleCollision: a new contact on a primary collider
            // destroys the part if its rigidbody speed exceeds crashTolerance (sim.rs)
            unsigned int touch = 0u, prev = (unsigned int)SV(S_HTOUCH);
            for (int c = 0; c < nc; c++) if (cgap[c] <= 0.01) {
                contact_any = true; double vn = dot(vv + cross(w, cr[c]), cn[c]); SV(S_IMPACT) = fmax(SV(S_IMPACT), (double)(float)(-vn));
                int k = chull[c]; touch |= 1u << k;
                if (P[P_CRASH] != 0.0 && P[P_HULLCRASH + k] != 0.0 && !(prev & (1u << k))) {
                    int pi = (int)P[P_HULLPART + k];
                    V3 rp = qrot(qq, v3(P[P_PCOM + 3 * pi], P[P_PCOM + 3 * pi + 1], P[P_PCOM + 3 * pi + 2]) - com_l);
                    if (norm(vv + cross(w, rp)) > ctol[c]) SV(S_STATUS) = 3.0;
                }
            }
            SV(S_HTOUCH) = (double)touch;
            double ln[MAX_HULL]; V3 lt[MAX_HULL];
            for (int c = 0; c < nc; c++) { ln[c] = 0.0; lt[c] = v3(0, 0, 0); }
            // patch warm start (sim.rs): one centroid constraint per part / normal, shared evenly
            {
                unsigned int done = 0u;
                for (int a = 0; a < nc; a++) {
                    if (done & (1u << a)) continue;
                    int pa = (int)P[P_HULLPART + chull[a]];
                    unsigned int grp = 0u; int cnt = 0;
                    for (int b = a; b < nc; b++)
                        if (!(done & (1u << b)) && (int)P[P_HULLPART + chull[b]] == pa && dot(cn[b], cn[a]) > 0.999) { grp |= 1u << b; cnt++; }
                    done |= grp;
                    if (cnt < 2) continue;
                    double inv = 1.0 / cnt; V3 rb = v3(0, 0, 0), nb = v3(0, 0, 0); double tb = 0.0;
                    for (int b = a; b < nc; b++) if (grp & (1u << b)) {
                        rb = rb + cr[b] * inv; nb = nb + cn[b] * inv;
                        tb += inv * (cgap[b] >= 0.0 ? -cgap[b] / dt : -cgap[b] / dt * 0.55);
                    }
                    nb = normalized(nb);
                    double vn = dot(vv + cross(w, rb), nb);
                    V3 rn = cross(rb, nb);
                    V3 irn; mat_vec(ilinv, qrot(qconj(qq), rn), &irn); irn = qrot(qq, irn);
                    double lam = fmax((tb - vn) / (1.0 / m + dot(rn, irn)), 0.0);
                    for (int b = a; b < nc; b++) if (grp & (1u << b)) {
                        double dl = lam * inv; ln[b] += dl;
                        vv = vv + cn[b] * (dl / m);
                        V3 ij; mat_vec(ilinv, qrot(qconj(qq), cross(cr[b], cn[b])), &ij); w = w + qrot(qq, ij) * dl;
                    }
                }
            }
            for (int it = 0; it < 8; it++) {
                for (int c = 0; c < nc; c++) {
                    double target = cgap[c] >= 0.0 ? -cgap[c] / dt : -cgap[c] / dt * 0.55;
                    V3 vp = vv + cross(w, cr[c]);
                    double vn = dot(vp, cn[c]);
                    V3 rn = cross(cr[c], cn[c]);
                    V3 irn; mat_vec(ilinv, qrot(qconj(qq), rn), &irn); irn = qrot(qq, irn);
                    double kn = 1.0 / m + dot(rn, irn);
                    double nw = fmax(ln[c] + (target - vn) / kn, 0.0);
                    double dj = nw - ln[c]; ln[c] = nw;
                    vv = vv + cn[c] * (dj / m); w = w + irn * dj;
                    vp = vv + cross(w, cr[c]);
                    V3 vt = vp - cn[c] * dot(vp, cn[c]);
                    double vtn = norm(vt);
                    if (vtn > 1e-12) {
                        V3 td = vt / vtn; V3 rtt = cross(cr[c], td);
                        V3 irt; mat_vec(ilinv, qrot(qconj(qq), rtt), &irt); irt = qrot(qq, irt);
                        double kt = 1.0 / m + dot(rtt, irt);
                        V3 want = lt[c] - td * (vtn / kt);
                        double cap = cmu[c] * ln[c];
                        V3 ltn = norm(want) > cap ? normalized(want) * cap : want;
                        V3 djv = ltn - lt[c]; lt[c] = ltn;
                        vv = vv + djv / m;
                        V3 ij; mat_vec(ilinv, qrot(qconj(qq), cross(cr[c], djv)), &ij); w = w + qrot(qq, ij);
                    }
                }
            }
            V3 dvv = vv - v_com_n;
            x_com_n = x_com_n + dvv * dt; v_com_n = vv;
        }
        double wn = norm(w);
        if (wn > 50.0) w = w * (50.0 / wn);
        Q qn = integrate_world(qq, w, dt);
        V3 rcn = qrot(qn, com_l);
        X = x_com_n - rcn; Vv = v_com_n - cross(w, rcn);
        SV(S_X) = X.x; SV(S_X + 1) = X.y; SV(S_X + 2) = X.z;
        SV(S_V) = Vv.x; SV(S_V + 1) = Vv.y; SV(S_V + 2) = Vv.z;
        SV(S_Q) = qn.x; SV(S_Q + 1) = qn.y; SV(S_Q + 2) = qn.z; SV(S_Q + 3) = qn.w;
        SV(S_W) = w.x; SV(S_W + 1) = w.y; SV(S_W + 2) = w.z;
        for (int r = 0; r < 4; r++) SV(S_RES + r) = res[r];
        SV(S_UT) = SV(S_UT) + dt;
        SV(S_TICKS) = SV(S_TICKS) + 1.0;
        if (SV(S_STATUS) != 3.0) {
            if (legs == nl && legs > 0) {
                bool still = norm(v_srf) < 0.3 && norm(w) < 0.1;
                SV(S_SETTLED) = still ? fmin(SV(S_SETTLED) + 1.0, 65535.0) : 0.0;
                SV(S_STATUS) = SV(S_SETTLED) >= 25.0 ? 2.0 : 1.0;
            } else { SV(S_SETTLED) = 0.0; SV(S_STATUS) = contact_any ? 1.0 : 0.0; }
        }
    }
}
