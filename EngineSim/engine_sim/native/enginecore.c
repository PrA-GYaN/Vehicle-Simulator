/* Compiled inner loop of the resolved engine (engine_sim/cylinder.py + cylinder_bank.py).
 *
 * A line-for-line transcription of Cylinder.step, begin_cycle and the substep loop of
 * CylinderBank.advance, including the exhaust plenum / turbine stage and the boost plenum,
 * so the compiled and the Python paths agree to rounding. State lives in flat double
 * arrays whose layout (fields.h) is generated from the Python attribute lists in
 * native_core.py; Python copies a cylinder's attributes in before the call and out after.
 * Build flags must not enable fast-math or FMA contraction (-ffp-contract=off).
 */
#include <math.h>
#include <string.h>
#include "fields.h"

#define R_AIR 287.058
#define GAMMA 1.4
#define N_COMPRESSION 1.32
#define CV_AIR_T (R_AIR / (GAMMA - 1.0))
#define CP_AIR_T (GAMMA * CV_AIR_T)
#define GRID_N 251
#define LAST_NODE (GRID_N - 1)
#define GRID_STEP 20.0
#define GRID_INV (1.0 / 20.0)
#define MIN_T 50.0
#define TRACE_BINS 180
#define MIN_PLENUM_MASS 1.0e-3

static double CVA[GRID_N], CVM[GRID_N], CVB[GRID_N];
static double UA[GRID_N], UM[GRID_N], UB[GRID_N];
static double DUA[GRID_N], DUM[GRID_N];

static double PR_CRIT, C_CHOKED, C_SUBSONIC, E_LOW, E_HIGH;
static double VM_PR_CRIT, VM_C_CHOKED, VM_C_SUB1;

void es_init_tables(const double *t)
{
    memcpy(CVA, t + 0 * GRID_N, sizeof CVA);
    memcpy(CVM, t + 1 * GRID_N, sizeof CVM);
    memcpy(CVB, t + 2 * GRID_N, sizeof CVB);
    memcpy(UA, t + 3 * GRID_N, sizeof UA);
    memcpy(UM, t + 4 * GRID_N, sizeof UM);
    memcpy(UB, t + 5 * GRID_N, sizeof UB);
    memcpy(DUA, t + 6 * GRID_N, sizeof DUA);
    memcpy(DUM, t + 7 * GRID_N, sizeof DUM);
    PR_CRIT = pow(2.0 / (GAMMA + 1.0), GAMMA / (GAMMA - 1.0));
    C_CHOKED = sqrt(2.0 / R_AIR * GAMMA / (GAMMA + 1.0)
                    * pow(2.0 / (GAMMA + 1.0), 1.0 / (GAMMA - 1.0)));
    C_SUBSONIC = sqrt(2.0 * GAMMA / (R_AIR * (GAMMA - 1.0)));
    E_LOW = 2.0 / GAMMA;
    E_HIGH = (GAMMA + 1.0) / GAMMA;
    VM_PR_CRIT = PR_CRIT;
}

/* Python float % */
static inline double pymod(double a, double b)
{
    double r = fmod(a, b);
    if (r != 0.0) {
        if ((b < 0.0) != (r < 0.0)) r += b;
    } else {
        r = copysign(0.0, b);
    }
    return r;
}

static inline int node_of(double t)
{
    int n = (int)(t * GRID_INV);
    if (n > LAST_NODE) n = LAST_NODE;
    if (n < 0) n = 0;
    return n;
}

static inline double fmax_(double a, double b) { return a > b ? a : b; }
static inline double fmin_(double a, double b) { return a < b ? a : b; }

static double fall(double x, double lo, double hi)
{
    if (lo == hi) return x < lo ? 0.0 : 1.0;
    double t = (x - lo) / (hi - lo);
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * (3.0 - 2.0 * t);
}

static double u_burned(double t)
{
    int n = node_of(t);
    return UB[n] + (t - n * GRID_STEP) * CVB[n];
}
static double u_mix(double t, double y, const double *uu, const double *cvu)
{
    int n = node_of(t);
    double d = t - n * GRID_STEP;
    double u = uu[n] + d * cvu[n];
    return u + (UB[n] + d * CVB[n] - u) * y;
}
static double cv_burned(double t) { return CVB[node_of(t)]; }
static double cv_mix(double t, double y, const double *cvu)
{
    int n = node_of(t);
    return cvu[n] + (CVB[n] - cvu[n]) * y;
}
static double invert_h_burned(double h, double guess)
{
    double t = guess > 200.0 ? guess : 200.0;
    for (int it = 0; it < 4; it++) {
        int n = node_of(t);
        double cp = CVB[n] + R_AIR;
        t += (h - (UB[n] + (t - n * GRID_STEP) * CVB[n] + R_AIR * t)) / cp;
        if (t < 200.0) t = 200.0;
        else if (t > 5000.0) t = 5000.0;
    }
    return t;
}

static double orifice(double acd, double p_cyl, double p_port, double t_cyl, double t_port)
{
    double up, dn, tu, sign;
    if (p_cyl >= p_port) { up = p_cyl; dn = p_port; tu = t_cyl; sign = -1.0; }
    else { up = p_port; dn = p_cyl; tu = t_port; sign = 1.0; }
    double pr = dn / up;
    if (pr <= PR_CRIT) return sign * acd * up * C_CHOKED / sqrt(tu);
    double b = pow(pr, E_LOW) - pow(pr, E_HIGH);
    if (b <= 0.0) return 0.0;
    return sign * acd * up * C_SUBSONIC * sqrt(b / tu);
}

/* valve_flow.valve_mass_flow with gamma = 1.4 */
static double vmf(double p_up, double p_down, double t_up, double area, double cd)
{
    if (area <= 0.0 || p_up <= p_down || t_up <= 0.0) return 0.0;
    double pr = p_down / p_up;
    double g = GAMMA;
    double pr_crit = pow(2.0 / (g + 1.0), g / (g - 1.0));
    if (pr <= pr_crit) {
        return (cd * area * p_up
                * sqrt(2.0 / (R_AIR * t_up) * g / (g + 1.0) * pow(2.0 / (g + 1.0), 1.0 / (g - 1.0))));
    }
    double bracket = fmax_(0.0, pow(pr, 2.0 / g) - pow(pr, (g + 1.0) / g));
    return (cd * area * p_up * sqrt(2.0 * g / (R_AIR * t_up * (g - 1.0)) * bracket));
}

typedef struct {
    const double *K;
    const double *vol, *ain, *aex;
    double *trace;
} Ctx;

#define C(n) c[F_##n]
#define KK(n) (x->K[K_##n])

/* ── zone helpers ─────────────────────────────────────────────────────── */

static void sel_tables(const double *c, const double **cvu, const double **uu, const double **dub)
{
    if (C(_fuel_tab) != 0.0) { *cvu = CVM; *uu = UM; *dub = DUM; }
    else { *cvu = CVA; *uu = UA; *dub = DUA; }
}

static void merge_zones(double *c)
{
    const double *cvu, *uu, *dub;
    sel_tables(c, &cvu, &uu, &dub);
    double mu = C(_mu), mb = C(_mb);
    double m = mu + mb;
    C(_zone_active) = 0.0;
    C(unburned_temperature_k) = 0.0;
    if (m <= 0.0) return;
    double yu = C(_yu);
    double u_total = mu * u_mix(C(_tu), yu, uu, cvu) + mb * u_burned(C(_tb));
    double y = (mb + yu * mu) / m;
    double t = (mu * C(_tu) + mb * C(_tb)) / m;
    for (int it = 0; it < 3; it++) {
        int n = node_of(t);
        double cv = cvu[n] + (CVB[n] - cvu[n]) * y;
        t += (u_total / m - u_mix(t, y, uu, cvu)) / cv;
    }
    C(temperature) = fmax_(MIN_T, t);
    C(burned_mass) = mb + yu * mu;
    C(mass) = m;
}

static double nox_rate(double t, double p, double o2, double n2, double dt)
{
    if (t < 1500.0) return 0.0;
    double A = 6.0e16, E_R = 69090.0, R_cal = 82.057;
    double p_atm = p / 101325.0;
    double conc_total = p_atm / (R_cal * t);
    double o2c = o2 * conc_total;
    double n2c = n2 * conc_total;
    double d = (A / sqrt(t) * exp(-E_R / t) * sqrt(fmax_(0.0, o2c)) * n2c);
    return d * dt;
}
static double nox_eq(double t, double p, double o2, double n2)
{
    if (t < 1500.0) return 0.0;
    double K_NO = 3.6e3 * exp(-31090.0 / t);
    double p_atm = p / 101325.0;
    double R_cal = 82.057;
    double conc_total = p_atm / (R_cal * t);
    double o2c = o2 * conc_total;
    double n2c = n2 * conc_total;
    double no_eq = sqrt(K_NO * o2c * n2c);
    return no_eq / conc_total;
}

static void two_zone_step(const Ctx *x, double *c, double dt, double v0, double v1, double q_chem,
                          double dx, double q_wall, double t_mean, double pressure, double mass,
                          double wall_t, double *work_out, double *p_out)
{
    const double R = R_AIR;
    const double *cvu, *uu_tab, *dub;
    sel_tables(c, &cvu, &uu_tab, &dub);
    if (C(_zone_active) == 0.0) {
        C(_zone_active) = 1.0;
        C(_mu) = mass; C(_mb) = 0.0;
        C(_tu) = t_mean; C(_tb) = t_mean;
        C(_yu) = mass > 0.0 ? C(burned_mass) / mass : 0.0;
        C(_m_at_spark) = mass;
    }
    double mu = C(_mu), mb = C(_mb), tu = C(_tu), tb = C(_tb), yu = C(_yu);
    double u_before = ((mu > 0.0 ? mu * u_mix(tu, yu, uu_tab, cvu) : 0.0)
                       + (mb > 0.0 ? mb * u_burned(tb) : 0.0));
    double q_lost = 0.0;

    if (q_wall != 0.0) {
        double wu = tu > wall_t ? mu * (tu - wall_t) : 0.0;
        double wb = tb > wall_t ? mb * (tb - wall_t) : 0.0;
        double tot = wu + wb;
        if (tot > 0.0) {
            double qw = q_wall * dt / tot;
            q_lost = q_wall * dt;
            if (mu > 0.0 && wu > 0.0) tu -= qw * wu / (mu * (cv_mix(tu, yu, cvu) + R));
            if (mb > 0.0 && wb > 0.0) tb -= qw * wb / (mb * (cv_burned(tb) + R));
        }
    }
    double dm_b = dx * C(_m_at_spark);
    if (dm_b > mu) dm_b = mu;
    if (dm_b > 0.0) {
        double h_f = u_mix(tu, yu, uu_tab, cvu) + R * tu + q_chem / dm_b;
        double t_f = invert_h_burned(h_f, tu + q_chem / (dm_b * (cv_burned(tu) + R)));
        if (mb > 0.0) {
            double h_mix = (mb * (u_burned(tb) + R * tb) + dm_b * h_f) / (mb + dm_b);
            tb = invert_h_burned(h_mix, (mb * tb + dm_b * t_f) / (mb + dm_b));
            C(no_mole_fraction) *= mb / (mb + dm_b);
        } else {
            tb = t_f;
        }
        mb += dm_b;
        mu -= dm_b;
    }

    double p0 = pressure;
    double tu1 = tu, tb1 = tb, p1 = p0;
    double cv_u = mu > 0.0 ? cv_mix(tu, yu, cvu) : 1.0;
    double cv_b = mb > 0.0 ? cv_burned(tb) : 1.0;
    for (int sweep = 0; sweep < 2; sweep++) {
        double ku = R / (cv_u + R);
        double kb = R / (cv_b + R);
        double ln_p = log(p0);
        p1 = p0;
        for (int it = 0; it < 4; it++) {
            double p_ratio = p1 / p0;
            double vu = mu > 0.0 ? mu * R * tu * pow(p_ratio, ku) / p1 : 0.0;
            double vb = mb > 0.0 ? mb * R * tb * pow(p_ratio, kb) / p1 : 0.0;
            double vol = vu + vb;
            double kappa = vol > 0.0 ? (vu * ku + vb * kb) / vol : 0.0;
            ln_p += log(vol / v1) / (1.0 - kappa);
            p1 = exp(ln_p);
        }
        tu1 = mu > 0.0 ? tu * pow(p1 / p0, ku) : tu;
        tb1 = mb > 0.0 ? tb * pow(p1 / p0, kb) : tb;
        if (sweep == 0) {
            if (mu > 0.0) cv_u = cv_mix(0.5 * (tu + tu1), yu, cvu);
            if (mb > 0.0) cv_b = cv_burned(0.5 * (tb + tb1));
        }
    }
    double u_after = ((mu > 0.0 ? mu * u_mix(tu1, yu, uu_tab, cvu) : 0.0)
                      + (mb > 0.0 ? mb * u_burned(tb1) : 0.0));
    double work = u_before + q_chem - q_lost - u_after;
    C(_mu) = mu; C(_mb) = mb; C(_tu) = tu1; C(_tb) = tb1;

    if (mb > 0.0 && tb1 > 1700.0) {
        double phi = C(_phi_cycle);
        double x_o2 = phi < 1.0 ? fmax_(0.003, 0.20 * (1.0 - phi)) : 0.003;
        double d_no = nox_rate(tb1, p1, x_o2, 0.72, dt);
        double conc = (p1 / 101325.0) / (82.057 * tb1);
        double x_eq = nox_eq(tb1, p1, x_o2, 0.72);
        C(no_mole_fraction) = fmin_(x_eq, C(no_mole_fraction) + d_no / conc);
    }
    if (tb1 > C(peak_burned_temperature_k)) C(peak_burned_temperature_k) = tb1;

    double m = mu + mb;
    if (mu <= 1.0e-9 * m) {
        C(_zone_active) = 0.0;
        C(temperature) = tb1;
        C(burned_mass) = m;
        C(unburned_temperature_k) = 0.0;
        C(last_no_ppm) = C(no_mole_fraction) * 1.0e6;
    } else {
        C(temperature) = (mu * tu1 + mb * tb1) / m;
        C(burned_mass) = mb + yu * mu;
        C(unburned_temperature_k) = tu1;
    }
    *work_out = work;
    *p_out = p1;
}

static void accumulate_knock(const Ctx *x, double *c, double angle, double dt)
{
    if (!(C(_ivc_deg) <= angle && angle < C(_exhaust_open))) return;
    if (C(burned_fraction) > 0.9 || C(mass) <= 0.0) return;
    double t_u;
    if (C(_zone_active) != 0.0) {
        t_u = C(unburned_temperature_k);
    } else if (C(combustion_occurred) != 0.0 && C(_motoring_reference_pa) > 0.0) {
        t_u = C(_motoring_reference_k)
              * pow(C(pressure) / C(_motoring_reference_pa), (N_COMPRESSION - 1.0) / N_COMPRESSION);
    } else {
        t_u = C(temperature);
    }
    double p = C(pressure);
    double tau;
    if (t_u <= 0.0 || p <= 0.0) {
        tau = 100.0;
    } else {
        double p_atm = fmax_(p / 101325.0, 0.01);
        double on_term = pow(fmax_(KK(octane), 1.0) / 100.0, 3.402);
        double tau_ms = 17.68 * on_term * pow(p_atm, -1.7) * exp(3800.0 / t_u);
        tau = fmax_(1.0e-6, tau_ms / 1000.0);
    }
    if (tau > 0.0) C(knock_integral) += dt / tau;
    if (C(knock_integral) >= 1.0 && C(knock_detected) == 0.0) {
        C(knock_detected) = 1.0;
        C(knock_events) += 1.0;
    }
}

static double burn_quality(const Ctx *x, double *c, double mass, double burned_mass)
{
    double y = mass > 0.0 ? burned_mass / mass : 0.0;
    C(residual_at_spark) = y;
    double q = 1.0 - fall(y, KK(res_lo), KK(res_hi));
    double fuel = C(_fuel_in_cycle);
    if (fuel > 0.0) {
        double lam = (mass - burned_mass - fuel) / (fuel * KK(stoich));
        C(_phi_cycle) = lam > 0.0 ? 1.0 / lam : 1.0;
        double a = 1.0 - fall(lam, KK(lean_lo), KK(lean_hi));
        double b = 1.0 - fall(lam, KK(rich_lo), KK(rich_hi));
        q = fmin_(q, fmin_(a, b));
    }
    return q;
}

static void close_cycle_books(double *c)
{
    if (C(_cycle_work) != 0.0 || C(_cycle_pumping) != 0.0 || C(peak_pressure_pa) > 0.0) {
        C(last_cycle_work_j) = C(_cycle_work);
        C(last_cycle_pumping_work_j) = C(_cycle_pumping);
        C(last_cycle_heat_loss_j) = C(_cycle_heat_loss);
        C(last_cycle_heat_release_j) = C(_cycle_heat_release);
        C(last_peak_pressure_pa) = C(peak_pressure_pa);
        C(last_peak_temperature_k) = C(peak_temperature_k);
        C(last_peak_burned_temperature_k) = C(peak_burned_temperature_k);
        if (C(_motoring_reference_pa) > 0.0) {
            C(last_reference_pressure_pa) = C(_motoring_reference_pa);
            C(last_reference_temperature_k) = C(_motoring_reference_k);
        }
        C(cycles_completed) += 1.0;
    }
    C(_cycle_work) = 0.0;
    C(_cycle_pumping) = 0.0;
    C(_cycle_heat_loss) = 0.0;
    C(_cycle_heat_release) = 0.0;
}

/* air_ref <= 0 means the legacy whole-cycle booking (Python passes None). */
static void begin_cycle(const Ctx *x, double *c, double fuel, double spark, double dur, double air_ref)
{
    close_cycle_books(c);
    C(knock_integral) = 0.0;
    C(knock_detected) = 0.0;
    C(residual_mass_kg) = C(mass);
    C(trapped_air_kg) = 0.0;
    C(trapped_mass_kg) = 0.0;
    C(_fuel_in_cycle) = 0.0;
    fuel = fuel > 0.0 ? fuel : 0.0;
    C(_pool_pending) = 0.0;
    C(energy_scale) = 1.0;
    if (air_ref > 0.0) {
        C(_fuel_per_air) = fuel / air_ref;
        C(_pool_pending) = 1.0;
        C(fuel_energy_remaining) = 0.0;
    } else {
        C(_fuel_per_air) = -1.0;
        C(fuel_mass) = fuel;
        C(fuel_energy_remaining) = fuel * KK(lhv_eff);
    }
    C(_pool_total) = C(fuel_energy_remaining);
    C(_fuel_tab) = fuel > 0.0 ? 1.0 : 0.0;
    C(spark_angle_deg) = spark;
    C(combustion_duration_deg) = fmax_(1.0, dur);
    C(_burn_window) = 2.0 * C(combustion_duration_deg) + 20.0;
    C(_burned_fraction_prev) = 0.0;
    C(_motoring_reference_pa) = 0.0;
    C(_motoring_reference_k) = C(temperature);
    C(_motoring_reference_m3) = KK(clearance_volume);
    C(combustion_occurred) = 0.0;
    C(fired_this_cycle) = 0.0;
    C(misfired) = 0.0;
    C(burned_fraction) = 0.0;
    C(peak_pressure_pa) = 0.0;
    C(peak_temperature_k) = 0.0;
    if (C(_zone_active) != 0.0) merge_zones(c);
    C(no_mole_fraction) = 0.0;
    C(peak_burned_temperature_k) = 0.0;
    C(unburned_temperature_k) = 0.0;
}

#define SECOND_ORDER_FRACTION 0.02
#define THIRD_PASS_FRACTION 0.10
#define C1_GAS_EXCHANGE 6.18
#define C1_COMPRESSION 2.28
#define C2_COMBUSTION 3.24e-3

/* ── one cylinder, one substep ────────────────────────────────────────── */

static void cyl_step(const Ctx *x, double *c, double angle_deg, double dtheta_deg, double dt,
                     double rpm, double p_in, double t_in, double p_ex, double t_ex, double p_cc)
{
    if (dt <= 0.0 || dtheta_deg <= 0.0) return;
    const double R = R_AIR;
    double angle = pymod(angle_deg, 720.0);
    double half = 0.5 * dtheta_deg;

    const double *vt = x->vol;
    double a0 = angle - half;
    double d_edge = C(_edge_a) - a0;
    double v0, v1, xx;
    int i;
    if (-1e-9 < d_edge && d_edge < 1e-9) {
        v0 = C(_edge_v);
    } else {
        xx = pymod(a0, 720.0) * 4.0;
        i = (int)xx;
        v0 = vt[i] + (xx - i) * (vt[i + 1] - vt[i]);
    }
    double a1 = angle + half;
    xx = pymod(a1, 720.0) * 4.0;
    i = (int)xx;
    v1 = vt[i] + (xx - i) * (vt[i + 1] - vt[i]);
    C(_edge_a) = a1;
    C(_edge_v) = v1;
    double volume = 0.5 * (v0 + v1);
    double dv = v1 - v0;

    double mass = C(mass);
    double temperature = C(temperature);
    double pressure = mass > 0.0 ? mass * R * temperature / v0 : 0.0;

    double last = C(_last_angle);
    if (last >= 0.0) {
        double ivc = C(_ivc_deg);
        int hit = (angle >= last) ? (last < ivc && ivc <= angle) : (ivc > last || ivc <= angle);
        if (hit) {
            C(trapped_mass_kg) = mass;
            C(_runner_burned) = 0.0;
            double unburned = mass - C(burned_mass);
            C(trapped_air_kg) = fmax_(0.0, unburned - C(_fuel_in_cycle));
            C(ivc_events) += 1.0;
        }
    }
    C(_last_angle) = angle;

    double a_in, a_ex;
    if (C(_cam_on) != 0.0) {
        a_in = x->ain[(int)(pymod(angle + C(_cam_in), 720.0) * 2.0)];
        a_ex = x->aex[(int)(pymod(angle + C(_cam_ex), 720.0) * 2.0)];
    } else {
        int ia = (int)(angle * 2.0);
        a_in = x->ain[ia];
        a_ex = x->aex[ia];
    }
    int gas_exchange = a_in > 0.0 || a_ex > 0.0;
    if (C(_zone_active) != 0.0 && gas_exchange) {
        merge_zones(c);
        temperature = C(temperature);
        pressure = mass > 0.0 ? mass * R * temperature / v0 : 0.0;
    }

    const double *cvu, *uu, *dub;
    sel_tables(c, &cvu, &uu, &dub);
    double burned0 = C(burned_mass);
    double y0 = mass > 0.0 ? burned0 / mass : 0.0;
    int node = (int)(temperature * GRID_INV);
    if (node > LAST_NODE) node = LAST_NODE;
    double cv0 = cvu[node] + (CVB[node] - cvu[node]) * y0;

    double q_wall = 0.0;
    if (mass > 0.0) {
        int comb = C(combustion_occurred) != 0.0;
        if (comb || gas_exchange || C(_h_age) >= 1.0) {
            double p_ref = C(_motoring_reference_pa);
            double w = (gas_exchange ? C1_GAS_EXCHANGE : C1_COMPRESSION) * KK(stroke) * rpm * (1.0 / 30.0);
            if (comb && !gas_exchange && p_ref > 0.0 && pressure > 0.0) {
                double v_ref = C(_motoring_reference_m3);
                double dp = pressure - p_ref * pow(v_ref / volume, N_COMPRESSION);
                if (dp > 0.0) {
                    w += (C2_COMBUSTION * KK(displaced_volume) * C(_motoring_reference_k)
                          / (p_ref * v_ref)) * dp;
                }
            }
            C(_h_c) = (KK(k_woschni) * pow(pressure * 1.0e-3, 0.8)
                       * pow(temperature > 300.0 ? temperature : 300.0, -0.55) * pow(w, 0.8));
            C(_h_age) = 0.0;
        } else {
            C(_h_age) += 1.0;
        }
        q_wall = C(_h_c) * (KK(area_const) + KK(area_per_volume) * volume) * (temperature - KK(wall_t));
    }
    C(wall_heat_transfer_rate) = q_wall;

    double work_gas = 0.0, pressure_end;
    double q_chem = 0.0;
    int quiet = (a_in == 0.0 && a_ex == 0.0 && mass > 0.0 && C(_zone_active) == 0.0
                 && !((C(_pool_pending) != 0.0 || C(fuel_energy_remaining) > 0.0)
                      && 0.0 < angle - C(spark_angle_deg) && angle - C(spark_angle_deg) < C(_burn_window)));
    if (quiet) {
        C(dm_in) = 0.0;
        C(dm_ex) = 0.0;
        C(_dt) = dt;
        double t1 = q_wall != 0.0 ? temperature - q_wall * dt / (mass * cv0) : temperature;
        if (t1 < MIN_T) t1 = MIN_T;
        work_gas = 0.0;
        if (dv != 0.0) {
            double ln_r = log(v0 / v1);
            int n_1 = (int)(t1 * GRID_INV);
            if (n_1 > LAST_NODE) n_1 = LAST_NODE;
            double cv_s = cvu[n_1] + (CVB[n_1] - cvu[n_1]) * y0;
            if (t1 > 1800.0) {
                int n_g = (int)(0.5 * (t1 + t1 * exp(R * ln_r / cv_s)) * GRID_INV);
                if (n_g > LAST_NODE) n_g = LAST_NODE;
                cv_s = cvu[n_g] + (CVB[n_g] - cvu[n_g]) * y0;
            }
            double t2 = t1 * exp(R * ln_r / cv_s);
            if (t2 < MIN_T) t2 = MIN_T;
            work_gas = mass * cv_s * (t1 - t2);
            t1 = t2;
        }
        C(temperature) = t1;
        pressure_end = mass * R * t1 / v1;
    } else {
        double dm_in = 0.0, dm_ex = 0.0;
        double m_run = mass;
        if (a_in > 0.0 || a_ex > 0.0) {
            double v_eq = v0 / (R * temperature);
            double ln_r = dv != 0.0 ? log(v0 / v1) : 0.0;
            double t_pred = temperature * exp(R * ln_r / cv0);
            if (a_in > 0.0 && p_in > 0.0) {
                double md1 = orifice(a_in, pressure, p_in, temperature, t_in);
                double dm = md1 * dt;
                if (dm > SECOND_ORDER_FRACTION * m_run || -dm > SECOND_ORDER_FRACTION * m_run) {
                    double p_end = (m_run + dm) * R * t_pred / v1;
                    dm = 0.5 * (md1 + orifice(a_in, p_end, p_in, t_pred, t_in)) * dt;
                    if (dm > THIRD_PASS_FRACTION * m_run || -dm > THIRD_PASS_FRACTION * m_run) {
                        p_end = (m_run + dm) * R * t_pred / v1;
                        dm = 0.5 * (md1 + orifice(a_in, p_end, p_in, t_pred, t_in)) * dt;
                    }
                }
                double m_eq = p_in * v_eq;
                if (dm > 0.0) {
                    double lim = m_eq - m_run;
                    dm = dm < lim ? dm : (lim > 0.0 ? lim : 0.0);
                } else {
                    double lim = m_run - m_eq;
                    dm = -dm < lim ? dm : (lim > 0.0 ? -lim : 0.0);
                }
                dm_in = dm;
                m_run += dm;
            }
            if (a_ex > 0.0 && p_ex > 0.0) {
                double md1 = orifice(a_ex, pressure, p_ex, temperature, t_ex);
                double dm = md1 * dt;
                if (dm > SECOND_ORDER_FRACTION * m_run || -dm > SECOND_ORDER_FRACTION * m_run) {
                    double p_end = (m_run + dm) * R * t_pred / v1;
                    dm = 0.5 * (md1 + orifice(a_ex, p_end, p_ex, t_pred, t_ex)) * dt;
                    if (dm > THIRD_PASS_FRACTION * m_run || -dm > THIRD_PASS_FRACTION * m_run) {
                        p_end = (m_run + dm) * R * t_pred / v1;
                        dm = 0.5 * (md1 + orifice(a_ex, p_end, p_ex, t_pred, t_ex)) * dt;
                    }
                }
                double m_eq = p_ex * v_eq;
                if (dm > 0.0) {
                    double lim = m_eq - m_run;
                    dm = dm < lim ? dm : (lim > 0.0 ? lim : 0.0);
                } else {
                    double lim = m_run - m_eq;
                    dm = -dm < lim ? dm : (lim > 0.0 ? -lim : 0.0);
                }
                dm_ex = dm;
            }
        }
        C(dm_in) = dm_in;
        C(dm_ex) = dm_ex;
        C(_dt) = dt;

        double d_burned_mass = 0.0;
        double dx_step = 0.0;
        if (C(_pool_pending) != 0.0 || C(fuel_energy_remaining) > 0.0) {
            double since = angle - C(spark_angle_deg);
            if (0.0 < since && since < C(_burn_window)) {
                if (C(_pool_pending) != 0.0) {
                    double q = burn_quality(x, c, mass, burned0);
                    C(burn_quality) = q;
                    if (q < KK(misfire_q)) {
                        q = 0.0;
                        C(misfire_count) += 1.0;
                        C(misfired) = 1.0;
                    }
                    C(_pool_total) = C(_fuel_in_cycle) * KK(lhv_eff) * C(energy_scale) * q;
                    C(fuel_energy_remaining) = C(_pool_total);
                    C(_pool_pending) = 0.0;
                }
                double dur = C(combustion_duration_deg);
                double xb;
                if (since >= dur) xb = 1.0;
                else xb = 1.0 - exp(-KK(wiebe_a) * pow(since / dur, KK(wiebe_m1)));
                double dx = xb - C(_burned_fraction_prev);
                if (dx > 0.0) {
                    if (C(combustion_occurred) == 0.0) {
                        C(combustion_occurred) = 1.0;
                        C(fired_this_cycle) = 1.0;
                        C(_motoring_reference_pa) = pressure;
                        C(_motoring_reference_k) = temperature;
                        C(_motoring_reference_m3) = v0;
                        C(_unburned_at_spark) = mass - burned0;
                    }
                    q_chem = dx * C(_pool_total);
                    if (q_chem > C(fuel_energy_remaining)) q_chem = C(fuel_energy_remaining);
                    C(fuel_energy_remaining) -= q_chem;
                    d_burned_mass = dx * C(_unburned_at_spark);
                    dx_step = dx;
                    C(_burned_fraction_prev) = xb;
                    C(burned_fraction) = xb;
                    C(heat_released_total) += q_chem;
                    C(_cycle_heat_release) += q_chem;
                }
            }
        }

        if (KK(two_zone) != 0.0 && dm_in == 0.0 && dm_ex == 0.0
            && (C(_zone_active) != 0.0 || q_chem > 0.0)) {
            two_zone_step(x, c, dt, v0, v1, q_chem, dx_step, q_wall, temperature, pressure, mass,
                          KK(wall_t), &work_gas, &pressure_end);
        } else {
            double dm_fuel = 0.0, dm_ret = 0.0;
            if (dm_in < 0.0) {
                C(_runner_burned) += -dm_in * y0;
            } else if (dm_in > 0.0 && C(_runner_burned) > 0.0) {
                dm_ret = dm_in < C(_runner_burned) ? dm_in : C(_runner_burned);
                C(_runner_burned) -= dm_ret;
            }
            double fpa = C(_fuel_per_air);
            int has_fpa = fpa >= 0.0;
            if (has_fpa && dm_in > 0.0) {
                dm_fuel = fpa * (dm_in - dm_ret);
                C(_fuel_in_cycle) += dm_fuel;
            }
            double dm_gas = dm_in + dm_ex;
            double mass_new = mass + dm_gas + dm_fuel;
            if (mass_new < 0.0) mass_new = 0.0;
            work_gas = 0.0;
            if (mass_new > 0.0) {
                double bm = burned0;
                double dm_out = (dm_in < 0.0 ? dm_in : 0.0) + (dm_ex < 0.0 ? dm_ex : 0.0);
                if (dm_out < 0.0 && mass > 0.0) {
                    double keep = 1.0 + dm_out / mass;
                    bm = keep > 0.0 ? bm * keep : 0.0;
                }
                if (dm_ex > 0.0) bm += dm_ex;
                bm += dm_ret + d_burned_mass;
                if (bm > mass_new) bm = mass_new;
                double y1 = bm / mass_new;

                double num = q_chem - q_wall * dt;
                double u_old = 0.0;
                if (dm_in > 0.0 || dm_ex > 0.0 || dm_fuel > 0.0) {
                    double u_u = uu[node] + (temperature - node * GRID_STEP) * cvu[node];
                    double u_b = UB[node] + (temperature - node * GRID_STEP) * CVB[node];
                    u_old = u_u + (u_b - u_u) * y0;
                }
                if (dm_in > 0.0 || dm_fuel > 0.0) {
                    if (C(_inlet_t) != t_in) {
                        int n_in = (int)(t_in * GRID_INV);
                        if (n_in > LAST_NODE) n_in = LAST_NODE;
                        C(_inlet_t) = t_in;
                        C(_inlet_h) = uu[n_in] + (t_in - n_in * GRID_STEP) * cvu[n_in] + R * t_in;
                    }
                    num += (dm_in - dm_ret + dm_fuel) * (C(_inlet_h) - u_old);
                    if (dm_ret > 0.0) {
                        double t_r = t_in + 0.35 * (temperature - t_in);
                        int n_r = (int)(t_r * GRID_INV);
                        if (n_r > LAST_NODE) n_r = LAST_NODE;
                        num += dm_ret * (UB[n_r] + (t_r - n_r * GRID_STEP) * CVB[n_r] + R * t_r - u_old);
                    }
                } else if (dm_in < 0.0) {
                    num += dm_in * R * temperature;
                }
                if (dm_ex > 0.0) {
                    int n_e = (int)(t_ex * GRID_INV);
                    if (n_e > LAST_NODE) n_e = LAST_NODE;
                    num += dm_ex * (UB[n_e] + (t_ex - n_e * GRID_STEP) * CVB[n_e] + R * t_ex - u_old);
                } else if (dm_ex < 0.0) {
                    num += dm_ex * R * temperature;
                }
                if (y1 != y0) num -= mass_new * (y1 - y0) * dub[node];

                double denom = mass_new * cv0;
                double t_mid = temperature + 0.5 * num / denom;
                int n_m = (int)((t_mid > 0.0 ? t_mid : 0.0) * GRID_INV);
                if (n_m > LAST_NODE) n_m = LAST_NODE;
                double y_m = 0.5 * (y0 + y1);
                double cv_m = cvu[n_m] + (CVB[n_m] - cvu[n_m]) * y_m;
                double t1 = temperature + num / (mass_new * cv_m);
                if (t1 < MIN_T) t1 = MIN_T;

                if (dv != 0.0) {
                    double ln_r = log(v0 / v1);
                    int n_1 = (int)(t1 * GRID_INV);
                    if (n_1 > LAST_NODE) n_1 = LAST_NODE;
                    double cv_g = cvu[n_1] + (CVB[n_1] - cvu[n_1]) * y1;
                    double t_guess = t1 * exp(R * ln_r / cv_g);
                    int n_g = (int)(0.5 * (t1 + t_guess) * GRID_INV);
                    if (n_g > LAST_NODE) n_g = LAST_NODE;
                    double cv_s = cvu[n_g] + (CVB[n_g] - cvu[n_g]) * y1;
                    double t2 = t1 * exp(R * ln_r / cv_s);
                    if (t2 < MIN_T) t2 = MIN_T;
                    work_gas = mass_new * cv_s * (t1 - t2);
                    t1 = t2;
                }
                C(temperature) = t1;
                C(burned_mass) = bm;
                if (has_fpa) {
                    if (dm_out < 0.0 && mass > 0.0) {
                        double keep = 1.0 + dm_out / mass;
                        if (keep < 0.0) keep = 0.0;
                        C(fuel_mass) *= keep;
                        C(_fuel_in_cycle) *= keep;
                    }
                    C(fuel_mass) += dm_fuel;
                }
                C(mass) = mass_new;
                pressure_end = mass_new * R * t1 / v1;
            } else {
                C(mass) = 0.0;
                C(burned_mass) = 0.0;
                C(temperature) = temperature > MIN_T ? temperature : MIN_T;
                pressure_end = 0.0;
            }
        }
    }

    double work = work_gas - p_cc * dv;
    double dtheta_rad = dtheta_deg * 0.017453292519943295;
    C(torque) = work / dtheta_rad;
    C(work_net) += work;
    C(_cycle_work) += work;
    if (angle < 180.0 || angle > 540.0) {
        C(work_pumping) += work;
        C(_cycle_pumping) += work;
    }
    C(angle_swept) += dtheta_rad;
    if (q_wall > 0.0) {
        double lost = q_wall * dt;
        C(heat_loss_total) += lost;
        C(_cycle_heat_loss) += lost;
    }

    C(pressure) = pressure_end;
    if (pressure_end > C(peak_pressure_pa)) C(peak_pressure_pa) = pressure_end;
    if (C(temperature) > C(peak_temperature_k)) C(peak_temperature_k) = C(temperature);
    if (KK(knock_on) != 0.0 && C(fuel_mass) > 0.0) accumulate_knock(x, c, angle, dt);
    if (C(record_trace) != 0.0) x->trace[(int)(angle * (TRACE_BINS / 720.0))] = pressure_end;
}

/* ── exhaust plenum (+ turbine stage) ─────────────────────────────────── */

static void exhaust_update(const Ctx *x, double *B, double mdot, double t_cyl, double dt)
{
    const double cp = GAMMA * CV_AIR_T;
    const double p_amb = KK(p_amb);
    const double t_amb_ex = KK(ex_t_amb);
    const double vol = KK(ex_volume);
    if (dt <= 0.0) return;
    if (KK(has_turbine) != 0.0) {
        double p3 = B[B_ex_p], t3 = B[B_ex_t], p4 = B[B_post_p], t4 = B[B_post_t];
        double m_t = 0.0, m_w = 0.0, w_t = 0.0;
        if (p3 > p4) {
            m_t = vmf(p3, p4, t3, KK(turb_area), 0.8);
            double wg_area = KK(wg_area) * B[B_wg_pos];
            if (wg_area > 0.0) m_w = vmf(p3, p4, t3, wg_area, 0.7);
            w_t = (m_t * cp * t3 * KK(eta_turb) * (1.0 - pow(p4 / p3, (GAMMA - 1.0) / GAMMA)));
        }
        double out = (m_t + m_w) * dt;
        double room = B[B_ex_m] - MIN_PLENUM_MASS;
        if (out > room && room > 0.0) {
            double scale = room / out;
            m_t *= scale; m_w *= scale; w_t *= scale;
        } else if (room <= 0.0) {
            m_t = m_w = w_t = 0.0;
        }
        B[B_turb_flow] = m_t; B[B_wg_flow] = m_w; B[B_turb_power] = w_t;

        double m_tail, h_tail;
        if (p4 >= p_amb) {
            m_tail = vmf(p4, p_amb, t4, KK(tail_area), KK(tail_cd));
            h_tail = cp * t4;
        } else {
            m_tail = -vmf(p_amb, p4, t_amb_ex, KK(tail_area), KK(tail_cd));
            h_tail = cp * t_amb_ex;
        }
        double room4 = B[B_post_m] + (m_t + m_w) * dt - MIN_PLENUM_MASS;
        if (m_tail * dt > room4 && room4 > 0.0) m_tail = room4 / dt;
        else if (room4 <= 0.0) m_tail = fmax_(0.0, m_tail);
        B[B_tail_flow] = m_tail;

        double t_in = mdot >= 0.0 ? t_cyl : t3;
        double e3 = B[B_ex_m] * CV_AIR_T * t3 + (mdot * cp * t_in - (m_t + m_w) * cp * t3) * dt;
        double m3 = fmax_(B[B_ex_m] + (mdot - m_t - m_w) * dt, MIN_PLENUM_MASS);
        B[B_ex_m] = m3;
        B[B_ex_t] = fmax_(e3 / (m3 * CV_AIR_T), 50.0);
        B[B_ex_p] = R_AIR * B[B_ex_t] * m3 / vol;

        double pvol = KK(post_volume);
        double e4 = B[B_post_m] * CV_AIR_T * t4 + (((m_t + m_w) * cp * t3 - w_t) - m_tail * h_tail) * dt;
        double m4 = fmax_(B[B_post_m] + (m_t + m_w - m_tail) * dt, MIN_PLENUM_MASS);
        B[B_post_m] = m4;
        B[B_post_t] = fmax_(e4 / (m4 * CV_AIR_T), 50.0);
        B[B_post_p] = R_AIR * B[B_post_t] * m4 / pvol;
        return;
    }
    double p = B[B_ex_p], T = B[B_ex_t], m = B[B_ex_m];
    double mdot_tail, tail_h;
    if (p >= p_amb) {
        mdot_tail = vmf(p, p_amb, T, KK(tail_area), KK(tail_cd));
        tail_h = GAMMA * CV_AIR_T * T;
    } else {
        mdot_tail = -vmf(p_amb, p, t_amb_ex, KK(tail_area), KK(tail_cd));
        tail_h = GAMMA * CV_AIR_T * t_amb_ex;
    }
    B[B_tail_flow] = mdot_tail;
    double available = m - mdot_tail * dt;
    if (available < MIN_PLENUM_MASS) {
        mdot_tail = fmax_(0.0, (m - MIN_PLENUM_MASS) / dt);
        available = m - mdot_tail * dt;
    }
    double t_in = mdot >= 0.0 ? t_cyl : T;
    double energy = m * CV_AIR_T * T;
    energy += (mdot * GAMMA * CV_AIR_T * t_in - mdot_tail * tail_h) * dt;
    m = fmax_(available + mdot * dt, MIN_PLENUM_MASS);
    B[B_ex_m] = m;
    B[B_ex_t] = fmax_(energy / (m * CV_AIR_T), 50.0);
    B[B_ex_p] = R_AIR * B[B_ex_t] * m / vol;
}

/* ── compressor map (turbo_system.Turbocharger) ───────────────────────── */

static double comp_flow(const Ctx *x, double omega, double pr)
{
    double xs = omega / KK(design_omega);
    if (xs <= 0.0) return 0.0;
    double head = (KK(design_ratio) - 1.0) * xs * xs * 1.15;
    double r = (pr - 1.0) / head;
    if (r >= 1.0) return 0.0;
    if (r < 0.0) r = 0.0;
    return KK(design_flow) * 1.25 * xs * sqrt(1.0 - r * r);
}
static double comp_power(const Ctx *x, double mf, double pr)
{
    if (mf <= 0.0 || pr <= 1.0) return 0.0;
    return mf * CP_AIR_T * KK(t_ambient) * (pow(pr, (GAMMA - 1.0) / GAMMA) - 1.0) / KK(eta_c);
}

/* ── the substep loop of CylinderBank.advance ─────────────────────────── */

void es_advance(const double *K, const double *vol, const double *ain, const double *aex,
                double *cs, int ncs, double *trace, double *B, double *loc, const double *begin_k,
                const double *escale, int n_es)
{
    Ctx ctx = { K, vol, ain, aex, trace };
    const Ctx *x = &ctx;
    int n_cyl = (int)KK(n_cyl);
    int n_sub = (int)B[B_n_sub];
    double sub_deg = B[B_sub_deg], sub_dt = B[B_sub_dt];
    double inv_sub_dt = 1.0 / sub_dt;
    double rpm = B[B_rpm];
    double p_man = B[B_p_man], t_man = B[B_t_man];
    double k_man = B[B_k_man], thr_cd = B[B_thr_cd], p_up = B[B_p_upstream];
    double map_f = B[B_map_f], alpha = B[B_alpha];
    double fuel = B[B_fuel_per_cycle], air_ref = B[B_air_ref];
    double spark_local = B[B_spark_local], burn_deg = B[B_burn_deg];
    int hold = B[B_hold_manifold] != 0.0;
    int use_turbo = B[B_use_turbo] != 0.0;
    double p_amb = KK(p_amb);
    double p_b = B[B_p_b], t_b = B[B_t_b], k_boost = B[B_k_boost];
    double by_cd = B[B_by_cd], bov_cd = B[B_bov_cd], t_amb = B[B_t_amb];
    double omega = B[B_omega];
    double e_turbine = 0.0, e_comp = 0.0;
    double p_min = p_man, p_max = p_man;
    double flow_in_sum = 0.0, thr_sum = 0.0;

    for (int k = 0; k < n_sub; k++) {
        double p_exh = B[B_ex_p], t_exh = B[B_ex_t];
        double d_in = 0.0, d_ex = 0.0, hot_dm = 0.0, hot_dm_t = 0.0;
        for (int i = 0; i < n_cyl; i++) {
            double *c = cs + (size_t)i * ncs;
            if ((int)begin_k[i] == k) {
                begin_cycle(x, c, fuel, spark_local, burn_deg, fuel > 0.0 ? air_ref : -1.0);
                if (n_es > 0) C(energy_scale) = escale[i % n_es];
            }
            double a = loc[i];
            cyl_step(x, c, a, sub_deg, sub_dt, rpm, p_man, t_man, p_exh, t_exh, p_amb);
            a += sub_deg;
            loc[i] = a >= 720.0 ? a - 720.0 : a;
            d_in += C(dm_in);
            double e = C(dm_ex);
            d_ex += e;
            if (e < 0.0) {
                hot_dm -= e;
                hot_dm_t -= e * C(temperature);
            }
        }
        exhaust_update(x, B, -d_ex * inv_sub_dt, hot_dm > 0.0 ? hot_dm_t / hot_dm : B[B_ex_t], sub_dt);

        double m_th = 0.0;
        if (use_turbo) {
            if (thr_cd > 0.0 && p_b > p_man) m_th = orifice(thr_cd, p_man, p_b, t_man, t_b);
            double pr = p_b / p_amb;
            double m_c = comp_flow(x, omega, pr);
            double m_in = m_c + (p_b < p_amb ? orifice(by_cd, p_b, p_amb, t_b, t_amb) : 0.0);
            if (bov_cd > 0.0 && p_b > p_amb) m_in += orifice(bov_cd, p_b, p_amb, t_b, t_amb);
            p_b += k_boost * (m_in - m_th) * sub_dt;
            if (p_b < 0.3 * p_amb) p_b = 0.3 * p_amb;
            e_comp += comp_power(x, m_c, pr) * sub_dt;
            e_turbine += B[B_turb_power] * sub_dt;
        } else if (thr_cd > 0.0 && p_up > p_man) {
            m_th = orifice(thr_cd, p_man, p_up, t_man, t_man);
        }
        if (!hold) {
            p_man += k_man * (m_th * sub_dt - d_in);
            if (p_man < 1000.0) p_man = 1000.0;
            else if (p_man > 300000.0) p_man = 300000.0;
        }
        if (p_man < p_min) p_min = p_man;
        else if (p_man > p_max) p_max = p_man;
        map_f += alpha * (p_man - map_f);
        flow_in_sum += d_in;
        thr_sum += m_th;
    }
    B[B_p_man] = p_man; B[B_p_min] = p_min; B[B_p_max] = p_max; B[B_map_f] = map_f;
    B[B_flow_in_sum] = flow_in_sum; B[B_thr_sum] = thr_sum;
    B[B_p_b] = p_b; B[B_e_turbine] = e_turbine; B[B_e_comp] = e_comp;
}
