#include "physics/ResolvedEngine.h"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "core/Units.h"
#include "physics/enginecore/enginecore.h"
#include "physics/enginecore/fields.h"

//===========================================================================
//  A TRANSCRIPTION, NOT A DESIGN. Each function below names the EngineSim
//  function it mirrors, and keeps that function's statement order and
//  arithmetic (down to the association of a product) so the two agree to
//  rounding. Where the Python reads oddly it is copied as it stands rather
//  than tidied: tools/engine_parity compares this file against a trace
//  recorded from EngineSim, and a "harmless" rearrangement shows up there.
//  Change the physics in EngineSim first, then here - see README, "Keeping
//  the port in step with EngineSim".
//===========================================================================

namespace vsim {
namespace {

// engine_sim/thermodynamics.py
constexpr double kRAir = 287.058;
constexpr double kGamma = 1.4;
constexpr double kKappa = (kGamma - 1.0) / kGamma;

// engine_sim/cylinder.py
constexpr double kCycleDeg = 720.0;
constexpr int    kTraceBins = 180;
constexpr double kMinTemperatureK = 50.0;
constexpr int    kVolumeN = 720 * 4 + 2;         // 0.25 deg grid
constexpr int    kValveN = 720 * 2;              // 0.5 deg grid
constexpr double kLiftDeadbandM = 1.0e-4;        // engine_sim/valve_flow.py

// engine_sim/cylinder_bank.py
constexpr double kRlsForget = 0.985;
constexpr double kRlsP0 = 50.0;
constexpr double kRlsPMax = 400.0;
constexpr double kRlsMove = 0.03;
constexpr int    kSummaryEvery = 8;
constexpr double kSlowMapTau = 0.15;
constexpr double kSteadyMapBand = 0.02;

// engine_sim/turbo_system.py
constexpr double kWgKp = 6.0;
constexpr double kWgKi = 3.0;

// engine_sim/exhaust_system.py - the tail pipe breathes 293 K air back in,
// whatever the engine's own intake temperature is.
constexpr double kExhaustAmbientK = 293.0;

const double kGasTables[] = {
#include "physics/enginecore/gas_tables.inc"
};
static_assert(sizeof(kGasTables) / sizeof(double) == 8 * 251, "gas_tables.inc is not 8 x 251");

void ensureTables() {
    static std::once_flag once;
    std::call_once(once, [] { es_init_tables(kGasTables); });
}

/// Python's float %: the result takes the sign of the divisor.
double pymod(double a, double b) {
    double r = std::fmod(a, b);
    if (r != 0.0) {
        if ((b < 0.0) != (r < 0.0)) r += b;
    } else {
        r = std::copysign(0.0, b);
    }
    return r;
}

/// Python's built-in sum() over floats, as CPython 3.12 and later compute it:
/// Neumaier-compensated, with the compensation added once at the end.
/// EngineSim sums per-cylinder figures and the air-model dot products with
/// it, and a plain left-to-right sum differs in the last bit - which is
/// enough to tip a gas-table node lookup a few cycles later, after which the
/// two runs drift apart for good. (On a Python older than 3.12 EngineSim
/// itself gives the plain sum, so its traces match this port only on 3.12+.)
class PySum {
public:
    void add(double x) {
        if (m_first) { m_sum = 0.0 + x; m_first = false; return; }
        const double t = m_sum + x;
        if (std::abs(m_sum) >= std::abs(x)) m_c += (m_sum - t) + x;
        else                                m_c += (x - t) + m_sum;
        m_sum = t;
    }
    double value() const {
        if (m_first) return 0.0;
        return (m_c != 0.0 && std::isfinite(m_c)) ? m_sum + m_c : m_sum;
    }
private:
    double m_sum = 0.0, m_c = 0.0;
    bool m_first = true;
};

double pysum3(double a, double b, double c) {
    PySum s;
    s.add(a); s.add(b); s.add(c);
    return s.value();
}

double smoothstep(double lo, double hi, double x) {
    if (x <= lo) return 0.0;
    if (x >= hi) return 1.0;
    const double t = (x - lo) / (hi - lo);
    return t * t * (3.0 - 2.0 * t);
}

/// engine_sim/spark_model.py::combustion_duration
double combustionDuration(double rpm, double loadFraction, double phi, double residualFraction) {
    const double speedFactor = std::pow(std::max(rpm, 300.0) / 3000.0, 0.5);
    const double loadFactor = 0.5 + 0.5 * loadFraction;
    const double dilutionFactor = 1.0 + 3.0 * residualFraction;
    const double phiFactor = 1.0 + 2.0 * ((phi - 1.0) * (phi - 1.0));
    const double d = 60.0 * speedFactor * dilutionFactor * phiFactor / std::max(loadFactor, 0.1);
    return std::max(20.0, std::min(120.0, d));
}

/// engine_sim/spark_model.py::mbt_timing
double mbtTiming(double combustionDurationDeg, double rpm) {
    double spark = 0.5 * combustionDurationDeg - 8.0;
    spark = std::max(5.0, std::min(50.0, spark));
    spark *= 1.0 + 0.3 * std::max(0.0, (rpm - 3000.0) / 4000.0);
    return std::max(5.0, std::min(50.0, spark));
}

/// engine_sim/manifold_model.py::throttle_effective_area - a butterfly plate.
double throttleEffectiveArea(double position, double maxArea) {
    const double pos = std::max(0.0, std::min(1.0, position));
    return maxArea * (1.0 - std::cos(pos * kPi / 2.0));
}

/// engine_sim/valve_profile.py::ValveProfile.lift
double valveLift(double angleDeg, double openDeg, double durationDeg,
                 const ValveTrainParameters& v) {
    if (durationDeg <= 0.0 || v.maxLiftM <= 0.0) return 0.0;
    const double x = pymod(angleDeg - openDeg, 720.0);
    if (x >= durationDeg) return 0.0;
    const double s = x / durationDeg;
    const double closeFrac = 1.0 - v.rampOpenFrac;
    if (s < v.rampOpenFrac) {
        return v.maxLiftM * 0.5 * (1.0 - std::cos(kPi * s / v.rampOpenFrac));
    }
    return v.maxLiftM * 0.5 * (1.0 + std::cos(kPi * (s - v.rampOpenFrac) / closeFrac));
}

} // namespace

//---------------------------------------------------------------------------
//  Construction
//---------------------------------------------------------------------------

ResolvedEngine::ResolvedEngine(const EngineParameters& params) : m_p(params) {
    m_p.validate();
    ensureTables();
    m_n = m_p.numCylinders;
    m_ncs = ES_NCS;

    const ResolvedEngineParameters& r = m_p.resolved;
    const std::size_t n = static_cast<std::size_t>(m_n);
    m_cs.assign(n * m_ncs, 0.0);
    m_trace.assign(kTraceBins, 0.0);
    m_loc.assign(n, 0.0);
    m_beginK.assign(n, -1.0);
    m_K.assign(ES_NK, 0.0);
    m_B.assign(ES_NB, 0.0);
    m_ivcSeen.assign(n, 0.0);
    m_w0.assign(n, 0.0);
    m_p0.assign(n, 0.0);
    m_h0.assign(n, 0.0);

    // EngineConfig.fire_angles_deg
    m_fire.assign(n, 0.0);
    double a = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        m_fire[static_cast<std::size_t>(r.firingOrder[k] - 1)] = pymod(a, 720.0);
        a += r.firingIntervalsDeg.empty() ? 720.0 / m_n : r.firingIntervalsDeg[k];
    }

    // NativeCylinderBank.__init__
    m_stepBudget = std::max(2 * m_n, r.stepsPerMs);
    m_resDeg = r.crankAngleResolutionDeg;

    // Turbocharger.__init__
    m_turbo = r.useTurbo;
    if (m_turbo) {
        const ResolvedTurboParameters& t = r.turbo;
        const int count = std::max(1, t.count);
        m_tInertia = t.inertiaKgM2 * count;
        m_tDesignOmega = t.designSpeedRps * 2.0 * kPi;
        m_tDesignFlow = t.designFlowKgS * count;
        m_tDesignRatio = t.designPressureRatio;
        m_tEtaC = t.compressorEfficiency;
        m_tEtaM = t.mechanicalEfficiency;
        m_tVolume = t.boostVolumeM3;
        m_tIntercooler = t.intercoolerEffectiveness;
        m_tTargetBoost = t.targetBoostPa;
        m_tGain = t.wastegateGain;
        m_tBypassArea = 3.0e-3 * count;      // check-valve path round an unspun compressor
        m_tBovArea = 6.0e-4 * count;         // blow-off valve
    }

    buildGeometry();

    // NativeCylinderBank._constants
    m_exVolume = std::max(r.exhaustPlenumVolumeM3, 1.0e-4);
    m_postVolume = std::max(r.exhaustPostVolumeM3, 1.0e-4);
    double* K = m_K.data();
    K[K_n_cyl] = m_n;
    K[K_stroke] = m_p.stroke;
    K[K_k_woschni] = 3.26 * std::pow(m_p.bore, -0.2);
    K[K_area_const] = kPi * (m_p.bore * m_p.bore) / 2.0;      // chamber area = pi B^2/2 + 4 V/B
    K[K_area_per_volume] = 4.0 / m_p.bore;
    K[K_displaced_volume] = m_displacedVolume;
    K[K_wall_t] = r.wallTemperatureK;
    K[K_stoich] = m_p.stoichAfr;
    K[K_lhv_eff] = m_p.lowerHeatingValue * m_p.combustionEfficiency;
    K[K_wiebe_a] = r.wiebeA;
    K[K_wiebe_m1] = r.wiebeM + 1.0;
    K[K_knock_on] = r.useKnockModel ? 1.0 : 0.0;
    K[K_misfire_q] = r.burnMisfireQuality;
    K[K_two_zone] = r.useTwoZone ? 1.0 : 0.0;
    K[K_clearance_volume] = m_clearanceVolume;
    K[K_res_lo] = r.burnResidualLo;
    K[K_res_hi] = r.burnResidualHi;
    K[K_lean_lo] = r.burnLeanLo;
    K[K_lean_hi] = r.burnLeanHi;
    K[K_rich_lo] = r.burnRichLo;
    K[K_rich_hi] = r.burnRichHi;
    K[K_octane] = r.fuelOctaneRon;
    K[K_p_amb] = m_p.ambientPressurePa;
    K[K_ex_t_amb] = kExhaustAmbientK;
    K[K_ex_volume] = m_exVolume;
    K[K_tail_area] = kPi / 4.0 * (r.exhaustTailDiaM * r.exhaustTailDiaM);
    K[K_tail_cd] = r.exhaustTailDischargeCoeff;
    K[K_has_turbine] = (m_turbo && r.turbo.turbineAreaM2 > 0.0) ? 1.0 : 0.0;
    K[K_turb_area] = m_turbo ? r.turbo.turbineAreaM2 : 0.0;
    K[K_wg_area] = m_turbo ? r.turbo.wastegateAreaM2 : 0.0;
    K[K_post_volume] = m_postVolume;
    K[K_eta_turb] = r.turbo.turbineEfficiency;
    K[K_design_omega] = m_turbo ? m_tDesignOmega : 1.0;
    K[K_design_ratio] = m_turbo ? m_tDesignRatio : 1.0;
    K[K_design_flow] = m_turbo ? m_tDesignFlow : 0.0;
    K[K_eta_c] = m_turbo ? m_tEtaC : 1.0;
    K[K_t_ambient] = m_turbo ? m_p.intakeTemperatureK : 293.0;

    reset();
}

//---------------------------------------------------------------------------
//  engine_sim/cylinder.py
//---------------------------------------------------------------------------

/// _geometry_tables: volume on a 0.25 deg grid, valve flow area (x Cd x valve
/// count) on a 0.5 deg grid.
void ResolvedEngine::buildGeometry() {
    const ResolvedEngineParameters& r = m_p.resolved;

    m_displacedVolume = (kPi / 4.0) * (m_p.bore * m_p.bore) * m_p.stroke;
    m_clearanceVolume = (m_p.compressionRatio > 1.0)
        ? m_displacedVolume / (m_p.compressionRatio - 1.0) : m_displacedVolume;
    m_pistonArea = kPi / 4.0 * (m_p.bore * m_p.bore);
    m_crankR = m_p.stroke / 2.0;
    m_rodRatioSq = (m_crankR / m_p.rodLength) * (m_crankR / m_p.rodLength);

    // EngineConfig.intake_lift_window / exhaust_lift_window. Cycle frame:
    // 0 = TDC at the start of the intake stroke, 360 = firing TDC.
    m_intakeOpenDeg = pymod(720.0 - r.intakeOpenBtdcDeg, 720.0);
    m_intakeDurationDeg = 180.0 + r.intakeOpenBtdcDeg + r.intakeCloseAbdcDeg;
    m_exhaustOpenDeg = 540.0 - r.exhaustOpenBbdcDeg;
    m_exhaustDurationDeg = 180.0 + r.exhaustOpenBbdcDeg + r.exhaustCloseAtdcDeg;
    m_ivcBaseDeg = pymod(m_intakeOpenDeg + m_intakeDurationDeg, kCycleDeg);

    const double l = m_p.rodLength;
    const double rr = m_crankR;
    m_vol.resize(kVolumeN);
    for (int k = 0; k < kVolumeN; ++k) {
        const double th = (k / 4.0) * (kPi / 180.0);
        const double s = std::sin(th);
        const double x = (l + rr) - rr * std::cos(th) - l * std::sqrt(1.0 - m_rodRatioSq * s * s);
        m_vol[static_cast<std::size_t>(k)] = m_clearanceVolume + m_pistonArea * x;
    }

    auto valveTable = [](std::vector<double>& out, double openDeg, double durationDeg,
                         const ValveTrainParameters& v, int count) {
        // One spare element: the core indexes with (angle mod 720) * 2, and a
        // mod that rounds up to exactly 720 must not read past the end.
        out.assign(kValveN + 1, 0.0);
        const double aPort = kPi / 4.0 * (v.portDiaM * v.portDiaM);
        const double cosB = std::cos(v.seatAngleDeg * (kPi / 180.0));
        for (int k = 0; k < kValveN; ++k) {
            const double lf = valveLift(k / 2.0, openDeg, durationDeg, v);
            if (lf > kLiftDeadbandM) {
                out[static_cast<std::size_t>(k)] =
                    v.dischargeCoeff * count * std::min(kPi * v.headDiaM * lf * cosB, aPort);
            }
        }
        out[kValveN] = out[0];
    };
    valveTable(m_areaIn, m_intakeOpenDeg, m_intakeDurationDeg, r.intake, r.intakeValveCount());
    valveTable(m_areaEx, m_exhaustOpenDeg, m_exhaustDurationDeg, r.exhaust, r.exhaustValveCount());
}

/// Cylinder.volume - exact, untabulated.
double ResolvedEngine::exactVolume(double angleDeg) const {
    const double th = angleDeg * (kPi / 180.0);
    const double s = std::sin(th);
    const double x = (m_p.rodLength + m_crankR) - m_crankR * std::cos(th)
                   - m_p.rodLength * std::sqrt(1.0 - m_rodRatioSq * s * s);
    return m_clearanceVolume + m_pistonArea * x;
}

/// ValveProfile.is_open for the intake - the un-phased cam, as EngineSim uses.
bool ResolvedEngine::intakeIsOpen(double angleDeg) const {
    if (m_intakeDurationDeg <= 0.0 || m_p.resolved.intake.maxLiftM <= 0.0) return false;
    return pymod(angleDeg - m_intakeOpenDeg, 720.0) < m_intakeDurationDeg;
}

/// Cylinder.__init__ + set_cam_phase + initialize_charge + the first
/// begin_cycle(0, spark, duration) of CylinderBank.reset: a cylinder full of
/// ambient air at `localAngleDeg`.
void ResolvedEngine::initCylinder(int index, double localAngleDeg) {
    const ResolvedEngineParameters& r = m_p.resolved;
    double* c = cyl(index);
    std::fill(c, c + m_ncs, 0.0);

    c[F__edge_a] = -1.0e9;
    c[F__h_age] = 99.0;
    c[F__phi_cycle] = 1.0;
    c[F_burn_quality] = 1.0;
    c[F_energy_scale] = 1.0;
    c[F__dt] = 1.0;
    c[F_record_trace] = (index == 0) ? 1.0 : 0.0;
    c[F__fuel_per_air] = -1.0;       // None
    c[F__last_angle] = -1.0;         // None
    c[F__fuel_tab] = 0.0;            // unburned charge = plain air
    c[F__inlet_t] = -1.0;
    c[F__inlet_h] = 0.0;

    c[F__cam_in] = m_camIn;
    c[F__cam_ex] = m_camEx;
    c[F__cam_on] = (m_camIn != 0.0 || m_camEx != 0.0) ? 1.0 : 0.0;
    c[F__ivc_deg] = pymod(m_ivcBaseDeg - m_camIn, kCycleDeg);
    c[F__exhaust_open] = pymod(m_exhaustOpenDeg - m_camEx, kCycleDeg);

    const double tIn = m_p.intakeTemperatureK;
    const double volume = exactVolume(localAngleDeg);
    const double mass = std::max(0.0, m_p.ambientPressurePa * volume / (kRAir * tIn));
    c[F_mass] = mass;
    c[F_burned_mass] = 0.0;
    c[F_temperature] = std::max(kMinTemperatureK, tIn);
    c[F_pressure] = mass * kRAir * c[F_temperature] / std::max(volume, 1.0e-9);

    c[F_residual_mass_kg] = mass;
    c[F_spark_angle_deg] = 360.0 - r.sparkTimingDegBtdc;
    c[F_combustion_duration_deg] = std::max(1.0, r.combustionDurationDeg);
    c[F__burn_window] = 2.0 * c[F_combustion_duration_deg] + 20.0;
    c[F__motoring_reference_k] = c[F_temperature];
    c[F__motoring_reference_m3] = m_clearanceVolume;
}

/// CylinderBank.set_cam_phase / Cylinder.set_cam_phase
void ResolvedEngine::setCamPhase(double intakeAdvanceDeg, double exhaustAdvanceDeg) {
    if (intakeAdvanceDeg == m_camIn && exhaustAdvanceDeg == m_camEx) return;
    m_camIn = intakeAdvanceDeg;
    m_camEx = exhaustAdvanceDeg;
    for (int i = 0; i < m_n; ++i) {
        double* c = cyl(i);
        c[F__cam_in] = m_camIn;
        c[F__cam_ex] = m_camEx;
        c[F__cam_on] = (m_camIn != 0.0 || m_camEx != 0.0) ? 1.0 : 0.0;
        c[F__ivc_deg] = pymod(m_ivcBaseDeg - m_camIn, kCycleDeg);
        c[F__exhaust_open] = pymod(m_exhaustOpenDeg - m_camEx, kCycleDeg);
    }
    m_cycleStartDeg = pymod(m_intakeOpenDeg - m_camIn, kCycleDeg);
}

//---------------------------------------------------------------------------
//  engine_sim/turbo_system.py
//---------------------------------------------------------------------------

void ResolvedEngine::resetTurbo() {
    m_tOmega = 0.0;
    m_boostPressure = m_p.ambientPressurePa;
    m_boostTemperature = m_p.intakeTemperatureK;
    m_wastegatePosition = 0.0;
    m_wgIntegral = 0.0;
}

/// Charge temperature leaving the intercooler: Heywood Eq 6.40 for the
/// compressor's rise, less what the intercooler takes back.
double ResolvedEngine::chargeTemperature(double pressureRatio) const {
    const double tAmb = m_p.intakeTemperatureK;
    if (pressureRatio <= 1.0) return tAmb;
    const double tOut = tAmb * (1.0 + (std::pow(pressureRatio, kKappa) - 1.0) / m_tEtaC);
    return tAmb + (1.0 - m_tIntercooler) * (tOut - tAmb);
}

/// 1/2 I d(w^2) = eta_m E_T - E_C: spool-up takes what the inertia and the
/// surplus turbine power say (Heywood Eq 14.51 in energy form).
void ResolvedEngine::advanceShaft(double turbineEnergyJ, double compressorEnergyJ) {
    const double e = 0.5 * m_tInertia * (m_tOmega * m_tOmega)
                   + m_tEtaM * turbineEnergyJ - compressorEnergyJ;
    m_tOmega = (e > 0.0) ? std::sqrt(2.0 * e / m_tInertia) : 0.0;
}

/// PI law on the boost error. AUTHORED in EngineSim (an integrate-only law
/// hunts with the spool-up lag - see ../EngineSim/UPDATES.md, 0a.2).
void ResolvedEngine::controlWastegate(double dt) {
    const double e = (m_boostPressure - m_tTargetBoost) / m_tTargetBoost;
    const double k = m_tGain;
    const double iMax = 1.0 / (kWgKi * k);
    m_wgIntegral = std::min(iMax, std::max(0.0, m_wgIntegral + e * dt));
    const double pos = k * (kWgKp * e + kWgKi * m_wgIntegral);
    m_wastegatePosition = std::min(1.0, std::max(0.0, pos));
}

//---------------------------------------------------------------------------
//  engine_sim/cylinder_bank.py
//---------------------------------------------------------------------------

/// CylinderBank.__init__ + reset: every cylinder full of ambient air at the
/// given crank angle, both plenums at ambient.
void ResolvedEngine::resetBank(double thetaDeg) {
    const double pAmb = m_p.ambientPressurePa;
    const double tIn = m_p.intakeTemperatureK;

    m_camIn = m_camEx = 0.0;
    m_cycleStartDeg = m_intakeOpenDeg;

    m_exP = pAmb;
    m_exT = tIn;
    m_exM = m_exP * m_exVolume / (kRAir * m_exT);
    m_postP = pAmb;
    m_postT = tIn;
    m_postM = m_postP * m_postVolume / (kRAir * m_postT);
    m_tailFlow = m_turbFlow = m_wgFlow = m_turbPower = 0.0;
    resetTurbo();

    for (int i = 0; i < m_n; ++i) {
        initCylinder(i, pymod(thetaDeg - m_fire[static_cast<std::size_t>(i)] + 360.0, kCycleDeg));
    }
    std::fill(m_trace.begin(), m_trace.end(), 0.0);

    m_mapFiltered = pAmb;
    m_veMeasured = m_p.maxVolumetricEfficiency;
    const double a0 = (m_p.maxVolumetricEfficiency < 1.0) ? m_p.maxVolumetricEfficiency : 0.95;
    m_theta = {a0 * 0.9 / 0.9, 0.0, -0.1};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) m_cov[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = (i == j) ? kRlsP0 : 0.0;
    }
    m_refX = m_refN = -1.0;
    m_mapSlow = m_mapFiltered;

    m_trappedAirKg = 0.0;
    m_indicatedTorque = m_pumpingTorque = m_grossTorque = 0.0;
    m_airMassFlow = m_heatLossW = m_throttleMassFlow = 0.0;
    m_bankManifoldPressure = pAmb;
    m_knockDetected = false;
    m_knockIntegral = 0.0;
    m_knockEvents = 0;
    m_residualFraction = 0.05;
    m_burnQuality = 1.0;
    m_misfireRate = 0.0;
    m_misfireTotal = 0;
    m_cyclesTotal = 0;
    m_cycleWorkJ = m_cyclePumpingWorkJ = 0.0;
    m_peakPressurePa = 0.0;
    m_substepsLast = 0;
    std::fill(m_ivcSeen.begin(), m_ivcSeen.end(), 0.0);
    std::fill(m_w0.begin(), m_w0.end(), 0.0);
    std::fill(m_p0.begin(), m_p0.end(), 0.0);
    std::fill(m_h0.begin(), m_h0.end(), 0.0);
    m_summaryDue = 0;
}

/// CylinderBank.air_estimate - what the ECU believes each cylinder inducts.
/// Speed-density from the learned model: normalised air = (a0 + a1 n) MAP - b.
double ResolvedEngine::airEstimate(double pManifold, double tManifold, double rpm) const {
    if (tManifold <= 0.0 || pManifold <= 0.0) return 0.0;
    double y = (m_theta[0] + m_theta[2] * (rpm / 6000.0)) * (pManifold / 1.0e5) - m_theta[1];
    if (y < 0.02) y = 0.02;
    return y * 1.0e5 / (kRAir * tManifold) * m_displacedVolume;
}

/// CylinderBank._learn_air_model - one recursive-least-squares update from an
/// intake event that has just closed. theta = (a0, b, a1).
void ResolvedEngine::learnAirModel(double pManifold, double tManifold, double rpm,
                                   double trappedAirKg) {
    const double scale = 1.0e5 / (kRAir * tManifold) * m_displacedVolume;
    const double y = trappedAirKg / scale;
    const double x = pManifold / 1.0e5;
    const double n = rpm / 6000.0;
    const double phi[3] = {x, -1.0, x * n};
    // Forget old data only when the operating point has moved (EngineSim
    // UPDATES.md 0.2, defect 5: at a steady point the regressors are collinear).
    double forget = 1.0;
    if (std::abs(x - m_refX) + std::abs(n - m_refN) > kRlsMove) {
        forget = kRlsForget;
        m_refX = x;
        m_refN = n;
    }
    auto& P = m_cov;
    double Pphi[3], gain[3];
    for (std::size_t i = 0; i < 3; ++i) Pphi[i] = pysum3(P[i][0] * phi[0], P[i][1] * phi[1], P[i][2] * phi[2]);
    const double denom = forget + pysum3(phi[0] * Pphi[0], phi[1] * Pphi[1], phi[2] * Pphi[2]);
    for (std::size_t i = 0; i < 3; ++i) gain[i] = Pphi[i] / denom;
    const double err = y - pysum3(m_theta[0] * phi[0], m_theta[1] * phi[1], m_theta[2] * phi[2]);
    for (std::size_t i = 0; i < 3; ++i) m_theta[i] += gain[i] * err;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) P[i][j] = (P[i][j] - gain[i] * Pphi[j]) / forget;
    }
    const double trace = P[0][0] + P[1][1] + P[2][2];
    if (trace > kRlsPMax) {
        const double k = kRlsPMax / trace;
        for (auto& row : P) for (double& v : row) v *= k;
    }
    m_theta[0] = std::min(1.5, std::max(0.3, m_theta[0]));
    m_theta[1] = std::min(0.5, std::max(-0.3, m_theta[1]));
    m_theta[2] = std::min(0.3, std::max(-0.8, m_theta[2]));
}

/// CylinderBank.advance: every cylinder, both plenums and the throttle over
/// one engine step.
void ResolvedEngine::advanceBank(double thetaDeg, double dthetaDeg, double dt, double rpm,
                                 double manifoldPressure, double intakeTemperature,
                                 double throttleArea, double fuelPerCycle, double airRef,
                                 double sparkBtdc, double burnDeg, bool holdManifold) {
    const ResolvedEngineParameters& r = m_p.resolved;
    const int n = m_n;
    if (dt <= 0.0 || dthetaDeg <= 0.0) {
        // A stopped crank produces no torque.
        m_grossTorque = m_pumpingTorque = m_indicatedTorque = 0.0;
        m_airMassFlow = m_heatLossW = m_throttleMassFlow = 0.0;
        m_bankManifoldPressure = manifoldPressure;
        return;
    }

    // Substeps: as fine as the crank-angle resolution, but never more than
    // the cylinder-step budget or the substep cap allow.
    const double res = (m_resDeg > 0.1) ? m_resDeg : 0.1;
    int nSub = static_cast<int>(std::ceil(dthetaDeg / res));
    int cap = static_cast<int>(m_stepBudget * dt * 1000.0 + 0.5) / n;
    if (cap < 1) cap = 1;
    if (nSub > cap) nSub = cap;
    if (nSub > r.maxSubstepsPerUpdate) nSub = r.maxSubstepsPerUpdate;
    if (nSub < 1) nSub = 1;
    const double subDeg = dthetaDeg / nSub;
    const double subDt = dt / nSub;
    const double sparkLocal = 360.0 - sparkBtdc;
    const double tMan = intakeTemperature;
    // MAP filter: one firing interval, so the ECU sees the mean not the pulses.
    double alpha = subDeg / (kCycleDeg / n);
    if (alpha > 1.0) alpha = 1.0;

    // Per cylinder: local angle at the middle of the first substep, and the
    // substep (if any) in which its cycle begins - the intake valve opening.
    const double thrX = kCycleDeg - subDeg - 1e-9;
    for (int i = 0; i < n; ++i) {
        const std::size_t ui = static_cast<std::size_t>(i);
        const double off = m_fire[ui];
        m_loc[ui] = pymod(thetaDeg - off + 360.0 + 0.5 * subDeg, kCycleDeg);
        const double x0 = pymod((thetaDeg - off + 360.0) - m_cycleStartDeg, kCycleDeg);
        if (x0 >= thrX) {
            m_beginK[ui] = 0.0;
        } else {
            const int kk = static_cast<int>(std::ceil((thrX - x0) / subDeg));
            m_beginK[ui] = (kk < nSub) ? kk : -1;
        }
    }
    if (fuelPerCycle > 0.0) {
        // A cylinder that began its cycle without fuel (a fuel cut that has
        // just ended) picks it up as soon as its intake valve is open.
        for (int i = 0; i < n; ++i) {
            const std::size_t ui = static_cast<std::size_t>(i);
            double* c = cyl(i);
            if (c[F__fuel_per_air] < 0.0 && m_beginK[ui] != 0.0 && intakeIsOpen(m_loc[ui])) {
                if (airRef > 0.0) {                       // Cylinder.load_fuel
                    c[F__fuel_per_air] = fuelPerCycle / airRef;
                    c[F__pool_pending] = 1.0;
                    c[F__fuel_tab] = 1.0;
                }
                c[F_spark_angle_deg] = sparkLocal;
                c[F_combustion_duration_deg] = std::max(1.0, burnDeg);
                c[F__burn_window] = 2.0 * c[F_combustion_duration_deg] + 20.0;
            }
        }
    }

    // NativeCylinderBank._substeps: hand the step to the compiled loop.
    double* B = m_B.data();
    B[B_n_sub] = nSub;
    B[B_sub_deg] = subDeg;
    B[B_sub_dt] = subDt;
    B[B_rpm] = rpm;
    B[B_p_man] = manifoldPressure;
    B[B_t_man] = tMan;
    B[B_k_man] = kRAir * tMan / r.manifoldVolumeM3;
    B[B_thr_cd] = throttleArea * r.throttleDischargeCoeff;
    B[B_p_upstream] = m_p.ambientPressurePa;
    B[B_map_f] = m_mapFiltered;
    B[B_alpha] = alpha;
    B[B_fuel_per_cycle] = fuelPerCycle;
    B[B_air_ref] = airRef;
    B[B_spark_local] = sparkLocal;
    B[B_burn_deg] = burnDeg;
    B[B_hold_manifold] = holdManifold ? 1.0 : 0.0;
    if (m_turbo) {
        B[B_use_turbo] = 1.0;
        B[B_p_b] = m_boostPressure;
        B[B_t_b] = m_boostTemperature;
        B[B_k_boost] = kRAir * m_boostTemperature / m_tVolume;
        B[B_by_cd] = m_tBypassArea * 0.7;
        B[B_bov_cd] = (throttleArea < 0.05 * r.throttleMaxAreaM2) ? m_tBovArea * 0.7 : 0.0;
        B[B_t_amb] = m_p.intakeTemperatureK;
        B[B_omega] = m_tOmega;
        B[B_wg_pos] = m_wastegatePosition;
    } else {
        B[B_use_turbo] = 0.0;
    }
    B[B_ex_p] = m_exP;
    B[B_ex_t] = m_exT;
    B[B_ex_m] = m_exM;
    B[B_post_p] = m_postP;
    B[B_post_t] = m_postT;
    B[B_post_m] = m_postM;
    B[B_tail_flow] = m_tailFlow;
    B[B_turb_flow] = m_turbFlow;
    B[B_wg_flow] = m_wgFlow;
    B[B_turb_power] = m_turbPower;

    static const double kNoScale[1] = {1.0};
    es_advance(m_K.data(), m_vol.data(), m_areaIn.data(), m_areaEx.data(), m_cs.data(),
               static_cast<int>(m_ncs), m_trace.data(), B, m_loc.data(), m_beginK.data(),
               kNoScale, 0);

    m_exP = B[B_ex_p];
    m_exT = B[B_ex_t];
    m_exM = B[B_ex_m];
    m_postP = B[B_post_p];
    m_postT = B[B_post_t];
    m_postM = B[B_post_m];
    m_tailFlow = B[B_tail_flow];
    m_turbFlow = B[B_turb_flow];
    m_wgFlow = B[B_wg_flow];
    m_turbPower = B[B_turb_power];
    const double pMan = B[B_p_man];
    const double mapF = B[B_map_f];

    const double pAmb = m_p.ambientPressurePa;
    m_mapFiltered = mapF;
    if (m_turbo) {
        m_boostPressure = B[B_p_b];
        m_boostTemperature = chargeTemperature(m_boostPressure / pAmb);
        advanceShaft(B[B_e_turbine], B[B_e_comp]);
        controlWastegate(dt);
    }

    // Learn the air model from the intake events that closed. On a boosted
    // engine only while the manifold has settled against its own slow
    // average (UPDATES.md 0a.2, defect 3): the trapped air also depends on
    // exhaust back-pressure, which a speed-density model cannot see.
    const double rho = mapF / (kRAir * tMan);
    m_mapSlow += std::min(1.0, dt / kSlowMapTau) * (mapF - m_mapSlow);
    const bool steady = !m_turbo || std::abs(mapF - m_mapSlow) < kSteadyMapBand * mapF;
    for (int i = 0; i < n; ++i) {
        const std::size_t ui = static_cast<std::size_t>(i);
        const double* c = cyl(i);
        if (c[F_ivc_events] != m_ivcSeen[ui]) {
            m_ivcSeen[ui] = c[F_ivc_events];
            if (rho > 0.0 && c[F_trapped_air_kg] > 0.0) {
                m_veMeasured = c[F_trapped_air_kg] / (rho * m_displacedVolume);
                if (steady) learnAirModel(mapF, tMan, rpm, c[F_trapped_air_kg]);
            }
        }
    }

    // Work and heat of this step, as differences of the running totals.
    const double swept = dthetaDeg * 0.017453292519943295;
    double wNet = 0.0, wPump = 0.0, heat = 0.0;
    for (int i = 0; i < n; ++i) {
        const std::size_t ui = static_cast<std::size_t>(i);
        const double* c = cyl(i);
        const double wn = c[F_work_net], wp = c[F_work_pumping], ht = c[F_heat_loss_total];
        wNet += wn - m_w0[ui];
        wPump += wp - m_p0[ui];
        heat += ht - m_h0[ui];
        m_w0[ui] = wn;
        m_p0[ui] = wp;
        m_h0[ui] = ht;
    }
    m_indicatedTorque = wNet / swept;
    m_pumpingTorque = -wPump / swept;
    m_grossTorque = m_indicatedTorque + m_pumpingTorque;
    m_airMassFlow = B[B_flow_in_sum] / dt;
    m_throttleMassFlow = B[B_thr_sum] / nSub;
    m_heatLossW = heat / dt;
    m_bankManifoldPressure = pMan;
    m_substepsLast = nSub;
    m_summaryDue -= 1;
    if (m_summaryDue <= 0) {
        m_summaryDue = kSummaryEvery;
        refreshCycleSummary();
    }
}

/// CylinderBank._refresh_cycle_summary - per-cycle figures change once a
/// cycle, so they are refreshed every few steps rather than every one.
void ResolvedEngine::refreshCycleSummary() {
    const double n = static_cast<double>(m_n);
    PySum work, pumping, peak, trapped, residual, quality;
    double knockMax = 0.0;
    double misfires = 0.0, cycles = 0.0, knockEvents = 0.0;
    int trappedCount = 0;
    bool knock = false;
    for (int i = 0; i < m_n; ++i) {
        const double* c = cyl(i);
        work.add(c[F_last_cycle_work_j]);
        pumping.add(c[F_last_cycle_pumping_work_j]);
        peak.add(c[F_last_peak_pressure_pa]);
        if (c[F_trapped_air_kg] > 0.0) { trapped.add(c[F_trapped_air_kg]); ++trappedCount; }
        knockMax = (i == 0) ? c[F_knock_integral] : std::max(knockMax, c[F_knock_integral]);
        knock = knock || c[F_knock_detected] != 0.0;
        knockEvents += c[F_knock_events];
        residual.add(c[F_residual_at_spark]);
        quality.add(c[F_burn_quality]);
        misfires += c[F_misfire_count];
        cycles += c[F_cycles_completed];
    }
    m_cycleWorkJ = work.value() / n;
    m_cyclePumpingWorkJ = pumping.value() / n;
    m_peakPressurePa = peak.value() / n;
    m_trappedAirKg = trappedCount ? trapped.value() / trappedCount : 0.0;
    m_knockIntegral = knockMax;
    m_knockDetected = knock;
    m_knockEvents = static_cast<long>(knockEvents);
    m_residualFraction = residual.value() / n;
    m_burnQuality = quality.value() / n;
    const long mis = static_cast<long>(misfires);
    const long cyc = static_cast<long>(cycles);
    if (cyc > m_cyclesTotal) {
        m_misfireRate += 0.25 * (static_cast<double>(mis - m_misfireTotal)
                                 / static_cast<double>(cyc - m_cyclesTotal) - m_misfireRate);
        m_misfireTotal = mis;
        m_cyclesTotal = cyc;
    }
}

/// CylinderBank.prime: pre-roll a few fired cycles at fixed manifold
/// pressure, so a started engine holds a realistic charge and residual, a
/// warm exhaust and a learned air model instead of an ambient-air fill.
void ResolvedEngine::primeBank(double thetaDeg, double rpm, double manifoldPressure,
                               double intakeTemperature, double sparkBtdc, double burnDeg,
                               int cycles) {
    m_exT = std::max(m_exT, 650.0);
    m_exM = m_exP * m_exVolume / (kRAir * m_exT);
    const double chunk = 10.0;
    const double dt = chunk / (6.0 * std::max(rpm, 1.0));
    const int steps = static_cast<int>(cycles * kCycleDeg / chunk);
    for (int k = 0; k < steps; ++k) {
        const double air = airEstimate(manifoldPressure, intakeTemperature, rpm);
        advanceBank(thetaDeg + k * chunk, chunk, dt, rpm, manifoldPressure, intakeTemperature,
                    0.0, air / m_p.stoichAfr, air, sparkBtdc, burnDeg, /*holdManifold=*/true);
    }
    m_mapFiltered = manifoldPressure;
    m_bankManifoldPressure = manifoldPressure;
    refreshCycleSummary();
}

//---------------------------------------------------------------------------
//  engine_sim/engine.py
//---------------------------------------------------------------------------

void ResolvedEngine::resetEcu(bool running) {
    m_thetaRad = 0.0;
    m_dfcoActive = false;
    m_limiterCut = false;
    m_iacIntegral = 0.0;
    m_synced = running;
    m_syncRevs = 0.0;
    m_fireTime = 0.0;
    // A warm start is already running: there is no start-up flare for the
    // decel fuel cut to wait out.
    m_dfcoClock = running ? 1.0e9 : 0.0;
    m_starterRecent = 0.0;
    m_wasFiring = false;
    m_vvtIn = m_vvtEx = 0.0;
    m_knockRetardDeg = 0.0;
    m_knockEventsSeen = 0;
    m_manifoldPressure = m_p.ambientPressurePa;
    m_out = Output{};
}

/// Engine.reset
void ResolvedEngine::reset() {
    m_omega = 0.0;
    resetEcu(/*running=*/false);
    resetBank(0.0);
    m_out.manifoldPressurePa = m_out.mapFilteredPa = m_manifoldPressure;
    m_out.boostPressurePa = m_p.ambientPressurePa;
    m_out.exhaustPressurePa = m_exP;
    m_out.exhaustTemperatureK = m_exT;
    m_out.intakeTemperatureK = m_p.intakeTemperatureK;
}

/// Engine.start + _prime_resolved
void ResolvedEngine::start() {
    m_omega = m_p.idleRpm * 2.0 * kPi / 60.0;
    resetEcu(/*running=*/true);
    resetBank(0.0);

    const double pIdle = 0.25 * m_p.ambientPressurePa;
    const double rpm = m_omega * 30.0 / kPi;
    const double burn = combustionDuration(rpm, 0.35, 1.0, 0.05);
    primeBank(0.0, rpm, pIdle, m_p.intakeTemperatureK, mbtTiming(burn, rpm), burn, 3);
    m_manifoldPressure = pIdle;

    m_out.rpm = rpm;
    m_out.omega = m_omega;
    m_out.firing = true;
    m_out.manifoldPressurePa = m_out.mapFilteredPa = pIdle;
    m_out.boostPressurePa = m_boostPressure;
    m_out.exhaustPressurePa = m_exP;
    m_out.exhaustTemperatureK = m_exT;
    m_out.intakeTemperatureK = m_p.intakeTemperatureK;
}

/// Engine._idle_target_rads: the target starts high after the first fire and
/// decays to idle - a fast-idle ramp, so the controller sees a small error
/// all the way down from the start-up flare.
double ResolvedEngine::idleTargetOmega() const {
    const ResolvedEngineParameters& r = m_p.resolved;
    const double idle = m_p.idleRpm * 2.0 * kPi / 60.0;
    if (r.fastIdleExtraRatio <= 0.0) return idle;
    return idle * (1.0 + r.fastIdleExtraRatio * std::exp(-m_fireTime / r.fastIdleDecayS));
}

/// Engine._idle_air_control: the idle-air bypass valve, as a fraction of the
/// WOT throttle area. PI on normalised speed error while idling; outside idle
/// the integral is frozen and the valve holds its last position (a dashpot).
/// AUTHORED ECU logic - EngineSim KB_GAPS.md G-E03.
double ResolvedEngine::idleAirControl(double throttle, double omega, bool fireOk, double dt) {
    const ResolvedEngineParameters& r = m_p.resolved;
    const double idle = m_p.idleRpm * 2.0 * kPi / 60.0;
    const double scale = r.iacResolvedGainScale;
    const double floor = std::max(r.iacMin, 0.55 * r.iacFeedforward);
    const double kp = r.iacKp * scale * r.iacResolvedKpFraction;
    const double ki = r.iacKi * scale;
    const double err = (idle > 0.0) ? (idleTargetOmega() - omega) / idle : 0.0;
    double base = r.iacFeedforward + m_iacIntegral;
    if (!fireOk) {
        // Cranking: hold the idle position rather than opening wide, or the
        // manifold is near atmospheric at first fire and the engine flares.
        return std::max(floor, std::min(r.iacMax, base));
    }
    const double iLo = -0.4 * r.iacFeedforward;
    const double iHi = 0.8 * r.iacFeedforward;
    if (throttle < 0.01 && !m_dfcoActive) {
        double u = base + kp * err;
        const double integral = m_iacIntegral + ki * err * dt;
        if ((floor <= u && u <= r.iacMax) || ((u > r.iacMax) == (err < 0.0))) {
            m_iacIntegral = std::max(iLo, std::min(iHi, integral));   // conditional anti-windup
        }
        u = r.iacFeedforward + m_iacIntegral + kp * err;
        return std::max(floor, std::min(r.iacMax, u));
    }
    if (m_dfcoActive) {
        // During a fuel cut the valve is held at least at its idle position,
        // so the manifold is not pumped down to a vacuum the engine cannot
        // recover from when fuel resumes.
        base = std::max(base, r.iacFeedforward);
    }
    return std::max(floor, std::min(r.iacMax, base));
}

/// Engine.vvt_schedule + _update_vvt: move the cams toward the ECU's target
/// at the actuator rate. `load` is MAP / ambient.
void ResolvedEngine::updateVvt(double rpm, double load, double dt) {
    const ResolvedEngineParameters& r = m_p.resolved;
    const bool canMove = r.vvtIntakeMaxDeg != r.vvtIntakeMinDeg
                      || r.vvtExhaustMaxDeg != r.vvtExhaustMinDeg;
    if (!canMove) return;
    double targetIn = r.vvtIntakeMinDeg, targetEx = r.vvtExhaustMinDeg;
    if (r.vvtAuto) {
        const double rr = (m_p.redlineRpm > 0.0) ? rpm / m_p.redlineRpm : 0.0;
        const double loadF = smoothstep(0.30, 0.90, load);
        const double fIn = loadF * smoothstep(0.12, 0.32, rr) * (1.0 - 0.7 * smoothstep(0.55, 0.95, rr));
        const double fEx = loadF * smoothstep(0.25, 0.80, rr);
        targetIn = r.vvtIntakeMinDeg + (r.vvtIntakeMaxDeg - r.vvtIntakeMinDeg) * fIn;
        targetEx = r.vvtExhaustMinDeg + (r.vvtExhaustMaxDeg - r.vvtExhaustMinDeg) * fEx;
    }
    const double step = r.vvtRateDegPerS * dt;
    m_vvtIn += std::max(-step, std::min(step, targetIn - m_vvtIn));
    m_vvtEx += std::max(-step, std::min(step, targetEx - m_vvtEx));
    setCamPhase(m_vvtIn, m_vvtEx);
}

const ResolvedEngine::Output& ResolvedEngine::update(const Input& input, double dt) {
    return step(input, dt, /*imposed=*/false, 0.0);
}

const ResolvedEngine::Output& ResolvedEngine::updateAtSpeed(double omega, const Input& input,
                                                            double dt) {
    return step(input, dt, /*imposed=*/true, std::max(0.0, sanitize(omega)));
}

/// Engine.update, resolved branch. Step numbers are EngineSim's.
const ResolvedEngine::Output& ResolvedEngine::step(const Input& input, double dt, bool imposed,
                                                   double imposedOmega) {
    const ResolvedEngineParameters& r = m_p.resolved;
    if (!(dt > 0.0)) return m_out;
    if (imposed) m_omega = imposedOmega;

    const double omega = m_omega;
    const double nRps = omega / (2.0 * kPi);
    const double nRpm = nRps * 60.0;
    const double throttle = std::max(0.0, std::min(1.0, sanitize(input.throttle)));

    const double idleRads = m_p.idleRpm * 2.0 * kPi / 60.0;
    const double redlineRads = m_p.redlineRpm * 2.0 * kPi / 60.0;
    const double hystRads = m_p.limiterHysteresisRpm * 2.0 * kPi / 60.0;
    const double minFiringRads = r.minFiringRpm * 2.0 * kPi / 60.0;
    const double stallRads = r.stallRpm * 2.0 * kPi / 60.0;

    //-- Step 0a: run state ---------------------------------------------------
    // Decel fuel cut: closed throttle and meaningfully above idle cuts fuel
    // entirely - engine braking is then friction + pumping, nothing else.
    // It waits out the flare after a starter start.
    const bool dfcoArmed = m_dfcoClock >= r.dfcoAfterStartDelayS;
    if (throttle < 0.01 && omega > idleRads * r.dfcoEnterRatio && dfcoArmed) {
        m_dfcoActive = true;
    } else if (throttle >= 0.01 || omega < idleRads * r.dfcoResumeRatio) {
        m_dfcoActive = false;
    }

    // Fuel-cut rev limiter: cut at redline, resume below redline - hysteresis.
    if (omega >= redlineRads) {
        m_limiterCut = true;
    } else if (omega < redlineRads - hystRads) {
        m_limiterCut = false;
    }

    // Spark needs ignition, enough crank speed and crank/cam sync. Below
    // stall speed without the starter, sync is lost and must be regained by
    // cranking - or by the driveline pushing the engine (a bump start).
    const bool cranking = input.starter || omega >= stallRads;
    if (!input.ignition || omega < 0.5 * minFiringRads || !cranking) {
        m_synced = false;
        m_syncRevs = 0.0;
    } else if (!m_synced) {
        m_syncRevs += omega * dt / (2.0 * kPi);
        m_synced = m_syncRevs >= r.crankRevsToFire;
    }
    const bool fireOk = input.ignition && m_synced && omega >= minFiringRads;
    m_fireTime = fireOk ? m_fireTime + dt : 0.0;
    m_starterRecent = input.starter ? 3.0 : std::max(0.0, m_starterRecent - dt);
    if (fireOk && !m_wasFiring) m_dfcoClock = (m_starterRecent > 0.0) ? 0.0 : 1.0e9;
    m_wasFiring = fireOk;
    m_dfcoClock = fireOk ? m_dfcoClock + dt : 0.0;
    const bool fuelCut = m_dfcoActive || m_limiterCut;
    const bool combusting = fireOk && !fuelCut;

    //-- Step 1: intake -------------------------------------------------------
    const double pAtm = m_p.ambientPressurePa;
    const double tIntake = m_turbo ? m_boostTemperature : m_p.intakeTemperatureK;
    const double iac = idleAirControl(throttle, omega, fireOk, dt);
    const double throttleArea = throttleEffectiveArea(throttle, r.throttleMaxAreaM2)
                              + iac * r.throttleMaxAreaM2;

    //-- Step 2/3: the ECU's air estimate, and the fuel for it ----------------
    const double etaV = m_veMeasured;       // as of the last closed intake event
    const double mAir = airEstimate(m_mapFiltered, tIntake, nRpm);
    double mFuel = (m_p.stoichAfr > 0.0) ? mAir / m_p.stoichAfr : 0.0;
    if (!combusting) mFuel = 0.0;
    // The air the cylinders actually trapped: a wrong estimate shows up here
    // as a genuine mixture error.
    const double mAirActual = (m_trappedAirKg > 0.0) ? m_trappedAirKg : mAir;
    const double afr = (mFuel > 0.0) ? mAirActual / mFuel : m_p.stoichAfr * 10.0;
    const double lambda = (m_p.stoichAfr > 0.0) ? afr / m_p.stoichAfr : 1.0;
    const double phi = (lambda > 0.0) ? 1.0 / lambda : 1.0;

    //-- Step 4: spark, then the cylinders ------------------------------------
    const double loadFrac = (pAtm > 0.0) ? std::min(1.0, m_mapFiltered / pAtm) : 1.0;
    double burnDeg = combustionDuration(nRpm, loadFrac, phi,
                                        0.05 + std::max(0.0, m_residualFraction - 0.12));
    burnDeg = std::min(burnDeg, 75.0);
    const double sparkBase = r.useMbtTiming ? std::min(35.0, mbtTiming(burnDeg, nRpm))
                                            : r.sparkTimingDegBtdc;
    // Idle spark trim: the fast torque loop at closed throttle. Idle runs
    // with the spark held back from MBT so the trim can add torque too.
    double idleTrim = 0.0;
    if (throttle < 0.01 && combusting && r.idleSparkGainDegPerRpm > 0.0) {
        idleTrim = std::max(-r.idleSparkMaxRetardDeg,
                            std::min(r.idleSparkMaxAdvanceDeg,
                                     r.idleSparkGainDegPerRpm
                                         * (idleTargetOmega() * 30.0 / kPi - nRpm)
                                         - r.idleSparkReserveDeg));
    }
    const double sparkTiming = std::max(0.0, sparkBase + idleTrim - m_knockRetardDeg);

    updateVvt(nRpm, loadFrac, dt);
    const double thetaDeg = pymod(m_thetaRad * 180.0 / kPi, 720.0);
    advanceBank(thetaDeg, (omega * dt) * (180.0 / kPi), dt, nRpm, m_manifoldPressure, tIntake,
                throttleArea, mFuel, mAir, sparkTiming, burnDeg, /*holdManifold=*/false);
    m_manifoldPressure = m_bankManifoldPressure;
    const double tInd = m_grossTorque;

    // Knock control: each detected knock retards the spark, which recovers.
    if (r.useKnockModel) {
        const long newEvents = m_knockEvents - m_knockEventsSeen;
        m_knockEventsSeen = m_knockEvents;
        if (newEvents > 0) {
            m_knockRetardDeg = std::min(r.knockRetardMaxDeg,
                                        m_knockRetardDeg + r.knockRetardStepDeg * static_cast<double>(newEvents));
        } else {
            m_knockRetardDeg = std::max(0.0, m_knockRetardDeg - r.knockRetardRecoveryDegPerS * dt);
        }
    }

    //-- Step 6: losses -------------------------------------------------------
    const double totalDisplacement = m_displacedVolume * m_n;
    const double tfmep = m_p.frictionC1Pa + m_p.frictionC2PaPerRevS * nRps
                       + m_p.frictionC3PaPerRevS2 * (nRps * nRps);
    const double tFric = std::max(tfmep * totalDisplacement / (4.0 * kPi), 0.0);
    const double accMep = 8000.0 * (1.0 + nRps / 50.0);
    const double tAcc = accMep * totalDisplacement / (4.0 * kPi);
    // The pumping loop is the gas work the cylinders just integrated on the
    // intake and exhaust strokes - intake vacuum and exhaust back-pressure
    // are in it. Positive = loss; negative = a boosted intake pushing.
    const double tPump = m_pumpingTorque;

    //-- Step 7-9: torque balance and crank dynamics (Heywood Eq 14.52) --------
    const double tLoad = sanitize(input.loadTorqueNm);
    double tNet = tInd - tFric - tPump - tAcc - tLoad;
    // Starter: torque falls linearly from stall to zero at free speed, and an
    // overrunning clutch means it can never drag the engine back.
    double tStarter = 0.0;
    const double starterFreeRads = r.starterFreeSpeedRpm * 2.0 * kPi / 60.0;
    if (input.starter && starterFreeRads > 0.0) {
        tStarter = r.starterStallTorqueNm * std::max(0.0, 1.0 - omega / starterFreeRads);
    }
    tNet += tStarter;

    double omegaNew = omega;
    if (!imposed) {
        const double alphaAcc = (m_p.engineInertia > 0.0) ? tNet / m_p.engineInertia : 0.0;
        omegaNew = std::max(omega + alphaAcc * dt, 0.0);
    }

    //-- Step 11: crank angle follows the distance the cylinders covered ------
    m_thetaRad += omega * dt;
    m_thetaRad = std::fmod(m_thetaRad, 4.0 * kPi);
    if (m_thetaRad < 0.0) m_thetaRad += 4.0 * kPi;
    m_omega = sanitize(omegaNew);

    //-- Step 15: outputs -----------------------------------------------------
    Output& o = m_out;
    o.omega = m_omega;
    o.rpm = m_omega * 60.0 / (2.0 * kPi);
    o.crankAngleDeg = m_thetaRad * 180.0 / kPi;
    o.grossTorqueNm = tInd;
    o.pumpingTorqueNm = tPump;
    o.frictionTorqueNm = tFric;
    o.accessoryTorqueNm = tAcc;
    o.starterTorqueNm = tStarter;
    o.loadTorqueNm = tLoad;
    o.brakeTorqueNm = tInd - tFric - tPump - tAcc;
    o.netTorqueNm = tNet;
    o.manifoldPressurePa = m_manifoldPressure;
    o.mapFilteredPa = m_mapFiltered;
    o.intakeTemperatureK = tIntake;
    o.airEstimateKg = mAir;
    o.trappedAirKg = m_trappedAirKg;
    o.fuelPerCycleKg = mFuel;
    o.lambda = lambda;
    o.volumetricEfficiency = etaV;
    o.airMassFlowKgS = m_airMassFlow;
    o.iacPosition = iac;
    o.sparkTimingDegBtdc = sparkTiming;
    o.combustionDurationDeg = burnDeg;
    const double imepNet = m_cycleWorkJ / m_displacedVolume;
    const double pmep = -m_cyclePumpingWorkJ / m_displacedVolume;
    o.imepNetBar = imepNet / 1e5;
    o.pmepBar = pmep / 1e5;
    o.peakPressurePa = m_peakPressurePa;
    const double qFuelCycle = mFuel * m_p.lowerHeatingValue;
    o.thermalEfficiency = (qFuelCycle > 0.0)
        ? std::max(0.0, std::min(1.0, m_cycleWorkJ / qFuelCycle)) : 0.0;
    o.residualFraction = m_residualFraction;
    o.burnQuality = m_burnQuality;
    o.misfireRate = m_misfireRate;
    o.knockRetardDeg = m_knockRetardDeg;
    o.heatLossW = m_heatLossW;
    o.vvtIntakeDeg = m_vvtIn;
    o.vvtExhaustDeg = m_vvtEx;
    o.exhaustPressurePa = m_exP;
    o.exhaustTemperatureK = m_exT;
    o.boostPressurePa = m_turbo ? m_boostPressure : pAtm;
    o.turboSpeedRps = m_turbo ? m_tOmega / (2.0 * kPi) : 0.0;
    o.wastegatePosition = m_turbo ? m_wastegatePosition : 0.0;
    o.turbinePowerW = m_turbPower;
    o.firing = fireOk;
    o.combusting = combusting;
    o.fuelCut = fuelCut;
    o.limiterActive = m_limiterCut;
    o.starter = input.starter;
    o.substeps = m_substepsLast;
    return m_out;
}

//---------------------------------------------------------------------------
//  Read-outs for the presentation layer (not part of EngineSim's Engine)
//---------------------------------------------------------------------------

void ResolvedEngine::blowdownState(double& pressurePa, double& temperatureK) const {
    // The last trace bin that lies wholly before the exhaust valve opens.
    const double evo = pymod(m_exhaustOpenDeg - m_camEx, kCycleDeg);
    int bin = static_cast<int>(evo * (kTraceBins / kCycleDeg)) - 1;
    if (bin < 0) bin += kTraceBins;
    const double p = m_trace[static_cast<std::size_t>(bin)];
    const double mass = cyl(0)[F_trapped_mass_kg];
    if (p > 0.0 && mass > 0.0) {
        const double volume = exactVolume((bin + 0.5) * (kCycleDeg / kTraceBins));
        pressurePa = p;
        temperatureK = p * volume / (mass * kRAir);
    } else {
        // Nothing recorded yet (the engine has not turned a cycle): the
        // cylinder is at whatever the exhaust plenum is.
        pressurePa = m_exP;
        temperatureK = m_exT;
    }
}

double ResolvedEngine::cylinderPressurePa(int index) const {
    if (index < 0 || index >= m_n) return 0.0;
    return cyl(index)[F_pressure];
}

} // namespace vsim
