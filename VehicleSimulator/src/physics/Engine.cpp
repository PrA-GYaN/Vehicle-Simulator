#include "physics/Engine.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace vsim {
namespace {

// Physical constants — Heywood Table D.1 / Assumption A13. Same values as
// EngineSim's engine_sim/thermodynamics.py.
constexpr double kRAir = 287.058;      // [J/(kg.K)] specific gas constant, air
constexpr double kGamma = 1.4;         // [-] specific heat ratio, air
constexpr double kCvAir = kRAir / (kGamma - 1.0);  // ~718 J/(kg.K)

// Polytropic exponents — Heywood Assumption A13. Same as EngineSim.
constexpr double kNCompression = 1.32;
constexpr double kNExpansion   = 1.28;

double polytropicTemperature(double t1, double v1, double v2, double n) {
    return (v2 > 0.0) ? t1 * std::pow(v1 / v2, n - 1.0) : t1;
}

/// Heywood Eq 2.26 (functional form authored to match its described shape,
/// same as EngineSim's air_model.py): eta_v peaks then falls with speed.
/// `refRpm` is EngineParameters::veReferenceRpm (6000 in EngineSim).
double volumetricEfficiency(double omega, double maxEtaV, double refRpm) {
    const double rpm = omega * 60.0 / kTwoPi;
    const double nNorm = rpm / refRpm;
    const double etaV = maxEtaV * (1.0 - 0.5 * std::pow(std::max(0.0, nNorm), 1.5));
    return std::max(etaV, 0.3);
}

/// KB gap G-E01 in EngineSim's own KB_GAPS.md: the KB does not give a
/// closed-form throttle-to-pressure relation. This is EngineSim's authored
/// approximation (air_model.py::intake_pressure), ported as-is.
double intakePressure(double throttle, double ambientPa) {
    const double pMin = 0.2 * ambientPa;
    const double pMax = ambientPa;
    return pMin + (pMax - pMin) * std::sqrt(clampd(throttle, 0.0, 1.0));
}

/// Charge trapped in one cylinder, per CYCLE [kg].
///
///     m_a = eta_v * rho_i * V_d            (Heywood Eq 2.27, per cylinder)
///
/// NOTE the absence of an n_R divisor. eta_v is DEFINED as
/// 2*m_dot_a/(rho_ai*V_d*N) for a four-stroke, so n_R = 2 is what converts a
/// per-cycle mass into a mass FLOW (m_dot = m_a*N/n_R) - it is not part of
/// the per-cycle charge itself. EngineSim's air_model.trapped_mass() applies
/// it here as well, which halves the charge; that is carried as a known
/// difference rather than copied, because halving the charge here silently
/// halves the combustion energy and therefore the torque.
double trappedMass(double etaV, double pIntake, double tIntake, double displacementPerCyl) {
    if (tIntake <= 0.0 || pIntake <= 0.0) return 0.0;
    const double rho = pIntake / (kRAir * tIntake);
    return etaV * rho * displacementPerCyl;
}

} // namespace

Engine::Engine(const EngineParameters& params) : m_params(params) {
    m_params.validate();
    if (m_params.useResolved) {
        m_resolved.emplace(m_params);
        // Heywood's yardstick for a naturally aspirated SI engine is 8.5-10.5
        // bar bmep; boost raises it roughly with intake density.
        const double densityRatio = m_params.hasTurbo
            ? m_params.resolved.turbo.targetBoostPa / m_params.ambientPressurePa : 1.0;
        m_peakTorqueSeen = 11.0e5 * densityRatio * m_params.totalDisplacement() / (4.0 * kPi);
    }
}

//---------------------------------------------------------------------------
//  Resolved core behind the cycle-averaged interface.
//---------------------------------------------------------------------------

Engine::TorqueBreakdown Engine::stepResolved(double throttle, double loadTorqueNm, double dt,
                                             bool imposed, double omega) {
    ResolvedEngine& engine = *m_resolved;
    TorqueBreakdown out;
    if (!(dt > 0.0)) return m_lastBreakdown;

    // The ECU loops inside the resolved engine were tuned at 1-2 ms. Run it
    // at its own step inside this (much longer) physics step, with the load
    // or the imposed speed held constant across the sub-steps.
    const int n = std::max(1, static_cast<int>(std::ceil(dt / m_params.resolved.engineTimestepS - 1e-9)));
    const double h = dt / n;

    ResolvedEngine::Input in;
    in.throttle = throttle;
    in.loadTorqueNm = loadTorqueNm;
    in.ignition = true;

    double net = 0.0;
    double stepAngle = 0.0;
    for (int k = 0; k < n; ++k) {
        // No key and no starter button in this simulator: an engine that is
        // not firing (stalled, or just reset cold) cranks itself until it is.
        // The starter has an overrunning clutch, so this is harmless when the
        // driveline is already turning the engine.
        in.starter = !engine.lastOutput().firing;
        const double omegaBefore = imposed ? omega : engine.omega();
        const ResolvedEngine::Output& o = imposed ? engine.updateAtSpeed(omega, in, h)
                                                  : engine.update(in, h);
        out.indicatedTorqueNm += o.grossTorqueNm;
        out.frictionTorqueNm  += o.frictionTorqueNm;
        out.pumpingTorqueNm   += o.pumpingTorqueNm;
        out.accessoryTorqueNm += o.accessoryTorqueNm;
        net += o.netTorqueNm;

        m_torqueWindow.push_back({omegaBefore * h, h, o.brakeTorqueNm + o.starterTorqueNm});
        stepAngle += omegaBefore * h;
    }
    m_shaftTorqueFiltered = windowedShaftTorque(stepAngle);
    const double inv = 1.0 / n;
    out.indicatedTorqueNm *= inv;
    out.frictionTorqueNm  *= inv;
    out.pumpingTorqueNm   *= inv;
    out.accessoryTorqueNm *= inv;
    out.loadTorqueNm = loadTorqueNm;
    out.brakeTorqueNm = m_shaftTorqueFiltered;
    out.netTorqueNm = imposed ? m_shaftTorqueFiltered : net * inv;

    const ResolvedEngine::Output& o = engine.lastOutput();
    out.intakePressurePa = o.mapFilteredPa;
    out.volumetricEfficiency = o.volumetricEfficiency;
    out.thermalEfficiency = o.thermalEfficiency;
    out.boostPa = std::max(0.0, o.boostPressurePa - m_params.ambientPressurePa);
    out.chargeTemperatureK = o.intakeTemperatureK;
    engine.blowdownState(out.exhaustBlowdownPressurePa, out.exhaustBlowdownTemperatureK);
    out.airMassPerCylinderKg = (o.trappedAirKg > 0.0) ? o.trappedAirKg : o.airEstimateKg;
    out.fuelMassPerCylinderKg = o.fuelPerCycleKg;

    m_omega = engine.omega();
    m_boostPa = out.boostPa;
    m_limiterActive = m_limiterWasActive = o.limiterActive;
    if (o.combusting) m_peakTorqueSeen = std::max(m_peakTorqueSeen, m_shaftTorqueFiltered);
    m_lastBreakdown = out;
    return out;
}

/// Flywheel torque as its mean over one firing period.
///
/// A sub-step at low speed sweeps only part of a firing interval, so its
/// torque swings between one cylinder's compression stroke and the next
/// one's power stroke - by more than the mean itself. The engine's own speed
/// integrates that ripple honestly (it is what makes an idle lumpy). But a
/// LOCKED driveline hands flywheel torque straight to the tyres, and this
/// simulator has neither the driveline compliance that absorbs the ripple
/// in a car nor a physics step that could resolve it (100 Hz against a
/// firing frequency of 30-500 Hz: it would alias into a slow, entirely
/// fictitious surge).
///
/// So what leaves the engine is a boxcar average over whole periods of the
/// torque waveform - one firing interval on an even-fire engine, the whole
/// 720 deg cycle otherwise - and as many of them as it takes to cover the
/// crank angle this physics step swept (`stepAngle`). Every sample is itself
/// work/angle over its sub-step, so the result is the true mean torque over
/// those periods at any speed and any phase: no ripple in steady state, and
/// a lag of at most one firing interval beyond the step itself (33 ms for a
/// four at 900 rpm). It is the same quantity the cycle-averaged model
/// reports by construction.
double Engine::windowedShaftTorque(double stepAngle) {
    const double onePeriod = degToRad(m_params.resolved.firingIntervalsDeg.empty()
                                          ? 720.0 / m_params.numCylinders : 720.0);
    const double period = onePeriod * std::max(1.0, std::ceil(stepAngle / onePeriod - 1e-9));
    // Forget anything older than this in time: a cranking or stalled engine
    // takes arbitrarily long to turn through a period.
    constexpr double kMaxAgeS = 0.25;

    double angle = 0.0, work = 0.0, age = 0.0;
    std::size_t keep = m_torqueWindow.size();
    for (std::size_t i = m_torqueWindow.size(); i-- > 0;) {
        const TorqueSample& s = m_torqueWindow[i];
        if (age >= kMaxAgeS) { keep = i + 1; break; }
        if (angle + s.angle >= period) {
            // The oldest sample that reaches into the window: only the part
            // of it that lies inside counts.
            work += s.torqueNm * (period - angle);
            angle = period;
            keep = i;
            break;
        }
        angle += s.angle;
        work += s.torqueNm * s.angle;
        age += s.dt;
        keep = i;
    }
    if (keep > 0) m_torqueWindow.erase(m_torqueWindow.begin(), m_torqueWindow.begin() + static_cast<std::ptrdiff_t>(keep));
    if (angle > 1e-9) return work / angle;
    // Not turning: nothing to average over. The latest value stands.
    return m_torqueWindow.empty() ? 0.0 : m_torqueWindow.back().torqueNm;
}

double Engine::peakTorqueEstimateNm() const {
    if (m_resolved) return std::max(1.0, m_peakTorqueSeen);
    // evaluateTorque() is const and stateless, so probing it is free of
    // side effects.
    double peak = 1.0;
    for (int i = 1; i <= 12; ++i) {
        const double w = m_params.redlineOmega() * i / 12.0;
        peak = std::max(peak, evaluateTorque(w, 1.0, 0.0).netTorqueNm);
    }
    return peak;
}

double Engine::rpm() const { return m_omega * 60.0 / kTwoPi; }

double Engine::targetBoostPa(double omega, double throttle) const {
    if (!m_params.hasTurbo) return 0.0;
    // Exhaust energy - and so turbine power - rises with engine speed; the
    // wastegate caps the result at maxBoostPa. Smoothstep between the two
    // authored speeds is the shape, not a derived characteristic.
    const double rpm = std::max(0.0, omega) * 60.0 / kTwoPi;
    const double x = clampd((rpm - m_params.boostThresholdRpm) /
                            (m_params.fullBoostRpm - m_params.boostThresholdRpm), 0.0, 1.0);
    const double spool = x * x * (3.0 - 2.0 * x);
    return m_params.maxBoostPa * spool * clampd(throttle, 0.0, 1.0);
}

void Engine::advanceBoost(double omega, double throttle, double dt) {
    if (!m_params.hasTurbo || dt <= 0.0) { m_boostPa = 0.0; return; }
    const double target = targetBoostPa(omega, throttle);
    // Building boost waits on the turbine (turbo lag); losing it does not -
    // on a lift the blow-off valve vents the charge almost at once.
    const double tau = (target > m_boostPa) ? m_params.spoolTimeConstantS
                                            : m_params.blowOffTimeConstantS;
    m_boostPa += (target - m_boostPa) * (1.0 - std::exp(-dt / tau));
}

Engine::TorqueBreakdown Engine::computeBreakdown(double omega, double throttle,
                                                  double loadTorqueNm, double boostPa) const {
    TorqueBreakdown out;
    const double t = clampd(sanitize(throttle), 0.0, 1.0);
    const double w = std::max(0.0, sanitize(omega));

    //-----------------------------------------------------------------------
    //  Air + fuel (Heywood Eq 2.26/2.27b, Eq 3.8/3.9 via EngineSim's
    //  fuel_model.py — always stoichiometric here, same simplification
    //  EngineSim's core makes).
    //-----------------------------------------------------------------------
    const double etaV = volumetricEfficiency(w, m_params.maxVolumetricEfficiency,
                                             m_params.veReferenceRpm);
    const double boost = m_params.hasTurbo ? std::max(0.0, boostPa) : 0.0;
    const double pIntake = intakePressure(t, m_params.ambientPressurePa) + boost;

    // Charge temperature. Naturally aspirated: ambient. Turbocharged: the
    // compressor's temperature rise, Heywood Eq 6.40 solved for T_02, less
    // what the aftercooler removes (Fig 14-24). A hotter charge is a less
    // dense one - trappedMass() below sees it through rho_i.
    double tIntake = m_params.intakeTemperatureK;
    if (boost > 0.0) {
        const double t1 = m_params.intakeTemperatureK;
        const double pr = (m_params.ambientPressurePa + boost) / m_params.ambientPressurePa;
        const double t2 = t1 * (1.0 + (std::pow(pr, (kGamma - 1.0) / kGamma) - 1.0)
                                      / m_params.compressorEfficiency);
        tIntake = t2 - m_params.intercoolerEffectiveness * (t2 - t1);
    }
    const double mAirPerCyl = trappedMass(etaV, pIntake, tIntake,
                                           m_params.displacementPerCylinder());
    double mFuelPerCyl = (m_params.stoichAfr > 0.0) ? mAirPerCyl / m_params.stoichAfr : 0.0;

    // Decel fuel cutoff (DFCO): real ECUs cut fuel entirely at closed
    // throttle once meaningfully above idle, producing genuine engine
    // braking (friction + pumping only) instead of coasting on unmodified
    // closed-throttle combustion. Needed because intakePressure() floors at
    // 0.2*p_amb rather than true vacuum (EngineSim KB gap G-E01), so without
    // this, closed-throttle combustion stays strong up to a very high RPM.
    // Fixed the same day in ../EngineSim/engine_sim/engine.py - see
    // ../EngineSim/README_COMPLETED.md. Standard ECU behaviour, not
    // invented physics.
    if (t < 0.01 && w > m_params.idleOmega() * 1.2) {
        mFuelPerCyl = 0.0;
    }

    out.intakePressurePa = pIntake;
    out.volumetricEfficiency = etaV;
    out.boostPa = boost;
    out.chargeTemperatureK = tIntake;

    //-----------------------------------------------------------------------
    //  Air-standard Otto cycle, ONE cylinder (Heywood Eq 2.21):
    //    1->2 polytropic compression, 2->3 constant-volume heat addition,
    //    3->4 polytropic expansion, 4->1 constant-volume heat rejection.
    //  Same energy-balance approach as EngineSim's combustion_model.py.
    //-----------------------------------------------------------------------
    double indicatedWorkPerCyl = 0.0;
    double thermalEfficiency = 0.0;
    if (mAirPerCyl > 0.0 && mFuelPerCyl > 0.0) {
        const double vc = m_params.clearanceVolume();
        const double vTdc = vc;
        const double vBdc = vc * m_params.compressionRatio;

        const double t2 = polytropicTemperature(tIntake, vBdc, vTdc, kNCompression);

        const double fuelEnergy = mFuelPerCyl * m_params.lowerHeatingValue;
        const double qIn = fuelEnergy * m_params.combustionEfficiency;
        const double t3 = (t2 > 0.0) ? t2 + qIn / (mAirPerCyl * kCvAir) : t2;

        const double t4 = polytropicTemperature(t3, vTdc, vBdc, kNExpansion);

        const double qOut = (t4 > tIntake) ? mAirPerCyl * kCvAir * (t4 - tIntake) : 0.0;

        indicatedWorkPerCyl = qIn - qOut;
        thermalEfficiency = (qIn > 0.0) ? indicatedWorkPerCyl / qIn : 0.0;

        out.exhaustBlowdownTemperatureK = t4;
        out.exhaustBlowdownPressurePa =
            (mAirPerCyl + mFuelPerCyl) * kRAir * t4 / vBdc;
    } else if (mAirPerCyl > 0.0) {
        // Fuel cut: the charge is compressed and re-expanded with no heat
        // added, so it leaves at roughly intake conditions - which is why a
        // fuel-cut engine goes quiet (no blowdown pulse to speak of).
        const double vBdc = m_params.clearanceVolume() * m_params.compressionRatio;
        out.exhaustBlowdownTemperatureK = tIntake;
        out.exhaustBlowdownPressurePa = mAirPerCyl * kRAir * tIntake / vBdc;
    }
    out.airMassPerCylinderKg = mAirPerCyl;
    out.fuelMassPerCylinderKg = mFuelPerCyl;

    //-----------------------------------------------------------------------
    //  Cycle-averaged indicated torque for the WHOLE engine.
    //
    //      P_i = W_net,total * (N / n_R)        cycles per second, 4-stroke
    //      T_i = P_i / omega = W_net,total / (2*pi*n_R) = W_net,total / (4*pi)
    //
    //  n_R DIVIDES. EngineSim multiplies by it instead (`W_net * 2/(2*pi)`),
    //  which is 4x too high - and that error is invisible on a 4-cylinder,
    //  because W_net_percyl * 4/(4*pi) happens to equal W_net_percyl/pi too.
    //  It only shows up once the cylinder count changes, or once the result
    //  is checked against BMEP. Do not "simplify" this back.
    //
    //  Checked against Heywood's own yardstick rather than by inspection:
    //  bmep = T*4*pi/V_d lands at ~9 bar at peak for the shipped 2.0 L
    //  naturally-aspirated config, inside his 8.5-10.5 bar NA SI range.
    //-----------------------------------------------------------------------
    const double indicatedWorkTotal =
        indicatedWorkPerCyl * static_cast<double>(m_params.numCylinders);
    out.indicatedTorqueNm = indicatedWorkTotal / (4.0 * kPi);
    out.thermalEfficiency = thermalEfficiency;

    //-----------------------------------------------------------------------
    //  Friction, pumping, accessory losses (Heywood Eq 13.5/13.6, §13.7).
    //-----------------------------------------------------------------------
    const double nRevS = w / kTwoPi;
    const double totalDisplacement = m_params.totalDisplacement();

    const double tfmep = m_params.frictionC1Pa
                        + m_params.frictionC2PaPerRevS * nRevS
                        + m_params.frictionC3PaPerRevS2 * nRevS * nRevS;
    out.frictionTorqueNm = std::max(tfmep * totalDisplacement / (4.0 * kPi), 0.0);

    const double pIGauge = pIntake - m_params.ambientPressurePa;
    // A turbine in the exhaust raises the pressure the engine pumps against
    // along with the boost it produces - Heywood Fig 6-2: exhaust manifold
    // pressure sits between cylinder and ambient, and rises with turbine
    // work. Rising one-for-one with boost is AUTHORED (no turbine map), and
    // keeps pumping work from turning into a free gain under boost.
    const double pEGauge = m_params.exhaustBackPressurePa + boost - m_params.ambientPressurePa;
    double pmep = std::max(0.0, -pIGauge + pEGauge);
    pmep *= 1.0 + 0.5 * (nRevS / 50.0);
    out.pumpingTorqueNm = pmep * totalDisplacement / (4.0 * kPi);

    const double accMep = 8000.0 * (1.0 + nRevS / 50.0);
    out.accessoryTorqueNm = accMep * totalDisplacement / (4.0 * kPi);

    // Signed: negative is a load that DRIVES the crank - a slipping clutch
    // whose wheel side is turning faster than the engine (Powertrain.cpp).
    out.loadTorqueNm = sanitize(loadTorqueNm);
    out.brakeTorqueNm = out.indicatedTorqueNm - out.frictionTorqueNm
                       - out.pumpingTorqueNm - out.accessoryTorqueNm;
    out.netTorqueNm = out.indicatedTorqueNm - out.frictionTorqueNm
                     - out.pumpingTorqueNm - out.accessoryTorqueNm - out.loadTorqueNm;
    return out;
}

Engine::TorqueBreakdown Engine::evaluateTorque(double omega, double throttle,
                                                double loadTorqueNm) const {
    // Steady state: the boost the turbo would settle at here. This is what a
    // dyno curve means, and it is all a const probe can honestly report.
    return computeBreakdown(omega, throttle, loadTorqueNm, targetBoostPa(omega, throttle));
}

Engine::TorqueBreakdown Engine::evaluateTorqueAt(double omega, double throttle, double dt) {
    if (m_resolved) return stepResolved(throttle, 0.0, dt, /*imposed=*/true, std::max(0.0, sanitize(omega)));
    advanceBoost(omega, throttle, dt);
    m_lastBreakdown = computeBreakdown(omega, throttle, 0.0, m_boostPa);
    return m_lastBreakdown;
}

double Engine::applyLimiter(double omegaNew) {
    const double redline = m_params.redlineOmega();
    const double hyst = m_params.limiterHysteresisOmega();
    const bool active = m_limiterWasActive ? (omegaNew > redline - hyst)
                                            : (omegaNew >= redline);
    m_limiterActive = active;
    m_limiterWasActive = active;
    return active ? std::min(omegaNew, redline) : omegaNew;
}

Engine::TorqueBreakdown Engine::update(double throttle, double loadTorqueNm, double dt) {
    if (m_resolved) return stepResolved(throttle, loadTorqueNm, dt, /*imposed=*/false, 0.0);
    const double t = clampd(sanitize(throttle), 0.0, 1.0);
    const double omega = m_omega;

    advanceBoost(omega, t, dt);
    TorqueBreakdown breakdown = computeBreakdown(omega, t, loadTorqueNm, m_boostPa);
    double netTorque = breakdown.netTorqueNm;

    //-----------------------------------------------------------------------
    //  Idle / closed-throttle governor. KB GAP G-E03 in EngineSim's own
    //  KB_GAPS.md: not from Heywood, an authored control loop. Corrects in
    //  BOTH directions and stays active at any speed while throttle is
    //  closed - an earlier version (ported as-is from EngineSim, then fixed
    //  in both places the same day) only added torque below 1.5x idle and
    //  did nothing above it, so closed-throttle combustion (never zero here
    //  - intake pressure floors at 0.2*p_amb, not zero, same G-E01
    //  simplification) ran unopposed to its own equilibrium around 6000 RPM
    //  instead of the intended ~800. See ../EngineSim/engine_sim/engine.py
    //  for the sibling fix and ../EngineSim/README_COMPLETED.md for the
    //  writeup - this is the same bug, ported and fixed the same way.
    //-----------------------------------------------------------------------
    const double idleOmega = m_params.idleOmega();
    if (t < 0.01 && omega > 0.0) {
        const double error = idleOmega - omega;
        m_idleIntegral = clampd(m_idleIntegral + error * dt, -50.0, 50.0);
        constexpr double kKp = 0.6;
        constexpr double kKi = 2.0;
        const double idleTorque = kKp * error + kKi * m_idleIntegral;
        netTorque += idleTorque;
    } else {
        m_idleIntegral = 0.0;
    }

    const double inertia = m_params.engineInertia;
    const double alpha = (inertia > 0.0) ? netTorque / inertia : 0.0;
    double omegaNew = std::max(0.0, omega + alpha * dt);
    omegaNew = applyLimiter(omegaNew);

    m_omega = omegaNew;
    m_lastBreakdown = breakdown;
    return breakdown;
}

void Engine::reset(bool idle) {
    if (m_resolved) {
        // Idle = EngineSim's warm start (a few fired cycles already behind
        // it); otherwise stopped and cold, and stepResolved() cranks it.
        if (idle) m_resolved->start(); else m_resolved->reset();
        m_shaftTorqueFiltered = 0.0;
        m_torqueWindow.clear();
    }
    m_omega = idle ? m_params.idleOmega() : 0.0;
    m_idleIntegral = 0.0;
    m_boostPa = 0.0;
    m_limiterWasActive = false;
    m_limiterActive = false;
    m_lastBreakdown = TorqueBreakdown{};
}

} // namespace vsim
