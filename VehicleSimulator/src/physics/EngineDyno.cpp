#include "physics/EngineDyno.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "core/Units.h"
#include "physics/Engine.h"

namespace vsim {
namespace {

// AUTHORED measurement policy - how long a dyno operator waits.
constexpr double kMinSettleS = 0.5;          // never accept a point sooner than this
constexpr double kMaxSettleNaS = 4.0;        // naturally aspirated: manifold + air model
constexpr double kMaxSettleTurboS = 12.0;    // boosted: the shaft and the wastegate as well
constexpr double kChunkCycles = 6.0;         // torque is averaged over this many whole cycles...
constexpr double kChunkMinS = 0.2;           // ...or this long, whichever is more
// Successive averages never agree exactly: the ECU's air model keeps
// trimming and the wastegate keeps working. 1 % is inside that chatter.
constexpr double kStableFraction = 0.01;
constexpr double kStableFloorNm = 0.5;       // for points near zero torque
constexpr int    kStableChunksNeeded = 2;

} // namespace

EngineDyno::EngineDyno(const EngineParameters& params, int points, double throttle)
    : m_params(params), m_throttle(clampd(sanitize(throttle, 1.0), 0.0, 1.0)) {
    m_params.validate();
    const int n = std::max(2, points);
    m_points.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        m_points[static_cast<std::size_t>(i)].rpm =
            m_params.idleRpm + (m_params.redlineRpm - m_params.idleRpm) * (i / double(n - 1));
    }
    // Stay just under the limiter at the top point, or it measures a fuel cut.
    m_points.back().rpm = m_params.redlineRpm - std::min(50.0, 0.5 * m_params.limiterHysteresisRpm);

    if (m_params.useResolved) {
        m_engine.emplace(m_params);
        m_engine->start();
        m_dt = m_params.resolved.engineTimestepS;
        beginPoint();
    } else {
        // Cycle-averaged: a pure function of speed, nothing to wait for.
        const Engine engine(m_params);
        for (Point& p : m_points) {
            const double w = p.rpm * kTwoPi / 60.0;
            const Engine::TorqueBreakdown b = engine.evaluateTorque(w, m_throttle, 0.0);
            p.torqueNm = b.netTorqueNm;
            p.powerKw = b.netTorqueNm * w / 1000.0;
            p.bmepBar = b.netTorqueNm * 4.0 * kPi / m_params.totalDisplacement() / 1e5;
            p.manifoldPressurePa = b.intakePressurePa;
            p.boostPa = b.boostPa;
            p.exhaustPressurePa = m_params.exhaustBackPressurePa + b.boostPa;
            p.volumetricEfficiency = b.volumetricEfficiency;
        }
        m_index = m_points.size();
    }
}

void EngineDyno::beginPoint() {
    if (done()) return;
    // The engine carries on from the previous point - a sweep, as on a real
    // dyno - so the turbo is already spooled and the air model already learned.
    m_omega = m_points[m_index].rpm * kTwoPi / 60.0;
    m_pointTimeS = 0.0;
    m_chunkAngle = m_chunkWork = 0.0;
    m_chunkMinAngle = std::max(kChunkCycles * 4.0 * kPi, m_omega * kChunkMinS);
    m_previousChunkTorque = 0.0;
    m_chunks = m_stableChunks = 0;
}

void EngineDyno::stepPoint() {
    ResolvedEngine::Input in;
    in.throttle = m_throttle;
    in.starter = !m_engine->lastOutput().firing;
    const ResolvedEngine::Output& o = m_engine->updateAtSpeed(m_omega, in, m_dt);
    m_pointTimeS += m_dt;
    m_simulatedS += m_dt;

    // Mean torque = work / angle, over whole cycles so that no cylinder's
    // power stroke is counted without its compression stroke.
    const double angle = m_omega * m_dt;
    m_chunkAngle += angle;
    m_chunkWork += o.brakeTorqueNm * angle;
    if (m_chunkAngle < m_chunkMinAngle) return;

    const double torque = m_chunkWork / m_chunkAngle;
    m_chunkAngle = m_chunkWork = 0.0;
    ++m_chunks;
    const double band = std::max(kStableFraction * std::abs(torque), kStableFloorNm);
    m_stableChunks = (m_chunks > 1 && std::abs(torque - m_previousChunkTorque) <= band)
        ? m_stableChunks + 1 : 0;
    const double previous = m_previousChunkTorque;
    m_previousChunkTorque = torque;

    const double limit = m_params.resolved.useTurbo ? kMaxSettleTurboS : kMaxSettleNaS;
    const bool settled = m_stableChunks >= kStableChunksNeeded && m_pointTimeS >= kMinSettleS;
    if (!settled && m_pointTimeS < limit) return;

    Point& p = m_points[m_index];
    p.torqueNm = (m_chunks > 1) ? 0.5 * (torque + previous) : torque;
    p.powerKw = p.torqueNm * m_omega / 1000.0;
    p.bmepBar = p.torqueNm * 4.0 * kPi / m_params.totalDisplacement() / 1e5;
    p.manifoldPressurePa = o.mapFilteredPa;
    p.boostPa = std::max(0.0, o.boostPressurePa - m_params.ambientPressurePa);
    p.exhaustPressurePa = o.exhaustPressurePa;
    p.lambda = o.lambda;
    p.volumetricEfficiency = o.volumetricEfficiency;
    p.sparkDegBtdc = o.sparkTimingDegBtdc;
    p.settleTimeS = m_pointTimeS;
    p.converged = settled;
    ++m_index;
    beginPoint();
}

bool EngineDyno::advance(double wallBudgetS) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::duration<double>(std::max(0.0, wallBudgetS));
    while (!done()) {
        for (int k = 0; k < 16 && !done(); ++k) stepPoint();
        if (clock::now() >= deadline) break;
    }
    return done();
}

void EngineDyno::runToCompletion() {
    while (!done()) stepPoint();
}

} // namespace vsim
