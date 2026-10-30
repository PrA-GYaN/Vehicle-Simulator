#pragma once
//===========================================================================
//  EngineDyno.h  --  a speed-holding dynamometer for the detailed engine.
//
//  WHY THIS EXISTS. The cycle-averaged engine can be asked "what torque at
//  this speed" as a pure function (Engine::evaluateTorque), and the UI draws
//  its torque curve by asking 64 times a frame. The resolved engine has no
//  such answer: its torque at a speed is whatever comes out once the intake
//  manifold has filled, the ECU's air model has converged on what the
//  cylinders really trap, the exhaust has warmed and - on a turbo engine -
//  the shaft has spooled against the wastegate. Finding it means doing what
//  a real dyno does: hold the speed, wait, measure.
//
//  This runs a PRIVATE copy of the engine, built from the same parameters,
//  held at each speed in turn. It never touches the engine in the car, so
//  using it cannot perturb a running simulation, and it is advanced in
//  wall-clock slices so a UI can run it a few milliseconds per frame.
//  tools/engine_dyno.cpp runs the same class to completion and prints it.
//
//  Works for both engine models (a cycle-averaged engine is simply probed),
//  so callers need no second code path.
//===========================================================================

#include <optional>
#include <vector>

#include "config/EngineParameters.h"
#include "physics/ResolvedEngine.h"

namespace vsim {

class EngineDyno {
public:
    struct Point {
        double rpm = 0.0;
        double torqueNm = 0.0;        ///< at the flywheel, mean over whole cycles
        double powerKw = 0.0;
        double bmepBar = 0.0;         ///< T 4 pi / V_d - Heywood's size-independent yardstick
        double manifoldPressurePa = 0.0;
        double boostPa = 0.0;         ///< gauge, upstream of the throttle; 0 if NA
        double exhaustPressurePa = 0.0;
        double lambda = 1.0;
        double volumetricEfficiency = 0.0;
        double sparkDegBtdc = 0.0;
        double settleTimeS = 0.0;     ///< simulated time spent reaching it
        bool   converged = true;      ///< false = the time limit was hit first
    };

    /// `points` speeds evenly spaced from idle to redline, at `throttle`.
    explicit EngineDyno(const EngineParameters& params, int points = 16, double throttle = 1.0);

    /// Run for about `wallBudgetS` of real time. Returns true once every
    /// point has been measured.
    bool advance(double wallBudgetS);
    void runToCompletion();

    bool done() const { return m_index >= m_points.size(); }
    /// Points measured so far are [0, measuredCount()).
    std::size_t measuredCount() const { return m_index; }
    const std::vector<Point>& points() const { return m_points; }
    /// Simulated engine time spent so far [s] - with the CPU time it took,
    /// the real-time factor of the engine model.
    double simulatedTimeS() const { return m_simulatedS; }

private:
    void beginPoint();
    /// One engine step at the current point; finishes the point when the
    /// torque has settled.
    void stepPoint();

    EngineParameters m_params;
    double m_throttle = 1.0;
    std::vector<Point> m_points;
    std::size_t m_index = 0;
    std::optional<ResolvedEngine> m_engine;

    // state of the point in progress
    double m_omega = 0.0;
    double m_dt = 0.002;
    double m_pointTimeS = 0.0;
    double m_chunkAngle = 0.0, m_chunkWork = 0.0;
    double m_chunkMinAngle = 0.0;
    double m_previousChunkTorque = 0.0;
    int    m_chunks = 0, m_stableChunks = 0;
    double m_simulatedS = 0.0;
};

} // namespace vsim
