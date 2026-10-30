#pragma once
//===========================================================================
//  Telemetry.h  --  fixed-rate sample recording and CSV export.
//
//  Samples are captured at the PHYSICS rate, not the render rate. That is the
//  whole point: a telemetry trace whose sample interval wobbles with frame
//  rate is useless for anything quantitative.
//===========================================================================

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "config/Config.h"
#include "core/VehicleState.h"

namespace vsim {

class Vehicle;

/// One row of the log. Column order here is the column order in the CSV.
struct TelemetrySample {
    double timestamp     = 0.0;  ///< [s]  simulation time since reset
    double x             = 0.0;  ///< [m]  rear axle reference point
    double y             = 0.0;  ///< [m]
    double z             = 0.0;  ///< [m]
    double yaw           = 0.0;  ///< [rad]
    double velocity      = 0.0;  ///< [m/s]
    double acceleration  = 0.0;  ///< [m/s^2] realised, not commanded
    double steeringAngle = 0.0;  ///< [rad] road wheel
    double yawRate       = 0.0;  ///< [rad/s]
    double throttle      = 0.0;  ///< [0,1] applied, after rate limiting
    double brake         = 0.0;  ///< [0,1] applied, after rate limiting

    //-- lateral / tyre state -------------------------------------------
    //
    // Zero under a model that does not compute them (the kinematic bicycle
    // model). `wheel_data_valid` records which case a row belongs to, so a
    // trace can never be misread as measurements that were never made.

    double lateralVelocity     = 0.0;  ///< [m/s] body y, positive right
    double lateralAcceleration = 0.0;  ///< [m/s^2] body y
    double sideslipAngle       = 0.0;  ///< [rad] beta
    double sideslipRate        = 0.0;  ///< [rad/s] beta_dot
    double rollAngle           = 0.0;  ///< [rad] phi, + = leaning right
    bool   wheelDataValid      = false;

    /// Per-wheel, in WheelIndex order: FL, FR, RL, RR.
    double verticalLoad[WheelCount]      = {0, 0, 0, 0};  ///< [N]
    double slipAngle[WheelCount]         = {0, 0, 0, 0};  ///< [rad]
    double camberAngle[WheelCount]       = {0, 0, 0, 0};  ///< [rad] gamma
    double slipRatio[WheelCount]         = {0, 0, 0, 0};  ///< [-] KB Eq. 10-1
    double longitudinalForce[WheelCount] = {0, 0, 0, 0};  ///< [N]
    double lateralForce[WheelCount]      = {0, 0, 0, 0};  ///< [N]
    double frictionUsage[WheelCount]     = {0, 0, 0, 0};  ///< [-]
    double wheelSpeed[WheelCount]        = {0, 0, 0, 0};  ///< [rad/s]

    //-- detailed engine/gearbox, see physics/Powertrain.h ---------------
    //
    // All zero/false under a vehicle that does not use the detailed engine
    // (the default) - `engineActive` records which case a row belongs to,
    // same convention as `wheelDataValid` above.
    bool   engineActive        = false;
    double engineRpm           = 0.0;
    double engineTorqueNm      = 0.0;
    int    engineGear          = 0;      ///< 0-based, 0 = 1st
    bool   engineLocked        = false;  ///< rigidly coupled to the wheels this sample
    bool   engineLimiterActive = false;

    // Engine air path. On the resolved engine (physics/ResolvedEngine.h)
    // these are states of the model; on the cycle-averaged one, the values
    // its formulas assume. Lambda is 1 there by construction and reads 10
    // (the model's "no fuel" marker) under a fuel cut on the resolved engine.
    double engineManifoldPressurePa = 0.0;  ///< intake manifold, absolute [Pa]
    double engineBoostPa            = 0.0;  ///< charge pressure above ambient [Pa]; 0 if NA
    double engineLambda             = 1.0;  ///< [-]
};

class Telemetry {
public:
    explicit Telemetry(const TelemetryConfig& cfg);

    /// Capture one sample. Called once per physics step by Simulation.
    /// No-op when recording is disabled.
    void record(const Vehicle& vehicle);

    void setRecording(bool on);
    bool isRecording() const { return m_recording; }
    void toggleRecording() { setRecording(!m_recording); }

    /// Discard all samples. Called on reset.
    void clear();

    std::size_t sampleCount() const { return m_samples.size(); }
    const std::deque<TelemetrySample>& samples() const { return m_samples; }

    /// Write to `<outputDirectory>/run_NNN.csv`, choosing the lowest NNN that
    /// does not already exist so previous runs are never overwritten.
    /// Returns the written path, or an empty string on failure.
    std::string exportCsv(const std::string& modelName = "");

    /// Path chosen by the most recent successful export.
    const std::string& lastExportPath() const { return m_lastExportPath; }
    const std::string& lastError() const { return m_lastError; }

    //-- rolling buffers for the real-time graphs ------------------------
    //
    // Kept separate from `m_samples` and always populated, so the graphs work
    // whether or not recording to disk is armed. Stored as float because that
    // is what ImGui::PlotLines consumes.
    struct PlotBuffers {
        std::vector<float> time;
        std::vector<float> speed;         // km/h
        std::vector<float> acceleration;  // m/s^2
        std::vector<float> steering;      // deg
        std::vector<float> yawRate;       // deg/s
    };
    const PlotBuffers& plots() const { return m_plots; }

    /// Number of samples retained for plotting.
    std::size_t plotCapacity() const { return m_plotCapacity; }
    void setPlotCapacity(std::size_t n);

    /// Always-on sampling for the graphs, independent of `record()`.
    void samplePlots(const Vehicle& vehicle);

private:
    TelemetryConfig m_config;
    bool m_recording = false;

    std::deque<TelemetrySample> m_samples;

    PlotBuffers m_plots;
    std::size_t m_plotCapacity = 1200;   // 12 s at 100 Hz

    std::string m_lastExportPath;
    std::string m_lastError;
};

} // namespace vsim
