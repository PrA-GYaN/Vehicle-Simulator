#include "telemetry/Telemetry.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "physics/Powertrain.h"
#include "vehicle/Vehicle.h"

namespace fs = std::filesystem;

namespace vsim {
namespace {

/// Append to a rolling buffer, dropping the oldest sample when full.
///
/// std::vector + erase(begin()) is O(n) per sample, which at 100 Hz over a
/// 1200-sample window is 120k element moves per second. Fine here, and it
/// keeps the memory contiguous for ImGui::PlotLines, which needs a plain
/// float array. A deque would avoid the copy but could not be plotted
/// directly.
void push(std::vector<float>& buf, float v, std::size_t cap) {
    if (cap == 0) return;
    if (buf.size() >= cap) {
        buf.erase(buf.begin(), buf.begin() + static_cast<long>(buf.size() - cap + 1));
    }
    buf.push_back(v);
}

} // namespace

Telemetry::Telemetry(const TelemetryConfig& cfg) : m_config(cfg) {
    m_recording = cfg.enabledAtStart;
    if (m_config.maxSamples == 0) m_config.maxSamples = 1;
}

void Telemetry::setRecording(bool on) {
    if (on == m_recording) return;
    m_recording = on;
    // Starting a new recording begins a new trace. Keeping the old samples
    // would silently splice two runs into one file.
    if (on) m_samples.clear();
}

void Telemetry::clear() {
    m_samples.clear();
    m_plots = PlotBuffers{};
}

void Telemetry::setPlotCapacity(std::size_t n) {
    m_plotCapacity = (n < 10) ? 10 : ((n > 100000) ? 100000 : n);
}

void Telemetry::record(const Vehicle& vehicle) {
    if (!m_recording) return;

    const VehicleState& s = vehicle.state();
    const ControlInput& c = vehicle.appliedInput();

    TelemetrySample sample;
    sample.timestamp     = s.time;
    sample.x             = s.position.x;
    sample.y             = s.position.y;
    sample.z             = s.position.z;
    sample.yaw           = s.yaw;
    sample.velocity      = s.velocity;
    sample.acceleration  = s.acceleration;
    sample.steeringAngle = s.steeringAngle;
    sample.yawRate       = s.yawRate;
    sample.throttle      = c.throttle;
    sample.brake         = c.brake;

    sample.lateralVelocity     = s.lateralVelocity;
    sample.lateralAcceleration = s.lateralAcceleration;
    sample.sideslipAngle       = s.sideslipAngle;
    sample.sideslipRate        = s.sideslipRate;
    sample.rollAngle           = s.rollAngle;
    sample.wheelDataValid      = s.wheelDataValid;

    for (int i = 0; i < WheelCount; ++i) {
        const WheelDynamics& w = s.wheels[i];
        sample.verticalLoad[i]      = w.verticalLoad;
        sample.slipAngle[i]         = w.slipAngle;
        sample.camberAngle[i]       = w.camberAngle;
        sample.slipRatio[i]         = w.slipRatio;
        sample.longitudinalForce[i] = w.longitudinalForce;
        sample.lateralForce[i]      = w.lateralForce;
        sample.frictionUsage[i]     = w.frictionUsage;
        sample.wheelSpeed[i]        = w.angularVelocity;
    }

    if (const Powertrain* pt = vehicle.powertrain()) {
        const PowertrainOutput& po = pt->lastOutput();
        sample.engineActive        = true;
        sample.engineRpm           = po.engineRpm;
        sample.engineTorqueNm      = po.engineTorqueNm;
        sample.engineGear          = po.currentGear;
        sample.engineLocked        = po.locked;
        sample.engineLimiterActive = po.limiterActive;

        const Engine& engine = pt->engine();
        sample.engineManifoldPressurePa = engine.lastBreakdown().intakePressurePa;
        sample.engineBoostPa            = engine.boostPa();
        if (const ResolvedEngine* resolved = engine.resolved()) {
            sample.engineLambda = resolved->lastOutput().lambda;
        }
    }

    m_samples.push_back(sample);

    // Bounded memory: an unbounded log would eat gigabytes over a long
    // session. Oldest samples are dropped first.
    while (m_samples.size() > m_config.maxSamples) m_samples.pop_front();
}

void Telemetry::samplePlots(const Vehicle& vehicle) {
    const VehicleState& s = vehicle.state();
    push(m_plots.time,         static_cast<float>(s.time),                    m_plotCapacity);
    push(m_plots.speed,        static_cast<float>(mpsToKph(s.velocity)),      m_plotCapacity);
    push(m_plots.acceleration, static_cast<float>(s.acceleration),            m_plotCapacity);
    push(m_plots.steering,     static_cast<float>(radToDeg(s.steeringAngle)), m_plotCapacity);
    push(m_plots.yawRate,      static_cast<float>(radToDeg(s.yawRate)),       m_plotCapacity);
}

std::string Telemetry::exportCsv(const std::string& modelName) {
    m_lastError.clear();

    if (m_samples.empty()) {
        m_lastError = "no samples recorded";
        return {};
    }

    const std::string dir = resolveResourcePath(m_config.outputDirectory);

    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec && !fs::is_directory(dir, ec)) {
        m_lastError = "cannot create directory '" + dir + "'";
        std::cerr << "[telemetry] " << m_lastError << "\n";
        return {};
    }

    // Find the lowest unused run number so nothing is ever overwritten.
    fs::path chosen;
    for (int n = 1; n <= 9999; ++n) {
        std::ostringstream name;
        name << "run_" << std::setw(3) << std::setfill('0') << n << ".csv";
        fs::path candidate = fs::path(dir) / name.str();
        if (!fs::exists(candidate, ec)) { chosen = candidate; break; }
    }
    if (chosen.empty()) {
        m_lastError = "all run numbers 001-9999 are taken";
        return {};
    }

    std::ofstream out(chosen);
    if (!out) {
        m_lastError = "cannot open '" + chosen.string() + "' for writing";
        std::cerr << "[telemetry] " << m_lastError << "\n";
        return {};
    }

    // Comment header. '#' is not part of CSV, but every tool that matters
    // (pandas, numpy, gnuplot, Excel-with-a-nudge) can skip it, and recording
    // which physics model produced a trace is worth the small friction.
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    out << "# VehicleSimulator telemetry\n";
    out << "# generated: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << "\n";
    if (!modelName.empty()) out << "# physics_model: " << modelName << "\n";
    out << "# units: s, m, m, m, rad, m/s, m/s^2, rad, rad/s, -, -,\n";
    out << "#        m/s, m/s^2, rad, rad/s, rad, bool, then per wheel FL,FR,RL,RR:\n";
    out << "#        N, rad, rad, -, N, N, -, rad/s\n";
    out << "# slip_ratio is the Knowledge Base's braked-wheel form "
           "S = (V - Re*w)/V: 0 free rolling, +1 locked, negative under drive\n";

    out << "timestamp,x,y,z,yaw,velocity,acceleration,steering_angle,"
           "yaw_rate,throttle,brake,"
           "lateral_velocity,lateral_acceleration,sideslip,sideslip_rate,"
           "roll_angle,wheel_data_valid";

    static const char* kW[WheelCount] = {"fl", "fr", "rl", "rr"};
    for (const char* q : {"fz", "slip_angle", "camber", "slip_ratio", "fx", "fy",
                          "friction_usage", "wheel_speed"}) {
        for (int i = 0; i < WheelCount; ++i) out << ',' << q << '_' << kW[i];
    }

    // Detailed engine/gearbox (physics/Powertrain.h) - all zero/false, and
    // engine_active=0, on a vehicle that does not use it (the default).
    out << ",engine_active,engine_rpm,engine_torque_nm,engine_gear,"
           "engine_locked,engine_limiter_active,"
           "engine_map_pa,engine_boost_pa,engine_lambda";
    out << '\n';

    // Enough digits to round-trip a double exactly (17 significant figures).
    // Telemetry that cannot reproduce the run it came from is not telemetry.
    out << std::setprecision(17);

    for (const TelemetrySample& s : m_samples) {
        out << s.timestamp << ',' << s.x << ',' << s.y << ',' << s.z << ','
            << s.yaw << ',' << s.velocity << ',' << s.acceleration << ','
            << s.steeringAngle << ',' << s.yawRate << ',' << s.throttle << ','
            << s.brake << ','
            << s.lateralVelocity << ',' << s.lateralAcceleration << ','
            << s.sideslipAngle << ',' << s.sideslipRate << ','
            << s.rollAngle << ',' << (s.wheelDataValid ? 1 : 0);

        for (const double* q : {s.verticalLoad, s.slipAngle, s.camberAngle, s.slipRatio,
                                s.longitudinalForce, s.lateralForce,
                                s.frictionUsage, s.wheelSpeed}) {
            for (int i = 0; i < WheelCount; ++i) out << ',' << q[i];
        }

        out << ',' << (s.engineActive ? 1 : 0) << ',' << s.engineRpm << ','
            << s.engineTorqueNm << ',' << s.engineGear << ','
            << (s.engineLocked ? 1 : 0) << ',' << (s.engineLimiterActive ? 1 : 0) << ','
            << s.engineManifoldPressurePa << ',' << s.engineBoostPa << ',' << s.engineLambda;
        out << '\n';
    }

    out.flush();
    if (!out) {
        m_lastError = "write failed (disk full?)";
        return {};
    }

    m_lastExportPath = chosen.string();
    std::cout << "[telemetry] wrote " << m_samples.size() << " samples to "
              << m_lastExportPath << "\n";
    return m_lastExportPath;
}

} // namespace vsim
