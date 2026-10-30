//===========================================================================
//  lap_test  --  can a circuit actually be driven?
//
//  Drives one flying lap of a track with a simple autopilot through the real
//  simulation (the same Simulation, Vehicle and dynamics model the windowed
//  program runs - no window, no renderer) and reports whether the car made
//  it round on the road.
//
//  This is a test of the TRACK, not a lap-time tool. The driver is a pure
//  pursuit steering law and a speed target from the curvature ahead, both
//  deliberately conservative: a layout whose corners connect, whose
//  curvature is continuous enough to follow and whose start meets its
//  finish is one this driver gets round without leaving the tarmac. A lap
//  that needs more than that is a lap with something wrong in it.
//
//      lap_test [--track <name>] [--vehicle <name>] [--lateral-g <g>] [--verbose]
//               [--tc <0-3>] [--tyres standard|soft|medium|hard] [--launch]
//
//  Exit status 0 if the lap was completed within the track limits.
//===========================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "config/Config.h"
#include "core/Coordinates.h"
#include "core/Units.h"
#include "environment/Track.h"
#include "simulation/Simulation.h"

using namespace vsim;

int main(int argc, char** argv) {
    setExecutablePath(argv[0]);

    std::string trackName = "apex_gp";
    std::string vehicleName;
    double lateralG = 0.32;
    bool verbose = false;
    bool launch = false;
    int tcLevel = -1;                    // -1: whatever the vehicle file says
    std::string tyres;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--verbose")) verbose = true;
        if (!std::strcmp(argv[i], "--launch")) launch = true;
    }
    for (int i = 1; i + 1 < argc; ++i) {
        if (!std::strcmp(argv[i], "--track")) trackName = argv[i + 1];
        else if (!std::strcmp(argv[i], "--vehicle")) vehicleName = argv[i + 1];
        else if (!std::strcmp(argv[i], "--lateral-g")) lateralG = std::atof(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--tc")) tcLevel = std::atoi(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--tyres")) tyres = argv[i + 1];
    }

    AppConfig config = loadAppConfig(resolveResourcePath("config/simulation.yaml"));
    if (vehicleName.empty()) vehicleName = config.defaultVehicle;
    const VehicleParameters params =
        loadVehicleParameters(resolveResourcePath("config/vehicles/" + vehicleName + ".yaml"));

    const Track track = Track::createByName(trackName);
    const auto& pts = track.centreline();
    if (!track.closed() || pts.size() < 50) {
        std::fprintf(stderr, "lap_test: '%s' is not a closed circuit\n", trackName.c_str());
        return 2;
    }
    const int n = static_cast<int>(pts.size());
    const double lapLength = pts.back().distance + glm::length(pts.front().position - pts.back().position);
    const double spacing = lapLength / n;

    config.initialState.position = pts.front().position;
    config.initialState.position.y = 0.0;
    config.initialState.yaw = coords::yawFromDirection(pts.front().tangent);
    config.initialState.velocity = 0.0;
    config.simulation.realtime = true;
    config.simulation.timeScale = 1.0;

    Simulation sim(config, params);
    const double dt = config.simulation.timestep;
    if (tcLevel >= 0) sim.vehicle().setTractionControl(tcLevel);
    if (!tyres.empty()) sim.vehicle().setTireCompound(tireCompoundFromString(tyres));

    //-- --launch: a standing start, flat out, in a straight line ------------
    //
    // What traction control and the tyre compound do to the one manoeuvre
    // that isolates them: time to 100 km/h, and how far past the road speed
    // the driven wheels ran on the way.
    if (launch) {
        ControlInput flat;
        flat.throttle = 1.0;
        double t100 = -1.0, peakSlip = 0.0, slipTime = 0.0, peakCut = 0.0;
        const double peak = params.tire.peakSlipRatio;
        for (long step = 0; step < static_cast<long>(12.0 / dt); ++step) {
            sim.advance(dt, flat);
            const VehicleState& s = sim.vehicle().state();
            double slip = 0.0;
            for (int i = 0; i < WheelCount; ++i) {
                if (!params.isDrivenWheel(i)) continue;
                // Same measure the traction control uses - see Vehicle::step.
                slip = std::max(slip, (s.wheels[i].effectiveRollingRadius * s.wheels[i].angularVelocity
                                       - s.velocity) / std::max(std::abs(s.velocity), 5.0));
            }
            if (sim.time() > 0.3) peakSlip = std::max(peakSlip, slip);
            if (slip > peak) slipTime += dt;
            peakCut = std::max(peakCut, sim.vehicle().tractionControlCut());
            if (t100 < 0.0 && s.speed() >= 100.0 / 3.6) t100 = sim.time();
        }
        std::printf("launch: %-12s tyres %-8s TC %d   0-100 km/h %5.2f s   peak slip %4.2f"
                    "   past-peak %4.2f s   max cut %3.0f %%   %.0f km/h after 12 s\n",
                    params.name.c_str(), tireCompoundName(sim.vehicle().tireCompound()),
                    sim.vehicle().tractionControl(), t100, peakSlip, slipTime, peakCut * 100.0,
                    sim.vehicle().state().speed() * 3.6);
        return 0;
    }

    const double aLat = lateralG * kGravity;
    const double aBrake = 3.5;          // [m/s^2] what the speed plan assumes it can shed
    const double vMax = 62.0;           // [m/s]

    int nearest = 0;
    double travelled = 0.0;             // centreline distance covered
    double maxOffset = 0.0, maxOffsetAt = 0.0, maxSpeed = 0.0, minSpeed = 1e9;
    double lapStartTime = -1.0;
    bool finished = false;

    // Two laps at most: the first from a standing start gets the car up to
    // speed, the timed one starts when it next crosses the start point.
    const long maxSteps = static_cast<long>(900.0 / dt);
    for (long step = 0; step < maxSteps; ++step) {
        const VehicleState& s = sim.vehicle().state();

        // Nearest centreline point, searched forward from the last one.
        double best = 1e30;
        int bestI = nearest;
        for (int k = -5; k <= 60; ++k) {
            const int i = ((nearest + k) % n + n) % n;
            const glm::dvec3 d = pts[static_cast<std::size_t>(i)].position - s.position;
            const double dd = d.x * d.x + d.z * d.z;
            if (dd < best) { best = dd; bestI = i; }
        }
        int advanced = bestI - nearest;
        if (advanced < -n / 2) advanced += n;
        if (advanced > 0) travelled += advanced * spacing;
        const bool crossedStart = bestI < nearest && advanced > 0;
        nearest = bestI;

        const TrackPoint& here = pts[static_cast<std::size_t>(nearest)];
        const glm::dvec3 right = coords::right(coords::yawFromDirection(here.tangent));
        const double offset = glm::dot(s.position - here.position, right);

        if (crossedStart) {
            if (lapStartTime < 0.0) {
                lapStartTime = sim.time();
                maxOffset = 0.0; maxSpeed = 0.0; minSpeed = 1e9;
            } else {
                finished = true;
                break;
            }
        }
        if (lapStartTime >= 0.0) {
            if (std::abs(offset) > maxOffset) { maxOffset = std::abs(offset); maxOffsetAt = here.distance; }
            maxSpeed = std::max(maxSpeed, s.speed());
            minSpeed = std::min(minSpeed, s.speed());
        }
        if (std::sqrt(best) > 60.0) {
            std::printf("lap_test: car left the circuit at %.0f m (%.1f m from the centreline)\n",
                        here.distance, std::sqrt(best));
            return 1;
        }

        //-- speed plan: the fastest speed here from which every corner ahead
        //   can still be reached at its own limit under aBrake --------------
        const double v = s.speed();
        double target = vMax;
        const int horizon = static_cast<int>((v * v / (2.0 * aBrake) + 40.0) / spacing);
        for (int k = 0; k <= horizon; ++k) {
            const TrackPoint& p = pts[static_cast<std::size_t>((nearest + k) % n)];
            const double kappa = std::max(std::abs(p.curvature), 1e-5);
            const double vCorner = std::sqrt(aLat / kappa);
            target = std::min(target, std::sqrt(vCorner * vCorner + 2.0 * aBrake * k * spacing));
        }

        //-- pure pursuit on a point ahead on the centreline ------------------
        const double lookahead = 7.0 + 0.45 * v;
        const TrackPoint& aim =
            pts[static_cast<std::size_t>((nearest + static_cast<int>(lookahead / spacing)) % n)];
        const glm::dvec3 toAim = aim.position - s.position;
        const double alpha = coords::wrapAngle(std::atan2(toAim.z, toAim.x) - s.yaw);
        const double dist = std::max(std::sqrt(toAim.x * toAim.x + toAim.z * toAim.z), 1.0);
        const double steer = std::atan(2.0 * params.wheelbase * std::sin(alpha) / dist);

        ControlInput in;
        in.steering = clampd(steer / std::max(params.maxSteeringAngle, 0.05), -1.0, 1.0);
        const double err = target - v;
        // Grip spent cornering is not available for driving: a driver who
        // flattens the throttle mid-corner spins a rear-drive car, which is
        // a fact about the driver and says nothing about the track.
        const double latUse = clampd(v * v * std::abs(here.curvature) / aLat, 0.0, 1.0);
        const double steerUse = clampd(std::abs(in.steering) * 2.5, 0.0, 1.0);
        const double grip = 1.0 - 0.85 * std::max(latUse, steerUse);
        in.throttle = clampd(err * 0.25, 0.0, 1.0) * grip;
        in.brake    = clampd(-err * 0.35, 0.0, 1.0);

        if (verbose && step % static_cast<long>(0.5 / dt) == 0) {
            std::printf("  t=%6.1f s=%6.0f v=%5.1f target=%5.1f offset=%6.2f steer=%5.2f thr=%4.2f brk=%4.2f\n",
                        sim.time(), here.distance, v, target, offset, in.steering, in.throttle, in.brake);
        }
        sim.advance(dt, in);
    }

    if (!finished) {
        std::printf("lap_test: no complete lap within the time limit (covered %.0f m)\n", travelled);
        return 1;
    }

    const double lapTime = sim.time() - lapStartTime;
    const double halfWidth = track.width() * 0.5;
    const bool onRoad = maxOffset < halfWidth;
    std::printf("lap_test: track '%s' (%.0f m), vehicle '%s'\n", trackName.c_str(), lapLength, params.name.c_str());
    std::printf("  flying lap        %.1f s  (average %.0f km/h)\n", lapTime, lapLength / lapTime * 3.6);
    std::printf("  speed range       %.0f - %.0f km/h\n", minSpeed * 3.6, maxSpeed * 3.6);
    std::printf("  furthest off line %.2f m at %.0f m  (half width %.1f m)  -> %s\n",
                maxOffset, maxOffsetAt, halfWidth, onRoad ? "stayed on the road" : "LEFT THE ROAD");
    if (sim.vehicle().tireCompound() != TireCompound::Standard) {
        const auto& w = sim.vehicle().state().wheels;
        std::printf("  tyres             %s; wear after two laps  FL %.1f%%  FR %.1f%%  RL %.1f%%  RR %.1f%%\n",
                    tireCompoundName(sim.vehicle().tireCompound()), w[FrontLeft].wear * 100.0,
                    w[FrontRight].wear * 100.0, w[RearLeft].wear * 100.0, w[RearRight].wear * 100.0);
    }
    return onRoad ? 0 : 1;
}
