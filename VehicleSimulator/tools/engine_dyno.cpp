//===========================================================================
//  engine_dyno  --  put a vehicle's detailed engine on a dynamometer.
//
//      engine_dyno <vehicle | file.yaml> [--points N] [--throttle X] [--csv]
//
//  Holds the engine at N speeds from idle to redline and prints the settled
//  flywheel torque at each, with the quantities that explain it (manifold
//  pressure, boost, exhaust back-pressure, lambda, volumetric efficiency).
//  This is how the engine blocks in config/vehicles/*.yaml are calibrated and
//  where the torque/power figures quoted in the README come from.
//
//      engine_dyno <vehicle | file.yaml> --transient RPM
//
//  holds the engine at RPM with the throttle shut, then opens it fully and
//  prints the response every 0.1 s - how long the manifold takes to fill and,
//  on a turbo engine, how long the turbos take to arrive (turbo lag is an
//  output of the resolved core, so this is the only way to know it).
//
//  `<vehicle>` is a name under config/vehicles/ (ferrari_f40) or a path to
//  any YAML with a `detailed:` engine block - including one written by
//  tools/engine_toml_to_yaml.py from an EngineSim engine file.
//
//  Offline tool, like extract_track_centreline: not on any startup path.
//  The simulator's Powertrain panel runs the same sweep (physics/EngineDyno.h)
//  a few milliseconds per frame.
//===========================================================================

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include "config/Config.h"
#include "core/Units.h"
#include "physics/EngineDyno.h"
#include "physics/ResolvedEngine.h"

int main(int argc, char** argv) {
    vsim::setExecutablePath(argc > 0 ? argv[0] : nullptr);

    std::string target;
    int points = 16;
    double throttle = 1.0;
    bool csv = false;
    double transientRpm = 0.0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--transient") == 0 && i + 1 < argc) { transientRpm = std::atof(argv[++i]); continue; }
        if (std::strcmp(argv[i], "--points") == 0 && i + 1 < argc)        points = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--throttle") == 0 && i + 1 < argc) throttle = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--csv") == 0)                      csv = true;
        else if (argv[i][0] != '-' && target.empty())                     target = argv[i];
        else {
            std::fprintf(stderr, "usage: engine_dyno <vehicle | file.yaml> [--points N] "
                                 "[--throttle X] [--csv] | --transient RPM\n");
            return 2;
        }
    }
    if (target.empty()) {
        std::fprintf(stderr, "usage: engine_dyno <vehicle | file.yaml> [--points N] "
                             "[--throttle X] [--csv]\n");
        return 2;
    }
    const bool isPath = target.find('/') != std::string::npos
                     || (target.size() > 5 && target.compare(target.size() - 5, 5, ".yaml") == 0);
    const std::string path = isPath ? target : "config/vehicles/" + target + ".yaml";

    vsim::EngineParameters params;
    if (!vsim::loadEngineParameters(path, params)) return 1;

    if (transientRpm > 0.0) {
        if (!params.useResolved) {
            std::fprintf(stderr, "engine_dyno: --transient needs a resolved engine (the "
                                 "cycle-averaged model's response is its authored time constants)\n");
            return 1;
        }
        vsim::ResolvedEngine engine(params);
        engine.start();
        const double omega = transientRpm * vsim::kTwoPi / 60.0;
        const double dt = params.resolved.engineTimestepS;
        vsim::ResolvedEngine::Input in;
        std::printf("%s: throttle snapped open at %.0f rpm\n\n", path.c_str(), transientRpm);
        std::printf("   time  torque     MAP   boost  turbo speed\n");
        std::printf("    [s]   [N.m]   [kPa]   [kPa]      [krpm]\n");
        double work = 0.0, angle = 0.0, nextPrint = 0.0;
        for (double t = -2.0; t < 6.0 + 0.5 * dt; t += dt) {
            in.throttle = (t >= 0.0) ? 1.0 : 0.0;
            in.starter = !engine.lastOutput().firing;
            const vsim::ResolvedEngine::Output& o = engine.updateAtSpeed(omega, in, dt);
            work += o.brakeTorqueNm * omega * dt;
            angle += omega * dt;
            if (t >= nextPrint - 0.5 * dt) {
                std::printf("  %5.1f  %6.1f  %6.1f  %6.1f      %6.1f\n", t, work / angle,
                            o.mapFilteredPa / 1e3,
                            (o.boostPressurePa - params.ambientPressurePa) / 1e3,
                            o.turboSpeedRps * 60.0 / 1000.0);
                nextPrint += (t < 1.0 - 0.5 * dt) ? 0.1 : 0.5;
                work = angle = 0.0;
            }
        }
        return 0;
    }

    const std::clock_t cpu0 = std::clock();
    vsim::EngineDyno dyno(params, points, throttle);
    dyno.runToCompletion();
    const double cpuS = double(std::clock() - cpu0) / CLOCKS_PER_SEC;

    if (csv) {
        std::printf("rpm,torque_nm,power_kw,bmep_bar,map_kpa,boost_kpa,exhaust_kpa,lambda,"
                    "volumetric_efficiency,spark_deg_btdc,settle_s,converged\n");
        for (const auto& p : dyno.points()) {
            std::printf("%.0f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%d\n",
                        p.rpm, p.torqueNm, p.powerKw, p.bmepBar, p.manifoldPressurePa / 1e3,
                        p.boostPa / 1e3, p.exhaustPressurePa / 1e3, p.lambda,
                        p.volumetricEfficiency, p.sparkDegBtdc, p.settleTimeS, p.converged ? 1 : 0);
        }
        return 0;
    }

    std::printf("%s\n", path.c_str());
    std::printf("  %d cyl, %.0f cc, CR %.1f, %s, throttle %.0f %%\n",
                params.numCylinders, params.totalDisplacement() * 1e6, params.compressionRatio,
                params.useResolved ? (params.resolved.useTurbo ? "resolved core, turbocharged"
                                                               : "resolved core, naturally aspirated")
                                   : "cycle-averaged model",
                throttle * 100.0);
    std::printf("\n    rpm   torque    power    bmep     MAP   boost  exh.p  lambda    VE  spark  settle\n");
    std::printf("          [N.m]     [kW]   [bar]   [kPa]   [kPa]  [kPa]     [-]   [-]  [deg]     [s]\n");
    double peakTorque = 0.0, peakTorqueRpm = 0.0, peakPower = 0.0, peakPowerRpm = 0.0;
    for (const auto& p : dyno.points()) {
        std::printf("  %5.0f  %7.1f  %7.1f  %6.2f  %6.1f  %6.1f  %5.1f  %6.3f  %4.2f  %5.1f  %6.2f%s\n",
                    p.rpm, p.torqueNm, p.powerKw, p.bmepBar, p.manifoldPressurePa / 1e3,
                    p.boostPa / 1e3, p.exhaustPressurePa / 1e3, p.lambda, p.volumetricEfficiency,
                    p.sparkDegBtdc, p.settleTimeS, p.converged ? "" : "  (not settled)");
        if (p.torqueNm > peakTorque) { peakTorque = p.torqueNm; peakTorqueRpm = p.rpm; }
        if (p.powerKw > peakPower)   { peakPower = p.powerKw;   peakPowerRpm = p.rpm; }
    }
    std::printf("\n  peak torque %.0f N.m at %.0f rpm,  peak power %.0f kW (%.0f PS) at %.0f rpm\n",
                peakTorque, peakTorqueRpm, peakPower, peakPower * 1.35962, peakPowerRpm);
    if (params.useResolved && cpuS > 0.0) {
        std::printf("  %.1f s of engine time in %.2f s of CPU: %.1fx real time\n",
                    dyno.simulatedTimeS(), cpuS, dyno.simulatedTimeS() / cpuS);
    }
    return 0;
}
