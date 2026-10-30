//===========================================================================
//  engine_parity  --  check physics/ResolvedEngine against EngineSim itself.
//
//      python tools/engine_parity_reference.py 1nz --out /tmp/p
//      engine_parity /tmp/p.yaml /tmp/p.csv [--tolerance 1e-6]
//
//  ResolvedEngine.cpp is a transcription of EngineSim's Python engine around
//  EngineSim's own C cylinder core. This replays a run recorded from
//  EngineSim (tools/engine_parity_reference.py) through the C++ engine and
//  compares every output of every step. The two share the compiled core and
//  the same arithmetic, so they should agree to rounding; anything more is a
//  transcription error, or EngineSim has moved on and the port has not.
//
//  Exit status 0 = every column within tolerance. Prints the worst relative
//  difference per column and where it first went wrong.
//
//  Offline tool; run it after tools/sync_enginecore.py or any edit to
//  ResolvedEngine.cpp. See README, "Keeping the port in step with EngineSim".
//===========================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "config/Config.h"
#include "physics/ResolvedEngine.h"

namespace {

struct Column {
    const char* name;
    double scale;        ///< differences are judged relative to max(|reference|, scale)
    double worst = 0.0;
    long   worstRow = -1;
    double worstRef = 0.0, worstGot = 0.0;
};

} // namespace

int main(int argc, char** argv) {
    vsim::setExecutablePath(argc > 0 ? argv[0] : nullptr);
    if (argc < 3) {
        std::fprintf(stderr, "usage: engine_parity <engine.yaml> <reference.csv> [--tolerance X]\n");
        return 2;
    }
    double tolerance = 1e-6;
    for (int i = 3; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--tolerance") == 0) tolerance = std::atof(argv[i + 1]);
    }

    vsim::EngineParameters params;
    if (!vsim::loadEngineParameters(argv[1], params)) return 2;
    if (!params.useResolved) {
        std::fprintf(stderr, "engine_parity: '%s' has no `resolved:` block\n", argv[1]);
        return 2;
    }

    std::ifstream in(argv[2]);
    if (!in) {
        std::fprintf(stderr, "engine_parity: cannot read '%s'\n", argv[2]);
        return 2;
    }
    std::string line;
    std::getline(in, line);     // header

    // Output columns, in the order tools/engine_parity_reference.py writes
    // them after the seven input columns.
    std::vector<Column> cols = {
        {"rpm", 1.0}, {"net_torque", 1.0}, {"gross_torque", 1.0}, {"pumping_torque", 1.0},
        {"friction_torque", 1.0}, {"manifold_pressure", 100.0}, {"lambda", 0.01},
        {"spark", 0.1}, {"burn", 0.1}, {"iac", 1e-4}, {"exhaust_pressure", 100.0},
        {"exhaust_temperature", 1.0}, {"boost_pressure", 100.0}, {"turbo_rps", 1.0},
        {"wastegate", 1e-3}, {"ve", 1e-3}, {"air_per_cycle", 1e-6}, {"fuel_per_cycle", 1e-7},
        {"crank_angle", 1.0}, {"imep_net_bar", 0.01}, {"peak_pressure", 1000.0},
    };

    vsim::ResolvedEngine engine(params);
    long row = 0;
    long firstBadRow = -1;
    std::string firstBad;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<double> v;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) v.push_back(std::strtod(cell.c_str(), nullptr));
        if (v.size() != 7 + cols.size()) {
            std::fprintf(stderr, "engine_parity: row %ld has %zu columns, expected %zu\n",
                         row, v.size(), 7 + cols.size());
            return 2;
        }
        const int cmd = static_cast<int>(v[0]);
        const bool imposed = v[1] != 0.0;
        const double dt = v[2];
        vsim::ResolvedEngine::Input input;
        input.throttle = v[3];
        input.loadTorqueNm = v[4];
        input.starter = v[5] != 0.0;

        if (cmd == 1) engine.start();
        else if (cmd == 2) engine.reset();
        const vsim::ResolvedEngine::Output& o = imposed ? engine.updateAtSpeed(v[6], input, dt)
                                                        : engine.update(input, dt);
        const double got[] = {
            o.rpm, o.netTorqueNm, o.grossTorqueNm, o.pumpingTorqueNm, o.frictionTorqueNm,
            o.manifoldPressurePa, o.lambda, o.sparkTimingDegBtdc, o.combustionDurationDeg,
            o.iacPosition, o.exhaustPressurePa, o.exhaustTemperatureK, o.boostPressurePa,
            o.turboSpeedRps, o.wastegatePosition, o.volumetricEfficiency, o.airEstimateKg,
            o.fuelPerCycleKg, o.crankAngleDeg, o.imepNetBar, o.peakPressurePa,
        };
        for (std::size_t c = 0; c < cols.size(); ++c) {
            // At an imposed speed EngineSim still reports the speed it would
            // have integrated to; the port reports the speed it was given.
            if (imposed && c == 0) continue;
            const double ref = v[7 + c];
            const double err = std::abs(got[c] - ref) / std::max(std::abs(ref), cols[c].scale);
            if (err > tolerance && (firstBadRow < 0 || firstBadRow == row)) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "\n    %-20s EngineSim %.12g, port %.12g",
                              cols[c].name, ref, got[c]);
                firstBadRow = row;
                firstBad += buf;
            }
            if (err > cols[c].worst) {
                cols[c].worst = err;
                cols[c].worstRow = row;
                cols[c].worstRef = ref;
                cols[c].worstGot = got[c];
            }
        }
        ++row;
    }

    bool ok = true;
    std::printf("engine_parity: %ld steps, %d cylinders, tolerance %.1e\n",
                row, engine.cylinderCount(), tolerance);
    for (const Column& c : cols) {
        const bool bad = c.worst > tolerance;
        ok = ok && !bad;
        if (c.worstRow < 0) {
            std::printf("  %-20s exact\n", c.name);
        } else {
            std::printf("  %-20s worst %.2e at step %ld (EngineSim %.9g, port %.9g)%s\n",
                        c.name, c.worst, c.worstRow, c.worstRef, c.worstGot, bad ? "   <-- FAIL" : "");
        }
    }
    if (firstBadRow >= 0) {
        std::printf("  first step out of tolerance: %ld%s\n", firstBadRow, firstBad.c_str());
    }
    std::printf(ok ? "engine_parity: PASS\n" : "engine_parity: FAIL\n");
    return ok ? 0 : 1;
}
