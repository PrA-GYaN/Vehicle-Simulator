#include "audio/SoundStateBuilder.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"
#include "physics/Powertrain.h"
#include "rendering/Camera.h"
#include "vehicle/Vehicle.h"

namespace vsim {
namespace {
constexpr double kRAir = 287.058;   // Heywood Table D.1, as physics/Engine.cpp
}

EngineGeometry soundEngineGeometry(const Vehicle& vehicle) {
    EngineGeometry g;
    if (const Powertrain* pt = vehicle.powertrain()) {
        const EngineParameters& ep = pt->engine().parameters();
        g.cylinders = ep.numCylinders;
        g.stroke = ep.stroke;
        g.rodLength = ep.rodLength;
        // EngineSim's engine sound: the spec its sound_spec.py would derive,
        // from this vehicle's engine and its `sound: engine_sim:` hardware.
        if (vehicle.parameters().sound.useEngineSim && ep.useResolved) {
            g.engineSim = makeEngineSimSoundSpec(vehicle.parameters().sound.engineSim, ep);
        }
    } else {
        g.cylinders = 0;
    }
    return g;
}

SoundState buildSoundState(const Vehicle& vehicle, const Camera& camera) {
    SoundState out;
    const VehicleState& s = vehicle.state();
    const VehicleParameters& p = vehicle.parameters();

    //-- engine ---------------------------------------------------------------
    if (const Powertrain* pt = vehicle.powertrain()) {
        const Engine& eng = pt->engine();
        const EngineParameters& ep = eng.parameters();
        const Engine::TorqueBreakdown& b = eng.lastBreakdown();
        const PowertrainOutput& po = pt->lastOutput();

        out.engineActive = true;
        // What the crank is actually doing: read from the wheels when the
        // clutch is locked, integrated by the engine otherwise.
        out.crankOmega = po.engineRpm * kTwoPi / 60.0;
        out.throttle = po.limiterActive ? 0.0 : vehicle.appliedInput().throttle;
        out.blowdownPressurePa = b.exhaustBlowdownPressurePa;
        out.blowdownTemperatureK = b.exhaustBlowdownTemperatureK;
        // What the exhaust valves discharge into. The resolved engine has
        // that plenum as a state; the cycle-averaged one assumes it.
        out.exhaustManifoldPa = eng.resolved()
            ? eng.resolved()->lastOutput().exhaustPressurePa
            : ep.exhaustBackPressurePa + (ep.hasTurbo ? eng.boostPa() : 0.0);
        out.airMassPerCylinderKg = b.airMassPerCylinderKg;
        out.referenceAirMassKg = ep.ambientPressurePa / (kRAir * ep.intakeTemperatureK)
                               * ep.displacementPerCylinder();
        out.ambientPressurePa = ep.ambientPressurePa;
        out.ambientTemperatureK = ep.intakeTemperatureK;
        out.intakePressurePa = b.intakePressurePa > 0.0 ? b.intakePressurePa : ep.ambientPressurePa;
        out.chargeTemperatureK = b.chargeTemperatureK > 0.0 ? b.chargeTemperatureK : ep.intakeTemperatureK;
        out.engineTorqueNm = po.engineTorqueNm;
        out.indicatedTorqueNm = b.indicatedTorqueNm;
        out.redlineOmega = ep.redlineOmega();
        out.numCylinders = ep.numCylinders;

        // Peak WOT torque normalises the gear whine.
        out.peakTorqueNm = eng.peakTorqueEstimateNm();

        if (const ResolvedEngine* re = eng.resolved()) {
            // The fields EngineSim's dashboard sends its synthesiser
            // (app.js feedAudio), from the same engine state.
            const ResolvedEngine::Output& e = re->lastOutput();
            out.manifoldRatio = e.manifoldPressurePa / ep.ambientPressurePa;
            out.pedal = vehicle.appliedInput().throttle;
            out.combusting = e.combusting;
            out.fuelCut = e.fuelCut;
            out.starter = e.starter;
            out.grossTorqueNm = e.grossTorqueNm;
            out.frictionTorqueNm = e.frictionTorqueNm;
            out.pumpingTorqueNm = e.pumpingTorqueNm;
            out.idleOmega = ep.idleOmega();
        }

        out.turbo = ep.hasTurbo;
        out.boostPa = eng.boostPa();
        out.maxBoostPa = ep.maxBoostPa;
        out.compressorEfficiency = ep.compressorEfficiency;

        out.clutchLocked = po.locked;
        out.inGear = po.locked;
    }

    //-- vehicle --------------------------------------------------------------
    out.speed = std::hypot(s.velocity, s.lateralVelocity);
    if (s.wheelDataValid) {
        for (std::size_t i = 0; i < 4; ++i) {
            const auto& w = s.wheels[i];
            // Sliding speed at the contact patch: the longitudinal part is
            // the slip ratio times road speed (S = (V - R_e w)/V), the
            // lateral part V tan(alpha). Times the force the tyre is making
            // = the power being dissipated by sliding - what squeals.
            const double v = std::abs(s.velocity);
            const double slide = v * std::hypot(w.slipRatio, std::tan(std::clamp(w.slipAngle, -1.3, 1.3)));
            const double force = std::hypot(w.longitudinalForce, w.lateralForce);
            out.wheels[i].slipPowerW = force * slide;
            out.wheels[i].frictionUsage = w.frictionUsage;
        }
    }

    //-- listener -------------------------------------------------------------
    const glm::dvec3 source = vehicle.bodyCentrePosition() + glm::dvec3(0.0, 0.4, 0.0);
    const glm::dvec3 toSource = source - camera.positionD();
    out.distance = std::max(glm::length(toSource), 0.5);
    out.pan = (out.distance > 1.0) ? glm::dot(toSource / out.distance, camera.viewRight()) : 0.0;
    out.interior = camera.interior();

    // Is the engine behind the driver? Only knowable if the vehicle says
    // where its engine is; the default assumption is a front engine.
    if (p.sound.engine.set) {
        const double nose = p.wheelbase + 0.54 * std::max(p.length - p.wheelbase, 0.2);
        const double eyeX = p.cameras.driverEye.set ? p.cameras.driverEye.xyz[0] : nose - 0.5 * p.length;
        out.engineBehindListener = p.sound.engine.xyz[0] < eyeX;
    }
    return out;
}

} // namespace vsim
