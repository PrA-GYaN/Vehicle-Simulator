#pragma once
//===========================================================================
//  SoundState.h  --  everything the synthesiser needs to know about one
//  instant of the simulation, as plain numbers.
//
//  Built from the simulation by audio/SoundStateBuilder (once per rendered
//  frame, or once per physics step when rendering a WAV headless), handed
//  across to the audio thread, and interpolated there sample by sample.
//  Nothing in here is invented for the sound: every field is a quantity the
//  physics already computed, or pure geometry between the car and the
//  camera. See audio/SoundSynth.h for what each one turns into.
//===========================================================================

#include <array>

namespace vsim {

struct SoundState {
    //-- engine (all zero / false on a vehicle without the detailed engine) --
    bool   engineActive = false;
    double crankOmega = 0.0;            ///< [rad/s]
    double throttle = 0.0;              ///< [0,1] what the engine actually sees
    double blowdownPressurePa = 0.0;    ///< p_4 at exhaust valve opening, absolute
    double blowdownTemperatureK = 0.0;  ///< T_4
    double exhaustManifoldPa = 101325.0;///< what the blowdown pulse discharges into
    double airMassPerCylinderKg = 0.0;
    double referenceAirMassKg = 1e-4;   ///< rho_amb * V_d per cylinder - the "full" charge
    double ambientPressurePa = 101325.0;
    double ambientTemperatureK = 293.15;
    double intakePressurePa = 101325.0;
    double chargeTemperatureK = 293.15;
    double engineTorqueNm = 0.0;
    double indicatedTorqueNm = 0.0;     ///< what combustion delivers, before losses
    double peakTorqueNm = 1.0;          ///< normalises the gear whine
    double redlineOmega = 700.0;
    int    numCylinders = 4;

    //-- resolved engine: what EngineSim's synthesiser is driven by ----------
    //   (audio/EngineSimSynth.h; all unused without `sound: engine_sim:`)
    double manifoldRatio = 1.0;         ///< intake manifold pressure / ambient
    double pedal = 0.0;                 ///< [0,1] the throttle itself, limiter or not
    bool   combusting = false;          ///< firing and fuelled
    bool   fuelCut = false;             ///< overrun cut or rev limiter
    bool   starter = false;             ///< starter motor engaged
    double grossTorqueNm = 0.0;         ///< gross indicated
    double frictionTorqueNm = 0.0;
    double pumpingTorqueNm = 0.0;
    double idleOmega = 80.0;

    //-- turbo --
    bool   turbo = false;
    double boostPa = 0.0;               ///< gauge
    double maxBoostPa = 1.0;
    double compressorEfficiency = 0.7;

    //-- driveline --
    bool   clutchLocked = false;
    bool   inGear = false;

    //-- vehicle --
    double speed = 0.0;                 ///< [m/s] magnitude
    struct Wheel {
        double slipPowerW = 0.0;        ///< |F| x sliding speed at the contact patch
        double frictionUsage = 0.0;     ///< [0,1] share of the friction ellipse in use
    };
    std::array<Wheel, 4> wheels{};

    //-- listener (the camera) --
    double distance = 8.0;              ///< camera to the car's sound source [m]
    double pan = 0.0;                   ///< [-1,1] left..right in the camera's view
    bool   interior = false;            ///< camera inside the cabin
    bool   engineBehindListener = false;///< mid/rear engine seen from the cockpit
};

} // namespace vsim
