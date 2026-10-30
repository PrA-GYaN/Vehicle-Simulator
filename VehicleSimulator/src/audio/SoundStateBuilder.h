#pragma once
//===========================================================================
//  SoundStateBuilder.h  --  read the simulation, fill a SoundState.
//
//  The only place that knows both the physics types and the audio types.
//  Pure: reads the vehicle and the camera, writes nothing.
//===========================================================================

#include "audio/SoundState.h"
#include "audio/SoundSynth.h"

namespace vsim {

class Vehicle;
class Camera;

SoundState buildSoundState(const Vehicle& vehicle, const Camera& camera);

/// The engine geometry the synthesiser needs (cylinders, stroke, rod), or
/// a default with zero cylinders for a vehicle without a detailed engine.
EngineGeometry soundEngineGeometry(const Vehicle& vehicle);

} // namespace vsim
