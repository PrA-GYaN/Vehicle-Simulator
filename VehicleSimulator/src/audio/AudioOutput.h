#pragma once
//===========================================================================
//  AudioOutput.h  --  the live audio device.
//
//  A thin shell around one miniaudio playback device: the device thread
//  calls back for samples, and the callback asks SoundSynth for them. All
//  of the sound itself is in audio/SoundSynth.h; this file only moves state
//  in (from the main thread, once a frame) and samples out (on the audio
//  thread), under one mutex.
//
//  Failing to open a device (no sound server, headless box, container) is
//  not an error the simulator should stop for: start() returns false, logs
//  why, and every other call becomes a no-op.
//===========================================================================

#include <functional>
#include <memory>

#include "audio/SoundState.h"
#include "audio/SoundSynth.h"

namespace vsim {

class AudioOutput {
public:
    AudioOutput();
    ~AudioOutput();
    AudioOutput(const AudioOutput&) = delete;
    AudioOutput& operator=(const AudioOutput&) = delete;

    /// Open and start the default playback device. False if none.
    bool start(double sampleRate = 48000.0);
    bool running() const;

    void configure(const VehicleSoundConfig& cfg, const EngineGeometry& engine);
    void setState(const SoundState& s);

    void setMuted(bool m);
    bool muted() const;

    /// Run `fn` against the synthesiser under the audio lock - for the UI
    /// (mix sliders, diagnostics).
    void withSynth(const std::function<void(SoundSynth&)>& fn);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace vsim
