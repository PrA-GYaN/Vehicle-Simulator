#include "audio/AudioOutput.h"

#include <iostream>
#include <mutex>

#include <miniaudio.h>

namespace vsim {

struct AudioOutput::Impl {
    ma_device device{};
    bool deviceOpen = false;
    std::mutex lock;
    SoundSynth synth{48000.0};

    static void callback(ma_device* dev, void* out, const void* /*in*/, ma_uint32 frames) {
        auto* self = static_cast<Impl*>(dev->pUserData);
        std::lock_guard<std::mutex> g(self->lock);
        self->synth.render(static_cast<float*>(out), static_cast<int>(frames));
    }
};

AudioOutput::AudioOutput() : m_impl(std::make_unique<Impl>()) {}

AudioOutput::~AudioOutput() {
    if (m_impl->deviceOpen) ma_device_uninit(&m_impl->device);
}

bool AudioOutput::start(double sampleRate) {
    {
        std::lock_guard<std::mutex> g(m_impl->lock);
        m_impl->synth = SoundSynth(sampleRate);
    }
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = static_cast<ma_uint32>(sampleRate);
    cfg.dataCallback = &Impl::callback;
    cfg.pUserData = m_impl.get();
    // ~10 ms buffers: short enough that a throttle blip is heard with the
    // frame it happens on, long enough not to underrun on a busy frame.
    cfg.periodSizeInMilliseconds = 10;

    if (ma_device_init(nullptr, &cfg, &m_impl->device) != MA_SUCCESS) {
        std::cerr << "[audio] no playback device - running without sound\n";
        return false;
    }
    m_impl->deviceOpen = true;
    if (ma_device_start(&m_impl->device) != MA_SUCCESS) {
        std::cerr << "[audio] could not start the playback device - running without sound\n";
        ma_device_uninit(&m_impl->device);
        m_impl->deviceOpen = false;
        return false;
    }
    std::cout << "[audio] " << m_impl->device.playback.name << " @ "
              << m_impl->device.sampleRate << " Hz\n";
    return true;
}

bool AudioOutput::running() const { return m_impl->deviceOpen; }

void AudioOutput::configure(const VehicleSoundConfig& cfg, const EngineGeometry& engine) {
    std::lock_guard<std::mutex> g(m_impl->lock);
    const SoundSynth::Mix mix = m_impl->synth.mix();
    m_impl->synth.configure(cfg, engine);
    m_impl->synth.mix() = mix;
}

void AudioOutput::setState(const SoundState& s) {
    std::lock_guard<std::mutex> g(m_impl->lock);
    m_impl->synth.setState(s);
}

void AudioOutput::setMuted(bool m) {
    std::lock_guard<std::mutex> g(m_impl->lock);
    m_impl->synth.mix().muted = m;
}

bool AudioOutput::muted() const {
    std::lock_guard<std::mutex> g(m_impl->lock);
    return m_impl->synth.mix().muted;
}

void AudioOutput::withSynth(const std::function<void(SoundSynth&)>& fn) {
    std::lock_guard<std::mutex> g(m_impl->lock);
    fn(m_impl->synth);
}

} // namespace vsim
