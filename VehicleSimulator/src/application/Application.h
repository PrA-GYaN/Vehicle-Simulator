#pragma once
//===========================================================================
//  Application.h  --  window, main loop, and system wiring.
//
//  DEPENDENCY DIRECTION
//  --------------------
//      Application  ->  Simulation  ->  Vehicle  ->  IVehicleDynamics
//           |
//           +->  Renderer, Input, DebugUI, Camera, Environment
//
//  Arrows point one way only. The Simulation never calls into the Renderer,
//  the Input, or the UI. The Application is the only place the two halves
//  meet, and it does so by reading simulation state and writing driver input.
//===========================================================================

#include <memory>
#include <string>
#include <vector>

#include "audio/AudioOutput.h"
#include "config/Config.h"
#include "environment/Track.h"
#include "input/AndroidUsbInput.h"
#include "input/Input.h"
#include "rendering/Camera.h"
#include "rendering/DebugDraw.h"
#include "rendering/Renderer.h"
#include "simulation/Simulation.h"
#include "ui/DebugUI.h"

struct GLFWwindow;

namespace vsim {

/// Command-line options.
struct LaunchOptions {
    std::string configPath  = "config/simulation.yaml";
    std::string vehicleName;     ///< overrides the config's default_vehicle
    std::string trackName;       ///< overrides the config's default_track
    std::string experiment;      ///< run this experiment at startup
    bool        headless = false;///< run without a window (experiments only)
    double      headlessDuration = 0.0;  ///< seconds; 0 = experiment duration
    bool        hideUi = false;  ///< start with the debug panels hidden (F1)
    std::string cameraName;      ///< initial camera view, see cameraModeFromString()
    std::string audioOut;        ///< headless: also render the sound to this WAV
    std::string tourDir;         ///< run the screenshot/benchmark tour into this directory
    bool        fullscreen = false; ///< open on the primary monitor at its own resolution
};

/// Parse argv. Returns false if the program should exit (e.g. --help).
bool parseCommandLine(int argc, char** argv, LaunchOptions& out);

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /// Create the window and all subsystems. Returns false on fatal error.
    bool initialize(const LaunchOptions& options);

    /// Run the main loop until the window closes.
    int run();

    /// Run without a window: step the simulation, record telemetry, export.
    int runHeadless();

    /// --tour: place the car at stations round the active track, render each
    /// from a few cameras, save a screenshot and time the frames, then exit.
    /// It is how a circuit is checked all the way round without driving it,
    /// and where the frame-time figures in the README come from.
    int runTour();

private:
    bool createWindow();
    void loadVehicle(const std::string& name);
    void loadTrack(const std::string& name);

    /// Load or clear the environment (terrain) model to match m_currentTrack,
    /// then - whether or not that model is active - (re)build terrainField()
    /// from whichever of it or m_track's own centreline should now back the
    /// physics, and wire the result into the vehicle. See
    /// config::EnvironmentModelConfig, Renderer::loadEnvironmentModel() and
    /// Renderer::buildTrackTerrain().
    ///
    /// Windowed mode only, for the environment-model HALF of this: the
    /// model's geometry is extracted during the GL upload GltfModel::load()
    /// does, which needs a current GL context - see README, Open Questions.
    /// A headless run therefore never calls this at all; it instead calls
    /// Renderer::buildTrackTerrain() directly once at startup (see
    /// initialize()), since that half has no such GL dependency.
    ///
    /// Returns true if it placed the vehicle itself (a spawn was applied),
    /// in which case the caller must NOT also run the generic
    /// track-centreline placement.
    bool applyEnvironmentModel();
    void handleActions();
    void handleUiRequests(const UiRequests& req);
    void updateCamera(double frameTime);

    static void framebufferSizeCallback(GLFWwindow* w, int width, int height);

    LaunchOptions m_options;
    AppConfig     m_config;

    GLFWwindow* m_window = nullptr;

    Track m_track;

    std::unique_ptr<Simulation> m_simulation;
    std::unique_ptr<Camera>     m_camera;

    Renderer m_renderer;
    Input    m_input;
    DebugUI  m_ui;

    /// Non-null only when config.android_input.enabled. See
    /// activeControlInput()/activeHandbrake() for how this is combined with
    /// keyboard input - the rest of Application never checks this directly.
    std::unique_ptr<AndroidUsbInput> m_androidInput;

    /// Live sound device - null when disabled in config or no device exists.
    std::unique_ptr<AudioOutput> m_audio;
    /// Headless only: WAV path to render the run's sound into.
    std::string m_audioOut;
    void configureAudio();

    /// Picks the Android controller when it is connected, keyboard/mouse
    /// otherwise. This is the ONLY place that decision is made.
    ControlInput activeControlInput() const;
    bool         activeHandbrake() const;

    DebugDrawOptions m_debugOptions;

    std::vector<std::string> m_availableVehicles;
    std::string m_currentVehicle;

    std::vector<std::string> m_availableTracks;
    std::string m_currentTrack;

    /// Level traction control comes back on at when toggled with H.
    int m_tcLastLevel = 2;

    bool m_running = false;
    bool m_headless = false;
    double m_lastFrameTime = 0.0;
};

} // namespace vsim
