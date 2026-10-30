#pragma once
//===========================================================================
//  DebugUI.h  --  ImGui panels.
//
//  PARAMETER MUTABILITY POLICY
//  ---------------------------
//  Every editable field is tagged so the user knows what they are changing:
//
//    [LIVE]     takes effect immediately and is safe mid-run
//    [RESTART]  stored now, applied on the next reset
//    [DERIVED]  computed, read-only, shown greyed out
//
//  This distinction is not cosmetic. Changing the wheelbase mid-corner is a
//  legitimate thing to want to do while tuning, but the user has to know that
//  the trajectory either side of the change came from two different vehicles.
//===========================================================================

#include <memory>
#include <string>
#include <vector>

#include "config/Config.h"
#include "experiments/Experiment.h"
#include "input/AndroidUsbInput.h"
#include "physics/EngineDyno.h"
#include "rendering/Camera.h"
#include "rendering/DebugDraw.h"
#include "rendering/Renderer.h"

struct GLFWwindow;

namespace vsim {

class Simulation;
class AudioOutput;

/// Things the UI wants the application to do. Polled after draw().
struct UiRequests {
    bool resetSimulation   = false;
    bool exportTelemetry   = false;
    bool reloadVehicle     = false;
    bool reloadTrack       = false;
    bool startExperiment   = false;
    bool stopExperiment    = false;
    bool clearTrajectory   = false;
    bool quit              = false;

    std::string     vehicleToLoad;
    std::string     trackToLoad;
    ExperimentType  experimentToStart = ExperimentType::None;
};

class DebugUI {
public:
    bool initialize(GLFWwindow* window);
    void shutdown();

    void beginFrame();
    /// Build all panels. Returns what the user asked for.
    /// `androidInput` is null when android_input.enabled is false in the
    /// config; the panel and HUD indicator simply don't appear in that case.
    UiRequests draw(Simulation& sim, Camera& camera, Renderer& renderer,
                    DebugDrawOptions& debugOptions,
                    const std::vector<std::string>& availableVehicles,
                    const std::string& currentVehicle,
                    const std::vector<std::string>& availableTracks,
                    const std::string& currentTrack,
                    double frameTimeSeconds,
                    const AndroidUsbInput* androidInput = nullptr,
                    AudioOutput* audio = nullptr);
    void endFrame();

    bool wantsKeyboard() const;
    bool wantsMouse() const;

    void setVisible(bool v) { m_visible = v; }
    void toggleVisible()    { m_visible = !m_visible; }
    bool visible() const    { return m_visible; }

    /// Post a transient message to the status line (e.g. "wrote run_003.csv").
    void notify(const std::string& message);

private:
    void drawMainPanel(Simulation& sim, Camera& camera, Renderer& renderer,
                       DebugDrawOptions& opt,
                       const std::vector<std::string>& vehicles,
                       const std::string& currentVehicle,
                       const std::vector<std::string>& tracks,
                       const std::string& currentTrack,
                       double frameTime, UiRequests& req,
                       const AndroidUsbInput* androidInput,
                       AudioOutput* audio = nullptr);
    void drawVehiclePanel(Simulation& sim, UiRequests& req);
    void drawTirePanel(Simulation& sim);
    void drawSimulationPanel(Simulation& sim, UiRequests& req);
    void drawRoadLoadPanel(Simulation& sim);
    void drawControlsPanel(Simulation& sim);
    void drawPowertrainPanel(Simulation& sim);
    void drawAndroidPanel(const AndroidUsbInput* androidInput);
    void drawTelemetryPanel(Simulation& sim, UiRequests& req);
    void drawGraphsPanel(Simulation& sim);
    void drawExperimentPanel(Simulation& sim, UiRequests& req);
    void drawDebugVisualisationPanel(DebugDrawOptions& opt, Renderer& renderer);
    void drawCameraPanel(Camera& camera);
    void drawSoundPanel(AudioOutput* audio);
    void drawHud(Simulation& sim, double frameTime, const AndroidUsbInput* androidInput);

    bool m_initialized = false;
    bool m_visible = true;

    bool m_showGraphs = true;
    bool m_showHud = true;
    bool m_showDemo = false;

    std::string m_status;
    double m_statusTimer = 0.0;

    /// Smoothed frame time for a readable FPS number.
    double m_smoothedFrameTime = 1.0 / 60.0;

    /// Torque curve of a resolved engine, measured on a private copy of it
    /// (physics/EngineDyno.h) a few milliseconds per frame while the
    /// Powertrain panel's curve is open. Presentation state: it never touches
    /// the engine in the car. Rebuilt when the vehicle changes.
    std::unique_ptr<EngineDyno> m_dyno;
    std::string m_dynoVehicle;
};

} // namespace vsim
