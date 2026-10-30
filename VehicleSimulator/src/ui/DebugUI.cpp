#include "ui/DebugUI.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include "audio/AudioOutput.h"
#include "physics/Powertrain.h"
#include "physics/SuspensionModel.h"
#include "physics/VehicleDynamicsFactory.h"
#include "simulation/Simulation.h"
#include "vehicle/Vehicle.h"

namespace vsim {
namespace {

//---------------------------------------------------------------------------
//  Small helpers for the mutability tagging described in the header.
//---------------------------------------------------------------------------

void tagLive()    { ImGui::SameLine(); ImGui::TextColored({0.45f, 0.85f, 0.45f, 1.0f}, "[LIVE]"); }
void tagRestart() { ImGui::SameLine(); ImGui::TextColored({0.95f, 0.75f, 0.35f, 1.0f}, "[RESTART]"); }

/// Read-only derived value, greyed out so it visibly differs from an input.
void derivedRow(const char* label, const char* fmt, double value, const char* unit) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.62f, 0.66f, 0.72f, 1.0f));
    ImGui::Text("%-26s", label);
    ImGui::SameLine();
    ImGui::Text(fmt, value);
    ImGui::SameLine();
    ImGui::TextUnformatted(unit);
    ImGui::SameLine();
    ImGui::TextColored({0.55f, 0.60f, 0.70f, 1.0f}, "[DERIVED]");
    ImGui::PopStyleColor();
}

void helpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

/// A labelled bar for a 0..1 control value.
void controlBar(const char* label, double value, const ImVec4& color) {
    ImGui::Text("%-10s", label);
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
    ImGui::ProgressBar(static_cast<float>(value), ImVec2(-1.0f, 0.0f));
    ImGui::PopStyleColor();
}

/// Bipolar bar for a -1..+1 value (steering), centred at zero.
void bipolarBar(const char* label, double value, const ImVec4& color) {
    ImGui::Text("%-10s", label);
    ImGui::SameLine();
    const float t = static_cast<float>((clampd(value, -1.0, 1.0) + 1.0) * 0.5);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
    char overlay[32];
    std::snprintf(overlay, sizeof(overlay), "%+.2f", value);
    ImGui::ProgressBar(t, ImVec2(-1.0f, 0.0f), overlay);
    ImGui::PopStyleColor();
}

} // namespace

//---------------------------------------------------------------------------
//  Lifecycle
//---------------------------------------------------------------------------

bool DebugUI::initialize(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;   // do not litter the working directory

    // NOTE: ImGuiConfigFlags_NavEnableKeyboard is deliberately NOT enabled.
    // Keyboard nav makes ImGui assert WantCaptureKeyboard as soon as a panel
    // has focus, which would swallow WASD and leave the car unresponsive
    // after the first click on the UI. For a driving simulator the vehicle
    // must keep responding to the keyboard unless the user is actually
    // typing into a field - see wantsKeyboard() below.

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding  = 3.0f;
    style.GrabRounding   = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.Colors[ImGuiCol_WindowBg].w = 0.92f;

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) return false;
    if (!ImGui_ImplOpenGL3_Init("#version 330 core")) return false;

    m_initialized = true;
    return true;
}

void DebugUI::shutdown() {
    if (!m_initialized) return;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    m_initialized = false;
}

void DebugUI::beginFrame() {
    if (!m_initialized) return;
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void DebugUI::endFrame() {
    if (!m_initialized) return;
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool DebugUI::wantsKeyboard() const {
    // WantTextInput, not WantCaptureKeyboard.
    //
    // WantCaptureKeyboard is true whenever any ImGui window is hovered or
    // focused, which for a driving sim would mean the car stops responding
    // the moment the mouse drifts over a panel. WantTextInput is true only
    // while a text/numeric field is being edited - exactly the case where a
    // keypress must go to the field instead of the vehicle.
    return m_initialized && ImGui::GetIO().WantTextInput;
}
bool DebugUI::wantsMouse() const {
    return m_initialized && ImGui::GetIO().WantCaptureMouse;
}

void DebugUI::notify(const std::string& message) {
    m_status = message;
    m_statusTimer = 4.0;
}

//---------------------------------------------------------------------------
//  Top-level draw
//---------------------------------------------------------------------------

UiRequests DebugUI::draw(Simulation& sim, Camera& camera, Renderer& renderer,
                         DebugDrawOptions& debugOptions,
                         const std::vector<std::string>& availableVehicles,
                         const std::string& currentVehicle,
                         const std::vector<std::string>& availableTracks,
                         const std::string& currentTrack,
                         double frameTimeSeconds,
                         const AndroidUsbInput* androidInput,
                         AudioOutput* audio) {
    UiRequests req;
    if (!m_initialized) return req;

    // Exponential smoothing on the frame time so the FPS readout is stable
    // enough to read rather than flickering every frame.
    m_smoothedFrameTime = m_smoothedFrameTime * 0.92 + frameTimeSeconds * 0.08;

    if (m_statusTimer > 0.0) m_statusTimer -= frameTimeSeconds;

    // The HUD stays visible even when the panels are hidden - it is the
    // minimum readout needed to drive, and that now includes whether the
    // Android controller is the thing actually driving.
    if (m_showHud) drawHud(sim, frameTimeSeconds, androidInput);

    if (!m_visible) return req;

    drawMainPanel(sim, camera, renderer, debugOptions,
                  availableVehicles, currentVehicle,
                  availableTracks, currentTrack, frameTimeSeconds, req,
                  androidInput, audio);

    if (m_showGraphs) drawGraphsPanel(sim);
    if (m_showDemo)   ImGui::ShowDemoWindow(&m_showDemo);

    return req;
}

//---------------------------------------------------------------------------
//  HUD
//---------------------------------------------------------------------------

void DebugUI::drawHud(Simulation& sim, double, const AndroidUsbInput* androidInput) {
    const Vehicle& v = sim.vehicle();
    const VehicleState& s = v.state();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                             vp->WorkPos.y + 12.0f},
                            ImGuiCond_Always, {1.0f, 0.0f});
    ImGui::SetNextWindowBgAlpha(0.72f);

    constexpr ImGuiWindowFlags kFlags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

    if (ImGui::Begin("##hud", nullptr, kFlags)) {
        //-------------------------------------------------------------------
        //  The readout requested in the specification, in SI plus the
        //  human-facing km/h and degrees.
        //-------------------------------------------------------------------
        ImGui::Text("Velocity:       %8.2f m/s",   s.velocity);
        ImGui::Text("Speed:          %8.1f km/h",  mpsToKph(s.velocity));
        ImGui::Text("Acceleration:   %8.2f m/s2",  s.acceleration);
        ImGui::Text("Steering:       %8.2f deg",   radToDeg(s.steeringAngle));
        ImGui::Text("Yaw rate:       %8.3f rad/s", s.yawRate);

        const double R = v.turningRadius();
        if (std::isfinite(R) && std::abs(R) < 1.0e4) {
            ImGui::Text("Turning radius: %8.1f m", std::abs(R));
        } else {
            ImGui::Text("Turning radius:  straight");
        }

        ImGui::Separator();
        ImGui::Text("Heading:        %8.1f deg", radToDeg(s.yaw));
        ImGui::Text("Position:  %7.1f, %7.1f m", s.position.x, s.position.z);

        //-------------------------------------------------------------------
        //  Lateral acceleration.
        //
        //  0.3 g is the boundary both sources agree on (KB Section 9.4.5 and,
        //  independently, Section 9.4.13) for where tyre behaviour stops being
        //  substantially linear. For the kinematic model that is the point
        //  where it stops telling the truth. For a tyre-force model it is
        //  simply the point where the interesting behaviour starts.
        //-------------------------------------------------------------------
        const double ay = v.lateralAcceleration();
        const double ayG = std::abs(ay) / kGravity;
        const ImVec4 ayColor = (ayG > 0.3) ? ImVec4(1.0f, 0.45f, 0.35f, 1.0f)
                                           : ImVec4(0.75f, 0.85f, 0.75f, 1.0f);
        ImGui::TextColored(ayColor, "Lateral accel:  %8.2f m/s2  (%.2f g)", ay, ayG);

        if (!s.wheelDataValid) {
            if (ayG > 0.3) {
                ImGui::TextColored({1.0f, 0.55f, 0.35f, 1.0f},
                                   "  ! beyond kinematic model validity");
            }
        } else {
            ImGui::Text("Sideslip:       %8.2f deg", radToDeg(s.sideslipAngle));

            // Grip headroom: the worst tyre on the car. This is the number
            // that says how much is left before something lets go.
            double worst = 0.0;
            int locked = 0, spinning = 0;
            for (int i = 0; i < WheelCount; ++i) {
                worst = std::max(worst, s.wheels[i].frictionUsage);
                if (s.wheels[i].locked)   ++locked;
                if (s.wheels[i].spinning) ++spinning;
            }
            const ImVec4 gripColor = (worst > 0.99) ? ImVec4(1.0f, 0.4f, 0.35f, 1.0f)
                                   : (worst > 0.85) ? ImVec4(1.0f, 0.8f, 0.3f, 1.0f)
                                                    : ImVec4(0.75f, 0.85f, 0.75f, 1.0f);
            ImGui::TextColored(gripColor, "Grip used:      %8.0f %%  (worst tyre)",
                               worst * 100.0);
            if (locked > 0) {
                ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f},
                                   "  WHEEL LOCK  x%d", locked);
            }
            if (spinning > 0) {
                ImGui::TextColored({1.0f, 0.75f, 0.25f, 1.0f},
                                   "  WHEELSPIN   x%d", spinning);
            }

            // Driver aid and tyres. Always two rows, so the HUD does not
            // change height when traction control cuts in.
            const Vehicle& car = sim.vehicle();
            const int tc = car.tractionControl();
            const double cut = car.tractionControlCut();
            if (tc == 0) {
                ImGui::TextDisabled("TC:  off                   (G/H)");
            } else if (cut > 0.02) {
                ImGui::TextColored({1.0f, 0.75f, 0.25f, 1.0f},
                                   "TC:  level %d   cutting %3.0f %%", tc, cut * 100.0);
            } else {
                ImGui::Text("TC:  level %d", tc);
            }

            const TireCompound compound = car.tireCompound();
            if (compound == TireCompound::Standard) {
                ImGui::TextDisabled("Tyres: Standard (no wear)    (Y)");
            } else {
                double wear = 0.0;
                for (int i = 0; i < WheelCount; ++i) wear = std::max(wear, s.wheels[i].wear);
                const ImVec4 c = (compound == TireCompound::Soft)   ? ImVec4(1.0f, 0.35f, 0.35f, 1.0f)
                               : (compound == TireCompound::Medium) ? ImVec4(1.0f, 0.85f, 0.25f, 1.0f)
                                                                    : ImVec4(0.92f, 0.92f, 0.92f, 1.0f);
                ImGui::TextColored(c, "Tyres: %-6s  worn %3.0f %%  grip %3.0f %%",
                                   tireCompoundName(compound), wear * 100.0,
                                   tireWearGripFactor(wear) * 100.0);
            }
        }

        //-------------------------------------------------------------------
        //  Android controller status. Only shown when the feature is
        //  enabled in config - see AndroidUsbInput. This is deliberately in
        //  the always-visible HUD, not just the collapsible panel below:
        //  connection state is exactly the thing you need to see without
        //  hunting for it before trusting the phone with the wheel.
        //-------------------------------------------------------------------
        if (androidInput) {
            if (androidInput->isConnected()) {
                ImGui::TextColored({0.4f, 0.9f, 0.4f, 1.0f},
                                   "ANDROID: CONNECTED  (port %d)", androidInput->port());
            } else {
                ImGui::TextColored({1.0f, 0.45f, 0.35f, 1.0f},
                                   "ANDROID: DISCONNECTED - keyboard active, safe state");
            }
        }

        //-------------------------------------------------------------------
        //  Powertrain, for a vehicle that has a real engine. Gear and revs
        //  belong in the always-visible HUD rather than only in a panel:
        //  they are what a driver actually watches.
        //-------------------------------------------------------------------
        if (const Powertrain* pt = sim.vehicle().powertrain()) {
            const PowertrainOutput& o = pt->lastOutput();
            const EngineParameters& ep = pt->engine().parameters();
            const ImVec4 rpmColor = o.limiterActive
                ? ImVec4{1.0f, 0.35f, 0.30f, 1.0f}
                : (o.engineRpm > ep.redlineRpm * 0.90)
                    ? ImVec4{1.0f, 0.70f, 0.25f, 1.0f}
                    : ImVec4{0.85f, 0.90f, 0.95f, 1.0f};
            ImGui::Separator();

            // NOTE: every SameLine() below is paired with a widget that is
            // ALWAYS drawn. A SameLine() followed by a conditional widget
            // leaks onto the next line when the condition is false, which
            // silently runs two HUD rows together.
            ImGui::Text("Gear:");
            ImGui::SameLine(110);
            if (o.currentGear <= kNeutralGear) {
                ImGui::TextColored({0.95f, 0.85f, 0.45f, 1.0f}, "N");
            } else {
                ImGui::TextColored({0.55f, 0.85f, 1.0f, 1.0f}, "%d", o.currentGear + 1);
            }
            ImGui::SameLine(140);
            ImGui::TextColored(rpmColor, "%5.0f rpm", o.engineRpm);

            const char* clutchTag = o.limiterActive ? "LIMIT"
                                  : (!o.locked)     ? "SLIP"
                                                    : "";
            if (clutchTag[0] != '\0') {
                ImGui::SameLine(250);
                ImGui::TextColored(o.limiterActive ? ImVec4{1.0f, 0.35f, 0.3f, 1.0f}
                                                   : ImVec4{1.0f, 0.75f, 0.3f, 1.0f},
                                   "%s", clutchTag);
            }

            const double powerKw = o.engineTorqueNm * o.engineRpm * kTwoPi / 60.0 / 1000.0;
            ImGui::Text("Engine:");
            ImGui::SameLine(110);
            ImGui::Text("%.0f N.m", o.engineTorqueNm);
            ImGui::SameLine(200);
            ImGui::Text("%.0f kW", powerKw);
        }

        ImGui::Separator();
        if (sim.isPaused()) {
            ImGui::TextColored({1.0f, 0.85f, 0.3f, 1.0f}, "PAUSED  (P to resume)");
        }
        if (sim.telemetry().isRecording()) {
            ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "REC  %zu samples",
                               sim.telemetry().sampleCount());
        }
        if (sim.experimentRunning() && sim.experiment()) {
            ImGui::TextColored({0.4f, 0.85f, 1.0f, 1.0f}, "EXPERIMENT  %s  t=%.2f s",
                               experimentTypeToString(sim.experiment()->type()),
                               sim.experimentTime());
        }
        if (!m_visible) {
            ImGui::TextDisabled("F1 - show panels");
        }
    }
    ImGui::End();
}

//---------------------------------------------------------------------------
//  Main panel
//---------------------------------------------------------------------------

void DebugUI::drawMainPanel(Simulation& sim, Camera& camera, Renderer& renderer,
                            DebugDrawOptions& opt,
                            const std::vector<std::string>& vehicles,
                            const std::string& currentVehicle,
                            const std::vector<std::string>& tracks,
                            const std::string& currentTrack,
                            double frameTime, UiRequests& req,
                            const AndroidUsbInput* androidInput,
                            AudioOutput* audio) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x + 12.0f, vp->WorkPos.y + 12.0f},
                            ImGuiCond_FirstUseEver);

    // Size relative to the viewport rather than a fixed 430x780. On a small
    // or tiled window a hardcoded width covers most of the 3D view, which
    // defeats the point of a visualiser.
    const float panelW = std::min(430.0f, vp->WorkSize.x * 0.34f);
    const float panelH = std::min(780.0f, vp->WorkSize.y * 0.86f);
    ImGui::SetNextWindowSize({panelW, panelH}, ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Vehicle Simulator", nullptr, ImGuiWindowFlags_MenuBar)) {
        ImGui::End();
        return;
    }

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("HUD", nullptr, &m_showHud);
            ImGui::MenuItem("Graphs", nullptr, &m_showGraphs);
            ImGui::MenuItem("ImGui demo", nullptr, &m_showDemo);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Reset", "R"))  req.resetSimulation = true;
            if (ImGui::MenuItem(sim.isPaused() ? "Resume" : "Pause", "P")) sim.togglePaused();
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Esc")) req.quit = true;
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    //-- run/pause/reset row ----------------------------------------------
    if (ImGui::Button(sim.isPaused() ? "Resume" : "Pause", {88, 0})) sim.togglePaused();
    ImGui::SameLine();
    if (ImGui::Button("Reset", {88, 0})) req.resetSimulation = true;
    ImGui::SameLine();
    ImGui::BeginDisabled(!sim.isPaused());
    if (ImGui::Button("Step", {88, 0})) sim.singleStep(ControlInput{});
    ImGui::EndDisabled();
    helpMarker("Advance exactly one physics timestep while paused.");

    ImGui::Text("FPS %.0f   (%.2f ms/frame)",
                (m_smoothedFrameTime > 0.0) ? 1.0 / m_smoothedFrameTime : 0.0,
                m_smoothedFrameTime * 1000.0);
    ImGui::Text("Sim time %.2f s   steps %lld   (%d this frame)",
                sim.time(), sim.stepCount(), sim.lastStepCount());

    if (sim.didClampLastFrame()) {
        ImGui::TextColored({1.0f, 0.5f, 0.3f, 1.0f},
                           "Step budget exceeded - simulation running slow");
        helpMarker("The machine could not execute all the physics steps the "
                   "elapsed time demanded, so surplus simulated time was "
                   "dropped. Increase the timestep or lower max_steps_per_frame.");
    }

    if (m_statusTimer > 0.0 && !m_status.empty()) {
        ImGui::TextColored({0.45f, 0.9f, 0.55f, 1.0f}, "%s", m_status.c_str());
    }

    ImGui::Separator();

    //-- vehicle selector --------------------------------------------------
    if (!vehicles.empty()) {
        if (ImGui::BeginCombo("Vehicle config", currentVehicle.c_str())) {
            for (const std::string& name : vehicles) {
                const bool selected = (name == currentVehicle);
                if (ImGui::Selectable(name.c_str(), selected)) {
                    req.reloadVehicle = true;
                    req.vehicleToLoad = name;
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        helpMarker("Loads config/vehicles/<name>.yaml and resets the simulation.");
    }

    //-- track selector ------------------------------------------------------
    if (!tracks.empty()) {
        if (ImGui::BeginCombo("Track", currentTrack.c_str())) {
            for (const std::string& name : tracks) {
                const bool selected = (name == currentTrack);
                if (ImGui::Selectable(name.c_str(), selected)) {
                    req.reloadTrack = true;
                    req.trackToLoad = name;
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        helpMarker("Rebuilds the road and puts the vehicle back on the new "
                   "track's start line. 'apex_gp' is the Apex Grand Prix "
                   "Circuit, a complete venue with pits and grandstands; "
                   "'race_circuit', 'nurburgring' and 'nordschleife' are closed "
                   "circuits too; the others are point-to-point routes.");
    }

    drawSimulationPanel(sim, req);
    drawVehiclePanel(sim, req);
    drawTirePanel(sim);
    drawRoadLoadPanel(sim);
    drawControlsPanel(sim);
    drawPowertrainPanel(sim);
    drawAndroidPanel(androidInput);
    drawExperimentPanel(sim, req);
    drawTelemetryPanel(sim, req);
    drawDebugVisualisationPanel(opt, renderer);
    drawCameraPanel(camera);
    drawSoundPanel(audio);

    if (ImGui::CollapsingHeader("Keyboard reference", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("W / Up      throttle");
        ImGui::BulletText("S / Down    brake");
        ImGui::BulletText("A / Left    steer left");
        ImGui::BulletText("D / Right   steer right");
        ImGui::BulletText("Space       emergency brake");
        ImGui::BulletText("Z / X       gear down / up   (Z from 1st = neutral)");
        ImGui::BulletText("Left Shift  clutch pedal     (manual clutch)");
        ImGui::BulletText("R           reset vehicle");
        ImGui::BulletText("P           pause / resume");
        ImGui::BulletText("N           single step (while paused)");
        ImGui::BulletText("T           toggle telemetry recording");
        ImGui::BulletText("F2          export telemetry to CSV");
        ImGui::BulletText("F1          toggle this UI");
        ImGui::BulletText("C           reset camera");
        ImGui::BulletText("V           cycle camera mode");
        ImGui::BulletText("Mouse drag  orbit camera");
        ImGui::BulletText("Scroll      zoom");
        ImGui::BulletText("IJKL + Q/E  move free camera");
        ImGui::BulletText("Esc         quit");
    }

    (void)frameTime;
    ImGui::End();
}

//---------------------------------------------------------------------------
//  Simulation panel
//---------------------------------------------------------------------------

void DebugUI::drawSimulationPanel(Simulation& sim, UiRequests& req) {
    if (!ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) return;

    SimulationConfig& cfg = sim.simConfig();

    //-- dynamics model ----------------------------------------------------
    //
    // The swap point of the whole project. Switching here does not reset the
    // vehicle: the new model picks up the state the old one left behind, so a
    // manoeuvre can be run through both and compared directly.
    {
        const std::vector<std::string> models = availableDynamicsModels();
        const std::string& active = sim.dynamicsModelName();
        if (ImGui::BeginCombo("Dynamics model", active.c_str())) {
            for (const std::string& name : models) {
                const bool selected = (name == active);
                if (ImGui::Selectable(name.c_str(), selected)) {
                    sim.setDynamicsModel(name);
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        tagLive();
        ImGui::TextWrapped("%s", sim.vehicle().dynamics().assumptions());
    }

    ImGui::Separator();

    //-- timestep ----------------------------------------------------------
    //
    // Presented as discrete choices because those are the values that matter,
    // and because a slider invites dragging the timestep through a corner,
    // which changes the integration error mid-manoeuvre.
    static const double kTimesteps[] = {0.001, 0.005, 0.01, 0.02, 0.05};
    static const char*  kLabels[]    = {"0.001 s (1000 Hz)", "0.005 s (200 Hz)",
                                        "0.010 s (100 Hz)",  "0.020 s (50 Hz)",
                                        "0.050 s (20 Hz)"};

    int current = 2;
    for (int i = 0; i < 5; ++i) {
        if (std::abs(cfg.timestep - kTimesteps[i]) < 1e-9) { current = i; break; }
    }
    if (ImGui::Combo("Physics timestep", &current, kLabels, 5)) {
        cfg.timestep = kTimesteps[current];
        cfg.validate();
    }
    tagLive();
    helpMarker("Fixed physics timestep. Physics always advances in exact "
               "increments of this value regardless of frame rate. The "
               "integrator is explicit Euler, so halving this halves the "
               "integration error.");

    float ts = static_cast<float>(cfg.timestep);
    if (ImGui::SliderFloat("  custom dt", &ts, 0.0001f, 0.05f, "%.4f s",
                           ImGuiSliderFlags_Logarithmic)) {
        cfg.timestep = ts;
        cfg.validate();
    }

    float scale = static_cast<float>(cfg.timeScale);
    if (ImGui::SliderFloat("Time scale", &scale, 0.0f, 4.0f, "%.2f x")) {
        cfg.timeScale = scale;
        cfg.validate();
    }
    tagLive();
    helpMarker("Simulated seconds per wall-clock second. 0 freezes time "
               "without pausing the renderer.");

    ImGui::Checkbox("Real-time pacing", &cfg.realtime);
    tagLive();
    helpMarker("On: physics is paced to wall-clock time via an accumulator. "
               "Off: exactly one step per rendered frame, as fast as possible.");

    ImGui::SliderInt("Max steps / frame", &cfg.maxStepsPerFrame, 1, 64);
    tagLive();
    helpMarker("Upper bound on physics steps per rendered frame. Prevents the "
               "'spiral of death' where a slow frame demands more steps than "
               "can be executed, which makes the next frame slower still.");

    ImGui::Text("Accumulator: %.1f%% of a step", sim.interpolationAlpha() * 100.0);

    ImGui::Separator();
    ImGui::TextDisabled("Physics model");
    ImGui::Text("%s", sim.vehicle().dynamics().name());
    ImGui::TextWrapped("%s", sim.vehicle().dynamics().assumptions());

    ImGui::Separator();
    ImGui::TextDisabled("Initial state (applied on reset)");
    InitialStateConfig& init = sim.appConfig().initialState;

    double pos[3] = {init.position.x, init.position.y, init.position.z};
    if (ImGui::InputScalarN("Position", ImGuiDataType_Double, pos, 3, nullptr,
                            nullptr, "%.2f")) {
        init.position = {pos[0], pos[1], pos[2]};
    }
    tagRestart();
    helpMarker("Rear axle reference point, in world metres.");

    float yawDeg = static_cast<float>(radToDeg(init.yaw));
    if (ImGui::SliderFloat("Yaw", &yawDeg, -180.0f, 180.0f, "%.1f deg")) {
        init.yaw = degToRad(yawDeg);
    }
    tagRestart();

    float v0 = static_cast<float>(init.velocity);
    if (ImGui::SliderFloat("Velocity", &v0, 0.0f, 60.0f, "%.1f m/s")) {
        init.velocity = v0;
    }
    tagRestart();

    if (ImGui::Button("Reset to initial state")) req.resetSimulation = true;
}

//---------------------------------------------------------------------------
//  Vehicle panel
//---------------------------------------------------------------------------

void DebugUI::drawVehiclePanel(Simulation& sim, UiRequests&) {
    if (!ImGui::CollapsingHeader("Vehicle", ImGuiTreeNodeFlags_DefaultOpen)) return;

    VehicleParameters& p = sim.vehicle().mutableParameters();

    ImGui::Text("Name: %s", p.name.c_str());
    ImGui::Separator();

    //-- driver aid and tyres ------------------------------------------------
    {
        Vehicle& car = sim.vehicle();
        int tc = car.tractionControl();
        ImGui::Text("Traction control");
        ImGui::SameLine();
        bool picked = ImGui::RadioButton("Off", &tc, 0);
        for (int level = 1; level <= Vehicle::kTractionControlLevels; ++level) {
            ImGui::SameLine();
            char label[8];
            std::snprintf(label, sizeof(label), "%d", level);
            picked |= ImGui::RadioButton(label, &tc, level);
        }
        if (picked) car.setTractionControl(tc);
        helpMarker("Closes the throttle when the driven wheels spin past a slip "
                   "target. 1 lets the tyres run a little past the peak of their "
                   "grip curve, 2 holds them at it, 3 keeps them well short. "
                   "G cycles the level, H switches it on and off. It limits "
                   "wheelspin only - it is not stability control.");

        int compound = static_cast<int>(car.tireCompound());
        const int before = compound;
        ImGui::Text("Tyres");
        for (int c = 0; c < 4; ++c) {
            ImGui::SameLine();
            ImGui::RadioButton(tireCompoundName(static_cast<TireCompound>(c)), &compound, c);
        }
        if (compound != before) car.setTireCompound(static_cast<TireCompound>(compound));
        ImGui::SameLine();
        if (ImGui::SmallButton("New set")) car.setTireCompound(car.tireCompound());
        helpMarker("Soft, Medium and Hard scale this car's tyre grip by +10 %, "
                   "+5 % and 0 %, and wear out after roughly 6, 12 and 21 MJ of "
                   "sliding work per tyre; grip fades slowly with wear and falls "
                   "away over the last fifth. Standard is the tyre as the vehicle "
                   "file describes it and does not wear. These numbers are "
                   "authored, not from the source material, and there is no tyre "
                   "temperature model. Y fits a new set of the next compound.");

        if (car.tireCompound() != TireCompound::Standard) {
            const auto& w = car.state().wheels;
            ImGui::TextDisabled("  wear  FL %2.0f%%  FR %2.0f%%  RL %2.0f%%  RR %2.0f%%",
                                w[FrontLeft].wear * 100.0, w[FrontRight].wear * 100.0,
                                w[RearLeft].wear * 100.0, w[RearRight].wear * 100.0);
        }
        ImGui::Separator();
    }

    bool changed = false;

    //-- mass and geometry -------------------------------------------------
    float mass = static_cast<float>(p.mass);
    if (ImGui::SliderFloat("Mass", &mass, 200.0f, 8000.0f, "%.0f kg")) {
        p.mass = mass; changed = true;
    }
    tagLive();
    helpMarker("Used by any model that solves a force balance (F = m a, "
               "Gillespie Eq. 1-1) and by every load-transfer term. The "
               "kinematic bicycle model ignores it entirely.");

    float wb = static_cast<float>(p.wheelbase);
    if (ImGui::SliderFloat("Wheelbase", &wb, 1.5f, 6.0f, "%.2f m")) {
        p.wheelbase = wb; changed = true;
    }
    tagLive();
    helpMarker("L in yaw_rate = (v / L) * tan(delta). Directly sets how "
               "sharply the vehicle turns - the single most influential "
               "parameter in this model.");

    float cgh = static_cast<float>(p.cgHeight);
    if (ImGui::SliderFloat("CG height", &cgh, 0.05f, 2.5f, "%.2f m")) {
        p.cgHeight = cgh; changed = true;
    }
    tagLive();
    helpMarker("h in the load-transfer terms. Longitudinal transfer is "
               "governed entirely by h/L (Gillespie Eq. 1-8/1-9) and lateral "
               "transfer by h/t, so this one number sets how much load moves "
               "under acceleration, braking and cornering.");

    float fwd = static_cast<float>(p.frontWeightDistribution);
    if (ImGui::SliderFloat("Front weight dist.", &fwd, 0.05f, 0.95f, "%.3f")) {
        p.frontWeightDistribution = fwd; changed = true;
    }
    tagLive();
    helpMarker("Fraction of static weight on the front axle. Determines where "
               "the CG sits along the wheelbase.");

    float tf = static_cast<float>(p.frontTrack);
    if (ImGui::SliderFloat("Front track", &tf, 0.8f, 3.0f, "%.2f m")) {
        p.frontTrack = tf; changed = true;
    }
    tagLive();
    float tr = static_cast<float>(p.rearTrack);
    if (ImGui::SliderFloat("Rear track", &tr, 0.8f, 3.0f, "%.2f m")) {
        p.rearTrack = tr; changed = true;
    }
    tagLive();

    ImGui::Separator();
    ImGui::TextDisabled("Dimensions (visual)");
    float dims[3] = {static_cast<float>(p.length), static_cast<float>(p.width),
                     static_cast<float>(p.height)};
    if (ImGui::SliderFloat3("L / W / H", dims, 0.5f, 12.0f, "%.2f m")) {
        p.length = dims[0]; p.width = dims[1]; p.height = dims[2];
        changed = true;
    }
    tagLive();

    ImGui::Separator();
    ImGui::TextDisabled("Limits");

    float maxSteer = static_cast<float>(radToDeg(p.maxSteeringAngle));
    if (ImGui::SliderFloat("Max steering", &maxSteer, 1.0f, 80.0f, "%.1f deg")) {
        p.maxSteeringAngle = degToRad(maxSteer); changed = true;
    }
    tagLive();

    float steerRate = static_cast<float>(radToDeg(p.steeringRate));
    if (ImGui::SliderFloat("Steering rate", &steerRate, 10.0f, 1000.0f, "%.0f deg/s")) {
        p.steeringRate = degToRad(steerRate);
        sim.vehicle().mutableControls().steeringRate = p.steeringRate;
        changed = true;
    }
    tagLive();
    helpMarker("Actuator slew rate. The road wheels move toward the requested "
               "angle at this rate rather than jumping to it.");

    float maxAcc = static_cast<float>(p.maxAcceleration);
    if (ImGui::SliderFloat("Max acceleration", &maxAcc, 0.1f, 25.0f, "%.2f m/s2")) {
        p.maxAcceleration = maxAcc; changed = true;
    }
    tagLive();

    float maxBrk = static_cast<float>(p.maxBraking);
    if (ImGui::SliderFloat("Max braking", &maxBrk, 0.1f, 30.0f, "%.2f m/s2")) {
        p.maxBraking = maxBrk; changed = true;
    }
    tagLive();

    float maxSpd = static_cast<float>(p.maxSpeed);
    if (ImGui::SliderFloat("Max speed", &maxSpd, 1.0f, 120.0f, "%.1f m/s")) {
        p.maxSpeed = maxSpd; changed = true;
    }
    tagLive();
    ImGui::Text("        = %.0f km/h", mpsToKph(p.maxSpeed));

    if (changed) p.validate();

    //-- derived -----------------------------------------------------------
    ImGui::Separator();
    ImGui::TextDisabled("Derived (read-only)");
    derivedRow("Front axle to CG",  "%8.3f", p.frontAxleToCg(), "m");
    derivedRow("Rear axle to CG",   "%8.3f", p.rearAxleToCg(), "m");
    derivedRow("Static front load", "%8.0f", p.staticFrontAxleLoad(), "N");
    derivedRow("Static rear load",  "%8.0f", p.staticRearAxleLoad(), "N");
    derivedRow("Min turn radius",   "%8.2f", p.minTurnRadius(), "m");
    helpMarker("R = L / tan(delta_max), measured about the rear axle. This is "
               "the Ackermann relation.");
}

//---------------------------------------------------------------------------
//  Tyre panel
//
//  Four tyres, one row each, showing exactly the quantities the Knowledge
//  Base's tyre block takes in and puts out (KB Section 2.2). This is the panel
//  that explains the vehicle's behaviour: if the car will not turn, one of
//  these rows is at 100 % of its friction circle.
//---------------------------------------------------------------------------

void DebugUI::drawTirePanel(Simulation& sim) {
    if (!ImGui::CollapsingHeader("Tyres", ImGuiTreeNodeFlags_DefaultOpen)) return;

    Vehicle& v = sim.vehicle();
    const VehicleState& s = v.state();

    if (!s.wheelDataValid) {
        ImGui::TextDisabled("The active model has no tyres.");
        helpMarker("The kinematic bicycle model assumes the vehicle goes "
                   "exactly where its wheels point. There are no slip angles, "
                   "no loads and no forces to show. Switch the model in the "
                   "Simulation panel.");
        return;
    }

    static const char* kNames[WheelCount] = {"FL", "FR", "RL", "RR"};

    if (ImGui::BeginTable("tyres", 8,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Fz [N]");
        ImGui::TableSetupColumn("alpha");
        ImGui::TableSetupColumn("slip");
        ImGui::TableSetupColumn("gamma");
        ImGui::TableSetupColumn("Fx [N]");
        ImGui::TableSetupColumn("Fy [N]");
        ImGui::TableSetupColumn("R(k)");
        ImGui::TableHeadersRow();

        for (int i = 0; i < WheelCount; ++i) {
            const WheelDynamics& w = s.wheels[i];
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            if (w.locked) {
                ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s LOCK", kNames[i]);
            } else if (w.spinning) {
                ImGui::TextColored({1.0f, 0.75f, 0.25f, 1.0f}, "%s SPIN", kNames[i]);
            } else {
                ImGui::Text("%s", kNames[i]);
            }

            ImGui::TableNextColumn(); ImGui::Text("%6.0f", w.verticalLoad);
            ImGui::TableNextColumn(); ImGui::Text("%6.2f", radToDeg(w.slipAngle));
            ImGui::TableNextColumn(); ImGui::Text("%6.3f", w.slipRatio);
            ImGui::TableNextColumn(); ImGui::Text("%6.2f", radToDeg(w.camberAngle));
            ImGui::TableNextColumn(); ImGui::Text("%6.0f", w.longitudinalForce);
            ImGui::TableNextColumn(); ImGui::Text("%6.0f", w.lateralForce);

            ImGui::TableNextColumn();
            const float used = static_cast<float>(w.frictionUsage);
            const ImVec4 col = (used > 0.99f) ? ImVec4(1.0f, 0.4f, 0.35f, 1.0f)
                             : (used > 0.85f) ? ImVec4(1.0f, 0.8f, 0.3f, 1.0f)
                                              : ImVec4(0.55f, 0.85f, 0.6f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
            ImGui::ProgressBar(used, {-1.0f, 0.0f});
            ImGui::PopStyleColor();
        }
        ImGui::EndTable();
    }

    helpMarker("alpha is the slip angle and gamma the inclination (camber) "
               "angle, both in degrees (KB Section 1.5). `slip` is the "
               "Knowledge Base's braked-wheel slip ratio, S = (V - Re*w)/V "
               "(Eq. 10-1): 0 free rolling, +1 locked, negative under drive. "
               "R(k) is the normalised resultant force of Milliken Eq. (14.16) "
               "- the fraction of the friction ellipse in use. It reaches 1.00 "
               "at the tyre's limit and cannot exceed it: the ellipse is an "
               "identity of the model, not a clamp applied afterwards.");

    //-- vehicle-level readings -------------------------------------------
    ImGui::Separator();
    ImGui::TextDisabled("Directional response");

    derivedRow("Body roll phi", "%8.2f", radToDeg(s.rollAngle), "deg");
    helpMarker("Positive means leaning to the vehicle's RIGHT. A right-hand "
               "turn therefore rolls it NEGATIVE - Milliken p.681: 'the car "
               "rolls to the outside of the turn.' Quasi-static: no source "
               "gives a roll equation of motion.");

    derivedRow("Sideslip beta",  "%8.2f", radToDeg(s.sideslipAngle), "deg");
    helpMarker("The angle between where the vehicle POINTS and where it is "
               "GOING (KB Section 1.3). Positive means the velocity vector "
               "lies to the vehicle's right of its heading.");

    derivedRow("Sideslip rate",  "%8.2f", radToDeg(s.sideslipRate), "deg/s");
    helpMarker("beta_dot = Ay/V - omega (KB Eq. 7.29). The part of the yaw "
               "motion that is NOT path-following - zero in steady state, and "
               "the quantity KB Section 9.4.15 says drivers actually feel.");

    derivedRow("Lateral accel",  "%8.2f", s.lateralAcceleration, "m/s2");
    derivedRow("             ",  "%8.3f", s.lateralAcceleration / kGravity, "g");

    const double us = v.understeerRatio();
    ImGui::Text("%-20s %8.3f    %s", "Understeer ratio US", us,
                (us > 1.02) ? "understeer"
                            : ((us < 0.98) ? "OVERSTEER" : "neutral"));
    helpMarker("US = omega_geometric / omega_actual (KB Eq. 7.16). Above 1 the "
               "vehicle is turning less than its steering geometry asks for. "
               "NOT the same quantity as Gillespie's understeer gradient K.");

    const double Rp = v.pathRadius();
    const double Rg = v.turningRadius();
    if (std::isfinite(Rp) && std::abs(Rp) < 1.0e4) {
        derivedRow("Path radius", "%8.1f", std::abs(Rp), "m");
    }
    if (std::isfinite(Rg) && std::abs(Rg) < 1.0e4) {
        derivedRow("Ackerman radius", "%8.1f", std::abs(Rg), "m");
    }

    //-- the roll chain ---------------------------------------------------
    //
    //  Everything below is DERIVED from the ride frequencies, roll centre
    //  heights and bar rates in the vehicle config. None of it is authored,
    //  which is the whole point of the chain: before the third source, the
    //  front/rear split of load transfer had to be a bare number typed into
    //  the YAML.
    ImGui::Separator();
    ImGui::TextDisabled("Suspension - the roll chain (derived)");

    const SuspensionState su = computeSuspension(v.parameters());

    if (su.rollUnstable) {
        ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f},
                           "ROLL-UNSTABLE: total roll rate does not exceed W_s*h_2");
        helpMarker("The gravity moment of the rolled sprung mass exceeds the "
                   "suspension's ability to resist it, so the roll gradient is "
                   "unbounded. Gillespie calls this 'a genuine physical "
                   "instability, not a modelling artefact'. The gradient is "
                   "clamped so the simulation stays finite.");
    }

    derivedRow("Roll gradient",  "%8.2f", radToDeg(std::abs(su.rollGradient)), "deg/g");
    helpMarker("Milliken Table 16.1 by class: very soft 8.5, soft 7.5, "
               "semi-soft 7.0, semi-firm 6.0, firm 5.0, very firm 4.2, "
               "extremely firm 3.0, racing cars only 1.5. Gillespie's range "
               "for road cars is 3-7 deg/g, consistent with the middle of it.");

    derivedRow("Sprung weight",  "%8.0f", su.sprungWeight, "N");
    derivedRow("Sprung CG h_s",  "%8.3f", su.sprungCgHeight, "m");
    derivedRow("CG above roll axis h_2", "%8.3f", su.rollMomentArm, "m");
    derivedRow("Roll rate front","%8.0f", su.rollRateFront, "N.m/rad");
    derivedRow("Roll rate rear", "%8.0f", su.rollRateRear,  "N.m/rad");

    ImGui::Text("%-24s %6.1f %%", "Roll rate dist. front",
                100.0 * su.rollRateDistributionFront);
    ImGui::Text("%-24s %6.1f %%", "Load transfer dist. front",
                100.0 * su.loadTransferDistributionFront);
    helpMarker("These are NOT the same number, and treating them as if they "
               "were is a named modelling error - Milliken p.679: 'total "
               "lateral load transfer distribution is sometimes erroneously "
               "called roll couple distribution. The problem is that roll "
               "couple distribution is only part of the mechanism.' In "
               "Milliken's own example the two differ by ten percentage "
               "points.");

    if (ImGui::TreeNode("Load transfer breakdown [N per g]")) {
        ImGui::Text("            elastic  geometric  unsprung     total");
        ImGui::Text("front      %8.0f   %8.0f  %8.0f  %8.0f",
                    su.elasticFrontPerG, su.geometricFrontPerG,
                    su.unsprungFrontPerG, su.loadTransferFrontPerG);
        ImGui::Text("rear       %8.0f   %8.0f  %8.0f  %8.0f",
                    su.elasticRearPerG, su.geometricRearPerG,
                    su.unsprungRearPerG, su.loadTransferRearPerG);
        helpMarker("Three mechanisms, not two. Gillespie Eq. 6-27 gives the "
                   "elastic path (through the springs and bars, needing the "
                   "body to actually roll) and the geometric one (through the "
                   "links at the roll centre, instantaneous). Milliken adds "
                   "the third: the unsprung mass transferring load about its "
                   "own CG height.");
        ImGui::TreePop();
    }
}

//---------------------------------------------------------------------------
//  Road load panel
//
//  Shows which resistance term dominates at the current speed. This is the
//  single most instructive thing about road load: rolling resistance rules at
//  low speed, aerodynamic drag takes over around 50-60 mph, and the crossover
//  is why fuel economy falls off a cliff on the motorway.
//---------------------------------------------------------------------------

void DebugUI::drawRoadLoadPanel(Simulation& sim) {
    if (!ImGui::CollapsingHeader("Road load (resistance)")) return;

    Vehicle& v = sim.vehicle();
    VehicleParameters& p = v.mutableParameters();
    const RoadLoad& rl = v.roadLoad();
    const double speed = std::abs(v.state().velocity);

    ImGui::TextDisabled("Live breakdown at %.1f m/s (%.0f km/h)",
                        speed, mpsToKph(speed));

    const double total = rl.resistiveTotal();

    // Stacked bars, each scaled against the total, so the dominant term is
    // obvious at a glance.
    auto term = [&](const char* label, double value, const ImVec4& color) {
        const float frac = (total > 1e-9) ? static_cast<float>(value / total) : 0.0f;
        char overlay[64];
        std::snprintf(overlay, sizeof(overlay), "%.4f m/s2  (%.0f%%)",
                      value, frac * 100.0f);
        ImGui::Text("%-18s", label);
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
        ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), overlay);
        ImGui::PopStyleColor();
    };

    term("Rolling resist.", rl.rollingResistance, {0.35f, 0.75f, 0.45f, 1.0f});
    term("Aerodynamic drag", rl.aerodynamicDrag,  {0.35f, 0.60f, 0.95f, 1.0f});
    term("Engine braking",   rl.engineBraking,    {0.85f, 0.65f, 0.30f, 1.0f});

    ImGui::Separator();
    ImGui::Text("Total resistance   %.4f m/s2   (%.3f g)",
                total, total / kGravity);
    if (std::abs(rl.grade) > 1e-6) {
        ImGui::Text("Grade              %+.4f m/s2  (%s)",
                    rl.grade, rl.grade > 0.0 ? "uphill, resists" : "downhill, assists");
    }
    ImGui::Text("Drag force         %8.1f N", rl.dragForce);
    ImGui::Text("Rolling force      %8.1f N", rl.rollingForce);
    ImGui::Text("Effective f_r      %8.5f", rl.effectiveFr);

    derivedRow("Drag = rolling at", "%8.1f", v.parameters().dragCrossoverSpeed(
                   v.environment().airDensity), "m/s");
    helpMarker("Speed at which aerodynamic drag equals rolling resistance. "
               "Gillespie Ch.4 puts this at roughly 50-60 mph (22-27 m/s) for "
               "a passenger car.");
    derivedRow("Coast-down at 30 m/s", "%8.3f",
               v.parameters().coastDownDeceleration(30.0,
                   v.environment().airDensity), "m/s2");
    helpMarker("Steady deceleration when coasting at 30 m/s, excluding engine "
               "braking. A passenger car sits around 0.04 g (0.4 m/s^2).");

    //-- editable parameters ----------------------------------------------
    ImGui::Separator();
    ImGui::TextDisabled("Vehicle road load parameters");

    bool changed = false;

    float fr = static_cast<float>(p.rollingResistanceCoeff);
    if (ImGui::SliderFloat("Rolling resist. f_r", &fr, 0.0f, 0.32f, "%.4f",
                           ImGuiSliderFlags_Logarithmic)) {
        p.rollingResistanceCoeff = fr; changed = true;
    }
    tagLive();
    helpMarker("0.015 = passenger car on concrete, 0.08 = medium-hard soil, "
               "0.30 = sand (Gillespie Ch.4). Set to 0 to disable.");

    float cd = static_cast<float>(p.dragCoefficient);
    if (ImGui::SliderFloat("Drag coeff. C_D", &cd, 0.0f, 1.2f, "%.3f")) {
        p.dragCoefficient = cd; changed = true;
    }
    tagLive();
    helpMarker("Only meaningful together with the frontal area. Cars run "
               "roughly 0.32 (modern) to 0.85 (1920s).");

    float area = static_cast<float>(p.frontalArea);
    if (ImGui::SliderFloat("Frontal area", &area, 0.0f, 12.0f, "%.2f m2")) {
        p.frontalArea = area; changed = true;
    }
    tagLive();

    float eb = static_cast<float>(p.engineBrakingCoeff);
    if (ImGui::SliderFloat("Engine braking", &eb, 0.0f, 0.15f, "%.4f /s")) {
        p.engineBrakingCoeff = eb; changed = true;
    }
    tagLive();
    helpMarker("NOT from Gillespie - a stand-in for overrun losses in gear. "
               "Deceleration per unit speed. Set to 0 to coast as if in "
               "neutral.");

    if (changed) p.validate();

    //-- environment -------------------------------------------------------
    ImGui::Separator();
    ImGui::TextDisabled("Environment");

    EnvironmentConditions& env = v.mutableEnvironment();

    float rho = static_cast<float>(env.airDensity);
    if (ImGui::SliderFloat("Air density", &rho, 0.0f, 1.6f, "%.3f kg/m3")) {
        env.airDensity = rho;
    }
    tagLive();
    helpMarker("1.225 at sea level and 15 C. Thinner air at altitude reduces "
               "drag (and, in a real car, engine power).");

    float gradeDeg = static_cast<float>(radToDeg(env.gradeAngle));
    if (ImGui::SliderFloat("Grade", &gradeDeg, -20.0f, 20.0f, "%.1f deg")) {
        env.gradeAngle = degToRad(gradeDeg);
    }
    tagLive();
    helpMarker("Positive is uphill. The ground is drawn flat regardless - this "
               "applies the gravity component only, so the vehicle will slow "
               "or roll away without the scenery agreeing with it.");

    float wind = static_cast<float>(env.headwind);
    if (ImGui::SliderFloat("Headwind", &wind, 0.0f, 40.0f, "%.1f m/s")) {
        env.headwind = wind;
    }
    tagLive();

    if (ImGui::Button("Disable all resistance (coast forever)", {-1, 0})) {
        p.rollingResistanceCoeff = 0.0;
        p.dragCoefficient        = 0.0;
        p.engineBrakingCoeff     = 0.0;
        env.gradeAngle           = 0.0;
        env.headwind             = 0.0;
        p.validate();
    }
    helpMarker("Restores the original behaviour: with no resistive force, "
               "Newton's first law keeps the vehicle at constant speed "
               "indefinitely once the throttle is released.");
}

//---------------------------------------------------------------------------
//  Controls panel
//---------------------------------------------------------------------------

void DebugUI::drawControlsPanel(Simulation& sim) {
    if (!ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const Vehicle& v = sim.vehicle();
    const ControlInput& req = v.requestedInput();
    const ControlInput& app = v.appliedInput();

    ImGui::TextDisabled("Requested (driver input)");
    controlBar("Throttle", req.throttle, {0.30f, 0.80f, 0.40f, 1.0f});
    controlBar("Brake",    req.brake,    {0.85f, 0.30f, 0.30f, 1.0f});
    bipolarBar("Steering", req.steering, {0.45f, 0.55f, 0.95f, 1.0f});

    ImGui::Spacing();
    ImGui::TextDisabled("Applied (after actuator rate limits)");
    controlBar("Throttle", app.throttle, {0.25f, 0.65f, 0.35f, 1.0f});
    controlBar("Brake",    app.brake,    {0.70f, 0.25f, 0.25f, 1.0f});
    ImGui::Text("%-10s %+.2f deg  (max %.1f)", "Steering",
                radToDeg(app.steering), radToDeg(v.parameters().maxSteeringAngle));

    if (v.emergencyBrake()) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "EMERGENCY BRAKE");
    }

    ImGui::Separator();
    ControlsConfig& cc = sim.vehicle().mutableControls();

    float thr = static_cast<float>(cc.throttleRate);
    if (ImGui::SliderFloat("Throttle rate", &thr, 0.2f, 20.0f, "%.2f /s")) {
        cc.throttleRate = thr;
    }
    tagLive();
    helpMarker("How fast the throttle ramps toward the requested value. "
               "2.0 means half a second from closed to fully open.");

    float brk = static_cast<float>(cc.brakeRate);
    if (ImGui::SliderFloat("Brake rate", &brk, 0.2f, 40.0f, "%.2f /s")) {
        cc.brakeRate = brk;
    }
    tagLive();

    float ret = static_cast<float>(radToDeg(cc.steeringReturnRate));
    if (ImGui::SliderFloat("Steering return", &ret, 10.0f, 1200.0f, "%.0f deg/s")) {
        cc.steeringReturnRate = degToRad(ret);
    }
    tagLive();
    helpMarker("Rate at which the steering returns to centre when no steering "
               "key is held. Usually faster than the turn-in rate.");
}

//---------------------------------------------------------------------------
//  Powertrain panel  --  only meaningful for a vehicle using the detailed
//  engine (physics/Powertrain.h). Vehicles on the simple max_acceleration
//  demand get a one-line explanation instead of an empty box.
//---------------------------------------------------------------------------

void DebugUI::drawPowertrainPanel(Simulation& sim) {
    if (!ImGui::CollapsingHeader("Powertrain", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const Vehicle& v = sim.vehicle();
    const Powertrain* pt = v.powertrain();

    if (!pt) {
        ImGui::TextDisabled("No detailed engine on this vehicle.");
        ImGui::TextWrapped(
            "Drive is a force demand: throttle x max_acceleration = %.1f m/s^2. "
            "Add an 'engine: detailed:' block to its YAML for a real engine "
            "and gearbox.", v.parameters().maxAcceleration);
        return;
    }

    const PowertrainOutput& o = pt->lastOutput();
    const EngineParameters& ep = pt->engine().parameters();
    const GearboxParameters& gp = pt->gearbox().parameters();

    //-- gear, big and unmissable ------------------------------------------
    const int gearCount = static_cast<int>(gp.gearRatios.size());
    const bool neutral = (o.currentGear <= kNeutralGear);
    ImGui::Text("Gear");
    ImGui::SameLine();
    if (neutral) {
        ImGui::TextColored({0.95f, 0.85f, 0.45f, 1.0f}, "N");
        ImGui::SameLine();
        ImGui::TextDisabled("neutral - no drive, engine free");
    } else {
        ImGui::TextColored({0.55f, 0.85f, 1.0f, 1.0f}, "%d", o.currentGear + 1);
        ImGui::SameLine();
        ImGui::TextDisabled("/ %d   (ratio %.2f x %.2f final)", gearCount,
                            gp.gearRatios[static_cast<std::size_t>(o.currentGear)],
                            gp.finalDrive);
    }

    //-- modes, switchable live --------------------------------------------
    //
    // Mutable at runtime rather than config-only: the entire point of having
    // the modes is to compare them, and reloading the vehicle to do that
    // would reset the car's state along with it.
    GearboxParameters& gpMut = sim.vehicle().mutableParameters().gearbox;
    {
        int shiftIdx = (gpMut.shiftMode == ShiftMode::Manual) ? 1 : 0;
        if (ImGui::Combo("Shifter", &shiftIdx, "Automatic\0Manual\0")) {
            gpMut.shiftMode = shiftIdx ? ShiftMode::Manual : ShiftMode::Automatic;
            sim.vehicle().syncPowertrainModes();
        }
        helpMarker("Automatic shifts itself against the configured RPM "
                   "thresholds. Manual puts you in charge: Z down, X up.");

        int clutchIdx = (gpMut.clutchMode == ClutchMode::Manual) ? 1 : 0;
        if (ImGui::Combo("Clutch", &clutchIdx, "Automatic\0Manual\0")) {
            gpMut.clutchMode = clutchIdx ? ClutchMode::Manual : ClutchMode::Automatic;
            sim.vehicle().syncPowertrainModes();
        }
        helpMarker("Automatic lets the clutch out for you on a launch. "
                   "Manual gives you the pedal: hold Left Shift.");
    }

    //-- clutch engagement --------------------------------------------------
    {
        const float eng = static_cast<float>(o.clutchEngagement);
        char lbl[48];
        std::snprintf(lbl, sizeof(lbl), "%.0f%% engaged", eng * 100.0f);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                              o.locked ? ImVec4{0.45f, 0.85f, 0.55f, 1.0f}
                                       : ImVec4{1.00f, 0.75f, 0.30f, 1.0f});
        ImGui::ProgressBar(eng, ImVec2(-FLT_MIN, 0), lbl);
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("clutch");
    }

    //-- tachometer --------------------------------------------------------
    const float rpmFrac = static_cast<float>(
        clampd(o.engineRpm / std::max(1.0, ep.redlineRpm), 0.0, 1.0));
    const bool nearRedline = (o.engineRpm > ep.redlineRpm * 0.90);
    const ImVec4 tachColor = o.limiterActive ? ImVec4{1.00f, 0.35f, 0.30f, 1.0f}
                           : nearRedline     ? ImVec4{1.00f, 0.70f, 0.25f, 1.0f}
                                             : ImVec4{0.35f, 0.80f, 0.45f, 1.0f};
    char rpmLabel[64];
    std::snprintf(rpmLabel, sizeof(rpmLabel), "%.0f rpm", o.engineRpm);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, tachColor);
    ImGui::ProgressBar(rpmFrac, ImVec2(-FLT_MIN, 0), rpmLabel);
    ImGui::PopStyleColor();
    ImGui::TextDisabled("idle %.0f    redline %.0f", ep.idleRpm, ep.redlineRpm);

    if (o.limiterActive) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "REV LIMITER");
    }

    ImGui::Separator();

    //-- what the engine is making right now -------------------------------
    const double powerKw = o.engineTorqueNm * o.engineRpm * kTwoPi / 60.0 / 1000.0;
    ImGui::Text("Engine torque   %8.1f N.m", o.engineTorqueNm);
    ImGui::Text("Engine power    %8.1f kW   (%.0f hp)", powerKw, powerKw * 1.34102);
    ImGui::Text("Drive           %8.2f m/s^2", o.driveAccelerationMps2);
    ImGui::Text("Engine braking  %8.2f m/s^2", o.engineBrakingMps2);
    if (ep.hasTurbo) {
        const Engine& eng = pt->engine();
        ImGui::Text("Boost           %8.2f bar  (max %.2f)", eng.boostPa() / 1e5, ep.maxBoostPa / 1e5);
        ImGui::ProgressBar(static_cast<float>(clampd(eng.boostPa() / ep.maxBoostPa, 0.0, 1.0)),
                           ImVec2(-FLT_MIN, 0), "");
        ImGui::TextDisabled("charge %.0f C after intercooler",
                            eng.lastBreakdown().chargeTemperatureK - 273.15);
        if (eng.isResolved()) {
            helpMarker("Boost is a state of the model: turbines in the exhaust "
                       "drive compressors through a shaft with inertia, and a "
                       "wastegate holds the target. Turbo lag is that shaft "
                       "spinning up - nothing here is a time constant. See "
                       "physics/ResolvedEngine.h.");
        } else {
            helpMarker("Turbo lag is a first-order lag toward a boost that rises "
                       "with engine speed; the charge temperature is Heywood "
                       "Eq 6.40 less the intercooler. See config/EngineParameters.h.");
        }
    }

    //-- inside a resolved engine ------------------------------------------
    //
    // Everything below is state the crank-angle-resolved core carries and the
    // cycle-averaged model has no equivalent of.
    if (const ResolvedEngine* re = pt->engine().resolved()) {
        const ResolvedEngine::Output& e = re->lastOutput();
        if (ImGui::TreeNodeEx("Engine internals (resolved core)", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* state = !e.firing      ? "CRANKING"
                              : e.limiterActive ? "FUEL CUT - limiter"
                              : e.fuelCut       ? "FUEL CUT - overrun"
                                                : "firing";
            ImGui::Text("State           %s", state);
            ImGui::Text("Manifold        %8.1f kPa", e.mapFilteredPa / 1e3);
            helpMarker("Intake manifold pressure, filtered over one firing "
                       "interval as the ECU sees it. Integrated from throttle "
                       "flow in and the cylinders' own intake flow out.");
            ImGui::Text("Exhaust         %8.1f kPa  %5.0f C", e.exhaustPressurePa / 1e3,
                        e.exhaustTemperatureK - 273.15);
            helpMarker("The plenum the exhaust valves discharge into. On a "
                       "turbo engine this is the back-pressure the turbines "
                       "cost: compare it with the manifold pressure.");
            if (e.fuelCut || !e.firing) ImGui::Text("Lambda               --");
            else                        ImGui::Text("Lambda          %8.3f", e.lambda);
            helpMarker("Air the cylinders actually trapped against the fuel the "
                       "ECU metered for what it BELIEVED they would trap. It "
                       "wanders off 1.0 in transients, as a real engine's does.");
            ImGui::Text("Vol. efficiency %8.2f", e.volumetricEfficiency);
            ImGui::Text("Spark           %8.1f deg BTDC", e.sparkTimingDegBtdc);
            ImGui::Text("IMEP / PMEP     %8.2f / %.2f bar", e.imepNetBar, e.pmepBar);
            ImGui::Text("Peak pressure   %8.1f bar", e.peakPressurePa / 1e5);
            ImGui::Text("Residual gas    %8.0f %%", e.residualFraction * 100.0);
            ImGui::Text("Idle-air valve  %8.2f %% of WOT area", e.iacPosition * 100.0);
            if (ep.resolved.vvtIntakeMaxDeg != ep.resolved.vvtIntakeMinDeg
                || ep.resolved.vvtExhaustMaxDeg != ep.resolved.vvtExhaustMinDeg) {
                ImGui::Text("Cam advance     %8.1f / %.1f deg (in / ex)", e.vvtIntakeDeg, e.vvtExhaustDeg);
            }
            if (ep.resolved.useTurbo) {
                ImGui::Text("Turbo speed     %8.0f krpm", e.turboSpeedRps * 60.0 / 1000.0);
                ImGui::Text("Wastegate       %8.0f %% open", e.wastegatePosition * 100.0);
            }
            if (ep.resolved.useKnockModel) {
                ImGui::Text("Knock retard    %8.1f deg", e.knockRetardDeg);
            }
            if (e.misfireRate > 0.005) {
                ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f}, "Misfires        %8.0f %%", e.misfireRate * 100.0);
            }

            // Cylinder 1's pressure over its 720 deg cycle - the pressure that
            // makes the torque, not a reconstruction.
            const std::vector<double>& trace = re->pressureTrace();
            static float traceBar[180];
            float traceMax = 1.0f;
            const int bins = std::min<int>(180, static_cast<int>(trace.size()));
            for (int i = 0; i < bins; ++i) {
                traceBar[i] = static_cast<float>(trace[static_cast<std::size_t>(i)] / 1e5);
                traceMax = std::max(traceMax, traceBar[i]);
            }
            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "cylinder 1  peak %.0f bar", traceMax);
            ImGui::PlotLines("##cylPressure", traceBar, bins, 0, overlay, 0.0f, traceMax * 1.1f,
                             ImVec2(-FLT_MIN, 60));
            ImGui::TextDisabled("intake | compression | power | exhaust   (%d substeps/step)", e.substeps);
            ImGui::TreePop();
        }
    }

    //-- clutch ------------------------------------------------------------
    if (o.locked) {
        ImGui::TextColored({0.45f, 0.85f, 0.55f, 1.0f}, "Clutch LOCKED");
        helpMarker("Driveline rigid: engine speed is wheel speed x gear ratio. "
                   "Torque flows both ways, so this is also where engine "
                   "braking comes from.");
    } else {
        ImGui::TextColored({1.0f, 0.75f, 0.30f, 1.0f}, "Clutch SLIPPING");
        helpMarker("Launch. The clutch transmits its capacity torque while the "
                   "two sides turn at different speeds; it locks once the "
                   "wheels catch the engine up. See physics/Powertrain.h.");
    }

    ImGui::Separator();

    //-- torque / power curve ------------------------------------------------
    //
    // CYCLE-AVERAGED engine: probed straight off the model at wide-open
    // throttle through evaluateTorque(), which is const and touches no state.
    //
    // RESOLVED engine: there is nothing to probe - its torque at a speed is
    // what comes out once manifold, turbo and ECU have settled there. So it
    // is MEASURED, on a private copy of the engine held at each speed in turn
    // (physics/EngineDyno.h), a few milliseconds per frame while this node is
    // open. Neither can perturb the running simulation.
    if (ImGui::TreeNodeEx("Torque / power curve (WOT)",
                          ImGuiTreeNodeFlags_DefaultOpen)) {
        constexpr int kSamples = 64;
        static float torqueCurve[kSamples];
        static float powerCurve[kSamples];
        float maxTorque = 1.0f, maxPower = 1.0f;
        int count = kSamples;
        const bool resolved = pt->engine().isResolved();

        const double rpmLo = ep.idleRpm;
        const double rpmHi = ep.redlineRpm;
        if (resolved) {
            if (!m_dyno || m_dynoVehicle != v.parameters().name) {
                m_dyno = std::make_unique<EngineDyno>(ep, 24);
                m_dynoVehicle = v.parameters().name;
            }
            m_dyno->advance(0.004);
            count = static_cast<int>(std::min<std::size_t>(m_dyno->measuredCount(), kSamples));
            for (int i = 0; i < count; ++i) {
                const EngineDyno::Point& p = m_dyno->points()[static_cast<std::size_t>(i)];
                torqueCurve[i] = static_cast<float>(p.torqueNm);
                powerCurve[i]  = static_cast<float>(p.powerKw);
                maxTorque = std::max(maxTorque, torqueCurve[i]);
                maxPower  = std::max(maxPower,  powerCurve[i]);
            }
        } else {
            for (int i = 0; i < kSamples; ++i) {
                const double rpm = rpmLo + (rpmHi - rpmLo) * (i / double(kSamples - 1));
                const double w = rpm * kTwoPi / 60.0;
                const double tq = pt->engine().evaluateTorque(w, 1.0, 0.0).netTorqueNm;
                torqueCurve[i] = static_cast<float>(tq);
                powerCurve[i]  = static_cast<float>(tq * w / 1000.0);
                maxTorque = std::max(maxTorque, torqueCurve[i]);
                maxPower  = std::max(maxPower,  powerCurve[i]);
            }
        }

        char overlay[64];
        std::snprintf(overlay, sizeof(overlay), "torque  peak %.0f N.m", maxTorque);
        ImGui::PlotLines("##torqueCurve", torqueCurve, count, 0, overlay,
                         0.0f, maxTorque * 1.1f, ImVec2(-FLT_MIN, 60));

        std::snprintf(overlay, sizeof(overlay), "power   peak %.0f kW", maxPower);
        ImGui::PlotLines("##powerCurve", powerCurve, count, 0, overlay,
                         0.0f, maxPower * 1.1f, ImVec2(-FLT_MIN, 60));

        if (resolved && !m_dyno->done()) {
            const std::size_t at = m_dyno->measuredCount();
            ImGui::TextColored({1.0f, 0.75f, 0.30f, 1.0f}, "dyno sweep: %zu / %zu points (at %.0f rpm)",
                               at, m_dyno->points().size(), m_dyno->points()[at].rpm);
        } else {
            ImGui::TextDisabled("%.0f - %.0f rpm, left to right", rpmLo, rpmHi);
        }
        ImGui::TextDisabled("bmep at peak torque: %.1f bar",
                            maxTorque * 4.0 * kPi / ep.totalDisplacement() / 1e5);
        if (resolved) {
            helpMarker("Measured, not calculated: a private copy of this engine "
                       "is held at each speed until its torque settles - "
                       "manifold filled, turbo spooled, air model converged - "
                       "exactly as on a dynamometer. Steady state, so no turbo "
                       "lag. Heywood puts naturally aspirated SI engines at "
                       "8.5-10.5 bar bmep. build/bin/engine_dyno prints the "
                       "same sweep with the numbers behind it.");
            if (m_dyno->done() && ImGui::SmallButton("Re-run sweep")) m_dyno.reset();
        } else {
            helpMarker("Brake mean effective pressure - the size-independent way "
                       "to sanity check an engine. Heywood puts naturally "
                       "aspirated SI engines at 8.5-10.5 bar; boost raises it "
                       "roughly with intake density. Curve is at steady-state "
                       "boost - no turbo lag.");
        }
        ImGui::TreePop();
    }
}

void DebugUI::drawAndroidPanel(const AndroidUsbInput* android) {
    // Nothing is drawn at all when the feature is off in config - this is
    // the "disable without affecting the rest of the simulator" requirement
    // made visible: an unconfigured user never even sees this section.
    if (!android) return;
    if (!ImGui::CollapsingHeader("Android Controller", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const bool connected = android->isConnected();
    ImGui::Text("Listening on 127.0.0.1:%d", android->port());
    if (connected) {
        ImGui::TextColored({0.4f, 0.9f, 0.4f, 1.0f}, "CONNECTED - driving from phone");
    } else {
        ImGui::TextColored({1.0f, 0.45f, 0.35f, 1.0f},
                           "DISCONNECTED - keyboard active, safe state applied");
    }

    const double age = android->secondsSinceLastPacket();
    if (std::isfinite(age)) {
        ImGui::Text("Last packet:      %.2f s ago", age);
    } else {
        ImGui::Text("Last packet:      never");
    }
    ImGui::Text("Packets received: %ld   malformed: %ld",
               android->packetsReceived(), android->malformedPackets());
    helpMarker("'malformed' counts lines that failed validation (wrong tag, "
               "wrong field count, non-numeric value) - see "
               "AndroidController/PROTOCOL.md. They are dropped, not applied.");

    ImGui::Separator();
    ImGui::TextDisabled("What the vehicle is receiving right now");
    const ControlInput c = android->controlInput();
    controlBar("Throttle", c.throttle, {0.30f, 0.80f, 0.40f, 1.0f});
    controlBar("Brake",    c.brake,    {0.85f, 0.30f, 0.30f, 1.0f});
    controlBar("Clutch",   c.clutch,   {0.90f, 0.70f, 0.25f, 1.0f});
    bipolarBar("Steering", c.steering, {0.45f, 0.55f, 0.95f, 1.0f});
    if (android->handbrake()) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "HANDBRAKE");
    }
    helpMarker("These are the post-safety values: zero/centred whenever "
               "disconnected or stale, exactly as fed to the vehicle - not "
               "necessarily the raw last packet. Gear-shift requests are "
               "consumed as a one-shot edge by the driving loop, not shown "
               "here, so this panel reading it too would never see one.");
}

//---------------------------------------------------------------------------
//  Experiment panel
//---------------------------------------------------------------------------

void DebugUI::drawExperimentPanel(Simulation& sim, UiRequests& req) {
    if (!ImGui::CollapsingHeader("Experiments")) return;

    ExperimentConfig& cfg = sim.appConfig().experiment;

    static int selected = 0;
    const std::vector<std::string> names = availableExperiments();

    std::vector<const char*> items;
    items.reserve(names.size());
    for (const std::string& n : names) items.push_back(n.c_str());

    // Keep the combo in sync with the configured type on first draw.
    static bool initialised = false;
    if (!initialised) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            if (names[i] == cfg.type) { selected = static_cast<int>(i); break; }
        }
        initialised = true;
    }

    ImGui::Combo("Type", &selected, items.data(), static_cast<int>(items.size()));

    float v0 = static_cast<float>(cfg.initialSpeed);
    if (ImGui::SliderFloat("Initial speed", &v0, 0.0f, 60.0f, "%.1f m/s")) {
        cfg.initialSpeed = v0;
    }

    float steer = static_cast<float>(radToDeg(cfg.steeringAngle));
    if (ImGui::SliderFloat("Steering angle", &steer, -40.0f, 40.0f, "%.1f deg")) {
        cfg.steeringAngle = degToRad(steer);
    }
    helpMarker("Road wheel angle. Positive steers RIGHT.");

    float dur = static_cast<float>(cfg.duration);
    if (ImGui::SliderFloat("Duration", &dur, 1.0f, 120.0f, "%.1f s")) {
        cfg.duration = dur;
    }

    float st = static_cast<float>(cfg.stepTime);
    if (ImGui::SliderFloat("Step time", &st, 0.0f, 30.0f, "%.2f s")) {
        cfg.stepTime = st;
    }
    helpMarker("When the step is applied, for step_steering and braking_test.");

    float per = static_cast<float>(cfg.period);
    if (ImGui::SliderFloat("Slalom period", &per, 0.5f, 20.0f, "%.2f s")) {
        cfg.period = per;
    }

    ImGui::Separator();

    if (sim.experimentRunning() && sim.experiment()) {
        const Experiment* e = sim.experiment();
        ImGui::TextColored({0.4f, 0.85f, 1.0f, 1.0f}, "Running: %s",
                           experimentTypeToString(e->type()));
        ImGui::TextWrapped("%s", e->description().c_str());
        ImGui::ProgressBar(
            static_cast<float>(sim.experimentTime() / std::max(1e-6, e->config().duration)),
            ImVec2(-1, 0));
        if (ImGui::Button("Stop experiment", {-1, 0})) req.stopExperiment = true;
    } else {
        if (ImGui::Button("Run experiment", {-1, 0})) {
            req.startExperiment = true;
            req.experimentToStart = experimentTypeFromString(names[selected]);
            cfg.type = names[selected];
        }
        ImGui::TextDisabled("Resets the vehicle, starts telemetry recording,");
        ImGui::TextDisabled("and drives the scripted input to completion.");
    }
}

//---------------------------------------------------------------------------
//  Telemetry panel
//---------------------------------------------------------------------------

void DebugUI::drawTelemetryPanel(Simulation& sim, UiRequests& req) {
    if (!ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) return;

    Telemetry& t = sim.telemetry();

    bool rec = t.isRecording();
    if (ImGui::Checkbox("Record (T)", &rec)) t.setRecording(rec);
    ImGui::SameLine();
    ImGui::Text("%zu samples", t.sampleCount());

    if (t.isRecording()) {
        const double dt = sim.simConfig().timestep;
        ImGui::Text("Rate %.0f Hz   duration %.2f s",
                    (dt > 0.0) ? 1.0 / dt : 0.0,
                    static_cast<double>(t.sampleCount()) * dt);
    }

    ImGui::BeginDisabled(t.sampleCount() == 0);
    if (ImGui::Button("Export CSV (F2)", {-1, 0})) req.exportTelemetry = true;
    ImGui::EndDisabled();

    if (!t.lastExportPath().empty()) {
        ImGui::TextDisabled("Last export:");
        ImGui::TextWrapped("%s", t.lastExportPath().c_str());
    }
    if (!t.lastError().empty()) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "%s", t.lastError().c_str());
    }

    ImGui::Separator();
    ImGui::Checkbox("Show graphs", &m_showGraphs);

    int cap = static_cast<int>(t.plotCapacity());
    if (ImGui::SliderInt("Graph window", &cap, 100, 6000, "%d samples")) {
        t.setPlotCapacity(static_cast<std::size_t>(cap));
    }
    ImGui::TextDisabled("= %.1f s at the current timestep",
                        static_cast<double>(cap) * sim.simConfig().timestep);
}

//---------------------------------------------------------------------------
//  Graphs
//---------------------------------------------------------------------------

void DebugUI::drawGraphsPanel(Simulation& sim) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x + 12.0f,
                             vp->WorkPos.y + vp->WorkSize.y - 12.0f},
                            ImGuiCond_FirstUseEver, {0.0f, 1.0f});
    ImGui::SetNextWindowSize({std::min(430.0f, vp->WorkSize.x * 0.34f),
                              std::min(420.0f, vp->WorkSize.y * 0.42f)},
                             ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Graphs", &m_showGraphs)) { ImGui::End(); return; }

    const Telemetry::PlotBuffers& p = sim.telemetry().plots();

    // Fixed plot height so the four graphs stay comparable in scale.
    const ImVec2 size(-1.0f, 78.0f);

    auto plot = [&](const char* label, const std::vector<float>& data,
                    const char* unit) {
        if (data.empty()) {
            ImGui::TextDisabled("%s: no data", label);
            return;
        }
        const float last = data.back();

        // Autoscale with a little headroom, and never collapse to a zero-height
        // range - a flat line at exactly zero would otherwise fill the plot.
        float lo = data[0], hi = data[0];
        for (float v : data) { lo = std::min(lo, v); hi = std::max(hi, v); }
        const float pad = std::max(0.5f, (hi - lo) * 0.12f);
        lo -= pad; hi += pad;

        char overlay[96];
        std::snprintf(overlay, sizeof(overlay), "%s: %.2f %s", label, last, unit);

        ImGui::PlotLines(label, data.data(), static_cast<int>(data.size()),
                         0, overlay, lo, hi, size);
    };

    plot("Speed",        p.speed,        "km/h");
    plot("Acceleration", p.acceleration, "m/s2");
    plot("Steering",     p.steering,     "deg");
    plot("Yaw rate",     p.yawRate,      "deg/s");

    ImGui::Separator();
    ImGui::TextDisabled("Sampled at the physics rate (%.0f Hz), not the frame rate.",
                        (sim.simConfig().timestep > 0.0)
                            ? 1.0 / sim.simConfig().timestep : 0.0);

    ImGui::End();
}

//---------------------------------------------------------------------------
//  Debug visualisation panel
//---------------------------------------------------------------------------

void DebugUI::drawDebugVisualisationPanel(DebugDrawOptions& opt, Renderer& renderer) {
    if (!ImGui::CollapsingHeader("Debug visualisation")) return;

    ImGui::Checkbox("Vehicle coordinate frame", &opt.vehicleFrame);
    ImGui::Checkbox("World coordinate frame",   &opt.worldFrame);
    ImGui::Checkbox("Centre of mass",           &opt.centreOfMass);
    ImGui::Checkbox("Velocity vector",          &opt.velocityVector);
    ImGui::Checkbox("Heading vector",           &opt.headingVector);
    ImGui::Checkbox("Steering direction",       &opt.steeringVector);
    ImGui::Checkbox("Wheel contact points",     &opt.wheelContacts);
    ImGui::Checkbox("Trajectory",               &opt.trajectory);
    ImGui::Checkbox("Turning radius",           &opt.turningRadius);
    helpMarker("Draws the instantaneous turn circle of radius L / tan(delta), "
               "centred on the extension of the rear axle. This is the circle "
               "the kinematic model says the vehicle is following right now.");
    ImGui::Checkbox("Ground grid",              &opt.grid);

    ImGui::SliderFloat("Vector scale", &opt.vectorScale, 0.05f, 2.0f, "%.2f");
    helpMarker("Metres of arrow per m/s of velocity.");

    ImGui::Separator();
    ImGui::TextDisabled("Axis colours");
    ImGui::TextColored({0.95f, 0.30f, 0.30f, 1.0f}, "  red    = +X  forward");
    ImGui::TextColored({0.35f, 0.90f, 0.40f, 1.0f}, "  green  = +Y  up");
    ImGui::TextColored({0.35f, 0.55f, 1.00f, 1.0f}, "  blue   = +Z  right");

    ImGui::Separator();
    RenderStyle& s = renderer.style();
    ImGui::Checkbox("Show road", &s.showRoad);
    helpMarker("Racing surface, paint, kerbs and runoff - everything generated "
               "from the track centreline by rendering/TrackMesh.h.");
    ImGui::SameLine();
    ImGui::Checkbox("Scenery", &s.showScenery);
    helpMarker("Barriers, gantry, pit complex, grandstands, marker boards and trees.");

    ImGui::Checkbox("Shadows", &s.shadows);
    helpMarker("Directional shadow map fitted around the vehicle. Also the only "
               "cue for how far the body has heaved and whether a wheel has "
               "lifted - see rendering/ShadowMap.h.");
    ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &s.wireframe);

    if (renderer.venueActive()) {
        // A venue circuit is lit through its own pipeline (linear light, tone
        // mapped - see rendering/Venue.h) and has its own set of numbers.
        ImGui::SliderFloat("Exposure", &s.venueExposure, 0.3f, 2.0f, "%.2f");
        ImGui::SliderFloat("Sun", &s.venueSunIntensity, 0.5f, 6.0f, "%.2f");
        ImGui::SliderFloat("Sky light", &s.venueAmbient, 0.0f, 2.5f, "%.2f");
        ImGui::SliderFloat("Haze", &s.venueFogDensity, 0.0f, 0.0015f, "%.5f");
        ImGui::SliderFloat("Shadow range", &s.venueShadowRadius, 10.0f, 80.0f, "%.0f m");
        helpMarker("The sharp shadow map round the car. A second, wider map "
                   "(buildings, stands, trees) takes over beyond it.");
        const FrameStats st = renderer.lastFrameStats();
        ImGui::TextDisabled("venue: %d draw calls, %lld k triangles", st.drawCalls,
                            st.triangles / 1000);
    } else {
        ImGui::SliderFloat("Ambient light", &s.ambient, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Shadow range", &s.shadowRadius, 10.0f, 120.0f, "%.0f m");
        helpMarker("Half-width of the area the shadow map covers. Smaller puts more "
                   "texels on the car and sharpens its shadow; larger reaches more "
                   "of the scenery.");
        ImGui::SliderFloat("Fog", &s.fogDensity, 0.0f, 0.004f, "%.4f");
    }
    ImGui::ColorEdit3("Body colour", &s.bodyColor.x);
    helpMarker("Rebuilds the car mesh: the generated model bakes colour into its "
               "vertices rather than reading a uniform.");
    ImGui::ColorEdit3("Sky colour",  &s.skyColor.x);
}

//---------------------------------------------------------------------------
//  Camera panel
//---------------------------------------------------------------------------

void DebugUI::drawCameraPanel(Camera& camera) {
    if (!ImGui::CollapsingHeader("Camera")) return;

    const auto& order = cameraCycleOrder();
    if (ImGui::BeginCombo("Mode (V, 1-0)", cameraModeName(camera.mode()))) {
        for (std::size_t i = 0; i < order.size(); ++i) {
            char label[64];
            std::snprintf(label, sizeof(label), "%zu  %s", (i + 1) % 10, cameraModeName(order[i]));
            if (ImGui::Selectable(label, order[i] == camera.mode())) camera.setMode(order[i]);
        }
        ImGui::EndCombo();
    }
    helpMarker("Chase views orbit with a mouse drag. Cockpit, bonnet, bumper "
               "and wheel are bolted to the body - they pitch and roll with "
               "it; drag to turn your head, scroll to change the field of "
               "view. TV cuts between fixed trackside cameras.");

    CameraConfig& c = camera.config();

    float dist = static_cast<float>(c.distance);
    if (ImGui::SliderFloat("Distance", &dist, 1.5f, 60.0f, "%.1f m")) c.distance = dist;

    float h = static_cast<float>(c.height);
    if (ImGui::SliderFloat("Height", &h, 0.2f, 40.0f, "%.1f m")) c.height = h;

    float fs = static_cast<float>(c.followSpeed);
    if (ImGui::SliderFloat("Follow speed", &fs, 0.2f, 30.0f, "%.1f /s")) c.followSpeed = fs;
    helpMarker("Exponential smoothing rate: alpha = 1 - exp(-k*dt). Higher is "
               "stiffer. Runs on render time, not physics time.");

    float la = static_cast<float>(c.lookAhead);
    if (ImGui::SliderFloat("Look ahead", &la, 0.0f, 25.0f, "%.1f m")) c.lookAhead = la;

    float fov = static_cast<float>(c.fovDegrees);
    if (ImGui::SliderFloat("Field of view", &fov, 20.0f, 110.0f, "%.0f deg")) {
        c.fovDegrees = fov;
    }

    if (ImGui::Button("Reset camera (C)", {-1, 0})) camera.resetToDefaults();

    if (camera.mode() == CameraMode::Free) {
        ImGui::TextDisabled("Free camera: IJKL to move, Q/E down/up,");
        ImGui::TextDisabled("drag to look, scroll to change speed.");
    }
}

//---------------------------------------------------------------------------
//  Sound panel
//---------------------------------------------------------------------------

void DebugUI::drawSoundPanel(AudioOutput* audio) {
    if (!ImGui::CollapsingHeader("Sound")) return;
    if (!audio) {
        ImGui::TextDisabled("No sound: disabled in config, or no audio device.");
        return;
    }
    audio->withSynth([](SoundSynth& synth) {
        SoundSynth::Mix& m = synth.mix();
        ImGui::Checkbox("Mute (M)", &m.muted);
        ImGui::SliderFloat("Master", &m.master, 0.0f, 1.5f, "%.2f");
        const bool engineSim = synth.usesEngineSim();
        ImGui::SliderFloat(engineSim ? "Engine (EngineSim)" : "Exhaust", &m.exhaust, 0.0f, 2.0f, "%.2f");
        if (!engineSim) {
            ImGui::SliderFloat("Intake / engine", &m.intake, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("Engine vibration", &m.structure, 0.0f, 2.0f, "%.2f");
        }
        ImGui::SliderFloat("Turbo", &m.turbo, 0.0f, 2.0f, "%.2f");
        ImGui::SliderFloat("Gearbox", &m.transmission, 0.0f, 2.0f, "%.2f");
        ImGui::SliderFloat("Tyres", &m.tyres, 0.0f, 2.0f, "%.2f");
        ImGui::SliderFloat("Wind", &m.wind, 0.0f, 2.0f, "%.2f");

        //-- EngineSim's dials (its README 6.6) ------------------------------
        //
        // The same fourteen, with the same ranges, defaults and presets as
        // EngineSim's dashboard. They start from the vehicle's
        // `sound: engine_sim: profile:` and go back to it on a reload.
        if (engineSim && ImGui::TreeNodeEx("Engine sound profile", ImGuiTreeNodeFlags_DefaultOpen)) {
            EngineSimSynth::Profile p = synth.engineSimProfile();
            bool changed = false;

            struct Preset { const char* name; const char* what; EngineSimSynth::Profile v; };
            auto preset = [](const char* name, const char* what, auto&& edit) {
                Preset out{name, what, EngineSimSynth::Profile{}};
                edit(out.v);
                return out;
            };
            // dashboard/ui.js SOUND_PRESETS: each lists only what differs from Stock.
            static const Preset presets[] = {
                preset("Stock", "The default sound.", [](auto&) {}),
                preset("Muscle", "Fat bass, long ring and a lumpy idle.", [](auto& v) {
                    v.lf = 5; v.hf = -1; v.ring = 0.7; v.rasp = 0.55; v.lope = 0.8; v.comp = 0.7;
                    v.drive = 0.15; v.nlf = 0.25; v.conv = 0.45; }),
                preset("Track", "Bright, raspy and even; a race engine.", [](auto& v) {
                    v.lf = -2; v.hf = 4; v.ring = 0.5; v.rasp = 0.85; v.lope = 0.1; v.comp = 0.8;
                    v.tick = 0.15; v.drive = 0.1; v.intk = 0.3; v.conv = 0.2; }),
                preset("Smooth", "Soft, quiet and refined.", [](auto& v) {
                    v.vol = 0.55; v.lf = 2; v.hf = -4; v.ring = 0.45; v.rasp = 0.25; v.lope = 0.05;
                    v.comp = 0.5; v.conv = 0.4; }),
                preset("Vintage", "Warm and gritty, with tappets and room.", [](auto& v) {
                    v.lf = 3; v.hf = -6; v.ring = 0.7; v.rasp = 0.4; v.lope = 0.7; v.drive = 0.45;
                    v.comp = 0.8; v.conv = 0.6; v.tick = 0.3; }),
                preset("Rumble", "Subsonic thunder for headphones and subs.", [](auto& v) {
                    v.lf = 9; v.hf = -2; v.ring = 0.8; v.rasp = 0.35; v.lope = 0.9; v.nlf = 0.8;
                    v.comp = 0.7; v.conv = 0.5; }),
            };
            for (std::size_t i = 0; i < sizeof(presets) / sizeof(presets[0]); ++i) {
                if (i % 3 != 0) ImGui::SameLine();
                if (ImGui::SmallButton(presets[i].name)) { p = presets[i].v; changed = true; }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", presets[i].what);
            }

            auto dial = [&changed](const char* label, double& v, double lo, double hi,
                                   const char* fmt, const char* help) {
                changed |= ImGui::SliderScalar(label, ImGuiDataType_Double, &v, &lo, &hi, fmt);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", help);
            };
            ImGui::TextDisabled("mix");
            dial("VOL  volume", p.vol, 0.0, 1.0, "%.2f", "Overall loudness of the engine.");
            dial("CONV room", p.conv, 0.0, 1.0, "%.2f", "Early reflections off nearby walls; above 0.35 a reverb tail.");
            dial("COMP compressor", p.comp, 0.0, 1.0, "%.2f", "Evens out loudness. Low: idle quiet, revs loud. High: everything close to the same.");
            dial("DRIVE saturation", p.drive, 0.0, 1.0, "%.2f", "Soft-clips the signal: warmer, then gritty, then distorted.");
            ImGui::TextDisabled("tone");
            dial("+LF  bass", p.lf, -12.0, 12.0, "%+.1f dB", "The deep thump below about 160 Hz.");
            dial("+HF  treble", p.hf, -12.0, 12.0, "%+.1f dB", "Bite, edge and air above about 2.5 kHz.");
            dial("RING exhaust resonance", p.ring, 0.0, 1.0, "%.2f", "How long each pulse keeps ringing in the pipes. Low: a stuffed muffler. High: an open straight pipe.");
            dial("RASP exhaust edge", p.rasp, 0.0, 1.0, "%.2f", "How much of each firing's sharp pressure edge the tailpipe radiates.");
            ImGui::TextDisabled("character");
            dial("LOPE unevenness", p.lope, 0.0, 1.0, "%.2f", "How uneven the cylinders and the crank speed are. High: a lumpy, cammed idle.");
            dial("~LF  rumble", p.nlf, 0.0, 1.0, "%.2f", "Adds deep rumble, growing with rpm and load.");
            dial("~HF  air", p.nhf, 0.0, 1.0, "%.2f", "Adds a steady rush of air and hiss, growing with rpm and throttle.");
            dial("TICK valvetrain", p.tick, 0.0, 1.0, "%.2f", "Mechanical clatter of valves and lifters.");
            dial("INTK intake", p.intk, 0.0, 1.0, "%.2f", "Induction roar and the suction pulses of the intake valves.");
            dial("POPS overrun", p.pops, 0.0, 1.0, "%.2f", "How often the exhaust pops when you lift off at speed. 0 = never.");
            if (changed) synth.setEngineSimProfile(p);
            ImGui::TreePop();
        }

        const SoundSynth::Diagnostics& d = synth.diagnostics();
        ImGui::Separator();
        ImGui::TextDisabled("Derived from the physics this frame:");
        ImGui::Text("Firing frequency   %7.1f Hz", d.firingFrequencyHz);
        if (!engineSim) {
            ImGui::Text("Blowdown dp        %7.2f bar", d.blowdownDeltaBar);
            ImGui::Text("Exhaust gas        %7.0f K", d.exhaustGasTempK);
            ImGui::Text("Pipe c/4L          %7.1f Hz", d.pipeFundamentalHz);
            ImGui::Text("Air box Helmholtz  %7.1f Hz", d.airboxHz);
            ImGui::Text("2nd-order shaking  %7.0f N", d.shakingForceN);
        }
        if (d.turboShaftRpm > 0.0) {
            ImGui::Text("Turbo shaft        %7.0f rpm", d.turboShaftRpm);
            ImGui::Text("Blade pass         %7.0f Hz", d.turboBladePassHz);
        }
        ImGui::Text("Gear mesh          %7.0f Hz", d.gearWhineHz);
        ImGui::Text("Doppler            %7.3f", d.dopplerRatio);
        ImGui::ProgressBar(static_cast<float>(std::min(d.peakLevel, 1.0)), ImVec2(-FLT_MIN, 0), "output peak");
    });
    helpMarker("Every sound is synthesised from the simulation - no "
               "recordings. Pulses fire on the engine's own crank angle, "
               "sized by the blowdown pressure its Otto cycle computes; the "
               "exhaust pipe is a waveguide at the speed of sound in the "
               "exhaust gas. See src/audio/SoundSynth.h. A vehicle with a "
               "`sound: engine_sim:` block is instead heard through "
               "EngineSim's own engine synthesiser, with its dials - see "
               "src/audio/EngineSimSynth.h.");
}

} // namespace vsim

