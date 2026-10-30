#include "application/Application.h"

#include "audio/SoundStateBuilder.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>

#include "core/Coordinates.h"
#include "rendering/GLLoader.h"

namespace vsim {
namespace {

void glfwErrorCallback(int code, const char* description) {
    std::cerr << "[glfw] error " << code << ": " << description << "\n";
}

/// A reasonable DEFAULT spawn point for an environment model with no
/// drivable centreline of its own. Not a real driving line - just a
/// location that is (a) real scanned geometry, not an unmapped gap, and
/// (b) locally flat, i.e. not next to a tall trackside object.
///
/// (b) matters specifically because this loader (rendering/GltfModel.h)
/// has no semantic classification of the scene - a full environment scan
/// bakes the actual ground/track surface AND every prop scattered across
/// it (grandstands, barriers, trees) into the same "static" geometry, so
/// TerrainHeightField's per-cell MAX-height binning will happily report a
/// grandstand roof as "ground" wherever one overlaps a cell. Picking a
/// candidate purely by proximity to the model's bounding-box centre can
/// land right next to such a structure - trackside objects cluster near
/// the track, which is usually near the middle of the scan. Preferring the
/// candidate with the smallest LOCAL height range among nearby points
/// avoids that without needing to know what any of the geometry actually
/// is. See Application::applyEnvironmentModel() and README, Open Questions.
///
/// Searches the raw point cloud directly rather than querying
/// TerrainHeightField::heightAt() over a spatial grid: a scan like a track
/// layout is a thin ribbon inside a much larger bounding box (most of the
/// box has no scanned surface at all), so a coarse world-space grid search
/// can miss the ribbon entirely and land in a gap, where heightAt()'s
/// fallback still returns SOME number that looks plausible but is not real
/// data.
glm::dvec3 pickFlattestSpawn(const std::vector<glm::vec3>& points) {
    if (points.empty()) return {0.0, 0.0, 0.0};

    glm::vec3 lo = points.front(), hi = points.front();
    for (const glm::vec3& p : points) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    const double cx = 0.5 * (lo.x + hi.x);
    const double cz = 0.5 * (lo.z + hi.z);

    // Candidates: a spread of points ordered by distance to the centre, not
    // just the single nearest one, so a bad (structure-adjacent) nearest
    // point does not automatically win before its local flatness is even
    // checked.
    std::vector<std::size_t> order(points.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const double dax = points[a].x - cx, daz = points[a].z - cz;
        const double dbx = points[b].x - cx, dbz = points[b].z - cz;
        return (dax * dax + daz * daz) < (dbx * dbx + dbz * dbz);
    });

    constexpr int kCandidates = 40;
    constexpr double kLocalRadius = 6.0;    // [m], roughly a car's footprint
    constexpr double kCandidateStride = 0.0025;   // fraction of `order` skipped per candidate

    glm::vec3 best = points[order.front()];
    double bestRange = std::numeric_limits<double>::max();

    for (int c = 0; c < kCandidates; ++c) {
        const std::size_t idx = order[static_cast<std::size_t>(
            static_cast<double>(order.size() - 1) * kCandidateStride * c)];
        const glm::vec3& cand = points[idx];

        float loY = cand.y, hiY = cand.y;
        for (const glm::vec3& p : points) {
            const double dx = p.x - cand.x;
            const double dz = p.z - cand.z;
            if (dx * dx + dz * dz > kLocalRadius * kLocalRadius) continue;
            loY = std::min(loY, p.y);
            hiY = std::max(hiY, p.y);
        }
        const double range = hiY - loY;
        if (range < bestRange) {
            bestRange = range;
            best = cand;
        }
    }
    return {best.x, 0.0, best.z};
}

void printUsage() {
    std::cout <<
        "VehicleSimulator - 3D vehicle dynamics sandbox\n"
        "\n"
        "Usage: VehicleSimulator [options]\n"
        "\n"
        "  --config <path>       simulation config (default config/simulation.yaml)\n"
        "  --vehicle <name>      vehicle config name, e.g. sports_car\n"
        "                        (resolved as config/vehicles/<name>.yaml)\n"
        "  --track <name>        test_track | race_circuit | apex_gp | nurburgring |\n"
        "                        nordschleife | straight | nurburgring_scan\n"
        "  --experiment <type>   run an experiment at startup. One of:\n"
        "                        straight_line, constant_steering, step_steering,\n"
        "                        acceleration_test, braking_test, slalom\n"
        "  --headless            run without a window; requires --experiment.\n"
        "                        Steps the simulation, records telemetry and\n"
        "                        writes a CSV, then exits.\n"
        "  --duration <seconds>  override the experiment duration\n"
        "  --hide-ui             start with the debug panels hidden (F1 shows them)\n"
        "  --camera <view>       chase | chase_far | cockpit | bonnet | bumper |\n"
        "                        wheel | tv | helicopter | free | top_down\n"
        "  --audio-out <file>    with --headless: also synthesise the run's sound\n"
        "                        (engine, turbo, tyres, wind) into a WAV file\n"
        "  --fullscreen          open full screen at the monitor's own resolution\n"
        "  --tour <directory>    render the active track from stations all the way\n"
        "                        round it, save a screenshot of each (PPM) and print\n"
        "                        frame times, then exit\n"
        "  --help                show this message\n"
        "\n"
        "Controls:\n"
        "  W/Up throttle   S/Down brake   A/Left steer left   D/Right steer right\n"
        "  Space emergency brake   R reset   P pause   N step   T telemetry\n"
        "  F1 UI   F2 export CSV   C reset camera   V cycle camera   Esc quit\n"
        "  1-0 camera view (chase, far, cockpit, bonnet, bumper, wheel, TV,\n"
        "      helicopter, free, top down)   M mute sound\n"
        "  G traction control level (1-3)   H traction control on/off\n"
        "  Y fit new tyres: Standard, Soft, Medium, Hard\n"
        "\n"
        "  With android_input.enabled in the config, an Android phone running\n"
        "  the AndroidController app takes over steering/throttle/brake/\n"
        "  handbrake automatically whenever it is connected; keyboard control\n"
        "  resumes the instant it disconnects. See AndroidController/README.md.\n";
}

} // namespace

//---------------------------------------------------------------------------
//  Command line
//---------------------------------------------------------------------------

bool parseCommandLine(int argc, char** argv, LaunchOptions& out) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[args] " << what << " requires a value\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            printUsage();
            return false;
        } else if (std::strcmp(a, "--config") == 0) {
            if (const char* v = next("--config")) out.configPath = v; else return false;
        } else if (std::strcmp(a, "--vehicle") == 0) {
            if (const char* v = next("--vehicle")) out.vehicleName = v; else return false;
        } else if (std::strcmp(a, "--track") == 0) {
            if (const char* v = next("--track")) out.trackName = v; else return false;
        } else if (std::strcmp(a, "--experiment") == 0) {
            if (const char* v = next("--experiment")) out.experiment = v; else return false;
        } else if (std::strcmp(a, "--duration") == 0) {
            if (const char* v = next("--duration")) out.headlessDuration = std::atof(v);
            else return false;
        } else if (std::strcmp(a, "--headless") == 0) {
            out.headless = true;
        } else if (std::strcmp(a, "--hide-ui") == 0) {
            out.hideUi = true;
        } else if (std::strcmp(a, "--camera") == 0) {
            if (const char* v = next("--camera")) out.cameraName = v; else return false;
        } else if (std::strcmp(a, "--fullscreen") == 0) {
            out.fullscreen = true;
        } else if (std::strcmp(a, "--tour") == 0) {
            if (const char* v = next("--tour")) out.tourDir = v; else return false;
        } else if (std::strcmp(a, "--audio-out") == 0) {
            if (const char* v = next("--audio-out")) out.audioOut = v; else return false;
        } else {
            std::cerr << "[args] unknown option '" << a << "'\n";
            printUsage();
            return false;
        }
    }

    if (out.headless && out.experiment.empty()) {
        std::cerr << "[args] --headless requires --experiment\n";
        return false;
    }
    return true;
}

//---------------------------------------------------------------------------
//  Lifecycle
//---------------------------------------------------------------------------

Application::Application() = default;

Application::~Application() {
    m_ui.shutdown();
    if (m_window) glfwDestroyWindow(m_window);
    if (!m_headless) glfwTerminate();
}

bool Application::initialize(const LaunchOptions& options) {
    m_options  = options;
    m_headless = options.headless;

    m_config = loadAppConfig(options.configPath);

    m_availableVehicles = listAvailableVehicles("config/vehicles");
    m_currentVehicle = options.vehicleName.empty() ? m_config.defaultVehicle
                                                   : options.vehicleName;

    const VehicleParameters params =
        loadVehicleParameters("config/vehicles/" + m_currentVehicle + ".yaml");

    std::cout << "[app] vehicle: " << params.name
              << "  (mass " << params.mass << " kg, wheelbase "
              << params.wheelbase << " m)\n";

    m_availableTracks = Track::availableTracks();
    m_currentTrack = options.trackName.empty() ? m_config.defaultTrack
                                               : options.trackName;

    m_track = Track::createByName(m_currentTrack);
    std::cout << "[app] track: '" << m_currentTrack << "', " << m_track.length()
              << " m, " << m_track.centreline().size() << " centreline points"
              << (m_track.closed() ? " (closed loop)" : "") << "\n";

    // A vehicle spawned at the config's fixed initial_state would start off
    // the road on any track other than the one that pose was written for -
    // put it on the new track's own start line and heading instead. The
    // config's initial_state is still respected for velocity.
    if (!m_track.centreline().empty()) {
        const TrackPoint& start = m_track.centreline().front();
        m_config.initialState.position = start.position;
        // Y is the one exception: VehicleState::position stays on the flat
        // ground-plane the rigid body is integrated on (see
        // Vehicle::cgPosition()'s comment) on every track, elevation or not
        // - real elevation is carried entirely by Vehicle::terrainHeave(),
        // which seeds itself from the terrain field on the very first
        // physics step regardless of this value. A track like
        // "nordschleife" carries real elevation in TrackPoint::position.y
        // for the MESH's benefit; copying it here too would place the
        // spawn's flat-plane reference tens or hundreds of metres up, and
        // FourWheelModel's own terrain-derived heave would then be added on
        // TOP of that for as long as the vehicle exists - exactly the "car
        // in the sky" this comment is here to prevent regressing again.
        m_config.initialState.position.y = 0.0;
        m_config.initialState.yaw = coords::yawFromDirection(start.tangent);
    }

    m_simulation = std::make_unique<Simulation>(m_config, params);
    m_camera     = std::make_unique<Camera>(m_config.camera);
    m_camera->setTrack(&m_track);
    if (!options.cameraName.empty()) {
        CameraMode mode;
        if (cameraModeFromString(options.cameraName, mode)) {
            m_camera->setMode(mode);
        } else {
            std::cerr << "[app] unknown camera '" << options.cameraName << "' - using chase\n";
        }
    }
    m_audioOut = options.audioOut;

    if (!options.experiment.empty()) {
        const ExperimentType t = experimentTypeFromString(options.experiment);
        if (t == ExperimentType::None) {
            std::cerr << "[app] unknown experiment '" << options.experiment << "'\n";
            return false;
        }
        if (options.headlessDuration > 0.0) {
            m_simulation->appConfig().experiment.duration = options.headlessDuration;
        }
        m_simulation->startExperiment(t);
    }

    if (m_headless) {
        // applyEnvironmentModel() is windowed-only - the scan's geometry can
        // only be read back after a GL upload (see its own comment) - so a
        // headless run never calls it and would otherwise always see flat
        // ground even on a track like "nordschleife" that carries real
        // elevation. That track's terrain has no such GL dependency (see
        // Renderer::buildTrackTerrain), so build and wire it here instead;
        // it is a no-op for every flat track, "nurburgring_scan" included.
        m_renderer.buildTrackTerrain(m_track);
        m_simulation->setTerrain(m_renderer.terrainField());
        m_camera->snapTo(m_simulation->vehicle());
        return true;
    }

    if (!createWindow()) return false;

    if (!m_renderer.initialize(m_track)) {
        std::cerr << "[app] renderer initialisation failed\n";
        return false;
    }
    m_renderer.rebuildVehicleMeshes(params);
    applyEnvironmentModel();

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    m_renderer.setViewport(fbw, fbh);

    m_input.attach(m_window);

    if (m_config.androidInput.enabled) {
        m_androidInput = std::make_unique<AndroidUsbInput>(m_config.androidInput);
    }

    if (!m_ui.initialize(m_window)) {
        std::cerr << "[app] ImGui initialisation failed\n";
        return false;
    }

    m_ui.setVisible(!options.hideUi);

    if (m_config.audio.enabled) {
        m_audio = std::make_unique<AudioOutput>();
        if (m_audio->start(m_config.audio.sampleRate)) {
            configureAudio();
        } else {
            m_audio.reset();   // no device: run silent, as documented
        }
    }

    m_camera->snapTo(m_simulation->vehicle());
    m_running = true;
    return true;
}

bool Application::createWindow() {
    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit()) {
        std::cerr << "[app] glfwInit failed\n";
        return false;
    }

    // OpenGL 3.3 core: widely supported, and enough for everything here.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);   // MSAA, cheap and helps the debug lines
    // The tour measures frame time, which is only comparable between runs at
    // a fixed size: ask for a window the desktop will not resize (a tiling
    // window manager floats a fixed-size window rather than stretching it).
    if (!m_options.tourDir.empty()) glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    GLFWmonitor* monitor = m_options.fullscreen ? glfwGetPrimaryMonitor() : nullptr;
    int width = m_config.windowWidth, height = m_config.windowHeight;
    if (monitor) {
        if (const GLFWvidmode* mode = glfwGetVideoMode(monitor)) {
            width = mode->width;
            height = mode->height;
        }
    }
    m_window = glfwCreateWindow(width, height, "Vehicle Simulator", monitor, nullptr);
    if (!m_window) {
        std::cerr << "[app] window creation failed - is an OpenGL 3.3 driver available?\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(m_config.vsync ? 1 : 0);

    glfwSetWindowUserPointer(m_window, this);
    glfwSetFramebufferSizeCallback(m_window, &Application::framebufferSizeCallback);

    if (!gl::load()) {
        std::cerr << "[app] OpenGL function loading failed\n";
        return false;
    }
    std::cout << "[app] OpenGL " << gl::versionString()
              << " on " << gl::rendererString() << "\n";

    // Input::attach() overwrites the user pointer, so the framebuffer callback
    // has to find the Application another way. Simplest correct fix: the
    // resize callback only needs the renderer, which we reach through a
    // file-scope pointer set here.
    return true;
}

void Application::framebufferSizeCallback(GLFWwindow* w, int width, int height) {
    // The window user pointer belongs to Input, so the viewport update is done
    // by polling the framebuffer size each frame instead (see run()). This
    // callback exists to keep GLFW from coalescing resize events oddly on some
    // platforms and to make the intent explicit.
    (void)w; (void)width; (void)height;
}

//---------------------------------------------------------------------------
//  Main loop
//---------------------------------------------------------------------------

int Application::runTour() {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(m_options.tourDir, ec);

    glfwSwapInterval(0);   // measuring, not presenting

    const auto& pts = m_track.centreline();
    if (pts.size() < 2) return 1;

    struct Shot {
        std::string name;
        glm::dvec3 position;      ///< where the car is put
        double yaw;
        CameraMode mode;
        bool freePose = false;    ///< free camera at an explicit pose instead
        glm::dvec3 eye{0.0};
        double eyeYaw = 0.0, eyePitch = 0.0;
    };
    std::vector<Shot> shots;

    // Stations evenly round the lap, alternating the two driving views.
    constexpr int kStations = 20;
    for (int k = 0; k < kStations; ++k) {
        const TrackPoint& tp = pts[pts.size() * static_cast<std::size_t>(k) / kStations];
        Shot sh;
        char name[64];
        std::snprintf(name, sizeof(name), "lap_%02d_%04dm", k, static_cast<int>(tp.distance));
        sh.name = name;
        sh.position = tp.position;
        sh.position.y = 0.0;
        sh.yaw = coords::yawFromDirection(tp.tangent);
        sh.mode = (k % 2) ? CameraMode::Bonnet : CameraMode::ThirdPerson;
        shots.push_back(sh);
    }
    // The pit lane, where there is one, and views from off the track.
    if (!m_track.pitLane().empty()) {
        const auto& pit = m_track.pitLane();
        for (double f : {0.18, 0.5, 0.9}) {
            const TrackPoint& tp = pit[static_cast<std::size_t>(f * static_cast<double>(pit.size() - 1))];
            Shot sh;
            sh.name = "pit_" + std::to_string(static_cast<int>(f * 100.0));
            sh.position = tp.position;
            sh.yaw = coords::yawFromDirection(tp.tangent);
            sh.mode = CameraMode::ThirdPerson;
            shots.push_back(sh);
        }
    }
    {
        const TrackPoint& start = pts.front();
        const double yaw = coords::yawFromDirection(start.tangent);
        const glm::dvec3 fwd = coords::heading(yaw), right = coords::right(yaw);
        auto free = [&](const std::string& name, const glm::dvec3& eye, double eyeYaw, double pitch) {
            Shot sh;
            sh.name = name;
            sh.position = start.position;
            sh.yaw = yaw;
            sh.mode = CameraMode::Free;
            sh.freePose = true;
            sh.eye = eye;
            sh.eyeYaw = eyeYaw;
            sh.eyePitch = pitch;
            shots.push_back(sh);
        };
        // From the grandstand side looking across the grid at the pits, from
        // the pit roof looking back, a helicopter over the start, and the
        // whole venue from high up.
        free("view_from_stand", start.position - right * 34.0 + coords::kWorldUp * 14.0 - fwd * 30.0, yaw + 0.95, -0.22);
        free("view_from_pits", start.position + right * 40.0 + coords::kWorldUp * 16.0 - fwd * 60.0, yaw - 0.75, -0.20);
        free("view_heli", start.position - fwd * 160.0 + coords::kWorldUp * 90.0 - right * 60.0, yaw + 0.25, -0.50);

        glm::dvec3 lo(1e30), hi(-1e30);
        for (const TrackPoint& p : pts) { lo = glm::min(lo, p.position); hi = glm::max(hi, p.position); }
        const glm::dvec3 mid = 0.5 * (lo + hi);
        free("view_overview", {mid.x, 1150.0, hi.z + 420.0}, -1.5707963, -1.02);
        free("view_overview_low", {lo.x - 250.0, 240.0, hi.z + 250.0}, -0.72, -0.30);
    }

    double worst = 0.0, sum = 0.0;
    int counted = 0;
    const ControlInput idle;

    for (const Shot& sh : shots) {
        if (glfwWindowShouldClose(m_window)) break;

        m_config.initialState.position = sh.position;
        m_config.initialState.yaw = sh.yaw;
        m_config.initialState.velocity = 0.0;
        m_simulation->appConfig().initialState = m_config.initialState;
        m_simulation->reset();
        m_simulation->clearTrajectory();   // or the trail joins one station to the next

        if (sh.freePose) {
            m_camera->setFreePose(sh.eye, sh.eyeYaw, sh.eyePitch);
        } else {
            m_camera->setMode(sh.mode);
            m_camera->snapTo(m_simulation->vehicle());
        }

        auto frame = [&]() {
            glfwPollEvents();
            int fbw = 0, fbh = 0;
            glfwGetFramebufferSize(m_window, &fbw, &fbh);
            if (fbw > 0 && fbh > 0 &&
                (fbw != m_renderer.viewportWidth() || fbh != m_renderer.viewportHeight())) {
                m_renderer.setViewport(fbw, fbh);
            }
            m_simulation->advance(1.0 / 60.0, idle);
            m_camera->update(m_simulation->vehicle(), 1.0 / 60.0);
            m_renderer.render(*m_simulation, *m_camera, m_debugOptions);
        };

        // Let the car settle on its springs and the far shadow map fill.
        for (int i = 0; i < 12; ++i) { frame(); glfwSwapBuffers(m_window); }

        constexpr int kTimed = 40;
        gl::Finish();
        const double t0 = glfwGetTime();
        for (int i = 0; i < kTimed; ++i) { frame(); glfwSwapBuffers(m_window); }
        gl::Finish();
        const double ms = (glfwGetTime() - t0) * 1000.0 / kTimed;

        frame();
        const std::string path = (fs::path(m_options.tourDir) / (sh.name + ".ppm")).string();
        const bool saved = m_renderer.saveScreenshot(path);
        const FrameStats st = m_renderer.lastFrameStats();
        glfwSwapBuffers(m_window);

        std::printf("[tour] %-22s %6.2f ms  %5.0f fps  %4d draws  %7lld tris%s\n",
                    sh.name.c_str(), ms, 1000.0 / ms, st.drawCalls, st.triangles,
                    saved ? "" : "  (screenshot FAILED)");
        std::fflush(stdout);
        worst = std::max(worst, ms);
        sum += ms;
        ++counted;
    }

    if (counted > 0) {
        std::printf("[tour] %d views at %dx%d: mean %.2f ms (%.0f fps), worst %.2f ms (%.0f fps)\n",
                    counted, m_renderer.viewportWidth(), m_renderer.viewportHeight(),
                    sum / counted, 1000.0 * counted / sum, worst, 1000.0 / worst);
    }
    return 0;
}

int Application::run() {
    if (m_headless) return runHeadless();
    if (!m_running) return 1;
    if (!m_options.tourDir.empty()) return runTour();

    double previous = glfwGetTime();

    while (!glfwWindowShouldClose(m_window)) {
        const double now = glfwGetTime();
        double frameTime = now - previous;
        previous = now;
        m_lastFrameTime = frameTime;

        glfwPollEvents();

        // Keep the viewport in sync with the framebuffer. Polling is simpler
        // and just as correct as a callback, and avoids fighting over the
        // window user pointer.
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(m_window, &fbw, &fbh);
        if (fbw > 0 && fbh > 0 &&
            (fbw != m_renderer.viewportWidth() || fbh != m_renderer.viewportHeight())) {
            m_renderer.setViewport(fbw, fbh);
        }

        // Tell the input layer whether ImGui owns the keyboard/mouse this
        // frame, so typing in a field does not also drive the car.
        m_input.setUiCapturesKeyboard(m_ui.wantsKeyboard());
        m_input.setUiCapturesMouse(m_ui.wantsMouse());
        m_input.beginFrame(m_window, frameTime);

        handleActions();

        //-------------------------------------------------------------------
        //  PHYSICS
        //
        //  One call. Everything about fixed timestepping lives inside
        //  Simulation::advance(). The render loop hands it wall-clock time and
        //  driver input, and gets nothing back but a step count.
        //-------------------------------------------------------------------
        m_simulation->vehicle().setEmergencyBrake(activeHandbrake());
        m_simulation->advance(frameTime, activeControlInput());

        updateCamera(frameTime);

        //-------------------------------------------------------------------
        //  SOUND - like rendering, reads the state and never writes it. The
        //  synthesiser glides between these once-a-frame snapshots.
        //-------------------------------------------------------------------
        if (m_audio) {
            SoundState ss = buildSoundState(m_simulation->vehicle(), *m_camera);
            if (m_simulation->isPaused()) {
                // A paused world is silent - the engine is not really idling.
                ss.engineActive = false;
                ss.speed = 0.0;
                for (auto& w : ss.wheels) w = SoundState::Wheel{};
            }
            m_audio->setState(ss);
        }

        //-------------------------------------------------------------------
        //  RENDER - reads simulation state, never writes it.
        //-------------------------------------------------------------------
        m_renderer.render(*m_simulation, *m_camera, m_debugOptions);

        m_ui.beginFrame();
        const UiRequests req = m_ui.draw(*m_simulation, *m_camera, m_renderer,
                                         m_debugOptions, m_availableVehicles,
                                         m_currentVehicle, m_availableTracks,
                                         m_currentTrack, frameTime,
                                         m_androidInput.get(), m_audio.get());
        m_ui.endFrame();

        handleUiRequests(req);

        glfwSwapBuffers(m_window);
    }
    return 0;
}

void Application::updateCamera(double frameTime) {
    if (m_input.mouseDragging()) {
        const glm::dvec2 d = m_input.mouseDelta();
        m_camera->applyMouseDelta(d.x, d.y);
    }
    if (std::abs(m_input.scrollDelta()) > 1e-9) {
        m_camera->applyScroll(m_input.scrollDelta());
    }
    if (m_camera->mode() == CameraMode::Free) {
        m_camera->applyFreeMove(m_input.freeCameraMove(), frameTime);
    }

    // Camera smoothing runs on RENDER time, deliberately: it is presentation,
    // not physics, and tying it to the fixed step would make it stutter
    // whenever the physics and render rates disagree.
    m_camera->update(m_simulation->vehicle(), frameTime);
}

//---------------------------------------------------------------------------
//  Input source selection
//
//  The Android controller is treated as just another input device, exactly
//  like the doc comment on Input::controlInput() promises: whoever is
//  "active" here hands the vehicle a plain ControlInput and the vehicle
//  never knows which source produced it. Priority is Android-while-connected
//  over keyboard, so plugging the phone in takes over immediately and
//  unplugging it falls back to the keyboard on the very next frame rather
//  than freezing the last Android value.
//---------------------------------------------------------------------------

ControlInput Application::activeControlInput() const {
    if (m_androidInput && m_androidInput->isConnected()) {
        ControlInput c = m_androidInput->controlInput();
        // Gear shifts are consumed separately from the rest of the state:
        // controlInput() is side-effect-free (the debug UI polls it too),
        // so the one-shot edge has to come from the one call site that
        // actually feeds the vehicle - see consumeShiftRequest().
        c.shiftRequest = m_androidInput->consumeShiftRequest();
        return c;
    }
    return m_input.controlInput();
}

bool Application::activeHandbrake() const {
    if (m_androidInput && m_androidInput->isConnected()) {
        return m_androidInput->handbrake();
    }
    return m_input.emergencyBrake();
}

//---------------------------------------------------------------------------
//  Actions
//---------------------------------------------------------------------------

void Application::handleActions() {
    const InputActions& a = m_input.actions();

    if (a.quit) glfwSetWindowShouldClose(m_window, GLFW_TRUE);

    if (a.reset) {
        m_simulation->reset();
        m_camera->snapTo(m_simulation->vehicle());
        m_ui.notify("Simulation reset");
    }
    if (a.togglePause) m_simulation->togglePaused();

    if (a.singleStep && m_simulation->isPaused()) {
        m_simulation->singleStep(m_input.controlInput());
    }

    if (a.toggleTelemetry) {
        m_simulation->telemetry().toggleRecording();
        m_ui.notify(m_simulation->telemetry().isRecording()
                        ? "Telemetry recording STARTED"
                        : "Telemetry recording stopped");
    }

    if (a.exportTelemetry) {
        const std::string path =
            m_simulation->telemetry().exportCsv(m_simulation->vehicle().dynamics().name());
        m_ui.notify(path.empty()
                        ? ("Export failed: " + m_simulation->telemetry().lastError())
                        : ("Exported " + path));
    }

    if (a.toggleDebugUI) m_ui.toggleVisible();

    if (a.resetCamera) {
        m_camera->resetToDefaults();
        m_camera->snapTo(m_simulation->vehicle());
    }
    if (a.cycleCamera) {
        m_camera->cycleMode();
        m_camera->snapTo(m_simulation->vehicle());
        m_ui.notify(std::string("Camera: ") + cameraModeName(m_camera->mode()));
    }
    if (a.selectCamera >= 0 &&
        a.selectCamera < static_cast<int>(cameraCycleOrder().size())) {
        m_camera->setMode(cameraCycleOrder()[static_cast<std::size_t>(a.selectCamera)]);
        m_camera->snapTo(m_simulation->vehicle());
        m_ui.notify(std::string("Camera: ") + cameraModeName(m_camera->mode()));
    }
    //-- driver aids and tyres ---------------------------------------------
    //
    // Two keys for traction control, because they are two different
    // questions: G is "how much", H is "at all". H remembers the level.
    Vehicle& car = m_simulation->vehicle();
    if (a.cycleTractionControl) {
        const int level = (car.tractionControl() % Vehicle::kTractionControlLevels) + 1;
        car.setTractionControl(level);
        m_tcLastLevel = level;
        m_ui.notify("Traction control: level " + std::to_string(level));
    }
    if (a.toggleTractionControl) {
        if (car.tractionControl() > 0) {
            m_tcLastLevel = car.tractionControl();
            car.setTractionControl(0);
            m_ui.notify("Traction control: OFF");
        } else {
            car.setTractionControl(m_tcLastLevel);
            m_ui.notify("Traction control: ON, level " + std::to_string(m_tcLastLevel));
        }
    }
    if (a.cycleTireCompound) {
        car.setTireCompound(nextTireCompound(car.tireCompound()));
        m_ui.notify(std::string("Tyres: new ") + tireCompoundName(car.tireCompound()));
    }

    if (a.toggleMute && m_audio) {
        m_audio->setMuted(!m_audio->muted());
        m_ui.notify(m_audio->muted() ? "Sound muted" : "Sound on");
    }
}

void Application::handleUiRequests(const UiRequests& req) {
    if (req.quit) glfwSetWindowShouldClose(m_window, GLFW_TRUE);

    if (req.resetSimulation) {
        m_simulation->reset();
        m_camera->snapTo(m_simulation->vehicle());
    }
    if (req.clearTrajectory) m_simulation->clearTrajectory();

    if (req.exportTelemetry) {
        const std::string path =
            m_simulation->telemetry().exportCsv(m_simulation->vehicle().dynamics().name());
        m_ui.notify(path.empty()
                        ? ("Export failed: " + m_simulation->telemetry().lastError())
                        : ("Exported " + path));
    }

    if (req.reloadVehicle && !req.vehicleToLoad.empty()) {
        loadVehicle(req.vehicleToLoad);
    }
    if (req.reloadTrack && !req.trackToLoad.empty()) {
        loadTrack(req.trackToLoad);
    }

    if (req.startExperiment) {
        m_simulation->startExperiment(req.experimentToStart);
        m_camera->snapTo(m_simulation->vehicle());
        m_ui.notify(std::string("Experiment started: ") +
                    experimentTypeToString(req.experimentToStart));
    }
    if (req.stopExperiment) {
        m_simulation->stopExperiment();
        m_ui.notify("Experiment stopped");
    }
}

void Application::loadVehicle(const std::string& name) {
    const VehicleParameters params =
        loadVehicleParameters("config/vehicles/" + name + ".yaml");

    m_currentVehicle = name;
    m_simulation->reloadVehicle(params);
    m_renderer.rebuildVehicleMeshes(params);
    m_camera->snapTo(m_simulation->vehicle());
    configureAudio();

    m_ui.notify("Loaded vehicle: " + params.name);
    std::cout << "[app] loaded vehicle '" << params.name << "'\n";
}

void Application::loadTrack(const std::string& name) {
    m_currentTrack = name;
    m_track = Track::createByName(name);
    m_renderer.rebuildTrackMeshes(m_track);

    // applyEnvironmentModel() handles both directions: switching TO
    // "nurburgring" loads the model, terrain and its own spawn; switching
    // AWAY from it clears a previously active one. It reports whether it
    // placed the vehicle itself, so this generic placement is skipped when
    // it did (its spawn is authoritative, not the placeholder centreline).
    if (!applyEnvironmentModel()) {
        // Put the vehicle back on the new track's own start line - see the
        // matching comment in initialize(). Simulation::reset() reads
        // initial state from ITS OWN stored copy of AppConfig
        // (Simulation::appConfig()), not Application's, so both are kept in
        // sync here.
        if (!m_track.centreline().empty()) {
            const TrackPoint& start = m_track.centreline().front();
            m_config.initialState.position = start.position;
            // See the matching comment in initialize(): keep the spawn on
            // the flat ground-plane VehicleState::position lives on -
            // terrainHeave() carries real elevation, not this.
            m_config.initialState.position.y = 0.0;
            m_config.initialState.yaw = coords::yawFromDirection(start.tangent);
            m_simulation->appConfig().initialState = m_config.initialState;
        }
        m_simulation->reset();
    }
    m_camera->snapTo(m_simulation->vehicle());

    m_ui.notify("Loaded track: " + name);
    std::cout << "[app] loaded track '" << name << "'\n";
}

bool Application::applyEnvironmentModel() {
    const bool want = (m_currentTrack == "nurburgring_scan") && m_config.environmentModel.hasModel();

    // Neither branch below leaves terrainField() at whatever the PREVIOUS
    // track set it to: buildTrackTerrain() is a well-defined no-op (clears
    // it) for a flat track, which today is every preset except
    // "nordschleife" - so it is correct to call unconditionally rather than
    // only when we know we need it.
    if (!want) {
        m_renderer.loadEnvironmentModel(EnvironmentModelConfig{});
        m_renderer.buildTrackTerrain(m_track);
        m_simulation->setTerrain(m_renderer.terrainField());
        return false;
    }

    if (!m_renderer.loadEnvironmentModel(m_config.environmentModel)) {
        m_renderer.buildTrackTerrain(m_track);
        m_simulation->setTerrain(m_renderer.terrainField());
        return false;
    }

    m_simulation->setTerrain(m_renderer.terrainField());

    const EnvironmentModelConfig& envCfg = m_config.environmentModel;
    glm::dvec3 spawnPos;
    double spawnYaw;
    if (envCfg.hasSpawn) {
        spawnPos = envCfg.spawnPosition;
        spawnYaw = envCfg.spawnYaw;
        std::cout << "[app] environment model spawn from config: ("
                  << spawnPos.x << ", " << spawnPos.z << ")\n";
    } else {
        spawnPos = pickFlattestSpawn(m_renderer.environmentModelPoints());
        spawnYaw = 0.0;
        std::cout << "[app] environment_model.spawn not set in config; guessed the scanned "
                  << "point nearest the model centre at (" << spawnPos.x << ", " << spawnPos.z
                  << ") - add a 'spawn:' block under environment_model: to override\n";
    }

    m_config.initialState.position = spawnPos;
    m_config.initialState.yaw = spawnYaw;
    m_simulation->appConfig().initialState = m_config.initialState;
    m_simulation->reset();
    return true;
}

//---------------------------------------------------------------------------
//  Headless
//---------------------------------------------------------------------------

int Application::runHeadless() {
    if (!m_simulation->experimentRunning()) {
        std::cerr << "[app] headless mode requires a running experiment\n";
        return 1;
    }

    const double duration = m_simulation->experiment()->config().duration;
    const double dt = m_simulation->simConfig().timestep;

    std::cout << "[app] headless run: "
              << m_simulation->experiment()->description() << "\n"
              << "[app] dt = " << dt << " s, duration = " << duration << " s\n";

    // Non-realtime so advance() takes exactly one step per call.
    m_simulation->simConfig().realtime = false;

    const long long maxSteps = static_cast<long long>(duration / dt) + 2;
    long long steps = 0;

    // Optional offline sound render: the same synthesiser the live device
    // uses, fed once per physics step, written to a WAV at the end. This is
    // how the sound is tested - it can be listened to AND analysed.
    std::unique_ptr<SoundSynth> synth;
    std::vector<float> samples;
    double sampleDebt = 0.0;
    if (!m_audioOut.empty()) {
        synth = std::make_unique<SoundSynth>(m_config.audio.sampleRate);
        synth->configure(m_simulation->vehicle().parameters().sound,
                         soundEngineGeometry(m_simulation->vehicle()));
        synth->mix().master = static_cast<float>(m_config.audio.masterVolume);
        samples.reserve(static_cast<std::size_t>(duration * m_config.audio.sampleRate * 2.0) + 4096);
    }

    while (m_simulation->experimentRunning() && steps < maxSteps) {
        m_simulation->advance(dt, ControlInput{});
        ++steps;
        if (synth) {
            m_camera->update(m_simulation->vehicle(), dt);
            synth->setState(buildSoundState(m_simulation->vehicle(), *m_camera));
            sampleDebt += dt * synth->sampleRate();
            const int n = static_cast<int>(sampleDebt);
            sampleDebt -= n;
            const std::size_t at = samples.size();
            samples.resize(at + static_cast<std::size_t>(2 * n));
            synth->render(samples.data() + at, n);
        }
    }
    if (synth) {
        if (writeWav(m_audioOut, samples, static_cast<int>(synth->sampleRate()))) {
            std::cout << "[audio] wrote " << samples.size() / 2 << " frames to " << m_audioOut << "\n";
        } else {
            std::cerr << "[audio] could not write " << m_audioOut << "\n";
        }
    }

    const VehicleState& s = m_simulation->vehicle().state();
    std::cout << "[app] finished after " << steps << " steps ("
              << s.time << " s simulated)\n"
              << "[app] final: position (" << s.position.x << ", " << s.position.z
              << ") m, speed " << s.velocity << " m/s, heading "
              << radToDeg(s.yaw) << " deg\n";

    const std::string path =
        m_simulation->telemetry().exportCsv(m_simulation->vehicle().dynamics().name());
    if (path.empty()) {
        std::cerr << "[app] telemetry export failed: "
                  << m_simulation->telemetry().lastError() << "\n";
        return 1;
    }
    return 0;
}

void Application::configureAudio() {
    if (!m_audio) return;
    const Vehicle& v = m_simulation->vehicle();
    m_audio->configure(v.parameters().sound, soundEngineGeometry(v));
    m_audio->withSynth([this](SoundSynth& s) {
        s.mix().master = static_cast<float>(m_config.audio.masterVolume);
    });
}

} // namespace vsim
