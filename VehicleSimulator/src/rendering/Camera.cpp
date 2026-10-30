#include "rendering/Camera.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

#include "core/Coordinates.h"
#include "environment/Track.h"
#include "vehicle/Vehicle.h"

namespace vsim {

const char* cameraModeName(CameraMode m) {
    switch (m) {
        case CameraMode::ThirdPerson: return "Chase";
        case CameraMode::Free:        return "Free";
        case CameraMode::TopDown:     return "Top Down";
        case CameraMode::ChaseFar:    return "Chase (far)";
        case CameraMode::Cockpit:     return "Cockpit";
        case CameraMode::Bonnet:      return "Bonnet";
        case CameraMode::Bumper:      return "Bumper";
        case CameraMode::Wheel:       return "Wheel";
        case CameraMode::TV:          return "TV trackside";
        case CameraMode::Helicopter:  return "Helicopter";
        case CameraMode::Count:       break;
    }
    return "?";
}

bool cameraModeFromString(const std::string& name, CameraMode& out) {
    static const std::pair<const char*, CameraMode> names[] = {
        {"chase", CameraMode::ThirdPerson}, {"chase_far", CameraMode::ChaseFar},
        {"cockpit", CameraMode::Cockpit},   {"bonnet", CameraMode::Bonnet},
        {"hood", CameraMode::Bonnet},       {"bumper", CameraMode::Bumper},
        {"wheel", CameraMode::Wheel},       {"tv", CameraMode::TV},
        {"helicopter", CameraMode::Helicopter}, {"free", CameraMode::Free},
        {"top_down", CameraMode::TopDown}};
    for (const auto& [n, m] : names) {
        if (name == n) { out = m; return true; }
    }
    return false;
}

const std::vector<CameraMode>& cameraCycleOrder() {
    static const std::vector<CameraMode> order = {
        CameraMode::ThirdPerson, CameraMode::ChaseFar, CameraMode::Cockpit,
        CameraMode::Bonnet, CameraMode::Bumper, CameraMode::Wheel,
        CameraMode::TV, CameraMode::Helicopter, CameraMode::Free, CameraMode::TopDown};
    return order;
}

namespace {

/// Vertical field of view each body-mounted view starts at [deg]. Wider
/// than the chase camera because the lens is close to the car.
double baseMountedFov(CameraMode m) {
    switch (m) {
        case CameraMode::Cockpit: return 75.0;
        case CameraMode::Bonnet:  return 65.0;
        case CameraMode::Bumper:  return 65.0;
        case CameraMode::Wheel:   return 70.0;
        default:                  return 60.0;
    }
}

/// Default look direction of a body-mounted view, as (pitch, yaw) [rad]
/// relative to the car's forward axis. Slightly down for the driver - the
/// road, not the horizon, is what they look at.
void baseMountedLook(CameraMode m, double& pitch, double& yaw) {
    pitch = 0.0; yaw = 0.0;
    switch (m) {
        case CameraMode::Cockpit: pitch = -0.07; break;
        case CameraMode::Bonnet:  pitch = -0.05; break;
        case CameraMode::Bumper:  pitch = -0.02; break;
        case CameraMode::Wheel:   pitch = -0.03; yaw = 0.05; break;   // toed in toward the car
        default: break;
    }
}

// Cockpit head motion: displacement per m/s^2 of sustained acceleration,
// and its limit. A driver's head is held by the neck and the seat; ~5 cm at
// 1.3 g of braking is a restrained-driver figure, not a crash test. AUTHORED.
constexpr double kHeadCompliance = 0.004;   // [m per m/s^2]
constexpr double kHeadMaxOffset  = 0.06;    // [m]

// TV camera layout. AUTHORED - how a broadcaster would rig a circuit.
constexpr double kTvSpacing   = 140.0;   // [m] along the centreline
constexpr double kTvSetback   = 9.0;     // [m] beyond the track edge
constexpr double kTvHeight    = 3.0;     // [m] above the road
constexpr double kTvFrameSize = 4.0;     // [m] half-height of the framed shot

} // namespace

Camera::Camera(const CameraConfig& config)
    : m_config(config), m_defaults(config), m_fov(config.fovDegrees) {}

//---------------------------------------------------------------------------
//  Body-mounted geometry
//---------------------------------------------------------------------------

bool Camera::bodyMounted() const {
    return m_mode == CameraMode::Cockpit || m_mode == CameraMode::Bonnet ||
           m_mode == CameraMode::Bumper  || m_mode == CameraMode::Wheel;
}

bool Camera::vehicleNearPass() const {
    return m_mode == CameraMode::Cockpit || m_mode == CameraMode::Bonnet ||
           m_mode == CameraMode::Wheel;
}

glm::dmat4 Camera::bodyMatrix(const Vehicle& vehicle) const {
    // Exactly the transform Renderer::drawVehicle() draws the body with, so
    // a mounted camera moves with the body it is bolted to - heave, pitch
    // and roll included.
    const VehicleState& s = vehicle.state();
    const glm::dvec3 heavePos = s.position + coords::kWorldUp * vehicle.terrainHeave();
    return coords::modelMatrixYPR(heavePos, s.yaw, vehicle.terrainPitch(),
                                  vehicle.terrainRoll() + s.rollAngle);
}

glm::dvec3 Camera::mountPoint(const Vehicle& vehicle, CameraMode m) const {
    const VehicleParameters& p = vehicle.parameters();
    const VehicleCameraConfig& c = p.cameras;
    auto authored = [](const LocalPoint& lp) {
        return glm::dvec3(lp.xyz[0], lp.xyz[1], lp.xyz[2]);
    };

    // Defaults from dimensions, using the same overhang split the generated
    // body uses (rendering/CarMesh.cpp): 54 % of the overhang at the front.
    const double L = p.wheelbase, len = p.length, H = p.height, W = p.width;
    const double nose = L + 0.54 * std::max(len - L, 0.2);

    switch (m) {
        case CameraMode::Cockpit:
            return c.driverEye.set ? authored(c.driverEye)
                                   : glm::dvec3(nose - 0.50 * len, 0.80 * H, -0.20 * W);
        case CameraMode::Bonnet:
            return c.bonnet.set ? authored(c.bonnet)
                                : glm::dvec3(nose - 0.30 * len, 0.86 * H, 0.0);
        case CameraMode::Bumper:
            return c.bumper.set ? authored(c.bumper)
                                : glm::dvec3(nose + 0.05, 0.35, 0.0);
        case CameraMode::Wheel:
            return c.wheel.set ? authored(c.wheel)
                               : glm::dvec3(0.35 * L, 0.40, -(0.5 * W + 0.10));
        default:
            return glm::dvec3(0.0);
    }
}

//---------------------------------------------------------------------------
//  Desired pose
//---------------------------------------------------------------------------

glm::dvec3 Camera::desiredPosition(const Vehicle& vehicle) const {
    const VehicleState& s = vehicle.state();
    const glm::dvec3 anchor = vehicle.bodyCentrePosition();

    switch (m_mode) {
        case CameraMode::ThirdPerson:
        case CameraMode::ChaseFar: {
            const bool far = (m_mode == CameraMode::ChaseFar);
            const double dist = m_config.distance * (far ? 1.8 : 1.0);
            const double hgt  = m_config.height * (far ? 1.35 : 1.0);
            // Orbit is expressed relative to the vehicle heading, so dragging
            // the mouse looks around the car while it keeps following.
            const double yaw   = s.yaw + m_orbitYaw;
            const double pitch = clampd(m_orbitPitch, -1.3, 1.3);

            const double horizontal = dist * std::cos(pitch);
            const double vertical   = hgt + dist * std::sin(pitch);

            // Sit BEHIND the vehicle: minus the heading direction.
            return anchor - coords::heading(yaw) * horizontal
                 + coords::kWorldUp * vertical;
        }

        case CameraMode::Helicopter:
            return anchor - coords::heading(s.yaw + m_orbitYaw) * 40.0
                 + coords::kWorldUp * 28.0;

        case CameraMode::TopDown:
            return anchor + coords::kWorldUp * m_topDownHeight;

        case CameraMode::Cockpit:
        case CameraMode::Bonnet:
        case CameraMode::Bumper:
        case CameraMode::Wheel: {
            glm::dvec3 local = mountPoint(vehicle, m_mode);
            if (m_mode == CameraMode::Cockpit) local += m_headOffset;
            return glm::dvec3(bodyMatrix(vehicle) * glm::dvec4(local, 1.0));
        }

        case CameraMode::Free:
        case CameraMode::TV:
        case CameraMode::Count:
            return m_position;   // these own their position
    }
    return m_position;
}

glm::dvec3 Camera::desiredTarget(const Vehicle& vehicle) const {
    const VehicleState& s = vehicle.state();
    const glm::dvec3 anchor = vehicle.bodyCentrePosition();

    switch (m_mode) {
        case CameraMode::ThirdPerson:
        case CameraMode::ChaseFar:
            // Aim ahead of the vehicle rather than at it. This is what makes a
            // chase camera feel like it is anticipating the corner instead of
            // trailing the car.
            return anchor + s.forward() * m_config.lookAhead
                 + coords::kWorldUp * 0.8;

        case CameraMode::Helicopter:
        case CameraMode::TopDown:
        case CameraMode::TV:
            return anchor;

        case CameraMode::Cockpit:
        case CameraMode::Bonnet:
        case CameraMode::Bumper:
        case CameraMode::Wheel: {
            double pitch = 0.0, yaw = 0.0;
            baseMountedLook(m_mode, pitch, yaw);
            pitch += m_orbitPitch;
            yaw   += m_orbitYaw;
            // Local frame: +X forward, +Y up, +Z right; positive yaw looks right.
            const glm::dvec3 dir{std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                                 std::cos(pitch) * std::sin(yaw)};
            glm::dvec3 local = mountPoint(vehicle, m_mode) + dir * 10.0;
            if (m_mode == CameraMode::Cockpit) local += m_headOffset;
            return glm::dvec3(bodyMatrix(vehicle) * glm::dvec4(local, 1.0));
        }

        case CameraMode::Free:
        case CameraMode::Count: {
            const glm::dvec3 dir{
                std::cos(m_freePitch) * std::cos(m_freeYaw),
                std::sin(m_freePitch),
                std::cos(m_freePitch) * std::sin(m_freeYaw)};
            return m_position + dir;
        }
    }
    return anchor;
}

//---------------------------------------------------------------------------
//  TV cameras
//---------------------------------------------------------------------------

void Camera::rebuildTvStations() {
    m_tvStations.clear();
    m_tvActive = -1;
    if (!m_track) return;
    const auto& pts = m_track->centreline();
    if (pts.size() < 2 || m_track->length() < 2.0 * kTvSpacing) return;

    double next = 0.0;
    int n = 0;
    for (const TrackPoint& tp : pts) {
        if (tp.distance < next) continue;
        next += kTvSpacing;
        // Put the camera on the OUTSIDE of a corner - that is where the car
        // is framed against the apex rather than driving away from the lens -
        // and alternate sides on the straights.
        const double side = (std::abs(tp.curvature) > 1e-4) ? (tp.curvature > 0.0 ? -1.0 : 1.0)
                                                            : ((n % 2) ? 1.0 : -1.0);
        const glm::dvec3 right{-tp.tangent.z, 0.0, tp.tangent.x};
        const glm::dvec3 lat = glm::length(right) > 1e-9 ? glm::normalize(right) : glm::dvec3(0, 0, 1);
        m_tvStations.push_back(tp.position + lat * side * (0.5 * m_track->width() + kTvSetback)
                               + coords::kWorldUp * kTvHeight);
        ++n;
    }
}

void Camera::updateTv(const Vehicle& vehicle, double dt) {
    const glm::dvec3 car = vehicle.bodyCentrePosition();

    const double sig = m_track ? m_track->length() + 1e-3 * m_track->centreline().size() : 0.0;
    if (sig != m_tvTrackSig) { m_tvTrackSig = sig; rebuildTvStations(); }

    if (!m_tvStations.empty()) {
        // Cut to the nearest camera - but only when it is clearly nearer,
        // so the view does not flicker between two equidistant cameras.
        int best = 0;
        double bestD = 1e30;
        for (std::size_t i = 0; i < m_tvStations.size(); ++i) {
            const double d = glm::length(m_tvStations[i] - car);
            if (d < bestD) { bestD = d; best = static_cast<int>(i); }
        }
        const double curD = (m_tvActive >= 0)
            ? glm::length(m_tvStations[static_cast<std::size_t>(m_tvActive)] - car) : 1e30;
        if (m_tvActive < 0 || bestD < 0.8 * curD) m_tvActive = best;
        m_position = m_tvStations[static_cast<std::size_t>(m_tvActive)];
    } else {
        // No usable circuit (a raw scan or a short track): rig a camera
        // ahead of the car each time it drives out of shot.
        const VehicleState& s = vehicle.state();
        if (!m_tvDynamicValid || glm::length(m_tvDynamic - car) > 160.0) {
            m_tvDynamic = car + s.forward() * 70.0 + s.right() * 12.0 + coords::kWorldUp * kTvHeight;
            if (vehicle.terrain()) m_tvDynamic.y = car.y + kTvHeight;
            m_tvDynamicValid = true;
        }
        m_position = m_tvDynamic;
    }

    // Pan: an operator follows the car, a little behind it.
    const double alpha = 1.0 - std::exp(-8.0 * dt);
    m_target += (car - m_target) * alpha;

    // Zoom: hold the car at a constant size in frame.
    const double dist = std::max(glm::length(car - m_position), 1.0);
    const double wantFov = std::clamp(2.0 * std::atan(kTvFrameSize / dist) * 180.0 / 3.14159265358979,
                                      4.0, 55.0);
    m_fov += (wantFov - m_fov) * (1.0 - std::exp(-4.0 * dt));
}

//---------------------------------------------------------------------------
//  Update
//---------------------------------------------------------------------------

void Camera::update(const Vehicle& vehicle, double dt) {
    if (!(dt > 0.0) || !isFinite(dt)) dt = 1.0 / 60.0;
    m_up = coords::kWorldUp;

    if (m_mode == CameraMode::Free) {
        m_fov = m_config.fovDegrees;
        m_target = desiredTarget(vehicle);
        return;
    }

    if (m_mode == CameraMode::TV) {
        updateTv(vehicle, dt);
        return;
    }

    if (bodyMounted()) {
        // Head motion: the driver's head lags the car under sustained
        // acceleration - forward under braking, outward in a corner.
        if (m_mode == CameraMode::Cockpit) {
            const glm::dvec3 want = glm::clamp(
                glm::dvec3(-vehicle.state().acceleration, 0.0, -vehicle.lateralAcceleration())
                    * kHeadCompliance,
                glm::dvec3(-kHeadMaxOffset), glm::dvec3(kHeadMaxOffset));
            m_headOffset += (want - m_headOffset) * (1.0 - std::exp(-6.0 * dt));
        }
        // Rigid: a camera bolted to the car has no smoothing of its own.
        m_position = desiredPosition(vehicle);
        m_target   = desiredTarget(vehicle);
        m_up       = glm::dvec3(bodyMatrix(vehicle) * glm::dvec4(0.0, 1.0, 0.0, 0.0));
        m_fov      = std::clamp(baseMountedFov(m_mode) + m_fovTrim, 30.0, 110.0);
        return;
    }

    m_fov = (m_mode == CameraMode::Helicopter) ? 40.0 : m_config.fovDegrees;

    const glm::dvec3 wantPos = desiredPosition(vehicle);
    const glm::dvec3 wantTgt = desiredTarget(vehicle);

    //-----------------------------------------------------------------------
    //  Exponential smoothing, framerate-independent.
    //
    //      alpha = 1 - exp(-k * dt)
    //
    //  The naive `pos += (want - pos) * k * dt` form is framerate-DEPENDENT:
    //  at low frame rates k*dt can exceed 1 and the camera overshoots or
    //  oscillates. The exponential form is the exact solution of the same
    //  first-order lag and is stable for any dt.
    //-----------------------------------------------------------------------
    double k = std::max(0.0, m_config.followSpeed);
    if (m_mode == CameraMode::Helicopter) k = std::min(k, 1.2);   // heavy, slow
    const double alpha = (k > 0.0) ? (1.0 - std::exp(-k * dt)) : 1.0;

    m_position += (wantPos - m_position) * alpha;
    m_target   += (wantTgt - m_target)   * alpha;
}

void Camera::snapTo(const Vehicle& vehicle) {
    m_headOffset = glm::dvec3(0.0);
    if (m_mode == CameraMode::Free) {
        // Place the free camera somewhere sensible relative to the vehicle
        // rather than leaving it wherever it was.
        const glm::dvec3 anchor = vehicle.bodyCentrePosition();
        m_position = anchor + coords::kWorldUp * 15.0
                   - vehicle.state().forward() * 20.0;
        m_freeYaw   = vehicle.state().yaw;
        m_freePitch = -0.4;
        m_target = desiredTarget(vehicle);
    } else if (m_mode == CameraMode::TV) {
        m_tvActive = -1;
        m_tvDynamicValid = false;
        m_target = vehicle.bodyCentrePosition();
        updateTv(vehicle, 10.0);   // a long dt settles pan and zoom at once
    } else {
        m_position = desiredPosition(vehicle);
        m_target = desiredTarget(vehicle);
        update(vehicle, 1.0 / 60.0);
        if (!bodyMounted()) {
            m_position = desiredPosition(vehicle);
            m_target = desiredTarget(vehicle);
        }
    }
}

//---------------------------------------------------------------------------
//  Matrices
//---------------------------------------------------------------------------

glm::dvec3 Camera::viewForward() const {
    const glm::dvec3 d = m_target - m_position;
    const double len = glm::length(d);
    return len > 1e-9 ? d / len : glm::dvec3(1.0, 0.0, 0.0);
}

glm::dvec3 Camera::viewRight() const {
    const glm::dvec3 r = glm::cross(viewForward(), coords::kWorldUp);
    const double len = glm::length(r);
    return len > 1e-9 ? r / len : glm::dvec3(0.0, 0.0, 1.0);
}

glm::mat4 Camera::viewMatrix() const {
    glm::dvec3 up = m_up;

    // Looking straight down makes the view direction parallel to the up
    // vector, which makes lookAt produce a degenerate (NaN) basis. Nudge the
    // up vector onto the ground plane in that case.
    const glm::dvec3 dir = m_target - m_position;
    const double len = glm::length(dir);
    if (len > 1e-9) {
        const glm::dvec3 n = dir / len;
        if (std::abs(glm::dot(n, glm::normalize(up))) > 0.999) up = glm::dvec3(1.0, 0.0, 0.0);
    }

    return glm::lookAt(glm::vec3(m_position), glm::vec3(m_target), glm::vec3(up));
}

glm::mat4 Camera::projectionMatrix(float aspect) const {
    if (!(aspect > 0.0f) || !std::isfinite(aspect)) aspect = 16.0f / 9.0f;
    return glm::perspective(glm::radians(static_cast<float>(m_fov)),
                            aspect,
                            static_cast<float>(m_config.nearPlane),
                            static_cast<float>(m_config.farPlane));
}

glm::mat4 Camera::nearProjectionMatrix(float aspect) const {
    if (!(aspect > 0.0f) || !std::isfinite(aspect)) aspect = 16.0f / 9.0f;
    // The car only: a few centimetres to a few tens of metres, so the depth
    // precision the world pass could not afford is plentiful here.
    return glm::perspective(glm::radians(static_cast<float>(m_fov)), aspect, 0.02f, 60.0f);
}

//---------------------------------------------------------------------------
//  Input
//---------------------------------------------------------------------------

void Camera::applyMouseDelta(double dx, double dy) {
    constexpr double kSensitivity = 0.005;

    switch (m_mode) {
        case CameraMode::ThirdPerson:
        case CameraMode::ChaseFar:
        case CameraMode::Helicopter:
            m_orbitYaw   -= dx * kSensitivity;
            m_orbitPitch += dy * kSensitivity;
            m_orbitPitch  = clampd(m_orbitPitch, -1.2, 1.2);
            break;

        case CameraMode::Cockpit:
        case CameraMode::Bonnet:
        case CameraMode::Bumper:
        case CameraMode::Wheel:
            // Head turn: drag right to look right.
            m_orbitYaw   += dx * kSensitivity;
            m_orbitPitch -= dy * kSensitivity;
            m_orbitYaw    = clampd(m_orbitYaw, -2.3, 2.3);
            m_orbitPitch  = clampd(m_orbitPitch, -0.9, 0.9);
            break;

        case CameraMode::Free:
            m_freeYaw   += dx * kSensitivity;
            m_freePitch -= dy * kSensitivity;
            m_freePitch  = clampd(m_freePitch, -1.5, 1.5);
            break;

        case CameraMode::TopDown:
        case CameraMode::TV:
        case CameraMode::Count:
            break;  // nothing to rotate
    }
}

void Camera::applyScroll(double amount) {
    switch (m_mode) {
        case CameraMode::ThirdPerson:
        case CameraMode::ChaseFar:
            // Multiplicative zoom, so each notch changes the distance by the
            // same proportion whether you are close in or far out.
            m_config.distance *= std::pow(0.9, amount);
            m_config.distance = clampd(m_config.distance, 1.5, 120.0);
            break;

        case CameraMode::Cockpit:
        case CameraMode::Bonnet:
        case CameraMode::Bumper:
        case CameraMode::Wheel:
            m_fovTrim = clampd(m_fovTrim - 2.0 * amount, -40.0, 35.0);
            break;

        case CameraMode::Free:
            m_freeSpeed *= std::pow(1.2, amount);
            m_freeSpeed  = clampd(m_freeSpeed, 0.5, 500.0);
            break;

        case CameraMode::TopDown:
            m_topDownHeight *= std::pow(0.9, amount);
            m_topDownHeight  = clampd(m_topDownHeight, 10.0, 800.0);
            break;

        case CameraMode::TV:
        case CameraMode::Helicopter:
        case CameraMode::Count:
            break;
    }
}

void Camera::applyFreeMove(const glm::vec3& localDirection, double dt) {
    if (m_mode != CameraMode::Free) return;
    if (glm::length(localDirection) < 1e-6) return;

    const glm::dvec3 forward{
        std::cos(m_freePitch) * std::cos(m_freeYaw),
        std::sin(m_freePitch),
        std::cos(m_freePitch) * std::sin(m_freeYaw)};

    const glm::dvec3 right = glm::normalize(glm::cross(forward, coords::kWorldUp));

    m_position += (forward * static_cast<double>(localDirection.z)
                 + right   * static_cast<double>(localDirection.x)
                 + coords::kWorldUp * static_cast<double>(localDirection.y))
                * (m_freeSpeed * dt);
}

void Camera::setMode(CameraMode m) {
    if (m == m_mode || m == CameraMode::Count) return;
    m_mode = m;
    // Clear the orbit / head turn so each view starts predictably.
    m_orbitYaw = 0.0;
    m_orbitPitch = 0.0;
    m_tvActive = -1;
    m_tvDynamicValid = false;
}

void Camera::cycleMode() {
    const auto& order = cameraCycleOrder();
    auto it = std::find(order.begin(), order.end(), m_mode);
    const std::size_t i = (it == order.end()) ? 0 : static_cast<std::size_t>(it - order.begin());
    setMode(order[(i + 1) % order.size()]);
}

void Camera::resetToDefaults() {
    m_config = m_defaults;
    m_orbitYaw = 0.0;
    m_orbitPitch = 0.0;
    m_freePitch = -0.3;
    m_freeSpeed = 20.0;
    m_topDownHeight = 60.0;
    m_fovTrim = 0.0;
    setMode(CameraMode::ThirdPerson);
}

} // namespace vsim
