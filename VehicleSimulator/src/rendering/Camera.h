#pragma once
//===========================================================================
//  Camera.h  --  view/projection generation and camera modes.
//
//  The camera reads simulation state but never writes it. Camera smoothing
//  runs on RENDER time (variable dt), deliberately, because it is a
//  presentation effect - putting it on the physics clock would make the view
//  stutter whenever the physics and render rates disagree.
//
//  THE VIEWS A DRIVING SIM IS EXPECTED TO HAVE
//  -------------------------------------------
//    Chase / Chase far  - smoothed, behind and above, orbit with the mouse
//    Cockpit            - the driver's eyes, rigidly mounted to the BODY
//                         (so it pitches and rolls with it), mouse-look, and
//                         the head moves against the acceleration it feels
//    Bonnet             - above the scuttle, looking over the bonnet
//    Bumper             - on the nose, near the ground; car not drawn
//    Wheel              - on the flank, looking forward past the front wheel
//    TV                 - fixed trackside cameras along the circuit, cut to
//                         whichever is nearest, panning and zooming to hold
//                         the car at a steady size - broadcast style
//    Helicopter         - high and behind, slow to follow
//    Free / Top down    - the existing debug views
//
//  Mount points come from the vehicle's `cameras:` YAML block
//  (VehicleCameraConfig), else are derived from its dimensions.
//
//  NEAR PLANE. The world cannot use a near plane small enough for a camera
//  that sits a few centimetres from the dashboard - see CameraConfig::
//  nearPlane on why. Body-mounted views therefore ask the renderer for a
//  second pass (vehicleNearPass()): the world is drawn with the normal
//  projection, the depth buffer cleared, and the car drawn on top with
//  nearProjectionMatrix(). From a camera mounted ON the car nothing outside
//  the car can be between the lens and the car's own bodywork, so drawing
//  it last is correct, not a cheat.
//===========================================================================

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "config/Config.h"

namespace vsim {

class Vehicle;
class Track;

enum class CameraMode {
    ThirdPerson,  ///< chase camera, follows behind the vehicle
    Free,         ///< detached orbit/fly camera
    TopDown,      ///< orthographic-feeling overhead debug view
    ChaseFar,     ///< chase, further back and higher
    Cockpit,      ///< driver's eye point, body-mounted
    Bonnet,       ///< above the scuttle, looking over the bonnet
    Bumper,       ///< on the nose, near the road
    Wheel,        ///< on the flank, looking forward past the front wheel
    TV,           ///< fixed trackside cameras, auto-cut
    Helicopter,   ///< high, trailing, slow
    Count
};

const char* cameraModeName(CameraMode m);

/// "chase", "chase_far", "cockpit", "bonnet", "bumper", "wheel", "tv",
/// "helicopter", "free", "top_down". False if unknown.
bool cameraModeFromString(const std::string& name, CameraMode& out);

/// Order used by cycleMode() and the number keys 1..0.
const std::vector<CameraMode>& cameraCycleOrder();

class Camera {
public:
    explicit Camera(const CameraConfig& config);

    /// Advance camera smoothing. `dt` is REAL frame time, not physics dt.
    void update(const Vehicle& vehicle, double dt);

    /// Snap immediately to the target pose, skipping smoothing. Used on reset
    /// and on mode change so the camera does not fly across the world.
    void snapTo(const Vehicle& vehicle);

    /// Track the TV cameras are placed along. May be null; the TV view then
    /// places its cameras ahead of the car as it goes.
    void setTrack(const Track* track) { m_track = track; m_tvStations.clear(); m_tvTrackSig = -1.0; }

    glm::mat4 viewMatrix() const;
    glm::mat4 projectionMatrix(float aspect) const;
    /// Projection for the car-only second pass (see file banner).
    glm::mat4 nearProjectionMatrix(float aspect) const;

    glm::vec3 position() const { return glm::vec3(m_position); }
    glm::vec3 target()   const { return glm::vec3(m_target); }
    glm::dvec3 positionD() const { return m_position; }
    /// Unit vectors of the view, world space.
    glm::dvec3 viewForward() const;
    glm::dvec3 viewRight() const;

    /// Body-mounted view: world first, then the car on a cleared depth buffer.
    bool vehicleNearPass() const;
    /// False for views the car's own body would block (bumper).
    bool drawsVehicle() const { return m_mode != CameraMode::Bumper; }
    /// Listener is inside the cabin (sound goes through the body shell).
    bool interior() const { return m_mode == CameraMode::Cockpit; }

    //-- input ------------------------------------------------------------

    /// Mouse drag in pixels. Orbits the chase views, turns the head in the
    /// body-mounted views, looks around in free.
    void applyMouseDelta(double dx, double dy);

    /// Scroll wheel. Zooms in third-person, changes speed in free mode.
    void applyScroll(double amount);

    /// WASD-style movement for free camera mode, in camera-local axes.
    void applyFreeMove(const glm::vec3& localDirection, double dt);

    void setMode(CameraMode m);
    CameraMode mode() const { return m_mode; }

    /// Switch to the free camera and put it at an explicit pose. `yaw` and
    /// `pitch` are radians, yaw by the convention in core/Coordinates.h.
    /// Takes effect on the next update(). Used by the screenshot tour.
    void setFreePose(const glm::dvec3& position, double yaw, double pitch) {
        setMode(CameraMode::Free);
        m_position  = position;
        m_freeYaw   = yaw;
        m_freePitch = pitch;
    }
    void cycleMode();

    /// Restore the configured defaults.
    void resetToDefaults();

    CameraConfig&       config()       { return m_config; }
    const CameraConfig& config() const { return m_config; }

    /// Current effective vertical field of view [deg] (TV zooms).
    double effectiveFov() const { return m_fov; }

private:
    glm::dvec3 desiredPosition(const Vehicle& vehicle) const;
    glm::dvec3 desiredTarget(const Vehicle& vehicle) const;

    /// Vehicle-local mount point for a body-mounted mode, authored or derived.
    glm::dvec3 mountPoint(const Vehicle& vehicle, CameraMode m) const;
    /// Body attitude matrix - identical to the one the renderer draws with.
    glm::dmat4 bodyMatrix(const Vehicle& vehicle) const;
    bool bodyMounted() const;

    void rebuildTvStations();
    void updateTv(const Vehicle& vehicle, double dt);

    CameraConfig m_config;
    CameraConfig m_defaults;
    CameraMode   m_mode = CameraMode::ThirdPerson;

    glm::dvec3 m_position{0.0, 5.0, -10.0};
    glm::dvec3 m_target{0.0};
    glm::dvec3 m_up{0.0, 1.0, 0.0};
    double m_fov = 60.0;

    /// Orbit offsets applied on top of the vehicle heading, in radians.
    /// Reset when the camera mode changes so the view is predictable. Also
    /// the head yaw/pitch in the body-mounted views.
    double m_orbitYaw   = 0.0;
    double m_orbitPitch = 0.0;

    /// Free-camera orientation and speed.
    double m_freeYaw    = 0.0;
    double m_freePitch  = -0.3;
    double m_freeSpeed  = 20.0;

    /// Top-down view height above the vehicle.
    double m_topDownHeight = 60.0;

    /// Field-of-view trim for the body-mounted views [deg] (scroll wheel),
    /// the "seat position / FOV" setting every sim has.
    double m_fovTrim = 0.0;

    /// Cockpit head displacement from sustained acceleration [m, local].
    glm::dvec3 m_headOffset{0.0};

    //-- TV --
    const Track* m_track = nullptr;
    std::vector<glm::dvec3> m_tvStations;
    double m_tvTrackSig = -1.0;
    int m_tvActive = -1;
    glm::dvec3 m_tvDynamic{0.0};
    bool m_tvDynamicValid = false;
};

} // namespace vsim
