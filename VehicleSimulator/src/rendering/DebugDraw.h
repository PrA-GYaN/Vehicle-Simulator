#pragma once
//===========================================================================
//  DebugDraw.h  --  immediate-mode line overlay.
//
//  Accumulates line segments during a frame and flushes them in one draw
//  call. This is the mechanism behind every physics visualisation: coordinate
//  frames, force/velocity vectors, the turning circle, the trajectory trail.
//
//  Purely a VISUALISATION of simulation state. It never feeds anything back.
//===========================================================================

#include <vector>

#include <glm/glm.hpp>

#include "rendering/Mesh.h"

namespace vsim {

class Shader;

/// Which overlays are enabled. Mirrors the checkbox list in the debug UI.
struct DebugDrawOptions {
    // Off by default, all of them, for the same reason as `grid` below: this
    // is what a car looks like the instant the app opens, before anyone has
    // touched a checkbox, and a vehicle wearing an axis frame, a floating
    // centre-of-mass marker and three arrows reads as an instrumented rig,
    // not a car - which is exactly backwards for a generated body this
    // detailed (see CarMesh.h). Every one of these is one click away in the
    // debug panel (F1) whenever the analysis they are for is actually
    // wanted.
    bool vehicleFrame   = false;
    bool worldFrame     = false;
    bool centreOfMass   = false;
    bool velocityVector = false;
    bool headingVector  = false;
    bool steeringVector = false;
    bool wheelContacts  = false;
    bool trajectory     = false;
    bool turningRadius  = false;

    /// Off by default since the circuit itself (rendering/TrackMesh.h) became
    /// the spatial reference. Drawn unlit, the 10 m grid covers every ground
    /// surface in the scene with bright lines and reads as graph paper under
    /// the grass rather than as a measurement aid.
    bool grid           = false;

    /// Metres per unit for the velocity arrow. At 1.0 a 30 m/s velocity draws
    /// a 30 m arrow, which is unusable; 0.35 keeps it readable.
    float vectorScale = 0.35f;
};

class DebugDraw {
public:
    void beginFrame();

    //-- primitives -------------------------------------------------------
    void line(const glm::dvec3& a, const glm::dvec3& b, const glm::vec3& color);
    void ray(const glm::dvec3& origin, const glm::dvec3& direction,
             double length, const glm::vec3& color);
    void arrow(const glm::dvec3& origin, const glm::dvec3& direction,
               double length, const glm::vec3& color);
    void cross(const glm::dvec3& centre, double size, const glm::vec3& color);
    void circle(const glm::dvec3& centre, double radius, const glm::vec3& color,
                int segments = 64);
    void axes(const glm::dvec3& origin, const glm::dvec3& x,
              const glm::dvec3& y, const glm::dvec3& z, double length);
    void polyline(const std::vector<glm::dvec3>& points, const glm::vec3& color);

    /// Upload and draw everything accumulated this frame.
    void flush(const Shader& shader);

    std::size_t lineCount() const { return m_vertices.size() / 2; }

private:
    std::vector<Vertex> m_vertices;
    DynamicMesh m_mesh;
};

//---------------------------------------------------------------------------
//  Standard colours, so the same quantity is always the same colour.
//---------------------------------------------------------------------------
namespace debugcolor {
inline constexpr glm::vec3 kAxisX     {0.95f, 0.30f, 0.30f};  // forward / +X
inline constexpr glm::vec3 kAxisY     {0.35f, 0.90f, 0.40f};  // up / +Y
inline constexpr glm::vec3 kAxisZ     {0.35f, 0.55f, 1.00f};  // right / +Z
inline constexpr glm::vec3 kVelocity  {1.00f, 0.85f, 0.20f};
inline constexpr glm::vec3 kHeading   {1.00f, 0.45f, 0.10f};
inline constexpr glm::vec3 kSteering  {0.60f, 0.35f, 1.00f};
inline constexpr glm::vec3 kContact   {1.00f, 0.25f, 0.55f};
inline constexpr glm::vec3 kTrajectory{0.20f, 0.95f, 0.85f};
inline constexpr glm::vec3 kTurnCircle{0.95f, 0.60f, 0.95f};
inline constexpr glm::vec3 kCentreOfMass{1.00f, 1.00f, 1.00f};
} // namespace debugcolor

} // namespace vsim
