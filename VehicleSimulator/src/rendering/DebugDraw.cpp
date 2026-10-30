#include "rendering/DebugDraw.h"

#include <cmath>

#include "core/Coordinates.h"
#include "core/Units.h"
#include "rendering/Shader.h"

namespace vsim {

void DebugDraw::beginFrame() {
    // Keep the capacity: the overlay set is stable frame to frame, so this
    // reaches steady state after one frame and never allocates again.
    m_vertices.clear();
}

void DebugDraw::line(const glm::dvec3& a, const glm::dvec3& b, const glm::vec3& c) {
    // A NaN here would poison the whole vertex buffer and make every debug
    // line vanish, which is a maddening thing to debug. Drop bad segments.
    if (!isFinite(a.x) || !isFinite(a.y) || !isFinite(a.z) ||
        !isFinite(b.x) || !isFinite(b.y) || !isFinite(b.z)) {
        return;
    }
    m_vertices.push_back({glm::vec3(a), {0, 1, 0}, c});
    m_vertices.push_back({glm::vec3(b), {0, 1, 0}, c});
}

void DebugDraw::ray(const glm::dvec3& o, const glm::dvec3& d, double len,
                    const glm::vec3& c) {
    const double l = glm::length(d);
    if (l < 1e-9) return;
    line(o, o + (d / l) * len, c);
}

void DebugDraw::arrow(const glm::dvec3& o, const glm::dvec3& d, double len,
                      const glm::vec3& c) {
    const double l = glm::length(d);
    if (l < 1e-9 || len < 1e-6) return;

    const glm::dvec3 dir = d / l;
    const glm::dvec3 tip = o + dir * len;
    line(o, tip, c);

    // Two barbs in the plane containing the direction and world up. If the
    // direction is nearly vertical, fall back to +X so the cross product does
    // not collapse.
    glm::dvec3 ref = coords::kWorldUp;
    if (std::abs(glm::dot(dir, ref)) > 0.99) ref = glm::dvec3(1.0, 0.0, 0.0);

    const glm::dvec3 side = glm::normalize(glm::cross(dir, ref));
    const double headLen = std::min(len * 0.18, 0.6);

    line(tip, tip - dir * headLen + side * headLen * 0.5, c);
    line(tip, tip - dir * headLen - side * headLen * 0.5, c);
}

void DebugDraw::cross(const glm::dvec3& p, double s, const glm::vec3& c) {
    line(p - glm::dvec3(s, 0, 0), p + glm::dvec3(s, 0, 0), c);
    line(p - glm::dvec3(0, s, 0), p + glm::dvec3(0, s, 0), c);
    line(p - glm::dvec3(0, 0, s), p + glm::dvec3(0, 0, s), c);
}

void DebugDraw::circle(const glm::dvec3& centre, double radius,
                       const glm::vec3& c, int segments) {
    if (!(radius > 0.0) || !isFinite(radius) || segments < 3) return;

    // Very large circles (near-straight paths) would run off to the horizon
    // and produce enormous coordinates. Cap it.
    if (radius > 5000.0) return;

    glm::dvec3 prev = centre + glm::dvec3(radius, 0.0, 0.0);
    for (int i = 1; i <= segments; ++i) {
        const double a = kTwoPi * static_cast<double>(i) / segments;
        const glm::dvec3 p = centre + glm::dvec3(std::cos(a) * radius, 0.0,
                                                 std::sin(a) * radius);
        line(prev, p, c);
        prev = p;
    }
}

void DebugDraw::axes(const glm::dvec3& o, const glm::dvec3& x,
                     const glm::dvec3& y, const glm::dvec3& z, double len) {
    arrow(o, x, len, debugcolor::kAxisX);
    arrow(o, y, len, debugcolor::kAxisY);
    arrow(o, z, len, debugcolor::kAxisZ);
}

void DebugDraw::polyline(const std::vector<glm::dvec3>& pts, const glm::vec3& c) {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) line(pts[i], pts[i + 1], c);
}

void DebugDraw::flush(const Shader& shader) {
    if (m_vertices.empty()) return;

    // Debug lines use the same shader as everything else, with the model
    // matrix set to identity because all coordinates are already in world
    // space. Unlit, so the vertex colour comes through exactly.
    shader.use();
    shader.setMat4("uModel", glm::mat4(1.0f));
    shader.setInt("uUnlit", 1);

    m_mesh.setVertices(m_vertices, gl::LINES);
    m_mesh.draw();

    shader.setInt("uUnlit", 0);
}

} // namespace vsim
