#include "rendering/MeshFactory.h"

#include <cmath>

#include "core/Coordinates.h"
#include "core/Units.h"

namespace vsim {

void MeshData::append(const MeshData& other) {
    const gl::GLuint base = static_cast<gl::GLuint>(vertices.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    indices.reserve(indices.size() + other.indices.size());
    for (gl::GLuint i : other.indices) indices.push_back(base + i);
}

void MeshData::uploadTo(Mesh& mesh, gl::GLenum primitive) const {
    mesh.upload(vertices, indices, primitive);
}

namespace meshfactory {
namespace {

void addLine(MeshData& m, const glm::vec3& a, const glm::vec3& b,
             const glm::vec3& color) {
    m.vertices.push_back({a, {0, 1, 0}, color});
    m.vertices.push_back({b, {0, 1, 0}, color});
}

/// Normal of the triangle a-b-c, or a zero vector for a degenerate one.
/// Degenerate triangles are normal in generated geometry - a lofted hull
/// closes to a point at the nose, a swept ribbon can have a zero-width
/// section - so this reports the failure instead of returning a NaN.
glm::vec3 faceNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    const glm::vec3 n = glm::cross(b - a, c - a);
    const float len = glm::length(n);
    return (len > 1e-12f) ? (n / len) : glm::vec3(0.0f);
}

} // namespace

//---------------------------------------------------------------------------
//  Low-level emitters
//---------------------------------------------------------------------------

void addQuad(MeshData& m, const glm::vec3& a, const glm::vec3& b,
             const glm::vec3& c, const glm::vec3& d,
             const glm::vec3& normal, const glm::vec3& color) {
    const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
    m.vertices.push_back({a, normal, color});
    m.vertices.push_back({b, normal, color});
    m.vertices.push_back({c, normal, color});
    m.vertices.push_back({d, normal, color});
    m.indices.insert(m.indices.end(),
                     {base, base + 1, base + 2, base, base + 2, base + 3});
}

void addQuadOriented(MeshData& m, const glm::vec3& a, const glm::vec3& b,
                     const glm::vec3& c, const glm::vec3& d,
                     const glm::vec3& outward, const glm::vec3& color) {
    // Take the normal from whichever of the two triangles is not degenerate -
    // a quad with one collapsed edge (the tip of a lofted nose) still has a
    // well-defined facing.
    glm::vec3 n = faceNormal(a, b, c);
    if (glm::length(n) < 0.5f) n = faceNormal(a, c, d);
    if (glm::length(n) < 0.5f) return;   // fully degenerate: nothing to draw

    if (glm::dot(n, outward) < 0.0f) {
        addQuad(m, d, c, b, a, -n, color);
    } else {
        addQuad(m, a, b, c, d, n, color);
    }
}

void addTriangleOriented(MeshData& m, const glm::vec3& a, const glm::vec3& b,
                         const glm::vec3& c, const glm::vec3& outward,
                         const glm::vec3& color) {
    glm::vec3 n = faceNormal(a, b, c);
    if (glm::length(n) < 0.5f) return;

    const bool flip = (glm::dot(n, outward) < 0.0f);
    if (flip) n = -n;

    const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
    m.vertices.push_back({flip ? c : a, n, color});
    m.vertices.push_back({b, n, color});
    m.vertices.push_back({flip ? a : c, n, color});
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
}

void addHexahedronTopped(MeshData& m, const glm::vec3 c[8],
                         const glm::vec3& side, const glm::vec3& top) {
    // Centroid gives every face an unambiguous "outward", so the caller never
    // has to reason about winding for a sheared or tapered solid.
    glm::vec3 centre(0.0f);
    for (int i = 0; i < 8; ++i) centre += c[i];
    centre /= 8.0f;

    auto face = [&](int i0, int i1, int i2, int i3, const glm::vec3& col) {
        const glm::vec3 mid = 0.25f * (c[i0] + c[i1] + c[i2] + c[i3]);
        addQuadOriented(m, c[i0], c[i1], c[i2], c[i3], mid - centre, col);
    };

    face(0, 1, 2, 3, side);   // bottom
    face(4, 5, 6, 7, top);    // top
    face(0, 1, 5, 4, side);
    face(1, 2, 6, 5, side);
    face(2, 3, 7, 6, side);
    face(3, 0, 4, 7, side);
}

void addHexahedron(MeshData& m, const glm::vec3 c[8], const glm::vec3& color) {
    addHexahedronTopped(m, c, color, color);
}

//---------------------------------------------------------------------------

MeshData box(const glm::vec3& h, const glm::vec3& color) {
    return boxAt(glm::vec3(0.0f), h, color);
}

MeshData boxAt(const glm::vec3& c, const glm::vec3& h, const glm::vec3& color) {
    MeshData m;

    const glm::vec3 p000{c.x - h.x, c.y - h.y, c.z - h.z};
    const glm::vec3 p100{c.x + h.x, c.y - h.y, c.z - h.z};
    const glm::vec3 p110{c.x + h.x, c.y + h.y, c.z - h.z};
    const glm::vec3 p010{c.x - h.x, c.y + h.y, c.z - h.z};
    const glm::vec3 p001{c.x - h.x, c.y - h.y, c.z + h.z};
    const glm::vec3 p101{c.x + h.x, c.y - h.y, c.z + h.z};
    const glm::vec3 p111{c.x + h.x, c.y + h.y, c.z + h.z};
    const glm::vec3 p011{c.x - h.x, c.y + h.y, c.z + h.z};

    // Winding is counter-clockwise when viewed from outside, matching the
    // gl::CCW front-face setting in Renderer.
    addQuad(m, p100, p101, p111, p110, { 1,  0,  0}, color);  // +X (front)
    addQuad(m, p001, p000, p010, p011, {-1,  0,  0}, color);  // -X (rear)
    addQuad(m, p010, p110, p111, p011, { 0,  1,  0}, color);  // +Y (roof)
    addQuad(m, p001, p101, p100, p000, { 0, -1,  0}, color);  // -Y (floor)
    addQuad(m, p101, p001, p011, p111, { 0,  0,  1}, color);  // +Z (right)
    addQuad(m, p000, p100, p110, p010, { 0,  0, -1}, color);  // -Z (left)

    return m;
}

//---------------------------------------------------------------------------

MeshData wheel(float radius, float width, int segments, const glm::vec3& color) {
    MeshData m;
    if (segments < 3) segments = 3;

    const float hw = width * 0.5f;

    // Tread band. The wheel spins about local +Z, so the circle is swept in
    // the local X-Y plane and extruded along Z.
    for (int i = 0; i < segments; ++i) {
        const float a0 = kTwoPi * static_cast<float>(i)       / static_cast<float>(segments);
        const float a1 = kTwoPi * static_cast<float>(i + 1)   / static_cast<float>(segments);

        const glm::vec3 d0{std::cos(a0), std::sin(a0), 0.0f};
        const glm::vec3 d1{std::cos(a1), std::sin(a1), 0.0f};

        const glm::vec3 o0 = d0 * radius;
        const glm::vec3 o1 = d1 * radius;

        // Darken one quadrant so the wheel's rotation is visible on screen.
        // Without this a smooth cylinder looks completely static while rolling.
        const bool marker = (a0 < kPi * 0.5f);
        const glm::vec3 c = marker ? color * 0.45f : color;

        addQuad(m,
                {o0.x, o0.y, -hw}, {o1.x, o1.y, -hw},
                {o1.x, o1.y,  hw}, {o0.x, o0.y,  hw},
                glm::normalize(d0 + d1), c);
    }

    // Side discs (hub caps), so the wheel is not hollow when seen from the side.
    for (int side = 0; side < 2; ++side) {
        const float z = (side == 0) ? -hw : hw;
        const glm::vec3 n = (side == 0) ? glm::vec3(0, 0, -1) : glm::vec3(0, 0, 1);

        const gl::GLuint centre = static_cast<gl::GLuint>(m.vertices.size());
        m.vertices.push_back({{0.0f, 0.0f, z}, n, color * 1.25f});

        for (int i = 0; i <= segments; ++i) {
            const float a = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
            m.vertices.push_back({{std::cos(a) * radius, std::sin(a) * radius, z},
                                  n, color * 1.1f});
        }
        for (int i = 0; i < segments; ++i) {
            const gl::GLuint a = centre + 1 + static_cast<gl::GLuint>(i);
            const gl::GLuint b = centre + 1 + static_cast<gl::GLuint>(i + 1);
            if (side == 0) m.indices.insert(m.indices.end(), {centre, b, a});
            else           m.indices.insert(m.indices.end(), {centre, a, b});
        }
    }

    return m;
}

//---------------------------------------------------------------------------

MeshData groundPlane(float halfExtent, const glm::vec3& color) {
    MeshData m;
    addQuad(m,
            {-halfExtent, 0.0f, -halfExtent},
            {-halfExtent, 0.0f,  halfExtent},
            { halfExtent, 0.0f,  halfExtent},
            { halfExtent, 0.0f, -halfExtent},
            {0, 1, 0}, color);
    return m;
}

MeshData grid(float halfExtent, float spacing,
              const glm::vec3& minorColor, const glm::vec3& majorColor,
              int majorEvery) {
    MeshData m;
    if (spacing <= 0.0f) return m;

    const int n = static_cast<int>(halfExtent / spacing);
    // Raised slightly above y=0 so the grid does not z-fight with the ground.
    const float y = 0.012f;

    for (int i = -n; i <= n; ++i) {
        const float p = static_cast<float>(i) * spacing;
        const bool major = (majorEvery > 0) && (i % majorEvery == 0);
        const glm::vec3 c = major ? majorColor : minorColor;

        addLine(m, {p, y, -halfExtent}, {p, y, halfExtent}, c);
        addLine(m, {-halfExtent, y, p}, {halfExtent, y, p}, c);
    }
    return m;
}

//---------------------------------------------------------------------------

MeshData roadSurface(const Track& track, const glm::vec3& color) {
    MeshData m;
    const auto& pts = track.centreline();
    if (pts.size() < 2) return m;

    const float halfWidth = static_cast<float>(track.width() * 0.5);
    // 1 cm above the ground plane. Enough to beat depth precision at the far
    // plane without being visible as a step.
    const float y = 0.01f;

    // A closed track (Track::createRaceCircuit()) gets one extra quad
    // connecting the last centreline point back to the first, so the road
    // ribbon forms an unbroken loop instead of stopping short of its own
    // start/finish line. An open, point-to-point track (everything else)
    // must NOT get that quad - there is no start/finish line to reconnect to,
    // and closing it would draw a spurious strip straight across the track.
    const std::size_t segments = track.closed() ? pts.size() : pts.size() - 1;

    for (std::size_t i = 0; i < segments; ++i) {
        const std::size_t j = (i + 1 == pts.size()) ? 0 : i + 1;
        const double yaw0 = coords::yawFromDirection(pts[i].tangent);
        const double yaw1 = coords::yawFromDirection(pts[j].tangent);

        const glm::dvec3 r0 = coords::right(yaw0) * static_cast<double>(halfWidth);
        const glm::dvec3 r1 = coords::right(yaw1) * static_cast<double>(halfWidth);

        const glm::dvec3 p0 = pts[i].position;
        const glm::dvec3 p1 = pts[j].position;

        const glm::vec3 a{static_cast<float>(p0.x - r0.x), y, static_cast<float>(p0.z - r0.z)};
        const glm::vec3 b{static_cast<float>(p0.x + r0.x), y, static_cast<float>(p0.z + r0.z)};
        const glm::vec3 c{static_cast<float>(p1.x + r1.x), y, static_cast<float>(p1.z + r1.z)};
        const glm::vec3 d{static_cast<float>(p1.x - r1.x), y, static_cast<float>(p1.z - r1.z)};

        addQuad(m, a, b, c, d, {0, 1, 0}, color);
    }
    return m;
}

MeshData roadMarkings(const Track& track, float dashLength, float gapLength,
                      float lineWidth, const glm::vec3& color) {
    MeshData m;
    const auto& pts = track.centreline();
    if (pts.size() < 2) return m;

    const float halfWidth = static_cast<float>(track.width() * 0.5);
    const float hw = lineWidth * 0.5f;
    // Above the road surface, which is itself above the ground plane.
    const float y = 0.02f;

    const float period = dashLength + gapLength;

    // Same closed-loop join as roadSurface() - see its comment.
    const std::size_t segments = track.closed() ? pts.size() : pts.size() - 1;

    auto emitStrip = [&](float lateralOffset, bool dashed) {
        for (std::size_t i = 0; i < segments; ++i) {
            const std::size_t j = (i + 1 == pts.size()) ? 0 : i + 1;
            if (dashed) {
                // Dash pattern keyed to arc length, so dashes stay a constant
                // physical length through curves rather than stretching.
                const float s = static_cast<float>(pts[i].distance);
                if (std::fmod(s, period) > dashLength) continue;
            }

            const double yaw0 = coords::yawFromDirection(pts[i].tangent);
            const double yaw1 = coords::yawFromDirection(pts[j].tangent);
            const glm::dvec3 rt0 = coords::right(yaw0);
            const glm::dvec3 rt1 = coords::right(yaw1);

            const glm::dvec3 c0 = pts[i].position + rt0 * static_cast<double>(lateralOffset);
            const glm::dvec3 c1 = pts[j].position + rt1 * static_cast<double>(lateralOffset);

            const glm::dvec3 e0 = rt0 * static_cast<double>(hw);
            const glm::dvec3 e1 = rt1 * static_cast<double>(hw);

            addQuad(m,
                    {static_cast<float>(c0.x - e0.x), y, static_cast<float>(c0.z - e0.z)},
                    {static_cast<float>(c0.x + e0.x), y, static_cast<float>(c0.z + e0.z)},
                    {static_cast<float>(c1.x + e1.x), y, static_cast<float>(c1.z + e1.z)},
                    {static_cast<float>(c1.x - e1.x), y, static_cast<float>(c1.z - e1.z)},
                    {0, 1, 0}, color);
        }
    };

    emitStrip(0.0f, true);                    // dashed centre line
    emitStrip(-halfWidth + hw * 2.0f, false); // solid left edge
    emitStrip( halfWidth - hw * 2.0f, false); // solid right edge

    return m;
}

//---------------------------------------------------------------------------

MeshData cone(float radius, float height, int segments, const glm::vec3& color) {
    MeshData m;
    if (segments < 3) segments = 3;

    const gl::GLuint apex = 0;
    m.vertices.push_back({{0.0f, height, 0.0f}, {0, 1, 0}, color * 1.3f});

    for (int i = 0; i <= segments; ++i) {
        const float a = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const glm::vec3 p{std::cos(a) * radius, 0.0f, std::sin(a) * radius};
        m.vertices.push_back({p, glm::normalize(glm::vec3(p.x, radius * 0.5f, p.z)), color});
    }
    for (int i = 0; i < segments; ++i) {
        m.indices.insert(m.indices.end(),
                         {apex,
                          static_cast<gl::GLuint>(i + 1),
                          static_cast<gl::GLuint>(i + 2)});
    }
    return m;
}

MeshData cylinderAlong(const glm::vec3& base, const glm::vec3& axis,
                       float radius, int segments, const glm::vec3& color,
                       bool capped) {
    MeshData m;
    if (segments < 3) segments = 3;

    const float len = glm::length(axis);
    if (len < 1e-6f || radius <= 0.0f) return m;

    const glm::vec3 w = axis / len;

    // Any vector not parallel to the axis gives a usable first basis vector;
    // picking the world axis the cylinder leans on LEAST keeps the cross
    // product well conditioned for a cylinder pointing any direction.
    const glm::vec3 seed = (std::abs(w.y) < 0.9f) ? glm::vec3(0, 1, 0)
                                                  : glm::vec3(1, 0, 0);
    const glm::vec3 u = glm::normalize(glm::cross(seed, w));
    const glm::vec3 v = glm::cross(w, u);

    const glm::vec3 top = base + axis;

    for (int i = 0; i < segments; ++i) {
        const float a0 = kTwoPi * static_cast<float>(i)     / static_cast<float>(segments);
        const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);

        const glm::vec3 d0 = u * std::cos(a0) + v * std::sin(a0);
        const glm::vec3 d1 = u * std::cos(a1) + v * std::sin(a1);

        addQuad(m, base + d0 * radius, base + d1 * radius,
                   top  + d1 * radius, top  + d0 * radius,
                glm::normalize(d0 + d1), color);

        if (capped) {
            const glm::vec3 capColor = color * 1.08f;
            addTriangleOriented(m, base, base + d1 * radius, base + d0 * radius,
                                -w, capColor);
            addTriangleOriented(m, top, top + d0 * radius, top + d1 * radius,
                                w, capColor);
        }
    }
    return m;
}

MeshData sphere(float radius, int subdivisions, const glm::vec3& color) {
    MeshData m;
    // UV sphere - plenty for a small marker, and far simpler than subdividing
    // an icosahedron for something a few pixels across.
    const int rings   = 2 + subdivisions * 2;
    const int sectors = 4 + subdivisions * 4;

    for (int r = 0; r <= rings; ++r) {
        const float phi = kPi * static_cast<float>(r) / static_cast<float>(rings);
        for (int s = 0; s <= sectors; ++s) {
            const float theta = kTwoPi * static_cast<float>(s) / static_cast<float>(sectors);
            const glm::vec3 n{std::sin(phi) * std::cos(theta),
                              std::cos(phi),
                              std::sin(phi) * std::sin(theta)};
            m.vertices.push_back({n * radius, n, color});
        }
    }
    const int stride = sectors + 1;
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const gl::GLuint a = static_cast<gl::GLuint>(r * stride + s);
            const gl::GLuint b = static_cast<gl::GLuint>(a + stride);
            m.indices.insert(m.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
        }
    }
    return m;
}

} // namespace meshfactory
} // namespace vsim
