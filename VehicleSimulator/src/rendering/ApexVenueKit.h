#pragma once
//===========================================================================
//  ApexVenueKit.h  --  the small toolkit ApexVenue.cpp is written in.
//
//  Internal to the venue builder (included by ApexVenue*.cpp only). Three
//  things live here:
//
//    * EMITTERS that write material-tagged geometry - the meshfactory ones
//      predate Vertex::material and always write material 0.
//    * A 5x7 DOT FONT, so signage can carry numerals and names without a
//      texture: a distance board has to say 100, not show three stripes.
//    * LAP, the centreline as a sequence of frames that can be sampled at
//      any arc length and wraps across the start line.
//===========================================================================

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "environment/Track.h"
#include "rendering/Venue.h"

namespace vsim::venuekit {

using glm::vec2;
using glm::vec3;

inline const vec3 kUp{0.0f, 1.0f, 0.0f};
constexpr float kPiF = 3.14159265358979f;

// Flat layers, as TrackMesh: a few millimetres apart, ordered by polygon
// offset in the renderer rather than by these heights alone.
constexpr float kGrassY   = 0.004f;
constexpr float kAsphaltY = 0.010f;
constexpr float kPaintY   = 0.016f;

/// Colour + material, the unit every emitter takes.
struct Brush {
    vec3  color{1.0f};
    float mat   = venuemat::kDefault;
    float param = 0.0f;

    Brush shade(float k) const { return {color * k, mat, param}; }
    Brush with(float p) const { return {color, mat, p}; }
};

/// Deterministic value in [0,1): the venue must be identical every launch.
inline float hash01(int a, int b = 0) {
    unsigned int h = static_cast<unsigned int>(a) * 374761393u +
                     static_cast<unsigned int>(b) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<float>((h ^ (h >> 16)) & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

inline float smooth01(float u) {
    u = std::min(std::max(u, 0.0f), 1.0f);
    return u * u * (3.0f - 2.0f * u);
}
inline float smoothstep(float a, float b, float x) { return smooth01((x - a) / (b - a)); }

//---------------------------------------------------------------------------
//  Emitters
//---------------------------------------------------------------------------

inline Vertex vtx(const vec3& p, const vec3& n, const Brush& br, const vec2& uv = vec2(0.0f)) {
    Vertex v;
    v.position = p;
    v.normal   = n;
    v.color    = br.color;
    v.uv       = uv;
    v.material = {br.mat, br.param};
    return v;
}

/// Four finished vertices, a-b-c-d around the quad, wound to face `outward`.
inline void quadV(MeshData& m, Vertex a, Vertex b, Vertex c, Vertex d, const vec3& outward) {
    vec3 n = glm::cross(b.position - a.position, c.position - a.position);
    if (glm::dot(n, n) < 1e-12f) n = glm::cross(c.position - a.position, d.position - a.position);
    const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
    if (glm::dot(n, outward) < 0.0f) {
        m.vertices.push_back(d); m.vertices.push_back(c);
        m.vertices.push_back(b); m.vertices.push_back(a);
    } else {
        m.vertices.push_back(a); m.vertices.push_back(b);
        m.vertices.push_back(c); m.vertices.push_back(d);
    }
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

/// Flat quad; the normal is derived and made to agree with `outward`.
inline void quad(MeshData& m, const vec3& a, const vec3& b, const vec3& c, const vec3& d,
                 const vec3& outward, const Brush& br,
                 const vec2& ua = vec2(0.0f), const vec2& ub = vec2(0.0f),
                 const vec2& uc = vec2(0.0f), const vec2& ud = vec2(0.0f)) {
    vec3 n = glm::cross(b - a, c - a);
    if (glm::dot(n, n) < 1e-12f) n = glm::cross(c - a, d - a);
    const float len = glm::length(n);
    if (len < 1e-9f) return;
    n /= len;
    if (glm::dot(n, outward) < 0.0f) n = -n;
    quadV(m, vtx(a, n, br, ua), vtx(b, n, br, ub), vtx(c, n, br, uc), vtx(d, n, br, ud), n);
}

inline void tri(MeshData& m, const vec3& a, const vec3& b, const vec3& c,
                const vec3& outward, const Brush& br) {
    vec3 n = glm::cross(b - a, c - a);
    const float len = glm::length(n);
    if (len < 1e-9f) return;
    n /= len;
    const bool flip = glm::dot(n, outward) < 0.0f;
    if (flip) n = -n;
    const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
    m.vertices.push_back(vtx(flip ? c : a, n, br));
    m.vertices.push_back(vtx(b, n, br));
    m.vertices.push_back(vtx(flip ? a : c, n, br));
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
}

/// A local frame on the ground: x along, z across, y up. `z` need not be
/// x rotated the "right" way - a mirrored frame is fine, because every
/// emitter orients its faces from the geometry, not from the winding.
struct Basis {
    vec3 o{0.0f};
    vec3 x{1.0f, 0.0f, 0.0f};
    vec3 z{0.0f, 0.0f, 1.0f};

    vec3 at(float px, float py, float pz) const { return o + x * px + kUp * py + z * pz; }
    Basis moved(float px, float py, float pz) const { return {at(px, py, pz), x, z}; }
};

/// Six-faced solid from eight corners (c[0..3] one face in order, c[4..7]
/// the face opposite). `top`, if given, paints the c[4..7] face.
inline void hexa(MeshData& m, const vec3 c[8], const Brush& br, const Brush* top = nullptr,
                 bool bottom = true) {
    vec3 centre(0.0f);
    for (int i = 0; i < 8; ++i) centre += c[i];
    centre /= 8.0f;
    auto face = [&](int i0, int i1, int i2, int i3, const Brush& b) {
        const vec3 mid = 0.25f * (c[i0] + c[i1] + c[i2] + c[i3]);
        quad(m, c[i0], c[i1], c[i2], c[i3], mid - centre, b);
    };
    if (bottom) face(0, 1, 2, 3, br);
    face(4, 5, 6, 7, top ? *top : br);
    face(0, 1, 5, 4, br);
    face(1, 2, 6, 5, br);
    face(2, 3, 7, 6, br);
    face(3, 0, 4, 7, br);
}

/// Box in a basis. The bottom face is left out when the box stands on the
/// ground: nothing can see it, and nothing it would shadow is visible.
inline void box(MeshData& m, const Basis& b, float x0, float x1, float y0, float y1,
                float z0, float z1, const Brush& br, const Brush* top = nullptr) {
    const vec3 c[8] = {
        b.at(x0, y0, z0), b.at(x0, y0, z1), b.at(x1, y0, z1), b.at(x1, y0, z0),
        b.at(x0, y1, z0), b.at(x0, y1, z1), b.at(x1, y1, z1), b.at(x1, y1, z0),
    };
    hexa(m, c, br, top, std::min(y0, y1) > 0.05f);
}

/// Cylinder from `base` along `axis`, smooth-shaded, optionally capped at
/// the far end.
inline void cylinder(MeshData& m, const vec3& base, const vec3& axis, float radius,
                     int segments, const Brush& br, bool cap = true) {
    const float len = glm::length(axis);
    if (len < 1e-6f) return;
    const vec3 a = axis / len;
    const vec3 ref = (std::abs(a.y) < 0.9f) ? kUp : vec3(1.0f, 0.0f, 0.0f);
    const vec3 u = glm::normalize(glm::cross(ref, a));
    const vec3 v = glm::cross(a, u);
    const vec3 top = base + axis;
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * kPiF * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = 2.0f * kPiF * static_cast<float>(i + 1) / static_cast<float>(segments);
        const vec3 n0 = u * std::cos(a0) + v * std::sin(a0);
        const vec3 n1 = u * std::cos(a1) + v * std::sin(a1);
        quadV(m, vtx(base + n0 * radius, n0, br), vtx(base + n1 * radius, n1, br),
              vtx(top + n1 * radius, n1, br), vtx(top + n0 * radius, n0, br), n0 + n1);
        if (cap) tri(m, top, top + n0 * radius, top + n1 * radius, a, br);
    }
}

/// Cone standing on `base`, smooth-shaded, open underneath.
inline void cone(MeshData& m, const vec3& base, float radius, float height, int segments,
                 const Brush& br) {
    const vec3 apex = base + kUp * height;
    const float slope = radius / std::max(height, 1e-4f);
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * kPiF * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = 2.0f * kPiF * static_cast<float>(i + 1) / static_cast<float>(segments);
        const vec3 d0(std::cos(a0), 0.0f, std::sin(a0));
        const vec3 d1(std::cos(a1), 0.0f, std::sin(a1));
        const vec3 n0 = glm::normalize(d0 + kUp * slope);
        const vec3 n1 = glm::normalize(d1 + kUp * slope);
        const gl::GLuint b0 = static_cast<gl::GLuint>(m.vertices.size());
        m.vertices.push_back(vtx(base + d0 * radius, n0, br));
        m.vertices.push_back(vtx(apex, glm::normalize(n0 + n1), br));
        m.vertices.push_back(vtx(base + d1 * radius, n1, br));
        m.indices.insert(m.indices.end(), {b0, b0 + 1, b0 + 2});
    }
}

/// Mark every vertex added since `from` as keeping its own colour under an
/// instance tint (see InstanceData).
inline void fixColour(MeshData& m, std::size_t from) {
    for (std::size_t i = from; i < m.vertices.size(); ++i) m.vertices[i].material.y = 1.0f;
}

//---------------------------------------------------------------------------
//  Dot font
//---------------------------------------------------------------------------

/// Width of `text` in dots: five per glyph plus one of spacing.
inline float textDots(const std::string& text) {
    return text.empty() ? 0.0f : static_cast<float>(text.size()) * 6.0f - 1.0f;
}

/// Rows of the 5x7 glyph for `ch`, top row first, bit 4 = leftmost dot.
const unsigned char* glyph(char ch);

/// Lay `text` out from `origin` (bottom-left) along `dx`, glyph-up along
/// `dy`, facing `normal`. One dot is `px` by `py`.
void textQuads(MeshData& m, const vec3& origin, const vec3& dx, const vec3& dy,
               const vec3& normal, const std::string& text, float px, float py,
               const Brush& br);

/// Upright text centred on `centre`, on a surface facing `normal`, read
/// from the side `normal` points at.
inline void label(MeshData& m, const vec3& centre, const vec3& normal,
                  const std::string& text, float px, const Brush& br) {
    const vec3 n = glm::normalize(normal);
    const vec3 dx = glm::normalize(glm::cross(kUp, n));
    const vec3 origin = centre - dx * (0.5f * textDots(text) * px) - kUp * (3.5f * px) + n * 0.02f;
    textQuads(m, origin, dx, kUp, n, text, px, px, br);
}

//---------------------------------------------------------------------------
//  The lap
//---------------------------------------------------------------------------

struct Frame {
    vec3  pos{0.0f};
    vec3  fwd{1.0f, 0.0f, 0.0f};
    vec3  right{0.0f, 0.0f, 1.0f};
    float s = 0.0f;
    float curv = 0.0f;   ///< signed, + = right-hand corner

    vec3 at(float lateral, float y) const { return pos + right * lateral + kUp * y; }
};

struct Lap {
    std::vector<Frame> f;
    float length = 0.0f;
    float hw = 7.0f;

    explicit Lap(const Track& track);

    int   n() const { return static_cast<int>(f.size()); }
    int   wrap(int i) const { return ((i % n()) + n()) % n(); }
    float wrapS(float s) const { s = std::fmod(s, length); return (s < 0.0f) ? s + length : s; }

    /// Signed shortest way round from `ref` to `s`, in (-length/2, length/2].
    float rel(float s, float ref) const {
        float d = std::fmod(s - ref, length);
        if (d > 0.5f * length) d -= length;
        if (d <= -0.5f * length) d += length;
        return d;
    }

    /// Index of the sample at or before `s`, and how far to the next, 0..1.
    int locate(float s, float& u) const;

    Frame at(float s) const;

    /// A per-sample array, sampled at an arbitrary arc length.
    float sample(const std::vector<float>& a, float s) const {
        float u = 0.0f;
        const int i = locate(s, u);
        return a[static_cast<std::size_t>(i)] * (1.0f - u) + a[static_cast<std::size_t>(wrap(i + 1))] * u;
    }
};

} // namespace vsim::venuekit
