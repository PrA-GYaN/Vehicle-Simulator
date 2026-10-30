#include "rendering/CarMesh.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "core/Units.h"

namespace vsim {
namespace {

constexpr float kPiF   = 3.14159265358979f;
constexpr float kTwoPiF = 6.28318530717959f;

constexpr int kRingSegments  = 56;   ///< cross-section samples, body loft
constexpr int kGhSegments    = 33;   ///< cross-section samples, greenhouse half-loft
constexpr int kBodySections  = 120;   ///< sections after spline resampling
constexpr int kGhSections    = 40;

float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

/// Signed power, the superellipse building block: sgnPow(cos t, 2/n) traces a
/// rounded square for n > 2 and an ellipse for n = 2.
float sgnPow(float v, float e) {
    return (v < 0.0f ? -1.0f : 1.0f) * std::pow(std::abs(v), e);
}

/// Uniform Catmull-Rom through p1 and p2. Used to interpolate the authored
/// control sections: linear interpolation would leave a visible crease in the
/// skin at every control station, which on a car body reads as a dent.
float catmull(float p0, float p1, float p2, float p3, float u) {
    const float u2 = u * u;
    const float u3 = u2 * u;
    return 0.5f * ((2.0f * p1) +
                   (-p0 + p2) * u +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * u2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * u3);
}

//===========================================================================
//  Cross-sections and lofting
//===========================================================================

/// One cross-section of a loft, in vehicle local metres.
///
/// The section is a superellipse split at `shoulder` - the widest point -
/// with its own exponent above and below. A high exponent is boxy (an SUV
/// flank, a roof), a low one is round (a nose cone). That single degree of
/// freedom is most of what separates the four body styles.
struct Station {
    float t        = 0.0f;   ///< 0 at the nose, 1 at the tail
    float x        = 0.0f;   ///< [m] vehicle local, from the rear axle
    float halfWidth= 0.0f;   ///< [m]
    float bottom   = 0.0f;   ///< [m] above ground
    float shoulder = 0.0f;
    float top      = 0.0f;
    float topPower = 3.0f;
    float botPower = 4.0f;
};

/// Authored control section: identical, but normalised - width as a fraction
/// of the half body width, heights as fractions of the body height. This is
/// what makes one table describe the shape of any car of that family.
struct Control {
    float t, halfWidth, bottom, shoulder, top, topPower, botPower;
};

glm::vec3 stationPoint(const Station& s, float theta) {
    const float c = std::cos(theta);
    const float sn = std::sin(theta);
    const bool upper = (sn >= 0.0f);

    const float n = std::max(upper ? s.topPower : s.botPower, 1.05f);
    const float b = upper ? (s.top - s.shoulder) : (s.shoulder - s.bottom);
    const float e = 2.0f / n;

    return {s.x, s.shoulder + b * sgnPow(sn, e), s.halfWidth * sgnPow(c, e)};
}

/// A skinned run of sections, kept as a grid so that anything else that needs
/// to sit ON the surface - glass, lights, vents, livery - can be generated
/// from the same samples instead of guessing where the surface is.
///
/// CREASES
/// -------
/// A car body is not a smooth tube. It has hard lines - the shoulder running
/// down the flank, the edge where the bonnet turns down into the wing, the
/// sill - and those lines are visible precisely because the surface normal
/// JUMPS across them. Smooth-shading the whole loft is what makes a generated
/// car look like a bar of soap no matter how good its silhouette is.
///
/// A crease is therefore stored as a DUPLICATED ring sample: the same angle
/// appears twice, in two different spans, and normals are computed from
/// neighbours within a span only. The two copies then carry the two different
/// normals that make the crease catch the light.
struct Loft {
    std::vector<Station>   st;
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> nrm;
    std::vector<float>     ang;   ///< angle of each ring sample
    std::vector<int>       span;  ///< which smooth run each sample belongs to
    int   ring   = 0;
    bool  closed = true;          ///< ring wraps at 2*pi (body) or stops (greenhouse)

    int sections() const { return static_cast<int>(st.size()); }
    int at(int i, int k) const { return i * ring + k; }
    const glm::vec3& P(int i, int k) const { return pos[at(i, k)]; }
    const glm::vec3& N(int i, int k) const { return nrm[at(i, k)]; }
    float theta(int k) const { return ang[static_cast<std::size_t>(k)]; }

    /// Section index nearest a normalised longitudinal position.
    int sectionAt(float t) const {
        int best = 0;
        float bestD = 1e9f;
        for (int i = 0; i < sections(); ++i) {
            const float d = std::abs(st[static_cast<std::size_t>(i)].t - t);
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    /// Ring index nearest an angle in degrees, measured from +Z (the car's
    /// right flank at shoulder height) toward +Y (the roof).
    int ringAt(float degrees) const {
        const float target = degrees * kPiF / 180.0f;
        int best = 0;
        float bestD = 1e9f;
        for (int k = 0; k < ring; ++k) {
            float d = std::abs(theta(k) - target);
            d = std::min(d, std::abs(kTwoPiF - d));
            if (d < bestD) { bestD = d; best = k; }
        }
        return best;
    }
};

/// Ring sample angles: `samples` evenly spaced, with every crease angle
/// inserted twice so the surface can break there.
void buildRingAngles(Loft& L, int samples, float theta0, float theta1,
                     bool closed, const std::vector<float>& creasesDeg) {
    const float span = theta1 - theta0;
    const int divisions = closed ? samples : samples - 1;

    struct Sample { float a; bool crease; };
    std::vector<Sample> list;
    list.reserve(static_cast<std::size_t>(samples + creasesDeg.size() * 2));

    for (int k = 0; k < samples; ++k) {
        list.push_back({theta0 + span * static_cast<float>(k) / static_cast<float>(divisions), false});
    }
    for (float deg : creasesDeg) {
        float a = deg * kPiF / 180.0f;
        while (a < theta0) a += kTwoPiF;
        while (a > theta1) a -= kTwoPiF;
        if (a < theta0 || a > theta1) continue;
        list.push_back({a, true});
        list.push_back({a, true});
    }

    std::sort(list.begin(), list.end(),
              [](const Sample& x, const Sample& y) { return x.a < y.a; });

    L.ang.clear();
    L.span.clear();
    int current = 0;
    for (std::size_t i = 0; i < list.size(); ++i) {
        // The FIRST of a duplicated pair ends the previous span; the second
        // starts the next one.
        if (list[i].crease && i > 0 && list[i - 1].crease &&
            std::abs(list[i].a - list[i - 1].a) < 1e-6f) {
            ++current;
        }
        L.ang.push_back(list[i].a);
        L.span.push_back(current);
    }
    L.ring = static_cast<int>(L.ang.size());
}

Loft buildLoft(std::vector<Station> stations, int ringSamples,
               float theta0, float theta1, bool closed,
               const std::vector<float>& creasesDeg = {}) {
    Loft L;
    L.st     = std::move(stations);
    L.closed = closed;
    buildRingAngles(L, ringSamples, theta0, theta1, closed, creasesDeg);

    const int n = L.sections();
    const int ring = L.ring;
    L.pos.resize(static_cast<std::size_t>(n * ring));
    L.nrm.resize(L.pos.size());

    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < ring; ++k) {
            L.pos[static_cast<std::size_t>(L.at(i, k))] =
                stationPoint(L.st[static_cast<std::size_t>(i)], L.theta(k));
        }
    }

    // Normals from the two surface tangents. Taking them from the grid rather
    // than from the faces makes the skin shade smoothly, which is the whole
    // point of lofting it instead of stacking boxes - except across a crease,
    // where the neighbour lookup deliberately stops.
    for (int i = 0; i < n; ++i) {
        const Station& s = L.st[static_cast<std::size_t>(i)];
        const glm::vec3 axis{s.x, s.shoulder, 0.0f};

        for (int k = 0; k < ring; ++k) {
            const int mySpan = L.span[static_cast<std::size_t>(k)];

            int kPrev = L.closed ? (k - 1 + ring) % ring : std::max(k - 1, 0);
            int kNext = L.closed ? (k + 1) % ring : std::min(k + 1, ring - 1);
            if (L.span[static_cast<std::size_t>(kPrev)] != mySpan) kPrev = k;
            if (L.span[static_cast<std::size_t>(kNext)] != mySpan) kNext = k;

            const int iPrev = std::max(i - 1, 0);
            const int iNext = std::min(i + 1, n - 1);

            const glm::vec3 du = L.P(i, kNext) - L.P(i, kPrev);
            const glm::vec3 dv = L.P(iNext, k) - L.P(iPrev, k);

            glm::vec3 nrm = glm::cross(dv, du);
            const float len = glm::length(nrm);

            // A collapsed section (the tip of a nose cone) has no usable
            // tangent pair; fall back to the radial direction, which is the
            // correct answer there anyway.
            const glm::vec3 radial = L.P(i, k) - axis;
            nrm = (len > 1e-9f) ? (nrm / len)
                                : (glm::length(radial) > 1e-9f ? glm::normalize(radial)
                                                               : glm::vec3(0, 1, 0));
            if (glm::dot(nrm, radial) < 0.0f) nrm = -nrm;

            L.nrm[static_cast<std::size_t>(L.at(i, k))] = nrm;
        }
    }
    return L;
}

/// Resample authored control sections onto a dense, evenly spaced run.
std::vector<Station> resample(const std::vector<Control>& ctrl, int count,
                              float length, float halfWidth, float height,
                              float xNose) {
    std::vector<Station> out;
    out.reserve(static_cast<std::size_t>(count));
    if (ctrl.size() < 2) return out;

    const int last = static_cast<int>(ctrl.size()) - 1;
    auto pick = [&](int i) -> const Control& {
        return ctrl[static_cast<std::size_t>(std::min(std::max(i, 0), last))];
    };

    for (int j = 0; j < count; ++j) {
        const float t = static_cast<float>(j) / static_cast<float>(count - 1);

        // Locate the control span containing t.
        int seg = 0;
        while (seg < last - 1 && ctrl[static_cast<std::size_t>(seg + 1)].t < t) ++seg;

        const Control& c1 = pick(seg);
        const Control& c2 = pick(seg + 1);
        const Control& c0 = pick(seg - 1);
        const Control& c3 = pick(seg + 2);

        const float span = std::max(c2.t - c1.t, 1e-6f);
        const float u = clampf((t - c1.t) / span, 0.0f, 1.0f);

        Station s;
        s.t         = t;
        s.x         = xNose - t * length;
        s.halfWidth = catmull(c0.halfWidth, c1.halfWidth, c2.halfWidth, c3.halfWidth, u) * halfWidth;
        s.bottom    = catmull(c0.bottom,    c1.bottom,    c2.bottom,    c3.bottom,    u) * height;
        s.shoulder  = catmull(c0.shoulder,  c1.shoulder,  c2.shoulder,  c3.shoulder,  u) * height;
        s.top       = catmull(c0.top,       c1.top,       c2.top,       c3.top,       u) * height;
        s.topPower  = catmull(c0.topPower,  c1.topPower,  c2.topPower,  c3.topPower,  u);
        s.botPower  = catmull(c0.botPower,  c1.botPower,  c2.botPower,  c3.botPower,  u);

        // The spline can overshoot at a sharp control step; a section with a
        // negative height or width would turn inside out.
        s.halfWidth = std::max(s.halfWidth, 0.02f);
        s.bottom    = std::max(s.bottom, 0.01f);
        s.shoulder  = clampf(s.shoulder, s.bottom + 0.01f, 1e3f);
        s.top       = std::max(s.top, s.shoulder + 0.01f);
        out.push_back(s);
    }
    return out;
}

//===========================================================================
//  Emitting a loft
//===========================================================================

using ColorFn = std::function<glm::vec3(const Loft&, int, int,
                                        const glm::vec3&, const glm::vec3&)>;

/// Quad with a per-vertex normal, wound to face the way those normals point.
void pushQuad(MeshData& m, const Vertex& a, const Vertex& b,
              const Vertex& c, const Vertex& d) {
    const glm::vec3 geom = glm::cross(b.position - a.position, c.position - a.position);
    if (glm::length(geom) < 1e-12f) return;

    const glm::vec3 avg = a.normal + b.normal + c.normal + d.normal;
    const bool flip = (glm::dot(geom, avg) < 0.0f);

    const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
    if (flip) {
        m.vertices.push_back(d);
        m.vertices.push_back(c);
        m.vertices.push_back(b);
        m.vertices.push_back(a);
    } else {
        m.vertices.push_back(a);
        m.vertices.push_back(b);
        m.vertices.push_back(c);
        m.vertices.push_back(d);
    }
    m.indices.insert(m.indices.end(),
                     {base, base + 1, base + 2, base, base + 2, base + 3});
}

void emitLoft(MeshData& m, const Loft& L, const ColorFn& colorOf) {
    const int kMax = L.closed ? L.ring : L.ring - 1;

    for (int i = 0; i + 1 < L.sections(); ++i) {
        for (int k = 0; k < kMax; ++k) {
            const int k2 = (k + 1) % L.ring;

            auto vertex = [&](int si, int sk) {
                Vertex v;
                v.position = L.P(si, sk);
                v.normal   = L.N(si, sk);
                v.color    = colorOf(L, si, sk, v.position, v.normal);
                return v;
            };
            pushQuad(m, vertex(i, k), vertex(i, k2), vertex(i + 1, k2), vertex(i + 1, k));
        }
    }
}

/// Close a closed-ring loft at both ends with a fan to the section centre.
void emitCaps(MeshData& m, const Loft& L, const glm::vec3& color) {
    if (!L.closed || L.sections() < 2) return;

    const int lastSection = L.sections() - 1;
    for (int end = 0; end < 2; ++end) {
        const int i = (end == 0) ? 0 : lastSection;
        const glm::vec3 outward = (end == 0) ? glm::vec3(1, 0, 0) : glm::vec3(-1, 0, 0);

        glm::vec3 centre(0.0f);
        for (int k = 0; k < L.ring; ++k) centre += L.P(i, k);
        centre /= static_cast<float>(L.ring);

        for (int k = 0; k < L.ring; ++k) {
            const int k2 = (k + 1) % L.ring;
            meshfactory::addTriangleOriented(m, centre, L.P(i, k), L.P(i, k2),
                                             outward, color);
        }
    }
}

/// Lift a rectangular region of the loft grid off the surface as its own
/// coloured skin: glass, light lenses, vents, livery panels. The surrounding
/// body is untouched, so the margin around a patch is what becomes the
/// A-pillar, the light surround, the panel gap.
void emitPatch(MeshData& m, const Loft& L, int i0, int i1, int k0, int k1,
               float offset, const glm::vec3& color) {
    i0 = std::max(i0, 0);
    i1 = std::min(i1, L.sections() - 1);
    if (i1 <= i0) return;

    const int kSpan = ((k1 - k0) % L.ring + L.ring) % L.ring;
    if (kSpan == 0) return;

    for (int i = i0; i < i1; ++i) {
        for (int kk = 0; kk < kSpan; ++kk) {
            const int ka = (k0 + kk) % L.ring;
            const int kb = (k0 + kk + 1) % L.ring;

            auto vertex = [&](int si, int sk) {
                Vertex v;
                v.normal   = L.N(si, sk);
                v.position = L.P(si, sk) + v.normal * offset;
                v.color    = color;
                return v;
            };
            pushQuad(m, vertex(i, ka), vertex(i, kb), vertex(i + 1, kb), vertex(i + 1, ka));
        }
    }
}

//===========================================================================
//  Small solids used by the add-on kit
//===========================================================================

/// Box given its eight corners as (two longitudinal faces) x (a rectangle in
/// Y-Z). A wing mount, a kerb, a spoke and a splitter are all this shape with
/// different numbers.
void addBoxCorners(MeshData& m, float x0, float x1,
                   float y0, float y1, float z0, float z1,
                   const glm::vec3& color) {
    const glm::vec3 c[8] = {
        {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0},
        {x1, y0, z0}, {x1, y0, z1}, {x1, y1, z1}, {x1, y1, z0},
    };
    meshfactory::addHexahedron(m, c, color);
}

/// Extrude a closed 2D polygon in the X-Y plane along Z. Wings and diffuser
/// strakes are aerofoil and fin sections, which no box can approximate.
void extrudeAlongZ(MeshData& m, const std::vector<glm::vec2>& poly,
                   float z0, float z1, const glm::vec3& color, bool caps) {
    if (poly.size() < 3) return;

    glm::vec2 centre(0.0f);
    for (const glm::vec2& p : poly) centre += p;
    centre /= static_cast<float>(poly.size());

    for (std::size_t i = 0; i < poly.size(); ++i) {
        const glm::vec2& a = poly[i];
        const glm::vec2& b = poly[(i + 1) % poly.size()];
        const glm::vec2 mid = 0.5f * (a + b) - centre;

        meshfactory::addQuadOriented(m,
                                     {a.x, a.y, z0}, {b.x, b.y, z0},
                                     {b.x, b.y, z1}, {a.x, a.y, z1},
                                     {mid.x, mid.y, 0.0f}, color);
    }

    if (caps) {
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const glm::vec2& a = poly[i];
            const glm::vec2& b = poly[(i + 1) % poly.size()];
            meshfactory::addTriangleOriented(m, {centre.x, centre.y, z0},
                                             {a.x, a.y, z0}, {b.x, b.y, z0},
                                             {0, 0, -1}, color);
            meshfactory::addTriangleOriented(m, {centre.x, centre.y, z1},
                                             {a.x, a.y, z1}, {b.x, b.y, z1},
                                             {0, 0, 1}, color);
        }
    }
}

/// Inverted aerofoil section, trailing edge raised by `aoaDeg`. Thickness is
/// the NACA 4-digit half-thickness distribution; the camber is a plain
/// parabola. Not an aerodynamic model - this project's downforce is zero (see
/// gt3_redbull.yaml) - just the right silhouette for a rear wing.
std::vector<glm::vec2> aerofoil(float xLeading, float yBase, float chord,
                                float thickness, float camber, float aoaDeg) {
    std::vector<glm::vec2> upper, lower;
    constexpr int kPoints = 14;
    const float tanA = std::tan(aoaDeg * kPiF / 180.0f);

    for (int i = 0; i <= kPoints; ++i) {
        const float s = static_cast<float>(i) / static_cast<float>(kPoints);
        const float yt = 5.0f * thickness *
                         (0.2969f * std::sqrt(s) - 0.1260f * s - 0.3516f * s * s +
                          0.2843f * s * s * s - 0.1015f * s * s * s * s);
        const float yc = -camber * 4.0f * s * (1.0f - s);
        const float rake = s * chord * tanA;

        const float x = xLeading - s * chord;
        upper.push_back({x, yBase + (yc + yt) * chord + rake});
        lower.push_back({x, yBase + (yc - yt) * chord + rake});
    }

    std::vector<glm::vec2> poly = upper;
    for (auto it = lower.rbegin(); it != lower.rend(); ++it) poly.push_back(*it);
    return poly;
}

//===========================================================================
//  Style tables
//
//  Heights are fractions of the configured body height, widths fractions of
//  the half body width, t runs 0 (nose tip) to 1 (tail). Every number is a
//  proportion, never a metre, so the same table describes a 4.3 m sports car
//  and a 4.9 m one.
//===========================================================================

struct GhControl { float t, halfWidth, bottom, top; };

struct StyleSpec {
    std::vector<Control>   body;
    std::vector<GhControl> greenhouse;
    float ghTopPower = 3.4f;

    /// Angles, in degrees around the section, at which the body surface
    /// BREAKS instead of curving - see Loft. These are the hard lines of the
    /// car: the shoulder down the flank, the turn-under at the sill, the edge
    /// where the bonnet and boot meet the wings.
    std::vector<float> creases;

    // Normalised longitudinal ranges of the window groups. The B-pillar is
    // the gap left between the two side windows; without one the whole flank
    // of the greenhouse becomes a single black band and the car loses its
    // roof structure entirely.
    float windscreen[2] {0.0f, 0.0f};
    float sideGlass[2]  {0.0f, 0.0f};
    float bPillar[2]    {0.0f, 0.0f};
    float rearGlass[2]  {0.0f, 0.0f};

    /// Angular limits of the side glass, and of the screens. Kept well inside
    /// the section so the surround, the roof rail and the pillars survive.
    float sideGlassDeg[2]  {8.0f, 44.0f};
    float screenDeg[2]     {52.0f, 128.0f};

    float fenderBulge = 0.05f;   ///< extra half width over an axle, fraction
    float archHeight  = 1.06f;   ///< arch crown, x wheel radius
    float archWidth   = 1.30f;   ///< arch opening half length, x wheel radius

    bool centreStripe = false;
    bool splitter     = false;
    bool divePlanes   = false;
    bool sideSkirt    = false;
    bool rearWing     = false;
    bool duckTail     = false;
    bool diffuser     = false;
    bool roofScoop    = false;
    bool roofRails    = false;
    bool bonnetVents  = false;
    bool doorPanel    = false;   ///< race number panel
    bool doorHandles  = false;
    bool cladding     = false;   ///< SUV lower-body plastic
    bool sideExhaust  = false;
    bool sideStrake   = false;   ///< intake vent on the flank ahead of the rear arch
    bool centreExhaust= false;   ///< pipes clustered on the centreline, not paired either side
    bool quadRoundTail= false;   ///< two smaller round lenses per side, not one wide band

    float mirrorT = 0.36f;
};

StyleSpec raceSpec() {
    StyleSpec s;
    s.body = {
        //  t     hw     bot    sh     top    tpow   bpow
        {0.000f, 0.46f, 0.050f, 0.12f, 0.26f, 3.4f, 3.4f},  // nose tip, blunt
        {0.030f, 0.78f, 0.038f, 0.15f, 0.33f, 4.0f, 4.6f},
        {0.090f, 0.93f, 0.038f, 0.18f, 0.42f, 4.6f, 5.4f},  // bumper
        {0.165f, 0.99f, 0.042f, 0.20f, 0.49f, 5.0f, 6.0f},  // front arch
        {0.245f, 0.93f, 0.042f, 0.22f, 0.45f, 5.4f, 6.4f},  // scuttle, dips
        {0.330f, 0.95f, 0.042f, 0.24f, 0.50f, 5.6f, 6.6f},  // cowl
        {0.500f, 1.00f, 0.042f, 0.26f, 0.54f, 6.0f, 7.0f},  // doors, flat flank
        {0.690f, 1.00f, 0.042f, 0.26f, 0.55f, 6.0f, 7.0f},
        {0.800f, 0.99f, 0.046f, 0.26f, 0.57f, 5.4f, 6.2f},  // rear arch
        {0.905f, 0.96f, 0.050f, 0.26f, 0.57f, 4.8f, 5.6f},  // rear deck
        {0.975f, 0.90f, 0.062f, 0.26f, 0.54f, 4.2f, 4.6f},
        {1.000f, 0.74f, 0.090f, 0.26f, 0.46f, 3.8f, 3.8f},  // tail, cut off
    };
    s.greenhouse = {
        {0.300f, 0.52f, 0.46f, 0.48f},
        {0.375f, 0.68f, 0.46f, 0.73f},
        {0.445f, 0.76f, 0.46f, 0.89f},
        {0.515f, 0.80f, 0.46f, 0.98f},   // roof front
        {0.640f, 0.80f, 0.46f, 1.00f},
        {0.735f, 0.78f, 0.46f, 0.97f},   // roof rear
        {0.835f, 0.72f, 0.46f, 0.76f},   // backlight
        {0.905f, 0.62f, 0.46f, 0.55f},
        {0.945f, 0.52f, 0.46f, 0.48f},
    };
    s.ghTopPower = 4.4f;
    s.creases = {0.0f, 180.0f, 58.0f, 122.0f, 214.0f, 326.0f};
    s.windscreen[0] = 0.325f; s.windscreen[1] = 0.515f;
    s.sideGlass[0]  = 0.525f; s.sideGlass[1]  = 0.815f;
    s.bPillar[0]    = 0.655f; s.bPillar[1]    = 0.700f;
    s.rearGlass[0]  = 0.745f; s.rearGlass[1]  = 0.900f;
    s.sideGlassDeg[0] = 7.0f;  s.sideGlassDeg[1] = 46.0f;
    s.screenDeg[0]    = 50.0f; s.screenDeg[1]    = 130.0f;

    s.fenderBulge = 0.085f;
    s.archHeight  = 1.16f;
    s.archWidth   = 1.38f;

    s.centreStripe = true;
    s.splitter = s.divePlanes = s.sideSkirt = true;
    s.rearWing = s.diffuser = s.roofScoop = true;
    s.bonnetVents = s.doorPanel = s.sideExhaust = true;
    s.mirrorT = 0.355f;
    return s;
}

/// Analog-era mid-engined homologation special: Ferrari F40 proportions -
/// see CarREf/ for the photo set this was measured against (a near-dead-on
/// side profile plus front, rear and 3/4 shots). Not traced vertex-for-
/// vertex - this is still a superellipse loft, not a scan - but every
/// silhouette cue that makes the reference photos instantly readable as
/// "that kind of car" is deliberately reproduced: the nose lower than the
/// tail, the power-dome dip in the bonnet between the front fenders, the
/// waist that tucks in at the doors between two flared arches, the low
/// cab-rearward greenhouse, and the details below (side strake, strut-
/// mounted wing, centre-exit exhaust, round quad tail lights).
StyleSpec sportsSpec() {
    StyleSpec s;
    s.body = {
        //  t       hw     bot    sh     top    tpow  bpow
        {0.000f, 0.44f, 0.020f, 0.14f, 0.280f, 3.2f, 3.2f},  // low, tapered nose
        {0.028f, 0.74f, 0.028f, 0.17f, 0.380f, 3.8f, 4.2f},  // splitter corners
        {0.075f, 0.90f, 0.030f, 0.19f, 0.430f, 4.4f, 5.0f},
        {0.150f, 0.97f, 0.034f, 0.21f, 0.465f, 5.0f, 5.6f},  // front arch / fender peak
        {0.230f, 0.90f, 0.034f, 0.20f, 0.415f, 5.0f, 5.6f},  // power-dome dip between fenders
        {0.320f, 0.90f, 0.034f, 0.20f, 0.435f, 5.2f, 5.8f},  // scuttle / screen base
        {0.470f, 0.88f, 0.034f, 0.19f, 0.445f, 5.4f, 6.0f},  // waist - doors tuck in
        {0.640f, 0.90f, 0.038f, 0.20f, 0.475f, 5.2f, 5.8f},
        {0.760f, 0.98f, 0.042f, 0.22f, 0.525f, 4.8f, 5.4f},  // rear arch, widest point
        {0.870f, 0.94f, 0.048f, 0.24f, 0.560f, 4.2f, 4.8f},  // engine deck, tallest body point
        {0.955f, 0.84f, 0.058f, 0.24f, 0.520f, 3.6f, 4.0f},
        {1.000f, 0.66f, 0.075f, 0.22f, 0.360f, 3.2f, 3.2f},  // tail / valance
    };
    // The greenhouse itself is small, low and set well back of a long nose -
    // a mid-engined car's cabin does not need to clear an engine ahead of
    // it, so it sits low and the bonnet line stays low with it. Narrower
    // than the body (a tucked-in glasshouse between two wide, flared
    // shoulders is one of the strongest "supercar" cues there is) and a
    // flatter cross-section (high ghTopPower) so the roof reads as a panel
    // with an edge, not a dome.
    s.greenhouse = {
        {0.310f, 0.44f, 0.39f, 0.455f},
        {0.365f, 0.58f, 0.39f, 0.680f},
        {0.420f, 0.64f, 0.39f, 0.840f},
        {0.480f, 0.66f, 0.39f, 0.920f},   // roof front
        {0.560f, 0.66f, 0.39f, 0.935f},
        {0.630f, 0.63f, 0.39f, 0.880f},   // fastback break - short roof
        {0.720f, 0.56f, 0.39f, 0.660f},
        {0.790f, 0.48f, 0.39f, 0.500f},
        {0.820f, 0.42f, 0.39f, 0.455f},
    };
    s.ghTopPower = 4.6f;
    s.creases = {0.0f, 180.0f, 54.0f, 126.0f, 214.0f, 326.0f};
    s.windscreen[0] = 0.335f; s.windscreen[1] = 0.478f;
    s.sideGlass[0]  = 0.488f; s.sideGlass[1]  = 0.700f;
    s.bPillar[0]    = 0.578f; s.bPillar[1]    = 0.606f;   // thin - one long door glass, not two
    s.rearGlass[0]  = 0.700f; s.rearGlass[1]  = 0.785f;   // small engine-cover window, not a screen
    s.sideGlassDeg[0] = 8.0f;  s.sideGlassDeg[1] = 44.0f;
    s.screenDeg[0]    = 56.0f; s.screenDeg[1]    = 124.0f;  // steeply raked screen

    // Wide, bonded-on-looking flares front and rear rather than a subtle
    // bulge - this is closer to the race style's own numbers than to a
    // touring coupe's, which is accurate: the arches are most of why the
    // reference photos read as a homologation special rather than a GT.
    s.fenderBulge = 0.085f;
    s.archHeight  = 1.05f;
    s.archWidth   = 1.20f;

    s.splitter = s.sideSkirt = s.rearWing = s.bonnetVents = true;
    s.sideStrake = s.centreExhaust = s.quadRoundTail = true;
    s.doorHandles = false;   // flush pull-strap, not a raised handle
    s.mirrorT = 0.38f;
    return s;
}

StyleSpec saloonSpec() {
    StyleSpec s;
    s.body = {
        {0.000f, 0.50f, 0.135f, 0.22f, 0.34f, 3.6f, 3.6f},  // blunt bumper face
        {0.032f, 0.76f, 0.112f, 0.25f, 0.43f, 4.2f, 4.6f},
        {0.090f, 0.90f, 0.100f, 0.27f, 0.50f, 4.8f, 5.4f},  // bumper top
        {0.165f, 0.99f, 0.100f, 0.29f, 0.55f, 5.4f, 6.0f},  // front arch
        {0.262f, 0.95f, 0.100f, 0.30f, 0.545f,5.8f, 6.4f},  // bonnet
        {0.330f, 0.97f, 0.100f, 0.31f, 0.58f, 6.0f, 6.8f},  // cowl
        {0.520f, 1.00f, 0.096f, 0.32f, 0.59f, 6.2f, 7.0f},  // doors
        {0.700f, 1.00f, 0.096f, 0.32f, 0.59f, 6.2f, 7.0f},
        {0.802f, 0.99f, 0.100f, 0.31f, 0.60f, 5.6f, 6.2f},  // rear arch
        {0.908f, 0.96f, 0.104f, 0.30f, 0.595f,4.8f, 5.4f},  // boot
        {0.978f, 0.90f, 0.118f, 0.29f, 0.56f, 4.0f, 4.4f},
        {1.000f, 0.74f, 0.155f, 0.29f, 0.49f, 3.6f, 3.6f},
    };
    s.greenhouse = {
        {0.305f, 0.58f, 0.53f, 0.55f},
        {0.388f, 0.72f, 0.53f, 0.79f},
        {0.462f, 0.79f, 0.53f, 0.94f},
        {0.542f, 0.83f, 0.53f, 1.00f},   // roof front
        {0.700f, 0.83f, 0.53f, 1.00f},   // roof rear
        {0.792f, 0.80f, 0.53f, 0.89f},
        {0.872f, 0.72f, 0.53f, 0.66f},   // backlight base
        {0.918f, 0.62f, 0.53f, 0.55f},
        {0.942f, 0.54f, 0.53f, 0.55f},
    };
    s.ghTopPower = 4.0f;
    s.creases = {0.0f, 180.0f, 54.0f, 126.0f, 210.0f, 330.0f};
    s.windscreen[0] = 0.338f; s.windscreen[1] = 0.542f;
    s.sideGlass[0]  = 0.552f; s.sideGlass[1]  = 0.862f;
    s.bPillar[0]    = 0.688f; s.bPillar[1]    = 0.726f;
    s.rearGlass[0]  = 0.712f; s.rearGlass[1]  = 0.916f;
    s.sideGlassDeg[0] = 8.0f;  s.sideGlassDeg[1] = 42.0f;
    s.screenDeg[0]    = 54.0f; s.screenDeg[1]    = 126.0f;

    s.fenderBulge = 0.035f;
    s.archHeight  = 1.12f;
    s.archWidth   = 1.32f;

    s.doorHandles = true;
    s.mirrorT = 0.385f;
    return s;
}

StyleSpec suvSpec() {
    StyleSpec s;
    s.body = {
        {0.000f, 0.54f, 0.135f, 0.24f, 0.38f, 4.0f, 4.0f},
        {0.032f, 0.80f, 0.115f, 0.27f, 0.47f, 4.8f, 5.0f},
        {0.088f, 0.92f, 0.105f, 0.29f, 0.53f, 5.4f, 5.8f},
        {0.162f, 0.99f, 0.105f, 0.30f, 0.575f,6.0f, 6.4f},  // front arch
        {0.248f, 0.96f, 0.105f, 0.31f, 0.585f,6.4f, 6.8f},
        {0.308f, 0.99f, 0.105f, 0.32f, 0.61f, 6.8f, 7.2f},  // cowl
        {0.520f, 1.00f, 0.100f, 0.33f, 0.62f, 7.0f, 7.4f},  // slab flank
        {0.722f, 1.00f, 0.100f, 0.33f, 0.62f, 7.0f, 7.4f},
        {0.822f, 0.99f, 0.105f, 0.32f, 0.62f, 6.4f, 6.8f},  // rear arch
        {0.928f, 0.97f, 0.110f, 0.31f, 0.61f, 5.6f, 5.8f},
        {0.986f, 0.92f, 0.122f, 0.30f, 0.575f,4.6f, 4.8f},
        {1.000f, 0.78f, 0.155f, 0.30f, 0.51f, 4.0f, 4.0f},
    };
    s.greenhouse = {
        {0.285f, 0.62f, 0.55f, 0.57f},
        {0.358f, 0.76f, 0.55f, 0.81f},
        {0.428f, 0.84f, 0.55f, 0.96f},
        {0.498f, 0.87f, 0.55f, 1.00f},   // roof front
        {0.790f, 0.87f, 0.55f, 1.00f},   // roof rear, long and flat
        {0.882f, 0.84f, 0.55f, 0.93f},
        {0.946f, 0.76f, 0.55f, 0.69f},   // near-vertical tailgate glass
        {0.976f, 0.66f, 0.55f, 0.58f},
        {0.990f, 0.58f, 0.55f, 0.57f},
    };
    s.ghTopPower = 5.0f;
    s.creases = {0.0f, 180.0f, 50.0f, 130.0f, 206.0f, 334.0f};
    s.windscreen[0] = 0.318f; s.windscreen[1] = 0.498f;
    s.sideGlass[0]  = 0.508f; s.sideGlass[1]  = 0.928f;
    s.bPillar[0]    = 0.660f; s.bPillar[1]    = 0.700f;
    s.rearGlass[0]  = 0.800f; s.rearGlass[1]  = 0.978f;
    s.sideGlassDeg[0] = 8.0f;  s.sideGlassDeg[1] = 40.0f;
    s.screenDeg[0]    = 56.0f; s.screenDeg[1]    = 124.0f;

    s.fenderBulge = 0.045f;
    s.archHeight  = 1.20f;
    s.archWidth   = 1.36f;

    s.roofRails = s.cladding = s.doorHandles = true;
    s.mirrorT = 0.355f;
    return s;
}

StyleSpec specFor(CarBodyStyle style) {
    switch (style) {
        case CarBodyStyle::Race:   return raceSpec();
        case CarBodyStyle::Sports: return sportsSpec();
        case CarBodyStyle::Suv:    return suvSpec();
        case CarBodyStyle::Saloon:
        case CarBodyStyle::Auto:
        default:                   return saloonSpec();
    }
}

//===========================================================================
//  Wheels
//===========================================================================

/// One point of a lathe profile, in the plane that contains the wheel axis.
/// `r` is the distance from the axis, `z` the position along it (the wheel
/// spins about local +Z - see Renderer::drawVehicle()).
struct LathePoint {
    float r;
    float z;
    glm::vec3 color;
};

/// Revolve a profile about the Z axis. Normals come from the profile tangent
/// rather than the faces, so a tyre shoulder rolls off smoothly instead of
/// faceting.
void revolveZ(MeshData& m, const std::vector<LathePoint>& profile, int segments) {
    if (profile.size() < 2 || segments < 3) return;

    // 2D outward normal per profile point, averaged across the two adjacent
    // segments. Walking the profile from the inboard bead, around the tread,
    // to the outboard bead makes (dz, -dr) point away from the solid.
    std::vector<glm::vec2> n2(profile.size(), glm::vec2(0.0f));
    for (std::size_t i = 0; i < profile.size(); ++i) {
        glm::vec2 acc(0.0f);
        if (i > 0) {
            const glm::vec2 d{profile[i].r - profile[i - 1].r, profile[i].z - profile[i - 1].z};
            acc += glm::vec2(d.y, -d.x);
        }
        if (i + 1 < profile.size()) {
            const glm::vec2 d{profile[i + 1].r - profile[i].r, profile[i + 1].z - profile[i].z};
            acc += glm::vec2(d.y, -d.x);
        }
        n2[i] = (glm::length(acc) > 1e-9f) ? glm::normalize(acc) : glm::vec2(1.0f, 0.0f);
    }

    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        for (int k = 0; k < segments; ++k) {
            const float a0 = kTwoPiF * static_cast<float>(k)     / static_cast<float>(segments);
            const float a1 = kTwoPiF * static_cast<float>(k + 1) / static_cast<float>(segments);

            auto vertex = [&](std::size_t pi, float a) {
                Vertex v;
                v.position = {profile[pi].r * std::cos(a), profile[pi].r * std::sin(a),
                              profile[pi].z};
                v.normal   = glm::normalize(glm::vec3(n2[pi].x * std::cos(a),
                                                      n2[pi].x * std::sin(a), n2[pi].y));
                v.color    = profile[pi].color;
                return v;
            };
            pushQuad(m, vertex(i, a0), vertex(i, a1), vertex(i + 1, a1), vertex(i + 1, a0));
        }
    }
}

/// Mirror a mesh through the X-Y plane, flipping winding so the result is
/// still front-facing. The left and right wheels are mirror images: "outboard"
/// is +Z on the right of the car and -Z on the left.
MeshData mirroredZ(const MeshData& src) {
    MeshData m = src;
    for (Vertex& v : m.vertices) {
        v.position.z = -v.position.z;
        v.normal.z   = -v.normal.z;
    }
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        std::swap(m.indices[i], m.indices[i + 2]);
    }
    return m;
}

/// A block bounded by two radii, two angles about the wheel axis and two
/// positions along it. Brake calipers and rim spokes are both this.
void addWheelBlock(MeshData& m, float r0, float r1, float a0Deg, float a1Deg,
                   float z0, float z1, const glm::vec3& color) {
    const float a0 = a0Deg * kPiF / 180.0f;
    const float a1 = a1Deg * kPiF / 180.0f;

    auto pt = [](float r, float a, float z) {
        return glm::vec3(r * std::cos(a), r * std::sin(a), z);
    };
    const glm::vec3 c[8] = {
        pt(r0, a0, z0), pt(r1, a0, z0), pt(r1, a1, z0), pt(r0, a1, z0),
        pt(r0, a0, z1), pt(r1, a0, z1), pt(r1, a1, z1), pt(r0, a1, z1),
    };
    meshfactory::addHexahedron(m, c, color);
}

/// Right-hand-side wheel: tyre carcass, rim barrel, spokes, hub and centre
/// nut. Drawn once and reused for both wheels on that side; the renderer
/// supplies the spin.
MeshData buildWheel(float radius, float width, CarBodyStyle style,
                    const CarPalette& pal) {
    MeshData m;

    const float hw = width * 0.5f;
    // A bigger rim relative to the tyre - a lower-profile tyre - is a real,
    // legible design signal for "sports car" (and more so for "race car"),
    // not a stylistic flourish: it is also most of why a wheel reads clearly
    // from a few car-lengths away instead of as a plain black disc, which a
    // saloon or SUV's smaller, higher-sidewall wheel does not need to do.
    const float rimR = radius * (style == CarBodyStyle::Race    ? 0.68f
                                : style == CarBodyStyle::Sports ? 0.665f
                                                                : 0.64f);

    const glm::vec3 tyre  = pal.tire;
    const glm::vec3 tread = pal.tire * 1.22f;
    const glm::vec3 wall  = pal.tire * 0.88f;
    const glm::vec3 letters = pal.tire * 2.1f;

    // Tyre cross-section, inboard bead -> sidewall -> shoulder -> tread ->
    // back again. The three shallow dips across the tread are circumferential
    // grooves; on a slick-shod race car they are left out.
    const bool slick = (style == CarBodyStyle::Race);
    std::vector<LathePoint> t;
    t.push_back({rimR,           -hw * 0.94f, wall});
    t.push_back({radius * 0.78f, -hw * 1.00f, wall});
    t.push_back({radius * 0.88f, -hw * 1.02f, letters});
    t.push_back({radius * 0.95f, -hw * 0.96f, wall});
    t.push_back({radius * 0.995f,-hw * 0.84f, tread});
    if (!slick) {
        t.push_back({radius,         -hw * 0.62f, tread});
        t.push_back({radius * 0.965f,-hw * 0.54f, tyre});
        t.push_back({radius,         -hw * 0.46f, tread});
        t.push_back({radius,         -hw * 0.12f, tread});
        t.push_back({radius * 0.965f,-hw * 0.04f, tyre});
        t.push_back({radius,          hw * 0.04f, tread});
        t.push_back({radius,          hw * 0.46f, tread});
        t.push_back({radius * 0.965f, hw * 0.54f, tyre});
        t.push_back({radius,          hw * 0.62f, tread});
    } else {
        t.push_back({radius,        -hw * 0.60f, tread});
        t.push_back({radius,         hw * 0.60f, tread});
    }
    t.push_back({radius * 0.995f, hw * 0.84f, tread});
    t.push_back({radius * 0.95f,  hw * 0.96f, wall});
    t.push_back({radius * 0.88f,  hw * 1.02f, letters});
    t.push_back({radius * 0.78f,  hw * 1.00f, wall});
    t.push_back({rimR,            hw * 0.94f, wall});
    revolveZ(m, t, 36);

    // Rim barrel: a short lathe from the inboard bead seat to the outboard
    // lip, so the gap between the spokes shows a rim rather than the sky.
    const glm::vec3 rim  = pal.rim;
    const glm::vec3 lip  = pal.rim * 1.25f;
    const std::vector<LathePoint> barrel = {
        {rimR * 0.93f, -hw * 0.92f, rim * 0.7f},
        {rimR * 0.93f,  hw * 0.55f, rim * 0.8f},
        {rimR,          hw * 0.80f, rim},
        {rimR,          hw * 0.96f, lip},
        {rimR * 0.92f,  hw * 0.99f, lip},
    };
    revolveZ(m, barrel, 36);

    // Spokes on the outboard face. These are what make wheel rotation legible
    // on screen; the box wheel they replace needed a painted quadrant to do
    // the same job.
    const int spokes = (style == CarBodyStyle::Race) ? 10
                     : (style == CarBodyStyle::Suv || style == CarBodyStyle::Sports) ? 5 : 6;
    const float hubR = radius * 0.19f;
    for (int i = 0; i < spokes; ++i) {
        const float a = 360.0f * static_cast<float>(i) / static_cast<float>(spokes);
        const float halfAngleInner = 180.0f / static_cast<float>(spokes) * 0.62f;
        const float halfAngleOuter = 180.0f / static_cast<float>(spokes) * 0.34f;

        // Two blocks make one tapered spoke: wide at the hub, narrow at the rim.
        addWheelBlock(m, hubR * 0.9f, rimR * 0.55f, a - halfAngleInner, a + halfAngleInner,
                      hw * 0.52f, hw * 0.86f, rim);
        addWheelBlock(m, rimR * 0.54f, rimR * 0.96f, a - halfAngleOuter, a + halfAngleOuter,
                      hw * 0.56f, hw * 0.88f, rim * 1.08f);
    }

    // Hub and centre lock.
    m.append(meshfactory::cylinderAlong({0.0f, 0.0f, hw * 0.35f}, {0.0f, 0.0f, hw * 0.58f},
                                        hubR, 20, rim * 0.85f));
    m.append(meshfactory::cylinderAlong({0.0f, 0.0f, hw * 0.88f}, {0.0f, 0.0f, hw * 0.22f},
                                        radius * 0.085f, 6,
                                        style == CarBodyStyle::Race ? pal.accent : rim * 1.1f));
    return m;
}

/// Right-hand brake: disc, bell and caliper. Separate from the wheel because
/// it must NOT spin with it - a rotating caliper is the single most obvious
/// tell of a faked wheel.
MeshData buildBrake(float radius, float width, const CarPalette& pal) {
    MeshData m;
    const float hw = width * 0.5f;
    const float discR = radius * 0.58f;

    const glm::vec3 face = pal.brakeDisc;
    const std::vector<LathePoint> disc = {
        {radius * 0.20f, hw * 0.02f, face * 0.8f},   // bell
        {discR * 0.55f,  hw * 0.02f, face * 0.8f},
        {discR * 0.55f,  hw * 0.16f, face * 0.9f},
        {discR,          hw * 0.16f, face * 1.15f},  // outboard friction face
        {discR,          hw * 0.02f, face},          // outer edge
        {discR,         -hw * 0.06f, face * 1.05f},  // inboard friction face
        {discR * 0.55f, -hw * 0.06f, face * 0.8f},
        {radius * 0.20f,-hw * 0.06f, face * 0.7f},
    };
    revolveZ(m, disc, 32);

    // Caliper, at the rear of the disc (vehicle local -X is rearward).
    addWheelBlock(m, discR * 0.86f, discR * 1.06f, 150.0f, 205.0f,
                  -hw * 0.20f, hw * 0.30f, pal.caliper);
    return m;
}

} // namespace

//===========================================================================
//  Style selection
//===========================================================================

CarBodyStyle carBodyStyleFromString(const std::string& name) {
    if (name == "race")   return CarBodyStyle::Race;
    if (name == "sports") return CarBodyStyle::Sports;
    if (name == "saloon" || name == "sedan") return CarBodyStyle::Saloon;
    if (name == "suv")    return CarBodyStyle::Suv;
    return CarBodyStyle::Auto;
}

const char* carBodyStyleName(CarBodyStyle style) {
    switch (style) {
        case CarBodyStyle::Race:   return "race";
        case CarBodyStyle::Sports: return "sports";
        case CarBodyStyle::Saloon: return "saloon";
        case CarBodyStyle::Suv:    return "suv";
        case CarBodyStyle::Auto:   break;
    }
    return "auto";
}

CarBodyStyle resolveCarBodyStyle(CarBodyStyle requested, const VehicleParameters& p) {
    if (requested != CarBodyStyle::Auto) return requested;

    const double height = std::max(p.height, 0.5);
    const double length = std::max(p.length, 1.0);
    const double ratio  = height / length;         // how tall the car stands
    const double widthRatio = p.width / length;    // how wide it is for its length

    if (ratio > 0.34) return CarBodyStyle::Suv;
    // A GT3 car is not merely low, it is low AND wide: the AMG GT3 is 2.05 m
    // across 4.65 m of length (0.44) where the sports car is 1.9/4.3 (0.44)
    // but 1.2 m tall against 1.26 m. Height alone separates them from a
    // saloon; the ground clearance the CG height implies separates race from
    // road, and it is the one parameter a race car cannot fake.
    if (ratio < 0.29 && widthRatio > 0.42 && p.cgHeight < 0.34) return CarBodyStyle::Race;
    if (ratio < 0.30) return CarBodyStyle::Sports;
    return CarBodyStyle::Saloon;
}

//===========================================================================
//  Assembly
//===========================================================================

CarMeshSet buildCarMeshes(const VehicleParameters& p, CarBodyStyle style,
                          const CarPalette& pal) {
    CarMeshSet out;

    const CarBodyStyle resolved = resolveCarBodyStyle(style, p);
    const StyleSpec spec = specFor(resolved);

    const float length    = static_cast<float>(p.length);
    const float halfWidth = static_cast<float>(p.width * 0.5);
    const float height    = static_cast<float>(p.height);
    const float wheelbase = static_cast<float>(p.wheelbase);
    const float wheelR    = static_cast<float>(p.wheelRadius);

    // Overhangs. The axles are fixed by the wheelbase; the body has to
    // distribute whatever length is left over around them, slightly more at
    // the front, which is what every car in the config directory does.
    const float overhang = std::max(length - wheelbase, 0.2f);
    const float frontOverhang = overhang * 0.54f;
    const float xFrontAxle = wheelbase;
    const float xNose = xFrontAxle + frontOverhang;

    //-----------------------------------------------------------------------
    //  Body shell
    //-----------------------------------------------------------------------
    std::vector<Station> stations =
        resample(spec.body, kBodySections, length, halfWidth, height, xNose);

    // Wheel arches and fender bulges, cut by the REAL axle positions rather
    // than authored into the table. This is the whole reason the model is
    // generated: change `wheelbase` in a YAML file and the arches follow.
    const float archR = wheelR * spec.archWidth;
    const float archY = wheelR * spec.archHeight;
    for (Station& s : stations) {
        for (int axle = 0; axle < 2; ++axle) {
            const float axleX = (axle == 0) ? xFrontAxle : 0.0f;
            const float d = std::abs(s.x - axleX);
            if (d >= archR) continue;

            const float f = std::sqrt(std::max(1.0f - (d / archR) * (d / archR), 0.0f));
            s.bottom    = std::max(s.bottom, archY * f);
            s.halfWidth *= 1.0f + spec.fenderBulge * f;
        }
        // The arch crown must stay below the widest point or the section
        // inverts (the "lower half" would have negative height).
        s.bottom   = std::min(s.bottom, s.shoulder - 0.02f);
        s.shoulder = std::min(s.shoulder, s.top - 0.02f);
    }

    const Loft body = buildLoft(stations, kRingSegments, 0.0f, kTwoPiF, true, spec.creases);

    const float sillBand = height * 0.18f;
    const ColorFn bodyColor = [&](const Loft& L, int i, int, const glm::vec3& pt,
                                  const glm::vec3& n) {
        // Underside and arch liners: never painted.
        if (n.y < -0.35f) return pal.trim * 0.8f;

        glm::vec3 c = pal.body;

        // Darken toward the sill. Real cars are shaded there by the ground;
        // with a single directional light and no shadows, painting it in is
        // what keeps the flank from looking like a flat slab.
        const float sillTop = L.st[static_cast<std::size_t>(i)].bottom + sillBand;
        if (pt.y < sillTop) {
            const float f = clampf((sillTop - pt.y) / sillBand, 0.0f, 1.0f);
            c = glm::mix(c, pal.trim, 0.55f * f);
        }
        if (spec.cladding && pt.y < sillTop * 1.35f) {
            c = glm::mix(c, pal.trim * 1.4f, 0.75f);
        }
        // Centre stripe over the nose, roof line and deck.
        if (spec.centreStripe && std::abs(pt.z) < halfWidth * 0.16f && n.y > 0.45f) {
            c = pal.accent;
        }
        return c;
    };

    emitLoft(out.body, body, bodyColor);
    emitCaps(out.body, body, pal.trim * 0.8f);

    //-----------------------------------------------------------------------
    //  Greenhouse
    //-----------------------------------------------------------------------
    std::vector<Control> ghControls;
    ghControls.reserve(spec.greenhouse.size());
    for (const GhControl& g : spec.greenhouse) {
        // The greenhouse base is pushed below the body's roof line at the same
        // station so the two solids overlap instead of leaving a seam. The
        // shoulder sits on the base, which collapses the lower half of the
        // section - only the upper half is lofted.
        const float bodyTop = body.st[static_cast<std::size_t>(body.sectionAt(g.t))].top / height;
        const float base = std::min(g.bottom, bodyTop - 0.05f);
        ghControls.push_back({g.t, g.halfWidth, base, base, std::max(g.top, base + 0.005f),
                              spec.ghTopPower, spec.ghTopPower});
    }
    std::vector<Station> ghStations =
        resample(ghControls, kGhSections, length, halfWidth, height, xNose);

    // resample() guarantees a minimum section height so the skin never
    // inverts; the greenhouse ends are meant to be flat, so re-flatten them.
    if (!ghStations.empty()) {
        ghStations.front().top = ghStations.front().shoulder + 0.004f;
        ghStations.back().top  = ghStations.back().shoulder + 0.004f;
    }

    const Loft gh = buildLoft(ghStations, kGhSegments, 0.0f, kPiF, false);

    const ColorFn cabinColor = [&](const Loft&, int, int, const glm::vec3& pt,
                                   const glm::vec3& n) {
        glm::vec3 c = pal.body;
        if (spec.centreStripe && std::abs(pt.z) < halfWidth * 0.16f && n.y > 0.45f) {
            c = pal.accent;
        }
        return c;
    };
    emitLoft(out.body, gh, cabinColor);

    // Windows: patches lifted off the cabin skin. What is NOT covered by a
    // patch stays body-coloured, and that remainder is exactly the A/B/C
    // pillars, the roof rail and the roof itself - so the angular limits
    // below are structure, not decoration. Covering the full section (which
    // is what the first version did) turns the entire greenhouse into one
    // black mass with no visible roof.
    const float glassLift = 0.012f;
    auto glassPatch = [&](float t0, float t1, float deg0, float deg1) {
        emitPatch(out.glass, gh, gh.sectionAt(t0), gh.sectionAt(t1),
                  gh.ringAt(deg0), gh.ringAt(deg1), glassLift, pal.glass);
    };
    const float sd0 = spec.sideGlassDeg[0];
    const float sd1 = spec.sideGlassDeg[1];

    glassPatch(spec.windscreen[0], spec.windscreen[1], spec.screenDeg[0], spec.screenDeg[1]);
    glassPatch(spec.rearGlass[0],  spec.rearGlass[1],  spec.screenDeg[0], spec.screenDeg[1]);

    // Side glass in two panes per side, split by the B-pillar gap.
    glassPatch(spec.sideGlass[0], spec.bPillar[0], sd0, sd1);
    glassPatch(spec.bPillar[1], spec.sideGlass[1], sd0, sd1);
    glassPatch(spec.sideGlass[0], spec.bPillar[0], 180.0f - sd1, 180.0f - sd0);
    glassPatch(spec.bPillar[1], spec.sideGlass[1], 180.0f - sd1, 180.0f - sd0);

    // Black surround below the side glass: the strip of body between the
    // window line and the door top, which on a real car is a shut line and a
    // seal rather than paint.
    emitPatch(out.body, gh, gh.sectionAt(spec.sideGlass[0]), gh.sectionAt(spec.sideGlass[1]),
              gh.ringAt(sd0 - 4.0f), gh.ringAt(sd0), 0.006f, pal.trim * 1.4f);
    emitPatch(out.body, gh, gh.sectionAt(spec.sideGlass[0]), gh.sectionAt(spec.sideGlass[1]),
              gh.ringAt(180.0f - sd0), gh.ringAt(180.0f - sd0 + 4.0f), 0.006f, pal.trim * 1.4f);

    //-----------------------------------------------------------------------
    //  Lights, vents and livery - the same patch mechanism on the body loft
    //-----------------------------------------------------------------------
    const float decal = 0.008f;
    auto bodyPatch = [&](float t0, float t1, float deg0, float deg1, const glm::vec3& c) {
        emitPatch(out.body, body, body.sectionAt(t0), body.sectionAt(t1),
                  body.ringAt(deg0), body.ringAt(deg1), decal, c);
    };

    bodyPatch(0.015f, 0.075f,  26.0f,  62.0f, pal.lightFront);   // headlight, right
    bodyPatch(0.015f, 0.075f, 118.0f, 154.0f, pal.lightFront);   // headlight, left

    if (spec.quadRoundTail) {
        // Two smaller round-ish lenses per side instead of one wide band -
        // the F40 (and plenty else from the era) reads as quad round tail
        // lights rather than a light bar, and the gap between the pair
        // becomes the lens bezel for free.
        bodyPatch(0.950f, 0.972f,  16.0f,  34.0f, pal.lightRear);
        bodyPatch(0.950f, 0.972f,  40.0f,  58.0f, pal.lightRear);
        bodyPatch(0.950f, 0.972f, 122.0f, 140.0f, pal.lightRear);
        bodyPatch(0.950f, 0.972f, 146.0f, 164.0f, pal.lightRear);
    } else {
        bodyPatch(0.955f, 0.995f,  14.0f,  56.0f, pal.lightRear);    // tail light, right
        bodyPatch(0.955f, 0.995f, 124.0f, 166.0f, pal.lightRear);    // tail light, left
    }

    bodyPatch(0.005f, 0.050f, 200.0f, 340.0f, pal.trim);         // front intake
    bodyPatch(0.940f, 1.000f, 195.0f, 345.0f, pal.trim);         // rear valance

    //-----------------------------------------------------------------------
    //  Panel shut lines.
    //
    //  One section of the grid is a few centimetres of body - the right width
    //  for a panel gap. These are what stop a lofted shell from reading as a
    //  single moulded lump: a car's surface is divided into a bonnet, four
    //  doors and a boot lid, and the eye looks for those divisions.
    //-----------------------------------------------------------------------
    {
        const float gap = 1.0f / static_cast<float>(kBodySections);
        const glm::vec3 shut = pal.trim * 1.15f;

        auto shutLine = [&](float t, float deg0, float deg1) {
            const int i0 = body.sectionAt(t);
            emitPatch(out.body, body, i0, i0 + 1, body.ringAt(deg0), body.ringAt(deg1),
                      0.004f, shut);
        };
        // Bonnet: across the top at the cowl, then down both wings.
        shutLine(spec.windscreen[0] - gap * 2.0f, 30.0f, 150.0f);
        // Boot lid / tailgate.
        shutLine(spec.rearGlass[1] + gap * 2.0f, 30.0f, 150.0f);
        // Door gaps, wrapping from the beltline down around the sill.
        const float doorFront = spec.windscreen[0] + 0.02f;
        const float doorMid   = 0.5f * (spec.bPillar[0] + spec.bPillar[1]);
        const float doorRear  = spec.sideGlass[1] + 0.01f;
        for (float t : {doorFront, doorMid, doorRear}) {
            shutLine(t, 318.0f, 42.0f);
            shutLine(t, 138.0f, 222.0f);
        }
    }

    //-----------------------------------------------------------------------
    //  Wheel arch liners.
    //
    //  The arch opening is cut into the loft by raising each section's floor
    //  over the axle (see above), which traces an ELLIPSE of semi-axes archR
    //  and archY in the flank. Without something behind it, that opening
    //  shows the far side of the body through the gap and the wheel looks
    //  like it is sitting in a painted pocket. This is the inner wing: the
    //  same ellipse, swept inboard, in shadow.
    //-----------------------------------------------------------------------
    for (int axle = 0; axle < 2; ++axle) {
        const float axleX = (axle == 0) ? xFrontAxle : 0.0f;
        const int   si    = body.sectionAt((xNose - axleX) / length);
        const float flank = body.st[static_cast<std::size_t>(si)].halfWidth;
        const float depth = std::min(flank * 0.42f, 0.34f);

        constexpr int kArchSteps = 18;
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            const float zOut = sgn * flank * 0.985f;
            const float zIn  = sgn * (flank - depth);

            for (int i = 0; i < kArchSteps; ++i) {
                const float p0 = kPiF * static_cast<float>(i)     / static_cast<float>(kArchSteps);
                const float p1 = kPiF * static_cast<float>(i + 1) / static_cast<float>(kArchSteps);

                auto arch = [&](float phi, float z) {
                    return glm::vec3(axleX + archR * std::cos(phi) * 0.99f,
                                     archY * std::sin(phi) * 0.985f, z);
                };
                // Facing down and inward, toward the wheel.
                const glm::vec3 inward{-std::cos(0.5f * (p0 + p1)),
                                       -std::sin(0.5f * (p0 + p1)), 0.0f};
                meshfactory::addQuadOriented(out.body, arch(p0, zOut), arch(p1, zOut),
                                             arch(p1, zIn), arch(p0, zIn),
                                             inward, pal.trim * 0.55f);
            }
        }
    }

    if (spec.bonnetVents) {
        bodyPatch(0.20f, 0.29f,  62.0f,  84.0f, pal.trim);
        bodyPatch(0.20f, 0.29f,  96.0f, 118.0f, pal.trim);
    }
    if (spec.sideStrake) {
        // Engine intake, not a decoration: sits ahead of the rear arch at
        // shoulder height, the single most recognisable styling cue on an
        // '80s mid-engined car's flank. Offset a few degrees off the dead
        // shoulder line (rather than straddling it) so the patch does not
        // have to wrap across the ringAt(0) seam.
        bodyPatch(0.640f, 0.740f,   4.0f,  26.0f, pal.trim);
        bodyPatch(0.640f, 0.740f, 154.0f, 176.0f, pal.trim);
    }
    if (spec.doorPanel) {
        bodyPatch(0.47f, 0.63f,  10.0f,  30.0f, pal.accent);
        bodyPatch(0.47f, 0.63f, 150.0f, 170.0f, pal.accent);
    }
    if (spec.doorHandles) {
        const float beltDeg = 16.0f;
        bodyPatch(0.47f, 0.51f, beltDeg, beltDeg + 12.0f, pal.trim * 2.4f);
        bodyPatch(0.47f, 0.51f, 180.0f - beltDeg - 12.0f, 180.0f - beltDeg, pal.trim * 2.4f);
        bodyPatch(0.70f, 0.74f, beltDeg, beltDeg + 12.0f, pal.trim * 2.4f);
        bodyPatch(0.70f, 0.74f, 180.0f - beltDeg - 12.0f, 180.0f - beltDeg, pal.trim * 2.4f);
    }

    //-----------------------------------------------------------------------
    //  Add-on kit
    //-----------------------------------------------------------------------
    const Station& noseSt = body.st.front();
    const Station& tailSt = body.st.back();
    const Station& midSt  = body.st[static_cast<std::size_t>(body.sectionAt(0.5f))];

    const float floorY = midSt.bottom;
    const float beltY  = midSt.top;
    const float roofY  = ghStations.empty() ? height
                                            : (*std::max_element(
                                                   ghStations.begin(), ghStations.end(),
                                                   [](const Station& a, const Station& b) {
                                                       return a.top < b.top;
                                                   })).top;

    if (spec.splitter) {
        // A flat plate under the nose, wider than the nose itself and
        // protruding ahead of it.
        const float y = noseSt.bottom * 0.55f;
        const float xf = xNose + 0.09f;
        const float xr = xNose - length * 0.13f;
        const float wF = halfWidth * 0.82f;
        const float wR = halfWidth * 1.00f;
        const glm::vec3 c[8] = {
            {xf, y,          -wF}, {xf, y,          wF},
            {xr, y,           wR}, {xr, y,         -wR},
            {xf, y + 0.022f, -wF}, {xf, y + 0.022f, wF},
            {xr, y + 0.022f,  wR}, {xr, y + 0.022f,-wR},
        };
        meshfactory::addHexahedron(out.body, c, pal.trim);
    }

    if (spec.divePlanes) {
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            const float z0 = sgn * halfWidth * 0.80f;
            const float z1 = sgn * halfWidth * 1.02f;
            const float x0 = xNose - length * 0.04f;
            const float x1 = xNose - length * 0.11f;
            const float y0 = height * 0.24f;
            const glm::vec3 c[8] = {
                {x0, y0,         z0}, {x0, y0,         z1},
                {x1, y0 + 0.05f, z1}, {x1, y0 + 0.05f, z0},
                {x0, y0 + 0.014f, z0}, {x0, y0 + 0.014f, z1},
                {x1, y0 + 0.064f, z1}, {x1, y0 + 0.064f, z0},
            };
            meshfactory::addHexahedron(out.body, c, pal.trim);
        }
    }

    if (spec.sideSkirt) {
        const float x0 = xFrontAxle - archR * 0.95f;
        const float x1 = archR * 0.95f;
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            addBoxCorners(out.body, x1, x0, floorY - 0.012f, floorY + 0.075f,
                          sgn * halfWidth * 0.95f, sgn * halfWidth * 1.035f, pal.trim);
        }
    }

    if (spec.diffuser) {
        // Underfloor kick-up plus vertical strakes, the one piece of aero that
        // is genuinely visible from behind a car on track.
        const float x0 = tailSt.x;
        const float x1 = tailSt.x + length * 0.20f;
        const float w  = halfWidth * 0.86f;
        const glm::vec3 c[8] = {
            {x1, floorY,          -w}, {x1, floorY,          w},
            {x0, floorY + 0.19f,   w}, {x0, floorY + 0.19f, -w},
            {x1, floorY + 0.022f, -w}, {x1, floorY + 0.022f, w},
            {x0, floorY + 0.212f,  w}, {x0, floorY + 0.212f,-w},
        };
        meshfactory::addHexahedron(out.body, c, pal.trim);

        for (int i = -2; i <= 2; ++i) {
            const float z = static_cast<float>(i) * w * 0.42f;
            const std::vector<glm::vec2> fin = {
                {x1, floorY + 0.02f}, {x0, floorY + 0.21f},
                {x0, floorY + 0.30f}, {x1, floorY + 0.10f},
            };
            extrudeAlongZ(out.body, fin, z - 0.012f, z + 0.012f, pal.trim * 1.5f, true);
        }
    }

    if (spec.rearWing) {
        const float span  = halfWidth * 0.96f;
        const float chord = length * 0.075f;
        const float wingX = tailSt.x + length * 0.055f + chord;
        // Height ABOVE THE DECK, not a fraction of roof height: a wing sits
        // on struts rising from the tail, and how far the roof happens to
        // be above the tail (large on a tall saloon-derived greenhouse,
        // small on a low cab-rearward one) has nothing to do with how tall
        // those struts should be. The previous roofY*0.94 put the wing
        // almost at roof height regardless, which reads as a broken stilt
        // on any body low enough that the roof and the tail are close
        // together - exactly the case for a low coupe.
        const float wingY = tailSt.top + height * 0.28f;

        const std::vector<glm::vec2> foil =
            aerofoil(wingX, wingY, chord, 0.11f, 0.085f, 14.0f);
        extrudeAlongZ(out.body, foil, -span, span, pal.trim * 1.3f, false);

        for (int side = 0; side < 2; ++side) {
            const float z = (side == 0) ? span : -span;
            const std::vector<glm::vec2> plate = {
                {wingX + chord * 0.30f, wingY - 0.06f},
                {wingX + chord * 0.34f, wingY + 0.16f},
                {wingX - chord * 1.10f, wingY + 0.30f},
                {wingX - chord * 1.16f, wingY + 0.02f},
            };
            extrudeAlongZ(out.body, plate, z, z + (side == 0 ? 0.016f : -0.016f),
                          pal.accent * 0.9f, true);
        }
        // Swan-neck mounts: over the wing, not under it, as GT3 cars run them.
        for (int side = 0; side < 2; ++side) {
            const float z = (side == 0 ? 1.0f : -1.0f) * halfWidth * 0.42f;
            const std::vector<glm::vec2> stay = {
                {wingX - chord * 0.55f, wingY + 0.02f},
                {wingX - chord * 0.30f, wingY + 0.02f},
                {tailSt.x + length * 0.02f, tailSt.top - 0.02f},
                {tailSt.x - length * 0.02f, tailSt.top - 0.02f},
            };
            extrudeAlongZ(out.body, stay, z - 0.02f, z + 0.02f, pal.trim, true);
        }
    }

    if (spec.duckTail) {
        // Trim-coloured, not body-coloured: a lip spoiler reads as an add-on
        // precisely because it is NOT painted the same as the panel it sits
        // on (it is usually bare carbon, satin black, or gloss black even on
        // an otherwise body-coloured car) - at 0.92x body colour the previous
        // version was only a shading gradient away from invisible.
        const float x0 = tailSt.x + length * 0.02f;
        const float x1 = x0 + length * 0.10f;
        const float w  = halfWidth * 0.88f;
        const Station& deck = body.st[static_cast<std::size_t>(body.sectionAt(0.94f))];
        const glm::vec3 c[8] = {
            {x1, deck.top - 0.03f,        -w}, {x1, deck.top - 0.03f,         w},
            {x0, deck.top + 0.075f,        w}, {x0, deck.top + 0.075f,       -w},
            {x1, deck.top - 0.03f + 0.02f,-w}, {x1, deck.top - 0.03f + 0.02f, w},
            {x0, deck.top + 0.100f,        w}, {x0, deck.top + 0.100f,       -w},
        };
        meshfactory::addHexahedron(out.body, c, pal.trim * 1.2f);
    }

    if (spec.roofScoop) {
        const float x0 = xNose - length * 0.50f;
        const float x1 = xNose - length * 0.62f;
        addBoxCorners(out.body, x1, x0, roofY - 0.02f, roofY + 0.055f,
                      -halfWidth * 0.17f, halfWidth * 0.17f, pal.trim);
    }

    if (spec.roofRails) {
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            addBoxCorners(out.body, xNose - length * 0.82f, xNose - length * 0.36f,
                          roofY, roofY + 0.045f,
                          sgn * halfWidth * 0.52f, sgn * halfWidth * 0.66f, pal.trim * 1.6f);
        }
    }

    // Mirrors: a stalk off the flank just ahead of the door, and a pod.
    {
        const int mi = body.sectionAt(spec.mirrorT);
        const float mx = body.st[static_cast<std::size_t>(mi)].x;
        const float my = beltY + height * (resolved == CarBodyStyle::Race ? 0.02f : 0.01f);
        const float mz = body.st[static_cast<std::size_t>(mi)].halfWidth;

        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            addBoxCorners(out.body, mx - 0.03f, mx + 0.03f, my - 0.012f, my + 0.012f,
                          sgn * mz * 0.92f, sgn * (mz + 0.10f), pal.trim);
            addBoxCorners(out.body, mx - 0.055f, mx + 0.075f, my - 0.045f, my + 0.045f,
                          sgn * (mz + 0.07f), sgn * (mz + 0.15f), pal.body * 0.95f);
        }
    }

    // Exhaust. Clustered on the centreline for a mid-engined car that vents
    // the whole bay out the back, side exit ahead of the rear wheel on a
    // race car, twin tips under the valance on everything else.
    if (spec.centreExhaust) {
        for (int i = -1; i <= 1; ++i) {
            const float z = static_cast<float>(i) * 0.115f;
            out.body.append(meshfactory::cylinderAlong(
                {tailSt.x + 0.05f, floorY + 0.115f, z},
                {-0.12f, 0.0f, 0.0f}, 0.044f, 12, pal.trim * 2.6f));
        }
    } else if (spec.sideExhaust) {
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            out.body.append(meshfactory::cylinderAlong(
                {archR * 1.15f, floorY + 0.10f, sgn * halfWidth * 0.90f},
                {0.0f, 0.02f, sgn * 0.13f}, 0.048f, 12, pal.trim * 2.6f));
        }
    } else {
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            out.body.append(meshfactory::cylinderAlong(
                {tailSt.x + 0.06f, floorY + 0.045f, sgn * halfWidth * 0.55f},
                {-0.14f, 0.0f, 0.0f}, 0.038f, 12, pal.trim * 2.6f));
        }
    }

    //-----------------------------------------------------------------------
    //  Wheels and brakes
    //-----------------------------------------------------------------------
    const float wheelWidth = static_cast<float>(p.wheelWidth);
    out.wheelRight = buildWheel(wheelR, wheelWidth, resolved, pal);
    out.wheelLeft  = mirroredZ(out.wheelRight);
    out.brakeRight = buildBrake(wheelR, wheelWidth, pal);
    out.brakeLeft  = mirroredZ(out.brakeRight);

    return out;
}

} // namespace vsim
