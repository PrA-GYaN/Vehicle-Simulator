#include "rendering/TrackMesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/Coordinates.h"
#include "core/Units.h"

namespace vsim {
namespace {

//---------------------------------------------------------------------------
//  Layering.
//
//  Everything flat is drawn at a fixed height, in a fixed order, so the paint
//  never fights the asphalt and the grass never fights the paint. A few
//  millimetres is enough at these depth ranges and is far below the ride
//  height of any wheel, so nothing visibly floats.
//---------------------------------------------------------------------------
constexpr float kGrassY   = 0.004f;
constexpr float kAsphaltY = 0.010f;
constexpr float kPaintY   = 0.016f;
constexpr float kRubberY  = 0.013f;   ///< tyre marks, between asphalt and paint

//---------------------------------------------------------------------------
//  Circuit furniture, in metres from the edge of the racing surface.
//---------------------------------------------------------------------------
constexpr float kKerbWidth    = 1.15f;
constexpr float kGravelWidth  = 7.5f;
constexpr float kBarrierLat   = 11.0f;
constexpr float kVergeLat     = 20.0f;
constexpr float kCornerKerb   = 1.0f / 230.0f;  ///< curvature above which a corner gets kerbs
constexpr float kTightCorner  = 1.0f / 85.0f;   ///< and above which it gets a tyre wall
constexpr float kStraightCurv = 1.0f / 600.0f;

struct Frame {
    glm::vec3 pos{0.0f};      ///< centreline point, on the ground plane
    glm::vec3 forward{1, 0, 0};
    glm::vec3 right{0, 0, 1};
    float distance  = 0.0f;
    float curvature = 0.0f;   ///< signed, positive = right-hand corner
    float corner    = 0.0f;   ///< |curvature| dilated along the track - see below
    float cornerSign = 0.0f;
    float clearance = 1e9f;  ///< see computeSelfClearance() below
};

struct Circuit {
    std::vector<Frame> f;
    bool  closed = false;
    float halfWidth = 4.5f;

    int count() const { return static_cast<int>(f.size()); }
    int next(int i) const { return (i + 1 == count()) ? (closed ? 0 : i) : i + 1; }
    int segments() const { return closed ? count() : count() - 1; }
    float length() const { return f.empty() ? 0.0f : f.back().distance; }
};

/// Deterministic value in [0,1). Used for surface patchiness and scatter -
/// anything that needs to look unplanned but must be identical every run, so
/// two launches of the same track produce the same circuit.
float hash01(int a, int b) {
    unsigned int h = static_cast<unsigned int>(a) * 374761393u +
                     static_cast<unsigned int>(b) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<float>((h ^ (h >> 16)) & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

glm::vec3 at(const Frame& f, float lateral, float y) {
    return f.pos + f.right * lateral + glm::vec3(0.0f, y, 0.0f);
}

/// Quad spanning two lateral offsets between two consecutive frames. Every
/// flat or sloped surface on the circuit is a run of these.
void band(MeshData& m, const Frame& a, const Frame& b,
          float lat0, float lat1, float y0, float y1, const glm::vec3& color) {
    meshfactory::addQuadOriented(m,
                                 at(a, lat0, y0), at(a, lat1, y1),
                                 at(b, lat1, y1), at(b, lat0, y0),
                                 {0.0f, 1.0f, 0.0f}, color);
}

/// Box in a local frame: x along `fwd`, z along `right`, y up.
void frameBox(MeshData& m, const glm::vec3& origin, const glm::vec3& fwd,
              const glm::vec3& right, float x0, float x1, float y0, float y1,
              float z0, float z1, const glm::vec3& color) {
    const glm::vec3 up{0.0f, 1.0f, 0.0f};
    auto pt = [&](float x, float y, float z) {
        return origin + fwd * x + up * y + right * z;
    };
    const glm::vec3 c[8] = {
        pt(x0, y0, z0), pt(x0, y0, z1), pt(x1, y0, z1), pt(x1, y0, z0),
        pt(x0, y1, z0), pt(x0, y1, z1), pt(x1, y1, z1), pt(x1, y1, z0),
    };
    meshfactory::addHexahedron(m, c, color);
}

void addCone(MeshData& m, const glm::vec3& base, float radius, float height,
             int segments, const glm::vec3& color) {
    const glm::vec3 apex = base + glm::vec3(0.0f, height, 0.0f);
    for (int i = 0; i < segments; ++i) {
        const float a0 = kTwoPi * static_cast<float>(i)     / static_cast<float>(segments);
        const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        const glm::vec3 p0 = base + glm::vec3(std::cos(a0) * radius, 0.0f, std::sin(a0) * radius);
        const glm::vec3 p1 = base + glm::vec3(std::cos(a1) * radius, 0.0f, std::sin(a1) * radius);

        meshfactory::addTriangleOriented(m, apex, p0, p1,
                                         glm::normalize(p0 + p1 - 2.0f * base +
                                                        glm::vec3(0.0f, radius, 0.0f)),
                                         color);
        meshfactory::addTriangleOriented(m, base, p0, p1, {0.0f, -1.0f, 0.0f}, color * 0.7f);
    }
}

//---------------------------------------------------------------------------
//  Self-clearance.
//
//  Every lateral extent below (kBarrierLat, kVergeLat, ...) is a fixed
//  distance from the edge of the track, on the assumption that what is out
//  there is open ground. That holds for a hand-built circuit, but not for a
//  real one measured off its actual road: the Nordschleife folds back
//  within a few tens of metres of itself around Hohe Acht, where two
//  separate parts of the lap run past each other on different sides of the
//  same hill. This project draws every track flat (see TrackMesh.h), so the
//  vertical separation that keeps those two sections apart in reality is
//  gone, and barriers or verge built out to their full width there would
//  plant one part of the circuit's scenery inside another.
//
//  computeSelfClearance() measures, for every point, half the distance to
//  the nearest OTHER point on the track that is not simply its own
//  neighbour (excluded by an arc-length window). Capping every lateral
//  reach at (that distance - halfWidth) is a genuinely safe bound: if two
//  points i and j are D apart and neither one's furniture reaches past
//  D/2 - halfWidth beyond its own edge, nothing built from i can ever meet
//  anything built from j, regardless of which way either section is
//  facing. A uniform grid keeps the search close to O(n) - the cell size is
//  bigger than the widest thing that ever reads this (the verge), so only a
//  small fixed neighbourhood around each point is ever visited, and it is a
//  no-op (clearance stays effectively infinite) for a track that never
//  comes near itself.
//---------------------------------------------------------------------------
void computeSelfClearance(std::vector<Frame>& frames, bool closed) {
    const int n = static_cast<int>(frames.size());
    if (n == 0) return;

    constexpr float kCellSize  = 24.0f;  ///< > kVergeLat, so a 5x5 block covers kSearchMax
    constexpr float kSearchMax = 60.0f;  ///< beyond this nothing below ever clamps anyway
    constexpr float kArcWindow = 80.0f;  ///< "same neighbourhood", not a fold-back

    auto cellKey = [](int gx, int gz) -> std::int64_t {
        return (static_cast<std::int64_t>(gx) << 32) ^
               static_cast<std::uint32_t>(gz);
    };
    auto cellOf = [&](const glm::vec3& p) {
        return std::pair<int, int>{static_cast<int>(std::floor(p.x / kCellSize)),
                                   static_cast<int>(std::floor(p.z / kCellSize))};
    };

    std::unordered_map<std::int64_t, std::vector<int>> grid;
    grid.reserve(static_cast<std::size_t>(n) / 2 + 1);
    for (int i = 0; i < n; ++i) {
        const auto [gx, gz] = cellOf(frames[static_cast<std::size_t>(i)].pos);
        grid[cellKey(gx, gz)].push_back(i);
    }

    const float loopLength = closed ? frames.back().distance : -1.0f;

    for (int i = 0; i < n; ++i) {
        Frame& fi = frames[static_cast<std::size_t>(i)];
        const auto [gx, gz] = cellOf(fi.pos);
        float best = kSearchMax;

        for (int dz = -2; dz <= 2; ++dz) {
            for (int dx = -2; dx <= 2; ++dx) {
                const auto it = grid.find(cellKey(gx + dx, gz + dz));
                if (it == grid.end()) continue;

                for (int j : it->second) {
                    if (j == i) continue;
                    float arcGap = std::abs(frames[static_cast<std::size_t>(j)].distance -
                                            fi.distance);
                    if (loopLength > 0.0f) arcGap = std::min(arcGap, loopLength - arcGap);
                    if (arcGap < kArcWindow) continue;

                    const float d = glm::length(frames[static_cast<std::size_t>(j)].pos - fi.pos);
                    if (d < best) best = d;
                }
            }
        }
        fi.clearance = best * 0.5f;
    }
}

/// Cap a furniture piece's reach so it stays on the near side of half the
/// gap to the nearest other part of the track (see computeSelfClearance).
/// Floored at 1.5 m rather than let a pinched stretch collapse to nothing -
/// a narrow barrier reads better than a hole where one vanishes entirely.
/// That floor gives up the zero-overlap guarantee only where two parts of
/// the same track pass within a few metres of the track width of each
/// other, which no real circuit does; every measured track this generator
/// draws, the Nordschleife included, clears it with metres to spare.
float clampReach(float desired, float halfWidth, float clearance) {
    return std::max(1.5f, std::min(desired, clearance - halfWidth));
}

//---------------------------------------------------------------------------
//  Centreline -> frames
//---------------------------------------------------------------------------

Circuit buildCircuit(const Track& track) {
    Circuit c;
    c.closed    = track.closed();
    c.halfWidth = static_cast<float>(track.width() * 0.5);

    const auto& pts = track.centreline();
    c.f.reserve(pts.size());

    for (const TrackPoint& p : pts) {
        Frame f;
        f.pos       = glm::vec3(p.position);
        const double yaw = coords::yawFromDirection(p.tangent);
        f.forward   = glm::vec3(coords::heading(yaw));
        f.right     = glm::vec3(coords::right(yaw));
        f.distance  = static_cast<float>(p.distance);
        f.curvature = static_cast<float>(p.curvature);
        c.f.push_back(f);
    }

    // Dilate the corner measure along the track. A kerb does not begin
    // exactly where the arc does - it runs in before the turn-in point and
    // out past the exit - and the centreline's curvature is a step function
    // (Track::appendArc), so taking a running maximum is both what the
    // geometry needs and what hides the step.
    constexpr int kDilate = 10;   // ~15 m at the 1.5 m centreline spacing
    const int n = c.count();
    for (int i = 0; i < n; ++i) {
        float best = 0.0f;
        float sign = 0.0f;
        for (int d = -kDilate; d <= kDilate; ++d) {
            int j = i + d;
            if (c.closed) {
                j = (j % n + n) % n;
            } else if (j < 0 || j >= n) {
                continue;
            }
            const float k = std::abs(c.f[static_cast<std::size_t>(j)].curvature);
            if (k > best) {
                best = k;
                sign = (c.f[static_cast<std::size_t>(j)].curvature >= 0.0f) ? 1.0f : -1.0f;
            }
        }
        c.f[static_cast<std::size_t>(i)].corner     = best;
        c.f[static_cast<std::size_t>(i)].cornerSign = sign;
    }

    computeSelfClearance(c.f, c.closed);
    return c;
}

/// Frame at an arbitrary arc length, interpolated between samples. Paint is
/// centimetres wide along the track, far finer than the centreline spacing,
/// so it cannot be snapped to samples.
Frame frameAtDistance(const Circuit& c, float s) {
    const int n = c.count();
    if (n == 0) return {};
    s = std::min(std::max(s, 0.0f), c.length());

    int lo = 0, hi = n - 1;
    while (lo + 1 < hi) {
        const int mid = (lo + hi) / 2;
        if (c.f[static_cast<std::size_t>(mid)].distance <= s) lo = mid; else hi = mid;
    }

    const Frame& a = c.f[static_cast<std::size_t>(lo)];
    const Frame& b = c.f[static_cast<std::size_t>(std::min(lo + 1, n - 1))];
    const float span = std::max(b.distance - a.distance, 1e-4f);
    const float u = std::min(std::max((s - a.distance) / span, 0.0f), 1.0f);

    Frame f;
    f.pos      = glm::mix(a.pos, b.pos, u);
    f.forward  = glm::normalize(glm::mix(a.forward, b.forward, u));
    f.right    = glm::normalize(glm::mix(a.right, b.right, u));
    f.distance = s;
    f.curvature = glm::mix(a.curvature, b.curvature, u);
    f.corner    = glm::mix(a.corner, b.corner, u);
    f.cornerSign = (u < 0.5f) ? a.cornerSign : b.cornerSign;
    f.clearance = std::min(a.clearance, b.clearance);
    return f;
}

/// Frames covering [s0, s1]: the two ends plus every centreline sample in
/// between, so a painted rectangle still follows the curve of the road.
std::vector<Frame> rangeFrames(const Circuit& c, float s0, float s1) {
    std::vector<Frame> out;
    if (s1 <= s0) return out;

    out.push_back(frameAtDistance(c, s0));
    for (const Frame& f : c.f) {
        if (f.distance > s0 + 1e-3f && f.distance < s1 - 1e-3f) out.push_back(f);
    }
    out.push_back(frameAtDistance(c, s1));
    return out;
}

/// Flat painted rectangle in track coordinates: [s0,s1] along, [lat0,lat1]
/// across. Edge lines, the start line, grid boxes and rubber marks are all
/// this call with different numbers.
void paintRect(MeshData& m, const Circuit& c, float s0, float s1,
               float lat0, float lat1, float y, const glm::vec3& color) {
    const std::vector<Frame> fr = rangeFrames(c, s0, s1);
    for (std::size_t i = 0; i + 1 < fr.size(); ++i) {
        band(m, fr[i], fr[i + 1], lat0, lat1, y, y, color);
    }
}

//---------------------------------------------------------------------------
//  Surfaces
//---------------------------------------------------------------------------

/// Height of the ground beside the track: flat for the first few metres, then
/// rising into a shallow embankment. Keeps the scenery from looking like it
/// was laid on a table, without ever touching the racing surface.
float vergeHeight(float lateralFromEdge) {
    const float d = std::max(lateralFromEdge - kKerbWidth, 0.0f);
    const float u = std::min(d / (kVergeLat - kKerbWidth), 1.0f);
    return kGrassY + 0.55f * u * u;
}

void buildAsphalt(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    constexpr int kBands = 12;
    const float hw = c.halfWidth;

    for (int i = 0; i < c.segments(); ++i) {
        const Frame& a = c.f[static_cast<std::size_t>(i)];
        const Frame& b = c.f[static_cast<std::size_t>(c.next(i))];

        for (int k = 0; k < kBands; ++k) {
            const float u0 = -1.0f + 2.0f * static_cast<float>(k)     / static_cast<float>(kBands);
            const float u1 = -1.0f + 2.0f * static_cast<float>(k + 1) / static_cast<float>(kBands);
            const float uMid = 0.5f * (u0 + u1);

            // Tarmac is not one colour: the middle is polished and darkened
            // by rubber, the edges are dusty and bleached. Plus a little
            // deterministic patchiness so long straights are not flat panels.
            float shade = 1.0f;
            shade *= 1.0f - 0.09f * std::max(0.0f, 1.0f - std::abs(uMid) / 0.6f);
            shade *= 1.0f + 0.10f * std::max(0.0f, (std::abs(uMid) - 0.72f) / 0.28f);
            shade *= 0.96f + 0.08f * hash01(i / 5, k);

            band(m, a, b, u0 * hw, u1 * hw, kAsphaltY, kAsphaltY, pal.asphalt * shade);
        }
    }
}

void buildMarkings(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    const float hw = c.halfWidth;
    const float lineW = 0.14f;
    const float lineIn = hw - 0.22f;

    for (int i = 0; i < c.segments(); ++i) {
        const Frame& a = c.f[static_cast<std::size_t>(i)];
        const Frame& b = c.f[static_cast<std::size_t>(c.next(i))];

        band(m, a, b, lineIn - lineW, lineIn, kPaintY, kPaintY, pal.marking);
        band(m, a, b, -lineIn, -lineIn + lineW, kPaintY, kPaintY, pal.marking);

        // Rubber laid down on the racing line through a corner: darker
        // asphalt on the inside of the turn, which is the one piece of
        // surface detail that tells you where the line goes.
        if (a.corner > kCornerKerb) {
            const float side = a.cornerSign;
            const float centre = side * hw * 0.42f;
            band(m, a, b, centre - hw * 0.30f, centre + hw * 0.30f,
                 kRubberY, kRubberY, pal.asphalt * 0.80f);
        }
    }
}

/// Kerbs are walked at a FIXED ARC LENGTH STEP rather than per centreline
/// sample, and every length in them is a whole multiple of that step:
///
///     step 0.40 m   rib period 0.80 m   colour block 1.60 m
///
/// The centreline is sampled every 1.5 m (Track.cpp), which is coarser than
/// both the ribs and the colour blocks - drawing a kerb on those samples
/// beats one pattern against the other and produces noise rather than a
/// kerb. Commensurate steps make every rib crest and every colour change
/// land exactly on a section boundary.
void buildKerbs(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    const float hw = c.halfWidth;

    constexpr float kStep      = 0.40f;
    constexpr float kRibPeriod = 0.80f;
    constexpr float kBlock     = 1.60f;
    constexpr float kRibHeight = 0.018f;

    /// How much kerb there is here, 0..1. Ramped rather than switched, so a
    /// kerb rises out of the asphalt over a couple of metres at the entry to
    /// a corner and sinks back at the exit instead of appearing as a step.
    auto amount = [](float corner) {
        const float u = (corner - kCornerKerb * 0.55f) / (kCornerKerb * 0.95f);
        const float t = std::min(std::max(u, 0.0f), 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };

    const int steps = static_cast<int>(c.length() / kStep);
    for (int i = 0; i < steps; ++i) {
        const float s0 = static_cast<float>(i) * kStep;
        const float s1 = s0 + kStep;

        const Frame a = frameAtDistance(c, s0);
        const Frame b = frameAtDistance(c, s1);

        const float amtA = amount(a.corner);
        const float amtB = amount(b.corner);
        if (amtA <= 0.001f && amtB <= 0.001f) continue;

        // Colour block. Both ends of this section fall inside the same block
        // because kBlock is a multiple of kStep.
        const bool red = (static_cast<int>(s0 / kBlock) % 2) == 0;
        const glm::vec3 top  = red ? pal.kerbRed : pal.kerbWhite;
        const glm::vec3 side = top * 0.72f;

        // Ribs: the crest alternates every step, which at kRibPeriod = 2 *
        // kStep is exactly one crest and one trough per period.
        const bool crestA = (i % 2) == 0;
        const float ribA = crestA ? kRibHeight : 0.0f;
        const float ribB = crestA ? 0.0f : kRibHeight;

        for (int sd = 0; sd < 2; ++sd) {
            const float sgn = (sd == 0) ? 1.0f : -1.0f;

            // The inside of the corner always gets a kerb; the outside gets a
            // narrower one, which is how a real circuit is built - the exit
            // kerb catches a car running wide.
            const bool inside = (sgn == a.cornerSign);
            const float fullWidth = inside ? kKerbWidth : kKerbWidth * 0.70f;
            const float crown     = inside ? 0.060f : 0.048f;

            const float wA = fullWidth * amtA;
            const float wB = fullWidth * amtB;
            const float cA = kAsphaltY + (crown - kAsphaltY) * amtA;
            const float cB = kAsphaltY + (crown - kAsphaltY) * amtB;

            auto lat = [&](float w, float f) { return sgn * (hw + w * f); };

            // Four bands: the rise off the asphalt, the ribbed tread, the
            // shoulder, and the drop to the grass.
            meshfactory::addQuadOriented(m,
                at(a, sgn * hw, kAsphaltY), at(a, lat(wA, 0.18f), cA),
                at(b, lat(wB, 0.18f), cB),  at(b, sgn * hw, kAsphaltY),
                {0, 1, 0}, side * 1.1f);

            meshfactory::addQuadOriented(m,
                at(a, lat(wA, 0.18f), cA + ribA * amtA), at(a, lat(wA, 0.84f), cA + ribA * amtA),
                at(b, lat(wB, 0.84f), cB + ribB * amtB), at(b, lat(wB, 0.18f), cB + ribB * amtB),
                {0, 1, 0}, top);

            meshfactory::addQuadOriented(m,
                at(a, lat(wA, 0.84f), cA), at(a, lat(wA, 1.0f), cA * 0.7f),
                at(b, lat(wB, 1.0f), cB * 0.7f), at(b, lat(wB, 0.84f), cB),
                {0, 1, 0}, side);

            meshfactory::addQuadOriented(m,
                at(a, lat(wA, 1.0f), cA * 0.7f), at(a, lat(wA, 1.0f) + sgn * 0.25f, vergeHeight(fullWidth)),
                at(b, lat(wB, 1.0f) + sgn * 0.25f, vergeHeight(fullWidth)), at(b, lat(wB, 1.0f), cB * 0.7f),
                {0, 1, 0}, pal.grass * 0.9f);
        }
    }
}

void buildVerge(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    const float hw = c.halfWidth;
    constexpr int kBands = 7;

    for (int i = 0; i < c.segments(); ++i) {
        const Frame& a = c.f[static_cast<std::size_t>(i)];
        const Frame& b = c.f[static_cast<std::size_t>(c.next(i))];

        for (int s = 0; s < 2; ++s) {
            const float sgn = (s == 0) ? 1.0f : -1.0f;

            // Gravel goes on the OUTSIDE of a corner, which is the only place
            // a car leaves the track fast enough to need it.
            const bool outside = (a.corner > kCornerKerb) && (sgn == -a.cornerSign);

            // Clamped per segment rather than a fixed kVergeLat: see
            // computeSelfClearance - a track that folds close to itself
            // (the Nordschleife, around Hohe Acht) must not draw grass or
            // gravel out past the point where it would meet the other side
            // of the same loop.
            const float vergeLat = clampReach(kVergeLat, hw, std::min(a.clearance, b.clearance));

            for (int k = 0; k < kBands; ++k) {
                const float t0 = static_cast<float>(k)     / static_cast<float>(kBands);
                const float t1 = static_cast<float>(k + 1) / static_cast<float>(kBands);
                const float d0 = kKerbWidth + t0 * (vergeLat - kKerbWidth);
                const float d1 = kKerbWidth + t1 * (vergeLat - kKerbWidth);

                const bool gravel = outside && (d1 <= kGravelWidth);
                glm::vec3 col = gravel ? pal.gravel : pal.grass;
                col *= 0.93f + 0.14f * hash01(i / 3, k * 7 + s);

                band(m, a, b, sgn * (hw + d0), sgn * (hw + d1),
                     vergeHeight(d0), vergeHeight(d1), col);
            }
        }
    }
}

//---------------------------------------------------------------------------
//  Barriers
//---------------------------------------------------------------------------

void buildBarriers(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    const float hw = c.halfWidth;
    const float base = vergeHeight(kBarrierLat);

    //-----------------------------------------------------------------------
    //  Armco.
    //
    //  A guard rail is a W in cross-section, and that profile is the whole
    //  reason it reads as a barrier from a distance: the two valleys stay in
    //  shadow while the three ridges catch the light, so the rail draws a
    //  bright line along the circuit. Two flat strips - the first attempt -
    //  just look like a wire fence.
    //
    //  Profile, as (height above the rail centre, lateral offset from the
    //  post face). Positive lateral is toward the track.
    //-----------------------------------------------------------------------
    struct RailPoint { float y, lat; };
    static const RailPoint kWBeam[] = {
        {-0.170f, 0.000f},   // bottom lip, folded back to the post
        {-0.120f, 0.075f},
        {-0.045f, 0.075f},   // lower valley
        { 0.000f, 0.020f},   // centre fold
        { 0.045f, 0.075f},   // upper valley
        { 0.120f, 0.075f},
        { 0.170f, 0.000f},   // top lip
    };
    constexpr int kWBeamPoints = static_cast<int>(sizeof(kWBeam) / sizeof(kWBeam[0]));

    // Rail panels and posts share one span: a real W-beam rail is bolted up
    // in ~4 m straight lengths, post to post - it does not flex to follow
    // the road - so a panel per centreline sample (1.5 m, the resolution
    // the PHYSICS needs, not the one a straight steel rail does) was both
    // less accurate and 2.5x more triangles than the shape calls for. On a
    // long circuit - this generator now also builds the 20.8 km
    // Nordschleife, not just circuits a third that length - that
    // redundancy is the difference between a barrier mesh of a few hundred
    // thousand triangles and one of a million.
    constexpr float kPostStep = 4.0f;
    const int panels = std::max(1, static_cast<int>(c.length() / kPostStep));

    auto railPanel = [&](const Frame& a, const Frame& b) {
        const float barrierLat = clampReach(kBarrierLat, hw, std::min(a.clearance, b.clearance));

        for (int s = 0; s < 2; ++s) {
            const float sgn = (s == 0) ? 1.0f : -1.0f;
            const float post = sgn * (hw + barrierLat);
            const float railY = base + 0.62f;

            for (int p = 0; p + 1 < kWBeamPoints; ++p) {
                const float y0 = railY + kWBeam[p].y;
                const float y1 = railY + kWBeam[p + 1].y;
                const float l0 = post - sgn * kWBeam[p].lat;
                const float l1 = post - sgn * kWBeam[p + 1].lat;

                // Shade each facet by how much it faces up: the ridges read
                // bright, the valleys dark, which is what draws the line.
                const float slope = std::abs(y1 - y0) /
                                    std::max(std::abs(y1 - y0) + std::abs(l1 - l0), 1e-4f);
                band(m, a, b, l0, l1, y0, y1, pal.steel * (1.25f - 0.45f * slope));
            }
        }
    };

    for (int i = 0; i < panels; ++i) {
        const float s0 = static_cast<float>(i) * kPostStep;
        const float s1 = s0 + kPostStep;
        railPanel(frameAtDistance(c, s0), frameAtDistance(c, std::min(s1, c.length())));
    }
    // Fixed-length panels leave up to one kPostStep of leftover distance
    // uncovered (length() is rarely an exact multiple of 4 m). On an open
    // road that trailing sliver is simply the last few metres before the
    // road ends and is invisible. On a closed loop it would otherwise be a
    // permanent gap in the rail at the seam - every other per-segment loop
    // here closes across that same seam via Circuit::next(), so this one
    // final panel, back from wherever the fixed steps left off to frame 0,
    // does the same for the rail.
    if (c.closed) {
        railPanel(frameAtDistance(c, static_cast<float>(panels) * kPostStep), c.f.front());
    }

    // Posts. Behind the rail, so the rail stands proud of them.
    for (float s = 0.0f; s < c.length(); s += kPostStep) {
        const Frame f = frameAtDistance(c, s);
        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;
            const float lat = sgn * (hw + clampReach(kBarrierLat, hw, f.clearance) + 0.06f);
            frameBox(m, f.pos, f.forward, f.right, -0.055f, 0.055f,
                     base, base + 0.80f, lat - 0.06f, lat + 0.06f, pal.steel * 0.68f);
        }
    }

    //-----------------------------------------------------------------------
    //  Tyre walls and debris fencing, at the OUTSIDE of the tightest corners
    //  only.
    //
    //  Both were previously run along every tight corner at close spacing and
    //  placed out in the middle of the gravel, which produced a black snake
    //  through the runoff. On a real circuit a tyre wall is bolted to the
    //  face of the barrier - it is the barrier's padding, not scenery - so it
    //  goes at kBarrierLat, and only where a car can actually reach it.
    //-----------------------------------------------------------------------
    for (float s = 0.0f; s < c.length(); s += 0.82f) {
        const Frame f = frameAtDistance(c, s);
        if (f.corner < kTightCorner) continue;

        const float sgn = -f.cornerSign;                  // outside of the turn
        const float lat = sgn * (hw + clampReach(kBarrierLat, hw, f.clearance) - 0.55f);
        const glm::vec3 origin = at(f, lat, base);

        // Three high, with a conveyor-belt cover over the top row - the
        // yellow-and-black or red-and-white banding is the only part of a
        // tyre wall visible at racing speed.
        for (int row = 0; row < 3; ++row) {
            const float y = static_cast<float>(row) * 0.29f;
            const bool topRow = (row == 2);
            const glm::vec3 col = topRow
                ? ((static_cast<int>(s / 3.2f) % 2 == 0) ? pal.accent * 0.8f : pal.marking * 0.85f)
                : pal.rubber * (0.9f + 0.25f * hash01(static_cast<int>(s * 7.0f), row));

            m.append(meshfactory::cylinderAlong(origin + glm::vec3(0.0f, y, 0.0f),
                                                {0.0f, 0.27f, 0.0f}, 0.33f, 9, col));
        }
    }

    for (float s = 0.0f; s < c.length(); s += 7.0f) {
        const Frame f = frameAtDistance(c, s);
        if (f.corner < kCornerKerb) continue;

        const float sgn = -f.cornerSign;
        const float lat = sgn * (hw + clampReach(kBarrierLat, hw, f.clearance) + 0.35f);

        frameBox(m, f.pos, f.forward, f.right, -0.05f, 0.05f,
                 base, base + 2.8f, lat - 0.05f, lat + 0.05f, pal.steel * 0.62f);
    }
}

//---------------------------------------------------------------------------
//  Scenery
//---------------------------------------------------------------------------

/// Longest run of near-zero curvature: the main straight, where the pits,
/// the grandstands, the gantry and the grid belong. Derived rather than
/// configured, so it is still right for a track this code has never seen.
void findMainStraight(const Circuit& c, float& s0, float& s1) {
    int bestStart = 0, bestLen = 0;
    int start = -1;

    for (int i = 0; i < c.count(); ++i) {
        const bool straight = std::abs(c.f[static_cast<std::size_t>(i)].curvature) < kStraightCurv;
        if (straight) {
            if (start < 0) start = i;
            if (i - start > bestLen) { bestLen = i - start; bestStart = start; }
        } else {
            start = -1;
        }
    }
    s0 = c.f[static_cast<std::size_t>(bestStart)].distance;
    s1 = c.f[static_cast<std::size_t>(bestStart + bestLen)].distance;
}

void buildStartLine(MeshData& markings, MeshData& scenery, const Circuit& c,
                    const TrackPalette& pal, float lineS) {
    const float hw = c.halfWidth;

    // Start/finish line, and a chequered band ahead of it.
    paintRect(markings, c, lineS, lineS + 0.55f, -hw, hw, kPaintY + 0.002f, pal.marking);
    for (int k = 0; k < 16; ++k) {
        const float u0 = -1.0f + 2.0f * static_cast<float>(k)     / 16.0f;
        const float u1 = -1.0f + 2.0f * static_cast<float>(k + 1) / 16.0f;
        for (int row = 0; row < 2; ++row) {
            const bool white = ((k + row) % 2) == 0;
            if (!white) continue;
            const float a = lineS + 0.65f + static_cast<float>(row) * 0.45f;
            paintRect(markings, c, a, a + 0.45f, u0 * hw, u1 * hw,
                      kPaintY + 0.002f, pal.marking);
        }
    }

    // Starting grid: staggered boxes behind the line, drawn as three bars so
    // they read as grid slots rather than painted rectangles.
    for (int slot = 0; slot < 20; ++slot) {
        const float s = lineS - 9.0f - static_cast<float>(slot) * 7.5f;
        if (s < 6.0f) break;

        const float lat = ((slot % 2) == 0 ? 1.0f : -1.0f) * hw * 0.44f;
        const float halfBox = 1.15f;
        const float bar = 0.12f;

        paintRect(markings, c, s, s + bar, lat - halfBox, lat + halfBox, kPaintY, pal.marking);
        paintRect(markings, c, s, s + 4.6f, lat - halfBox, lat - halfBox + bar, kPaintY, pal.marking);
        paintRect(markings, c, s, s + 4.6f, lat + halfBox - bar, lat + halfBox, kPaintY, pal.marking);
    }

    // Gantry over the line.
    const Frame f = frameAtDistance(c, lineS);
    const float towerLat = hw + 1.6f;
    const float beamY = 6.4f;
    for (int side = 0; side < 2; ++side) {
        const float sgn = (side == 0) ? 1.0f : -1.0f;
        frameBox(scenery, f.pos, f.forward, f.right, -0.35f, 0.35f,
                 kGrassY, beamY, sgn * towerLat - 0.35f, sgn * towerLat + 0.35f,
                 pal.concrete * 0.85f);
    }
    frameBox(scenery, f.pos, f.forward, f.right, -0.30f, 0.30f,
             beamY, beamY + 0.9f, -towerLat, towerLat, pal.concrete);
    for (int light = 0; light < 5; ++light) {
        const float z = (-2.0f + static_cast<float>(light)) * hw * 0.28f;
        frameBox(scenery, f.pos, f.forward, f.right, -0.34f, -0.24f,
                 beamY - 0.62f, beamY - 0.02f, z - 0.26f, z + 0.26f, pal.rubber * 1.4f);
        frameBox(scenery, f.pos, f.forward, f.right, -0.40f, -0.34f,
                 beamY - 0.54f, beamY - 0.14f, z - 0.18f, z + 0.18f, pal.accent);
    }
}

/// Pit complex and grandstands along the main straight.
///
/// These are built in ONE frame - the straight's midpoint - and extend along
/// its tangent as true straight lines, because that is what a grandstand is.
/// The extent is therefore clamped to the part of the straight that is
/// actually straight, or the ends would drift away from a curving track.
void buildPitsAndStands(MeshData& surfaces, MeshData& m, const Circuit& c,
                        const TrackPalette& pal, float s0, float s1) {
    const float hw = c.halfWidth;
    const float mid = 0.5f * (s0 + s1);
    const Frame f = frameAtDistance(c, mid);

    const float half = std::min((s1 - s0) * 0.42f, 90.0f);
    if (half < 12.0f) return;

    const glm::vec3 up{0.0f, 1.0f, 0.0f};
    auto ground = [&](float x, float z) {
        return f.pos + f.forward * x + f.right * z + up * kGrassY;
    };

    //-- pit lane ----------------------------------------------------------
    //
    // There has to be a road surface between the pit wall and the garages.
    // Without it the wall and the building sit in a field, which is what the
    // first version looked like.
    const float wallLat   = hw + 2.4f;
    const float laneInner = wallLat + 0.35f;
    const float laneOuter = hw + 14.0f;

    for (int k = 0; k < 4; ++k) {
        const float t0 = static_cast<float>(k)     / 4.0f;
        const float t1 = static_cast<float>(k + 1) / 4.0f;
        const float z0 = laneInner + t0 * (laneOuter - laneInner);
        const float z1 = laneInner + t1 * (laneOuter - laneInner);

        meshfactory::addQuadOriented(surfaces,
                                     ground(-half - 25.0f, z0), ground(-half - 25.0f, z1),
                                     ground(half + 25.0f, z1), ground(half + 25.0f, z0),
                                     up, pal.asphalt * (1.06f + 0.03f * static_cast<float>(k)));
    }
    // Pit box markings: one white bay outline per garage.
    const int garages = std::max(2, static_cast<int>(half * 2.0f / 9.0f));
    for (int g = 0; g < garages; ++g) {
        const float x = -half + (static_cast<float>(g) + 0.5f) * (2.0f * half) /
                                static_cast<float>(garages);
        const float z0 = laneInner + 1.2f;
        const float z1 = laneOuter - 1.0f;
        for (float edge : {x - 3.0f, x + 3.0f}) {
            meshfactory::addQuadOriented(surfaces,
                                         ground(edge - 0.07f, z0) + up * 0.006f,
                                         ground(edge - 0.07f, z1) + up * 0.006f,
                                         ground(edge + 0.07f, z1) + up * 0.006f,
                                         ground(edge + 0.07f, z0) + up * 0.006f,
                                         up, pal.marking);
        }
    }

    //-- pit wall ----------------------------------------------------------
    frameBox(m, f.pos, f.forward, f.right, -half, half, kGrassY, 1.05f,
             wallLat, wallLat + 0.3f, pal.concrete);
    frameBox(m, f.pos, f.forward, f.right, -half, half, 1.05f, 1.22f,
             wallLat - 0.03f, wallLat + 0.33f, pal.accent);

    //-- garages -----------------------------------------------------------
    const float garageLat = laneOuter;
    frameBox(m, f.pos, f.forward, f.right, -half, half, kGrassY, 6.2f,
             garageLat, garageLat + 11.0f, pal.concrete * 1.05f);
    frameBox(m, f.pos, f.forward, f.right, -half - 0.7f, half + 0.7f, 6.2f, 6.5f,
             garageLat - 1.6f, garageLat + 11.6f, pal.concrete * 0.78f);
    // Canopy posts, so the overhang is supported rather than floating.
    for (int p = 0; p <= garages; ++p) {
        const float x = -half + 2.0f * half * static_cast<float>(p) / static_cast<float>(garages);
        frameBox(m, f.pos, f.forward, f.right, x - 0.12f, x + 0.12f, kGrassY, 6.2f,
                 garageLat - 1.5f, garageLat - 1.26f, pal.steel * 0.75f);
    }
    for (int g = 0; g < garages; ++g) {
        const float x = -half + (static_cast<float>(g) + 0.5f) * (2.0f * half) /
                                static_cast<float>(garages);
        frameBox(m, f.pos, f.forward, f.right, x - 2.6f, x + 2.6f, kGrassY, 3.5f,
                 garageLat - 0.14f, garageLat + 0.02f, pal.rubber * 1.9f);
        // Team board above each door.
        frameBox(m, f.pos, f.forward, f.right, x - 2.6f, x + 2.6f, 3.7f, 4.5f,
                 garageLat - 0.16f, garageLat - 0.02f,
                 (g % 2 == 0) ? pal.accent * 0.75f : glm::vec3(0.22f, 0.28f, 0.40f));
    }

    //-- grandstand --------------------------------------------------------
    //
    // Tiered seating, raked back from the track, with a roof carried on a
    // rear truss. The roof spans only the back two thirds - a slab over the
    // whole stand hides the seating completely from any camera above it,
    // which is where this one is normally seen from.
    const float standLat = -(hw + 15.0f);
    const int rows = 16;
    const float depth = 0.95f;
    const float rise  = 0.46f;

    for (int r = 0; r < rows; ++r) {
        const float z0 = standLat - static_cast<float>(r) * depth;
        const float y1 = 0.8f + static_cast<float>(r) * rise;

        frameBox(m, f.pos, f.forward, f.right, -half, half, kGrassY, y1,
                 z0 - depth, z0, pal.concrete * 0.88f);
        frameBox(m, f.pos, f.forward, f.right, -half, half, y1, y1 + 0.30f,
                 z0 - depth + 0.18f, z0 - 0.12f,
                 (r % 4 == 0) ? pal.accent * 0.8f : glm::vec3(0.26f, 0.31f, 0.44f));
    }

    const float backZ = standLat - static_cast<float>(rows) * depth;
    const float topY  = 0.8f + static_cast<float>(rows) * rise;
    const float roofY = topY + 3.6f;

    frameBox(m, f.pos, f.forward, f.right, -half, half, roofY, roofY + 0.30f,
             backZ - 1.2f, standLat - static_cast<float>(rows) * depth * 0.35f,
             pal.steel * 0.88f);
    for (int p = 0; p <= 8; ++p) {
        const float x = -half + 2.0f * half * static_cast<float>(p) / 8.0f;
        frameBox(m, f.pos, f.forward, f.right, x - 0.16f, x + 0.16f, kGrassY, roofY,
                 backZ - 1.1f, backZ - 0.75f, pal.steel * 0.72f);
    }
    // Back wall, so the stand is not an open staircase from behind.
    frameBox(m, f.pos, f.forward, f.right, -half, half, kGrassY, topY,
             backZ - 1.0f, backZ - 0.7f, pal.concrete * 0.7f);
}

void buildBrakingBoards(MeshData& m, const Circuit& c, const TrackPalette& pal) {
    const float hw = c.halfWidth;

    // A board goes 150/100/50 m before the entry to every real corner. Corner
    // entry is where the dilated corner measure first rises above the kerb
    // threshold, which is the same test the kerbs themselves use.
    for (int i = 1; i < c.count(); ++i) {
        const Frame& prev = c.f[static_cast<std::size_t>(i - 1)];
        const Frame& cur  = c.f[static_cast<std::size_t>(i)];
        if (!(prev.corner < kTightCorner && cur.corner >= kTightCorner)) continue;

        for (int b = 1; b <= 3; ++b) {
            const float s = cur.distance - static_cast<float>(b) * 50.0f;
            if (s < 2.0f) continue;

            const Frame f = frameAtDistance(c, s);
            const float sgn = -cur.cornerSign;
            const float lat = sgn * (hw + 3.2f);
            const float ground = vergeHeight(3.2f);

            frameBox(m, f.pos, f.forward, f.right, -0.06f, 0.06f, ground, ground + 1.1f,
                     lat - 0.06f, lat + 0.06f, pal.steel * 0.7f);
            frameBox(m, f.pos, f.forward, f.right, -0.05f, 0.05f,
                     ground + 1.1f, ground + 1.9f, lat - 0.55f, lat + 0.55f, pal.marking);
            // b stripes, the way a distance board is read at speed.
            for (int stripe = 0; stripe < b; ++stripe) {
                const float y = ground + 1.22f + static_cast<float>(stripe) * 0.22f;
                frameBox(m, f.pos, f.forward, f.right, -0.06f, -0.05f, y, y + 0.14f,
                         lat - 0.42f, lat + 0.42f, pal.rubber * 1.6f);
            }
        }
    }
}

void buildTrees(MeshData& m, const Circuit& c, const TrackPalette& pal,
                float straightStart, float straightEnd) {
    for (float s = 0.0f; s < c.length(); s += 11.0f) {
        const Frame f = frameAtDistance(c, s);

        for (int side = 0; side < 2; ++side) {
            const float sgn = (side == 0) ? 1.0f : -1.0f;

            // Keep the main straight clear: that is where the pit complex and
            // the grandstand stand.
            if (s > straightStart - 30.0f && s < straightEnd + 30.0f) continue;

            const float r = hash01(static_cast<int>(s), side);
            if (r < 0.35f) continue;

            const float lat = sgn * (c.halfWidth + kVergeLat + 2.0f + r * 26.0f);
            const float scale = 0.75f + hash01(static_cast<int>(s) + 977, side) * 0.9f;
            const glm::vec3 base = at(f, lat, vergeHeight(kVergeLat) - 0.2f);

            m.append(meshfactory::cylinderAlong(base, {0.0f, 1.1f * scale, 0.0f},
                                                0.16f * scale, 6, pal.timber));
            addCone(m, base + glm::vec3(0.0f, 0.9f * scale, 0.0f),
                    1.5f * scale, 3.4f * scale, 8, pal.foliage);
            addCone(m, base + glm::vec3(0.0f, 2.3f * scale, 0.0f),
                    1.1f * scale, 2.8f * scale, 8, pal.foliage * 1.15f);
        }
    }
}

} // namespace

//===========================================================================

TrackMeshSet buildTrackMeshes(const Track& track, const TrackPalette& pal) {
    TrackMeshSet out;

    const Circuit c = buildCircuit(track);
    if (c.count() < 2) return out;

    buildAsphalt(out.asphalt, c, pal);
    buildMarkings(out.markings, c, pal);
    buildKerbs(out.kerbs, c, pal);
    buildVerge(out.verge, c, pal);
    buildBarriers(out.barriers, c, pal);

    float s0 = 0.0f, s1 = 0.0f;
    findMainStraight(c, s0, s1);

    // The line goes near the end of the main straight so the grid behind it
    // still fits on the straight.
    const float lineS = s0 + (s1 - s0) * 0.80f;
    buildStartLine(out.markings, out.scenery, c, pal, lineS);
    // The pit lane is a driving surface, so it goes in with the asphalt
    // rather than the scenery - it has to layer with the road, not with the
    // buildings standing on it.
    buildPitsAndStands(out.asphalt, out.scenery, c, pal, s0, s1);
    buildBrakingBoards(out.scenery, c, pal);
    buildTrees(out.scenery, c, pal, s0, s1);

    return out;
}

} // namespace vsim
