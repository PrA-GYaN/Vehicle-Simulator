#include "rendering/ApexVenue.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "rendering/ApexVenueBuilder.h"

namespace vsim {
namespace apexvenue {
namespace {

/// Static geometry is batched on this grid; one cell of one layer is one
/// draw call and one frustum test.
constexpr float kChunkSize        = 180.0f;
constexpr float kTerrainChunkSize = 640.0f;

/// Box filter, wrapped around the lap. Three passes of radius r approximate
/// a Gaussian of sigma ~ r samples.
void smoothLoop(std::vector<float>& a, int radius, int passes) {
    const int n = static_cast<int>(a.size());
    if (n == 0 || radius <= 0) return;
    std::vector<float> tmp(a.size());
    for (int p = 0; p < passes; ++p) {
        double sum = 0.0;
        for (int d = -radius; d <= radius; ++d) sum += a[static_cast<std::size_t>(((d % n) + n) % n)];
        for (int i = 0; i < n; ++i) {
            tmp[static_cast<std::size_t>(i)] = static_cast<float>(sum / (2 * radius + 1));
            sum += a[static_cast<std::size_t>((i + radius + 1) % n)];
            sum -= a[static_cast<std::size_t>(((i - radius) % n + n) % n)];
        }
        a.swap(tmp);
    }
}

/// Running maximum over +-radius samples, wrapped.
void dilateLoop(std::vector<float>& a, int radius) {
    const int n = static_cast<int>(a.size());
    std::vector<float> out(a.size());
    for (int i = 0; i < n; ++i) {
        float best = a[static_cast<std::size_t>(i)];
        for (int d = -radius; d <= radius; ++d) {
            best = std::max(best, a[static_cast<std::size_t>(((i + d) % n + n) % n)]);
        }
        out[static_cast<std::size_t>(i)] = best;
    }
    a.swap(out);
}

} // namespace

//---------------------------------------------------------------------------
//  Bookkeeping
//---------------------------------------------------------------------------

Builder::Builder(const Track& t) : track(t), lap(t) {
    for (int i = 0; i < lap.n(); i += 3) {
        const vec3& p = lap.f[static_cast<std::size_t>(i)].pos;
        m_outline.push_back({p.x, p.z});
    }
}

MeshData& Builder::mesh(VenueLayer layer, const vec3& where, float minDistance,
                        float maxDistance) {
    const float cell = (layer == VenueLayer::Terrain) ? kTerrainChunkSize : kChunkSize;
    const auto key = std::make_tuple(static_cast<int>(layer),
                                     static_cast<int>(std::floor(where.x / cell)),
                                     static_cast<int>(std::floor(where.z / cell)),
                                     static_cast<int>(minDistance) * 4096 +
                                         static_cast<int>(std::min(maxDistance, 4000.0f)));
    auto it = m_chunkIndex.find(key);
    if (it == m_chunkIndex.end()) {
        VenueChunkData c;
        c.layer = layer;
        c.minDistance = minDistance;
        c.maxDistance = maxDistance;
        m_chunks.push_back(std::move(c));
        it = m_chunkIndex.emplace(key, m_chunks.size() - 1).first;
    }
    return m_chunks[it->second].mesh;
}

VenueInstanceGroupData& Builder::group(const std::string& name) {
    auto it = m_groupIndex.find(name);
    if (it == m_groupIndex.end()) {
        m_groups.emplace_back();
        it = m_groupIndex.emplace(name, m_groups.size() - 1).first;
    }
    return m_groups[it->second];
}

VenueData Builder::finish() {
    VenueData out;
    for (VenueChunkData& c : m_chunks) out.chunks.push_back(std::move(c));
    for (VenueInstanceGroupData& g : m_groups) out.groups.push_back(std::move(g));
    m_chunks.clear();
    m_groups.clear();
    return out;
}

void Builder::place(const std::string& groupName, const vec3& pos, const vec3& facing,
                    const vec3& scale, const vec3& tint) {
    vec3 f(facing.x, 0.0f, facing.z);
    const float len = glm::length(f);
    f = (len > 1e-6f) ? f / len : vec3(1.0f, 0.0f, 0.0f);
    const vec3 r(-f.z, 0.0f, f.x);   // forward x up, as core/Coordinates.h

    InstanceData inst;
    inst.transform = glm::mat4(glm::vec4(f * scale.x, 0.0f),
                               glm::vec4(kUp * scale.y, 0.0f),
                               glm::vec4(r * scale.z, 0.0f),
                               glm::vec4(pos, 1.0f));
    inst.tint = glm::vec4(tint, 1.0f);
    group(groupName).instances.push_back(inst);
}

float Builder::pitLateral(float x) const {
    const auto& pit = track.pitLane();
    if (pit.empty()) return 0.0f;
    const float x0 = static_cast<float>(pit.front().position.x);
    const float x1 = static_cast<float>(pit.back().position.x);
    if (x <= x0) return static_cast<float>(pit.front().position.z);
    if (x >= x1) return static_cast<float>(pit.back().position.z);
    const float t = (x - x0) / (x1 - x0) * static_cast<float>(pit.size() - 1);
    const std::size_t i = std::min(static_cast<std::size_t>(t), pit.size() - 2);
    const float u = t - static_cast<float>(i);
    return static_cast<float>(pit[i].position.z * (1.0 - u) + pit[i + 1].position.z * u);
}

void Builder::reserveLeft(float s0, float s1, float depth) {
    const float span = lap.wrapS(s1 - s0);
    for (int i = 0; i < lap.n(); ++i) {
        const float u = lap.wrapS(lap.f[static_cast<std::size_t>(i)].s - s0);
        if (u <= span) {
            reserve[static_cast<std::size_t>(i)] =
                std::max(reserve[static_cast<std::size_t>(i)],
                         barrier[1][static_cast<std::size_t>(i)] + depth);
        }
    }
}

float Builder::trackDistance(float x, float z, bool* inside, int* nearest) const {
    float best = 1e30f;
    int bestI = 0;
    bool in = false;
    const std::size_t n = m_outline.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const vec2& a = m_outline[i];
        const vec2& b = m_outline[j];
        const float dx = a.x - x, dz = a.y - z;
        const float d = dx * dx + dz * dz;
        if (d < best) { best = d; bestI = static_cast<int>(i); }
        // Crossing-number test against the centreline as a polygon.
        if (((a.y > z) != (b.y > z)) &&
            (x < (b.x - a.x) * (z - a.y) / (b.y - a.y) + a.x)) in = !in;
    }
    if (inside) *inside = in;
    if (nearest) *nearest = std::min(bestI * 3, lap.n() - 1);
    return std::sqrt(best);
}

//---------------------------------------------------------------------------
//  Profile: everything about the cross-section that varies round the lap.
//---------------------------------------------------------------------------

void Builder::buildProfile() {
    const int n = lap.n();
    const std::size_t N = static_cast<std::size_t>(n);

    //-- corners --------------------------------------------------------------
    constexpr float kCornerCurv = 1.0f / 600.0f;
    bool in = false;
    Corner cur;
    for (int i = 0; i < n; ++i) {
        const Frame& f = lap.f[static_cast<std::size_t>(i)];
        const bool bent = std::abs(f.curv) > kCornerCurv;
        if (bent && !in) { in = true; cur = Corner{}; cur.sa = f.s; cur.sign = (f.curv > 0.0f) ? 1.0f : -1.0f; }
        if (!bent && in) { in = false; cur.sb = f.s; corners.push_back(cur); }
    }

    // What each corner is given. Authored per corner, in lap order; if the
    // layout ever changes shape the defaults in Corner apply instead.
    struct Spec { float runoff; int surface; bool brake; bool stand; };
    static const Spec kSpecs[] = {
        {50.0f, kSurfPaved,  true,  true },   // T1
        {20.0f, kSurfGravel, false, false},   // T2
        {28.0f, kSurfGravel, false, false},   // T3
        {30.0f, kSurfGravel, true,  false},   // T4
        {26.0f, kSurfGravel, false, false},   // T5
        {38.0f, kSurfPaved,  true,  true },   // T6 hairpin
        {12.0f, kSurfPaved,  true,  false},   // T7
        {12.0f, kSurfPaved,  false, false},   // T8
        {28.0f, kSurfGravel, false, true },   // T9
        {24.0f, kSurfGravel, true,  false},   // T10
        {26.0f, kSurfPaved,  true,  true },   // T11
        {16.0f, kSurfGravel, false, false},   // T12
        {28.0f, kSurfGravel, true,  false},   // T13
    };
    if (corners.size() == sizeof(kSpecs) / sizeof(kSpecs[0])) {
        for (std::size_t c = 0; c < corners.size(); ++c) {
            corners[c].runoff  = kSpecs[c].runoff;
            corners[c].surface = kSpecs[c].surface;
            corners[c].brake   = kSpecs[c].brake;
            corners[c].stand   = kSpecs[c].stand;
        }
    } else {
        std::cerr << "[venue] expected 13 corners, found " << corners.size()
                  << "; using default runoff everywhere\n";
    }

    //-- runoff and barrier line ---------------------------------------------
    std::vector<float> depth[2];
    for (int sd = 0; sd < 2; ++sd) {
        outside[sd].assign(N, 0.0f);
        surface[sd].assign(N, kSurfGrass);
        kerb[sd].assign(N, 0.0f);
        wall[sd].assign(N, 0);
        depth[sd].assign(N, 0.0f);
    }

    for (const Corner& c : corners) {
        const float L = c.length();
        const int out = sideIndex(-c.sign);
        const int ins = sideIndex(c.sign);
        for (int i = 0; i < n; ++i) {
            const std::size_t I = static_cast<std::size_t>(i);
            const float u = lap.rel(lap.f[I].s, c.sa);
            if (u < -90.0f || u > L + 50.0f) continue;

            // Runoff is deepest straight on from the turn-in point - where a
            // car that has not slowed goes - and tapers away to the exit.
            const float w = smoothstep(-70.0f, 0.2f * L, u) *
                            (1.0f - smoothstep(0.85f * L, L + 35.0f, u));
            if (w > outside[out][I]) {
                outside[out][I] = w;
                if (w > 0.02f) surface[out][I] = c.surface;
            }
            depth[out][I] = std::max(depth[out][I], c.runoff * w);

            // Apex kerb on the inside; entry and exit kerbs on the outside.
            const float apex = smoothstep(0.08f * L - 6.0f, 0.08f * L + 2.0f, u) *
                               (1.0f - smoothstep(0.92f * L - 2.0f, 0.92f * L + 6.0f, u));
            const float entry = smoothstep(-28.0f, -20.0f, u) *
                                (1.0f - smoothstep(0.18f * L, 0.18f * L + 8.0f, u));
            const float exit = smoothstep(0.55f * L, 0.55f * L + 8.0f, u) *
                               (1.0f - smoothstep(L + 22.0f, L + 30.0f, u));
            kerb[ins][I] = std::max(kerb[ins][I], apex);
            kerb[out][I] = std::max(kerb[out][I], std::max(entry, exit));
        }
    }

    for (int sd = 0; sd < 2; ++sd) {
        barrier[sd].resize(N);
        for (std::size_t i = 0; i < N; ++i) barrier[sd][i] = lap.hw + kVergeBase + depth[sd][i];
        smoothLoop(barrier[sd], 8, 2);
    }

    // Two limits on how far a barrier may stand off. On the inside of a
    // corner an offset curve folds over itself at the corner's own radius.
    // And where the lap passes close to itself (the hairpin's two legs),
    // each side may only have its half of the gap. The second test is
    // side-aware: the outside of the hairpin has open ground beyond it and
    // must not be squeezed just because the inside is tight.
    for (int sd = 0; sd < 2; ++sd) {
        const float sgn = (sd == 0) ? 1.0f : -1.0f;
        for (int i = 0; i < n; ++i) {
            const std::size_t I = static_cast<std::size_t>(i);
            const Frame& fi = lap.f[I];
            float cap = 1e9f;
            if (fi.curv * sgn > 1e-5f) cap = 0.8f / std::abs(fi.curv);
            for (int j = 0; j < n; j += 2) {
                const Frame& fj = lap.f[static_cast<std::size_t>(j)];
                if (std::abs(lap.rel(fj.s, fi.s)) < 150.0f) continue;
                const vec3 d = fj.pos - fi.pos;
                if (glm::dot(d, fi.right) * sgn <= 0.0f) continue;
                cap = std::min(cap, 0.5f * glm::length(d) - 0.75f);
            }
            barrier[sd][I] = std::max(lap.hw + 2.5f, std::min(barrier[sd][I], cap));
        }
    }

    //-- main straight: pit side and grandstand side ---------------------------
    onMain.assign(N, 0);
    for (int i = 0; i < n; ++i) {
        const std::size_t I = static_cast<std::size_t>(i);
        const Frame& f = lap.f[I];
        if (f.fwd.x < 0.9999f || std::abs(f.pos.z) > 0.01f) continue;
        onMain[I] = 1;
        const float x = f.pos.x;

        if (x > -300.0f && x < 570.0f) {
            barrier[1][I] = lap.hw + 5.5f;
            wall[1][I] = 1;
        }
        if (x >= -300.0f && x <= 325.0f) {
            kerb[0][I] = 0.0f;
            wall[0][I] = 1;
            surface[0][I] = kSurfPaved;
            outside[0][I] = 0.0f;
            // Along the pit apron the "barrier" is the pit wall; either side
            // of it the barrier steps out to pass round the pit lane.
            barrier[0][I] = (x >= kPitX0 && x <= kPitX1)
                                ? kPitWallZ
                                : std::max(lap.hw + kVergeBase, pitLateral(x) + 4.1f);
        }
    }

    for (int sd = 0; sd < 2; ++sd) {
        for (std::size_t i = 0; i < N; ++i) {
            if (outside[sd][i] > 0.25f) wall[sd][i] = 1;
        }
    }

    reserve.resize(N);
    for (std::size_t i = 0; i < N; ++i) reserve[i] = barrier[1][i] + 2.0f;

    //-- racing line and rubber -----------------------------------------------
    //
    // Not a lap-time optimiser - just the shape every racing line has: out
    // wide on the way in, across to the inside at the apex, wide again on
    // the way out. A narrow blur of the (saturated) curvature says where the
    // apexes are; a wide one says a corner is coming or just went.
    std::vector<float> apex(N), wide(N);
    for (std::size_t i = 0; i < N; ++i) {
        const float k = lap.f[i].curv;
        apex[i] = wide[i] = k / (std::abs(k) + 1.0f / 60.0f);
    }
    smoothLoop(apex, 12, 3);
    smoothLoop(wide, 50, 3);

    line.resize(N);
    rubber.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
        const float a = apex[i];
        const float approach = std::min(std::max(3.5f * wide[i], -1.0f), 1.0f);
        const float raw = 1.25f * a - (1.0f - std::min(std::abs(a) * 1.25f, 1.0f)) * approach;
        line[i] = lap.hw * 0.72f * std::min(std::max(raw, -1.0f), 1.0f);
        rubber[i] = 0.28f + 0.60f * std::min(std::abs(wide[i]) * 2.5f + std::abs(a), 1.0f);
    }
    smoothLoop(line, 6, 2);
}

void Builder::finishLeftSide() {
    // The service road runs behind whatever has been built on the left:
    // take the deepest reservation nearby, stand off it, and smooth the
    // result into something a truck could drive.
    service = reserve;
    dilateLoop(service, 24);
    for (float& v : service) v += 7.0f;
    smoothLoop(service, 18, 3);

    for (int i = 0; i < lap.n(); ++i) {
        const std::size_t I = static_cast<std::size_t>(i);
        // Left-hand corners put the left side on the inside of the bend.
        const float k = lap.f[I].curv;
        if (k < -1e-5f) service[I] = std::min(service[I], 0.85f / std::abs(k));
    }
}

} // namespace apexvenue

//===========================================================================

bool isApexVenueTrack(const Track& track) {
    return track.id() == "apex_gp" && track.closed() && track.centreline().size() > 100;
}

VenueData buildApexVenue(const Track& track) {
    apexvenue::Builder b(track);
    if (b.lap.n() < 100) return {};

    b.buildProfile();

    // Structures first: they claim ground on the left and decide where the
    // barrier is a wall, and both of those shape everything built after.
    b.buildPitComplex();
    b.buildGrandstands();
    b.buildFacilities();
    b.finishLeftSide();

    b.buildRoad();
    b.buildMarkings();
    b.buildKerbs();
    b.buildVerge();
    b.buildServiceRoad();
    b.buildBarriers();
    b.buildTrackside();
    b.buildLandscape();

    return b.finish();
}

} // namespace vsim
