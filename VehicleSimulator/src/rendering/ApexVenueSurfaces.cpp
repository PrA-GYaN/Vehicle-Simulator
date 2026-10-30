//===========================================================================
//  ApexVenueSurfaces.cpp  --  everything flat: racing surface, paint, kerbs,
//  runoff and the service road.
//===========================================================================

#include "rendering/ApexVenueBuilder.h"

#include <functional>

#include "core/Coordinates.h"

namespace vsim::apexvenue {
namespace {

/// A strip in track coordinates from s0 to s1 (either may lie outside
/// [0, length); the strip wraps across the start line). `shape` is asked at
/// each station, with u running 0..1 along the strip, for the two lateral
/// edges and the brush - so a strip can drift, taper and fade.
void strip(Builder& b, VenueLayer layer, float s0, float s1, float y, float maxStep,
           const std::function<void(float u, float& lat0, float& lat1, Brush& br)>& shape) {
    if (s1 <= s0) return;
    const int steps = std::max(1, static_cast<int>(std::ceil((s1 - s0) / maxStep)));
    Frame prev = b.lap.at(s0);
    float pl0 = 0.0f, pl1 = 0.0f;
    Brush pbr;
    shape(0.0f, pl0, pl1, pbr);
    for (int i = 1; i <= steps; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(steps);
        const float s = s0 + (s1 - s0) * u;
        const Frame cur = b.lap.at(s);
        float l0 = 0.0f, l1 = 0.0f;
        Brush br;
        shape(u, l0, l1, br);
        const float sp = s0 + (s1 - s0) * static_cast<float>(i - 1) / static_cast<float>(steps);
        quadV(b.mesh(layer, prev.pos),
              vtx(prev.at(pl0, y), kUp, pbr, {pl0, sp}), vtx(prev.at(pl1, y), kUp, pbr, {pl1, sp}),
              vtx(cur.at(l1, y), kUp, br, {l1, s}),      vtx(cur.at(l0, y), kUp, br, {l0, s}), kUp);
        prev = cur; pl0 = l0; pl1 = l1; pbr = br;
    }
}

/// Constant-width, constant-colour strip.
void paintStrip(Builder& b, float s0, float s1, float lat0, float lat1, const Brush& br,
                float y = kPaintY) {
    strip(b, VenueLayer::Paint, s0, s1, y, 2.0f,
          [&](float, float& l0, float& l1, Brush& out) { l0 = lat0; l1 = lat1; out = br; });
}

/// The pit lane centreline as frames.
std::vector<Frame> pitFrames(const Track& track) {
    std::vector<Frame> out;
    for (const TrackPoint& p : track.pitLane()) {
        Frame f;
        f.pos = vec3(p.position);
        const double yaw = coords::yawFromDirection(p.tangent);
        f.fwd   = vec3(coords::heading(yaw));
        f.right = vec3(coords::right(yaw));
        f.s     = static_cast<float>(p.distance);
        out.push_back(f);
    }
    return out;
}

} // namespace

//---------------------------------------------------------------------------
//  Racing surface
//---------------------------------------------------------------------------

void Builder::buildRoad() {
    static const float kLat[] = {-1.0f, -0.82f, -0.62f, -0.42f, -0.20f, 0.0f,
                                 0.20f, 0.42f, 0.62f, 0.82f, 1.0f};
    constexpr int kBands = static_cast<int>(sizeof(kLat) / sizeof(kLat[0])) - 1;
    const int n = lap.n();

    // One vertex description per (sample, lateral station): colour and the
    // amount of rubber are continuous across the whole surface.
    auto vertexAt = [&](int i, int k, float s) {
        const std::size_t I = static_cast<std::size_t>(lap.wrap(i));
        const Frame& f = lap.f[I];
        const float lat = kLat[k] * lap.hw;
        // Polished and dark on the line, dusty and bleached toward the edge.
        const float edge = std::max(0.0f, (std::abs(kLat[k]) - 0.70f) / 0.30f);
        const float d = (lat - line[I]) / 1.7f;
        Brush br = pal::asphalt.shade(1.0f + 0.12f * edge);
        br.param = rubber[I] * std::exp(-d * d);
        return vtx(f.at(lat, kAsphaltY), kUp, br, {lat, s});
    };

    for (int i = 0; i < n; ++i) {
        const float s0 = lap.f[static_cast<std::size_t>(i)].s;
        const float s1 = (i + 1 < n) ? lap.f[static_cast<std::size_t>(i + 1)].s : lap.length;
        MeshData& m = mesh(VenueLayer::Asphalt, lap.f[static_cast<std::size_t>(i)].pos);
        for (int k = 0; k < kBands; ++k) {
            quadV(m, vertexAt(i, k, s0), vertexAt(i, k + 1, s0),
                  vertexAt(i + 1, k + 1, s1), vertexAt(i + 1, k, s1), kUp);
        }
    }

    // Pit entry and exit roads. Between them the lane runs over the pit
    // apron, which the pit complex paves as one slab. Where the lane is
    // still inside the track's own width it IS the track, so the ribbon is
    // clipped at the track edge rather than laid over it.
    const std::vector<Frame> pit = pitFrames(track);
    const float half = static_cast<float>(track.pitLaneWidth()) * 0.5f;
    for (std::size_t i = 0; i + 1 < pit.size(); ++i) {
        const Frame& a = pit[i];
        const Frame& c = pit[i + 1];
        if (a.pos.x >= kPitX0 - 0.1f && c.pos.x <= kPitX1 + 0.1f) continue;
        const float a0 = std::max(-half, lap.hw - a.pos.z);
        const float c0 = std::max(-half, lap.hw - c.pos.z);
        if (a0 >= half && c0 >= half) continue;
        const Brush br = pal::pitAsphalt.with(0.10f);
        quadV(mesh(VenueLayer::Asphalt, a.pos),
              vtx(a.at(a0, kAsphaltY), kUp, br, {a0, a.s}), vtx(a.at(half, kAsphaltY), kUp, br, {half, a.s}),
              vtx(c.at(half, kAsphaltY), kUp, br, {half, c.s}), vtx(c.at(c0, kAsphaltY), kUp, br, {c0, c.s}), kUp);
    }
}

//---------------------------------------------------------------------------
//  Paint
//---------------------------------------------------------------------------

void Builder::buildMarkings() {
    const int n = lap.n();
    const float hw = lap.hw;

    //-- track limits ---------------------------------------------------------
    for (int i = 0; i < n; ++i) {
        const std::size_t I = static_cast<std::size_t>(i);
        const Frame& a = lap.f[I];
        const Frame& c = lap.f[static_cast<std::size_t>(lap.wrap(i + 1))];
        const float s0 = a.s;
        const float s1 = (i + 1 < n) ? c.s : lap.length;
        MeshData& m = mesh(VenueLayer::Paint, a.pos);

        for (int sd = 0; sd < 2; ++sd) {
            const float sgn = (sd == 0) ? 1.0f : -1.0f;
            // Where the pit lane peels off or rejoins, the edge line breaks
            // into dashes: the line a car crosses to enter or leave.
            if (sd == 0 && onMain[I]) {
                const float lane = pitLateral(a.pos.x);
                const bool opening = lane > 5.6f && lane - 2.5f < hw;
                if (opening && (static_cast<int>(std::floor(a.pos.x / 3.0f)) & 1)) continue;
            }
            const float l0 = sgn * (hw - 0.22f), l1 = sgn * (hw - 0.07f);
            quadV(m, vtx(a.at(l0, kPaintY), kUp, pal::paint, {l0, s0}),
                     vtx(a.at(l1, kPaintY), kUp, pal::paint, {l1, s0}),
                     vtx(c.at(l1, kPaintY), kUp, pal::paint, {l1, s1}),
                     vtx(c.at(l0, kPaintY), kUp, pal::paint, {l0, s1}), kUp);
        }
    }

    //-- start line and grid --------------------------------------------------
    const float lineS = static_cast<float>(track.startLineDistance());
    paintStrip(*this, lineS, lineS + 0.6f, -hw + 0.07f, hw - 0.07f, pal::paint, kPaintY + 0.002f);

    constexpr int kSlots = 22;
    for (int k = 0; k < kSlots; ++k) {
        const float front = lineS - 7.8f - 8.0f * static_cast<float>(k);
        const float lat = ((k % 2) == 0 ? -1.0f : 1.0f) * 3.1f;
        // An open bracket: a bar across the front and two short legs.
        paintStrip(*this, front - 0.12f, front, lat - 1.35f, lat + 1.35f, pal::paint);
        paintStrip(*this, front - 1.8f, front - 0.12f, lat - 1.35f, lat - 1.23f, pal::paint);
        paintStrip(*this, front - 1.8f, front - 0.12f, lat + 1.23f, lat + 1.35f, pal::paint);
        // The yellow centre tick a driver lines the front wheels up on.
        paintStrip(*this, front - 0.9f, front - 0.3f, lat - 0.05f, lat + 0.05f, pal::paintYellow);

        // Launch marks: two stripes of rubber leaving every slot.
        for (float wheel : {-0.78f, 0.78f}) {
            const float fade = 0.22f + 0.3f * hash01(k, 91);
            strip(*this, VenueLayer::Paint, front - 3.6f, front + 14.0f, kPaintY - 0.003f, 2.5f,
                  [&](float u, float& l0, float& l1, Brush& br) {
                      l0 = lat + wheel - 0.17f; l1 = lat + wheel + 0.17f;
                      br = {{0.02f, 0.02f, 0.022f}, venuemat::kDecal, fade * (1.0f - u) * smooth01(u * 8.0f)};
                  });
        }
    }

    //-- skid marks into the heavy braking zones -------------------------------
    for (std::size_t ci = 0; ci < corners.size(); ++ci) {
        const Corner& c = corners[ci];
        if (!c.brake) continue;
        for (int k = 0; k < 7; ++k) {
            const int seed = static_cast<int>(ci) * 31 + k;
            const float start = c.sa + 8.0f - 120.0f * hash01(seed, 1);
            const float len = 16.0f + 42.0f * hash01(seed, 2);
            const float strength = 0.16f + 0.34f * hash01(seed, 3);
            // A locked wheel slides straight on while the line turns in, so
            // marks start near the line and run wide of it.
            const float lat0 = lap.sample(line, start) + (hash01(seed, 4) - 0.5f) * 2.4f;
            const float lat1 = lat0 - c.sign * (0.3f + 1.6f * hash01(seed, 5));
            for (float wheel : {-0.8f, 0.8f}) {
                if (wheel > 0.0f && hash01(seed, 6) < 0.3f) continue;   // one wheel only
                strip(*this, VenueLayer::Paint, start, start + len, kPaintY - 0.003f, 3.0f,
                      [&](float u, float& l0, float& l1, Brush& br) {
                          const float centre = glm::mix(lat0, lat1, u) + wheel;
                          const float lim = hw - 0.6f;
                          const float cl = std::min(std::max(centre, -lim), lim);
                          l0 = cl - 0.13f; l1 = cl + 0.13f;
                          br = {{0.02f, 0.02f, 0.022f}, venuemat::kDecal,
                                strength * smooth01(u * 5.0f) * smooth01((1.0f - u) * 2.5f)};
                      });
            }
        }
    }

    //-- pit lane lines ----------------------------------------------------------
    const std::vector<Frame> pit = pitFrames(track);
    const float half = static_cast<float>(track.pitLaneWidth()) * 0.5f;
    for (std::size_t i = 0; i + 1 < pit.size(); ++i) {
        const Frame& a = pit[i];
        const Frame& c = pit[i + 1];
        MeshData& m = mesh(VenueLayer::Paint, a.pos);
        auto edge = [&](float l0, float l1, const Brush& br) {
            quadV(m, vtx(a.at(l0, kPaintY), kUp, br), vtx(a.at(l1, kPaintY), kUp, br),
                     vtx(c.at(l1, kPaintY), kUp, br), vtx(c.at(l0, kPaintY), kUp, br), kUp);
        };
        edge(half - 0.15f, half, pal::paint);
        if (a.pos.z - half > hw + 0.4f && c.pos.z - half > hw + 0.4f) {
            edge(-half, -half + 0.15f, pal::paint);
        }
    }
}

//---------------------------------------------------------------------------
//  Kerbs
//
//  Walked at a fixed 0.5 m step with colour blocks of three steps, so every
//  colour change lands on a section boundary whatever the centreline
//  spacing is (the same reasoning as TrackMesh's kerbs). The profile is the
//  usual one: a short ramp off the asphalt, a long shallow rise, and a drop
//  down the back.
//---------------------------------------------------------------------------

void Builder::buildKerbs() {
    constexpr float kStep = 0.5f;
    constexpr int   kBlockSteps = 3;
    const int steps = static_cast<int>(lap.length / kStep);

    for (int sd = 0; sd < 2; ++sd) {
        const float sgn = (sd == 0) ? 1.0f : -1.0f;
        Frame a = lap.at(0.0f);
        float amtA = lap.sample(kerb[sd], 0.0f);

        for (int i = 0; i < steps; ++i) {
            const float s0 = static_cast<float>(i) * kStep;
            const float s1 = s0 + kStep;
            const Frame c = lap.at(s1);
            const float amtC = lap.sample(kerb[sd], s1);

            if (amtA > 0.02f || amtC > 0.02f) {
                const bool red = ((i / kBlockSteps) % 2) == 0;
                const Brush top = red ? pal::kerbRed : pal::kerbWhite;
                const Brush back = top.shade(0.70f);

                // (fraction of width, height) of the four profile points.
                auto point = [&](const Frame& f, float amt, int p, float s, const Brush& br) {
                    static const float kAcross[] = {0.0f, 0.20f, 0.82f, 1.0f};
                    static const float kHeight[] = {0.0f, 0.042f, 0.078f, 0.0f};
                    const float lat = sgn * (lap.hw + kAcross[p] * kKerbWidth * amt);
                    const float y = (p == 3) ? kGrassY + 0.002f : kAsphaltY + kHeight[p] * amt;
                    return vtx(f.at(lat, y), kUp, br, {lat, s});
                };

                MeshData& m = mesh(VenueLayer::Kerbs, a.pos);
                for (int p = 0; p < 3; ++p) {
                    const Brush& br = (p == 2) ? back : top;
                    Vertex v0 = point(a, amtA, p, s0, br), v1 = point(a, amtA, p + 1, s0, br);
                    Vertex v2 = point(c, amtC, p + 1, s1, br), v3 = point(c, amtC, p, s1, br);
                    vec3 nrm = glm::cross(v1.position - v0.position, v3.position - v0.position);
                    if (glm::dot(nrm, nrm) < 1e-12f) continue;
                    nrm = glm::normalize(nrm);
                    if (nrm.y < 0.0f) nrm = -nrm;
                    v0.normal = v1.normal = v2.normal = v3.normal = nrm;
                    quadV(m, v0, v1, v2, v3, nrm);
                }
            }
            a = c;
            amtA = amtC;
        }
    }
}

//---------------------------------------------------------------------------
//  Verge and runoff: from the edge of the track to just behind the barrier.
//---------------------------------------------------------------------------

void Builder::buildVerge() {
    const int n = lap.n();
    const float hw = lap.hw;

    for (int i = 0; i < n; ++i) {
        const std::size_t I = static_cast<std::size_t>(i);
        const std::size_t J = static_cast<std::size_t>(lap.wrap(i + 1));
        const Frame& a = lap.f[I];
        const Frame& c = lap.f[J];
        const float s0 = a.s;
        const float s1 = (i + 1 < n) ? c.s : lap.length;
        MeshData& m = mesh(VenueLayer::Verge, a.pos);

        for (int sd = 0; sd < 2; ++sd) {
            const float sgn = (sd == 0) ? 1.0f : -1.0f;

            auto band = [&](float da0, float da1, float dc0, float dc1, const Brush& br) {
                if (da1 - da0 < 0.02f && dc1 - dc0 < 0.02f) return;
                quadV(m, vtx(a.at(sgn * (hw + da0), kGrassY), kUp, br, {da0, s0}),
                         vtx(a.at(sgn * (hw + da1), kGrassY), kUp, br, {da1, s0}),
                         vtx(c.at(sgn * (hw + dc1), kGrassY), kUp, br, {dc1, s1}),
                         vtx(c.at(sgn * (hw + dc0), kGrassY), kUp, br, {dc0, s1}), kUp);
            };

            const float bdA = barrier[sd][I] - hw;
            const float bdC = barrier[sd][J] - hw;
            // A step in the barrier line (either end of the pit wall) is two
            // different cross-sections meeting, not one to interpolate.
            const float endA = bdA + 1.2f;
            const float endC = (std::abs(bdC - bdA) > 2.5f) ? endA : bdC + 1.2f;

            // Pit side of the main straight: paved right across.
            if (sd == 0 && onMain[I] && surface[0][I] == kSurfPaved) {
                const bool apron = a.pos.x >= kPitX0 && a.pos.x <= kPitX1;
                const Brush br = apron ? pal::concrete : pal::runoff;
                band(0.0f, apron ? bdA + 0.35f : endA, 0.0f, apron ? bdA + 0.35f : endC, br);
                continue;
            }

            // Runoff: a strip of grass behind the kerb, then gravel or paved
            // runoff out to a few metres short of the barrier. Its width is
            // the extra room the corner was given, so it tapers to nothing
            // at both ends on its own.
            const int surf = std::max(surface[sd][I], surface[sd][J]);
            const float r0 = 1.6f;
            const float r1A = r0 + std::max(0.0f, bdA - kVergeBase) * 0.92f;
            const float r1C = (endC == endA && bdC != bdA) ? r1A
                              : r0 + std::max(0.0f, bdC - kVergeBase) * 0.92f;
            const Brush grass = pal::grass.with(1.0f);

            if (surf == kSurfGrass || (r1A - r0 < 0.05f && r1C - r0 < 0.05f)) {
                band(0.0f, endA, 0.0f, endC, grass);
            } else {
                const Brush br = (surf == kSurfGravel) ? pal::gravel : pal::runoff;
                band(0.0f, r0, 0.0f, r0, grass);
                band(r0, r1A, r0, r1C, br);
                band(r1A, endA, r1C, endC, grass);
            }
        }
    }
}

//---------------------------------------------------------------------------
//  Service road, round the outside of the whole circuit.
//---------------------------------------------------------------------------

void Builder::buildServiceRoad() {
    const int n = lap.n();
    constexpr float kHalf = 2.2f;

    for (int i = 0; i < n; ++i) {
        const std::size_t I = static_cast<std::size_t>(i);
        const std::size_t J = static_cast<std::size_t>(lap.wrap(i + 1));
        // No road where the ground is too tight for one (the inside of the
        // left-handers); it resumes on the far side.
        if (service[I] - barrier[1][I] < kHalf + 2.2f || service[J] - barrier[1][J] < kHalf + 2.2f) continue;

        const Frame& a = lap.f[I];
        const Frame& c = lap.f[J];
        const float s0 = a.s;
        const float s1 = (i + 1 < n) ? c.s : lap.length;
        const float la = -service[I], lc = -service[J];
        const float y = kGrassY + 0.004f;
        MeshData& m = mesh(VenueLayer::Verge, a.at(la, 0.0f));
        quadV(m, vtx(a.at(la - kHalf, y), kUp, pal::service, {-kHalf, s0}),
                 vtx(a.at(la + kHalf, y), kUp, pal::service, {kHalf, s0}),
                 vtx(c.at(lc + kHalf, y), kUp, pal::service, {kHalf, s1}),
                 vtx(c.at(lc - kHalf, y), kUp, pal::service, {-kHalf, s1}), kUp);
    }
}

} // namespace vsim::apexvenue
