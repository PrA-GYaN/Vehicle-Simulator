//===========================================================================
//  ApexVenueLandscape.cpp  --  the ground out to the horizon, the trees on
//  it and the fence round the venue.
//
//  The terrain is a height field over a square several kilometres across.
//  It is exactly flat (y = 0) across the infield and for a few hundred
//  metres outside the track - the physics runs on flat ground here, and
//  every building assumes it - then rolls, and finally rises into a ring of
//  hills. Those hills are what hides the edge of the world: the camera's far
//  plane is 2.5 km out and they stand inside it on every side.
//===========================================================================

#include "rendering/ApexVenueBuilder.h"

namespace vsim::apexvenue {
namespace {

constexpr float kFlatRadius  = 300.0f;   ///< from the track, outside the lap
constexpr float kHillsRadius = 720.0f;   ///< rolling ground is at full height by here
const vec2 kVenueCentre{270.0f, 470.0f}; ///< roughly the middle of the layout

/// Smooth pseudo-noise from a few incommensurate sines. Not noise in any
/// rigorous sense - just something continuous that does not visibly repeat
/// over a few kilometres.
float waves(float x, float z) {
    return 0.50f * std::sin(x * 0.0061f + 1.3f) * std::cos(z * 0.0052f + 0.4f) +
           0.30f * std::sin(x * 0.0130f + z * 0.0090f + 2.1f) +
           0.20f * std::sin(x * 0.0270f - z * 0.0210f + 0.7f);
}

/// Low-polygon ellipsoid, smooth-shaded: a broadleaf crown.
void blob(MeshData& m, const vec3& centre, float rx, float ry, int segments, int rings,
          const Brush& br) {
    auto point = [&](int i, int j, vec3& n) {
        const float a = 2.0f * kPiF * static_cast<float>(i) / static_cast<float>(segments);
        const float b = kPiF * static_cast<float>(j) / static_cast<float>(rings);
        n = vec3(std::sin(b) * std::cos(a), std::cos(b), std::sin(b) * std::sin(a));
        return centre + vec3(n.x * rx, n.y * ry, n.z * rx);
    };
    for (int j = 0; j < rings; ++j) {
        for (int i = 0; i < segments; ++i) {
            vec3 n0, n1, n2, n3;
            const vec3 p0 = point(i, j, n0), p1 = point(i + 1, j, n1);
            const vec3 p2 = point(i + 1, j + 1, n2), p3 = point(i, j + 1, n3);
            // Darker underneath: the cheapest ambient occlusion there is.
            auto v = [&](const vec3& p, const vec3& n) { return vtx(p, n, br.shade(0.72f + 0.28f * (n.y * 0.5f + 0.5f))); };
            quadV(m, v(p0, n0), v(p1, n1), v(p2, n2), v(p3, n3), n0 + n1 + n2 + n3);
        }
    }
}

bool inKeepout(const std::vector<Keepout>& keepouts, float x, float z) {
    for (const Keepout& k : keepouts) {
        const vec3 d(x - k.centre.x, 0.0f, z - k.centre.z);
        const vec3 across(-k.axis.z, 0.0f, k.axis.x);
        if (std::abs(glm::dot(d, k.axis)) < k.halfLength &&
            std::abs(glm::dot(d, across)) < k.halfWidth) return true;
    }
    return false;
}

} // namespace

float Builder::groundHeight(float x, float z) const {
    bool inside = false;
    const float d = trackDistance(x, z, &inside);
    float h = 0.0f;
    if (!inside) {
        const float t = smoothstep(kFlatRadius, kHillsRadius, d);
        h += t * (9.0f + 16.0f * waves(x, z));
    }
    // The ring of hills.
    const float dx = x - kVenueCentre.x, dz = z - kVenueCentre.y;
    const float r = std::sqrt(dx * dx + dz * dz);
    const float ang = std::atan2(dz, dx);
    const float rise = smoothstep(1150.0f, 2050.0f, r);
    if (rise > 0.0f) {
        h += rise * rise * (150.0f + 70.0f * std::sin(ang * 3.0f + 1.0f) +
                            42.0f * std::sin(ang * 7.0f + 2.1f) + 24.0f * std::sin(ang * 13.0f + 0.3f) +
                            40.0f * waves(x * 1.7f, z * 1.7f));
    }
    return h;
}

void Builder::buildLandscape() {
    //-- terrain --------------------------------------------------------------------
    {
        constexpr float kCell = 28.0f;
        constexpr float kHalf = 2380.0f;
        const int cells = static_cast<int>(2.0f * kHalf / kCell);
        const int verts = cells + 1;

        std::vector<float> height(static_cast<std::size_t>(verts) * verts);
        std::vector<float> dist(height.size());
        auto idx = [&](int i, int j) { return static_cast<std::size_t>(j) * verts + i; };
        auto px = [&](int i) { return kVenueCentre.x - kHalf + kCell * static_cast<float>(i); };
        auto pz = [&](int j) { return kVenueCentre.y - kHalf + kCell * static_cast<float>(j); };

        for (int j = 0; j < verts; ++j) {
            for (int i = 0; i < verts; ++i) {
                height[idx(i, j)] = groundHeight(px(i), pz(j));
                dist[idx(i, j)] = trackDistance(px(i), pz(j));
            }
        }

        auto vertex = [&](int i, int j) {
            const float h = height[idx(i, j)];
            const int i0 = std::max(i - 1, 0), i1 = std::min(i + 1, verts - 1);
            const int j0 = std::max(j - 1, 0), j1 = std::min(j + 1, verts - 1);
            const vec3 n = glm::normalize(vec3(
                (height[idx(i0, j)] - height[idx(i1, j)]) / (kCell * static_cast<float>(i1 - i0)), 1.0f,
                (height[idx(i, j0)] - height[idx(i, j1)]) / (kCell * static_cast<float>(j1 - j0))));

            // Mown grass near the circuit, rougher and darker where the woods
            // are, and a greyer green up on the hills.
            const float x = px(i), z = pz(j);
            const float wood = smoothstep(260.0f, 520.0f, dist[idx(i, j)]) *
                               smoothstep(-0.25f, 0.35f, waves(x * 2.3f + 400.0f, z * 2.3f));
            vec3 c = glm::mix(pal::grass.color, vec3(0.150f, 0.270f, 0.125f), wood * 0.85f);
            c = glm::mix(c, vec3(0.270f, 0.330f, 0.250f), smoothstep(60.0f, 230.0f, h));
            c *= 0.94f + 0.12f * hash01(i, j);
            return vtx({x, h, z}, n, Brush{c, venuemat::kGrass});
        };

        for (int j = 0; j < cells; ++j) {
            for (int i = 0; i < cells; ++i) {
                MeshData& m = mesh(VenueLayer::Terrain, {px(i), 0.0f, pz(j)});
                quadV(m, vertex(i, j), vertex(i + 1, j), vertex(i + 1, j + 1), vertex(i, j + 1), kUp);
            }
        }
    }

    //-- trees -------------------------------------------------------------------------
    //
    // Two species, each in four forms:
    //
    //   near   a bark-textured trunk and a crown of leaf CARDS - flat quads
    //          carrying a cut-out photograph of a spray of leaves, at many
    //          angles, which is how a tree is drawn in any game that has to
    //          draw a forest of them;
    //   far    a solid low-polygon crown, textured with the same leaves,
    //          and beyond that a single shape;
    //   shadow the solid crown again, drawn only into the shadow maps, since
    //          a card cannot cast the shadow of the leaves printed on it.
    //
    // Woodland is laid out by a density field, not scattered evenly - and
    // nothing grows inside the barriers, on the service road or through a
    // building.
    {
        const Brush bark{{0.36f, 0.31f, 0.26f}, venuemat::kBark};
        const Brush pine{{0.105f, 0.235f, 0.120f}, venuemat::kFoliage};
        const Brush leaf{{0.185f, 0.330f, 0.115f}, venuemat::kFoliage};
        constexpr float kNear = 110.0f;   // leaf cards inside this
        constexpr float kMid  = 300.0f;   // full solid crown inside this, one shape beyond

        // One leaf card: a square of side 2*half about `c`, spanned by two
        // unit vectors. Its normals point away from the crown's middle
        // rather than off the card, so the whole crown shades as one mass.
        auto card = [](MeshData& m, const vec3& c, const vec3& u, const vec3& v, float half,
                       const vec3& crown, const vec3& colour, int turn) {
            const vec3 corner[4] = {c - u * half - v * half, c + u * half - v * half,
                                    c + u * half + v * half, c - u * half + v * half};
            static const vec2 kUv[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
            const gl::GLuint base = static_cast<gl::GLuint>(m.vertices.size());
            for (int k = 0; k < 4; ++k) {
                const vec3 n = glm::normalize(corner[k] - crown + kUp * 0.6f);
                // Cards deep in the crown are shaded darker: light does not
                // reach the inside of a tree.
                const float depth = std::min(glm::length(corner[k] - crown) / 2.6f, 1.0f);
                m.vertices.push_back(vtx(corner[k], n, Brush{colour * (0.62f + 0.38f * depth), venuemat::kLeafCard},
                                         kUv[(k + turn) % 4]));
            }
            m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        };
        auto unit = [](int seed) {
            const float a = hash01(seed, 1) * 2.0f * kPiF;
            const float y = hash01(seed, 2) * 2.0f - 1.0f;
            const float r = std::sqrt(std::max(0.0f, 1.0f - y * y));
            return vec3(r * std::cos(a), y, r * std::sin(a));
        };

        //-- broadleaf --
        VenueInstanceGroupData& bn = group("broadleaf_near");
        cylinder(bn.mesh, vec3(0.0f), kUp * 3.4f, 0.21f, 7, bark, false);
        {
            // Two limbs, so the crown is visibly held up by something.
            const vec3 l0[2] = {{0.0f, 2.3f, 0.0f}, {0.0f, 2.7f, 0.0f}};
            const vec3 l1[2] = {{1.3f, 4.2f, 0.5f}, {-1.1f, 4.5f, -0.7f}};
            for (int k = 0; k < 2; ++k) cylinder(bn.mesh, l0[k], l1[k] - l0[k], 0.09f, 5, bark, false);
        }
        fixColour(bn.mesh, 0);
        {
            const vec3 crown{0.0f, 4.7f, 0.0f};
            for (int k = 0; k < 18; ++k) {
                const vec3 dir = unit(k * 13 + 5);
                const vec3 c = crown + vec3(dir.x * 1.55f, dir.y * 1.15f, dir.z * 1.55f) * (0.35f + 0.65f * hash01(k, 31));
                vec3 u = glm::normalize(glm::cross(unit(k * 7 + 90), dir) + vec3(1e-3f));
                const vec3 v = glm::normalize(glm::cross(dir, u));
                card(bn.mesh, c, u, v, 1.25f + 0.55f * hash01(k, 33), crown,
                     leaf.color * (0.85f + 0.35f * hash01(k, 35)), k);
            }
        }
        bn.maxDistance = kNear;
        bn.castsShadow = false;
        bn.twoSided = true;

        VenueInstanceGroupData& bf = group("broadleaf_far");
        cylinder(bf.mesh, vec3(0.0f), kUp * 2.8f, 0.20f, 5, bark, false);
        fixColour(bf.mesh, 0);
        blob(bf.mesh, {0.0f, 4.6f, 0.0f}, 2.7f, 2.3f, 7, 4, leaf);
        blob(bf.mesh, {1.0f, 3.6f, 0.6f}, 1.7f, 1.4f, 6, 3, leaf.shade(0.9f));
        blob(bf.mesh, {-0.9f, 3.9f, -0.8f}, 1.6f, 1.4f, 6, 3, leaf.shade(1.08f));
        bf.minDistance = kNear;
        bf.maxDistance = kMid;
        bf.castsShadow = false;

        VenueInstanceGroupData& bd = group("broadleaf_distant");
        blob(bd.mesh, {0.0f, 4.3f, 0.0f}, 2.8f, 2.8f, 5, 3, leaf);
        bd.minDistance = kMid;
        bd.maxDistance = 1500.0f;
        bd.castsShadow = false;

        VenueInstanceGroupData& bs = group("broadleaf_shadow");
        blob(bs.mesh, {0.0f, 4.6f, 0.0f}, 2.6f, 2.2f, 6, 3, leaf);
        cylinder(bs.mesh, vec3(0.0f), kUp * 3.0f, 0.21f, 5, bark, false);
        bs.maxDistance = 240.0f;
        bs.shadowOnly = true;

        //-- conifer --
        // Solid tiers, as the far version, with a ring of leaf cards lying
        // just proud of each one: the cones give it mass, the cards give it
        // a broken, feathery outline instead of a geometric one.
        VenueInstanceGroupData& cn = group("conifer_near");
        cylinder(cn.mesh, vec3(0.0f), kUp * 2.2f, 0.16f, 6, bark, false);
        fixColour(cn.mesh, 0);
        {
            const vec3 crown{0.0f, 3.8f, 0.0f};
            static const float kBase[3] = {1.3f, 3.2f, 5.0f};
            static const float kRadius[3] = {2.05f, 1.55f, 1.00f};
            static const float kHeight[3] = {3.6f, 3.2f, 2.9f};
            for (int tier = 0; tier < 3; ++tier) {
                cone(cn.mesh, {0, kBase[tier], 0}, kRadius[tier] * 0.86f, kHeight[tier], 8,
                     pine.shade(0.74f + 0.14f * static_cast<float>(tier)));
                for (int k = 0; k < 6; ++k) {
                    const float a = 2.0f * kPiF * (static_cast<float>(k) + 0.5f * static_cast<float>(tier)) / 6.0f;
                    const vec3 out(std::cos(a), 0.0f, std::sin(a));
                    const vec3 across(-out.z, 0.0f, out.x);
                    // Down the slope of the tier, a third of the way up it.
                    const vec3 slope = glm::normalize(out * kRadius[tier] - kUp * kHeight[tier]);
                    const vec3 c = vec3(0.0f, kBase[tier] + kHeight[tier] * 0.30f, 0.0f) +
                                   out * (kRadius[tier] * 0.74f);
                    card(cn.mesh, c, across, slope, kRadius[tier] * 0.72f, crown,
                         pine.color * (0.95f + 0.35f * hash01(tier * 9 + k, 41)), k + tier);
                }
            }
        }
        cn.maxDistance = kNear;
        cn.castsShadow = false;
        cn.twoSided = true;

        VenueInstanceGroupData& cf = group("conifer_far");
        cone(cf.mesh, {0, 1.3f, 0}, 2.05f, 3.6f, 7, pine.shade(0.82f));
        cone(cf.mesh, {0, 3.2f, 0}, 1.55f, 3.2f, 7, pine.shade(0.95f));
        cone(cf.mesh, {0, 5.0f, 0}, 1.00f, 2.9f, 6, pine.shade(1.10f));
        cf.minDistance = kNear;
        cf.maxDistance = kMid;
        cf.castsShadow = false;

        VenueInstanceGroupData& cd = group("conifer_distant");
        cone(cd.mesh, {0, 1.0f, 0}, 1.95f, 6.9f, 5, pine);
        cd.minDistance = kMid;
        cd.maxDistance = 1500.0f;
        cd.castsShadow = false;

        VenueInstanceGroupData& cs = group("conifer_shadow");
        cone(cs.mesh, {0, 1.3f, 0}, 2.0f, 6.4f, 6, pine);
        // A cone is open underneath; close it, or the light sees through.
        for (int k = 0; k < 6; ++k) {
            const float a0 = 2.0f * kPiF * static_cast<float>(k) / 6.0f;
            const float a1 = 2.0f * kPiF * static_cast<float>(k + 1) / 6.0f;
            tri(cs.mesh, {0, 1.3f, 0}, vec3(std::cos(a0) * 2.0f, 1.3f, std::sin(a0) * 2.0f),
                vec3(std::cos(a1) * 2.0f, 1.3f, std::sin(a1) * 2.0f), -kUp, pine);
        }
        cs.maxDistance = 240.0f;
        cs.shadowOnly = true;

        vec2 lo(1e30f), hi(-1e30f);
        for (const Frame& f : lap.f) {
            lo = glm::min(lo, vec2(f.pos.x, f.pos.z));
            hi = glm::max(hi, vec2(f.pos.x, f.pos.z));
        }
        constexpr float kReach = 560.0f;
        constexpr float kPitch = 12.5f;
        int id = 0;
        for (float gz = lo.y - kReach; gz < hi.y + kReach; gz += kPitch) {
            for (float gx = lo.x - kReach; gx < hi.x + kReach; gx += kPitch, ++id) {
                const float x = gx + (hash01(id, 1) - 0.5f) * kPitch * 0.9f;
                const float z = gz + (hash01(id, 2) - 0.5f) * kPitch * 0.9f;

                bool inside = false;
                int near = 0;
                const float d = trackDistance(x, z, &inside, &near);
                if (d > kReach) continue;
                const std::size_t N = static_cast<std::size_t>(near);

                // Clear of the track's own furniture, on whichever side.
                const float clear = inside ? barrier[0][N] + 9.0f
                                           : std::max(service[N] + 11.0f, barrier[1][N] + 12.0f);
                if (d < clear) continue;
                if (inKeepout(keepouts, x, z)) continue;

                // Density: open parkland in the infield and close to the
                // circuit, closing up into woodland further out.
                const float field = waves(x * 2.3f + 400.0f, z * 2.3f);
                const float want = inside ? smoothstep(0.25f, 0.75f, field) * 0.30f
                                          : 0.10f + 0.80f * smoothstep(-0.25f, 0.35f, field) *
                                                        smoothstep(clear, clear + 90.0f, d + 40.0f);
                if (hash01(id, 3) > want) continue;

                const bool conifer = hash01(id / 7, 4) * 0.6f + hash01(id, 5) * 0.4f > (inside ? 0.62f : 0.45f);
                const float size = 0.85f + 0.75f * hash01(id, 6);
                const float tone = 0.80f + 0.40f * hash01(id, 7);
                const vec3 tint(tone * (0.92f + 0.16f * hash01(id, 8)), tone, tone * (0.90f + 0.2f * hash01(id, 9)));
                const vec3 pos(x, groundHeight(x, z) - 0.15f, z);
                const float ang = hash01(id, 10) * 2.0f * kPiF;
                const vec3 facing(std::cos(ang), 0.0f, std::sin(ang));
                const vec3 scale(size, size * (0.9f + 0.3f * hash01(id, 11)), size);
                place(conifer ? "conifer_near" : "broadleaf_near", pos, facing, scale, tint);
                place(conifer ? "conifer_far" : "broadleaf_far", pos, facing, scale, tint);
                place(conifer ? "conifer_distant" : "broadleaf_distant", pos, facing, scale, tint);
                place(conifer ? "conifer_shadow" : "broadleaf_shadow", pos, facing, scale, tint);
            }
        }
    }

    //-- perimeter fence, outside the service road ----------------------------------------
    {
        VenueInstanceGroupData& g = group("perimeter_post");
        box(g.mesh, Basis{}, -0.04f, 0.04f, 0.0f, 2.5f, -0.04f, 0.04f, pal::steelDark);
        g.maxDistance = 200.0f;
        g.castsShadow = false;

        const int panels = static_cast<int>(lap.length / 6.0f);
        const float step = lap.length / static_cast<float>(panels);
        for (int p = 0; p < panels; ++p) {
            const float s0 = static_cast<float>(p) * step, s1 = s0 + step;
            const Frame a = lap.at(s0), c = lap.at(s1);
            const float la = -(lap.sample(service, s0) + 5.5f);
            const float lc = -(lap.sample(service, s1) + 5.5f);
            // No fence where the service road itself was squeezed out.
            if (-la - barrierAt(-1.0f, s0) < 9.0f) continue;
            const vec3 pa = a.at(la, 0.0f), pc = c.at(lc, 0.0f);
            quad(mesh(VenueLayer::Transparent, pa), pa, pc, pc + kUp * 2.4f, pa + kUp * 2.4f,
                 a.right, pal::fence, {s0, 0.0f}, {s1, 0.0f}, {s1, 2.4f}, {s0, 2.4f});
            place("perimeter_post", pa, a.fwd);
        }
    }
}

} // namespace vsim::apexvenue
