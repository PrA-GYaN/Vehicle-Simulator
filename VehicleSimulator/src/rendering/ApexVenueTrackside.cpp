//===========================================================================
//  ApexVenueTrackside.cpp  --  what lines the track: barriers, catch
//  fencing, tyre walls, marshal posts, boards, the start gantry and bridges.
//===========================================================================

#include "rendering/ApexVenueBuilder.h"

namespace vsim::apexvenue {
namespace {

/// Board on a pole, facing oncoming traffic, at `lateral` from the centreline.
void signBoard(Builder& b, const Frame& f, float lateral, const std::string& text,
               const Brush& panel, const Brush& ink, float width, float height,
               float poleHeight, float px) {
    MeshData& m = b.mesh(VenueLayer::Solid, f.pos);
    const Basis at{f.at(lateral, 0.0f), f.fwd, f.right};
    box(m, at, -0.04f, 0.04f, 0.0f, poleHeight, -0.04f, 0.04f, pal::steelDark);
    box(m, at, -0.03f, 0.03f, poleHeight, poleHeight + height, -0.5f * width, 0.5f * width, panel);
    label(m, at.at(-0.03f, poleHeight + 0.5f * height, 0.0f), -f.fwd, text, px, ink);
}

} // namespace

//---------------------------------------------------------------------------
//  Barriers
//
//  One pass round the lap per side, in 4 m panels - the length a W-beam rail
//  or a precast wall unit really comes in. Each panel is either:
//
//    ARMCO   two W-beam rails on posts. Straights and corner insides.
//    WALL    concrete wall with catch fencing above. Wherever there are
//            people behind it (pits, grandstands) or cars arrive fast and
//            straight (the outside of a corner) - and there it also gets a
//            tyre wall on its face.
//
//  Posts, fence posts and tyre stacks are instanced.
//---------------------------------------------------------------------------

void Builder::buildBarriers() {
    //-- instanced parts ------------------------------------------------------
    {
        VenueInstanceGroupData& g = group("armco_post");
        box(g.mesh, Basis{}, -0.05f, 0.05f, 0.0f, 0.88f, 0.0f, 0.12f, pal::steelDark);
        g.maxDistance = 220.0f;
        g.castsShadow = false;
    }
    {
        // Local +z points at the track: the top of the post cranks inward,
        // which is what stops debris clearing the fence.
        VenueInstanceGroupData& g = group("fence_post");
        box(g.mesh, Basis{}, -0.05f, 0.05f, 0.0f, 3.1f, -0.05f, 0.05f, pal::steel);
        const vec3 c[8] = {
            {-0.05f, 3.05f, -0.05f}, {-0.05f, 3.05f, 0.05f}, {0.05f, 3.05f, 0.05f}, {0.05f, 3.05f, -0.05f},
            {-0.05f, 3.60f, 0.41f},  {-0.05f, 3.60f, 0.51f}, {0.05f, 3.60f, 0.51f}, {0.05f, 3.60f, 0.41f},
        };
        hexa(g.mesh, c, pal::steel);
        g.maxDistance = 320.0f;
        g.castsShadow = false;
    }
    {
        // White rubber, tinted per stack: mostly black, with painted runs.
        VenueInstanceGroupData& g = group("tyre_stack");
        cylinder(g.mesh, vec3(0.0f), kUp * 1.02f, 0.34f, 8, Brush{vec3(1.0f), venuemat::kRubber});
        g.maxDistance = 340.0f;
        g.castsShadow = false;
    }

    const int panels = std::max(1, static_cast<int>(std::round(lap.length / 4.0f)));
    const float step = lap.length / static_cast<float>(panels);

    // W-beam section: (height from the rail centre, stand-off toward the track).
    static const float kBeam[5][2] = {
        {-0.155f, 0.0f}, {-0.085f, 0.075f}, {0.0f, 0.02f}, {0.085f, 0.075f}, {0.155f, 0.0f}};

    for (int sd = 0; sd < 2; ++sd) {
        const float sgn = (sd == 0) ? 1.0f : -1.0f;

        for (int p = 0; p < panels; ++p) {
            const float s0 = static_cast<float>(p) * step;
            const float s1 = s0 + step;
            const Frame a = lap.at(s0);
            const Frame c = lap.at(s1);
            const float ba = barrierAt(sgn, s0);
            const float bc = barrierAt(sgn, s1);
            if (std::abs(ba - bc) > 2.5f) continue;   // the two ends of the pit wall

            float u = 0.0f;
            const std::size_t I = static_cast<std::size_t>(lap.locate(s0, u));
            const bool isWall = wall[sd][I] != 0;
            const vec3 toTrack = -sgn * a.right;
            const vec3 facing = -sgn * a.fwd;        // instance +z -> toward the track
            MeshData& m = mesh(VenueLayer::Solid, a.at(sgn * ba, 0.0f));

            auto pt = [&](const Frame& f, float bLat, float inward, float y) {
                return f.at(sgn * (bLat - inward), y);
            };

            if (isWall) {
                const float H = kWallHeight;
                // Face (battered), top, back.
                quad(m, pt(a, ba, 0.14f, 0.0f), pt(c, bc, 0.14f, 0.0f),
                        pt(c, bc, 0.0f, H),     pt(a, ba, 0.0f, H), toTrack, pal::concrete);
                quad(m, pt(a, ba, 0.0f, H),   pt(c, bc, 0.0f, H),
                        pt(c, bc, -0.22f, H), pt(a, ba, -0.22f, H), kUp, pal::concrete.shade(1.06f));
                quad(m, pt(a, ba, -0.22f, H),    pt(c, bc, -0.22f, H),
                        pt(c, bc, -0.30f, 0.0f), pt(a, ba, -0.30f, 0.0f), -toTrack, pal::concrete.shade(0.9f));

                // Catch fence: upright to 3.1 m, then cranked in over the track.
                MeshData& t = mesh(VenueLayer::Transparent, a.at(sgn * ba, 0.0f));
                quad(t, pt(a, ba, -0.11f, H), pt(c, bc, -0.11f, H),
                        pt(c, bc, -0.11f, 3.1f), pt(a, ba, -0.11f, 3.1f), toTrack, pal::fence,
                     {s0, H}, {s1, H}, {s1, 3.1f}, {s0, 3.1f});
                quad(t, pt(a, ba, -0.11f, 3.1f), pt(c, bc, -0.11f, 3.1f),
                        pt(c, bc, 0.35f, kFenceHeight), pt(a, ba, 0.35f, kFenceHeight), toTrack, pal::fence,
                     {s0, 3.1f}, {s1, 3.1f}, {s1, 3.75f}, {s0, 3.75f});
                // Top cable rail, so the fence has an edge against the sky.
                quad(m, pt(a, ba, 0.33f, kFenceHeight - 0.03f), pt(c, bc, 0.33f, kFenceHeight - 0.03f),
                        pt(c, bc, 0.37f, kFenceHeight + 0.03f), pt(a, ba, 0.37f, kFenceHeight + 0.03f),
                     toTrack, pal::steelDark);
                place("fence_post", pt(a, ba, -0.11f, 0.0f), facing);

                // Tyre wall on the face, two rows deep, where cars arrive.
                if (lap.sample(outside[sd], s0) > 0.5f) {
                    const int stacks = std::max(1, static_cast<int>(step / 0.70f));
                    for (int k = 0; k < stacks; ++k) {
                        const float us = (static_cast<float>(k) + 0.5f) / static_cast<float>(stacks);
                        const Frame f = lap.at(s0 + step * us);
                        const float bl = glm::mix(ba, bc, us);
                        const int run = static_cast<int>((s0 + step * us) / 2.8f);
                        for (int row = 0; row < 2; ++row) {
                            vec3 tint(0.075f + 0.03f * hash01(p * 7 + k, row));
                            // The front row is painted in runs, red and white.
                            if (row == 1 && (run % 3) == 0) {
                                tint = ((run / 3) % 2) ? vec3(0.80f, 0.80f, 0.78f) : vec3(0.70f, 0.10f, 0.08f);
                            }
                            place("tyre_stack",
                                  f.at(sgn * (bl - 0.52f - 0.66f * static_cast<float>(row)), 0.0f),
                                  f.fwd, vec3(1.0f), tint);
                        }
                    }
                }
            } else {
                for (float railY : {0.37f, 0.71f}) {
                    for (int k = 0; k < 4; ++k) {
                        const float y0 = railY + kBeam[k][0], y1 = railY + kBeam[k + 1][0];
                        const float o0 = kBeam[k][1], o1 = kBeam[k + 1][1];
                        quad(m, pt(a, ba, o0, y0), pt(c, bc, o0, y0), pt(c, bc, o1, y1), pt(a, ba, o1, y1),
                             toTrack + kUp * ((o1 > o0) ? -0.5f : 0.5f), pal::steel);
                    }
                    // Flat back, so the rail does not vanish seen from behind.
                    quad(m, pt(a, ba, 0.0f, railY - 0.155f), pt(c, bc, 0.0f, railY - 0.155f),
                            pt(c, bc, 0.0f, railY + 0.155f), pt(a, ba, 0.0f, railY + 0.155f),
                         -toTrack, pal::steel.shade(0.8f));
                }
                place("armco_post", pt(a, ba, -0.01f, 0.0f), -facing);
            }
        }
    }
}

//---------------------------------------------------------------------------
//  Trackside furniture
//---------------------------------------------------------------------------

void Builder::buildTrackside() {
    const float hw = lap.hw;

    //-- advertising on the walls, on the straights ----------------------------
    {
        int count = 0;
        for (float s = 2.0f; s + 8.0f < lap.length; s += 8.0f) {
            float u = 0.0f;
            const std::size_t I = static_cast<std::size_t>(lap.locate(s, u));
            const Frame a = lap.at(s);
            const Frame c = lap.at(s + 7.0f);
            if (std::abs(a.curv) > 1.0f / 300.0f || std::abs(c.curv) > 1.0f / 300.0f) continue;
            for (int sd = 0; sd < 2; ++sd) {
                if (!wall[sd][I]) continue;
                const float sgn = (sd == 0) ? 1.0f : -1.0f;
                const float ba = barrierAt(sgn, s), bc = barrierAt(sgn, s + 7.0f);
                if (std::abs(ba - bc) > 0.6f) continue;
                const vec3 pa = a.at(sgn * (ba - 0.16f), 0.0f);
                const vec3 pc = c.at(sgn * (bc - 0.16f), 0.0f);
                const vec3 toTrack = -sgn * a.right;

                const int which = (count++) % (kTeamCount + 2);
                const bool house = which >= kTeamCount;
                const Brush bg{house ? vec3(0.88f, 0.88f, 0.86f) : kTeams[which].color, venuemat::kPanel};
                const float lum = bg.color.r * 0.3f + bg.color.g * 0.6f + bg.color.b * 0.1f;
                const Brush ink = (lum > 0.5f) ? pal::charcoal : pal::white;

                MeshData& m = mesh(VenueLayer::Solid, pa);
                quad(m, pa + kUp * 0.10f, pc + kUp * 0.10f, pc + kUp * 0.98f, pa + kUp * 0.98f, toTrack, bg);
                label(m, 0.5f * (pa + pc) + kUp * 0.54f, toTrack,
                      house ? "APEX GP" : kTeams[which].name, 0.085f, ink);
            }
        }
    }

    //-- distance boards and corner numbers -------------------------------------
    for (std::size_t ci = 0; ci < corners.size(); ++ci) {
        const Corner& c = corners[ci];
        const float outSgn = -c.sign;

        if (c.brake) {
            for (int metres : {150, 100, 50}) {
                const Frame f = lap.at(c.sa - static_cast<float>(metres));
                signBoard(*this, f, outSgn * (hw + 2.8f), std::to_string(metres),
                          pal::white, pal::black, 1.25f, 0.75f, 0.55f, 0.062f);
            }
        }
        // Corner number, just before turn-in, in front of the outside barrier.
        const float s = c.sa - 12.0f;
        const Frame f = lap.at(s);
        const float lat = std::max(hw + 3.6f, barrierAt(outSgn, s) - 1.4f);
        signBoard(*this, f, outSgn * lat, std::to_string(ci + 1),
                  pal::charcoal, pal::white, 0.95f, 0.95f, 1.9f, 0.085f);
    }

    //-- marshal posts ------------------------------------------------------------
    {
        int k = 0;
        for (float s = 95.0f; s < lap.length - 60.0f; s += 205.0f, ++k) {
            const Frame f = lap.at(s);
            float u = 0.0f;
            const std::size_t I = static_cast<std::size_t>(lap.locate(s, u));
            // Posts go on the outside of the venue except every third one;
            // never on the pit side of the main straight.
            float sgn = ((k % 3) == 2) ? 1.0f : -1.0f;
            if (onMain[I] && f.pos.x > -310.0f && f.pos.x < 340.0f) sgn = -1.0f;
            const int sd = sideIndex(sgn);
            if (wall[sd][I] && onMain[I]) continue;     // grandstand frontage
            const float bl = barrier[sd][I];

            // x along the track, z away from it.
            const Basis at{f.at(sgn * bl, 0.0f), f.fwd, sgn * f.right};
            MeshData& m = mesh(VenueLayer::Solid, at.o);
            box(m, at, -1.7f, 1.7f, 0.0f, 0.22f, 0.5f, 3.0f, pal::concrete);
            for (float px : {-1.6f, 1.6f}) {
                for (float pz : {0.6f, 2.9f}) box(m, at, px - 0.04f, px + 0.04f, 0.22f, 2.5f, pz - 0.04f, pz + 0.04f, pal::steelDark);
            }
            box(m, at, -1.8f, 1.8f, 2.5f, 2.6f, 0.4f, 3.1f, pal::orange);
            box(m, at, -1.6f, 1.6f, 0.22f, 2.5f, 2.86f, 2.92f, pal::offWhite);
            box(m, at, -1.6f, 1.6f, 0.22f, 1.15f, 0.58f, 0.62f, pal::orange);
            box(m, at, 1.2f, 1.45f, 0.22f, 0.80f, 2.5f, 2.75f, pal::red);          // extinguisher
            // Light panel toward oncoming traffic, showing green.
            box(m, at, -1.86f, -1.80f, 1.55f, 2.15f, 0.7f, 1.3f, pal::black);
            box(m, at, -1.88f, -1.86f, 1.62f, 2.08f, 0.77f, 1.23f,
                Brush{{0.10f, 0.85f, 0.25f}, venuemat::kLit, 2.2f});
            // Service gate through the barrier beside the post.
            if (!wall[sd][I]) box(m, at, -4.6f, -2.0f, 0.18f, 0.98f, -0.10f, 0.03f, pal::orange);
        }
    }

    //-- start gantry ---------------------------------------------------------------
    {
        const Frame f = lap.at(static_cast<float>(track.startLineDistance()));
        const Basis at{f.pos, f.fwd, f.right};
        MeshData& m = mesh(VenueLayer::Solid, f.pos);
        const float zl = -(hw + 2.0f), zr = kPitWallZ + 0.75f;
        for (float z : {zl, zr}) {
            box(m, at, -0.30f, 0.30f, 0.0f, 7.2f, z - 0.30f, z + 0.30f, pal::charcoal);
            box(m, at, -0.45f, 0.45f, 0.0f, 0.5f, z - 0.45f, z + 0.45f, pal::concrete);
        }
        box(m, at, -0.40f, 0.40f, 7.0f, 8.15f, zl - 0.6f, zr + 0.6f, pal::charcoal);
        box(m, at, -0.44f, 0.44f, 8.15f, 8.25f, zl - 0.7f, zr + 0.7f, pal::red);
        const vec3 mid = at.at(0.0f, 7.58f, 0.5f * (zl + zr));
        label(m, mid - f.fwd * 0.40f, -f.fwd, "APEX GRAND PRIX", 0.105f, pal::white);
        label(m, mid + f.fwd * 0.40f,  f.fwd, "APEX GRAND PRIX", 0.105f, pal::white);
        // Five start-light pods, each two lamps high. Dim: the race is not on.
        for (int k = 0; k < 5; ++k) {
            const float z = -4.4f + 2.2f * static_cast<float>(k);
            box(m, at, -0.52f, -0.40f, 6.05f, 6.95f, z - 0.42f, z + 0.42f, pal::black);
            for (float y : {6.18f, 6.58f}) {
                box(m, at, -0.54f, -0.52f, y, y + 0.26f, z - 0.30f, z - 0.04f,
                    Brush{{0.55f, 0.04f, 0.03f}, venuemat::kLit, 0.25f});
                box(m, at, -0.54f, -0.52f, y, y + 0.26f, z + 0.04f, z + 0.30f,
                    Brush{{0.55f, 0.04f, 0.03f}, venuemat::kLit, 0.25f});
            }
        }
    }

    //-- footbridges ------------------------------------------------------------------
    auto bridge = [&](float s, const std::string& text, const Brush& colour) {
        const Frame f = lap.at(s);
        const Basis at{f.pos, f.fwd, f.right};
        MeshData& m = mesh(VenueLayer::Solid, f.pos);
        const float zr = barrierAt(1.0f, s) + 2.2f;
        const float zl = -(barrierAt(-1.0f, s) + 2.2f);
        for (float z : {zl, zr}) {
            // Stair tower with a dark doorway.
            box(m, at, -1.9f, 1.9f, 0.0f, 8.3f, z - 1.6f, z + 1.6f, pal::concrete);
            const float face = (z < 0.0f) ? z + 1.62f : z - 1.62f;
            box(m, at, -0.6f, 0.6f, 0.0f, 2.2f, face - 0.02f, face + 0.02f, pal::black);
        }
        box(m, at, -1.7f, 1.7f, 5.7f, 6.05f, zl, zr, pal::concreteDark);
        box(m, at, -1.7f, 1.7f, 8.0f, 8.2f, zl, zr, pal::offWhite);
        for (float x : {-1.72f, 1.62f}) {
            box(m, at, x, x + 0.10f, 6.05f, 7.45f, zl + 1.6f, zr - 1.6f, colour);
        }
        const float lum = colour.color.r * 0.3f + colour.color.g * 0.6f + colour.color.b * 0.1f;
        const Brush ink = (lum > 0.5f) ? pal::charcoal : pal::white;
        const vec3 mid = at.at(0.0f, 6.75f, 0.5f * (zl + zr));
        label(m, mid - f.fwd * 1.73f, -f.fwd, text, 0.15f, ink);
        label(m, mid + f.fwd * 1.73f,  f.fwd, text, 0.15f, ink);
    };
    if (corners.size() >= 10) {
        bridge(0.5f * (corners[5].sb + corners[6].sa), "KESTREL",
               Brush{kTeams[4].color, venuemat::kPanel});
        bridge(0.5f * (corners[2].sb + corners[3].sa), "NORDIK",
               Brush{kTeams[1].color, venuemat::kPanel});
        bridge(0.5f * (corners[8].sb + corners[9].sa), "APEX GP", pal::white);
    }

    //-- cones marking the pit entry -----------------------------------------------------
    for (int k = 0; k < 7; ++k) {
        const float x = -262.0f + 5.0f * static_cast<float>(k);
        const float z = std::max(hw + 0.5f, pitLateral(x) - 3.1f);
        MeshData& m = mesh(VenueLayer::Solid, {x, 0.0f, z});
        const Basis at{{x, 0.0f, z}, {1, 0, 0}, {0, 0, 1}};
        box(m, at, -0.19f, 0.19f, 0.0f, 0.04f, -0.19f, 0.19f, pal::orange);
        cone(m, {x, 0.04f, z}, 0.14f, 0.62f, 8, pal::orange);
    }
}

} // namespace vsim::apexvenue
