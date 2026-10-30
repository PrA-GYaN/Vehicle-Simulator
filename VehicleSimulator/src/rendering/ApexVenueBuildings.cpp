//===========================================================================
//  ApexVenueBuildings.cpp  --  the pit complex, grandstands and the rest of
//  the facility: paddock, race control, car parks, background buildings.
//
//  The main straight is the world X axis with the infield on +Z (see
//  ApexGrandPrix.cpp), so everything along it is authored directly in world
//  coordinates. Stands at the corners are placed from the lap instead.
//===========================================================================

#include "rendering/ApexVenueBuilder.h"

namespace vsim::apexvenue {
namespace {

const Basis kWorld{};

Brush inkFor(const vec3& background) {
    const float lum = background.r * 0.3f + background.g * 0.6f + background.b * 0.1f;
    return (lum > 0.5f) ? pal::charcoal : pal::white;
}

/// A plain modern block: cladding, a band of glazing per floor on all four
/// faces, a parapet and some plant on the roof. `b.o` is one ground corner.
void block(MeshData& m, const Basis& b, float w, float d, float h, int floors, const Brush& body) {
    box(m, b, 0.0f, w, 0.0f, h, 0.0f, d, body);
    const float fh = h / static_cast<float>(floors);
    for (int f = 0; f < floors; ++f) {
        const float y0 = static_cast<float>(f) * fh + fh * 0.28f;
        const float y1 = static_cast<float>(f) * fh + fh * 0.86f;
        box(m, b, 0.6f, w - 0.6f, y0, y1, -0.06f, 0.0f, pal::glass);
        box(m, b, 0.6f, w - 0.6f, y0, y1, d, d + 0.06f, pal::glass);
        box(m, b, -0.06f, 0.0f, y0, y1, 0.6f, d - 0.6f, pal::glass);
        box(m, b, w, w + 0.06f, y0, y1, 0.6f, d - 0.6f, pal::glass);
    }
    box(m, b, -0.2f, w + 0.2f, h, h + 0.5f, -0.2f, d + 0.2f, pal::charcoal);
    box(m, b, w * 0.2f, w * 0.45f, h + 0.5f, h + 2.4f, d * 0.3f, d * 0.7f, pal::steel);
}

/// A rectangular pad of paving, cut into pieces so it chunks and shades well.
void pad(Builder& b, VenueLayer layer, float x0, float x1, float z0, float z1, float y,
         const Brush& br, float piece = 20.0f) {
    const int nx = std::max(1, static_cast<int>(std::ceil((x1 - x0) / piece)));
    const int nz = std::max(1, static_cast<int>(std::ceil((z1 - z0) / piece)));
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < nz; ++j) {
            const float xa = x0 + (x1 - x0) * static_cast<float>(i) / static_cast<float>(nx);
            const float xb = x0 + (x1 - x0) * static_cast<float>(i + 1) / static_cast<float>(nx);
            const float za = z0 + (z1 - z0) * static_cast<float>(j) / static_cast<float>(nz);
            const float zb = z0 + (z1 - z0) * static_cast<float>(j + 1) / static_cast<float>(nz);
            quad(b.mesh(layer, {xa, 0.0f, za}), {xa, y, za}, {xb, y, za}, {xb, y, zb}, {xa, y, zb},
                 kUp, br, {za, xa}, {za, xb}, {zb, xb}, {zb, xa});
        }
    }
}

} // namespace

//---------------------------------------------------------------------------
//  Pit complex
//---------------------------------------------------------------------------

void Builder::buildPitComplex() {
    const float bz = kPitBuildingZ;
    const float back = bz + kPitBuildingDepth;

    //-- ground ----------------------------------------------------------------
    pad(*this, VenueLayer::Asphalt, kPitX0, kPitX1, kPitWallZ + 0.35f, bz, kAsphaltY,
        pal::pitAsphalt.with(0.06f), 13.0f);
    pad(*this, VenueLayer::Asphalt, kPitX0, kPitX1, back, 96.0f, kAsphaltY, pal::pitAsphalt.shade(1.08f));
    pad(*this, VenueLayer::Asphalt, kTowerX1, kPitX1, bz, back, kAsphaltY, pal::pitAsphalt.shade(1.08f));
    keepouts.push_back({{-5.0f, 0.0f, 58.0f}, {1, 0, 0}, 215.0f, 52.0f});

    //-- paint ---------------------------------------------------------------------
    {
        const float y = kPaintY;
        // Speed-limit lines across the lane at each end of the limited zone,
        // and the limit itself painted on the surface.
        for (float x : {kPitX0 + 14.0f, kPitX1 - 0.5f}) {
            quad(mesh(VenueLayer::Paint, {x, 0, 13.0f}), {x, y, kPitLaneZ0}, {x + 0.3f, y, kPitLaneZ0},
                 {x + 0.3f, y, kPitLaneZ1}, {x, y, kPitLaneZ1}, kUp, pal::paint);
        }
        for (float x : {kPitX0 + 22.0f, -55.0f, 70.0f}) {
            textQuads(mesh(VenueLayer::Paint, {x, 0, 13.0f}), {x, y, 13.0f - 1.2f},
                      {0, 0, 1}, {1, 0, 0}, kUp, "60", 0.22f, 0.5f, pal::paint);
        }
        // Pit boxes in the working lane, one per garage, with the team's
        // colour along the garage side.
        for (int g = 0; g < kGarageCount; ++g) {
            const float xc = kGarageX0 + (static_cast<float>(g) + 0.5f) * kGaragePitch;
            MeshData& m = mesh(VenueLayer::Paint, {xc, 0, 17.0f});
            const float x0 = xc - 3.1f, x1 = xc + 3.1f, z0 = kPitLaneZ1 + 0.35f, z1 = 19.3f;
            auto bar = [&](float xa, float xb, float za, float zb, const Brush& br) {
                quad(m, {xa, y, za}, {xb, y, za}, {xb, y, zb}, {xa, y, zb}, kUp, br);
            };
            bar(x0, x1, z0, z0 + 0.1f, pal::paint);
            bar(x0, x0 + 0.1f, z0, z1, pal::paint);
            bar(x1 - 0.1f, x1, z0, z1, pal::paint);
            bar(x0, x1, z1, z1 + 0.35f, Brush{kTeams[(g / 3) % kTeamCount].color, venuemat::kPaint});
        }
    }

    MeshData* cur = nullptr;
    auto M = [&](float x, float z) -> MeshData& { cur = &mesh(VenueLayer::Solid, {x, 0.0f, z}); return *cur; };

    //-- pit wall gantries: where a team watches its cars go by ---------------------
    for (int g = 1; g < kGarageCount; g += 2) {
        const float xc = kGarageX0 + static_cast<float>(g) * kGaragePitch;
        MeshData& m = M(xc, 9.5f);
        const float z0 = kPitWallZ + 0.35f, z1 = z0 + 1.3f;
        for (float x : {xc - 1.7f, xc + 1.7f}) {
            box(m, kWorld, x - 0.04f, x + 0.04f, 0.0f, 2.45f, z1 - 0.08f, z1, pal::steelDark);
        }
        box(m, kWorld, xc - 1.9f, xc + 1.9f, 2.45f, 2.55f, z0 - 0.5f, z1 + 0.15f,
            Brush{kTeams[(g / 3) % kTeamCount].color, venuemat::kPanel});
        box(m, kWorld, xc - 1.6f, xc + 1.6f, 0.0f, 0.95f, z1 - 0.06f, z1 - 0.02f, pal::charcoal);
    }

    //-- garages -------------------------------------------------------------------
    for (int g = 0; g <= kGarageCount; ++g) {
        const float x0 = kGarageX0 + static_cast<float>(g) * kGaragePitch;
        MeshData& m = M(x0, bz);
        box(m, kWorld, x0 - 0.3f, x0 + 0.3f, 0.0f, 4.3f, bz - 0.5f, bz + 0.3f, pal::white);
        if (g == kGarageCount) break;

        const Team& team = kTeams[(g / 3) % kTeamCount];
        const float xa = x0 + 0.3f, xb = x0 + kGaragePitch - 0.3f, xc = x0 + 0.5f * kGaragePitch;
        const bool open = hash01(g, 5) > 0.3f;
        const float zd = bz + 0.28f;
        const vec3 front{0, 0, -1};
        if (open) {
            // An open garage is a dark room with a strip of light at the back.
            quad(m, {xa, 0, zd}, {xb, 0, zd}, {xb, 3.7f, zd}, {xa, 3.7f, zd}, front,
                 Brush{{0.035f, 0.036f, 0.04f}, venuemat::kPanel});
            quad(m, {xa + 0.5f, 2.9f, zd - 0.01f}, {xb - 0.5f, 2.9f, zd - 0.01f},
                 {xb - 0.5f, 3.1f, zd - 0.01f}, {xa + 0.5f, 3.1f, zd - 0.01f}, front,
                 Brush{{0.9f, 0.9f, 0.85f}, venuemat::kLit, 0.9f});
            quad(m, {xa + 0.4f, 0.0f, zd - 0.01f}, {xb - 0.4f, 0.0f, zd - 0.01f},
                 {xb - 0.4f, 1.1f, zd - 0.01f}, {xa + 0.4f, 1.1f, zd - 0.01f}, front,
                 Brush{team.color * 0.35f, venuemat::kPanel});
        } else {
            // Roller shutter: alternate slats, so it reads as ribbed.
            for (int r = 0; r < 12; ++r) {
                const float y0 = 3.7f * static_cast<float>(r) / 12.0f;
                const float y1 = 3.7f * static_cast<float>(r + 1) / 12.0f;
                quad(m, {xa, y0, zd}, {xb, y0, zd}, {xb, y1, zd}, {xa, y1, zd}, front,
                     pal::steel.shade((r % 2) ? 0.78f : 0.92f));
            }
        }
        // Team board over the door.
        const Brush board{team.color, venuemat::kPanel};
        box(m, kWorld, xa, xb, 3.7f, 4.3f, bz - 0.12f, bz + 0.3f, board);
        label(m, {xc, 4.0f, bz - 0.12f}, front, team.name, 0.058f, inkFor(team.color));
    }

    //-- pit building ------------------------------------------------------------------
    {
        const float x0 = kBuildingX0, x1 = kBuildingX1;
        const float half = 0.5f * (x0 + x1);
        // Built in two halves so each lands in its own chunk.
        for (int h = 0; h < 2; ++h) {
            const float xa = h ? half : x0, xb = h ? x1 : half;
            MeshData& m = M(0.5f * (xa + xb), bz + 8.0f);
            box(m, kWorld, xa, xb, 0.0f, 4.3f, bz + 0.3f, back, pal::offWhite);          // ground floor
            box(m, kWorld, xa, xb, 4.3f, 8.6f, bz, back, pal::white);                    // first floor
            box(m, kWorld, xa + (h ? 0.0f : 0.8f), xb - (h ? 0.8f : 0.0f), 5.0f, 8.0f,
                bz - 0.07f, bz, pal::glass);                                            // glazing, front
            box(m, kWorld, xa + (h ? 0.0f : 0.8f), xb - (h ? 0.8f : 0.0f), 5.0f, 8.0f,
                back, back + 0.07f, pal::glass);                                        // and back
            for (float x = xa + 3.75f; x < xb - 1.0f; x += 3.75f) {
                box(m, kWorld, x - 0.05f, x + 0.05f, 5.0f, 8.0f, bz - 0.13f, bz - 0.07f, pal::steelDark);
            }
            // Balcony over the garage doors, glass balustrade and a handrail.
            box(m, kWorld, xa, xb, 4.3f, 4.5f, bz - 1.7f, bz, pal::concrete);
            box(m, kWorld, xa, xb, 4.5f, 5.5f, bz - 1.7f, bz - 1.66f, pal::glass);
            box(m, kWorld, xa, xb, 5.5f, 5.56f, bz - 1.74f, bz - 1.62f, pal::steel);
            // Roof, oversailing the pit lane side.
            box(m, kWorld, xa - (h ? 0.0f : 1.5f), xb + (h ? 1.5f : 0.0f), 8.6f, 8.95f,
                bz - 2.6f, back + 1.2f, pal::offWhite, &pal::roofing);
            box(m, kWorld, xa, xb, 8.95f, 9.95f, bz - 2.5f, bz - 2.46f, pal::glass);     // terrace
            // Back of the ground floor: service doors to the paddock.
            for (float x = xa + 6.0f; x < xb - 4.0f; x += 15.0f) {
                box(m, kWorld, x, x + 3.2f, 0.0f, 3.2f, back, back + 0.05f, pal::steel.shade(0.8f));
            }
            // Roof plant.
            for (float x = xa + 12.0f; x < xb - 8.0f; x += 28.0f) {
                box(m, kWorld, x, x + 5.0f, 8.95f, 10.6f, back - 6.5f, back - 2.5f, pal::steel);
            }
        }
        // End bays either side of the garages.
        MeshData& m = M(half, bz);
        box(m, kWorld, x0, kGarageX0 - 0.3f, 0.0f, 4.3f, bz - 0.1f, bz + 0.3f, pal::offWhite);
        box(m, kWorld, kGarageX0 + kGaragePitch * kGarageCount + 0.3f, x1, 0.0f, 4.3f, bz - 0.1f, bz + 0.3f, pal::offWhite);

        // The circuit's name on the roof, to the pits and to the paddock.
        box(m, kWorld, -64.0f, -16.0f, 8.95f, 11.6f, bz - 2.3f, bz - 2.0f, pal::charcoal);
        label(m, {-40.0f, 10.3f, bz - 2.3f}, {0, 0, -1}, "APEX GRAND PRIX CIRCUIT", 0.26f, pal::white);
        label(m, {-40.0f, 10.3f, bz - 2.0f}, {0, 0, 1}, "APEX GRAND PRIX CIRCUIT", 0.26f, pal::white);

        // Big screen on the roof, facing the main grandstand.
        box(m, kWorld, 30.0f, 48.0f, 8.95f, 10.4f, bz + 1.5f, bz + 2.1f, pal::steelDark);
        box(m, kWorld, 28.0f, 50.0f, 10.4f, 20.0f, bz + 1.3f, bz + 2.3f, pal::black);
        box(m, kWorld, 28.6f, 49.4f, 11.0f, 19.4f, bz + 1.22f, bz + 1.3f,
            Brush{{0.05f, 0.11f, 0.22f}, venuemat::kLit, 1.6f});
        label(m, {39.0f, 15.2f, bz + 1.22f}, {0, 0, -1}, "APEX GP", 0.36f,
              Brush{{1.0f, 1.0f, 1.0f}, venuemat::kLit, 2.5f});
    }

    //-- race control tower ----------------------------------------------------------------
    {
        MeshData& m = M(0.5f * (kBuildingX1 + kTowerX1), bz + 9.0f);
        const float xa = kBuildingX1, xb = kTowerX1, za = bz, zb = bz + 19.0f;
        box(m, kWorld, xa, xb, 0.0f, 22.0f, za, zb, pal::charcoal);
        for (int f = 0; f < 5; ++f) {
            const float y0 = static_cast<float>(f) * 4.4f + 1.0f, y1 = y0 + 2.9f;
            box(m, kWorld, xa + 0.8f, xb - 0.8f, y0, y1, za - 0.07f, za, pal::glass);
            box(m, kWorld, xa + 0.8f, xb - 0.8f, y0, y1, zb, zb + 0.07f, pal::glass);
            box(m, kWorld, xb, xb + 0.07f, y0, y1, za + 0.8f, zb - 0.8f, pal::glass);
            if (y0 > 9.0f) box(m, kWorld, xa - 0.07f, xa, y0, y1, za + 0.8f, zb - 0.8f, pal::glass);
        }
        // Control room: a glazed floor oversailing the tower on every side.
        box(m, kWorld, xa - 3.0f, xb + 3.0f, 22.0f, 22.5f, za - 3.0f, zb + 3.0f, pal::white);
        box(m, kWorld, xa - 2.6f, xb + 2.6f, 22.5f, 26.0f, za - 2.6f, zb + 2.6f, pal::glass);
        box(m, kWorld, xa - 3.4f, xb + 3.4f, 26.0f, 26.6f, za - 3.4f, zb + 3.4f, pal::white);
        box(m, kWorld, xa - 2.0f, xb + 2.0f, 26.6f, 28.3f, za - 3.3f, za - 3.1f, pal::charcoal);
        label(m, {0.5f * (xa + xb), 27.45f, za - 3.3f}, {0, 0, -1}, "RACE CONTROL", 0.19f, pal::white);
        cylinder(m, {xa + 6.0f, 26.6f, zb - 4.0f}, kUp * 9.0f, 0.09f, 6, pal::steel);
        cylinder(m, {xb - 5.0f, 26.6f, zb - 6.0f}, kUp * 5.5f, 0.07f, 6, pal::steel);
        box(m, kWorld, xb - 8.0f, xb - 5.5f, 26.6f, 28.4f, za + 4.0f, za + 6.5f, pal::steel);
    }

    //-- paddock ------------------------------------------------------------------------------
    {
        // Transporters, parked nose-out behind the garages they serve.
        VenueInstanceGroupData& g = group("transporter");
        box(g.mesh, Basis{}, -7.0f, 6.4f, 1.05f, 4.0f, -1.27f, 1.27f, Brush{vec3(1.0f), venuemat::kPanel});
        box(g.mesh, Basis{}, 6.7f, 9.0f, 0.55f, 3.3f, -1.22f, 1.22f, Brush{vec3(1.0f), venuemat::kPanel});
        const std::size_t fixed = g.mesh.vertices.size();
        box(g.mesh, Basis{}, -6.6f, 8.6f, 0.0f, 1.05f, -1.1f, 1.1f, Brush{{0.06f, 0.06f, 0.065f}, venuemat::kRubber});
        box(g.mesh, Basis{}, 9.0f, 9.05f, 1.9f, 3.0f, -1.05f, 1.05f, pal::glass);
        fixColour(g.mesh, fixed);
        g.maxDistance = 520.0f;

        for (int k = 0; k < kGarageCount / 2; ++k) {
            const float x = kGarageX0 + (2.0f * static_cast<float>(k) + 1.0f) * kGaragePitch;
            const vec3 c = kTeams[((2 * k) / 3) % kTeamCount].color;
            place("transporter", {x - 2.2f, 0.0f, back + 15.0f}, {0, 0, 1}, vec3(1.0f),
                  glm::mix(c, vec3(0.85f), 0.35f));
            if (hash01(k, 77) > 0.4f) {
                place("transporter", {x + 2.4f, 0.0f, back + 15.0f}, {0, 0, 1}, vec3(1.0f),
                      glm::mix(c, vec3(0.9f), 0.75f));
            }
        }

        // Hospitality units along the far side.
        for (int k = 0; k < 10; ++k) {
            const float xa = -188.0f + 37.0f * static_cast<float>(k);
            const Team& team = kTeams[(k * 5 + 2) % kTeamCount];
            MeshData& m = M(xa + 12.0f, 82.0f);
            const float za = 76.0f, zb = 88.0f;
            box(m, kWorld, xa, xa + 24.0f, 0.0f, 7.2f, za, zb, pal::white);
            box(m, kWorld, xa + 0.6f, xa + 23.4f, 0.4f, 3.2f, za - 0.06f, za, pal::glass);
            box(m, kWorld, xa + 0.6f, xa + 23.4f, 4.0f, 6.6f, za - 0.06f, za, pal::glass);
            box(m, kWorld, xa - 0.3f, xa + 24.3f, 7.2f, 7.5f, za - 2.2f, zb + 0.3f, pal::charcoal);
            const Brush board{team.color, venuemat::kPanel};
            box(m, kWorld, xa, xa + 24.0f, 3.25f, 3.95f, za - 0.12f, za, board);
            label(m, {xa + 12.0f, 3.6f, za - 0.12f}, {0, 0, -1}, team.name, 0.07f, inkFor(team.color));
        }

        // Boundary wall round the paddock and the two ends of the pit apron.
        MeshData& m = M(0.0f, 96.0f);
        const Brush w = pal::offWhite;
        box(M(-100.0f, 96.0f), kWorld, kPitX0, -5.0f, 0.0f, 2.3f, 96.0f, 96.25f, w);
        box(M(100.0f, 96.0f), kWorld, 5.0f, kPitX1, 0.0f, 2.3f, 96.0f, 96.25f, w);     // gate between
        box(M(kPitX0, 60.0f), kWorld, kPitX0 - 0.25f, kPitX0, 0.0f, 2.3f, 17.4f, 96.25f, w);
        box(M(kPitX1, 60.0f), kWorld, kPitX1, kPitX1 + 0.25f, 0.0f, 2.3f, 17.4f, 96.25f, w);
        (void)m;

        // Floodlight masts.
        for (float x : {-185.0f, -65.0f, 55.0f, 175.0f}) {
            MeshData& mm = M(x, 98.0f);
            cylinder(mm, {x, 0.0f, 98.0f}, kUp * 30.0f, 0.32f, 8, pal::steel);
            box(mm, Basis{{x, 0.0f, 98.0f}}, -2.6f, 2.6f, 29.6f, 31.6f, -0.9f, -0.5f, pal::steelDark);
            box(mm, Basis{{x, 0.0f, 98.0f}}, -2.4f, 2.4f, 29.8f, 31.4f, -0.94f, -0.9f,
                Brush{{0.9f, 0.9f, 0.85f}, venuemat::kLit, 0.5f});
        }
    }
}

//---------------------------------------------------------------------------
//  Grandstand
//
//  `b.o` is the front corner on the ground, b.x runs along the stand and b.z
//  away from the track. The seating is one riser and one tread per row,
//  drawn with the crowd material: the shader fills a hashed share of the
//  seats with people, so the stand reads as occupied from any distance at
//  the cost of two quads a row. Close up, instanced figures sit in more of
//  the seats.
//---------------------------------------------------------------------------

void Builder::grandstand(const Basis& b, float length, int rows, bool roofed,
                         const std::string& name, const vec3& seatColor) {
    constexpr float kTread = 0.85f, kRise = 0.43f, kFrontH = 2.4f, kFrontGap = 1.3f;
    constexpr float kAislePitch = 14.0f, kAisleHalf = 0.65f, kSeat = 0.5f;

    MeshData& m = mesh(VenueLayer::Solid, b.at(0.5f * length, 0.0f, 8.0f));
    const vec3 toTrack = -b.z;
    const int seed = static_cast<int>(std::hash<std::string>{}(name) % 9973);
    const float seatOffset = static_cast<float>(seed % 500);

    const float depth = kFrontGap + static_cast<float>(rows) * kTread;
    const float topY = kFrontH + static_cast<float>(rows) * kRise;

    //-- base and front ----------------------------------------------------------
    box(m, b, 0.0f, length, 0.0f, kFrontH, 0.0f, 0.3f, pal::concrete);
    box(m, b, 0.0f, length, kFrontH - 0.2f, kFrontH, 0.3f, kFrontGap, pal::concrete);
    box(m, b, 0.0f, length, kFrontH, kFrontH + 1.05f, 0.0f, 0.04f, pal::glass);
    box(m, b, 0.0f, length, kFrontH + 1.05f, kFrontH + 1.11f, -0.03f, 0.07f, pal::steel);
    {
        const Brush band{seatColor, venuemat::kPanel};
        const vec3 c = b.at(0.5f * length, 1.3f, 0.0f) + toTrack * 0.02f;
        const float half = std::min(0.5f * length - 1.0f, 0.5f * textDots(name) * 0.16f + 1.5f);
        quad(m, c - b.x * half - kUp * 0.75f, c + b.x * half - kUp * 0.75f,
                c + b.x * half + kUp * 0.75f, c - b.x * half + kUp * 0.75f, toTrack, band);
        label(m, c, toTrack, name, 0.16f, inkFor(seatColor));
    }

    //-- seating --------------------------------------------------------------------
    {
        VenueInstanceGroupData& g = group("spectator");
        if (g.mesh.vertices.empty()) {
            // Seated figure facing local +x: torso, lap, head.
            box(g.mesh, Basis{}, -0.13f, 0.13f, 0.0f, 0.56f, -0.21f, 0.21f, Brush{vec3(1.0f)});
            const std::size_t fixed = g.mesh.vertices.size();
            box(g.mesh, Basis{}, 0.13f, 0.46f, 0.0f, 0.15f, -0.19f, 0.19f, Brush{{0.12f, 0.13f, 0.18f}});
            box(g.mesh, Basis{}, -0.09f, 0.10f, 0.60f, 0.83f, -0.09f, 0.09f, Brush{{0.78f, 0.60f, 0.48f}});
            fixColour(g.mesh, fixed);
            g.maxDistance = 150.0f;
            g.castsShadow = false;
        }
    }

    const Brush seats{seatColor, venuemat::kCrowd, 0.50f};
    const Brush step = pal::concrete.shade(0.95f);
    const int aisles = std::max(0, static_cast<int>(length / kAislePitch));

    for (int r = 0; r < rows; ++r) {
        const float z0 = kFrontGap + static_cast<float>(r) * kTread;
        const float y0 = kFrontH + static_cast<float>(r) * kRise;
        const float rf = static_cast<float>(r);

        auto run = [&](float xa, float xb, const Brush& br) {
            if (xb - xa < 0.05f) return;
            const float ua = xa / kSeat + seatOffset, ub = xb / kSeat + seatOffset;
            quad(m, b.at(xa, y0, z0), b.at(xb, y0, z0), b.at(xb, y0 + kRise, z0), b.at(xa, y0 + kRise, z0),
                 toTrack, br, {ua, rf + 0.08f}, {ub, rf + 0.08f}, {ub, rf + 0.5f}, {ua, rf + 0.5f});
            quad(m, b.at(xa, y0 + kRise, z0), b.at(xb, y0 + kRise, z0),
                    b.at(xb, y0 + kRise, z0 + kTread), b.at(xa, y0 + kRise, z0 + kTread),
                 kUp, br, {ua, rf + 0.5f}, {ub, rf + 0.5f}, {ub, rf + 0.92f}, {ua, rf + 0.92f});
        };

        float x = 0.0f;
        for (int a = 0; a <= aisles; ++a) {
            const float aisleX = static_cast<float>(a + 1) * kAislePitch;
            const bool last = (a == aisles) || (aisleX + kAisleHalf >= length);
            const float xb = last ? length : aisleX - kAisleHalf;
            // Seats come in whole numbers: snap the block to the seat grid so
            // the shader's seat cells and the instanced figures agree.
            const float xa = std::ceil(x / kSeat) * kSeat;
            const float xe = std::floor(xb / kSeat) * kSeat;
            run(x, xa, step);
            run(xa, xe, seats);
            run(xe, xb, step);

            for (float sx = xa; sx + kSeat <= xe + 1e-3f; sx += kSeat) {
                const int id = static_cast<int>(sx * 2.0f) + seed * 131;
                if (hash01(id, r) > 0.30f) continue;
                const float h1 = hash01(id, r + 1000), h2 = hash01(id, r + 2000);
                vec3 cloth(0.15f + 0.75f * h1, 0.15f + 0.6f * h2, 0.18f + 0.7f * hash01(id, r + 3000));
                if (h2 > 0.7f) cloth = vec3(0.85f, 0.85f, 0.83f);
                if (h1 > 0.85f) cloth = seatColor * 1.3f + vec3(0.1f);
                place("spectator", b.at(sx + 0.5f * kSeat, y0 + kRise, z0 + 0.48f), toTrack,
                      vec3(1.0f, 0.88f + 0.24f * h1, 1.0f), cloth);
            }
            if (last) break;
            run(xb, aisleX + kAisleHalf, step);
            x = aisleX + kAisleHalf;
        }
    }

    //-- back and sides ---------------------------------------------------------------
    box(m, b, 0.0f, length, topY - 0.2f, topY, depth, depth + 2.2f, pal::concrete);
    box(m, b, 0.0f, length, 0.0f, topY + 1.15f, depth + 2.2f, depth + 2.5f, pal::offWhite);
    {
        const Brush band{seatColor, venuemat::kPanel};
        box(m, b, 0.0f, length, topY - 1.6f, topY + 0.4f, depth + 2.5f, depth + 2.56f, band);
        label(m, b.at(0.5f * length, topY - 0.6f, depth + 2.56f), b.z, name, 0.2f, inkFor(seatColor));
    }
    for (float x : {0.0f, length}) {
        const vec3 out = (x == 0.0f) ? -b.x : b.x;
        quad(m, b.at(x, 0.0f, 0.3f), b.at(x, kFrontH, 0.3f), b.at(x, topY, depth), b.at(x, 0.0f, depth),
             out, pal::offWhite);
        quad(m, b.at(x, 0.0f, depth), b.at(x, topY, depth), b.at(x, topY + 1.15f, depth + 2.2f),
                b.at(x, 0.0f, depth + 2.2f), out, pal::offWhite);
        // Guard rail up the open side of the seating.
        const vec3 c[8] = {
            b.at(x - 0.03f, kFrontH, kFrontGap), b.at(x + 0.03f, kFrontH, kFrontGap),
            b.at(x + 0.03f, topY, depth),        b.at(x - 0.03f, topY, depth),
            b.at(x - 0.03f, kFrontH + 1.1f, kFrontGap), b.at(x + 0.03f, kFrontH + 1.1f, kFrontGap),
            b.at(x + 0.03f, topY + 1.1f, depth),        b.at(x - 0.03f, topY + 1.1f, depth)};
        hexa(m, c, pal::glass);
    }

    //-- roof ----------------------------------------------------------------------------
    if (roofed) {
        const float backZ = depth + 2.5f;
        const float roofBack = topY + 5.2f, roofFront = topY + 7.6f;
        const float frontZ = depth * 0.12f;
        const vec3 c[8] = {
            b.at(-1.0f, roofFront, frontZ), b.at(length + 1.0f, roofFront, frontZ),
            b.at(length + 1.0f, roofBack, backZ + 1.4f), b.at(-1.0f, roofBack, backZ + 1.4f),
            b.at(-1.0f, roofFront + 0.35f, frontZ), b.at(length + 1.0f, roofFront + 0.35f, frontZ),
            b.at(length + 1.0f, roofBack + 0.35f, backZ + 1.4f), b.at(-1.0f, roofBack + 0.35f, backZ + 1.4f)};
        hexa(m, c, pal::charcoal, &pal::roofing);
        box(m, b, -1.0f, length + 1.0f, roofFront - 0.5f, roofFront + 0.9f, frontZ - 0.25f, frontZ,
            Brush{seatColor, venuemat::kPanel});

        // Columns behind the top row, each carrying a raking beam forward
        // under the roof and a tie back to the ground.
        const int bays = std::max(1, static_cast<int>(std::round(length / 12.0f)));
        for (int k = 0; k <= bays; ++k) {
            const float x = length * static_cast<float>(k) / static_cast<float>(bays);
            box(m, b, x - 0.3f, x + 0.3f, 0.0f, roofBack, backZ, backZ + 0.7f, pal::steelDark);
            const vec3 r[8] = {
                b.at(x - 0.15f, roofFront - 0.25f, frontZ + 0.5f), b.at(x + 0.15f, roofFront - 0.25f, frontZ + 0.5f),
                b.at(x + 0.15f, roofBack - 1.5f, backZ + 0.3f),    b.at(x - 0.15f, roofBack - 1.5f, backZ + 0.3f),
                b.at(x - 0.15f, roofFront, frontZ + 0.5f),         b.at(x + 0.15f, roofFront, frontZ + 0.5f),
                b.at(x + 0.15f, roofBack, backZ + 0.3f),           b.at(x - 0.15f, roofBack, backZ + 0.3f)};
            hexa(m, r, pal::steelDark);
            const vec3 t[8] = {
                b.at(x - 0.1f, 0.0f, backZ + 6.0f),      b.at(x + 0.1f, 0.0f, backZ + 6.0f),
                b.at(x + 0.1f, 0.0f, backZ + 6.3f),      b.at(x - 0.1f, 0.0f, backZ + 6.3f),
                b.at(x - 0.1f, roofBack, backZ + 0.9f),  b.at(x + 0.1f, roofBack, backZ + 0.9f),
                b.at(x + 0.1f, roofBack, backZ + 1.2f),  b.at(x - 0.1f, roofBack, backZ + 1.2f)};
            hexa(m, t, pal::steelDark);
        }
    }

    keepouts.push_back({b.at(0.5f * length, 0.0f, 0.5f * depth + 3.0f), b.x,
                        0.5f * length + 6.0f, 0.5f * depth + 10.0f});
}

void Builder::buildGrandstands() {
    const float hw = lap.hw;
    const vec3 blue{0.10f, 0.20f, 0.46f};
    const vec3 red{0.62f, 0.10f, 0.09f};
    const vec3 teal{0.08f, 0.40f, 0.38f};
    const vec3 amber{0.80f, 0.50f, 0.08f};

    //-- main grandstand, opposite the pits: four roofed blocks ----------------------
    const float frontZ = -(hw + 5.5f + 6.0f);
    for (int k = 0; k < 4; ++k) {
        const float x0 = -168.0f + 81.0f * static_cast<float>(k);
        const Basis b{{x0, 0.0f, frontZ}, {1, 0, 0}, {0, 0, -1}};
        grandstand(b, 77.0f, 30, true, std::string("MAIN STAND ") + static_cast<char>('A' + k),
                   (k % 2) ? red : blue);
    }
    // Open stands continuing either side of it.
    grandstand({{172.0f, 0.0f, frontZ}, {1, 0, 0}, {0, 0, -1}}, 150.0f, 18, false, "EAST STAND", teal);
    grandstand({{-292.0f, 0.0f, frontZ}, {1, 0, 0}, {0, 0, -1}}, 112.0f, 16, false, "WEST STAND", amber);

    // Ground the main stand and the plaza behind it take up, in lap terms.
    reserveLeft(lap.length - 300.0f, lap.length - 176.0f, 30.0f);
    reserveLeft(lap.length - 176.0f, 160.0f, 68.0f);
    reserveLeft(160.0f, 330.0f, 32.0f);

    //-- plaza behind the main stand ----------------------------------------------------
    pad(*this, VenueLayer::Verge, -176.0f, 160.0f, -80.0f, -47.5f, kGrassY + 0.002f, pal::concrete);
    for (int k = 0; k < 9; ++k) {
        const float x = -160.0f + 37.0f * static_cast<float>(k);
        const Basis b{{x, 0.0f, -74.0f}};
        MeshData& m = mesh(VenueLayer::Solid, b.o);
        const vec3 c = kTeams[(k * 7 + 3) % kTeamCount].color;
        box(m, b, 0.0f, 7.0f, 0.0f, 3.0f, 0.0f, 4.0f, pal::offWhite);
        box(m, b, 0.3f, 6.7f, 1.0f, 2.3f, 4.0f, 4.05f, pal::black);            // serving hatch
        box(m, b, -0.4f, 7.4f, 3.0f, 3.25f, -0.3f, 6.2f, Brush{c, venuemat::kPanel});   // awning
    }
    {
        VenueInstanceGroupData& g = group("flag");
        cylinder(g.mesh, vec3(0.0f), kUp * 9.5f, 0.05f, 6, pal::steel);
        fixColour(g.mesh, 0);
        box(g.mesh, Basis{}, 0.05f, 1.9f, 8.2f, 9.4f, -0.015f, 0.015f, Brush{vec3(1.0f), venuemat::kPanel});
        g.maxDistance = 420.0f;
        g.castsShadow = false;
        for (int k = 0; k < 18; ++k) {
            place("flag", {-170.0f + 19.0f * static_cast<float>(k), 0.0f, -78.5f},
                  glm::normalize(vec3(1.0f, 0.0f, 0.35f)), vec3(1.0f), kTeams[k % kTeamCount].color);
        }
    }

    //-- stands on the outside of the corners that earn one -----------------------------
    const vec3 cornerColours[] = {red, blue, teal, amber};
    int placed = 0;
    for (std::size_t ci = 0; ci < corners.size(); ++ci) {
        const Corner& c = corners[ci];
        if (!c.stand || c.sign < 0.0f) continue;     // outside must be the left
        const bool big = (ci == 0);
        const float len = big ? 100.0f : 78.0f;
        const int rows = big ? 22 : 16;
        const float s = c.sa + c.length() * ((ci == 5) ? 0.5f : 0.38f);
        const Frame f = lap.at(s);
        const float bl = barrierAt(-1.0f, s);

        // A stand is straight; the barrier in front of it is not. Tangent to
        // the barrier's offset curve on the outside of a right-hander, the
        // stand only ever gets further from the track toward its ends.
        const vec3 o = f.at(-(bl + 5.5f), 0.0f) - f.fwd * (0.5f * len);
        const float depth = 1.3f + static_cast<float>(rows) * 0.85f + 2.5f;
        grandstand({o, f.fwd, -f.right}, len, rows, big,
                   "T" + std::to_string(ci + 1) + " STAND", cornerColours[placed++ % 4]);
        reserveLeft(s - 0.5f * len - 12.0f, s + 0.5f * len + 12.0f, 5.5f + depth + 14.0f);
    }
}

//---------------------------------------------------------------------------
//  The rest of the facility
//---------------------------------------------------------------------------

void Builder::buildFacilities() {
    //-- public car park, beyond the service road behind the main stand -----------------
    {
        const float x0 = -200.0f, x1 = 200.0f, z0 = -196.0f, z1 = -98.0f;
        pad(*this, VenueLayer::Verge, x0, x1, z0, z1, kGrassY + 0.002f, pal::service);
        keepouts.push_back({{0.0f, 0.0f, 0.5f * (z0 + z1)}, {1, 0, 0}, 0.5f * (x1 - x0) + 8.0f, 0.5f * (z1 - z0) + 8.0f});

        VenueInstanceGroupData& g = group("parked_car");
        box(g.mesh, Basis{}, -2.1f, 2.1f, 0.28f, 0.86f, -0.86f, 0.86f, Brush{vec3(1.0f), venuemat::kDefault});
        const std::size_t fixed = g.mesh.vertices.size();
        const vec3 c[8] = {
            {-1.5f, 0.86f, -0.80f}, {-1.5f, 0.86f, 0.80f}, {1.0f, 0.86f, 0.80f}, {1.0f, 0.86f, -0.80f},
            {-1.1f, 1.40f, -0.68f}, {-1.1f, 1.40f, 0.68f}, {0.4f, 1.40f, 0.68f}, {0.4f, 1.40f, -0.68f}};
        hexa(g.mesh, c, pal::glass, nullptr, false);
        for (float wx : {-1.35f, 1.35f}) {
            box(g.mesh, Basis{}, wx - 0.33f, wx + 0.33f, 0.0f, 0.62f, -0.90f, 0.90f,
                Brush{{0.05f, 0.05f, 0.055f}, venuemat::kRubber});
        }
        fixColour(g.mesh, fixed);
        g.maxDistance = 480.0f;
        g.castsShadow = false;

        static const vec3 kPaints[] = {
            {0.80f, 0.80f, 0.80f}, {0.12f, 0.12f, 0.13f}, {0.45f, 0.46f, 0.48f}, {0.55f, 0.08f, 0.07f},
            {0.10f, 0.20f, 0.42f}, {0.85f, 0.85f, 0.83f}, {0.20f, 0.22f, 0.24f}, {0.30f, 0.34f, 0.26f}};
        int id = 0;
        for (float z = z0 + 6.0f; z < z1 - 4.0f; z += 15.5f) {
            // Two rows nose to nose, then an aisle.
            for (int row = 0; row < 2; ++row) {
                const float rz = z + 5.2f * static_cast<float>(row);
                for (float x = x0 + 4.0f; x < x1 - 3.0f; x += 2.75f, ++id) {
                    if (std::fmod(x - x0, 60.0f) < 6.5f) continue;        // cross aisles
                    if (hash01(id, 3) > 0.62f) continue;
                    place("parked_car", {x, 0.0f, rz}, {0.0f, 0.0f, row ? 1.0f : -1.0f},
                          vec3(0.94f + 0.14f * hash01(id, 9)), kPaints[static_cast<int>(hash01(id, 5) * 7.99f)]);
                }
            }
            MeshData& m = mesh(VenueLayer::Paint, {0.0f, 0.0f, z});
            quad(m, {x0 + 3.0f, kPaintY, z + 2.55f}, {x1 - 3.0f, kPaintY, z + 2.55f},
                 {x1 - 3.0f, kPaintY, z + 2.67f}, {x0 + 3.0f, kPaintY, z + 2.67f}, kUp, pal::paint);
        }
    }

    //-- buildings beyond the car park: the venue does not end at the fence ----------------
    {
        MeshData& a = mesh(VenueLayer::Solid, {285.0f, 0.0f, -150.0f});
        block(a, Basis{{262.0f, 0.0f, -168.0f}}, 46.0f, 20.0f, 44.0f, 11, pal::white);     // hotel
        box(a, Basis{{262.0f, 0.0f, -168.0f}}, 6.0f, 40.0f, 44.5f, 47.0f, -0.2f, 0.0f, pal::charcoal);
        label(a, {285.0f, 45.75f, -168.2f}, {0, 0, -1}, "MERIDIAN", 0.30f, pal::white);
        label(a, {285.0f, 40.4f, -148.0f + 0.07f}, {0, 0, 1}, "MERIDIAN", 0.30f, pal::charcoal);
        keepouts.push_back({{285.0f, 0.0f, -158.0f}, {1, 0, 0}, 34.0f, 22.0f});

        MeshData& b2 = mesh(VenueLayer::Solid, {-285.0f, 0.0f, -150.0f});
        block(b2, Basis{{-320.0f, 0.0f, -172.0f}}, 64.0f, 26.0f, 21.0f, 5, pal::offWhite);  // offices
        keepouts.push_back({{-288.0f, 0.0f, -159.0f}, {1, 0, 0}, 42.0f, 24.0f});

        MeshData& c = mesh(VenueLayer::Solid, {0.0f, 0.0f, -250.0f});
        block(c, Basis{{-70.0f, 0.0f, -278.0f}}, 140.0f, 52.0f, 15.0f, 2, pal::white);      // exhibition hall
        label(c, {0.0f, 12.2f, -226.0f + 0.08f}, {0, 0, 1}, "APEX EXPO", 0.45f, pal::charcoal);
        keepouts.push_back({{0.0f, 0.0f, -252.0f}, {1, 0, 0}, 80.0f, 36.0f});
    }

    //-- infield: medical centre, helipad, and the road that serves them -------------------
    {
        pad(*this, VenueLayer::Verge, kPitX1, 318.0f, 62.0f, 68.5f, kGrassY + 0.004f, pal::service);
        MeshData& m = mesh(VenueLayer::Solid, {245.0f, 0.0f, 84.0f});
        const Basis b{{228.0f, 0.0f, 74.0f}};
        box(m, b, 0.0f, 32.0f, 0.0f, 4.6f, 0.0f, 15.0f, pal::white);
        box(m, b, 1.0f, 31.0f, 1.1f, 3.3f, -0.06f, 0.0f, pal::glass);
        box(m, b, -0.4f, 32.4f, 4.6f, 4.95f, -2.5f, 15.4f, pal::charcoal);
        box(m, b, 14.0f, 18.0f, 5.6f, 6.5f, -0.3f, -0.2f, pal::red);          // red cross
        box(m, b, 15.55f, 16.45f, 4.95f, 7.15f, -0.3f, -0.2f, pal::red);
        keepouts.push_back({{244.0f, 0.0f, 80.0f}, {1, 0, 0}, 24.0f, 16.0f});

        pad(*this, VenueLayer::Verge, 278.0f, 318.0f, 72.0f, 112.0f, kGrassY + 0.004f, pal::concrete);
        textQuads(mesh(VenueLayer::Paint, {298.0f, 0.0f, 92.0f}), {294.0f, kPaintY, 86.0f},
                  {1, 0, 0}, {0, 0, 1}, kUp, "H", 1.6f, 1.6f, pal::paintYellow);
        keepouts.push_back({{298.0f, 0.0f, 92.0f}, {1, 0, 0}, 30.0f, 30.0f});
        keepouts.push_back({{254.0f, 0.0f, 65.0f}, {1, 0, 0}, 68.0f, 8.0f});
    }

    //-- infield lake -------------------------------------------------------------------------
    {
        const vec3 centre{385.0f, kGrassY + 0.002f, 545.0f};
        const float rx = 105.0f, rz = 62.0f;
        const Brush water{{0.045f, 0.105f, 0.13f}, venuemat::kWater};
        MeshData& m = mesh(VenueLayer::Verge, centre);
        constexpr int kSegs = 36;
        for (int i = 0; i < kSegs; ++i) {
            const float a0 = 2.0f * kPiF * static_cast<float>(i) / kSegs;
            const float a1 = 2.0f * kPiF * static_cast<float>(i + 1) / kSegs;
            // An ellipse pulled about a little, so it is not a swimming pool.
            auto rim = [&](float a) {
                const float k = 1.0f + 0.13f * std::sin(a * 3.0f + 0.7f) + 0.07f * std::sin(a * 5.0f);
                return centre + vec3(std::cos(a) * rx * k, 0.0f, std::sin(a) * rz * k);
            };
            tri(m, centre, rim(a0), rim(a1), kUp, water);
        }
        keepouts.push_back({centre, {1, 0, 0}, rx * 1.25f, rz * 1.3f});
    }
}

} // namespace vsim::apexvenue
