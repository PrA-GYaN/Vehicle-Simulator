#pragma once
//===========================================================================
//  ApexVenueBuilder.h  --  shared state of the Apex venue generator.
//
//  Internal (included by ApexVenue*.cpp only). The generator is split over
//  several files by what is being built; this is what they share:
//
//    * the lap and the per-sample PROFILE derived from it - where the
//      barrier stands on each side, how much runoff there is, where the
//      kerbs and the rubber go;
//    * the chunk and instance-group bookkeeping that turns "emit this here"
//      into the batched, culled VenueData the renderer draws.
//
//  SIDES. Index 0 is the driver's right (+lateral), 1 the left. The circuit
//  runs clockwise and does not cross itself, so the right is always the
//  infield and the left always the outside of the whole venue. A good deal
//  below leans on that: the service road, the perimeter fence and the
//  spectator banks all follow the left side and can never run into another
//  part of the track.
//===========================================================================

#include <deque>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "rendering/ApexVenueKit.h"

namespace vsim::apexvenue {

using namespace vsim::venuekit;

//---------------------------------------------------------------------------
//  Palette. Display-referred values; the shader linearises them.
//---------------------------------------------------------------------------
namespace pal {
inline const Brush asphalt   {{0.200f, 0.200f, 0.210f}, venuemat::kAsphalt};
inline const Brush pitAsphalt{{0.235f, 0.235f, 0.245f}, venuemat::kAsphalt};
inline const Brush runoff    {{0.285f, 0.285f, 0.290f}, venuemat::kAsphalt};
inline const Brush service   {{0.300f, 0.298f, 0.295f}, venuemat::kAsphalt};
inline const Brush paint     {{0.900f, 0.900f, 0.870f}, venuemat::kPaint};
inline const Brush paintYellow{{0.880f, 0.720f, 0.120f}, venuemat::kPaint};
inline const Brush kerbRed   {{0.640f, 0.085f, 0.075f}, venuemat::kKerb};
inline const Brush kerbWhite {{0.840f, 0.840f, 0.810f}, venuemat::kKerb};
inline const Brush grass     {{0.232f, 0.315f, 0.150f}, venuemat::kGrass};
inline const Brush gravel    {{0.590f, 0.535f, 0.410f}, venuemat::kGravel};
inline const Brush concrete  {{0.640f, 0.640f, 0.620f}, venuemat::kConcrete};
inline const Brush concreteDark{{0.430f, 0.430f, 0.420f}, venuemat::kConcrete};
inline const Brush steel     {{0.640f, 0.670f, 0.700f}, venuemat::kSteel};
inline const Brush steelDark {{0.300f, 0.320f, 0.340f}, venuemat::kSteel};
inline const Brush roofing   {{0.430f, 0.445f, 0.460f}, venuemat::kRoofing};
inline const Brush rubber    {{0.075f, 0.075f, 0.080f}, venuemat::kRubber};
inline const Brush glass     {{0.060f, 0.090f, 0.120f}, venuemat::kGlass};
inline const Brush white     {{0.900f, 0.900f, 0.890f}, venuemat::kPanel};
inline const Brush offWhite  {{0.780f, 0.780f, 0.760f}, venuemat::kPanel};
inline const Brush charcoal  {{0.140f, 0.150f, 0.165f}, venuemat::kPanel};
inline const Brush black     {{0.035f, 0.035f, 0.040f}, venuemat::kPanel};
inline const Brush red       {{0.760f, 0.110f, 0.090f}, venuemat::kPanel};
inline const Brush navy      {{0.090f, 0.160f, 0.330f}, venuemat::kPanel};
inline const Brush orange    {{0.900f, 0.420f, 0.060f}, venuemat::kPanel};
inline const Brush fence     {{0.560f, 0.580f, 0.600f}, venuemat::kFence};
} // namespace pal

/// Fictional entrants and sponsors. Nothing here is a real brand.
struct Team { const char* name; vec3 color; };
inline const Team kTeams[] = {
    {"VELOCE",   {0.74f, 0.10f, 0.09f}}, {"NORDIK",   {0.10f, 0.32f, 0.62f}},
    {"HELIX",    {0.08f, 0.46f, 0.40f}}, {"ARGENT",   {0.62f, 0.64f, 0.67f}},
    {"KESTREL",  {0.86f, 0.46f, 0.07f}}, {"ORION",    {0.13f, 0.14f, 0.30f}},
    {"SOLARA",   {0.86f, 0.72f, 0.10f}}, {"VERTEX",   {0.30f, 0.52f, 0.14f}},
    {"MERIDIAN", {0.36f, 0.14f, 0.42f}}, {"ONYX",     {0.11f, 0.11f, 0.12f}},
    {"CORSA",    {0.66f, 0.08f, 0.28f}}, {"TITAN",    {0.20f, 0.42f, 0.58f}},
};
constexpr int kTeamCount = static_cast<int>(sizeof(kTeams) / sizeof(kTeams[0]));

//---------------------------------------------------------------------------
//  Authored dimensions shared between files.
//---------------------------------------------------------------------------
constexpr float kKerbWidth   = 1.30f;
constexpr float kVergeBase   = 6.0f;    ///< track edge to barrier where there is no runoff
constexpr float kWallHeight  = 1.05f;
constexpr float kFenceHeight = 3.60f;

// Pit complex, in main-straight coordinates: x along the straight, z to the
// driver's right. The straight is the world X axis (ApexGrandPrix.cpp).
constexpr float kPitX0        = -200.0f;  ///< pit apron, start
constexpr float kPitX1        =  190.0f;  ///< pit apron, end
constexpr float kPitWallZ     =    8.9f;  ///< centre of the pit wall
constexpr float kPitLaneZ0    =   10.5f;  ///< fast lane, inner edge
constexpr float kPitLaneZ1    =   15.5f;  ///< fast lane, outer edge
constexpr float kPitBuildingZ =   21.0f;  ///< garage fronts
constexpr float kPitBuildingDepth = 16.0f;
constexpr float kGarageX0     = -180.0f;
constexpr float kGaragePitch  =    7.5f;
constexpr int   kGarageCount  =   36;
constexpr float kBuildingX0   = -190.0f;
constexpr float kBuildingX1   =  110.0f;
constexpr float kTowerX1      =  135.0f;

struct Corner {
    float sa = 0.0f, sb = 0.0f;   ///< arc length of entry and exit
    float sign = 1.0f;            ///< +1 right-hander, -1 left
    float runoff = 20.0f;         ///< depth of the outside runoff [m]
    int   surface = 1;            ///< 1 gravel, 2 paved
    bool  brake = false;          ///< gets distance boards and skid marks
    bool  stand = false;          ///< gets a grandstand on the outside
    float length() const { return sb - sa; }
};

enum Surface { kSurfGrass = 0, kSurfGravel = 1, kSurfPaved = 2 };

/// An oriented rectangle on the ground that scenery must keep out of.
struct Keepout { vec3 centre; vec3 axis; float halfLength; float halfWidth; };

struct Builder {
    const Track& track;
    Lap lap;

    //-- per-sample profile, [side][sample] ---------------------------------
    std::vector<Corner> corners;
    std::vector<float> barrier[2];   ///< barrier face, metres from the centreline
    std::vector<float> outside[2];   ///< runoff weight, 0..1
    std::vector<int>   surface[2];
    std::vector<float> kerb[2];      ///< how much kerb, 0..1
    std::vector<char>  wall[2];      ///< 1 = concrete wall + catch fence, 0 = armco
    std::vector<float> reserve;      ///< left side: depth already spoken for
    std::vector<float> service;      ///< left side: service road centre
    std::vector<float> line;         ///< racing line, lateral
    std::vector<float> rubber;       ///< how much rubber is down, 0..1
    std::vector<char>  onMain;       ///< sample lies on the main straight
    std::vector<Keepout> keepouts;

    explicit Builder(const Track& t);

    static int sideIndex(float sgn) { return (sgn > 0.0f) ? 0 : 1; }
    /// Barrier line at an arbitrary arc length. Interpolated - except across
    /// a step in the line (the two ends of the pit wall), where blending the
    /// two sides would invent a diagonal piece of barrier across the pit lane.
    float barrierAt(float sgn, float s) const {
        const std::vector<float>& b = barrier[sideIndex(sgn)];
        float u = 0.0f;
        const int i = lap.locate(s, u);
        const float b0 = b[static_cast<std::size_t>(i)];
        const float b1 = b[static_cast<std::size_t>(lap.wrap(i + 1))];
        if (std::abs(b1 - b0) > 2.5f) return (u < 0.5f) ? b0 : b1;
        return b0 + (b1 - b0) * u;
    }
    float pitLateral(float x) const;

    /// Reserve `depth` metres behind the left barrier over [s0, s1], so the
    /// service road and the trees go round whatever is being put there.
    void reserveLeft(float s0, float s1, float depth);

    //-- output ---------------------------------------------------------------
    //
    // References returned by mesh() and group() stay valid for the life of
    // the builder (the stores are deques), so a stage can hold several open
    // at once.
    MeshData& mesh(VenueLayer layer, const vec3& where, float minDistance = 0.0f,
                   float maxDistance = 1.0e9f);
    VenueInstanceGroupData& group(const std::string& name);
    void place(const std::string& groupName, const vec3& pos, const vec3& facing,
               const vec3& scale = vec3(1.0f), const vec3& tint = vec3(1.0f));

    //-- stages, in build order (one or two per source file) ------------------
    void buildProfile();          // ApexVenue.cpp
    void buildRoad();             // ApexVenueSurfaces.cpp
    void buildMarkings();
    void buildKerbs();
    void buildVerge();
    void buildBarriers();         // ApexVenueTrackside.cpp
    void buildTrackside();
    void buildPitComplex();       // ApexVenueBuildings.cpp
    void buildGrandstands();
    void buildFacilities();
    void finishLeftSide();        // ApexVenue.cpp - after everything reserved
    void buildServiceRoad();      // ApexVenueSurfaces.cpp
    void buildLandscape();        // ApexVenueLandscape.cpp

    /// Hand over everything built. The builder is spent afterwards.
    VenueData finish();

    /// Shared by the buildings file and anything that wants a stand.
    void grandstand(const Basis& b, float length, int rows, bool roofed,
                    const std::string& name, const vec3& seatColor);

    /// Ground height of the landscape at a world position (0 over the venue).
    float groundHeight(float x, float z) const;
    /// Distance from the centreline, and whether the point is in the infield.
    float trackDistance(float x, float z, bool* inside = nullptr, int* nearest = nullptr) const;

private:
    std::deque<VenueChunkData> m_chunks;
    std::deque<VenueInstanceGroupData> m_groups;
    std::map<std::tuple<int, int, int, int>, std::size_t> m_chunkIndex;
    std::map<std::string, std::size_t> m_groupIndex;
    std::vector<vec2> m_outline;   ///< decimated centreline, for inside tests
};

} // namespace vsim::apexvenue
