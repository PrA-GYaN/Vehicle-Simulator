#pragma once
//===========================================================================
//  Venue.h  --  a complete circuit venue, as the renderer draws it.
//
//  WHY THIS IS NOT TrackMesh
//  -------------------------
//  TrackMesh turns ANY centreline into a recognisable circuit and hands back
//  six meshes. That is the right shape for a generator that has to work on a
//  track it has never seen, and the wrong one for an authored venue: a full
//  Grand Prix facility is a few hundred thousand triangles spread over
//  several square kilometres, most of it behind the camera, much of it the
//  same seat or fence post or tree repeated thousands of times.
//
//  So a venue is delivered as:
//
//    * CHUNKS - static geometry, batched by draw layer and by a coarse
//      ground-plane grid, each with a bounding box. A frame draws the chunks
//      the view frustum touches and skips the rest.
//    * INSTANCE GROUPS - one small mesh plus a list of placements (seats,
//      spectators, fence posts, trees, tyre stacks). Placements are sorted
//      into the same grid and drawn a cell at a time with one instanced call.
//    * DISTANCE RANGES on both, which is all the level of detail there is:
//      a tree is one group for nearby cells and a cheaper group beyond.
//
//  Vertices carry a material id (Vertex::material, venuemat:: below) that
//  the venue path of basic.frag switches on, so a chunk holding concrete,
//  steel and glass is still a single draw call.
//
//  This file knows nothing about any particular circuit. The one venue that
//  exists is built by rendering/ApexVenue.h.
//===========================================================================

#include <vector>

#include <glm/glm.hpp>

#include "rendering/Mesh.h"
#include "rendering/MeshFactory.h"
#include "rendering/Shader.h"

namespace vsim {

/// Material ids understood by the venue path of assets/shaders/basic.frag.
namespace venuemat {
constexpr float kDefault  = 0.0f;   ///< vertex colour, gloss from uGloss
constexpr float kAsphalt  = 1.0f;   ///< param: rubber laid down, 0..1
constexpr float kGrass    = 2.0f;   ///< param: 1 = mown in stripes
constexpr float kGravel   = 3.0f;
constexpr float kConcrete = 4.0f;
constexpr float kSteel    = 5.0f;
constexpr float kGlass    = 6.0f;
constexpr float kPaint    = 7.0f;   ///< road paint
constexpr float kKerb     = 8.0f;
constexpr float kRubber   = 9.0f;
constexpr float kFoliage  = 10.0f;
constexpr float kDecal    = 11.0f;  ///< param: opacity. Blended.
constexpr float kFence    = 12.0f;  ///< wire mesh, uv in metres. Blended.
constexpr float kCrowd    = 13.0f;  ///< uv = (seat, row), param: occupancy
constexpr float kLit      = 14.0f;  ///< param: emission strength
constexpr float kPanel    = 15.0f;  ///< painted cladding, hoardings
constexpr float kWater    = 16.0f;
constexpr float kLeafCard = 17.0f;  ///< foliage cut-out, uv across the card. Alpha-tested.
constexpr float kBark     = 18.0f;
constexpr float kRoofing  = 19.0f;  ///< profiled steel sheet
} // namespace venuemat

/// Draw layers, in the order the lit pass draws them. The first four are
/// flat and coplanar to within millimetres and are separated by polygon
/// offset (see Renderer::drawEnvironment); none of those casts a shadow.
enum class VenueLayer {
    Terrain,      ///< ground out to the horizon
    Verge,        ///< grass, gravel and service roads beside the track
    Asphalt,      ///< racing surface, pit lane, paved runoff
    Paint,        ///< lines, grid, decals - blended over the asphalt
    Kerbs,        ///< raised, but too low to bother the shadow map with
    Solid,        ///< everything that stands up: barriers, buildings, stands
    Transparent,  ///< catch fencing. Drawn last, no depth writes.
    Count
};

struct VenueChunkData {
    VenueLayer layer = VenueLayer::Solid;
    MeshData   mesh;
    float minDistance = 0.0f;     ///< drawn when the camera is within
    float maxDistance = 1.0e9f;   ///< [min, max) of the chunk's bounds
};

struct VenueInstanceGroupData {
    MeshData mesh;
    std::vector<InstanceData> instances;
    float minDistance = 0.0f;
    float maxDistance = 1.0e9f;
    bool  castsShadow = true;
    /// Drawn ONLY into the shadow maps: a solid stand-in for a group whose
    /// visible form cannot cast a sensible shadow of its own. A tree's
    /// crown is cut-out cards, and the depth pass does not read textures,
    /// so each card would cast a square.
    bool  shadowOnly = false;
    /// Seen from both sides (foliage cards): drawn without face culling.
    bool  twoSided = false;
};

/// A venue as generated, before upload.
struct VenueData {
    std::vector<VenueChunkData>         chunks;
    std::vector<VenueInstanceGroupData> groups;
};

/// What a pass is looking at. `viewProj` is the camera's matrix in the lit
/// pass and the light's in a shadow pass - culling is the same test either
/// way. `eye` is always the CAMERA position: level of detail must not
/// change between the shadow pass and the pass that samples it.
struct VenueView {
    glm::mat4 viewProj{1.0f};
    glm::vec3 eye{0.0f};
    const Shader* shader = nullptr;
    bool depthPass = false;
};

class Venue {
public:
    Venue() = default;
    ~Venue() { clear(); }
    Venue(const Venue&) = delete;
    Venue& operator=(const Venue&) = delete;

    /// Replace the venue's contents. Requires a current GL context.
    void upload(VenueData&& data);
    void clear();

    bool empty() const { return m_chunks.empty() && m_groups.empty(); }

    void drawLayer(VenueLayer layer, const VenueView& view) const;

    /// Draw every instance group. In a depth pass only the shadow casters.
    void drawInstances(const VenueView& view) const;

    //-- statistics, for the performance readout --------------------------
    struct Stats {
        int drawCalls = 0;
        long long triangles = 0;
    };
    void resetStats() const { m_stats = {}; }
    const Stats& stats() const { return m_stats; }

    std::size_t totalTriangles() const { return m_totalTriangles; }
    std::size_t totalInstances() const { return m_totalInstances; }
    std::size_t chunkCount() const { return m_chunks.size(); }

private:
    struct Chunk {
        VenueLayer layer;
        Mesh mesh;
        glm::vec3 lo, hi;
        float minDistance, maxDistance;
    };
    struct Range {
        std::size_t first, count;
        glm::vec3 lo, hi;
    };
    struct Group {
        Mesh mesh;
        gl::GLuint buffer = 0;
        std::vector<Range> ranges;
        float minDistance, maxDistance;
        bool castsShadow, shadowOnly, twoSided;
    };

    std::vector<Chunk> m_chunks;
    std::vector<Group> m_groups;
    std::size_t m_totalTriangles = 0;
    std::size_t m_totalInstances = 0;
    mutable Stats m_stats;
};

} // namespace vsim
