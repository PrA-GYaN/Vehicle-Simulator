#pragma once
//===========================================================================
//  TerrainHeightField.h  --  a world-space ground height lookup.
//
//  Built once from the raw vertex positions of an imported static scene
//  model (see rendering/GltfModel), then queried every physics step by the
//  per-wheel quarter-car suspension model (physics/FourWheelModel.cpp) to
//  find the ground height under each wheel.
//
//  This is deliberately NOT built from the triangle mesh. Photogrammetry/CAD
//  scans of this scale (the reference asset this was built for is ~424k
//  vertices) are dense enough that binning the vertices alone into an XZ
//  grid, keeping the highest point per cell, gives good coverage at a a few
//  metres of cell size - at O(vertex count) cost instead of the cost of a
//  full triangle rasterisation. This is a discretised APPROXIMATION of the
//  surface, not an exact one; see README -> Open Questions.
//
//  GL-FREE, like everything else under src/environment/ - this is part of
//  vsim_core (see the CMakeLists.txt architectural rule) so the physics can
//  query it directly. It is populated from the rendering layer, which reads
//  the geometry out of an already-loaded GltfModel and hands this class
//  plain points - it never touches cgltf, stb_image or OpenGL itself.
//===========================================================================

#include <vector>

#include <glm/glm.hpp>

namespace vsim {

class TerrainHeightField {
public:
    /// Bin `points` (world-space positions) into an XZ grid of `cellSize`
    /// metres, keeping the highest Y per cell. Replaces any previously built
    /// data. A field built from an empty or degenerate point set is left
    /// invalid() and heightAt() falls back to 0.
    void build(const std::vector<glm::vec3>& points, double cellSize = 4.0);

    bool valid() const { return m_valid; }

    /// Ground height [m] at world (x, z). Bilinearly interpolated between
    /// filled neighbouring cells; a query that falls in or beyond an empty
    /// region of the grid clamps to the nearest filled cell rather than
    /// dropping to 0, so there is no discontinuous cliff at the mesh's edge.
    /// Returns 0.0 if !valid().
    double heightAt(double worldX, double worldZ) const;

    glm::vec3 boundsMin() const { return m_boundsMin; }
    glm::vec3 boundsMax() const { return m_boundsMax; }

private:
    /// Nearest filled cell's height, by expanding rings around (cx, cz).
    /// Used both directly (out-of-grid queries) and as the fallback for a
    /// bilinear sample whose corner cell is itself empty. Capped at
    /// kMaxSearchRadius rings - a real scan can have large genuinely EMPTY
    /// regions (the bounding box of a track layout is mostly not track), and
    /// without a cap a query that lands in one degrades to an O(grid area)
    /// scan on every physics step. Beyond the cap this returns 0.0, the same
    /// "no data here" fallback as a query outside the grid entirely.
    double nearestFilledHeight(int cx, int cz) const;

    static constexpr int kMaxSearchRadius = 40;   // cells (= 160 m at the default 4 m cell size)

    double cellHeight(int cx, int cz, bool& filled) const;

    int m_cols = 0;
    int m_rows = 0;
    double m_cellSize = 4.0;
    glm::vec3 m_boundsMin{0.0f};
    glm::vec3 m_boundsMax{0.0f};
    bool m_valid = false;

    std::vector<float> m_height;   ///< m_cols * m_rows, row-major over Z then X
    std::vector<bool>  m_filled;
};

} // namespace vsim
