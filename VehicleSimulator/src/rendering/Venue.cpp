#include "rendering/Venue.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace vsim {
namespace {

/// Instances are bucketed on this grid so a group can be culled a cell at a
/// time. Coarse on purpose: a cell is one draw call.
constexpr float kInstanceCell = 120.0f;

struct Frustum {
    glm::vec4 planes[6];

    explicit Frustum(const glm::mat4& m) {
        // Gribb-Hartmann: rows of the clip matrix, combined.
        const glm::vec4 r0(m[0][0], m[1][0], m[2][0], m[3][0]);
        const glm::vec4 r1(m[0][1], m[1][1], m[2][1], m[3][1]);
        const glm::vec4 r2(m[0][2], m[1][2], m[2][2], m[3][2]);
        const glm::vec4 r3(m[0][3], m[1][3], m[2][3], m[3][3]);
        planes[0] = r3 + r0; planes[1] = r3 - r0;
        planes[2] = r3 + r1; planes[3] = r3 - r1;
        planes[4] = r3 + r2; planes[5] = r3 - r2;
    }

    bool intersects(const glm::vec3& lo, const glm::vec3& hi) const {
        for (const glm::vec4& p : planes) {
            // The box corner furthest along the plane normal.
            const glm::vec3 v(p.x >= 0.0f ? hi.x : lo.x,
                              p.y >= 0.0f ? hi.y : lo.y,
                              p.z >= 0.0f ? hi.z : lo.z);
            if (p.x * v.x + p.y * v.y + p.z * v.z + p.w < 0.0f) return false;
        }
        return true;
    }
};

float distanceToBox(const glm::vec3& p, const glm::vec3& lo, const glm::vec3& hi) {
    const glm::vec3 d = glm::max(glm::max(lo - p, p - hi), glm::vec3(0.0f));
    return glm::length(d);
}

} // namespace

void Venue::clear() {
    for (Group& g : m_groups) {
        if (g.buffer) gl::DeleteBuffers(1, &g.buffer);
    }
    m_groups.clear();
    m_chunks.clear();
    m_totalTriangles = 0;
    m_totalInstances = 0;
}

void Venue::upload(VenueData&& data) {
    clear();

    m_chunks.reserve(data.chunks.size());
    for (VenueChunkData& cd : data.chunks) {
        if (cd.mesh.vertices.empty()) continue;

        Chunk c;
        c.layer = cd.layer;
        c.minDistance = cd.minDistance;
        c.maxDistance = cd.maxDistance;
        c.lo = c.hi = cd.mesh.vertices.front().position;
        for (const Vertex& v : cd.mesh.vertices) {
            c.lo = glm::min(c.lo, v.position);
            c.hi = glm::max(c.hi, v.position);
        }
        cd.mesh.uploadTo(c.mesh);
        m_totalTriangles += cd.mesh.indices.size() / 3;
        m_chunks.push_back(std::move(c));
    }

    m_groups.reserve(data.groups.size());
    for (VenueInstanceGroupData& gd : data.groups) {
        if (gd.mesh.vertices.empty() || gd.instances.empty()) continue;

        // Bounding radius of the mesh about its own origin, to pad cells by.
        float radius = 0.0f;
        for (const Vertex& v : gd.mesh.vertices) radius = std::max(radius, glm::length(v.position));

        std::map<std::pair<int, int>, std::vector<InstanceData>> cells;
        for (const InstanceData& inst : gd.instances) {
            const glm::vec3 p(inst.transform[3]);
            cells[{static_cast<int>(std::floor(p.x / kInstanceCell)),
                   static_cast<int>(std::floor(p.z / kInstanceCell))}].push_back(inst);
        }

        Group g;
        g.minDistance = gd.minDistance;
        g.maxDistance = gd.maxDistance;
        g.castsShadow = gd.castsShadow || gd.shadowOnly;
        g.shadowOnly  = gd.shadowOnly;
        g.twoSided    = gd.twoSided;

        std::vector<InstanceData> sorted;
        sorted.reserve(gd.instances.size());
        for (auto& [key, list] : cells) {
            Range r;
            r.first = sorted.size();
            r.count = list.size();
            r.lo = glm::vec3(1e30f);
            r.hi = glm::vec3(-1e30f);
            for (const InstanceData& inst : list) {
                const glm::vec3 p(inst.transform[3]);
                const float scale = std::max({glm::length(glm::vec3(inst.transform[0])),
                                              glm::length(glm::vec3(inst.transform[1])),
                                              glm::length(glm::vec3(inst.transform[2]))});
                r.lo = glm::min(r.lo, p - glm::vec3(radius * scale));
                r.hi = glm::max(r.hi, p + glm::vec3(radius * scale));
                sorted.push_back(inst);
            }
            g.ranges.push_back(r);
        }

        gd.mesh.uploadTo(g.mesh);
        gl::GenBuffers(1, &g.buffer);
        gl::BindBuffer(gl::ARRAY_BUFFER, g.buffer);
        gl::BufferData(gl::ARRAY_BUFFER,
                       static_cast<gl::GLsizeiptr>(sorted.size() * sizeof(InstanceData)),
                       sorted.data(), gl::STATIC_DRAW);
        gl::BindBuffer(gl::ARRAY_BUFFER, 0);

        m_totalInstances += sorted.size();
        m_totalTriangles += gd.mesh.indices.size() / 3 * sorted.size();
        m_groups.push_back(std::move(g));
    }
}

void Venue::drawLayer(VenueLayer layer, const VenueView& view) const {
    const Frustum frustum(view.viewProj);
    for (const Chunk& c : m_chunks) {
        if (c.layer != layer) continue;
        const float d = distanceToBox(view.eye, c.lo, c.hi);
        if (d < c.minDistance || d >= c.maxDistance) continue;
        if (!frustum.intersects(c.lo, c.hi)) continue;
        c.mesh.draw();
        ++m_stats.drawCalls;
        m_stats.triangles += static_cast<long long>(c.mesh.indexCount() / 3);
    }
}

void Venue::drawInstances(const VenueView& view) const {
    if (m_groups.empty() || !view.shader) return;

    const Frustum frustum(view.viewProj);
    view.shader->setInt("uInstanced", 1);
    for (const Group& g : m_groups) {
        if (view.depthPass ? !g.castsShadow : g.shadowOnly) continue;
        if (g.twoSided) gl::Disable(gl::CULL_FACE);
        for (const Range& r : g.ranges) {
            const float d = distanceToBox(view.eye, r.lo, r.hi);
            if (d < g.minDistance || d >= g.maxDistance) continue;
            if (!frustum.intersects(r.lo, r.hi)) continue;
            g.mesh.drawInstanced(g.buffer, r.first, r.count);
            ++m_stats.drawCalls;
            m_stats.triangles += static_cast<long long>(g.mesh.indexCount() / 3 * r.count);
        }
        if (g.twoSided) gl::Enable(gl::CULL_FACE);
    }
    view.shader->setInt("uInstanced", 0);
}

} // namespace vsim
