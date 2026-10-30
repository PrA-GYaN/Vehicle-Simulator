#include "environment/TerrainHeightField.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vsim {

void TerrainHeightField::build(const std::vector<glm::vec3>& points, double cellSize) {
    m_valid = false;
    m_cols = m_rows = 0;
    m_height.clear();
    m_filled.clear();

    if (points.empty() || !(cellSize > 0.01)) return;

    m_cellSize = cellSize;
    m_boundsMin = points.front();
    m_boundsMax = points.front();
    for (const glm::vec3& p : points) {
        m_boundsMin = glm::min(m_boundsMin, p);
        m_boundsMax = glm::max(m_boundsMax, p);
    }

    const double spanX = std::max(0.0, static_cast<double>(m_boundsMax.x - m_boundsMin.x));
    const double spanZ = std::max(0.0, static_cast<double>(m_boundsMax.z - m_boundsMin.z));
    m_cols = static_cast<int>(std::floor(spanX / m_cellSize)) + 1;
    m_rows = static_cast<int>(std::floor(spanZ / m_cellSize)) + 1;
    if (m_cols < 1 || m_rows < 1) return;

    m_height.assign(static_cast<std::size_t>(m_cols) * static_cast<std::size_t>(m_rows), 0.0f);
    m_filled.assign(m_height.size(), false);

    for (const glm::vec3& p : points) {
        const int cx = static_cast<int>(std::floor((p.x - m_boundsMin.x) / m_cellSize));
        const int cz = static_cast<int>(std::floor((p.z - m_boundsMin.z) / m_cellSize));
        const int ccx = std::clamp(cx, 0, m_cols - 1);
        const int ccz = std::clamp(cz, 0, m_rows - 1);
        const std::size_t idx = static_cast<std::size_t>(ccz) * static_cast<std::size_t>(m_cols) +
                                 static_cast<std::size_t>(ccx);
        if (!m_filled[idx] || p.y > m_height[idx]) {
            m_height[idx] = p.y;
            m_filled[idx] = true;
        }
    }

    m_valid = true;
}

double TerrainHeightField::cellHeight(int cx, int cz, bool& filled) const {
    if (cx < 0 || cz < 0 || cx >= m_cols || cz >= m_rows) {
        filled = false;
        return 0.0;
    }
    const std::size_t idx = static_cast<std::size_t>(cz) * static_cast<std::size_t>(m_cols) +
                             static_cast<std::size_t>(cx);
    filled = m_filled[idx];
    return static_cast<double>(m_height[idx]);
}

double TerrainHeightField::nearestFilledHeight(int cx, int cz) const {
    bool filled = false;
    double h = cellHeight(cx, cz, filled);
    if (filled) return h;

    // Visit ONLY the perimeter cells of each ring - O(r) per ring, not
    // O(r^2) - capped at kMaxSearchRadius. A real scan's bounding box is
    // mostly not the scanned surface itself (see the header), so an
    // unbounded/quadratic search here turns one query in an empty region
    // into a scan of the entire grid, on every physics step.
    for (int r = 1; r <= kMaxSearchRadius; ++r) {
        double sum = 0.0;
        int count = 0;
        const int xlo = cx - r, xhi = cx + r;
        const int zlo = cz - r, zhi = cz + r;

        auto sample = [&](int x, int z) {
            bool f = false;
            const double ch = cellHeight(x, z, f);
            if (f) { sum += ch; ++count; }
        };

        for (int x = xlo; x <= xhi; ++x) { sample(x, zlo); sample(x, zhi); }
        for (int z = zlo + 1; z <= zhi - 1; ++z) { sample(xlo, z); sample(xhi, z); }

        if (count > 0) return sum / count;
    }
    return 0.0;   // nothing filled within range - see kMaxSearchRadius
}

double TerrainHeightField::heightAt(double worldX, double worldZ) const {
    if (!m_valid) return 0.0;

    const double fx = (worldX - m_boundsMin.x) / m_cellSize;
    const double fz = (worldZ - m_boundsMin.z) / m_cellSize;

    const int cx0 = static_cast<int>(std::floor(fx));
    const int cz0 = static_cast<int>(std::floor(fz));

    // Fully outside the grid: no interpolation possible, just the nearest
    // filled cell to the clamped coordinate.
    if (cx0 < -1 || cz0 < -1 || cx0 > m_cols || cz0 > m_rows) {
        const int ccx = std::clamp(cx0, 0, m_cols - 1);
        const int ccz = std::clamp(cz0, 0, m_rows - 1);
        return nearestFilledHeight(ccx, ccz);
    }

    const double tx = fx - static_cast<double>(cx0);
    const double tz = fz - static_cast<double>(cz0);

    const double h00 = nearestFilledHeight(cx0,     cz0);
    const double h10 = nearestFilledHeight(cx0 + 1, cz0);
    const double h01 = nearestFilledHeight(cx0,     cz0 + 1);
    const double h11 = nearestFilledHeight(cx0 + 1, cz0 + 1);

    const double h0 = h00 * (1.0 - tx) + h10 * tx;
    const double h1 = h01 * (1.0 - tx) + h11 * tx;
    return h0 * (1.0 - tz) + h1 * tz;
}

} // namespace vsim
