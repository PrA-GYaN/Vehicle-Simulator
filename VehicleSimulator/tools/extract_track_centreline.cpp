//===========================================================================
//  extract_track_centreline  --  offline tool.
//
//  WHAT THIS IS FOR
//  ----------------
//  The project was handed a third-party photogrammetry/CAD scan of the
//  Nurburgring GP-Strecke (assets/models/nurburgring_gp.glb, 91 MB). Drawing
//  that scan directly is what every other track in this simulator stopped
//  doing: it is opaque geometry with no centreline, so nothing can be derived
//  from it - not a start line, not a kerb, not a spawn point, not a lap
//  distance - and the application is reduced to GUESSING a spawn location by
//  hunting for a locally flat patch of scanned vertices.
//
//  So the scan is used as a REFERENCE and not as an asset: this tool reads
//  the road surface out of it, recovers the centreline the layout implies,
//  and emits that centreline as a small C++ table. The simulator then
//  generates the circuit from the centreline exactly like every other track
//  (src/rendering/TrackMesh.h), and the 91 MB file is no longer needed at
//  runtime.
//
//  This is deliberately an OFFLINE tool, run by hand and its output committed.
//  It is not part of the simulator, it is not on the startup path, and the
//  generated header is readable, reviewable data.
//
//  HOW THE ROAD IS FOUND
//  ---------------------
//  The scan's materials are called things like "Object5321Mtl" - there is no
//  semantic labelling to key off. What the road surface does have is a
//  distinctive geometric signature, and `--stats` prints it per primitive:
//    * a large area of NEARLY HORIZONTAL triangles,
//    * spread over a footprint that is mostly EMPTY (a ribbon inside its own
//      bounding box), unlike a field, a car park or a roof.
//
//  HOW THE CENTRELINE IS RECOVERED
//  -------------------------------
//  By walking the ribbon. From a seed point, the local direction of travel is
//  the principal axis of the road points nearby; step along it, then re-centre
//  laterally on the road points in a slab across that direction. Repeat until
//  the walk returns to the seed. This follows a closed circuit exactly and
//  needs no skeletonisation.
//
//  USAGE
//      extract_track_centreline <file.glb> --stats
//      extract_track_centreline <file.glb> --trace --prims 3,7 \
//          [--seed X,Z] [--step 4] --out <path.h> --name kNurburgringGp
//===========================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <cgltf.h>

namespace {

struct Vec3 { double x = 0, y = 0, z = 0; };

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s)      { return {a.x * s, a.y * s, a.z * s}; }

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double length(const Vec3& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

/// One triangle of the scan, in world space, tagged with the primitive it
/// came from so `--stats` can report per primitive and `--trace` can select.
struct Triangle {
    Vec3 a, b, c;
    int  prim = 0;
};

//---------------------------------------------------------------------------
//  glTF traversal
//---------------------------------------------------------------------------

Vec3 transformPoint(const cgltf_float m[16], const Vec3& p) {
    return {m[0] * p.x + m[4] * p.y + m[8]  * p.z + m[12],
            m[1] * p.x + m[5] * p.y + m[9]  * p.z + m[13],
            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
}

void collectPrimitive(const cgltf_primitive& prim, const cgltf_float world[16],
                      int primIndex, std::vector<Triangle>& out) {
    if (prim.type != cgltf_primitive_type_triangles || !prim.indices) return;

    const cgltf_accessor* pos = nullptr;
    for (cgltf_size i = 0; i < prim.attributes_count; ++i) {
        if (prim.attributes[i].type == cgltf_attribute_type_position) {
            pos = prim.attributes[i].data;
            break;
        }
    }
    if (!pos) return;

    std::vector<Vec3> points(pos->count);
    for (cgltf_size i = 0; i < pos->count; ++i) {
        cgltf_float v[3] = {0, 0, 0};
        cgltf_accessor_read_float(pos, i, v, 3);
        points[i] = transformPoint(world, {v[0], v[1], v[2]});
    }

    const cgltf_size tris = prim.indices->count / 3;
    for (cgltf_size t = 0; t < tris; ++t) {
        const cgltf_size i0 = cgltf_accessor_read_index(prim.indices, t * 3 + 0);
        const cgltf_size i1 = cgltf_accessor_read_index(prim.indices, t * 3 + 1);
        const cgltf_size i2 = cgltf_accessor_read_index(prim.indices, t * 3 + 2);
        if (i0 >= points.size() || i1 >= points.size() || i2 >= points.size()) continue;
        out.push_back({points[i0], points[i1], points[i2], primIndex});
    }
}

bool loadTriangles(const std::string& path, std::vector<Triangle>& out,
                   int& primitiveCount) {
    cgltf_options options{};
    cgltf_data* data = nullptr;

    if (cgltf_parse_file(&options, path.c_str(), &data) != cgltf_result_success) {
        std::cerr << "cannot parse '" << path << "'\n";
        return false;
    }
    if (cgltf_load_buffers(&options, data, path.c_str()) != cgltf_result_success) {
        std::cerr << "cannot load buffers for '" << path << "'\n";
        cgltf_free(data);
        return false;
    }

    int primIndex = 0;
    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (!node.mesh) continue;

        cgltf_float world[16];
        cgltf_node_transform_world(&node, world);

        for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p) {
            collectPrimitive(node.mesh->primitives[p], world, primIndex, out);
            ++primIndex;
        }
    }
    primitiveCount = primIndex;
    cgltf_free(data);
    return true;
}

//---------------------------------------------------------------------------
//  Stats
//---------------------------------------------------------------------------

struct PrimStats {
    long   triangles = 0;
    double flatArea  = 0;   ///< area of near-horizontal triangles [m^2]
    double totalArea = 0;
    double minX = 1e30, maxX = -1e30;
    double minY = 1e30, maxY = -1e30;
    double minZ = 1e30, maxZ = -1e30;
};

void reportStats(const std::vector<Triangle>& tris, int primitiveCount) {
    std::vector<PrimStats> stats(static_cast<std::size_t>(primitiveCount));

    for (const Triangle& t : tris) {
        PrimStats& s = stats[static_cast<std::size_t>(t.prim)];
        const Vec3 n = cross(t.b - t.a, t.c - t.a);
        const double area = 0.5 * length(n);
        if (!(area > 0.0)) continue;

        s.triangles++;
        s.totalArea += area;
        // Horizontal within about 15 degrees. A racing surface is graded but
        // never steep; a wall or a grandstand tier is not.
        if (std::abs(n.y) / (2.0 * area) > 0.96) s.flatArea += area;

        for (const Vec3& p : {t.a, t.b, t.c}) {
            s.minX = std::min(s.minX, p.x); s.maxX = std::max(s.maxX, p.x);
            s.minY = std::min(s.minY, p.y); s.maxY = std::max(s.maxY, p.y);
            s.minZ = std::min(s.minZ, p.z); s.maxZ = std::max(s.maxZ, p.z);
        }
    }

    std::printf("%5s %9s %12s %12s %7s %9s %9s %8s %6s\n",
                "prim", "tris", "flatArea", "totalArea", "fill%",
                "spanX", "spanZ", "spanY", "flat%");
    for (int i = 0; i < primitiveCount; ++i) {
        const PrimStats& s = stats[static_cast<std::size_t>(i)];
        if (s.triangles == 0) continue;

        const double spanX = s.maxX - s.minX;
        const double spanZ = s.maxZ - s.minZ;
        const double spanY = s.maxY - s.minY;
        const double foot  = spanX * spanZ;

        // A ribbon fills a tiny fraction of its own bounding box. This is the
        // single most discriminating number in the table.
        const double fill = (foot > 1.0) ? (s.flatArea / foot * 100.0) : 0.0;
        const double flatPct = (s.totalArea > 0.0) ? (s.flatArea / s.totalArea * 100.0) : 0.0;

        std::printf("%5d %9ld %12.0f %12.0f %7.2f %9.1f %9.1f %8.1f %6.1f\n",
                    i, s.triangles, s.flatArea, s.totalArea, fill,
                    spanX, spanZ, spanY, flatPct);
    }
}

//---------------------------------------------------------------------------
//  Ribbon walk
//---------------------------------------------------------------------------

struct Pt2 { double x, z; };

/// Road sample points, scattered over the road surface at a roughly uniform
/// DENSITY rather than one per triangle.
///
/// Per-triangle centroids are not good enough: a scanned road is tessellated
/// as long strips, so a 5 km circuit came out as ~3500 points - under one per
/// metre of track - and the ribbon walk starved immediately. Area-weighted
/// sampling gives the walk the same point density everywhere regardless of
/// how the source mesh happens to be cut up.
std::vector<Pt2> roadPoints(const std::vector<Triangle>& tris,
                            const std::vector<int>& prims, double perPoint) {
    std::vector<Pt2> pts;

    for (const Triangle& t : tris) {
        if (std::find(prims.begin(), prims.end(), t.prim) == prims.end()) continue;

        const Vec3 n = cross(t.b - t.a, t.c - t.a);
        const double area = 0.5 * length(n);
        if (!(area > 0.0) || std::abs(n.y) / (2.0 * area) < 0.96) continue;

        const int samples = std::max(1, std::min(4000,
                                static_cast<int>(area / perPoint + 0.5)));

        // Golden-ratio sequence mapped into the triangle. Deterministic, so a
        // re-run of the tool produces exactly the same centreline, and better
        // distributed than random sampling at these counts.
        for (int i = 0; i < samples; ++i) {
            const double u0 = std::fmod(static_cast<double>(i) * 0.7548776662466927, 1.0);
            const double v0 = std::fmod(static_cast<double>(i) * 0.5698402909980532, 1.0);
            const double su = std::sqrt(u0);
            const double u = 1.0 - su;
            const double v = v0 * su;
            const Vec3 p = t.a * u + t.b * v + t.c * (1.0 - u - v);
            pts.push_back({p.x, p.z});
        }
    }
    return pts;
}

/// Uniform grid over the road points, so the slab queries the walk makes are
/// not linear scans of a third of a million points.
struct PointGrid {
    double cell = 8.0;
    double minX = 0, minZ = 0;
    int cols = 0, rows = 0;
    std::vector<std::vector<int>> bucket;
    const std::vector<Pt2>* pts = nullptr;

    void build(const std::vector<Pt2>& p, double cellSize) {
        pts = &p;
        cell = cellSize;
        double maxX = -1e30, maxZ = -1e30;
        minX = minZ = 1e30;
        for (const Pt2& q : p) {
            minX = std::min(minX, q.x); maxX = std::max(maxX, q.x);
            minZ = std::min(minZ, q.z); maxZ = std::max(maxZ, q.z);
        }
        cols = static_cast<int>((maxX - minX) / cell) + 2;
        rows = static_cast<int>((maxZ - minZ) / cell) + 2;
        bucket.assign(static_cast<std::size_t>(cols * rows), {});
        for (int i = 0; i < static_cast<int>(p.size()); ++i) {
            const int cx = static_cast<int>((p[static_cast<std::size_t>(i)].x - minX) / cell);
            const int cz = static_cast<int>((p[static_cast<std::size_t>(i)].z - minZ) / cell);
            bucket[static_cast<std::size_t>(cz * cols + cx)].push_back(i);
        }
    }

    /// Indices of every point within `radius` of (x,z).
    void query(double x, double z, double radius, std::vector<int>& out) const {
        out.clear();
        const int c0 = std::max(0, static_cast<int>((x - radius - minX) / cell));
        const int c1 = std::min(cols - 1, static_cast<int>((x + radius - minX) / cell));
        const int r0 = std::max(0, static_cast<int>((z - radius - minZ) / cell));
        const int r1 = std::min(rows - 1, static_cast<int>((z + radius - minZ) / cell));

        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                for (int idx : bucket[static_cast<std::size_t>(r * cols + c)]) {
                    const Pt2& q = (*pts)[static_cast<std::size_t>(idx)];
                    const double dx = q.x - x, dz = q.z - z;
                    if (dx * dx + dz * dz <= radius * radius) out.push_back(idx);
                }
            }
        }
    }
};

/// Principal direction of a point set about (x,z): the eigenvector of the 2x2
/// covariance with the larger eigenvalue, which for a slab of road points is
/// the direction the road runs.
void principalDirection(const std::vector<Pt2>& pts, const std::vector<int>& idx,
                        double cx, double cz, double& dx, double& dz) {
    double sxx = 0, sxz = 0, szz = 0;
    for (int i : idx) {
        const double ax = pts[static_cast<std::size_t>(i)].x - cx;
        const double az = pts[static_cast<std::size_t>(i)].z - cz;
        sxx += ax * ax; sxz += ax * az; szz += az * az;
    }
    const double tr = sxx + szz;
    const double det = sxx * szz - sxz * sxz;
    const double disc = std::max(tr * tr / 4.0 - det, 0.0);
    const double lambda = tr / 2.0 + std::sqrt(disc);

    if (std::abs(sxz) > 1e-9) {
        dx = lambda - szz;
        dz = sxz;
    } else {
        dx = (sxx >= szz) ? 1.0 : 0.0;
        dz = (sxx >= szz) ? 0.0 : 1.0;
    }
    const double len = std::sqrt(dx * dx + dz * dz);
    if (len > 1e-12) { dx /= len; dz /= len; }
}

struct WalkResult {
    std::vector<Pt2> path;
    bool closed = false;
};

WalkResult walkRibbon(const std::vector<Pt2>& pts, const PointGrid& grid,
                      Pt2 seed, double step, double halfWidth) {
    WalkResult result;

    std::vector<int> local;
    // Initial direction from a generous neighbourhood, so the first step is
    // along the road rather than across it.
    grid.query(seed.x, seed.z, halfWidth * 2.0, local);
    if (local.size() < 8) {
        std::cerr << "seed is not on the road surface (" << local.size() << " points near it)\n";
        return result;
    }
    double dx = 0, dz = 0;
    principalDirection(pts, local, seed.x, seed.z, dx, dz);

    Pt2 p = seed;
    double travelled = 0.0;
    const double maxLength = 40000.0;   // a circuit, not a road network

    while (travelled < maxLength) {
        result.path.push_back(p);

        // Step along the current direction, then re-centre across the road.
        Pt2 next{p.x + dx * step, p.z + dz * step};

        grid.query(next.x, next.z, halfWidth, local);
        if (local.size() < 6) break;      // ran off the end of an open road

        // Lateral mean of the nearby points: the centre of the ribbon.
        const double nx = -dz, nz = dx;
        double lateral = 0.0;
        int used = 0;
        for (int i : local) {
            const double ax = pts[static_cast<std::size_t>(i)].x - next.x;
            const double az = pts[static_cast<std::size_t>(i)].z - next.z;
            // Only points roughly abreast of us, so the slab does not pick up
            // the road ahead or behind through a corner.
            if (std::abs(ax * dx + az * dz) > step * 1.5) continue;
            lateral += ax * nx + az * nz;
            ++used;
        }
        if (used < 6) break;
        lateral /= static_cast<double>(used);

        next.x += nx * lateral;
        next.z += nz * lateral;

        // New direction, smoothed against the old one. The smoothing is what
        // keeps the walk from turning around inside a wide corner or jumping
        // onto an adjacent piece of tarmac.
        grid.query(next.x, next.z, halfWidth * 1.4, local);
        double ndx = dx, ndz = dz;
        if (local.size() >= 8) {
            principalDirection(pts, local, next.x, next.z, ndx, ndz);
            if (ndx * dx + ndz * dz < 0.0) { ndx = -ndx; ndz = -ndz; }
        }
        const double blend = 0.45;
        dx = dx * (1.0 - blend) + ndx * blend;
        dz = dz * (1.0 - blend) + ndz * blend;
        const double len = std::sqrt(dx * dx + dz * dz);
        if (len < 1e-9) break;
        dx /= len; dz /= len;

        travelled += step;
        p = next;

        // Closed the loop?
        if (travelled > 400.0) {
            const double ddx = p.x - seed.x, ddz = p.z - seed.z;
            if (ddx * ddx + ddz * ddz < step * step * 4.0) {
                result.closed = true;
                break;
            }
        }
    }
    return result;
}

/// Light smoothing pass. The walk's re-centring is a mean over a finite slab
/// and therefore noisy at the metre scale; a circuit centreline is not.
void smooth(std::vector<Pt2>& path, bool closed, int passes) {
    const int n = static_cast<int>(path.size());
    if (n < 5) return;

    for (int pass = 0; pass < passes; ++pass) {
        std::vector<Pt2> out = path;
        for (int i = 0; i < n; ++i) {
            const int a = closed ? (i - 1 + n) % n : std::max(i - 1, 0);
            const int b = closed ? (i + 1) % n     : std::min(i + 1, n - 1);
            out[static_cast<std::size_t>(i)].x =
                0.25 * path[static_cast<std::size_t>(a)].x +
                0.50 * path[static_cast<std::size_t>(i)].x +
                0.25 * path[static_cast<std::size_t>(b)].x;
            out[static_cast<std::size_t>(i)].z =
                0.25 * path[static_cast<std::size_t>(a)].z +
                0.50 * path[static_cast<std::size_t>(i)].z +
                0.25 * path[static_cast<std::size_t>(b)].z;
        }
        path.swap(out);
    }
}

/// Resample to roughly even spacing, and drop points that add nothing - a
/// straight needs two points, not two hundred.
std::vector<Pt2> resample(const std::vector<Pt2>& path, bool closed, double spacing) {
    std::vector<Pt2> out;
    if (path.size() < 2) return out;

    const int n = static_cast<int>(path.size());
    double carry = 0.0;
    out.push_back(path.front());

    const int limit = closed ? n : n - 1;
    for (int i = 0; i < limit; ++i) {
        const Pt2& a = path[static_cast<std::size_t>(i)];
        const Pt2& b = path[static_cast<std::size_t>((i + 1) % n)];
        const double seg = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
        if (seg < 1e-9) continue;

        double t = -carry;
        while (t + spacing <= seg) {
            t += spacing;
            out.push_back({a.x + (b.x - a.x) * t / seg, a.z + (b.z - a.z) * t / seg});
        }
        carry = seg - t;
    }
    return out;
}

/// Rasterise the extracted road points (and optionally a traced path) to a
/// PPM. Purely a diagnostic: choosing the right primitives and a sane seed is
/// a judgement call about a 91 MB scan nobody has looked at, and looking at
/// it is much faster than inferring it from numbers.
void writeImage(const std::string& path, const std::vector<Pt2>& pts,
                const std::vector<Pt2>& overlay, double metresPerPixel) {
    if (pts.empty()) return;

    double minX = 1e30, maxX = -1e30, minZ = 1e30, maxZ = -1e30;
    for (const Pt2& p : pts) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minZ = std::min(minZ, p.z); maxZ = std::max(maxZ, p.z);
    }
    const int w = static_cast<int>((maxX - minX) / metresPerPixel) + 3;
    const int h = static_cast<int>((maxZ - minZ) / metresPerPixel) + 3;
    if (w < 2 || h < 2 || static_cast<long>(w) * h > 40000000L) return;

    std::vector<unsigned char> img(static_cast<std::size_t>(w * h * 3), 20);

    auto plot = [&](const Pt2& p, unsigned char r, unsigned char g, unsigned char b, int rad) {
        const int cx = static_cast<int>((p.x - minX) / metresPerPixel) + 1;
        const int cy = static_cast<int>((p.z - minZ) / metresPerPixel) + 1;
        for (int dy = -rad; dy <= rad; ++dy) {
            for (int dx = -rad; dx <= rad; ++dx) {
                const int x = cx + dx, y = cy + dy;
                if (x < 0 || y < 0 || x >= w || y >= h) continue;
                const std::size_t o = static_cast<std::size_t>((y * w + x) * 3);
                img[o] = r; img[o + 1] = g; img[o + 2] = b;
            }
        }
    };

    for (const Pt2& p : pts) plot(p, 200, 200, 200, 0);
    for (const Pt2& p : overlay) plot(p, 255, 60, 40, 1);

    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << w << " " << h << "\n255\n";
    out.write(reinterpret_cast<const char*>(img.data()),
              static_cast<std::streamsize>(img.size()));
    std::cout << "wrote " << path << " (" << w << "x" << h << " px, "
              << metresPerPixel << " m/px, origin " << minX << "," << minZ << ")\n";
}

int writeHeader(const std::string& path, const std::string& name,
                const std::vector<Pt2>& pts, bool closed, double length,
                const std::string& source) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "cannot write '" << path << "'\n";
        return 1;
    }

    out << "#pragma once\n"
        << "//===================================================================="
           "=======\n"
        << "//  GENERATED FILE - do not edit by hand.\n"
        << "//\n"
        << "//  Circuit centreline recovered from a reference scan by\n"
        << "//  tools/extract_track_centreline.cpp. See that file for what the\n"
        << "//  tool does and why the scan itself is not shipped.\n"
        << "//\n"
        << "//  source      : " << source << "\n"
        << "//  waypoints   : " << pts.size() << "\n"
        << "//  closed loop : " << (closed ? "yes" : "no") << "\n"
        << "//  lap length  : " << static_cast<long>(length + 0.5) << " m\n"
        << "//\n"
        << "//  Coordinates are metres in the simulator's world frame (X east,\n"
        << "//  Z south - see core/Coordinates.h), recentred on the centroid of\n"
        << "//  the layout. Elevation is deliberately NOT carried: the vehicle\n"
        << "//  model runs on flat ground for every generated track, and a\n"
        << "//  centreline that rises and falls where the physics does not would\n"
        << "//  be a lie told by the renderer.\n"
        << "//====================================================================="
           "======\n\n"
        << "namespace vsim::trackdata {\n\n"
        << "/// (x, z) waypoint pairs, in metres.\n"
        << "inline constexpr double " << name << "[][2] = {\n";

    out.setf(std::ios::fixed);
    out.precision(2);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        out << "    {" << pts[i].x << ", " << pts[i].z << "},";
        if ((i % 4) == 3) out << "\n";
    }
    out << "\n};\n\n"
        << "inline constexpr bool " << name << "Closed = " << (closed ? "true" : "false")
        << ";\n\n"
        << "} // namespace vsim::trackdata\n";

    std::cout << "wrote " << path << " (" << pts.size() << " waypoints, "
              << static_cast<long>(length + 0.5) << " m)\n";
    return 0;
}

void printUsage() {
    std::cout <<
        "extract_track_centreline <file.glb> [--stats | --trace] [options]\n"
        "\n"
        "  --stats              per-primitive geometry report; use it to find\n"
        "                       the road surface (large flat area, low fill%)\n"
        "  --trace              recover the centreline and emit a header\n"
        "  --prims a,b,c        primitive indices making up the road surface\n"
        "  --seed X,Z           start the walk here (default: a road point\n"
        "                       near the middle of the layout)\n"
        "  --step N             walk step in metres (default 4)\n"
        "  --width N            expected half width of the road, metres\n"
        "                       (default 9)\n"
        "  --spacing N          output waypoint spacing, metres (default 25)\n"
        "  --out PATH           header to write\n"
        "  --name IDENT         array name in the generated header\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { printUsage(); return 1; }

    const std::string file = argv[1];
    bool stats = false, trace = false;
    std::vector<int> prims;
    double step = 4.0, halfWidth = 9.0, spacing = 25.0;
    bool haveSeed = false;
    Pt2 seed{0, 0};
    std::string outPath = "centreline.h";
    std::string name = "kCentreline";
    std::string imagePath;
    double imageScale = 2.0;

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };

        if (a == "--stats") stats = true;
        else if (a == "--trace") trace = true;
        else if (a == "--step") step = std::atof(next().c_str());
        else if (a == "--width") halfWidth = std::atof(next().c_str());
        else if (a == "--spacing") spacing = std::atof(next().c_str());
        else if (a == "--out") outPath = next();
        else if (a == "--name") name = next();
        else if (a == "--image") imagePath = next();
        else if (a == "--image-scale") imageScale = std::atof(next().c_str());
        else if (a == "--seed") {
            const std::string v = next();
            const std::size_t comma = v.find(',');
            if (comma != std::string::npos) {
                seed.x = std::atof(v.substr(0, comma).c_str());
                seed.z = std::atof(v.substr(comma + 1).c_str());
                haveSeed = true;
            }
        } else if (a == "--prims") {
            std::string v = next();
            std::size_t pos = 0;
            while (!v.empty()) {
                pos = v.find(',');
                prims.push_back(std::atoi(v.substr(0, pos).c_str()));
                if (pos == std::string::npos) break;
                v = v.substr(pos + 1);
            }
        } else {
            std::cerr << "unknown option '" << a << "'\n";
            return 1;
        }
    }

    std::vector<Triangle> tris;
    int primitiveCount = 0;
    std::cout << "loading " << file << " ...\n";
    if (!loadTriangles(file, tris, primitiveCount)) return 1;
    std::cout << tris.size() << " triangles across " << primitiveCount << " primitives\n\n";

    if (stats || (!stats && !trace)) {
        reportStats(tris, primitiveCount);

        // One picture per candidate primitive, from a single load of the
        // file. Identifying the racing surface in an unlabelled scan is a
        // "look at it" problem, and the table above only narrows the field.
        if (!imagePath.empty()) {
            for (int p = 0; p < primitiveCount; ++p) {
                const std::vector<Pt2> pp = roadPoints(tris, {p}, 6.0);
                if (pp.size() < 400) continue;
                writeImage(imagePath + "/prim" + std::to_string(p) + ".ppm",
                           pp, {}, imageScale);
            }
        }
        if (!trace) return 0;
    }

    if (prims.empty()) {
        std::cerr << "--trace needs --prims (run --stats first)\n";
        return 1;
    }

    const std::vector<Pt2> pts = roadPoints(tris, prims, 4.0);
    std::cout << "road surface: " << pts.size() << " sample points\n";
    if (pts.size() < 100) {
        std::cerr << "not enough road points to trace\n";
        return 1;
    }

    PointGrid grid;
    grid.build(pts, std::max(halfWidth, 6.0));

    if (!haveSeed) {
        // Default seed: the road point closest to the centroid of the road
        // itself, which is on the surface by construction.
        double cx = 0, cz = 0;
        for (const Pt2& p : pts) { cx += p.x; cz += p.z; }
        cx /= static_cast<double>(pts.size());
        cz /= static_cast<double>(pts.size());

        double best = 1e30;
        for (const Pt2& p : pts) {
            const double d = (p.x - cx) * (p.x - cx) + (p.z - cz) * (p.z - cz);
            if (d < best) { best = d; seed = p; }
        }
        std::cout << "seed (auto): " << seed.x << ", " << seed.z << "\n";
    }

    WalkResult walk = walkRibbon(pts, grid, seed, step, halfWidth);
    std::cout << "walk: " << walk.path.size() << " steps, "
              << static_cast<long>(walk.path.size() * step) << " m, "
              << (walk.closed ? "CLOSED loop" : "open (did not return to seed)") << "\n";

    if (!imagePath.empty()) writeImage(imagePath, pts, walk.path, imageScale);

    if (walk.path.size() < 20) {
        std::cerr << "walk failed - try a different --seed or --width\n";
        return 1;
    }

    smooth(walk.path, walk.closed, 6);
    std::vector<Pt2> out = resample(walk.path, walk.closed, spacing);

    // Recentre on the layout's own centroid so the track sits around the
    // world origin instead of wherever the scan happened to be authored.
    double cx = 0, cz = 0;
    for (const Pt2& p : out) { cx += p.x; cz += p.z; }
    cx /= static_cast<double>(out.size());
    cz /= static_cast<double>(out.size());

    double length = 0.0;
    for (std::size_t i = 0; i < out.size(); ++i) {
        const Pt2& a = out[i];
        const Pt2& b = out[(i + 1) % out.size()];
        if (!walk.closed && i + 1 == out.size()) break;
        length += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
    }
    for (Pt2& p : out) { p.x -= cx; p.z -= cz; }

    return writeHeader(outPath, name, out, walk.closed, length, file);
}
