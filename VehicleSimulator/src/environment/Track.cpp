#include "environment/Track.h"

#include <cmath>
#include <iostream>
#include <limits>

#include "core/Coordinates.h"
#include "core/Units.h"
#include "environment/data/NordschleifeCentreline.h"
#include "environment/data/NurburgringGpCentreline.h"

namespace vsim {

//---------------------------------------------------------------------------
//  Construction helpers
//
//  The track is built by walking a cursor. Each segment advances the cursor
//  and appends points at roughly `stepLength` spacing. Curvature is recorded
//  per point so the renderer can taper the centre line markings and a future
//  physics model could query banking or grip per segment.
//---------------------------------------------------------------------------

void Track::appendStraight(double length, double stepLength) {
    const int steps = std::max(1, static_cast<int>(std::ceil(length / stepLength)));
    const double ds = length / steps;

    const glm::dvec3 dir = coords::heading(m_heading);

    for (int i = 0; i < steps; ++i) {
        TrackPoint p;
        p.position  = m_cursor;
        p.tangent   = dir;
        p.distance  = m_distance;
        p.curvature = 0.0;
        m_points.push_back(p);

        m_cursor   += dir * ds;
        m_distance += ds;
    }
}

void Track::appendArc(double radius, double sweepRadians, double stepLength) {
    if (std::abs(radius) < 1e-6) return;

    const double arcLength = std::abs(radius * sweepRadians);
    const int steps = std::max(2, static_cast<int>(std::ceil(arcLength / stepLength)));
    const double dTheta = sweepRadians / steps;
    const double ds     = arcLength / steps;

    // Sign convention matches the vehicle: a positive sweep increases yaw,
    // which turns toward +Z, which is a RIGHT turn. Using the same rule here
    // as in Coordinates.h means a track curve and a vehicle turn of the same
    // sign bend the same way.
    const double curvature = (sweepRadians >= 0.0) ? (1.0 / radius) : (-1.0 / radius);

    for (int i = 0; i < steps; ++i) {
        const glm::dvec3 dir = coords::heading(m_heading);

        TrackPoint p;
        p.position  = m_cursor;
        p.tangent   = dir;
        p.distance  = m_distance;
        p.curvature = curvature;
        m_points.push_back(p);

        // Advance along the chord, then rotate. For the step sizes used here
        // (~1 m on radii of 40 m+) the chord/arc difference is well under a
        // centimetre, which is invisible and keeps the code simple.
        m_cursor   += dir * ds;
        m_heading   = coords::wrapAngle(m_heading + dTheta);
        m_distance += ds;
    }
}

void Track::appendCorner(double radius, double sweepRadians, double stepLength,
                         double transition) {
    if (std::abs(radius) < 1e-6 || std::abs(sweepRadians) < 1e-9) return;

    const double sign  = (sweepRadians >= 0.0) ? 1.0 : -1.0;
    const double sweep = std::abs(sweepRadians);
    const double kMax  = 1.0 / std::abs(radius);

    // Each ramp of length Lt turns the heading by kMax * Lt / 2, so the
    // fraction `transition` of the sweep per ramp fixes Lt; the rest of the
    // sweep is the constant-radius middle.
    const double Lt = std::min(2.0 * transition * sweep / kMax, sweep / kMax);
    const double Lc = sweep / kMax - Lt;
    const double length = Lc + 2.0 * Lt;

    const int steps = std::max(4, static_cast<int>(std::ceil(length / stepLength)));
    const double ds = length / steps;

    for (int i = 0; i < steps; ++i) {
        const double s = (static_cast<double>(i) + 0.5) * ds;
        double k = kMax;
        if (Lt > 1e-9) k *= std::min(1.0, std::min(s / Lt, (length - s) / Lt));
        k *= sign;

        TrackPoint p;
        p.position  = m_cursor;
        p.tangent   = coords::heading(m_heading);
        p.distance  = m_distance;
        p.curvature = k;
        m_points.push_back(p);

        // Midpoint rule: advance along the heading at the middle of the step.
        m_heading += 0.5 * k * ds;
        m_cursor  += coords::heading(m_heading) * ds;
        m_heading  = coords::wrapAngle(m_heading + 0.5 * k * ds);
        m_distance += ds;
    }
}

void Track::finalise() {
    if (m_points.empty()) return;

    // Cap the ground plane generously around the track's bounding box so the
    // world does not visibly end while driving.
    glm::dvec3 lo(std::numeric_limits<double>::max());
    glm::dvec3 hi(std::numeric_limits<double>::lowest());
    for (const TrackPoint& p : m_points) {
        lo = glm::min(lo, p.position);
        hi = glm::max(hi, p.position);
    }
    const double spanX = hi.x - lo.x;
    const double spanZ = hi.z - lo.z;
    m_groundExtent = std::max(200.0, 0.75 * std::max(spanX, spanZ) + 150.0);
}

//---------------------------------------------------------------------------
//  Presets
//---------------------------------------------------------------------------

Track Track::createStraight(double length, double width) {
    Track t;
    t.m_width = width;
    t.appendStraight(length, 2.0);
    t.finalise();
    return t;
}

Track Track::createTestTrack() {
    Track t;
    t.m_width = 9.0;

    constexpr double kStep = 1.5;   // centreline sample spacing [m]

    // Start at the world origin heading along +X, which is yaw = 0 - the same
    // heading the vehicle has at its default initial state, so the car starts
    // on the road pointing down it.
    t.m_cursor  = {0.0, 0.0, 0.0};
    t.m_heading = 0.0;

    //  1. Long straight - acceleration and braking runs.
    t.appendStraight(220.0, kStep);

    //  2. Wide right-hand sweeper, 90 degrees at 70 m radius.
    //     At 70 m radius, 0.3 g lateral (the kinematic model's rough validity
    //     ceiling) corresponds to about 14 m/s / 50 km/h.
    t.appendArc(70.0, degToRad(90.0), kStep);

    //  3. Short straight.
    t.appendStraight(90.0, kStep);

    //  4. Tighter right-hander, 90 degrees at 45 m radius.
    t.appendArc(45.0, degToRad(90.0), kStep);

    //  5. Slalom: alternating 40 m radius arcs, 60 degrees each.
    t.appendStraight(40.0, kStep);
    for (int i = 0; i < 4; ++i) {
        const double sweep = (i % 2 == 0) ? degToRad(-60.0) : degToRad(60.0);
        t.appendArc(40.0, sweep, kStep);
    }
    t.appendStraight(40.0, kStep);

    //  6. Return leg back toward the start.
    t.appendArc(60.0, degToRad(90.0), kStep);
    t.appendStraight(180.0, kStep);
    t.appendArc(60.0, degToRad(90.0), kStep);
    t.appendStraight(120.0, kStep);

    t.finalise();
    return t;
}

void Track::appendCircuitHalf(double stepLength) {
    //  Five corners of varying radius and direction, sweeps summing to
    //  exactly +180 degrees - see createRaceCircuit() for why that total,
    //  not the individual values, is what matters.
    appendStraight(300.0, stepLength);          // start/finish or back straight
    appendArc(120.0, degToRad( 25.0), stepLength);  // T1: fast right kink
    appendStraight(80.0, stepLength);
    appendArc( 90.0, degToRad(-20.0), stepLength);  // T2: left-hander
    appendStraight(40.0, stepLength);
    appendArc( 70.0, degToRad( 55.0), stepLength);  // T3: right, tightening
    appendStraight(60.0, stepLength);
    appendArc( 50.0, degToRad( 70.0), stepLength);  // T4: tight right hairpin
    appendStraight(50.0, stepLength);
    appendArc( 90.0, degToRad( 50.0), stepLength);  // T5: right, opening up
    // 25 - 20 + 55 + 70 + 50 = 180.
}

Track Track::createRaceCircuit() {
    Track t;
    t.m_width  = 12.0;   // wide enough for side-by-side racing, not just a lane
    t.m_closed = true;

    constexpr double kStep = 1.5;

    t.m_cursor  = {0.0, 0.0, 0.0};
    t.m_heading = 0.0;

    //-------------------------------------------------------------------
    //  CLOSING THE LOOP EXACTLY, NOT APPROXIMATELY
    //
    //  A race circuit has to reconnect to its own start/finish line - a
    //  point-to-point route like createTestTrack() does not need to. Rather
    //  than hand-tuning enough segments to land back on (0,0) at heading 0
    //  by trial and error, this exploits a symmetry that guarantees it:
    //
    //  If a sequence of straights and arcs turns the heading by exactly
    //  +180 degrees in total (any mix of corners, any order - only the sum
    //  matters, since each segment's world-frame displacement depends on
    //  the heading *at that point*), then walking the IDENTICAL sequence a
    //  second time, starting from wherever the first one ended, produces a
    //  displacement that is the exact negative of the first: every segment
    //  is now executed at a heading rotated 180 degrees from its first
    //  occurrence, which negates its contribution to position. Two such
    //  halves therefore always meet back at the start, facing the original
    //  direction (180 + 180 = 360). This is the same principle that makes a
    //  simple oval close - a straight plus a 180 degree arc, repeated once -
    //  generalised to an arbitrary corner sequence, which is what turns the
    //  oval into something that reads as an actual circuit.
    //-------------------------------------------------------------------
    t.appendCircuitHalf(kStep);
    t.appendCircuitHalf(kStep);

    t.finalise();
    return t;
}

//---------------------------------------------------------------------------
//  Waypoint tracks
//---------------------------------------------------------------------------

namespace {

/// Centripetal Catmull-Rom (alpha = 0.5) through p1 and p2, in Barry-Goldman
/// form. Unlike the uniform variant this cannot cusp or loop between control
/// points however unevenly they are spaced, which is what makes it the right
/// choice for waypoints measured off a real layout.
///
/// Packed as glm::dvec3(x, z, elevation) rather than a plain 2D point so a
/// track's elevation profile, when it has one, rides through the IDENTICAL
/// parameterisation as its ground-plane shape instead of a parallel,
/// easier-to-desync one - .y here is ground-plane Z, not world up. Knotted
/// on the (x, z) distance only (elevation never affects the knot spacing),
/// so a hill neither stretches nor compresses the curve fitting; a flat
/// track (elevation always 0) is unaffected either way.
glm::dvec3 catmullRomXZY(const glm::dvec3& p0, const glm::dvec3& p1,
                         const glm::dvec3& p2, const glm::dvec3& p3, double t) {
    auto xz = [](const glm::dvec3& p) { return glm::dvec2(p.x, p.y); };
    auto knot = [&](double ti, const glm::dvec3& a, const glm::dvec3& b) {
        return ti + std::pow(glm::length(xz(b) - xz(a)), 0.5);
    };
    const double t0 = 0.0;
    const double t1 = knot(t0, p0, p1);
    const double t2 = knot(t1, p1, p2);
    const double t3 = knot(t2, p2, p3);

    if (t1 - t0 < 1e-9 || t2 - t1 < 1e-9 || t3 - t2 < 1e-9) {
        return p1 + (p2 - p1) * t;
    }

    const double tt = t1 + (t2 - t1) * t;
    const glm::dvec3 a1 = p0 * ((t1 - tt) / (t1 - t0)) + p1 * ((tt - t0) / (t1 - t0));
    const glm::dvec3 a2 = p1 * ((t2 - tt) / (t2 - t1)) + p2 * ((tt - t1) / (t2 - t1));
    const glm::dvec3 a3 = p2 * ((t3 - tt) / (t3 - t2)) + p3 * ((tt - t2) / (t3 - t2));
    const glm::dvec3 b1 = a1 * ((t2 - tt) / (t2 - t0)) + a2 * ((tt - t0) / (t2 - t0));
    const glm::dvec3 b2 = a2 * ((t3 - tt) / (t3 - t1)) + a3 * ((tt - t1) / (t3 - t1));
    return b1 * ((t2 - tt) / (t2 - t1)) + b2 * ((tt - t1) / (t2 - t1));
}

} // namespace

Track Track::fromWaypoints(const std::vector<glm::dvec2>& waypoints,
                           double width, bool closed, double stepLength,
                           const std::vector<double>* elevations) {
    Track t;
    t.m_width  = width;
    t.m_closed = closed;

    const int n = static_cast<int>(waypoints.size());
    if (n < 4) {
        std::cerr << "[environment] fromWaypoints needs at least 4 waypoints; "
                  << "falling back to a straight\n";
        return createStraight(500.0, width);
    }
    if (elevations && elevations->size() != waypoints.size()) {
        std::cerr << "[environment] fromWaypoints: elevations size does not match "
                  << "waypoints (" << elevations->size() << " vs " << waypoints.size()
                  << "); ignoring elevation\n";
        elevations = nullptr;
    }
    t.m_hasElevation = (elevations != nullptr);

    auto wrapIndex = [&](int i) {
        return closed ? (i % n + n) % n : std::min(std::max(i, 0), n - 1);
    };
    // Packed as (x, z, elevation) - see catmullRomXZY - so elevation, when
    // given, rides through the identical spline/resample pipeline as the
    // ground-plane shape below instead of a parallel, easier-to-desync one.
    auto wp3 = [&](int i) -> glm::dvec3 {
        const glm::dvec2& p = waypoints[static_cast<std::size_t>(wrapIndex(i))];
        const double e = elevations ? (*elevations)[static_cast<std::size_t>(wrapIndex(i))] : 0.0;
        return {p.x, p.y, e};
    };
    auto xz = [](const glm::dvec3& v) { return glm::dvec2(v.x, v.y); };

    // 1. Sample the spline densely. The samples are not evenly spaced in arc
    //    length yet - a spline parameter is not arc length - so this is only
    //    an intermediate, dense polyline.
    constexpr int kSubSamples = 24;
    std::vector<glm::dvec3> dense;
    dense.reserve(static_cast<std::size_t>(n * kSubSamples));

    const int spans = closed ? n : n - 1;
    for (int i = 0; i < spans; ++i) {
        for (int s = 0; s < kSubSamples; ++s) {
            const double u = static_cast<double>(s) / kSubSamples;
            dense.push_back(catmullRomXZY(wp3(i - 1), wp3(i), wp3(i + 1), wp3(i + 2), u));
        }
    }
    if (!closed) dense.push_back(wp3(n - 1));

    // 2. Resample at uniform arc length. Every downstream consumer - the dash
    //    pattern, the kerb blocks, the distance readout - assumes points are
    //    evenly spaced along the road. Arc length is measured on the ground
    //    plane (x, z) only: a published lap length is a map distance, and
    //    the slope correction for anything this project's terrain looks
    //    like is well under the resampling error anyway.
    const int denseCount = static_cast<int>(dense.size());
    double carry = 0.0;
    std::vector<glm::dvec3> even;
    even.push_back(dense.front());

    const int denseSpans = closed ? denseCount : denseCount - 1;
    for (int i = 0; i < denseSpans; ++i) {
        const glm::dvec3& a = dense[static_cast<std::size_t>(i)];
        const glm::dvec3& b = dense[static_cast<std::size_t>((i + 1) % denseCount)];
        const double seg = glm::length(xz(b) - xz(a));
        if (seg < 1e-9) continue;

        double d = -carry;
        while (d + stepLength <= seg) {
            d += stepLength;
            even.push_back(a + (b - a) * (d / seg));
        }
        carry = seg - d;
    }
    // On a closed loop the last sample can land almost on top of the first;
    // keeping both would give one zero-length segment with an undefined
    // tangent.
    if (closed && even.size() > 2 &&
        glm::length(xz(even.back()) - xz(even.front())) < stepLength * 0.5) {
        even.pop_back();
    }

    // 3. Tangents and curvature from the resampled line. Central differences
    //    throughout, so both are consistent with the geometry actually drawn
    //    rather than with the spline that produced it. Tangent stays purely
    //    horizontal even with elevation - see TrackPoint::tangent - so yaw
    //    and curvature below are completely unaffected by it.
    const int count = static_cast<int>(even.size());
    t.m_points.reserve(static_cast<std::size_t>(count));

    double distance = 0.0;
    for (int i = 0; i < count; ++i) {
        const int prev = closed ? (i - 1 + count) % count : std::max(i - 1, 0);
        const int next = closed ? (i + 1) % count : std::min(i + 1, count - 1);

        const glm::dvec2 d = xz(even[static_cast<std::size_t>(next)]) -
                             xz(even[static_cast<std::size_t>(prev)]);
        const double len = glm::length(d);

        TrackPoint p;
        p.position = {even[static_cast<std::size_t>(i)].x,
                      even[static_cast<std::size_t>(i)].z,
                      even[static_cast<std::size_t>(i)].y};
        p.tangent  = (len > 1e-9) ? glm::dvec3(d.x / len, 0.0, d.y / len)
                                  : glm::dvec3(1.0, 0.0, 0.0);
        p.distance = distance;
        t.m_points.push_back(p);

        if (i + 1 < count || closed) {
            distance += glm::length(xz(even[static_cast<std::size_t>(next)]) -
                                    xz(even[static_cast<std::size_t>(i)]));
        }
    }

    // Curvature as the turning rate of the tangent: kappa = d(yaw)/ds, signed
    // by the same convention as appendArc() - positive turns right.
    for (int i = 0; i < count; ++i) {
        const int prev = closed ? (i - 1 + count) % count : std::max(i - 1, 0);
        const int next = closed ? (i + 1) % count : std::min(i + 1, count - 1);

        const double yawPrev = coords::yawFromDirection(
            t.m_points[static_cast<std::size_t>(prev)].tangent);
        const double yawNext = coords::yawFromDirection(
            t.m_points[static_cast<std::size_t>(next)].tangent);

        const double dYaw = coords::wrapAngle(yawNext - yawPrev);
        const double ds = 2.0 * stepLength;
        t.m_points[static_cast<std::size_t>(i)].curvature = dYaw / ds;
    }

    // A measured centreline carries sampling noise that a hand-built one does
    // not, and curvature is a second derivative, so it is the noisiest thing
    // here. The renderer keys kerbs and gravel traps off it, and unsmoothed
    // it would scatter them along the straights.
    std::vector<double> smoothed(static_cast<std::size_t>(count));
    constexpr int kWindow = 4;
    for (int i = 0; i < count; ++i) {
        double sum = 0.0;
        int used = 0;
        for (int w = -kWindow; w <= kWindow; ++w) {
            int j = i + w;
            if (closed) j = (j % count + count) % count;
            else if (j < 0 || j >= count) continue;
            sum += t.m_points[static_cast<std::size_t>(j)].curvature;
            ++used;
        }
        smoothed[static_cast<std::size_t>(i)] = sum / static_cast<double>(used);
    }
    for (int i = 0; i < count; ++i) {
        t.m_points[static_cast<std::size_t>(i)].curvature =
            smoothed[static_cast<std::size_t>(i)];
    }

    // Elevation gets the identical treatment, for a related but distinct
    // reason: a DEM is sampled on its own grid (tens of metres), independent
    // of where the road's waypoints happen to fall, so two waypoints that
    // are close together in reality (a tightly-curved stretch the source
    // digitised densely) can land in adjacent DEM cells with a real but
    // abrupt step between them - the interpolating spline above reproduces
    // that step faithfully rather than smoothing it away, because it has no
    // way to tell "the road actually does that" from "the two heights it
    // was given for two nearby points do not agree". Averaged AFTER
    // resampling, over a fixed few metres of arc length rather than a fixed
    // number of (unevenly spaced) source waypoints, which is what makes this
    // robust to exactly that unevenness.
    if (elevations) {
        std::vector<double> smoothedY(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            double sum = 0.0;
            int used = 0;
            for (int w = -kWindow; w <= kWindow; ++w) {
                int j = i + w;
                if (closed) j = (j % count + count) % count;
                else if (j < 0 || j >= count) continue;
                sum += t.m_points[static_cast<std::size_t>(j)].position.y;
                ++used;
            }
            smoothedY[static_cast<std::size_t>(i)] = sum / static_cast<double>(used);
        }
        for (int i = 0; i < count; ++i) {
            t.m_points[static_cast<std::size_t>(i)].position.y =
                smoothedY[static_cast<std::size_t>(i)];
        }
    }

    t.finalise();
    return t;
}

Track Track::createNurburgringGp() {
    std::vector<glm::dvec2> waypoints;
    const std::size_t count = sizeof(trackdata::kNurburgringGp) /
                              sizeof(trackdata::kNurburgringGp[0]);
    waypoints.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        waypoints.push_back({trackdata::kNurburgringGp[i][0],
                             trackdata::kNurburgringGp[i][1]});
    }
    // 13 m: the measured road surface area of the reference scan divided by
    // the measured lap length. Not a guess, and close to the 12-15 m a modern
    // Grand Prix circuit actually is.
    return fromWaypoints(waypoints, 13.0, trackdata::kNurburgringGpClosed);
}

Track Track::createNordschleife() {
    std::vector<glm::dvec2> waypoints;
    std::vector<double>     elevations;
    const std::size_t count = sizeof(trackdata::kNordschleife) /
                              sizeof(trackdata::kNordschleife[0]);
    waypoints.reserve(count);
    elevations.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        waypoints.push_back({trackdata::kNordschleife[i][0],
                             trackdata::kNordschleife[i][1]});
        elevations.push_back(trackdata::kNordschleife[i][2]);
    }
    // 9 m: the Nordschleife is a public toll road first and a circuit
    // second, and it is narrower than a purpose-built Grand Prix track for
    // almost its entire 20.8 km - published sources put most of it at 8-9 m
    // kerb to kerb, only a little over the 8.5-9 m car this project's own
    // GT3 preset models. Left uniform for the same reason every other
    // preset here is: TrackPoint carries one width for the whole lap, not a
    // per-metre profile, and the alternative is inventing numbers the
    // source data does not have.
    return fromWaypoints(waypoints, 9.0, trackdata::kNordschleifeClosed, 1.5, &elevations);
}

Track Track::createPlaceholder() {
    // Not drawn - see Renderer, which suppresses the procedural road/ground
    // meshes whenever an environment model is active. This only backs
    // nearestPoint()/lateralOffset() and the ground extent used before the
    // model (and the TerrainHeightField-derived extent that replaces it)
    // has finished loading.
    return createStraight(50.0, 12.0);
}

Track Track::createByName(const std::string& name) {
    Track t = createByNameImpl(name);
    t.m_id = name;
    return t;
}

Track Track::createByNameImpl(const std::string& name) {
    if (name == "test_track" || name.empty()) return createTestTrack();
    if (name == "race_circuit") return createRaceCircuit();
    if (name == "straight") return createStraight(500.0, 9.0);
    if (name == "nurburgring") return createNurburgringGp();
    if (name == "nordschleife") return createNordschleife();
    if (name == "apex_gp") return createApexGrandPrix();

    // The raw reference scan, drawn as imported geometry with no centreline.
    // Kept because it is the only track with no centreline of its own to
    // build a TerrainHeightField FROM - createNordschleife() now also has
    // real elevation, but generates its terrain from that centreline (see
    // Renderer::buildTrackTerrain) rather than needing an imported scan.
    if (name == "nurburgring_scan") return createPlaceholder();

    std::cerr << "[environment] unknown track '" << name
              << "'; falling back to test_track\n";
    return createTestTrack();
}

std::vector<std::string> Track::availableTracks() {
    return {"test_track", "race_circuit", "apex_gp", "nurburgring", "nordschleife", "straight", "nurburgring_scan"};
}

//---------------------------------------------------------------------------
//  Queries
//---------------------------------------------------------------------------

std::size_t Track::nearestPoint(const glm::dvec3& worldPos) const {
    std::size_t best = 0;
    double bestDistSq = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < m_points.size(); ++i) {
        const glm::dvec3 d = m_points[i].position - worldPos;
        const double distSq = d.x * d.x + d.z * d.z;   // ground plane only
        if (distSq < bestDistSq) { bestDistSq = distSq; best = i; }
    }
    return best;
}

double Track::lateralOffset(const glm::dvec3& worldPos) const {
    if (m_points.empty()) return 0.0;

    const TrackPoint& p = m_points[nearestPoint(worldPos)];
    const glm::dvec3 toVehicle = worldPos - p.position;

    // Right-hand normal of the tangent, using the same forward x up = right
    // rule as the vehicle frame.
    const double yaw = coords::yawFromDirection(p.tangent);
    const glm::dvec3 rightVec = coords::right(yaw);

    return glm::dot(toVehicle, rightVec);
}

} // namespace vsim
