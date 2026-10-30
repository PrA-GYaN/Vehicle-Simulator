#pragma once
//===========================================================================
//  Track.h  --  the test environment, as pure geometry data.
//
//  This file produces a centreline polyline and nothing else. It contains no
//  OpenGL, no vertex buffers and no colours. The renderer turns this data
//  into meshes; the environment does not know the renderer exists.
//
//  It is also completely independent of vehicle physics: the track does not
//  push the car, and driving off it has no effect. Surface friction, banking
//  and terrain height would attach here later, as data the physics could
//  optionally query.
//===========================================================================

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace vsim {

/// One point on the road centreline.
struct TrackPoint {
    glm::dvec3 position{0.0};   ///< world position on the ground plane
    glm::dvec3 tangent{1.0, 0.0, 0.0};  ///< unit direction of travel
    double     distance = 0.0;  ///< arc length from the start [m]
    double     curvature = 0.0; ///< 1/R [1/m], signed; 0 on straights
};

/// A closed or open road defined by its centreline.
class Track {
public:
    /// Build the default test track: a long straight, a pair of constant
    /// radius curves, a slalom section, and a return leg.
    static Track createTestTrack();

    /// Build a plain straight road of the given length.
    static Track createStraight(double length, double width);

    /// Build a closed-loop race circuit: two straights (start/finish and
    /// back straight) joined by two mirrored sequences of five corners of
    /// varying radius and direction. See Track.cpp for the construction and
    /// why it closes exactly rather than approximately.
    static Track createRaceCircuit();

    /// The Nurburgring GP-Strecke, as a generated track like every other one
    /// here.
    ///
    /// Its centreline is not authored and not invented: it was MEASURED off
    /// the reference scan this project was given, by
    /// tools/extract_track_centreline.cpp, and committed as a waypoint table
    /// (data/NurburgringGpCentreline.h). So the layout is the real one, while
    /// the circuit drawn around it - surface, kerbs, runoff, barriers, pit
    /// complex - is generated from that centreline exactly like the other
    /// presets, and the 91 MB scan is not needed at runtime.
    ///
    /// The measured lap is 3.95 km. The published GP-Strecke is 5.148 km;
    /// the difference is the reference model's own, not a tracing error (the
    /// walk follows the scanned road surface continuously and closes on
    /// itself), and the figure is left as measured rather than scaled to a
    /// number the geometry does not support.
    static Track createNurburgringGp();

    /// The full Nordschleife ("Green Hell"), as a generated track like every
    /// other one here.
    ///
    /// There is no scan of this circuit in the project to measure the way
    /// createNurburgringGp() does - the only reference is a schematic corner
    /// map with no coordinates (Nordschleife-full-lap.webp). Its centreline
    /// is instead the real, surveyed road geometry from OpenStreetMap
    /// (relation 38566), recovered by
    /// tools/fetch_nordschleife_centreline.py, which is a better source than
    /// digitising the schematic by eye would have been: it is measured, not
    /// approximated, and it is cross-checked in that tool against five named
    /// corners to confirm both the lap length (~20.8 km, matching the
    /// published 20.83 km) and the corner order.
    ///
    /// Unlike every other preset here, this track's surface is NOT flat: its
    /// waypoints carry real elevation (DEM-sourced, see the fetch tool), and
    /// it is the one generated track whose physics is wired to a
    /// TerrainHeightField built from that same centreline (see
    /// Renderer::buildTrackTerrain and Application::applyEnvironmentModel) -
    /// the real ~300 m of climb and descent is both drawn AND driven on,
    /// not just drawn. That is only possible because two loops of this
    /// circuit (around Hohe Acht) that pass within a few tens of metres of
    /// each other in the horizontal plane stay apart in reality because they
    /// are on different parts of the hillside; TrackMesh's self-clearance
    /// check exists because of exactly this track.
    static Track createNordschleife();

    /// The Apex Grand Prix Circuit: an ORIGINAL 4.9 km, 13-corner clockwise
    /// layout, authored for this project rather than measured off anything.
    /// Built from straights and clothoid-eased corners (appendCorner), with
    /// two of its straights solved for so the lap closes exactly - see
    /// environment/circuits/ApexGrandPrix.cpp, which also lays out its pit
    /// lane (pitLane()). Flat, like every preset except the Nordschleife.
    static Track createApexGrandPrix();

    /// Build a track from a closed or open run of ground-plane waypoints.
    ///
    /// The waypoints are threaded with a centripetal Catmull-Rom spline and
    /// resampled at a uniform arc length, then tangents and curvature are
    /// derived from the resampled line. Centripetal (rather than uniform)
    /// parameterisation matters here: waypoints measured off a real layout
    /// are not evenly spaced, and a uniform Catmull-Rom overshoots into a
    /// cusp wherever the spacing changes sharply - which on a circuit is
    /// exactly at the corners.
    ///
    /// `elevations`, if given, is one height [m] per waypoint (same size and
    /// indexing as `waypoints`) and is threaded through the IDENTICAL
    /// Catmull-Rom parameterisation as the ground-plane shape - knotted on
    /// horizontal (x, z) distance only, so hills never stretch or compress
    /// the curve fitting - ending up in TrackPoint::position.y. Omitted (the
    /// default), every point is flat at y = 0, which is what every preset
    /// except createNordschleife() wants and is exactly the previous
    /// behaviour of this function.
    static Track fromWaypoints(const std::vector<glm::dvec2>& waypoints,
                               double width, bool closed, double stepLength = 1.5,
                               const std::vector<double>* elevations = nullptr);

    /// Minimal placeholder centreline for the "nurburgring_scan" preset - see
    /// createByName(). The road surface for that preset comes from the
    /// imported environment model (config::EnvironmentModelConfig), not from
    /// this centreline; this exists only so Track's own bookkeeping
    /// (nearestPoint/lateralOffset, ground extent before the model loads)
    /// has something non-degenerate to work with.
    static Track createPlaceholder();

    /// Build by name - "test_track" / "race_circuit" / "straight" /
    /// "apex_gp" / "nurburgring" / "nordschleife" / "nurburgring_scan". Unknown names
    /// fall back to the test track with a warning, the same policy
    /// VehicleDynamicsFactory uses for an unknown dynamics model, so a typo
    /// in a config file never leaves the simulator without a track.
    static Track createByName(const std::string& name);

    /// Names accepted by createByName(), for the UI dropdown.
    static std::vector<std::string> availableTracks();

    /// The name this track was built under by createByName() ("apex_gp",
    /// "nurburgring", ...). The renderer keys circuit-specific venues off it.
    const std::string& id() const { return m_id; }

    /// Centreline of the pit lane, for a circuit that has one (currently
    /// only createApexGrandPrix()); empty otherwise. Same conventions as
    /// centreline(), with `distance` running from pit entry to pit exit.
    /// It is geometry only: nothing in the simulation enforces a pit lane,
    /// a speed limit or a stop - the ground is drivable everywhere.
    const std::vector<TrackPoint>& pitLane() const { return m_pitLane; }
    double pitLaneWidth() const { return m_pitLaneWidth; }

    /// Arc length of the start/finish line from the first centreline point.
    double startLineDistance() const { return m_startLine; }

    const std::vector<TrackPoint>& centreline() const { return m_points; }
    double width() const { return m_width; }
    double length() const { return m_points.empty() ? 0.0 : m_points.back().distance; }

    /// True for a track whose last centreline point connects back to the
    /// first (createRaceCircuit()). The renderer closes the road ribbon and
    /// its markings across that join only when this is true - every other
    /// preset is a point-to-point route and must not be visually joined.
    bool closed() const { return m_closed; }

    /// Half-extent of the ground plane to draw around the track [m].
    double groundExtent() const { return m_groundExtent; }

    /// True if this track's centreline carries real elevation (currently
    /// only createNordschleife()) rather than sitting flat at y = 0. Lets
    /// the renderer decide whether a TerrainHeightField needs building for
    /// this track at all - see Renderer::buildTrackTerrain().
    bool hasElevation() const { return m_hasElevation; }

    /// Nearest centreline point to a world position. Returns the index.
    /// Linear scan - the track is a few thousand points and this is only
    /// used by debug tooling, so an acceleration structure is not warranted.
    std::size_t nearestPoint(const glm::dvec3& worldPos) const;

    /// Signed lateral offset from the centreline [m]. Positive = right of
    /// the direction of travel. Useful for a future lane-keeping display.
    double lateralOffset(const glm::dvec3& worldPos) const;

private:
    void appendStraight(double length, double stepLength);
    void appendArc(double radius, double sweepRadians, double stepLength);

    /// A corner with clothoid entry and exit: curvature ramps linearly from
    /// zero up to 1/radius, holds, and ramps back down, so the steering
    /// input it asks for is continuous instead of the step appendArc()
    /// produces. `transition` is the fraction of the sweep spent in EACH
    /// ramp (0 = a plain arc, 0.5 = all ramp, no constant-radius middle).
    /// The total heading change is exactly `sweepRadians` either way.
    void appendCorner(double radius, double sweepRadians, double stepLength,
                      double transition);

    /// One half of createRaceCircuit(): a fixed sequence of straights and
    /// arcs whose sweeps sum to exactly +180 degrees. Called twice - see
    /// Track.cpp for why that is what makes the loop close exactly.
    void appendCircuitHalf(double stepLength);

    void finalise();

    static Track createByNameImpl(const std::string& name);

    std::vector<TrackPoint> m_points;
    double m_width = 8.0;
    double m_groundExtent = 400.0;
    bool   m_closed = false;
    bool   m_hasElevation = false;

    std::string m_id;
    std::vector<TrackPoint> m_pitLane;
    double m_pitLaneWidth = 0.0;
    double m_startLine = 0.0;

    // Cursor used while building.
    glm::dvec3 m_cursor{0.0};
    double     m_heading = 0.0;
    double     m_distance = 0.0;
};

} // namespace vsim
