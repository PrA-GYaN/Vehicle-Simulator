//===========================================================================
//  ApexGrandPrix.cpp  --  the Apex Grand Prix Circuit ("apex_gp").
//
//  An original layout: 13 corners, clockwise, a shade under 5 km. Unlike the
//  two Nurburgring presets nothing here is measured - it is authored, as a
//  list of straights and corners, the same way createRaceCircuit() is.
//
//  CLOSING THE LAP
//  ---------------
//  createRaceCircuit() closes by symmetry (the same half walked twice),
//  which is exact but forces the circuit to be point-symmetric. A real
//  layout is not, so this one closes a different way. The corner sweeps sum
//  to exactly +360 degrees, which fixes the final heading. That leaves the
//  end POSITION, and the end position is a LINEAR function of the straight
//  lengths - lengthening a straight slides everything after it along that
//  straight's heading and changes nothing else. Two straights with different
//  headings therefore span the plane, and a 2x2 solve finds the lengths that
//  land the last point exactly back on the first.
//
//  FRAME
//  -----
//  The first centreline point is the origin, heading +X, on the main
//  straight just behind the start line - where the application spawns the
//  car. With +Z to the right of +X (core/Coordinates.h) and every net turn
//  to the right, the whole circuit lies on the +Z side of the main straight
//  and the infield is always on the driver's right. The pits are on that
//  side; the main grandstand faces them from the left.
//===========================================================================

#include "environment/Track.h"

#include <cmath>

#include "core/Coordinates.h"
#include "core/Units.h"

namespace vsim {
namespace {

struct Segment {
    enum Kind { Straight, Corner } kind;
    double a;      ///< straight: length [m]            corner: radius [m]
    double b;      ///< straight: 1/2 = solved-for      corner: sweep [deg], + = right
};

constexpr double kMainStraightAhead  = 590.0;  ///< start point to the T1 turn-in
constexpr double kMainStraightBehind = 310.0;  ///< T13 exit to the start point

// The lap, in driving order.
const Segment kLap[] = {
    {Segment::Straight, kMainStraightAhead, 0},
    {Segment::Corner,    42.0,  100.0},   // T1   heavy stop off the main straight
    {Segment::Straight,  55.0, 0},
    {Segment::Corner,    62.0,  -58.0},   // T2   left, opening
    {Segment::Straight, 170.0, 0},
    {Segment::Corner,   210.0,   50.0},   // T3   fast right sweeper
    {Segment::Straight, 140.0, 0},
    {Segment::Corner,    80.0,   70.0},   // T4   medium right
    {Segment::Straight, 100.0, 0},
    {Segment::Corner,   110.0, -150.0},   // T5   long left carousel
    {Segment::Straight, 180.0, 0},
    {Segment::Corner,    28.0,  178.0},   // T6   hairpin
    {Segment::Straight, 620.0, 2},        //      back straight (solved)
    {Segment::Corner,    30.0,  -52.0},   // T7   chicane, left
    {Segment::Straight,  22.0, 0},
    {Segment::Corner,    30.0,   52.0},   // T8   chicane, right
    {Segment::Straight, 150.0, 0},
    {Segment::Corner,   125.0,   79.0},   // T9   fast right
    {Segment::Straight, 110.0, 1},        //      (solved)
    {Segment::Corner,    58.0,  -84.0},   // T10  left
    {Segment::Straight, 120.0, 0},
    {Segment::Corner,    46.0,  100.0},   // T11  tight right
    {Segment::Straight,  70.0, 0},
    {Segment::Corner,    90.0,  -46.0},   // T12  left kink
    {Segment::Straight,  90.0, 0},
    {Segment::Corner,    80.0,  121.0},   // T13  final corner
    {Segment::Straight, kMainStraightBehind, 0},
};
// 100 - 58 + 50 + 70 - 150 + 178 - 52 + 52 + 79 - 84 + 100 - 46 + 121 = 360.

constexpr double kStep       = 1.5;    ///< centreline spacing [m], as every other preset
constexpr double kTransition = 0.19;   ///< share of each corner's sweep per clothoid
constexpr double kWidth      = 14.0;

//-- pit lane, in main-straight coordinates (x along, lateral to the right) --
constexpr double kPitEntryStart = -295.0;
constexpr double kPitEntryEnd   = -185.0;
constexpr double kPitExitStart  =  190.0;
constexpr double kPitExitEnd    =  320.0;
constexpr double kPitLaneLat    =   13.0;  ///< lane centre, from the track centreline
constexpr double kPitMergeLat   =    5.5;  ///< where the lane leaves / rejoins the track
constexpr double kPitLaneWidth  =    5.0;

double smooth(double u) {
    u = clampd(u, 0.0, 1.0);
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0);   // C2 at both ends
}

double pitLateral(double x) {
    if (x < kPitEntryEnd) {
        return kPitMergeLat + (kPitLaneLat - kPitMergeLat) *
               smooth((x - kPitEntryStart) / (kPitEntryEnd - kPitEntryStart));
    }
    if (x > kPitExitStart) {
        return kPitLaneLat + (kPitMergeLat - kPitLaneLat) *
               smooth((x - kPitExitStart) / (kPitExitEnd - kPitExitStart));
    }
    return kPitLaneLat;
}

} // namespace

Track Track::createApexGrandPrix() {
    auto walk = [](double extraA, double extraB) {
        Track t;
        t.m_cursor  = {0.0, 0.0, 0.0};
        t.m_heading = 0.0;
        for (const Segment& s : kLap) {
            if (s.kind == Segment::Straight) {
                double len = s.a;
                if (s.b == 1) len += extraA;
                if (s.b == 2) len += extraB;
                t.appendStraight(len, kStep);
            } else {
                t.appendCorner(s.a, degToRad(s.b), kStep, kTransition);
            }
        }
        return t;
    };

    // End position is affine in the two extras: probe it, then solve.
    const glm::dvec3 e0 = walk(0.0, 0.0).m_cursor;
    const glm::dvec3 ea = walk(1.0, 0.0).m_cursor - e0;
    const glm::dvec3 eb = walk(0.0, 1.0).m_cursor - e0;
    const double det = ea.x * eb.z - eb.x * ea.z;
    double extraA = 0.0, extraB = 0.0;
    if (std::abs(det) > 1e-9) {
        extraA = (-e0.x * eb.z + e0.z * eb.x) / det;
        extraB = (-ea.x * e0.z + ea.z * e0.x) / det;
    }

    Track t = walk(extraA, extraB);
    t.m_width  = kWidth;
    t.m_closed = true;

    // The car spawns on the first centreline point (Application), so the
    // line goes a little ahead of it and the grid is painted back from there.
    t.m_startLine = 12.0;

    // Pit lane. The main straight is the X axis here, so the lane is simply
    // a lateral offset profile along it.
    t.m_pitLaneWidth = kPitLaneWidth;
    double distance = 0.0;
    glm::dvec3 prev{0.0};
    for (double x = kPitEntryStart; x <= kPitExitEnd + 1e-6; x += kStep) {
        const double d = 0.25;
        TrackPoint p;
        p.position = {x, 0.0, pitLateral(x)};
        p.tangent  = glm::normalize(glm::dvec3(2.0 * d, 0.0, pitLateral(x + d) - pitLateral(x - d)));
        if (!t.m_pitLane.empty()) distance += glm::length(p.position - prev);
        p.distance = distance;
        prev = p.position;
        t.m_pitLane.push_back(p);
    }
    const std::size_t n = t.m_pitLane.size();
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double dyaw = coords::wrapAngle(
            coords::yawFromDirection(t.m_pitLane[i + 1].tangent) -
            coords::yawFromDirection(t.m_pitLane[i - 1].tangent));
        t.m_pitLane[i].curvature = dyaw / (2.0 * kStep);
    }

    t.finalise();
    return t;
}

} // namespace vsim
