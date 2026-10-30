#pragma once
//===========================================================================
//  TrackMesh.h  --  the circuit, generated from its own centreline.
//
//  WHY THIS EXISTS
//  ---------------
//  Track.h produces a centreline and nothing else, deliberately: it is part
//  of the GL-free simulation core. Turning that line into a circuit you can
//  recognise - asphalt, painted edges, kerbs at the corners, runoff, armco,
//  a start line with a grid, a gantry, grandstands - is a rendering job, and
//  this is where it happens.
//
//  Everything is derived from the centreline the physics already uses, so
//  the drawn circuit cannot drift away from the one the car is driving on.
//  That is the property the imported scan could never have: a 90 MB mesh of
//  a real track has no centreline at all, which is why the environment-model
//  path has to guess a spawn point from raw scanned vertices.
//
//  WHAT IS GEOMETRY AND WHAT IS PAINT
//  ----------------------------------
//  Kerbs, barriers, gantries and grandstands are real geometry standing off
//  the ground. Lane markings, the start line and the grid boxes are flat
//  quads lifted a few millimetres, layered in a fixed order (asphalt, then
//  paint, then kerbs) so nothing z-fights.
//
//  THE SURFACE IS STILL FLAT
//  -------------------------
//  Every drivable surface here sits at y ~ 0, because that is what the
//  physics believes: on any track other than the imported scan the vehicle
//  model runs on flat ground (Renderer::terrainField() is null, so
//  FourWheelModel sees ground height 0 everywhere). Banking and elevation
//  are therefore NOT drawn - a cambered corner the car cannot feel would be
//  a lie told by the renderer. The scenery around the track rises and falls;
//  the racing surface does not.
//===========================================================================

#include <glm/glm.hpp>

#include "environment/Track.h"
#include "rendering/MeshFactory.h"

namespace vsim {

/// Colours of the circuit. Exposed so RenderStyle can drive them.
struct TrackPalette {
    glm::vec3 asphalt  {0.205f, 0.205f, 0.215f};
    glm::vec3 marking  {0.90f, 0.90f, 0.87f};
    glm::vec3 kerbRed  {0.68f, 0.13f, 0.12f};
    glm::vec3 kerbWhite{0.86f, 0.86f, 0.83f};
    glm::vec3 grass    {0.235f, 0.345f, 0.165f};
    glm::vec3 gravel   {0.60f, 0.545f, 0.40f};
    glm::vec3 concrete {0.60f, 0.60f, 0.585f};
    glm::vec3 steel    {0.52f, 0.545f, 0.575f};
    glm::vec3 rubber   {0.085f, 0.085f, 0.09f};
    glm::vec3 foliage  {0.145f, 0.30f, 0.145f};
    glm::vec3 timber   {0.26f, 0.19f, 0.13f};
    glm::vec3 accent   {0.78f, 0.16f, 0.12f};
};

/// The circuit, split by draw order and by what a user might want to switch
/// off rather than by what each piece is.
struct TrackMeshSet {
    MeshData asphalt;   ///< racing surface
    MeshData markings;  ///< edge lines, start line, grid boxes (paint)
    MeshData kerbs;     ///< corner kerbs, raised
    MeshData verge;     ///< grass, gravel traps, embankments
    MeshData barriers;  ///< armco, posts, tyre walls, catch fencing
    MeshData scenery;   ///< gantry, grandstands, pit building, boards, trees
};

/// Build every piece of the circuit from `track`.
TrackMeshSet buildTrackMeshes(const Track& track, const TrackPalette& palette);

} // namespace vsim
