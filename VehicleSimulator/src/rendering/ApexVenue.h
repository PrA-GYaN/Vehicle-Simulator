#pragma once
//===========================================================================
//  ApexVenue.h  --  the Apex Grand Prix Circuit, as a venue.
//
//  Generates everything drawn for the "apex_gp" track (environment/circuits/
//  ApexGrandPrix.cpp): racing surface and paint, kerbs, runoff, barriers and
//  catch fencing, the pit lane and pit building, race control, the paddock,
//  grandstands, marshal posts, signage, terrain, trees and the hills that
//  close the horizon. See rendering/Venue.h for how it is drawn.
//
//  Like TrackMesh, all of it is derived from the centreline the physics
//  uses, and all of the driving surface is flat at y ~ 0 because the
//  vehicle model runs on flat ground on this track. Nothing here is
//  collidable: the simulator has no collision with scenery on any track.
//===========================================================================

#include "environment/Track.h"
#include "rendering/Venue.h"

namespace vsim {

/// True for a track this file can build a venue for.
bool isApexVenueTrack(const Track& track);

/// Build the venue. Deterministic: the same track gives the same venue.
VenueData buildApexVenue(const Track& track);

} // namespace vsim
