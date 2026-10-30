#pragma once
//===========================================================================
//  CarMesh.h  --  the vehicle model, generated rather than imported.
//
//  WHY THIS EXISTS
//  ---------------
//  The simulator previously had two ways to draw a car, and neither was
//  good: a stack of three boxes (MeshFactory::box), or a third-party .glb
//  scan. The scan is a black box - its geometry does not follow the
//  configured wheelbase, its wheels only animate because four node names
//  happened to be guessable, and nothing about it can be corrected from
//  inside this project.
//
//  Everything here is built from the vehicle's OWN parameters. Change the
//  wheelbase in a YAML file and the wheel arches move with it, because the
//  arches are cut by the axle positions rather than drawn where an artist
//  once put them. That is the same argument MeshFactory.h already makes for
//  the primitives; this file just carries it far enough to produce a shape
//  that reads as an actual car.
//
//  HOW THE SHAPE IS BUILT
//  ----------------------
//  A car body is a LOFT: a run of closed cross-sections threaded along the
//  car's length, skinned between consecutive sections. Each cross-section
//  here is a superellipse - flat-bottomed, round-shouldered, with separate
//  exponents above and below the widest point, which is what a car section
//  actually looks like. A style supplies a dozen control sections; those are
//  resampled with a Catmull-Rom spline into ~90 sections so the skin is
//  smooth, and only then are the wheel arches cut and the fender bulges
//  applied from the real axle positions.
//
//  The greenhouse (roof, pillars, glass) is a second, narrower loft sitting
//  on top of and slightly inside the first. Windows are patches lifted a
//  centimetre off that same surface, so the glass follows the body curvature
//  exactly and the gaps between patches ARE the pillars. Lights, vents and
//  livery panels are placed the same way.
//
//  COORDINATE FRAME - vehicle local, as core/Coordinates.h defines it:
//      +X forward, +Y up, +Z right, origin at the REAR AXLE, y = 0 is the
//      ground. Identical to what Renderer::rebuildVehicleMeshes() expected
//      from the box body it replaces, so the draw path is unchanged.
//===========================================================================

#include <string>

#include <glm/glm.hpp>

#include "config/VehicleParameters.h"
#include "rendering/MeshFactory.h"

namespace vsim {

/// Body shape family. The physics does not read this - a vehicle's mass,
/// grip and geometry are what they are in its YAML file regardless of what
/// it looks like - but drawing an SUV with a GT3 silhouette would make the
/// on-screen car a lie about the configuration, so it is selectable.
enum class CarBodyStyle {
    Auto,    ///< choose from the vehicle's own proportions - see resolveCarBodyStyle()
    Race,    ///< GT3-style: splitter, arch vents, swan-neck wing, diffuser
    Sports,  ///< low coupe, fastback roof, ducktail lip
    Saloon,  ///< upright three-box family car
    Suv      ///< tall, boxy, roof rails, body cladding
};

CarBodyStyle carBodyStyleFromString(const std::string& name);
const char*  carBodyStyleName(CarBodyStyle style);

/// Resolve CarBodyStyle::Auto against the vehicle's own dimensions: height
/// relative to length is what separates an SUV from a GT3 car, and the ratio
/// is already in the config file for every vehicle. Anything other than Auto
/// is returned unchanged.
CarBodyStyle resolveCarBodyStyle(CarBodyStyle requested, const VehicleParameters& p);

/// Colours used by the generator. Defaults are a neutral scheme; the Renderer
/// overrides `body` and `glass` from RenderStyle so the UI colour pickers
/// still work.
struct CarPalette {
    glm::vec3 body       {0.16f, 0.42f, 0.78f};
    glm::vec3 accent     {0.93f, 0.94f, 0.96f};  ///< stripes, number panel
    glm::vec3 trim       {0.085f, 0.09f, 0.105f}; ///< carbon, plastic, aero, vents
    glm::vec3 glass      {0.13f, 0.17f, 0.22f};
    glm::vec3 lightFront {0.90f, 0.91f, 0.82f};
    glm::vec3 lightRear  {0.58f, 0.05f, 0.05f};
    glm::vec3 tire       {0.072f, 0.075f, 0.082f};
    glm::vec3 rim        {0.60f, 0.61f, 0.65f};
    glm::vec3 brakeDisc  {0.29f, 0.29f, 0.31f};
    glm::vec3 caliper    {0.76f, 0.13f, 0.09f};
};

/// One complete vehicle, split by how each part is drawn rather than by what
/// it is: `body` and `glass` ride the chassis transform, the wheels spin,
/// and the brakes steer with the wheel but do NOT spin with it.
struct CarMeshSet {
    MeshData body;        ///< painted shell, aero, trim, lights
    MeshData glass;       ///< windows, lifted off the greenhouse surface
    MeshData wheelLeft;   ///< tyre + rim, outboard face toward -Z
    MeshData wheelRight;  ///< tyre + rim, outboard face toward +Z
    MeshData brakeLeft;   ///< disc + caliper; steers, never spins
    MeshData brakeRight;
};

/// Build the whole vehicle. `style` may be Auto.
CarMeshSet buildCarMeshes(const VehicleParameters& p, CarBodyStyle style,
                          const CarPalette& palette);

} // namespace vsim
