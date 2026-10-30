#pragma once
//===========================================================================
//  VehicleDynamicsFactory.h  --  model selection by name.
//
//  Adding a future model (dynamic bicycle, four-wheel, ...) means writing the
//  class and adding one line to the factory. No other file changes.
//===========================================================================

#include <memory>
#include <string>
#include <vector>

#include "physics/IVehicleDynamics.h"

namespace vsim {

/// Create a dynamics model by name. Unknown names fall back to the kinematic
/// bicycle model with a warning, so a typo in a config file never leaves the
/// simulation without physics.
std::unique_ptr<IVehicleDynamics> createVehicleDynamics(const std::string& name);

/// Names accepted by createVehicleDynamics(), for the UI dropdown.
std::vector<std::string> availableDynamicsModels();

} // namespace vsim
