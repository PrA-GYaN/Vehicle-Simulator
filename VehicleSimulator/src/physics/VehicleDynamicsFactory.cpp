#include "physics/VehicleDynamicsFactory.h"

#include <iostream>

#include "physics/FourWheelModel.h"
#include "physics/KinematicBicycleModel.h"

namespace vsim {

std::unique_ptr<IVehicleDynamics> createVehicleDynamics(const std::string& name) {
    if (name == "four_wheel" || name.empty()) {
        return std::make_unique<FourWheelModel>();
    }
    if (name == "kinematic_bicycle") {
        return std::make_unique<KinematicBicycleModel>();
    }

    std::cerr << "[physics] unknown dynamics model '" << name
              << "'; falling back to four_wheel\n";
    return std::make_unique<FourWheelModel>();
}

std::vector<std::string> availableDynamicsModels() {
    return {"four_wheel", "kinematic_bicycle"};
}

} // namespace vsim
