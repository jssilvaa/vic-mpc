#pragma once 

#include <pinocchio/fwd.hpp> 
#include <string>

#include "humanoid_common_mpc/pinocchio_model/PinocchioInterface.h"


namespace ocs2 {

/// @brief Creates a standard pinocchio model from the urdf
/// @param urdfFilePath path to the udrf file 
/// @return the pinocchio interface 
PinocchioInterface createDefaultPinocchioInterface(const std::string& urdfFilePath); 

} // namespace ocs2 