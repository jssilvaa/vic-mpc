// Out-of-line special members for PinocchioInterface. Kept out of the header so
// that consumers don't pay the pinocchio::Model/Data template instantiation cost
// in every translation unit that includes PinocchioInterface.h.

#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/pinocchio_model/PinocchioInterface.h"

namespace ocs2 {

PinocchioInterface::PinocchioInterface(const PinocchioInterface&) = default;
PinocchioInterface::PinocchioInterface(PinocchioInterface&&) noexcept = default;
PinocchioInterface& PinocchioInterface::operator=(const PinocchioInterface&) = default;
PinocchioInterface& PinocchioInterface::operator=(PinocchioInterface&&) noexcept = default;
PinocchioInterface::~PinocchioInterface() = default;

}  // namespace ocs2
