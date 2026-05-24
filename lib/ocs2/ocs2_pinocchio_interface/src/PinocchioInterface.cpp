// PinocchioInterfaceTpl is now a header-defined template (special members are inline
// = default). This TU just forces the scalar_t instantiation into the library so the
// symbols live in one place.

#include <pinocchio/fwd.hpp>

#include "ocs2_pinocchio_interface/PinocchioInterface.h"

namespace ocs2 {

template class PinocchioInterfaceTpl<scalar_t>;

}  // namespace ocs2
