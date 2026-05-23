#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioInterface.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/multibody/joint/joint-spherical.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace ocs2 {

  static pinocchio::JointModelComposite getBaseJointComposite() {
  pinocchio::JointModelComposite baseJointComposite(2); 
  baseJointComposite.addJoint(pinocchio::JointModelTranslation());
  baseJointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  return baseJointComposite; 
}

PinocchioInterface createDefaultPinocchioInterface(const std::string& urdfFilePath) {
  urdf::ModelInterfaceSharedPtr urdfTree = urdf::parseURDFFile(urdfFilePath); 
  if (urdfTree == nullptr) {
    throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
  }
  pinocchio::JointModelComposite baseJointComposite = getBaseJointComposite(); 
  PinocchioInterface::Model model;
  pinocchio::urdf::buildModel(urdfTree, baseJointComposite, model); 
  return PinocchioInterface(std::move(model), std::move(urdfTree)); 
}

} // namespace ocs2 