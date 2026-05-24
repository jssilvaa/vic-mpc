#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/contact/ContactPolygon.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"


namespace ocs2::humanoid {

static pinocchio::JointModelComposite getBaseJointComposite() {
  // add 6 DoF for the floating base
  pinocchio::JointModelComposite baseJointComposite(2); // add two joints, each with 3 DoF
  // baseJointComposite.addJoint(pinocchio::jointModelFreeFlyer()); // adds 7 coordinates, we want 6 (eulerZYX and not a quaternion)
  baseJointComposite.addJoint(pinocchio::JointModelTranslation());
  baseJointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  return baseJointComposite; 
}

/// @brief Adds a contact center frame to the location the contact wrench is applied to the system to the pinocchio model.
/// @param[in] contactPolygon A contact polygon containing information about the contact center frame and corner points.
/// @param[in] model The model to which the frames are added.

static void addContactCenterFrames(const ContactPolygon& contactPolygon, pinocchio::ModelTpl<scalar_t>& model) {
  const ContactCenterPoint& ccp = contactPolygon.getContactCenterPoint();
  pinocchio::SE3 relPoseToParent(matrix3_t::Identity(), ccp.translationFromParent);
  pinocchio::Frame contactCenterFrame(ccp.frameName, model.getJointId(ccp.parentJointName), model.getFrameId(ccp.parentJointName),
                                      relPoseToParent, pinocchio::FIXED_JOINT);
  model.addFrame(contactCenterFrame);
}

/// @brief Adds a the collision avoidance frames to the pinocchio model.
/// @param[in] contactPolygon A contact polygon containinginformation about the contact center frame and corner points
/// @param[in] model The model to which the frames are added.

static void addCollisionCenterFrames(const ContactPolygon& contactPolygon, pinocchio::ModelTpl<scalar_t>& model, scalar_t radius = 0.075) {
  const ContactCenterPoint& ccp = contactPolygon.getContactCenterPoint();
  scalar_t y_half = (contactPolygon.getBounds().y_max - contactPolygon.getBounds().y_min) / 2.0;
  // scalar_t collisionCenterDistance = sqrt(radius * radius - y_half * y_half);
  pinocchio::SE3 relPoseToParentCP1(matrix3_t::Identity(),
                                    ccp.translationFromParent + vector3_t(contactPolygon.getBounds().x_max * 0.6, 0.0, 0.0));
  pinocchio::Frame contactCenterFrameCP1(ccp.frameName + "_collision_p_1", model.getJointId(ccp.parentJointName),
                                         model.getFrameId(ccp.parentJointName), relPoseToParentCP1, pinocchio::FIXED_JOINT);
  pinocchio::SE3 relPoseToParentCP2(matrix3_t::Identity(),
                                    ccp.translationFromParent + vector3_t(contactPolygon.getBounds().x_min * 0.6, 0.0, 0.0));
  pinocchio::Frame contactCenterFrameCP2(ccp.frameName + "_collision_p_2", model.getJointId(ccp.parentJointName),
                                         model.getFrameId(ccp.parentJointName), relPoseToParentCP2, pinocchio::FIXED_JOINT);
  model.addFrame(contactCenterFrameCP1);
  model.addFrame(contactCenterFrameCP2);
}

/// @brief Adds a frame in each corner of the contact polygon to the pinocchio model.
/// @param[in] contactPolygon A contact polygon containing information about the contact center frame and corner points.
/// @param[in] model The model to which the frames are added.

static void addContactPolygonFrames(const ContactPolygon& contactPolygon, pinocchio::ModelTpl<scalar_t>& model) {
  addContactCenterFrames(contactPolygon, model);
  addCollisionCenterFrames(contactPolygon, model);
  const vector3_t& contactCenterTranslation = contactPolygon.getContactCenterPoint().translationFromParent;  // from parent Joint
  int nPoints = contactPolygon.getNumberOfContactPoints();
  for (int i = 0; i < nPoints; i++) {
    vector3_t contactPointTranslation = contactCenterTranslation + contactPolygon.getContactPointTranslation(i);
    pinocchio::SE3 relPoseToParentJoint(matrix3_t::Identity(), contactPointTranslation);

    pinocchio::Frame currContactFrame(contactPolygon.getPolygonPointFrameName(i), model.getJointId(contactPolygon.getParentJointName()),
                                      model.getFrameId(contactPolygon.getParentJointName()), relPoseToParentJoint, pinocchio::FIXED_JOINT);
    model.addFrame(currContactFrame);
  }
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

PinocchioInterface createCustomPinocchioInterface(const std::string& taskFilePath,
                                                  const std::string& urdfFilePath,
                                                  const ModelSettings& modelSettings,
                                                  bool scaleTotalMass,
                                                  scalar_t totalMass,
                                                  bool verbose) {
  urdf::ModelInterfaceSharedPtr urdfTree = urdf::parseURDFFile(urdfFilePath);
  if (urdfTree == nullptr) {
    throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
  }

  using joint_pair_t = std::pair<const std::string, std::shared_ptr<urdf::Joint>>;
                                
  // remove extraneous joints from urdf, i.e. joints not in mpcModelJointNames
  urdf::ModelInterfaceSharedPtr newModel = std::make_shared<urdf::ModelInterface>(*urdfTree);
  const std::vector<std::string>& mpcModelJointNames = modelSettings.mpcModelJointNames;
  for (joint_pair_t& jointPair : newModel ->joints_) {
    if (std::find(mpcModelJointNames.begin(), mpcModelJointNames.end(), jointPair.first) == mpcModelJointNames.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  
  pinocchio::ModelTpl<scalar_t> model;
  pinocchio::urdf::buildModel(newModel, getBaseJointComposite(), model);

  for (int i = 0; i < N_CONTACTS; i++) {
    ContactRectangle contactRectangle = ContactRectangle::loadContactRectangle(taskFilePath, modelSettings, i, verbose);
    addContactPolygonFrames(contactRectangle, model);
  }

  if (scaleTotalMass) {
    scalePinocchioModelInertia(model, totalMass, true); 
  }

  PinocchioInterface pinocchioInterface(model, urdfTree);
  checkPinocchioJointNaming(pinocchioInterface, modelSettings);

  return pinocchioInterface;
} 

} // namespace ocs2 