#pragma once

#include <pinocchio/fwd.hpp>
#include <cmath>
#include <vector>

#include <robot_core/Types.h>  // joint_index_t

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>  // getEulerAnglesZyxDerivativesFromLocalAngularVelocity

#include "humanoid_common_mpc/common/Types.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"

namespace robot::model { class RobotState; }

namespace ocs2::humanoid {

  /// @brief Computes Euler ZYX angles written in vector3_t from a quaternion_t
  /// @param quat the Eigen quaternion 
  /// @return vector3_t (euler_z, euler_y, euler_x)
  static inline vector3_t quaternionToEulerZYX(const quaternion_t& quat) {
    scalar_t w = quat.w(); 
    scalar_t x = quat.x(); 
    scalar_t y = quat.y(); 
    scalar_t z = quat.z(); 

    scalar_t yaw, pitch, roll; 
    pitch = std::asin(2.0 * (w * y - z * x)); 
    yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)); 
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)); 

    return vector3_t(yaw, pitch, roll);
  }

  // getEulerAnglesZyxDerivativesFromLocalAngularVelocity now comes from
  // <ocs2_robotic_tools/common/RotationDerivativesTransforms.h> (namespace ocs2);
  // it was hand-written here during D1 and is identical to the upstream math.
  // quaternionToEulerZYX stays here — this (humanoid DynamicsHelperFunctions.h) is
  // its reference home.

  // Fills plain Pinocchio (q, v) generalized coordinates from a RobotState.
  //   q = [pos(3), eulerZYX(3), joints],  v = [linVel_world(3), eulerZyxRate(3), jointVel].
  // jointIds must be in Pinocchio joint order (model.names[i+2] for the i-th joint).
  // Extracted from CentroidalMpcMrtJointController::updateMpcState phase A (no centroidal packing).
  void robotStateToPinocchio(const PinocchioInterface::Model& model, const ::robot::model::RobotState& robotState,
                             const std::vector<robot::joint_index_t>& jointIds, vector_t& q, vector_t& v);

} // namespace ocs2::humanoid
