#pragma once

#include <pinocchio/fwd.hpp>
#include <cmath>
#include <vector>

#include <robot_core/Types.h>  // joint_index_t

#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioInterface.h"

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

  template <typename SCALAR_T> 
  VECTOR3_T<SCALAR_T> getEulerAnglesZyxDerivativesFromLocalAngularVelocity(
    const VECTOR3_T<SCALAR_T>& eulerZYX, 
    const VECTOR3_T<SCALAR_T>& omega_local) {

      SCALAR_T yaw_rate, pitch_rate, roll_rate;

      yaw_rate = (std::sin(eulerZYX[2]) * omega_local[1] + std::cos(eulerZYX[2]) * omega_local[2]) / std::cos(eulerZYX[1]);
      pitch_rate = std::cos(eulerZYX[2]) * omega_local[1] - std::sin(eulerZYX[2]) * omega_local[2];
      roll_rate = omega_local[0] + std::tan(eulerZYX[1]) * (std::sin(eulerZYX[2]) * omega_local[1] + std::cos(eulerZYX[2]) * omega_local[2]);

      return VECTOR3_T<SCALAR_T>(yaw_rate, pitch_rate, roll_rate);
    }

  // Fills plain Pinocchio (q, v) generalized coordinates from a RobotState.
  //   q = [pos(3), eulerZYX(3), joints],  v = [linVel_world(3), eulerZyxRate(3), jointVel].
  // jointIds must be in Pinocchio joint order (model.names[i+2] for the i-th joint).
  // Extracted from CentroidalMpcMrtJointController::updateMpcState phase A (no centroidal packing).
  void robotStateToPinocchio(const PinocchioInterface::Model& model, const ::robot::model::RobotState& robotState,
                             const std::vector<robot::joint_index_t>& jointIds, vector_t& q, vector_t& v);

} // namespace ocs2::humanoid
