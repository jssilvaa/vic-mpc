#include <robot_model/RobotState.h>


#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioInterface.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"



namespace ocs2::humanoid {

  void robotStateToPinocchio(const PinocchioInterface::Model& model, const ::robot::model::RobotState& robotState,
  const std::vector<robot::joint_index_t>& jointIds, vector_t& q, vector_t& v) {
    
    const size_t n = jointIds.size();
    // Euler-ZYX base => nq == nv == 6 + n_joints. Size from the model to stay correct
    // even if the caller hands us a default-constructed (empty) vector.
    q.resize(model.nq);
    v.resize(model.nv);

    const vector3_t euler_zyx = quaternionToEulerZYX(robotState.getRootRotationLocalToWorldFrame());
    
    q.head<3>() = robotState.getRootPositionInWorldFrame();
    q.segment<3>(3) = euler_zyx;
    q.tail(n) = robotState.getJointPositions(jointIds);

    v.head<3>() = robotState.getRootRotationLocalToWorldFrame() * robotState.getRootLinearVelocityInLocalFrame();
    v.segment<3>(3) = getEulerAnglesZyxDerivativesFromLocalAngularVelocity(euler_zyx, robotState.getRootAngularVelocityInLocalFrame());
    v.tail(n) = robotState.getJointVelocities(jointIds);
  }

} // namespace ocs2::humanoid
