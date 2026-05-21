#pragma once

#include <robot_core/ThreadSafe.h>
#include <robot_model/RobotJointAction.h>
#include <robot_model/RobotState.h>

#include "robot_model/RobotDescription.h"

namespace robot::model {

// Public interface describing how to interact with the robot.
class RobotHWInterfaceBase {
 public:
  RobotHWInterfaceBase(const std::string& urdfPath)
      : robotDescription_(urdfPath),
        robotState_((model::RobotState(robotDescription_))),
        robotJointAction_(model::RobotJointAction(robotDescription_)),
        threadSafeRobotState_(robotState_),
        threadSafeRobotJointAction_(robotJointAction_) {}

  const model::RobotDescription& getRobotDescription() const { return robotDescription_; }

  // Read-only access to the latest robot state snapshot.
  const RobotState& getRobotState() const { return robotState_; }

  // Refresh the internal robot state snapshot from the thread-safe source.
  void updateInterfaceStateFromRobot() { threadSafeRobotState_.copy_value(robotState_); }

  // Reference to fill in the updated joint control action.
  RobotJointAction& getRobotJointAction() { return robotJointAction_; }

  // Publish the new action to the robot.
  void applyJointAction() { threadSafeRobotJointAction_.set(robotJointAction_); }

 private:
  const RobotDescription robotDescription_;
  RobotState robotState_;
  RobotJointAction robotJointAction_;

 protected:
  ThreadSafe<RobotState> threadSafeRobotState_;
  ThreadSafe<RobotJointAction> threadSafeRobotJointAction_;
};

}  // namespace robot::model
