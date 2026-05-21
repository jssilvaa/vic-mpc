#pragma once

#include <robot_model/RobotJointAction.h>
#include <robot_model/RobotState.h>

namespace robot::model {

class ControlBase {
 public:
  virtual ~ControlBase() = default;

  virtual bool ready() const = 0;

  // Compute joint commands given the current robot state.
  virtual void computeJointControlAction(scalar_t time,
                                         const ::robot::model::RobotState& robotState,
                                         ::robot::model::RobotJointAction& robotJointAction) = 0;
};

}  // namespace robot::model
