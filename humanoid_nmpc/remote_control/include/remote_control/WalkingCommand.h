#pragma once

// Stub of humanoid_common_mpc/include/humanoid_common_mpc/command/WalkingVelocityCommand.h
// for D0 (no humanoid_common_mpc package yet). Same namespace and field names so
// the struct can be moved to humanoid_common_mpc at D2 without touching call sites.

#include <algorithm>

namespace ocs2::humanoid {

struct WalkingVelocityCommand {
  WalkingVelocityCommand() = default;
  WalkingVelocityCommand(double v_x, double v_y, double desired_pelvis_h, double v_yaw)
      : linear_velocity_x(v_x), linear_velocity_y(v_y), desired_pelvis_height(desired_pelvis_h), angular_velocity_z(v_yaw) {}

  double linear_velocity_x = 0.0;
  double linear_velocity_y = 0.0;
  double desired_pelvis_height = 0.8;
  double angular_velocity_z = 0.0;

  void setToDefaultCommand() {
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    desired_pelvis_height = 0.8;
    angular_velocity_z = 0.0;
  }
};

}  // namespace ocs2::humanoid
