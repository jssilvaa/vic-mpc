#pragma once

// C++ replacement of examples/wb_humanoid_mpc/humanoid_nmpc/remote_control/
// remote_control/xbox_controller_interface.py. Same role: read gamepad axes
// and emit a WalkingVelocityCommand. Backed by SDL2 instead of pygame.

#include <remote_control/WalkingCommand.h>

#include <SDL.h>

#include <chrono>
#include <string>

namespace robot::remote_control {

class XboxControllerInterface {
 public:
  // Matches the Python interface's publisher_rate argument (used to scale the
  // pelvis-height increment per poll). Default 25 Hz matches the reference's
  // ROS2 timer.
  explicit XboxControllerInterface(double publisher_rate_hz = 25.0);
  ~XboxControllerInterface();

  XboxControllerInterface(const XboxControllerInterface&) = delete;
  XboxControllerInterface& operator=(const XboxControllerInterface&) = delete;

  bool connected() const { return controller_ != nullptr; }

  // Refresh SDL state, scan for a controller if one is not connected, and fill
  // out the WalkingVelocityCommand with normalized stick and trigger inputs.
  // Returns true if a controller is connected and the command was populated.
  bool poll(ocs2::humanoid::WalkingVelocityCommand& cmd);

 private:
  void scanForController();
  void closeController();

  SDL_GameController* controller_ = nullptr;

  // Pelvis-height state (Python: current_pelvis_height_target / min / max).
  double current_pelvis_height_target_ = 0.8;
  double min_pelvis_height_ = 0.2;
  double max_pelvis_height_ = 1.0;

  double publisher_rate_hz_;

  // Throttle the rescan to once per second so a disconnected controller doesn't
  // hammer SDL on every poll.
  std::chrono::steady_clock::time_point lastScanAttempt_{};
};

}  // namespace robot::remote_control
