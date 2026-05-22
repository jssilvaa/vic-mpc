#pragma once

// C++ replacement of examples/wb_humanoid_mpc/humanoid_nmpc/remote_control/
// remote_control/xbox_controller_interface.py. Same role: read gamepad axes
// and emit a WalkingVelocityCommand. Backed by SDL2 instead of pygame.

#include <remote_control/WalkingCommand.h>

#include <SDL.h>

#include <chrono>
#include <optional>

namespace robot::remote_control {

class XboxControllerInterface {
 public:
  // The Python interface took a publisher_rate_hz to scale the per-poll
  // pelvis-height increment; we measure elapsed time inside poll() instead so
  // the caller's actual loop rate is what's used (the Python version
  // double-counted when the loop ran at a different rate than declared).
  XboxControllerInterface();
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

  // Last poll() timestamp for trapezoidal integration of trigger differential.
  // std::nullopt on first poll so the first sample doesn't integrate a huge dt.
  std::optional<std::chrono::steady_clock::time_point> lastPollTime_;

  // Throttle the rescan to once per second so a disconnected controller doesn't
  // hammer SDL on every poll.
  std::chrono::steady_clock::time_point lastScanAttempt_{};
};

}  // namespace robot::remote_control
