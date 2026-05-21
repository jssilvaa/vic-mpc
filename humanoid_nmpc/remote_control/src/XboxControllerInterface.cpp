#include "remote_control/XboxControllerInterface.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace robot::remote_control {

namespace {

// Same exponential shaping as the Python interface:
//   y = 0.2 * x + 0.8 * x^3
// applied to normalized stick values in [-1, 1].
double expoShape(double x) { return 0.2 * x + 0.8 * x * x * x; }

// Deadzone match (Python: |x| >= 0.02).
double applyDeadzone(double x, double dz = 0.02) { return std::abs(x) >= dz ? x : 0.0; }

// SDL stick axes are int16 in [-32768, 32767]; normalize and clamp to [-1, 1].
double normAxis(Sint16 raw) {
  constexpr double kInvMax = 1.0 / 32767.0;
  return std::clamp(static_cast<double>(raw) * kInvMax, -1.0, 1.0);
}

// SDL trigger axes are int16 in [0, 32767]; normalize to [0, 1].
double normTrigger(Sint16 raw) {
  constexpr double kInvMax = 1.0 / 32767.0;
  return std::clamp(static_cast<double>(raw) * kInvMax, 0.0, 1.0);
}

}  // namespace

XboxControllerInterface::XboxControllerInterface(double publisher_rate_hz)
    : publisher_rate_hz_(publisher_rate_hz) {
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
    std::cerr << "[XboxControllerInterface] SDL_Init failed: " << SDL_GetError() << std::endl;
    return;
  }
  scanForController();
}

XboxControllerInterface::~XboxControllerInterface() {
  closeController();
  SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
}

void XboxControllerInterface::scanForController() {
  // One-shot diagnostic: log what SDL sees on the first scan so the user can
  // tell apart "no device attached" vs "device attached but no SDL controller
  // mapping" without watching the rescan output every second.
  static bool dumpedJoystickInfo = false;
  if (!dumpedJoystickInfo) {
    std::cerr << "[XboxControllerInterface] SDL_NumJoysticks=" << SDL_NumJoysticks() << "\n";
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
      std::cerr << "  joy[" << i << "] name=" << SDL_JoystickNameForIndex(i)
                << "  is_gamecontroller=" << SDL_IsGameController(i) << "\n";
    }
    dumpedJoystickInfo = true;
  }

  lastScanAttempt_ = std::chrono::steady_clock::now();
  const int n = SDL_NumJoysticks();
  for (int i = 0; i < n; ++i) {
    if (SDL_IsGameController(i)) {
      controller_ = SDL_GameControllerOpen(i);
      if (controller_) {
        const char* name = SDL_GameControllerName(controller_);
        std::cerr << "[XboxControllerInterface] Connected to: " << (name ? name : "unknown") << std::endl;
        return;
      }
    }
  }
}

void XboxControllerInterface::closeController() {
  if (controller_) {
    SDL_GameControllerClose(controller_);
    controller_ = nullptr;
  }
}

bool XboxControllerInterface::poll(ocs2::humanoid::WalkingVelocityCommand& cmd) {
  SDL_GameControllerUpdate();

  if (!controller_) {
    // Throttle rescan to ~1 Hz.
    auto now = std::chrono::steady_clock::now();
    if (now - lastScanAttempt_ > std::chrono::seconds(1)) scanForController();
    if (!controller_) return false;
  }

  // Some platforms surface a disconnect by returning false on attached().
  if (!SDL_GameControllerGetAttached(controller_)) {
    std::cerr << "[XboxControllerInterface] Lost controller; will rescan." << std::endl;
    closeController();
    return false;
  }

  // SDL maps the left/right sticks to a normalized "game controller" view, so
  // we don't need the Bluetooth/USB branch the Python code carries.
  // Forward is +X for the robot; the left-stick Y axis is "down-positive" in
  // SDL, hence the sign flip to match the Python convention (raw_x_left = -axis(1)).
  double raw_x_left = -normAxis(SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY));
  double raw_y_left = -normAxis(SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTX));
  double raw_y_right = -normAxis(SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_RIGHTX));

  double lt = normTrigger(SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERLEFT));
  double rt = normTrigger(SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT));

  raw_x_left = applyDeadzone(expoShape(raw_x_left));
  raw_y_left = applyDeadzone(expoShape(raw_y_left));
  raw_y_right = applyDeadzone(expoShape(raw_y_right));

  cmd.linear_velocity_x = raw_x_left;
  cmd.linear_velocity_y = raw_y_left;
  cmd.angular_velocity_z = raw_y_right;

  // Integrate trigger differential into pelvis height target.
  const double pelvis_height_vel = rt - lt;
  current_pelvis_height_target_ += pelvis_height_vel / publisher_rate_hz_;
  current_pelvis_height_target_ = std::clamp(current_pelvis_height_target_, min_pelvis_height_, max_pelvis_height_);
  cmd.desired_pelvis_height = current_pelvis_height_target_;

  return true;
}

}  // namespace robot::remote_control
