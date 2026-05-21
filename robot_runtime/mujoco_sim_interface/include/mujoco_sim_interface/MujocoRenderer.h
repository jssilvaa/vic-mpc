#pragma once

// GLEW is required on Linux to load modern GL function pointers. On macOS the
// system OpenGL framework exposes them directly via GLFW, so skip GLEW there.
#if !defined(__APPLE__)
#include <GL/glew.h>
#endif

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "mujoco_sim_interface/MujocoUtils.h"
#include "robot_core/FPSTracker.h"

namespace robot::mujoco_sim_interface {

class MujocoSimInterface;

class MujocoRenderer {
 public:
  MujocoRenderer(const MujocoSimInterface* simInterface);

  ~MujocoRenderer();

  bool ok() const;

  void launchRenderThread();

  void waitForInit() const;

  // MujocoSimInterface drives renderLoop() directly on the main thread on
  // macOS where GLFW disallows window creation off the main thread.
  friend class MujocoSimInterface;

 private:
  // GLFW requires static callbacks; user pointer routes back to this instance.
  static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
  static void mouse_button(GLFWwindow* window, int button, int act, int mods);
  static void mouse_move(GLFWwindow* window, double xpos, double ypos);
  static void scroll(GLFWwindow* window, double xoffset, double yoffset);

  void setTransparency(float transparency) const;

  void renderLoop();
  void renderExternalForces();

  // Init must occur in the same thread that uses the OpenGL context.
  void initialize();
  void cleanup();

  const MujocoSimInterface* simInterface_;
  MjState simState_;

  std::thread render_thread_;

  GLFWwindow* window_;
  mjrRect viewport_ = {0, 0, 0, 0};

  int viewportWidth{1920};
  int viewportHeight{1024};

  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;

  double lastx = 0;
  double lasty = 0;

  double lastclicktm = 0;
  bool model_transparent = false;

  mjvCamera mujocoCam_;
  mjvOption mujocoOptions_;
  mjvScene mujocoScene_;
  mjrContext mujocoContext_;

  size_t timeStepMicro_;

  std::atomic<bool> window_closed_{false};
  std::atomic<bool> init_complete_{false};

  FPSTracker rendererFps_{"renderer"};
};

}  // namespace robot::mujoco_sim_interface
