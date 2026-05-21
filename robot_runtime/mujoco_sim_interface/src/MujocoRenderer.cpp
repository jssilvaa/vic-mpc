#include "mujoco_sim_interface/MujocoRenderer.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <condition_variable>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

#include "mujoco_sim_interface/MujocoSimInterface.h"

namespace robot::mujoco_sim_interface {

// GLFW callbacks

void MujocoRenderer::keyboard(GLFWwindow* window, int key, int, int act, int) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));

  if (act == GLFW_PRESS && key == GLFW_KEY_C) {
    renderer->mujocoOptions_.flags[mjVIS_CONTACTPOINT] = !renderer->mujocoOptions_.flags[mjVIS_CONTACTPOINT];
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_F) {
    renderer->mujocoOptions_.flags[mjVIS_CONTACTFORCE] = !renderer->mujocoOptions_.flags[mjVIS_CONTACTFORCE];
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_M) {
    renderer->mujocoOptions_.flags[mjVIS_COM] = !renderer->mujocoOptions_.flags[mjVIS_COM];
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_T) {
    renderer->model_transparent = !renderer->model_transparent;
    renderer->setTransparency(renderer->model_transparent ? 0.3f : 1.0f);
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_I) {
    renderer->mujocoOptions_.flags[mjVIS_INERTIA] = !renderer->mujocoOptions_.flags[mjVIS_INERTIA];
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_H) {
    renderer->mujocoOptions_.flags[mjVIS_CONVEXHULL] = !renderer->mujocoOptions_.flags[mjVIS_CONVEXHULL];
  }
  if (act == GLFW_PRESS && key == GLFW_KEY_P) {
    std::cerr << "\n\n==========================================================="
              << "\nHotkeys\n===========================================================\n"
              << "c => toggle contact point visualization\n"
              << "f => toggle contact force visualization\n"
              << "m => toggle center of mass visualization\n"
              << "t => toggle model transparency\n"
              << "i => toggle inertia visualization\n"
              << "h => toggle hull visualization\n";
  }
}

void MujocoRenderer::mouse_button(GLFWwindow* window, int, int, int) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  renderer->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  renderer->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  renderer->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &(renderer->lastx), &(renderer->lasty));
}

void MujocoRenderer::mouse_move(GLFWwindow* window, double xpos, double ypos) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  if (!renderer->button_left && !renderer->button_middle && !renderer->button_right) return;

  double dx = xpos - renderer->lastx;
  double dy = ypos - renderer->lasty;
  renderer->lastx = xpos;
  renderer->lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (renderer->button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (renderer->button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(renderer->simInterface_->getModel(), action, dx / width, dy / height, &renderer->mujocoScene_, &renderer->mujocoCam_);
}

void MujocoRenderer::scroll(GLFWwindow* window, double, double yoffset) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  mjv_moveCamera(renderer->simInterface_->getModel(), mjMOUSE_ZOOM, 0, -0.05 * yoffset, &renderer->mujocoScene_, &renderer->mujocoCam_);
}

MujocoRenderer::MujocoRenderer(const MujocoSimInterface* simInterface)
    : simInterface_(simInterface),
      simState_(simInterface_->getModel()),
      timeStepMicro_(1e6 / simInterface_->getConfig().renderFrequencyHz) {
  mujocoScene_.flags[mjRND_SHADOW] = 1;
  mujocoScene_.flags[mjRND_REFLECTION] = 1;
}

MujocoRenderer::~MujocoRenderer() {
  std::cerr << "Cleaning up renderer ..." << std::endl;
  if (render_thread_.joinable()) {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
    render_thread_.join();
  }
}

bool MujocoRenderer::ok() const { return !window_closed_.load(std::memory_order_acquire); }

void MujocoRenderer::launchRenderThread() { render_thread_ = std::thread(&MujocoRenderer::renderLoop, this); }

void MujocoRenderer::waitForInit() const {
  while (!init_complete_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MujocoRenderer::setTransparency(float transparency) const {
  for (int i = 0; i < simInterface_->getModel()->ngeom; i++) {
    simInterface_->getModel()->geom_rgba[4 * i + 3] = transparency;
  }
}

namespace {
void renderMetrics(const mjrContext* con, const mjrRect& viewport, const MjState& state, double fpsRender, double elapsed_time) {
  std::ostringstream metrics;
  metrics << "Render FPS: " << static_cast<int>(fpsRender) << "\n";
  metrics << "Sim FPS: " << static_cast<int>(state.metrics.fpsSim) << "\n";
  metrics << "Real Time[s]: " << std::fixed << std::setprecision(3) << elapsed_time << "\n";
  metrics << "Sim  Time[s]: " << std::fixed << std::setprecision(3) << state.data->time << "\n\n";
  metrics << "RTF: " << std::fixed << std::setprecision(3) << state.metrics.rtfTick << "\n";
  metrics << "Drift[ms]: " << std::fixed << std::setprecision(3) << state.metrics.driftTick * 1e3 << "\n";
  metrics << "Cumulative Drift[ms]: " << std::fixed << std::setprecision(3) << state.metrics.driftCumulative * 1e3;
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, metrics.str().c_str(), nullptr, con);
}
}  // namespace

void MujocoRenderer::renderExternalForces() {
  auto* model = simInterface_->getModel();
  auto* data = simState_.data;
  if (!model || !data) return;

  for (int body_id = 0; body_id < model->nbody; ++body_id) {
    const double* force = &data->xfrc_applied[6 * body_id];
    double fx = force[0], fy = force[1], fz = force[2];

    double magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (magnitude < 1e-6) continue;

    const double* xpos = &data->xpos[3 * body_id];

    mjvGeom* arrow = nullptr;
    if (mujocoScene_.ngeom < mujocoScene_.maxgeom) {
      arrow = &mujocoScene_.geoms[mujocoScene_.ngeom];
      mujocoScene_.ngeom++;
    } else {
      continue;
    }
    std::memset(arrow, 0, sizeof(mjvGeom));

    const double base_scale = 0.1;
    const double force_scale = 0.05;
    const double scale = base_scale + force_scale * magnitude;

    double end_x = xpos[0] + scale * (fx / magnitude);
    double end_y = xpos[1] + scale * (fy / magnitude);
    double end_z = xpos[2] + scale * (fz / magnitude);

    mjtNum from[3] = {xpos[0], xpos[1], xpos[2]};
    mjtNum to[3] = {end_x, end_y, end_z};

    mjv_connector(arrow, mjGEOM_ARROW, 0.005, from, to);

    arrow->rgba[0] = 0.5f;
    arrow->rgba[1] = 1.0f;
    arrow->rgba[2] = 0.0f;
    arrow->rgba[3] = 1.0f;
    arrow->category = mjCAT_DECOR;
    arrow->emission = 1.0f;
  }
}

void MujocoRenderer::renderLoop() {
  initialize();
  init_complete_.store(true);

  const auto start_time = std::chrono::steady_clock::now();

  while (!glfwWindowShouldClose(window_)) {
    auto start = std::chrono::steady_clock::now();

    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

    simInterface_->copyMjState(simState_);
    mj_forward(simInterface_->getModel(), simState_.data);

    mjv_updateScene(simInterface_->getModel(), simState_.data, &mujocoOptions_, nullptr, nullptr, mjCAT_ALL, &mujocoScene_);

    renderExternalForces();

    mjv_updateCamera(simInterface_->getModel(), simState_.data, &mujocoCam_, &mujocoScene_);
    mjr_render(viewport_, &mujocoScene_, &mujocoContext_);

    const auto current_time = std::chrono::steady_clock::now();
    const auto elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
    renderMetrics(&mujocoContext_, viewport_, simState_, rendererFps_.fps(), elapsed_time);

    glfwSwapBuffers(window_);
    glfwPollEvents();

    std::this_thread::sleep_until(start + std::chrono::microseconds(timeStepMicro_));

    rendererFps_.tick();
  }

  std::cerr << "Exited Mujoco renderLoop." << std::endl;

  window_closed_.store(true);
  cleanup();
}

void MujocoRenderer::initialize() {
  if (!glfwInit()) mju_error("Could not initialize GLFW");

  window_ = glfwCreateWindow(viewportWidth, viewportHeight, "Mujoco Robot Sim", nullptr, nullptr);
  glfwMakeContextCurrent(window_);

#if !defined(__APPLE__)
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return;
  }
#endif

  glfwSwapInterval(1);

  mjv_defaultCamera(&mujocoCam_);
  mjv_defaultOption(&mujocoOptions_);
  mjv_defaultScene(&mujocoScene_);
  mjr_defaultContext(&mujocoContext_);
  mjv_makeScene(simInterface_->getModel(), &mujocoScene_, 2000);
  mjr_makeContext(simInterface_->getModel(), &mujocoContext_, mjFONTSCALE_150);

  mujocoOptions_.flags[mjVIS_CONTACTPOINT] = 0;
  mujocoOptions_.flags[mjVIS_CONTACTFORCE] = 0;
  mujocoOptions_.flags[mjVIS_COM] = 0;
  mujocoOptions_.flags[mjVIS_INERTIA] = 0;

  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, keyboard);
  glfwSetCursorPosCallback(window_, mouse_move);
  glfwSetMouseButtonCallback(window_, mouse_button);
  glfwSetScrollCallback(window_, scroll);

  // Camera default: side view at 3m, target ~hip height.
  double arr_view[] = {89.608063, -5.588379, 3, 0.000000, 0.000000, 0.500000};
  mujocoCam_.azimuth = arr_view[0];
  mujocoCam_.elevation = arr_view[1];
  mujocoCam_.distance = arr_view[2];
  mujocoCam_.lookat[0] = arr_view[3];
  mujocoCam_.lookat[1] = arr_view[4];
  mujocoCam_.lookat[2] = arr_view[5];

  simInterface_->copyMjState(simState_);

  mj_step(simInterface_->getModel(), simState_.data);
  glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

  mjv_updateScene(simInterface_->getModel(), simState_.data, &mujocoOptions_, nullptr, &mujocoCam_, mjCAT_ALL, &mujocoScene_);

  mjr_render(viewport_, &mujocoScene_, &mujocoContext_);
  glfwSwapBuffers(window_);
  glfwPollEvents();
}

void MujocoRenderer::cleanup() {
  mjv_freeScene(&mujocoScene_);
  mjr_freeContext(&mujocoContext_);

  glfwDestroyWindow(window_);
  glfwTerminate();
}

}  // namespace robot::mujoco_sim_interface
