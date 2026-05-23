#pragma once

#include <iostream>
#include <string>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <chrono>
#include <ctime>
#include <memory>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <robot_model/RobotState.h>
#include "mujoco_sim_interface/MujocoRenderer.h"
#include "mujoco_sim_interface/MujocoUtils.h"
#include "robot_core/FPSTracker.h"
#include "robot_core/Types.h"
#include "robot_model/RobotHWInterfaceBase.h"

namespace robot::mujoco_sim_interface {

struct MujocoSimConfig {
  std::string scenePath;
  std::shared_ptr<model::RobotState> initStatePtr_;
  double dt{0.0005};
  double renderFrequencyHz{60.0};
  bool headless{false};
  bool verbose{false};
};

class MujocoSimInterface : public robot::model::RobotHWInterfaceBase {
 public:
  MujocoSimInterface(const MujocoSimConfig& config, const std::string& urdfPath);

  ~MujocoSimInterface();

  void initSim();
  void startSim();
  void simulationStep();
  void reset();

  // Allows the renderer to make a thread-safe copy of the state at its own frequency.
  void copyMjState(MjState& state) const;

  const mjModel* getModel() const { return mujocoModel_; }
  const MujocoSimConfig& getConfig() const { return config_; }

  // Inspection hooks for offline tools (e.g. pinocchioCrossCheck) that drive the
  // sim single-threaded: perturb qpos/qvel, mj_forward, then read qM/qfrc_bias.
  // Not for use while the sim thread is running.
  mjData* getData() { return mujocoData_; }
  // Refresh the thread-safe RobotState snapshot from the current mjData (so
  // getRobotState() reflects a pose set directly on mjData + mj_forward).
  void syncStateFromData() { updateThreadSafeRobotState(); }

 private:
  void setupJointIndexMaps();
  void setSimState(const model::RobotState& robotState);
  void updateThreadSafeRobotState();
  void simulationLoop();
  void runRendererOnThisThread(); 
  void printModelInfo();
  void updateMetrics();

  MujocoSimConfig config_;

  model::RobotState robotStateInternal_;
  mjtNum* qpos_init_{nullptr};
  mjtNum* qvel_init_{nullptr};
  model::RobotJointAction robotJointActionInternal_;

  size_t timeStepMicro_;
  double simStart_;
  size_t nActiveJoints_;
  size_t nActuators_;
  std::vector<std::string> activeMuJoCoJointNames_;
  std::vector<std::string> activeMuJoCoActuatorNames_;
  std::vector<joint_index_t> activeRobotJointStateIndices_;
  std::vector<joint_index_t> activeRobotActuatorIndices_;

  mjModel* mujocoModel_ = nullptr;
  mjData* mujocoData_ = nullptr;
  mjContact* mujocoContact_ = nullptr;

  bool simInit_{false};
  const bool headless_;
  const bool verbose_;
  std::atomic<bool> terminate_{false};
  std::atomic<bool> guiInitialized_{false};

  mutable std::mutex mujocoMutex_;  // Guards mujoco model+data across sim and render threads.
  std::thread simulate_thread_;
  std::unique_ptr<MujocoRenderer> renderer_;

  FPSTracker simFps_{"mujoco_sim"};
  std::chrono::high_resolution_clock::time_point lastRealTime_;
  Metrics metrics_{};

  // Sentinel value when the matching sensor isn't present in the MJCF.
  static constexpr size_t kInvalidSensorAddr = static_cast<size_t>(-1);
  size_t right_foot_sensor_addr_{kInvalidSensorAddr};
  size_t left_foot_sensor_addr_{kInvalidSensorAddr};
  size_t right_foot_touch_sensor_addr_{kInvalidSensorAddr};
  size_t left_foot_touch_sensor_addr_{kInvalidSensorAddr};
};

}  // namespace robot::mujoco_sim_interface
