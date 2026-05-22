#include "mujoco_sim_interface/MujocoSimInterface.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace robot::mujoco_sim_interface {

MjState::MjState(const mjModel* mujocoModel_) : data(mj_makeData(mujocoModel_)) {}

MujocoSimInterface::MujocoSimInterface(const MujocoSimConfig& config, const std::string& urdfPath)
    : RobotHWInterfaceBase(urdfPath),
      config_(config),
      robotStateInternal_(model::RobotState(this->getRobotDescription())),
      robotJointActionInternal_(model::RobotJointAction(this->getRobotDescription())),
      headless_(config.headless),
      verbose_(config.verbose) {
  lastRealTime_ = std::chrono::high_resolution_clock::now();
  const int errstr_sz = 1000;
  char errstr[errstr_sz];

  mujocoModel_ = mj_loadXML(config.scenePath.c_str(), nullptr, errstr, errstr_sz);
  if (!mujocoModel_) {
    std::cerr << "Could not load MuJoCo model: " << config.scenePath << ". Error: " << errstr << std::endl;
    throw std::runtime_error("Could not load MuJoCo: " + std::string(errstr));
  }

  mujocoData_ = mj_makeData(mujocoModel_);

  srand(time(nullptr));

  mujocoContact_ = mujocoData_->contact;

  simStart_ = mujocoData_->time;

  mujocoModel_->opt.timestep = config_.dt;

  timeStepMicro_ = static_cast<size_t>(config_.dt * 1000000);

  if (verbose_) printModelInfo();

  setupJointIndexMaps();

  model::RobotState initRobotState(getRobotDescription(), 2);

  if (config_.initStatePtr_ != nullptr) {
    initRobotState = *config.initStatePtr_;
  } else {
    initRobotState.setConfigurationToZero();
    initRobotState.setRootPositionInWorldFrame(vector3_t(0.0, 0.0, 1.0));
  }
  setSimState(initRobotState);

  scalar_t defaultJointDamping = 10.0;

  for (int i = 6; i < mujocoModel_->nv; ++i) {
    std::string mjJointName(&mujocoModel_->names[mujocoModel_->name_jntadr[mujocoModel_->dof_jntid[i]]]);
    if (verbose_) std::cerr << "mjJointName: " << mjJointName << std::endl;
    mujocoModel_->dof_damping[i] = defaultJointDamping;
  }

  for (int i = 0; i < mujocoModel_->nsensor; i++) {
    std::string sensorName(&mujocoModel_->names[mujocoModel_->name_sensoradr[i]]);

    if (sensorName == "right_foot_touch_sensor") right_foot_touch_sensor_addr_ = mujocoModel_->sensor_adr[i];
    if (sensorName == "left_foot_touch_sensor") left_foot_touch_sensor_addr_ = mujocoModel_->sensor_adr[i];
    if (sensorName == "right_foot_force_sensor") right_foot_sensor_addr_ = mujocoModel_->sensor_adr[i];
    if (sensorName == "left_foot_force_sensor") left_foot_sensor_addr_ = mujocoModel_->sensor_adr[i];
  }

  qpos_init_ = new mjtNum[mujocoModel_->nq];
  qvel_init_ = new mjtNum[mujocoModel_->nv];

  memcpy(qpos_init_, mujocoData_->qpos, mujocoModel_->nq * sizeof(mjtNum));
  memcpy(qvel_init_, mujocoData_->qvel, mujocoModel_->nv * sizeof(mjtNum));

  updateThreadSafeRobotState();
  updateInterfaceStateFromRobot();
}

MujocoSimInterface::~MujocoSimInterface() {
  terminate_.store(true);
  if (simulate_thread_.joinable()) simulate_thread_.join();
}

void MujocoSimInterface::reset() {
  memcpy(mujocoData_->qpos, qpos_init_, mujocoModel_->nq * sizeof(mjtNum));
  memcpy(mujocoData_->qvel, qvel_init_, mujocoModel_->nv * sizeof(mjtNum));
}

void MujocoSimInterface::copyMjState(MjState& state) const {
  std::lock_guard<std::mutex> guard(mujocoMutex_);
  state.timestamp = mujocoData_->time;
  mj_copyData(state.data, mujocoModel_, mujocoData_);
  state.metrics = metrics_;
}

void MujocoSimInterface::setupJointIndexMaps() {
  for (int i = 1; i < mujocoModel_->njnt; ++i) {
    const std::string jointName(&mujocoModel_->names[mujocoModel_->name_jntadr[i]]);
    if (getRobotDescription().containsJoint(jointName)) {
      activeMuJoCoJointNames_.emplace_back(jointName);
    } else {
      std::cerr << "WARNING: Joint contained in mujoco xml not exposed to RobotHWInterface: " << jointName << std::endl;
    }
  }

  activeRobotJointStateIndices_ = getRobotDescription().getJointIndices(activeMuJoCoJointNames_);

  for (int i = 0; i < mujocoModel_->nu; ++i) {
    const std::string actuator_name = mj_id2name(mujocoModel_, mjOBJ_ACTUATOR, i);
    if (getRobotDescription().containsJoint(actuator_name)) {
      activeMuJoCoActuatorNames_.emplace_back(actuator_name);
    } else {
      std::cerr << "WARNING: Actuator contained in mujoco xml not exposed to RobotHWInterface: " << actuator_name << std::endl;
    }
  }

  activeRobotActuatorIndices_ = getRobotDescription().getJointIndices(activeMuJoCoActuatorNames_);

  nActiveJoints_ = activeRobotJointStateIndices_.size();
  nActuators_ = activeRobotActuatorIndices_.size();
  if (verbose_) {
    std::cerr << "Initialized " << nActiveJoints_ << " active Joints" << std::endl;
    std::cerr << "Initialized " << nActuators_ << " active Actuators" << std::endl;
  }
}

void MujocoSimInterface::printModelInfo() {
  std::cerr << "timeStepMicro_: " << timeStepMicro_ << std::endl;
  std::cerr << "njnt: " << mujocoModel_->njnt << std::endl;
  std::cerr << "nq: " << mujocoModel_->nq << std::endl;
  std::cerr << "nv: " << mujocoModel_->nv << std::endl;
  std::cerr << "nu: " << mujocoModel_->nu << std::endl;

  for (int i = 0; i < mujocoModel_->nbody; ++i) {
    std::string bodyName(&mujocoModel_->names[mujocoModel_->name_bodyadr[i]]);
    std::cerr << "Body " << i << ": " << bodyName << std::endl;
    std::cerr << "  Position: ";
    for (size_t j = 0; j < 3; ++j) std::cerr << mujocoData_->xpos[i * 3 + j] << " ";
    std::cerr << std::endl;
    std::cerr << "  Orientation (Quaternion): ";
    for (size_t j = 0; j < 4; ++j) std::cerr << mujocoData_->xquat[i * 4 + j] << " ";
    std::cerr << std::endl;
  }

  scalar_t totalMass = 0.0;
  for (int i = 0; i < mujocoModel_->nbody; i++) totalMass += mujocoModel_->body_mass[i];
  std::cerr << "Total MuJoCo model mass: " << totalMass << std::endl;
}

void MujocoSimInterface::setSimState(const model::RobotState& robotState) {
  vector3_t rootPosition = robotState.getRootPositionInWorldFrame();
  quaternion_t quat_l_to_w = robotState.getRootRotationLocalToWorldFrame();

  mujocoData_->qpos[0] = rootPosition[0];
  mujocoData_->qpos[1] = rootPosition[1];
  mujocoData_->qpos[2] = rootPosition[2];
  mujocoData_->qpos[3] = quat_l_to_w.w();
  mujocoData_->qpos[4] = quat_l_to_w.x();
  mujocoData_->qpos[5] = quat_l_to_w.y();
  mujocoData_->qpos[6] = quat_l_to_w.z();

  vector3_t root_vel_lin_world_frame = quat_l_to_w * robotState.getRootLinearVelocityInLocalFrame();
  vector3_t root_vel_ang_local_frame = robotState.getRootAngularVelocityInLocalFrame();

  mujocoData_->qvel[0] = root_vel_lin_world_frame[0];
  mujocoData_->qvel[1] = root_vel_lin_world_frame[1];
  mujocoData_->qvel[2] = root_vel_lin_world_frame[2];
  mujocoData_->qvel[3] = root_vel_ang_local_frame[0];
  mujocoData_->qvel[4] = root_vel_ang_local_frame[1];
  mujocoData_->qvel[5] = root_vel_ang_local_frame[2];

  for (size_t i = 0; i < nActiveJoints_; ++i) {
    mujocoData_->qpos[i + 7] = robotState.getJointPosition(activeRobotJointStateIndices_[i]);
    mujocoData_->qvel[i + 6] = robotState.getJointVelocity(activeRobotJointStateIndices_[i]);
  }
}

void MujocoSimInterface::updateThreadSafeRobotState() {
  for (size_t i = 0; i < nActiveJoints_; ++i) {
    robotStateInternal_.setJointPosition(activeRobotJointStateIndices_[i], mujocoData_->qpos[i + 7]);
    robotStateInternal_.setJointVelocity(activeRobotJointStateIndices_[i], mujocoData_->qvel[i + 6]);
  }

  // MuJoCo free joint quaternion is [w, x, y, z].
  quaternion_t quat_l_to_w = quaternion_t(mujocoData_->qpos[3], mujocoData_->qpos[4], mujocoData_->qpos[5], mujocoData_->qpos[6]);
  vector3_t pelvisAngularVelLocal = vector3_t(mujocoData_->qvel[3], mujocoData_->qvel[4], mujocoData_->qvel[5]);

  // TODO: hook up real contact detection from foot touch sensors when present.
  bool leftFootContact = true;
  bool rightFootContact = true;

  robotStateInternal_.setRootPositionInWorldFrame(vector3_t(mujocoData_->qpos[0], mujocoData_->qpos[1], mujocoData_->qpos[2]));
  robotStateInternal_.setRootRotationLocalToWorldFrame(quat_l_to_w);
  robotStateInternal_.setRootLinearVelocityInLocalFrame(quat_l_to_w.inverse() *
                                                        vector3_t(mujocoData_->qvel[0], mujocoData_->qvel[1], mujocoData_->qvel[2]));
  robotStateInternal_.setRootAngularVelocityInLocalFrame(pelvisAngularVelLocal);
  robotStateInternal_.setContactFlag(0, leftFootContact);
  robotStateInternal_.setContactFlag(1, rightFootContact);

  robotStateInternal_.setTime(mujocoData_->time);

  threadSafeRobotState_.set(robotStateInternal_);
}

void MujocoSimInterface::updateMetrics() {
  simFps_.tick();
  metrics_.fpsSim = simFps_.fps();

  auto nowRealTime = std::chrono::high_resolution_clock::now();
  auto realElapsedTime = std::chrono::duration<double>(nowRealTime - lastRealTime_).count();
  lastRealTime_ = nowRealTime;

  metrics_.driftTick = config_.dt - realElapsedTime;
  metrics_.driftCumulative += metrics_.driftTick;
  metrics_.rtfTick = config_.dt / realElapsedTime;
}

void MujocoSimInterface::simulationStep() {
  threadSafeRobotJointAction_.copy_value(robotJointActionInternal_);
  for (size_t i = 0; i < nActuators_; ++i) {
    joint_index_t idx = activeRobotActuatorIndices_[i];
    const robot::model::JointAction& jointAction = robotJointActionInternal_.at(idx).value();
    mujocoData_->ctrl[i] =
        jointAction.getTotalFeedbackTorque(robotStateInternal_.getJointPosition(idx), robotStateInternal_.getJointVelocity(idx));
  }

  bool collapsed = false;
  {
    std::lock_guard<std::mutex> guard(mujocoMutex_);
    mj_step(mujocoModel_, mujocoData_);
    updateThreadSafeRobotState();
    updateMetrics();

    // Auto-reset if the robot collapses.
    if (mujocoData_->qpos[2] < 0.2) {
      reset();
      for (size_t i = 0; i < nActuators_; ++i) mujocoData_->ctrl[i] = 0.0;
      mj_step(mujocoModel_, mujocoData_);
      updateThreadSafeRobotState();
      simFps_.reset();
      metrics_.reset();
      updateMetrics();
      collapsed = true;
    }
  }

  // Sleep after releasing the mutex so the renderer can keep drawing during the
  // 1 s settle pause.
  if (collapsed) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void MujocoSimInterface::simulationLoop() {
  simFps_.reset();
  metrics_.reset();
  auto nextWakeup = std::chrono::steady_clock::now();
  while (!terminate_.load()) {
    simulationStep();
    nextWakeup += std::chrono::microseconds(timeStepMicro_);
    std::this_thread::sleep_until(nextWakeup);
  }
}

void MujocoSimInterface::runRendererOnThisThread() {
  if (renderer_) renderer_->renderLoop();
}

void MujocoSimInterface::initSim() {
  simulationStep();
  simInit_ = true;

  if (!headless_) {
    // Renderer owned but not launched here — on macOS GLFW requires the
    // main thread, so startSim() drives renderLoop() on the calling thread
    // *after* the sim thread is spawned.
    renderer_.reset(new MujocoRenderer(this));
  }
}

void MujocoSimInterface::startSim() {
  if (!simInit_) initSim();
  // Sim runs on a side thread; renderer runs on the calling (main) thread
  // and blocks until the viewer window is closed. Order matters: spawn the
  // sim thread *before* entering the blocking renderer.
  simulate_thread_ = std::thread(&MujocoSimInterface::simulationLoop, this);
  if (!headless_) runRendererOnThisThread();
}

}  // namespace robot::mujoco_sim_interface
