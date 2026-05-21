// D0 deliverable: load G1 + floor in MuJoCo, hold a default joint pose with
// per-joint PD around the centroidal-MPC reference posture, and (if a gamepad
// is connected) print live xbox-controller axes to verify the gamepad path.
//
// Modeled on examples/wb_humanoid_mpc/robot_runtime/mujoco_sim_interface/exe/
// mujocoSimNoTorques.cpp from the reference impl.

#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <remote_control/XboxControllerInterface.h>
#include <remote_control/WalkingCommand.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

namespace {

// Default joint posture from
// models/unitree_g1/g1_centroidal_mpc/config/command/reference.info
// (defaultJointState block). Wrist joints (excluded from MPC) are left at 0.
const std::unordered_map<std::string, double>& defaultJointPosture() {
  static const std::unordered_map<std::string, double> q = {
      {"left_hip_pitch_joint", -0.05},   {"left_hip_roll_joint", 0.0},      {"left_hip_yaw_joint", 0.0},
      {"left_knee_joint", 0.1},          {"left_ankle_pitch_joint", -0.05}, {"left_ankle_roll_joint", 0.0},
      {"right_hip_pitch_joint", -0.05},  {"right_hip_roll_joint", 0.0},     {"right_hip_yaw_joint", 0.0},
      {"right_knee_joint", 0.1},         {"right_ankle_pitch_joint", -0.05},{"right_ankle_roll_joint", 0.0},
      {"waist_yaw_joint", 0.0},          {"waist_roll_joint", 0.0},         {"waist_pitch_joint", 0.0},
      {"left_shoulder_pitch_joint", 0.0},{"left_shoulder_roll_joint", 0.0}, {"left_shoulder_yaw_joint", 0.0},
      {"left_elbow_joint", 0.0},         {"right_shoulder_pitch_joint", 0.0},{"right_shoulder_roll_joint", 0.0},
      {"right_shoulder_yaw_joint", 0.0}, {"right_elbow_joint", 0.0},        {"left_wrist_roll_joint", 0.0},
      {"left_wrist_pitch_joint", 0.0},   {"left_wrist_yaw_joint", 0.0},     {"right_wrist_roll_joint", 0.0},
      {"right_wrist_pitch_joint", 0.0},  {"right_wrist_yaw_joint", 0.0},
  };
  return q;
}

constexpr double kDefaultBaseHeight = 0.7925;  // from reference.info
constexpr double kJointKp = 1500.0;
constexpr double kJointKd = 2.0;

void gamepadPollLoop() {
  // No-op until a controller is plugged in; logs once on connect.
  robot::remote_control::XboxControllerInterface gamepad;
  ocs2::humanoid::WalkingVelocityCommand cmd;

  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  const auto period = std::chrono::milliseconds(100);  // 10 Hz console log

  while (true) {
    gamepad.poll(cmd);

    std::cout << std::fixed << std::setprecision(3)
              << "[gamepad] connected=" << (gamepad.connected() ? "yes" : "no ")
              << "  vx=" << cmd.linear_velocity_x
              << "  vy=" << cmd.linear_velocity_y
              << "  wz=" << cmd.angular_velocity_z
              << "  pelvis_h=" << cmd.desired_pelvis_height << "        \r" << std::flush;

    next += period;
    std::this_thread::sleep_until(next);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <scene.xml> <robot.urdf>\n"
              << "Example:\n"
              << "  " << argv[0]
              << " models/unitree_g1/g1_description/urdf/g1_scene.xml"
              << " models/unitree_g1/g1_description/urdf/g1_29dof.urdf\n";
    return 1;
  }
  const std::string scenePath = argv[1];
  const std::string urdfPath = argv[2];

  if (!std::filesystem::exists(scenePath)) {
    std::cerr << "Scene file not found: " << scenePath << std::endl;
    return 1;
  }
  if (!std::filesystem::exists(urdfPath)) {
    std::cerr << "URDF file not found: " << urdfPath << std::endl;
    return 1;
  }

  // Build the desired initial robot state from the centroidal-MPC reference posture.
  robot::model::RobotDescription robotDescription(urdfPath);
  auto initStatePtr = std::make_shared<robot::model::RobotState>(robotDescription);
  initStatePtr->setConfigurationToZero();
  initStatePtr->setRootPositionInWorldFrame(robot::vector3_t(0.0, 0.0, kDefaultBaseHeight));
  for (const auto& [name, q] : defaultJointPosture()) {
    if (robotDescription.containsJoint(name)) {
      initStatePtr->setJointPosition(robotDescription.getJointIndex(name), q);
    }
  }

  // Configure and start the sim.
  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = scenePath;
  config.dt = 0.001;
  config.initStatePtr_ = initStatePtr;
  config.verbose = true;

  robot::mujoco_sim_interface::MujocoSimInterface sim(config, urdfPath);

  // Stamp PD setpoints + gains into every joint action once. The sim thread
  // reads these every step.
  auto& action = sim.getRobotJointAction();
  for (const auto& [name, q_des] : defaultJointPosture()) {
    if (!robotDescription.containsJoint(name)) continue;
    auto& opt = action.at(robotDescription.getJointIndex(name));
    if (opt) {
      opt->q_des = q_des;
      opt->qd_des = 0.0;
      opt->kp = kJointKp;
      opt->kd = kJointKd;
      opt->feed_forward_effort = 0.0;
    }
  }
  sim.applyJointAction();

  // Gamepad poller runs as a daemon side thread — detached because its loop
  // is unconditional and we have no shutdown signal for it. When main exits,
  // the OS reclaims the thread.
  std::thread(&gamepadPollLoop).detach();

  // Blocking: spawns the sim thread internally, then drives the renderer on
  // this (main) thread. Returns when the viewer window is closed; sim thread
  // is then joined in the MujocoSimInterface destructor.
  sim.startSim();

  return 0;
}
