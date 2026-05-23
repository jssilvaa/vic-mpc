// Faithful port of the reference's mujocoSimNoTorques.cpp baseline: load the G1
// in MuJoCo and apply NO actuation. The robot falls under gravity and the
// MujocoSimInterface auto-resets when qpos[2] < 0.2. Useful as a zero-torque
// reference when something in the PD/MPC path misbehaves.
//
// Deviations from the reference (documented):
//  - No ROS: the reference resolves paths via ament_index_cpp::get_package_share_directory.
//    We take <scene.xml> <robot.urdf> as argv, matching mujocoSimPDStand.
//  - The reference runs a post-startSim() control loop that sets kp/kd; it's a
//    no-op there (it mutates by-value copies and never applies them). On macOS our
//    startSim() blocks (renderer on the calling/main thread), so there is no
//    post-startSim() loop at all — which is equivalent: zero torque either way.

#include <mujoco_sim_interface/MujocoSimInterface.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <scene.xml> <robot.urdf>\n"
              << "Example:\n"
              << "  " << argv[0]
              << " models/unitree_g1/g1_description/urdf/g1_29dof.xml"
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

  // Spawn the robot upright at 0.85 m (matches the reference exe).
  robot::model::RobotDescription robotDescription(urdfPath);
  auto initStatePtr = std::make_shared<robot::model::RobotState>(robotDescription);
  initStatePtr->setConfigurationToZero();
  initStatePtr->setRootPositionInWorldFrame(robot::vector3_t(0.0, 0.0, 0.85));

  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = scenePath;
  config.dt = 0.001;
  config.initStatePtr_ = initStatePtr;
  config.verbose = true;

  robot::mujoco_sim_interface::MujocoSimInterface sim(config, urdfPath);

  // No joint action is set: every JointAction keeps its default zero gains and
  // zero feed-forward, so getTotalFeedbackTorque() returns 0 for all joints.
  // The robot falls under gravity; the interface auto-resets at qpos[2] < 0.2.
  //
  // Blocks: spawns the sim thread internally, drives the renderer on this (main)
  // thread, returns when the viewer window is closed.
  sim.startSim();

  return 0;
}
