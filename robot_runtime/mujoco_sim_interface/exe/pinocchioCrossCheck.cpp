// D1 validation gate: cross-check Pinocchio's mass matrix M(q), foot Jacobian,
// and nonlinear bias forces against MuJoCo's, at a non-trivial perturbed pose.
//
// This is a vic-mpc-only test harness (the reference has no MuJoCo<->Pinocchio
// cross-check). It validates createDefaultPinocchioInterface + robotStateToPinocchio
// end-to-end before any centroidal/WB dynamics trust those quantities.
//
// IMPORTANT — joint blocks only: MuJoCo's free-joint base velocity is
// [v_world, omega_local] while our Pinocchio model uses the Euler-ZYX base
// [v_world, eulerZYX_rate]. Those are different coordinates for the same 6 base
// DoF, so the base 6x6 / base-joint coupling blocks of M (and base columns of J)
// are EXPECTED to differ. We compare only the joint-space blocks (indices >= 6),
// which are coordinate-independent given matched configuration + joint ordering.

#include <pinocchio/fwd.hpp>

#include <mujoco/mujoco.h>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

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

constexpr double kDefaultBaseHeight = 0.7925;
constexpr double kTol = 1e-9;

// Foot frame present identically in URDF (pinocchio frame) and MJCF (mujoco body).
constexpr char kFootFrame[] = "left_ankle_roll_link";

}  // namespace

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

  // ---- §1 Sim setup (headless, zero-gain actions so initSim's step is a no-op) ----
  robot::model::RobotDescription robotDescription(urdfPath);
  auto initStatePtr = std::make_shared<robot::model::RobotState>(robotDescription);
  initStatePtr->setConfigurationToZero();
  initStatePtr->setRootPositionInWorldFrame(robot::vector3_t(0.0, 0.0, kDefaultBaseHeight));
  for (const auto& [name, q] : defaultJointPosture()) {
    if (robotDescription.containsJoint(name)) {
      initStatePtr->setJointPosition(robotDescription.getJointIndex(name), q);
    }
  }

  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = scenePath;
  config.headless = true;
  config.initStatePtr_ = initStatePtr;
  robot::mujoco_sim_interface::MujocoSimInterface sim(config, urdfPath);

  auto& action = sim.getRobotJointAction();
  for (robot::joint_index_t id : sim.getRobotDescription().getJointIndices()) {
    auto& opt = action.at(id);
    if (opt) { opt->kp = 0; opt->kd = 0; opt->feed_forward_effort = 0; }
  }
  sim.applyJointAction();
  sim.initSim();

  // ---- §2 Pinocchio interface ----
  auto pin = ocs2::createDefaultPinocchioInterface(urdfPath);
  const auto& model = pin.getModel();
  auto& data = pin.getData();
  const mjModel* m = sim.getModel();
  mjData* d = sim.getData();

  if (model.nv != m->nv) {
    std::cerr << "nv mismatch: pinocchio " << model.nv << " vs mujoco " << m->nv << std::endl;
    return 1;
  }
  const int nv = m->nv;

  // ---- §3/§4 Joint id list (Pinocchio order) + velocity-column permutation ----
  // jointIds: robot::model joint index per Pinocchio joint, in Pinocchio order
  //           (model.names[0]="universe", [1]=composite base, real joints from 2).
  // columnPermutation: (pinocchio velocity-column, mujoco DoF index) per joint.
  std::vector<robot::joint_index_t> jointIds;
  std::vector<std::pair<int, int>> columnPermutation;  // (pinCol, mjCol)
  for (pinocchio::JointIndex jid = 2; jid < (pinocchio::JointIndex)model.njoints; ++jid) {
    const std::string& name = model.names[jid];
    jointIds.push_back(sim.getRobotDescription().getJointIndex(name));

    const int pinCol = model.idx_vs[jid];
    const int mjJointId = mj_name2id(m, mjOBJ_JOINT, name.c_str());
    if (mjJointId < 0) {
      std::cerr << "Joint '" << name << "' not found in MuJoCo model." << std::endl;
      return 1;
    }
    const int mjCol = m->jnt_dofadr[mjJointId];
    columnPermutation.emplace_back(pinCol, mjCol);
  }

  // ---- §5 Perturb to a non-trivial pose, then mj_forward ----
  // Base position.
  d->qpos[0] = 0.10; d->qpos[1] = -0.20; d->qpos[2] = 0.85;
  // Base orientation: ZYX-Euler -> quaternion, pitch well clear of +/-pi/2.
  {
    const double yaw = -0.35, pitch = 0.25, roll = 0.15;
    Eigen::Quaterniond quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    quat.normalize();
    d->qpos[3] = quat.w(); d->qpos[4] = quat.x(); d->qpos[5] = quat.y(); d->qpos[6] = quat.z();
  }
  // Joint positions (qpos[7..]) and all velocities (qvel) — deterministic, nonzero.
  for (int i = 0; i < nv - 6; ++i) d->qpos[7 + i] = 0.15 * std::sin(0.7 * i + 1.0);
  d->qvel[0] = 0.10; d->qvel[1] = -0.20; d->qvel[2] = 0.15;   // base linear (world frame)
  d->qvel[3] = 0.05; d->qvel[4] = -0.10; d->qvel[5] = 0.20;   // base angular (local frame)
  for (int i = 0; i < nv - 6; ++i) d->qvel[6 + i] = 0.10 * std::cos(0.5 * i + 0.3);

  mj_forward(m, d);                  // fills d->qM, d->qfrc_bias, kinematics
  sim.syncStateFromData();           // mjData -> thread-safe RobotState
  sim.updateInterfaceStateFromRobot();
  const auto& rs = sim.getRobotState();

  // ---- §6 Fill Pinocchio (q, v) from the RobotState ----
  ocs2::vector_t q, v;
  ocs2::humanoid::robotStateToPinocchio(model, rs, jointIds, q, v);

  // ---- §7 Compute both sides ----
  // Pinocchio mass matrix (crba fills upper triangle only -> symmetrize).
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // Pinocchio nonlinear effects C(q,v)v + g.
  pinocchio::nonLinearEffects(model, data, q, v);
  // Pinocchio foot Jacobian, world-aligned at the frame origin (matches mj_jacBody).
  const auto fid = model.getFrameId(kFootFrame);
  Eigen::Matrix<double, 6, Eigen::Dynamic> Jpin(6, nv);
  Jpin.setZero();
  pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, Jpin);

  // MuJoCo mass matrix (dense, row-major), bias forces, body Jacobian.
  std::vector<mjtNum> Mfull(static_cast<size_t>(nv) * nv);
  mj_fullM(m, Mfull.data(), d->qM);
  const int bid = mj_name2id(m, mjOBJ_BODY, kFootFrame);
  if (bid < 0) {
    std::cerr << "Body '" << kFootFrame << "' not found in MuJoCo model." << std::endl;
    return 1;
  }
  std::vector<mjtNum> Jp(static_cast<size_t>(3) * nv), Jr(static_cast<size_t>(3) * nv);
  mj_jacBody(m, d, Jp.data(), Jr.data(), bid);

  // ---- Model-parameter diagnostic: total mass (URDF vs MJCF)
  // The URDF and MJCF are authored separately for the G1; mass differs
  const double massPin = pinocchio::computeTotalMass(model);
  double massMj = 0.0;
  for (int b = 1; b < m->nbody; ++b) massMj += m->body_mass[b];  // body 0 = world

  // ---- §8 Compare joint blocks only
  double maxErrM = 0.0, maxErrBias = 0.0, maxErrJ = 0.0;
  int worstBiasPin = -1, worstBiasMj = -1;

  for (size_t a = 0; a < columnPermutation.size(); ++a) {
    const int pinI = columnPermutation[a].first;
    const int mjI = columnPermutation[a].second;

    const double biasErr = std::abs(data.nle[pinI] - d->qfrc_bias[mjI]);
    if (biasErr > maxErrBias) { maxErrBias = biasErr; worstBiasPin = pinI; worstBiasMj = mjI; }

    // Jacobian column: pinocchio rows [0:3]=linear, [3:6]=angular;
    //                  mujoco Jp (3xnv linear), Jr (3xnv angular), row-major.
    for (int r = 0; r < 3; ++r) {
      maxErrJ = std::max(maxErrJ, std::abs(Jpin(r, pinI) - Jp[static_cast<size_t>(r) * nv + mjI]));
      maxErrJ = std::max(maxErrJ, std::abs(Jpin(3 + r, pinI) - Jr[static_cast<size_t>(r) * nv + mjI]));
    }

    // Mass matrix: data.M(pinI,pinJ) vs Mfull[mjI*nv + mjJ]  (Mfull row-major).
    for (size_t b = 0; b < columnPermutation.size(); ++b) {
      const int pinJ = columnPermutation[b].first;
      const int mjJ = columnPermutation[b].second;
      maxErrM = std::max(maxErrM,
                         std::abs(data.M(pinI, pinJ) - Mfull[static_cast<size_t>(mjI) * nv + mjJ]));
    }
  }
  
  constexpr double kTolJ = 1e-6;     // kinematics: world-aligned foot Jacobian
  constexpr double kTolM = 1e-3;     // joint-space inertia (model-param limited)
  constexpr double kTolBias = 1e-1;  // joint-space bias forces (model-param limited)

  std::cout << std::scientific << std::setprecision(3);
  std::cout << "pinocchioCrossCheck @ perturbed pose (joint blocks, indices >= 6)\n";
  std::cout << "  total mass:  pinocchio(URDF) = " << massPin
            << "   mujoco(MJCF) = " << massMj
            << "   |diff| = " << std::abs(massPin - massMj) << " kg\n";
  std::cout << "  max |J_pin   - J_mj   | = " << maxErrJ    << "   (tol " << kTolJ    << ")  [kinematics]\n";
  std::cout << "  max |M_pin   - M_mj   | = " << maxErrM    << "   (tol " << kTolM    << ")  [inertia]\n";
  std::cout << "  max |nle_pin - bias_mj| = " << maxErrBias << "   (tol " << kTolBias << ")  [bias]"
            << "   worst @ pinCol=" << worstBiasPin << " mjCol=" << worstBiasMj << "\n";

  const bool ok = (maxErrJ < kTolJ) && (maxErrM < kTolM) && (maxErrBias < kTolBias);
  std::cout << (ok ? "PASS — kinematics+ordering validated; M/bias within model-authoring precision\n"
                   : "FAIL\n");
  return ok ? 0 : 1;
}
