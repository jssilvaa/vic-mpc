// D3 #29d verify (throwaway): exercise the centroidal model layer end-to-end.
//   config + URDF -> custom Pinocchio interface (with foot_l/r_contact frames)
//   -> CentroidalModelInfo -> CentroidalMpcRobotModel.
// Checks: dims 35/35, robotMass ~ 35.1, endEffectorFrameIndices resolved to REAL
// frames (not the model.frames.size() sentinel — the whole point of 29a/29b), a state
// get/set round-trip, and the A_b^-1 generalized-velocity solve (getGeneralizedVelocities).

#include <pinocchio/fwd.hpp>

#include <iomanip>
#include <iostream>
#include <string>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <task.info> <robot.urdf> <reference.info>\n";
    return 1;
  }
  const std::string taskFile = argv[1];
  const std::string urdfFile = argv[2];
  const std::string referenceFile = argv[3];

  using namespace ocs2;
  using namespace ocs2::humanoid;

  int failures = 0;
  auto check = [&](bool ok, const std::string& what) {
    std::cout << (ok ? "  [ok]   " : "  [FAIL] ") << what << "\n";
    if (!ok) ++failures;
  };

  // ---- config -> custom Pinocchio interface (adds foot_l/r_contact frames) ----
  ModelSettings settings(taskFile, urdfFile, "centroidal_", /*verbose=*/false);
  PinocchioInterface pin = createCustomPinocchioInterface(taskFile, urdfFile, settings);
  const auto& model = pin.getModel();

  // ---- CentroidalModelInfo ----
  const auto type = centroidal_model::loadCentroidalType(taskFile);
  const vector_t nominalJointAngles = centroidal_model::loadDefaultJointState(settings.mpc_joint_dim, referenceFile);
  const auto info = centroidal_model::createCentroidalModelInfo(pin, type, nominalJointAngles, settings.contactNames3DoF,
                                                                settings.contactNames6DoF);

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "=== CentroidalModelInfo ===\n";
  std::cout << "  stateDim=" << info.stateDim << "  inputDim=" << info.inputDim
            << "  genCoords=" << info.generalizedCoordinatesNum << "  robotMass=" << info.robotMass << " kg\n";
  check(info.stateDim == 35, "stateDim == 35");
  check(info.inputDim == 35, "inputDim == 35");
  check(info.generalizedCoordinatesNum == 29, "generalizedCoordinatesNum == 29");
  check(info.robotMass > 34.5 && info.robotMass < 35.5, "robotMass ~ 35 kg");
  check(info.numSixDofContacts == 2, "numSixDofContacts == 2");

  // The 29a/29b payoff: contact frames must resolve to REAL frames, not the sentinel.
  const auto sentinel = static_cast<size_t>(model.frames.size());
  std::cout << "  endEffectorFrameIndices:";
  bool framesValid = (info.endEffectorFrameIndices.size() == info.numSixDofContacts);
  for (size_t i = 0; i < info.endEffectorFrameIndices.size(); ++i) {
    const size_t fid = info.endEffectorFrameIndices[i];
    const bool valid = fid < sentinel;
    framesValid = framesValid && valid;
    std::cout << " " << settings.contactNames6DoF[i] << "->" << fid << (valid ? "" : "(SENTINEL!)");
  }
  std::cout << "\n";
  check(framesValid, "endEffectorFrameIndices resolve to real frames (foot_l/r_contact)");

  // ---- CentroidalMpcRobotModel: state get/set round-trip ----
  CentroidalMpcRobotModel<scalar_t> robotModel(settings, pin, info);
  std::cout << "=== CentroidalMpcRobotModel ===\n";
  check(robotModel.getStateDim() == 35, "robotModel.getStateDim() == 35");
  check(robotModel.getInputDim() == 35, "robotModel.getInputDim() == 35");

  vector_t state = vector_t::Zero(robotModel.getStateDim());
  vector6_t basePose;
  basePose << 0.1, -0.2, 0.79, 0.15, 0.05, -0.10;  // [x,y,z, eulerZYX]
  robotModel.setBasePose(state, basePose);
  const vector_t jointAngles = vector_t::Constant(settings.mpc_joint_dim, 0.07);
  robotModel.setJointAngles(state, jointAngles);
  check((robotModel.getBasePose(state) - basePose).norm() < 1e-12, "base pose set/get round-trips");
  check((robotModel.getJointAngles(state) - jointAngles).norm() < 1e-12, "joint angles set/get round-trip");

  // ---- A_b^-1 generalized-velocity solve (getGeneralizedVelocities) ----
  vector_t input = vector_t::Zero(robotModel.getInputDim());
  // Non-zero normalized centroidal momentum (state.head(6)) + joint velocities so the
  // base velocity comes out non-trivial through the A_b^-1 mapping.
  state.head(6) << 0.1, 0.0, -0.05, 0.0, 0.02, 0.0;
  input.tail(settings.mpc_joint_dim) = vector_t::Constant(settings.mpc_joint_dim, 0.03);
  const vector_t v = robotModel.getGeneralizedVelocities(state, input);
  std::cout << "=== getGeneralizedVelocities (A_b^-1 solve) ===\n";
  std::cout << "  v.size()=" << v.size() << "  v.head(6)=[" << v.head(6).transpose() << "]\n";
  check(v.size() == 29, "generalized velocity size == 29");
  check(v.allFinite(), "generalized velocity is finite");
  check(v.tail(settings.mpc_joint_dim).isApprox(input.tail(settings.mpc_joint_dim)),
        "joint-velocity block passes through unchanged");

  // ---- #30: PinocchioCentroidalDynamics flow map + analytic linearization ----
  // getValue needs updateCentroidalDynamics(q); getLinearApproximation needs
  // updateCentroidalDynamicsDerivatives(q, v) cached first.
  PinocchioCentroidalDynamics dynamics(info);
  dynamics.setPinocchioInterface(pin);
  const vector_t q = state.tail(info.generalizedCoordinatesNum);

  updateCentroidalDynamicsDerivatives(pin, info, q, v);
  const auto lin = dynamics.getLinearApproximation(0.0, state, input);

  updateCentroidalDynamics(pin, info, q);
  const vector_t xdot = dynamics.getValue(0.0, state, input);

  std::cout << "=== CentroidalDynamics (flow map + linearization) ===\n";
  std::cout << "  xdot.size()=" << xdot.size() << "\n";
  check(xdot.size() == 35, "flow map size == 35");
  check(xdot.allFinite(), "flow map finite");
  check(xdot.tail(settings.mpc_joint_dim).isApprox(input.tail(settings.mpc_joint_dim)),
        "qdot_joints == input joint velocities (passthrough)");
  check(lin.dfdx.rows() == 35 && lin.dfdx.cols() == 35, "A = dfdx is 35x35");
  check(lin.dfdu.rows() == 35 && lin.dfdu.cols() == 35, "B = dfdu is 35x35");

  // Central finite-difference of the flow map vs the analytic Jacobians.
  const scalar_t eps = 1e-6;
  matrix_t Afd(35, 35), Bfd(35, 35);
  for (int i = 0; i < 35; ++i) {
    vector_t sp = state, sm = state;
    sp[i] += eps;
    sm[i] -= eps;
    const vector_t qp = sp.tail(info.generalizedCoordinatesNum);
    const vector_t qm = sm.tail(info.generalizedCoordinatesNum);
    updateCentroidalDynamics(pin, info, qp);
    const vector_t fp = dynamics.getValue(0.0, sp, input);
    updateCentroidalDynamics(pin, info, qm);
    const vector_t fm = dynamics.getValue(0.0, sm, input);
    Afd.col(i) = (fp - fm) / (2.0 * eps);
  }
  updateCentroidalDynamics(pin, info, q);  // input perturbation leaves q (cache) unchanged
  for (int i = 0; i < 35; ++i) {
    vector_t up = input, um = input;
    up[i] += eps;
    um[i] -= eps;
    const vector_t fp = dynamics.getValue(0.0, state, up);
    const vector_t fm = dynamics.getValue(0.0, state, um);
    Bfd.col(i) = (fp - fm) / (2.0 * eps);
  }
  const double errA = (Afd - lin.dfdx).cwiseAbs().maxCoeff();
  const double errB = (Bfd - lin.dfdu).cwiseAbs().maxCoeff();
  std::cout << std::scientific << "  max|A_analytic - A_fd| = " << errA << "   max|B_analytic - B_fd| = " << errB << std::fixed << "\n";
  check(errA < 1e-4, "analytic dfdx matches finite-difference");
  check(errB < 1e-4, "analytic dfdu matches finite-difference");

  std::cout << (failures == 0 ? "\nPASS — centroidal model layer validated\n" : "\nFAIL\n");
  return failures == 0 ? 0 : 1;
}
