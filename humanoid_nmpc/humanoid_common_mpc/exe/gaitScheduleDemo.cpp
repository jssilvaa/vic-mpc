// D2 behavior check (throwaway): exercise the config -> joint-partition -> gait
// pipeline end-to-end. Builds ModelSettings from task.info + URDF, loads a
// GaitSchedule from reference.info, and prints the resulting mode schedule over a
// window. Confirms ModelSettings parses to 23 active / 29 full joints and that
// GaitSchedule emits a sane tiled schedule.

#include <iostream>
#include <string>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/gait/GaitSchedule.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"

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

  // ---- ModelSettings: config + URDF -> joint partition ----
  ModelSettings settings(taskFile, urdfFile, "centroidal_", /*verbose=*/false);

  std::cout << "=== ModelSettings ===\n";
  std::cout << "robotName       : " << settings.robotName << "\n";
  std::cout << "full_joint_dim  : " << settings.full_joint_dim << "  (expect 29)\n";
  std::cout << "mpc_joint_dim   : " << settings.mpc_joint_dim << "  (expect 23)\n";
  std::cout << "fixed joints    : " << settings.fixedJointNames.size() << "  (expect 6)\n";
  std::cout << "active MPC joints:\n";
  for (const auto& name : settings.mpcModelJointNames) std::cout << "  " << name << "\n";
  std::cout << "arm-swing indices: l_sh=" << settings.j_l_shoulder_y_index
            << " r_sh=" << settings.j_r_shoulder_y_index << " l_el=" << settings.j_l_elbow_y_index
            << " r_el=" << settings.j_r_elbow_y_index << "\n";

  // ---- GaitSchedule: reference.info -> tiled mode schedule ----
  auto gaitSchedulePtr = GaitSchedule::loadGaitSchedule(referenceFile, settings, /*verbose=*/false);
  const ModeSchedule ms = gaitSchedulePtr->getModeSchedule(0.0, 2.0);

  std::cout << "\n=== ModeSchedule over [0, 2] s ===\n";
  std::cout << "eventTimes (" << ms.eventTimes.size() << "): ";
  for (scalar_t t : ms.eventTimes) std::cout << t << " ";
  std::cout << "\nmodeSequence (" << ms.modeSequence.size() << "): ";
  for (size_t m : ms.modeSequence) std::cout << modeNumber2String(m) << " ";
  std::cout << "\n";

  // Spot-check modeAtTime at a few instants.
  std::cout << "modeAtTime: ";
  for (scalar_t t : {0.0, 0.5, 1.0, 1.5}) std::cout << "t=" << t << "->" << modeNumber2String(ms.modeAtTime(t)) << "  ";
  std::cout << "\n";

  return 0;
}
