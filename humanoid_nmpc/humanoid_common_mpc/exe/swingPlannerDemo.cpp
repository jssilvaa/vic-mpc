// D2 behavior check (throwaway): exercise the swing-foot planner with a HAND-BUILT
// walking gait (the default gait is all-STANCE => no swings, so we can't use it).
//
// Mode semantics (MotionPhaseDefinition, contact flags {LF=leg0, RF=leg1}):
//   STANCE(3) both in contact; RF(1) -> LEFT foot (leg 0) swings; LF(2) -> RIGHT
//   foot (leg 1) swings. We schedule one left swing then one right swing and sample
//   the per-foot Z constraint, expecting each to arc up to ~swingHeight and back.

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>

#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <task.info>\n";
    return 1;
  }
  using namespace ocs2;
  using namespace ocs2::humanoid;

  const auto config = loadSwingTrajectorySettings(argv[1], "swing_trajectory_config", /*verbose=*/true);
  SwingTrajectoryPlanner planner(config, /*numFeet=*/2);

  // Hand-built walking schedule: STANCE | RF (left swings) | STANCE | LF (right swings) | STANCE
  const std::vector<scalar_t> eventTimes{0.3, 0.7, 1.0, 1.4};
  const std::vector<size_t> modeSequence{STANCE, RF, STANCE, LF, STANCE};
  ModeSchedule modeSchedule(eventTimes, modeSequence);

  planner.update(modeSchedule, /*terrainHeight=*/0.0);

  std::cout << "\n=== Swing Z over the schedule (swingHeight cfg = " << config.swingHeight << ") ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "   t      mode    z_left    z_right\n";
  for (scalar_t t = 0.0; t <= 1.8001; t += 0.1) {
    const size_t mode = modeSchedule.modeAtTime(t);
    std::cout << "  " << std::setw(4) << t << "   " << std::setw(6) << modeNumber2String(mode)
              << "  " << std::setw(8) << planner.getZpositionConstraint(0, t)
              << "  " << std::setw(8) << planner.getZpositionConstraint(1, t) << "\n";
  }

  // Report the swing apex of each foot.
  scalar_t leftApex = 0.0, rightApex = 0.0;
  for (scalar_t t = 0.0; t <= 1.8001; t += 0.01) {
    leftApex = std::max(leftApex, planner.getZpositionConstraint(0, t));
    rightApex = std::max(rightApex, planner.getZpositionConstraint(1, t));
  }
  std::cout << "\nleft  foot swing apex = " << leftApex << "  (expect ~" << config.swingHeight << ")\n";
  std::cout << "right foot swing apex = " << rightApex << "  (expect ~" << config.swingHeight << ")\n";
  return 0;
}
