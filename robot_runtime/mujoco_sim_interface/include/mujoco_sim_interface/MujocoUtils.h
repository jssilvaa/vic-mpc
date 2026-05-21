#pragma once

#include <mujoco/mujoco.h>

namespace robot::mujoco_sim_interface {

struct Metrics {
  /// FPS of Simulation::step
  double fpsSim;
  /// Real time factor for current sim step: RTF = dt_sim / dt_real
  double rtfTick;
  /// Time drift per-tick.
  double driftTick;
  /// Total time drift since starting the sim.
  double driftCumulative;

  void reset() {
    fpsSim = 0.0;
    rtfTick = 0.0;
    driftTick = 0.0;
    driftCumulative = 0.0;
  }
};

struct MjState {
  explicit MjState(const mjModel* mujocoModel_);

  int64_t timestamp{0};
  mjData* data;
  Metrics metrics;
};

}  // namespace robot::mujoco_sim_interface
