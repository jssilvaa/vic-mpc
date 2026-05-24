/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_common_mpc/swing_foot_planner/CubicSpline.h"

#include <cassert>

namespace ocs2::humanoid {

/// @brief Creates a cubic spline from t0_ to t1_, setting the position and velocity coefficients separately for numerical stability
/// @param[in] start Start Node with (t0, p0, v0) tuple data
/// @param[in] end End Node with (t1, p1, v1) tuple data
CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;

  dc0_ = 0.0;
  dc1_ = start.velocity;
  dc2_ = -(3.0 * start.velocity + dv);
  dc3_ = (2.0 * start.velocity + dv);

  c0_ = dc0_ * dt_ + start.position;
  c1_ = dc1_ * dt_;
  c2_ = dc2_ * dt_ + 3.0 * dp;
  c3_ = dc3_ * dt_ - 2.0 * dp;
}

/// @brief Returns the position at time t 
/// @param[in] time the time instant 
/// @returns the position at time t
scalar_t CubicSpline::position(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}


/// @brief Returns the velocity at time t 
/// @param[in] time the time instant 
/// @returns the velocity at time t
scalar_t CubicSpline::velocity(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

/// @brief Returns the acceleration at time t 
/// @param[in] time the time instant 
/// @returns the acceleration at time t
scalar_t CubicSpline::acceleration(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

/// @brief the derivative of the position function w.r.t. the start time of the spline 
/// @param[in] t the time instant
/// @return the value of the derivative of x(t)|[t0,t1] w.r.t. t0
scalar_t CubicSpline::startTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dCoff = -(dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t1_ - t) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

/// @brief the derivative of the position function w.r.t. the end time of the spline 
/// @param[in] t the time instant
/// @return the value of the derivative of x(t)|[t0,t1] w.r.t. t1
scalar_t CubicSpline::finalTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dCoff = (dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t - t0_) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

/// @brief the normalized time at time t between [t0, t1]
/// @param[in] t the time instant
/// @return the normalized time
scalar_t CubicSpline::normalizedTime(scalar_t t) const {
  return (t - t0_) / dt_;
}

}  // namespace ocs2::humanoid
