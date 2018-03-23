/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_PIDTUNER_HPP_
#define _OKAPI_PIDTUNER_HPP_

#include "api.h"
#include "okapi/chassis/model/chassisModel.hpp"
#include "okapi/control/iterative/posPidController.hpp"
#include <vector>

namespace okapi {
class PIDTuner {
  public:
  PIDTuner(const ChassisModelParams &imodelParams, const uint32_t itimeout, const int32_t igoal,
           const double ikPMin, const double ikPMax, const double ikIMin, const double ikIMax,
           const double ikDMin, const double ikDMax, const size_t inumIterations = 5,
           const size_t inumParticles = 16, const double ikSettle = 1, const double ikITAE = 2);

  virtual ~PIDTuner();

  PosPIDControllerParams autotune();

  private:
  static constexpr double inertia = 0.5;   // Particle intertia
  static constexpr double confSelf = 1.1;  // Self confidence
  static constexpr double confSwarm = 1.2; // Particle swarm confidence
  static constexpr int increment = 5;
  static constexpr int divisor = 5;

  struct particle {
    double pos, vel, best;
  };

  struct particleSet {
    particle kP, kI, kD;
    double bestError;
  };

  std::shared_ptr<const ChassisModel> model;
  const uint32_t timeout;
  const int32_t goal;
  const double kPMin;
  const double kPMax;
  const double kIMin;
  const double kIMax;
  const double kDMin;
  const double kDMax;
  const size_t numIterations;
  const size_t numParticles;
  const double kSettle;
  const double kITAE;

  double itae;
  std::vector<particleSet> particles;
  PosPIDController leftController;
  PosPIDController rightController;

  uint32_t moveDistance(const int itarget);
};
} // namespace okapi

#endif
