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
#include "okapi/control/iterative/iterativePosPidController.hpp"
#include "okapi/units/QTime.hpp"
#include <memory>
#include <vector>

namespace okapi {
class PIDTuner {
  public:
  PIDTuner(std::shared_ptr<ChassisModel> imodel, const QTime itimeout, const std::int32_t igoal,
           const double ikPMin, const double ikPMax, const double ikIMin, const double ikIMax,
           const double ikDMin, const double ikDMax, const std::size_t inumIterations = 5,
           const std::size_t inumParticles = 16, const double ikSettle = 1,
           const double ikITAE = 2);

  virtual ~PIDTuner();

  virtual IterativePosPIDControllerArgs autotune();

  protected:
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

  std::shared_ptr<ChassisModel> model;
  const QTime timeout;
  const std::int32_t goal;
  const double kPMin;
  const double kPMax;
  const double kIMin;
  const double kIMax;
  const double kDMin;
  const double kDMax;
  const std::size_t numIterations;
  const std::size_t numParticles;
  const double kSettle;
  const double kITAE;

  double itae = 0;
  std::vector<particleSet> particles{};
  IterativePosPIDController leftController{0, 0, 0};
  IterativePosPIDController rightController{0, 0, 0};

  std::uint32_t moveDistance(const int itarget);
};
} // namespace okapi

#endif
