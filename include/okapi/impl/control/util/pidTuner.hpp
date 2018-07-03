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
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/impl/control/util/settledUtil.hpp"
#include "okapi/impl/control/util/pidTuner.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/impl/control/iterative/iterativePosPidController.hpp"
#include <memory>
#include <vector>

namespace okapi {
class PIDTuner {
  public:
  PIDTuner(std::shared_ptr<ControllerOutput> ioutput, std::shared_ptr<SettledUtil> isettle,
           const QTime itimeout, const std::int32_t igoal, const double ikPMin,
           const double ikPMax, const double ikIMin, const double ikIMax,
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

  std::shared_ptr<ControllerOutput>output;
  std::shared_ptr<SettledUtil>settle;
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

  std::vector<particleSet> particles{};
  IterativePosPIDController testController{0, 0, 0};
};
} // namespace okapi

#endif
