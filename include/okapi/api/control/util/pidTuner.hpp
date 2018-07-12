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
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include <memory>
#include <vector>

namespace okapi {
class PIDTuner {
  public:
  PIDTuner(std::shared_ptr<ControllerOutput> ioutput, std::unique_ptr<AbstractTimer> itimer,
           std::unique_ptr<SettledUtil> isettle, std::unique_ptr<AbstractRate> irate,
           QTime itimeout, std::int32_t igoal, double ikPMin, double ikPMax, double ikIMin,
           double ikIMax, double ikDMin, double ikDMax, std::size_t inumIterations = 5,
           std::size_t inumParticles = 16, double ikSettle = 1, double ikITAE = 2);

  virtual ~PIDTuner();

  virtual IterativePosPIDControllerArgs autotune();

  protected:
  static constexpr double inertia = 0.5;   // Particle inertia
  static constexpr double confSelf = 1.1;  // Self confidence
  static constexpr double confSwarm = 1.2; // Particle swarm confidence
  static constexpr int increment = 5;
  static constexpr int divisor = 5;
  static constexpr QTime loopDelta = 10_ms; // NOLINT

  struct Particle {
    double pos, vel, best;
  };

  struct ParticleSet {
    Particle kP, kI, kD;
    double bestError;
  };

  std::shared_ptr<ControllerOutput> output;
  std::unique_ptr<AbstractTimer> timer;
  std::unique_ptr<SettledUtil> settle;
  std::unique_ptr<AbstractRate> rate;

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

  std::vector<ParticleSet> particles{};
};
} // namespace okapi

#endif
