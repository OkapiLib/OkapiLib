/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/pidTuner.hpp"
#include <limits>
#include <random>

namespace okapi {
PIDTuner::PIDTuner(const ChassisModelParams &imodelParams, const uint32_t itimeout,
                   const int32_t igoal, const double ikPMin, const double ikPMax,
                   const double ikIMin, const double ikIMax, const double ikDMin,
                   const double ikDMax, const size_t inumIterations, const size_t inumParticles,
                   const double ikSettle, const double ikITAE)
  : model(imodelParams.make()),
    timeout(itimeout),
    goal(igoal),
    kPMin(ikPMin),
    kPMax(ikPMax),
    kIMin(ikIMin),
    kIMax(ikIMax),
    kDMin(ikDMin),
    kDMax(ikDMax),
    numIterations(inumIterations),
    numParticles(inumParticles),
    kSettle(ikSettle),
    kITAE(ikITAE) {
}

PosPIDControllerParams PIDTuner::autotune() {
  std::random_device rd;  // Random seed
  std::mt19937 gen(rd()); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, 1);

  for (size_t i = 0; i < numParticles; i++) {
    particleSet set;
    set.kP.pos = kPMin + (kPMax - kPMin) * dist(gen);
    set.kP.vel = set.kP.pos / increment;
    set.kP.best = set.kP.pos;

    set.kI.pos = kIMin + (kIMax - kIMin) * dist(gen);
    set.kI.vel = set.kI.pos / increment;
    set.kI.best = set.kI.pos;

    set.kD.pos = kDMin + (kDMax - kDMin) * dist(gen);
    set.kD.vel = set.kD.pos / increment;
    set.kD.best = set.kD.pos;

    set.bestError = std::numeric_limits<double>::max();
  }

  particleSet global;
  global.kP.best = 0;
  global.kI.best = 0;
  global.kD.best = 0;
  global.bestError = std::numeric_limits<double>::max();

  // Run the optimization
  for (size_t i = 0; i < numIterations; i++) {
    // Test constants then calculate fitness function
    bool firstGoal = true;
    for (size_t i = 0; i < numParticles; i++) {
      leftController.setGains(particles[i].kP.pos, particles[i].kI.pos, particles[i].kD.pos);
      rightController.setGains(particles[i].kP.pos, particles[i].kI.pos, particles[i].kD.pos);
    }
  }
}

PIDTuner::~PIDTuner() = default;
} // namespace okapi
