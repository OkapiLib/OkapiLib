/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/pidTuner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace okapi {
PIDTuner::PIDTuner(std::shared_ptr<ControllerOutput> ioutput, std::unique_ptr<AbstractTimer> itimer,
                   std::unique_ptr<SettledUtil> isettle, std::unique_ptr<AbstractRate> irate,
                   const QTime itimeout, const std::int32_t igoal, const double ikPMin,
                   const double ikPMax, const double ikIMin, const double ikIMax,
                   const double ikDMin, const double ikDMax, const std::size_t inumIterations,
                   const std::size_t inumParticles, const double ikSettle, const double ikITAE)
  : output(ioutput),
    settle(std::move(isettle)),
    rate(std::move(irate)),
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
    kITAE(ikITAE),
    testController(0, 0, 0, 0, std::move(itimer), std::move(isettle)) {
}

PIDTuner::~PIDTuner() = default;

IterativePosPIDControllerArgs PIDTuner::autotune() {
  std::random_device rd;  // Random seed
  std::mt19937 gen(rd()); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, 1);

  for (size_t i = 0; i < numParticles; i++) {
    ParticleSet set{};
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

  ParticleSet global{};
  global.kP.best = 0;
  global.kI.best = 0;
  global.kD.best = 0;
  global.bestError = std::numeric_limits<double>::max();

  // Run the optimization
  for (size_t iteration = 0; iteration < numIterations; iteration++) {
    // Test constants then calculate fitness function
    bool firstGoal = true;

    for (size_t particleIndex = 0; particleIndex < numParticles; particleIndex++) {
      // TODO: Index out of bounds here (particles is empty at this point)
      testController.setGains(particles.at(particleIndex).kP.pos,
                              particles.at(particleIndex).kI.pos,
                              particles.at(particleIndex).kD.pos);

      // Reverse the goal every iteration to stay in the same general area
      std::int32_t target = goal;
      if (!firstGoal) {
        target *= -1;
      }
      firstGoal = !firstGoal;

      testController.setTarget(target);

      QTime settleTime = 0_ms;
      int itae = 0;
      while (!testController.isSettled()) {
        settleTime += loopDelta;
        if (settleTime > timeout)
          break;

        const double error = testController.getError();
        // sum of the error emphasizing later error
        itae += (settleTime.convert(millisecond) * abs((int)error)) / divisor;

        output->controllerSet(testController.step(error));
        rate->delayUntil(loopDelta);
      }

      double error = kSettle * settleTime.convert(millisecond) + kITAE * itae;
      if (error < particles.at(particleIndex).bestError) {
        particles.at(particleIndex).kP.best = particles.at(particleIndex).kP.pos;
        particles.at(particleIndex).kI.best = particles.at(particleIndex).kI.pos;
        particles.at(particleIndex).kD.best = particles.at(particleIndex).kD.pos;
        particles.at(particleIndex).bestError = error;

        if (error < global.bestError) {
          global.kP.best = particles.at(particleIndex).kP.pos;
          global.kI.best = particles.at(particleIndex).kI.pos;
          global.kD.best = particles.at(particleIndex).kD.pos;
          global.bestError = error;
        }
      }
    }

    // Update particle trajectories
    for (size_t i = 0; i < numParticles; i++) {
      // Factor in the particles inertia to keep on the same trajectory
      particles.at(i).kP.vel *= inertia;
      // Move towards particle's best
      particles.at(i).kP.vel +=
        confSelf * ((particles.at(i).kP.best - particles.at(i).kP.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles.at(i).kP.vel +=
        confSwarm * ((global.kP.best - particles.at(i).kP.pos) / increment) * dist(gen);
      // Kinematics
      particles.at(i).kP.pos += particles.at(i).kP.vel * increment;

      // Factor in the particles inertia to keep on the same trajectory
      particles.at(i).kI.vel *= inertia;
      // Move towards particle's best
      particles.at(i).kI.vel +=
        confSelf * ((particles.at(i).kI.best - particles.at(i).kI.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles.at(i).kI.vel +=
        confSwarm * ((global.kI.best - particles.at(i).kI.pos) / increment) * dist(gen);
      // Kinematics
      particles.at(i).kI.pos += particles.at(i).kI.vel * increment;

      // Factor in the particles inertia to keep on the same trajectory
      particles.at(i).kD.vel *= inertia;
      // Move towards particle's best
      particles.at(i).kD.vel +=
        confSelf * ((particles.at(i).kD.best - particles.at(i).kD.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles.at(i).kD.vel +=
        confSwarm * ((global.kD.best - particles.at(i).kD.pos) / increment) * dist(gen);
      // Kinematics
      particles.at(i).kD.pos += particles.at(i).kD.vel * increment;

      particles.at(i).kP.pos = std::clamp(particles.at(i).kP.pos, kPMin, kPMax);
      particles.at(i).kI.pos = std::clamp(particles.at(i).kI.pos, kIMin, kIMax);
      particles.at(i).kD.pos = std::clamp(particles.at(i).kD.pos, kDMin, kDMax);
    }
  }

  return IterativePosPIDControllerArgs(global.kP.best, global.kI.best, global.kD.best);
}
} // namespace okapi
