/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/pidTuner.hpp"
#include "api.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace okapi {
PIDTuner::PIDTuner(std::shared_ptr<ControllerInput> iinput,
                   std::shared_ptr<ControllerOutput> ioutput, const TimeUtil &itimeUtil,
                   QTime itimeout, std::int32_t igoal, double ikPMin, double ikPMax, double ikIMin,
                   double ikIMax, double ikDMin, double ikDMax, std::int32_t inumIterations,
                   std::int32_t inumParticles, double ikSettle, double ikITAE)
  : input(iinput),
    output(ioutput),
    timeUtil(itimeUtil),
    rate(std::move(itimeUtil.getRate())),
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
  particles.resize(numParticles);
}

PIDTuner::~PIDTuner() = default;

IterativePosPIDControllerArgs PIDTuner::autotune() {
  std::random_device rd;  // Random seed
  std::mt19937 gen(rd()); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, 1);

  IterativePosPIDController testController(0, 0, 0, 0, timeUtil);

  for (int i = 0; i < numParticles; i++) {
    ParticleSet set{};
    set = particles.at(i);
    set.kP.pos = kPMin + (kPMax - kPMin) * dist(gen);
    pros::c::lcd_print(6, "kP %lf", set.kP.pos);
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
  for (int iteration = 0; iteration < numIterations; iteration++) {
    // Test constants then calculate fitness function
    bool firstGoal = true;

    for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
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
      int count = 0;
      while (!testController.isSettled()) {
        settleTime += loopDelta;
        if (settleTime > timeout)
          break;

        const double inputVal = input->controllerGet();
        const double outputVal = testController.step(inputVal);
        const double error = testController.getError();
        count++;
        pros::c::lcd_print(3, "out: %d in %lf err %lf c %d", outputVal, inputVal, error, count);
        // sum of the error emphasizing later error
        itae += (settleTime.convert(millisecond) * abs((int)error)) / divisor;

        output->controllerSet(outputVal);
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
    for (int i = 0; i < numParticles; i++) {
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
