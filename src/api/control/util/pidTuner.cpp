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
PIDTuner::PIDTuner(const std::shared_ptr<ControllerInput<double>> &iinput,
                   const std::shared_ptr<ControllerOutput<double>> &ioutput,
                   const TimeUtil &itimeUtil,
                   QTime itimeout,
                   std::int32_t igoal,
                   double ikPMin,
                   double ikPMax,
                   double ikIMin,
                   double ikIMax,
                   double ikDMin,
                   double ikDMax,
                   std::size_t inumIterations,
                   std::size_t inumParticles,
                   double ikSettle,
                   double ikITAE)
  : logger(Logger::instance()),
    input(iinput),
    output(ioutput),
    timeUtil(itimeUtil),
    rate(itimeUtil.getRate()),
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
  input = iinput;
}

PIDTuner::~PIDTuner() = default;

PIDTuner::Output PIDTuner::autotune() {
  std::random_device rd;  // Random seed
  std::mt19937 gen(rd()); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, 1);

  IterativePosPIDController testController(0, 0, 0, 0, timeUtil);
  std::vector<ParticleSet> particles;
  for (std::size_t i = 0; i < numParticles; i++) {
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
    particles.push_back(set);
  }

  ParticleSet global{};
  global.kP.best = 0;
  global.kI.best = 0;
  global.kD.best = 0;
  global.bestError = std::numeric_limits<double>::max();

  // Run the optimization
  for (std::size_t iteration = 0; iteration < numIterations; iteration++) {
    logger->info("PIDTuner: Iteration number " + std::to_string(iteration));
    bool firstGoal = true;

    for (std::size_t particleIndex = 0; particleIndex < numParticles; particleIndex++) {
      logger->info("PIDTuner: Particle number " + std::to_string(particleIndex));

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
      const double start_val = input->controllerGet();

      QTime settleTime = 0_ms;
      double itae = 0;
      // Test constants then calculate fitness function
      while (!testController.isSettled()) {
        settleTime += loopDelta;
        if (settleTime > timeout)
          break;

        const double inputVal = input->controllerGet() - start_val;
        const double outputVal = testController.step(inputVal);
        const double error = testController.getError();
        // sum of the error emphasizing later error
        itae += (settleTime.convert(millisecond) * abs((int)error)) / divisor;

        output->controllerSet(outputVal);
        rate->delayUntil(loopDelta);
      }

      output->controllerSet(0);
      testController.reset();

      const double error = kSettle * settleTime.convert(millisecond) + kITAE * itae;

      logger->info("PIDTuner: New error is " + std::to_string(error));

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
    for (std::size_t i = 0; i < numParticles; i++) {
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

  return Output{global.kP.best, global.kI.best, global.kD.best};
}
} // namespace okapi
