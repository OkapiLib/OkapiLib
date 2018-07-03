/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/pidTuner.hpp"
#include "okapi/api/util/timer.hpp"
#include <algorithm>
#include <limits>
#include <random>
#include <cmath>

const QTime loopDelta = 10_ms;
#define DIVISOR 5

namespace okapi {
PIDTuner::PIDTuner(std::shared_ptr<ControllerOutput> ioutput, std::shared_ptr<SettledUtil> isettle,
           const QTime itimeout, const std::int32_t igoal, const double ikPMin,
           const double ikPMax, const double ikIMin, const double ikIMax,
           const double ikDMin, const double ikDMax, const std::size_t inumIterations,
           const std::size_t inumParticles, const double ikSettle,
           const double ikITAE )
  : output(ioutput),
    settle(isettle),
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

PIDTuner::~PIDTuner() = default;

IterativePosPIDControllerArgs PIDTuner::autotune() {
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
      testController.setGains(particles[i].kP.pos, particles[i].kI.pos, particles[i].kD.pos);

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

				int error = testController.getError();
				itae += (settleTime.convert(millisecond) * abs(error)) / DIVISOR; // sum of the error emphasizing later error

				output->controllerSet(testController.step(error));
				pros::c::delay(loopDelta.convert(millisecond));
			}

      double error = kSettle * settleTime.convert(millisecond) + kITAE * itae;
      if (error < particles[i].bestError) {
        particles[i].kP.best = particles[i].kP.pos;
        particles[i].kI.best = particles[i].kI.pos;
        particles[i].kD.best = particles[i].kD.pos;
        particles[i].bestError = error;

        if (error < global.bestError) {
          global.kP.best = particles[i].kP.pos;
          global.kI.best = particles[i].kI.pos;
          global.kD.best = particles[i].kD.pos;
          global.bestError = error;
        }
      }
    }

    // Update particle trajectories
    for (size_t i = 0; i < numParticles; i++) {
      // Factor in the particles inertia to keep on the same trajectory
      particles[i].kP.vel *= inertia;
      // Move towards particle's best
      particles[i].kP.vel +=
        confSelf * ((particles[i].kP.best - particles[i].kP.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles[i].kP.vel +=
        confSwarm * ((global.kP.best - particles[i].kP.pos) / increment) * dist(gen);
      // Kinematics
      particles[i].kP.pos += particles[i].kP.vel * increment;

      // Factor in the particles inertia to keep on the same trajectory
      particles[i].kI.vel *= inertia;
      // Move towards particle's best
      particles[i].kI.vel +=
        confSelf * ((particles[i].kI.best - particles[i].kI.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles[i].kI.vel +=
        confSwarm * ((global.kI.best - particles[i].kI.pos) / increment) * dist(gen);
      // Kinematics
      particles[i].kI.pos += particles[i].kI.vel * increment;

      // Factor in the particles inertia to keep on the same trajectory
      particles[i].kD.vel *= inertia;
      // Move towards particle's best
      particles[i].kD.vel +=
        confSelf * ((particles[i].kD.best - particles[i].kD.pos) / increment) * dist(gen);
      // Move towards swarm's best
      particles[i].kD.vel +=
        confSwarm * ((global.kD.best - particles[i].kD.pos) / increment) * dist(gen);
      // Kinematics
      particles[i].kD.pos += particles[i].kD.vel * increment;

      particles[i].kP.pos = std::clamp(particles[i].kP.pos, kPMin, kPMax);
      particles[i].kI.pos = std::clamp(particles[i].kI.pos, kIMin, kIMax);
      particles[i].kD.pos = std::clamp(particles[i].kD.pos, kDMin, kDMax);
    }
  }

  return IterativePosPIDControllerArgs(global.kP.best, global.kI.best, global.kD.best);
}

// std::uint32_t PIDTuner::moveDistance(const int itarget) {
//   const auto encStartVals = model->getSensorVals();
//   double distanceElapsed = 0, lastDistance = 0;
//   std::uint32_t prevWakeTime = pros::millis();
//
//   leftController.reset();
//   rightController.reset();
//   leftController.setTarget(static_cast<double>(itarget));
//   rightController.setTarget(static_cast<double>(itarget));
//
//   bool atTarget = false;
//   const int atTargetDistance = 15;
//   const int threshold = 2;
//
//   Timer atTargetTimer;
//   const QTime timeoutPeriod = 250_ms;
//
//   std::valarray<std::int32_t> encVals;
//
//   while (!atTarget) {
//     encVals = model->getSensorVals() - encStartVals;
//     distanceElapsed = static_cast<double>(encVals[0] + encVals[1]) / 2.0;
//     itae +=
//       ((atTargetTimer.getDtFromStart().convert(millisecond) * std::abs(itarget - distanceElapsed)) /
//        (divisor * std::abs(itarget)));
//
//     model->left(leftController.step(distanceElapsed));
//     model->right(rightController.step(distanceElapsed));
//
//     if (std::abs(itarget - static_cast<int>(distanceElapsed)) <= atTargetDistance) {
//       atTargetTimer.placeHardMark();
//     } else if (std::abs(static_cast<int>(distanceElapsed) - static_cast<int>(lastDistance)) <=
//                threshold) {
//       atTargetTimer.placeHardMark();
//     } else {
//       atTargetTimer.clearHardMark();
//     }
//
//     lastDistance = distanceElapsed;
//
//     if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod) {
//       atTarget = true;
//     }
//
//     pros::Task::delay_until(&prevWakeTime, 15);
//   }
//
//   auto settleTime = atTargetTimer.getDtFromStart();
//
//   model->stop();
//   pros::Task::delay(1000); // Let the robot settle
//   return settleTime.convert(millisecond);
// }
} // namespace okapi
