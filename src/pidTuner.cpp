/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/pidTuner.hpp"
#include "okapi/util/timer.hpp"
#include <algorithm>
#include <limits>
#include <random>

namespace okapi {
PIDTuner::PIDTuner(const ChassisModel &imodel, const uint32_t itimeout, const int32_t igoal,
                   const double ikPMin, const double ikPMax, const double ikIMin,
                   const double ikIMax, const double ikDMin, const double ikDMax,
                   const size_t inumIterations, const size_t inumParticles, const double ikSettle,
                   const double ikITAE)
  : model(imodel),
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
    itae(0),
    leftController(0, 0, 0),
    rightController(0, 0, 0) {
}

PIDTuner::~PIDTuner() = default;

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

      // Reverse the goal every iteration to stay in the same general area
      int target = 660;
      if (!firstGoal) {
        target *= -1;
      }
      firstGoal = !firstGoal;

      auto settleTime = moveDistance(target); // 16 in on 3" wheels (really 3.125")

      double error = kSettle * settleTime + kITAE * itae;
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

  return PosPIDControllerParams(global.kP.best, global.kI.best, global.kD.best);
}

uint32_t PIDTuner::moveDistance(const int itarget) {
  const auto encStartVals = model.getSensorVals();
  float distanceElapsed = 0, lastDistance = 0;
  uint32_t prevWakeTime = millis();

  leftController.reset();
  rightController.reset();
  leftController.setTarget(static_cast<float>(itarget));
  rightController.setTarget(static_cast<float>(itarget));

  bool atTarget = false;
  const int atTargetDistance = 15;
  const int threshold = 2;

  Timer atTargetTimer;
  const int timeoutPeriod = 250;

  std::valarray<int> encVals{0, 0};

  while (!atTarget) {
    encVals = model.getSensorVals() - encStartVals;
    distanceElapsed = static_cast<float>((encVals[0] + encVals[1])) / 2.0;
    itae += ((atTargetTimer.getDtFromStart() * abs(itarget - distanceElapsed)) /
             (divisor * abs(itarget)));

    model.left(leftController.step(distanceElapsed));
    model.right(rightController.step(distanceElapsed));

    if (abs(itarget - static_cast<int>(distanceElapsed)) <= atTargetDistance) {
      atTargetTimer.placeHardMark();
    } else if (abs(static_cast<int>(distanceElapsed) - static_cast<int>(lastDistance)) <=
               threshold) {
      atTargetTimer.placeHardMark();
    } else {
      atTargetTimer.clearHardMark();
    }

    lastDistance = distanceElapsed;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod) {
      atTarget = true;
    }

    task_delay_until(&prevWakeTime, 15);
  }

  auto settleTime = atTargetTimer.getDtFromStart();

  model.stop();
  task_delay(1000); // Let the robot settle
  return settleTime;
}
} // namespace okapi
