/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/chassisControllerPid.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::~ChassisControllerPID() = default;

void ChassisControllerPID::driveStraight(const int itarget) {
  const auto encStartVals = model->getSensorVals();
  double distanceElapsed = 0, angleChange = 0, lastDistance = 0;
  uint32_t prevWakeTime = pros::millis();

  distancePid.reset();
  anglePid.reset();
  distancePid.setTarget(static_cast<double>(itarget));
  anglePid.setTarget(0);

  bool atTarget = false;
  const int atTargetDistance = 15;
  const int threshold = 2;

  Timer atTargetTimer;

  const int timeoutPeriod = 250;

  std::valarray<int> encVals{0, 0};
  double distOutput, angleOutput;

  while (!atTarget) {
    encVals = model->getSensorVals() - encStartVals;
    distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);

    distOutput = distancePid.step(distanceElapsed);
    angleOutput = anglePid.step(angleChange);
    model->driveVector(static_cast<int>(distOutput * 127), static_cast<int>(angleOutput * 127));

    if (abs(itarget - static_cast<int>(distanceElapsed)) <= atTargetDistance)
      atTargetTimer.placeHardMark();
    else if (abs(static_cast<int>(distanceElapsed) - static_cast<int>(lastDistance)) <= threshold)
      atTargetTimer.placeHardMark();
    else
      atTargetTimer.clearHardMark();

    lastDistance = distanceElapsed;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
      atTarget = true;

    task_delay_until(&prevWakeTime, 15);
  }

  model->stop();
}

void ChassisControllerPID::pointTurn(double idegTarget) {
  const auto encStartVals = model->getSensorVals();
  double angleChange = 0, lastAngle = 0;
  uint32_t prevWakeTime = pros::millis();

  while (idegTarget > 180)
    idegTarget -= 360;
  while (idegTarget <= -180)
    idegTarget += 360;

  anglePid.reset();
  anglePid.setTarget(static_cast<double>(idegTarget));

  bool atTarget = false;
  const int atTargetAngle = 10;
  const int threshold = 2;

  Timer atTargetTimer;

  const int timeoutPeriod = 250;

  std::valarray<int> encVals{0, 0};

  while (!atTarget) {
    encVals = model->getSensorVals() - encStartVals;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);

    model->turnClockwise(static_cast<int>(anglePid.step(angleChange) * 127));

    if (fabs(idegTarget - angleChange) <= atTargetAngle)
      atTargetTimer.placeHardMark();
    else if (fabs(angleChange - lastAngle) <= threshold)
      atTargetTimer.placeHardMark();
    else
      atTargetTimer.clearHardMark();

    lastAngle = angleChange;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
      atTarget = true;

    task_delay_until(&prevWakeTime, 15);
  }

  model->stop();
}
} // namespace okapi
