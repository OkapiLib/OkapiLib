/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/chassisControllerPid.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
  ChassisControllerPid::~ChassisControllerPid() = default;

  void ChassisControllerPid::driveStraight(const int itarget) {
    using namespace std;

    const auto encStartVals = model.getSensorVals();
    float distanceElapsed = 0, angleChange = 0, lastDistance = 0;
    uint32_t prevWakeTime = pros::millis();

    distancePid.reset();
    anglePid.reset();
    distancePid.setTarget(static_cast<float>(itarget));
    anglePid.setTarget(0);

    bool atTarget = false;
    const int atTargetDistance = 15;
    const int threshold = 2;

    Timer atTargetTimer;

    const int timeoutPeriod = 250;

    valarray<int> encVals{0, 0};
    float distOutput, angleOutput;


    while (!atTarget) {
      encVals = model.getSensorVals() - encStartVals;
      distanceElapsed = static_cast<float>((encVals[0] + encVals[1])) / 2.0;
      angleChange = static_cast<float>(encVals[1] - encVals[0]);

      distOutput = distancePid.step(distanceElapsed);
      angleOutput = anglePid.step(angleChange);
      model.driveVector(static_cast<int>(distOutput), static_cast<int>(angleOutput));

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

    model.driveForward(0);
  }

  void ChassisControllerPid::pointTurn(float idegTarget) {
    using namespace std;
    
    const auto encStartVals = model.getSensorVals();
    float angleChange = 0, lastAngle = 0;
    uint32_t prevWakeTime = pros::millis();

    while (idegTarget > 180)
      idegTarget -= 360;
    while (idegTarget <= -180)
      idegTarget += 360;

    anglePid.reset();
    anglePid.setTarget(static_cast<float>(idegTarget));

    bool atTarget = false;
    const int atTargetAngle = 10;
    const int threshold = 2;

    Timer atTargetTimer;

    const int timeoutPeriod = 250;

    valarray<int> encVals{0, 0};

    while (!atTarget) {
      encVals = model.getSensorVals() - encStartVals;
      angleChange = static_cast<float>(encVals[1] - encVals[0]);

      model.turnClockwise(static_cast<int>(anglePid.step(angleChange)));

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

    model.driveForward(0);
  }
}
