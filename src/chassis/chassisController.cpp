#include "chassis/chassisController.h"
#include "util/timer.h"
#include "PAL/PAL.h"
#include <cmath>

namespace okapi {
  void ChassisControllerPid::driveStraight(const int itarget) {
    const auto encStartVals = model->getSensorVals();
    float distanceElapsed = 0, angleChange = 0, lastDistance = 0;
    unsigned long prevWakeTime = PAL::millis();

    distancePid.reset();
    anglePid.reset();
    distancePid.setTarget(static_cast<float>(itarget));
    anglePid.setTarget(0);

    bool atTarget = false;
    const int atTargetDistance = 15;
    const int threshold = 2;

    Timer atTargetTimer;

    const int timeoutPeriod = 250;

    std::valarray<int> encVals{0, 0};
    float distOutput, angleOutput;

    while (!atTarget) {
      encVals = model->getSensorVals() - encStartVals;
      distanceElapsed = static_cast<float>((encVals[0] + encVals[1])) / 2.0;
      angleChange = static_cast<float>(encVals[1] - encVals[0]);

      distOutput = distancePid.step(distanceElapsed);
      angleOutput = anglePid.step(angleChange);
      model->driveVector(static_cast<int>(distOutput), static_cast<int>(angleOutput));

      if (std::abs(itarget - distanceElapsed) <= atTargetDistance)
        atTargetTimer.placeHardMark();
      else if (std::abs(distanceElapsed - lastDistance) <= threshold)
        atTargetTimer.placeHardMark();
      else
        atTargetTimer.clearHardMark();

      lastDistance = distanceElapsed;

      if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
        atTarget = true;

      PAL::taskDelayUntil(&prevWakeTime, 15);
    }

    model->driveForward(0);
  }

  void ChassisControllerPid::pointTurn(float idegTarget) {
    const auto encStartVals = model->getSensorVals();
    float angleChange = 0, lastAngle = 0;
    unsigned long prevWakeTime = PAL::millis();

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

    std::valarray<int> encVals{0, 0};

    while (!atTarget) {
      encVals = model->getSensorVals() - encStartVals;
      angleChange = static_cast<float>(encVals[1] - encVals[0]);

      model->turnClockwise(static_cast<int>(anglePid.step(angleChange)));

      if (std::abs(idegTarget - angleChange) <= atTargetAngle)
        atTargetTimer.placeHardMark();
      else if (std::abs(angleChange - lastAngle) <= threshold)
        atTargetTimer.placeHardMark();
      else
        atTargetTimer.clearHardMark();

      lastAngle = angleChange;

      if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
        atTarget = true;

      PAL::taskDelayUntil(&prevWakeTime, 15);
    }

    model->driveForward(0);
  }

  void ChassisControllerMP::driveStraight(const int itarget) {
    controller.setTarget(static_cast<float>(itarget));

    const auto encStartVals = model->getSensorVals();
    float distanceElapsed = 0;
    unsigned long prevWakeTime = PAL::millis();

    std::valarray<int> encVals{0, 0};

    while (!controller.isComplete()) {
      encVals = model->getSensorVals() - encStartVals;
      distanceElapsed = static_cast<float>((encVals[0] + encVals[1])) / 2.0;

      model->driveForward(static_cast<int>(controller.step(distanceElapsed)));
      
      PAL::taskDelayUntil(&prevWakeTime, 15);
    }

    model->driveForward(0);
  }

  void ChassisControllerMP::pointTurn(float idegTarget) {
    controller.setTarget(static_cast<float>(idegTarget)); //TODO: Might not be able to use the same params for turning

    const auto encStartVals = model->getSensorVals();
    float angleChange = 0;
    unsigned long prevWakeTime = PAL::millis();

    std::valarray<int> encVals{0, 0};

    while (!controller.isComplete()) {
      encVals = model->getSensorVals() - encStartVals;
      angleChange = static_cast<float>(encVals[1] - encVals[0]);

      model->turnClockwise(static_cast<int>(controller.step(angleChange)));

      PAL::taskDelayUntil(&prevWakeTime, 15);
    }

    model->driveForward(0);
  }
}
