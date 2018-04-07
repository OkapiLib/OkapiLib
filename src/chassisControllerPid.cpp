/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/chassisControllerPid.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(const ChassisModel &imodel,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(imodel),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerPID::ChassisControllerPID(const AbstractMotor &ileftSideMotor,
                                           const AbstractMotor &irightSideMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(SkidSteerModel(ileftSideMotor, irightSideMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerPID::ChassisControllerPID(const AbstractMotor &itopLeftMotor,
                                           const AbstractMotor &itopRightMotor,
                                           const AbstractMotor &ibottomRightMotor,
                                           const AbstractMotor &ibottomLeftMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(
      XDriveModel(itopLeftMotor, itopRightMotor, ibottomRightMotor, ibottomLeftMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

void ChassisControllerPID::moveDistance(const int itarget) {
  const auto encStartVals = model.getSensorVals();
  float distanceElapsed = 0, angleChange = 0, lastDistance = 0;
  uint32_t prevWakeTime = millis();

  const double newTarget = itarget * straightScale;

  distancePid.reset();
  anglePid.reset();
  distancePid.setTarget(static_cast<float>(newTarget));
  anglePid.setTarget(0);

  bool atTarget = false;
  const int atTargetDistance = 15;
  const int threshold = 2;

  Timer atTargetTimer;

  const int timeoutPeriod = 250;

  std::valarray<int> encVals{0, 0};
  float distOutput, angleOutput;

  while (!atTarget) {
    encVals = model.getSensorVals() - encStartVals;
    distanceElapsed = static_cast<float>((encVals[0] + encVals[1])) / 2.0;
    angleChange = static_cast<float>(encVals[1] - encVals[0]);

    distOutput = distancePid.step(distanceElapsed);
    angleOutput = anglePid.step(angleChange);
    model.driveVector(static_cast<int>(distOutput * 127), static_cast<int>(angleOutput * 127));

    if (abs(newTarget - static_cast<int>(distanceElapsed)) <= atTargetDistance)
      atTargetTimer.placeHardMark();
    else if (abs(static_cast<int>(distanceElapsed) - static_cast<int>(lastDistance)) <= threshold)
      atTargetTimer.placeHardMark();
    else
      atTargetTimer.clearHardMark();

    lastDistance = distanceElapsed;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
      atTarget = true;

    task_delay_until(&prevWakeTime, 10);
  }

  model.stop();
}

void ChassisControllerPID::turnAngle(float idegTarget) {
  const auto encStartVals = model.getSensorVals();
  float angleChange = 0, lastAngle = 0;
  uint32_t prevWakeTime = millis();

  const float newTarget = idegTarget * turnScale;

  anglePid.reset();
  anglePid.setTarget(static_cast<float>(newTarget));

  bool atTarget = false;
  const int atTargetAngle = 10;
  const int threshold = 2;

  Timer atTargetTimer;

  const int timeoutPeriod = 250;

  std::valarray<int> encVals{0, 0};

  while (!atTarget) {
    encVals = model.getSensorVals() - encStartVals;
    angleChange = static_cast<float>(encVals[1] - encVals[0]);

    model.rotate(static_cast<int>(anglePid.step(angleChange) * 127));

    if (fabs(newTarget - angleChange) <= atTargetAngle)
      atTargetTimer.placeHardMark();
    else if (fabs(angleChange - lastAngle) <= threshold)
      atTargetTimer.placeHardMark();
    else
      atTargetTimer.clearHardMark();

    lastAngle = angleChange;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
      atTarget = true;

    task_delay_until(&prevWakeTime, 10);
  }

  model.stop();
}
} // namespace okapi
