/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/util/timer.hpp"

namespace okapi {
ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<ChassisModel> imodel, const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
  const AsyncPosIntegratedControllerArgs &irightControllerArgs, const double istraightScale,
  const double iturnScale)
  : ChassisController(imodel),
    leftController(ileftControllerArgs),
    rightController(irightControllerArgs),
    lastTarget(0),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerIntegrated::ChassisControllerIntegrated(Motor ileftSideMotor,
                                                         Motor irightSideMotor,
                                                         const double istraightScale,
                                                         const double iturnScale)
  : ChassisControllerIntegrated(std::make_shared<Motor>(ileftSideMotor),
                                std::make_shared<Motor>(irightSideMotor), istraightScale,
                                iturnScale) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(MotorGroup ileftSideMotor,
                                                         MotorGroup irightSideMotor,
                                                         const double istraightScale,
                                                         const double iturnScale)
  : ChassisControllerIntegrated(std::make_shared<MotorGroup>(ileftSideMotor),
                                std::make_shared<MotorGroup>(irightSideMotor), istraightScale,
                                iturnScale) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  const double istraightScale, const double iturnScale)
  : ChassisController(std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor)),
    leftController(ileftSideMotor),
    rightController(irightSideMotor),
    lastTarget(0),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<AbstractMotor> itopLeftMotor, std::shared_ptr<AbstractMotor> itopRightMotor,
  std::shared_ptr<AbstractMotor> ibottomRightMotor, std::shared_ptr<AbstractMotor> ibottomLeftMotor,
  const double istraightScale, const double iturnScale)
  : ChassisController(std::make_shared<XDriveModel>(itopLeftMotor, itopRightMotor,
                                                    ibottomRightMotor, ibottomLeftMotor)),
    leftController(itopLeftMotor),
    rightController(itopRightMotor),
    lastTarget(0),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

void ChassisControllerIntegrated::moveDistance(const int itarget) {
  leftController.reset();
  rightController.reset();

  int distanceElapsed = 0, lastDistance = 0;
  uint32_t prevWakeTime = pros::millis();
  bool atTarget = false;
  const int atTargetDistance = 15;
  const int threshold = 2;

  const auto encStartVals = model->getSensorVals();
  std::valarray<int> encVals{0, 0};

  Timer atTargetTimer;
  const uint32_t timeoutPeriod = 250;

  const double newTarget = (itarget + lastTarget) * straightScale;
  leftController.setTarget(newTarget);
  rightController.setTarget(newTarget);

  while (!atTarget) {
    encVals = model->getSensorVals() - encStartVals;
    distanceElapsed = static_cast<int>((encVals[0] + encVals[1]) / 2.0);

    if (abs(itarget - distanceElapsed) <= atTargetDistance &&
        abs(distanceElapsed - lastDistance) <= threshold)
      atTargetTimer.placeHardMark();
    else
      atTargetTimer.clearHardMark();

    lastDistance = distanceElapsed;

    if (atTargetTimer.getDtFromHardMark() >= timeoutPeriod)
      atTarget = true;

    pros::c::task_delay_until(&prevWakeTime, 10);
  }
}

void ChassisControllerIntegrated::turnAngle(float idegTarget) {
  lastTarget = 0;
  leftController.reset();
  rightController.reset();

  const double newTarget = idegTarget * turnScale;
  leftController.setTarget(newTarget);
  rightController.setTarget(-1 * newTarget);
}
} // namespace okapi
