/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/control/util/settledUtil.hpp"
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
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = itarget * straightScale;
  leftController.setTarget(newTarget);
  rightController.setTarget(newTarget);

  std::uint32_t prevWakeTime = pros::millis();

  while (!leftController.isSettled() && !rightController.isSettled()) {
    pros::c::task_delay_until(&prevWakeTime, 10);
  }

  leftController.flipDisable(true);
  rightController.flipDisable(true);
}

void ChassisControllerIntegrated::turnAngle(float idegTarget) {
  leftController.reset();
  rightController.reset();
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = idegTarget * turnScale;
  leftController.setTarget(newTarget);
  rightController.setTarget(-1 * newTarget);

  SettledUtil leftControllerSettled;
  SettledUtil rightControllerSettled;
  std::uint32_t prevWakeTime = pros::millis();

  while (!leftControllerSettled.isSettled(leftController.getError()) &&
         !rightControllerSettled.isSettled(rightController.getError())) {
    pros::c::task_delay_until(&prevWakeTime, 10);
  }

  leftController.flipDisable(true);
  rightController.flipDisable(true);
}
} // namespace okapi
