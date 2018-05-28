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
ChassisControllerIntegrated::ChassisControllerIntegrated(Motor ileftSideMotor,
                                                         Motor irightSideMotor,
                                                         const pros::c::motor_gearset_e_t igearset,
                                                         const ChassisScales &iscales)
  : ChassisControllerIntegrated(std::make_shared<Motor>(ileftSideMotor),
                                std::make_shared<Motor>(irightSideMotor), igearset, iscales) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(MotorGroup ileftSideMotor,
                                                         MotorGroup irightSideMotor,
                                                         const pros::c::motor_gearset_e_t igearset,
                                                         const ChassisScales &iscales)
  : ChassisControllerIntegrated(std::make_shared<MotorGroup>(ileftSideMotor),
                                std::make_shared<MotorGroup>(irightSideMotor), igearset, iscales) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(Motor itopLeftMotor, Motor itopRightMotor,
                                                         Motor ibottomRightMotor,
                                                         Motor ibottomLeftMotor,
                                                         const pros::c::motor_gearset_e_t igearset,
                                                         const ChassisScales &iscales)
  : ChassisControllerIntegrated(std::make_shared<Motor>(itopLeftMotor),
                                std::make_shared<Motor>(itopRightMotor),
                                std::make_shared<Motor>(ibottomRightMotor),
                                std::make_shared<Motor>(ibottomLeftMotor), igearset, iscales) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  const pros::c::motor_gearset_e_t igearset, const ChassisScales &iscales)
  : ChassisController(std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor)),
    leftController(ileftSideMotor),
    rightController(irightSideMotor),
    lastTarget(0),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(pros::c::E_MOTOR_ENCODER_DEGREES);
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<AbstractMotor> itopLeftMotor, std::shared_ptr<AbstractMotor> itopRightMotor,
  std::shared_ptr<AbstractMotor> ibottomRightMotor, std::shared_ptr<AbstractMotor> ibottomLeftMotor,
  const pros::c::motor_gearset_e_t igearset, const ChassisScales &iscales)
  : ChassisController(std::make_shared<XDriveModel>(itopLeftMotor, itopRightMotor,
                                                    ibottomRightMotor, ibottomLeftMotor)),
    leftController(itopLeftMotor),
    rightController(itopRightMotor),
    lastTarget(0),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(pros::c::E_MOTOR_ENCODER_DEGREES);
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<ChassisModel> imodel, const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
  const AsyncPosIntegratedControllerArgs &irightControllerArgs,
  const pros::c::motor_gearset_e_t igearset, const ChassisScales &iscales)
  : ChassisController(imodel),
    leftController(ileftControllerArgs),
    rightController(irightControllerArgs),
    lastTarget(0),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(pros::c::E_MOTOR_ENCODER_DEGREES);
}

void ChassisControllerIntegrated::moveDistance(const QLength itarget) {
  const auto enc = model->getSensorVals();

  // Exit if there was an error
  // TODO: Waiting on a more specific error code
  if ((enc[0] == PROS_ERR_F || enc[1] == PROS_ERR_F) && errno == EINVAL) {
    return;
  }

  leftController.reset();
  rightController.reset();
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = itarget.convert(meter) * straightScale;
  leftController.setTarget(newTarget + enc[0]);
  rightController.setTarget(newTarget + enc[1]);

  std::uint32_t prevWakeTime = pros::millis();

  while (!leftController.isSettled() && !rightController.isSettled()) {
    pros::Task::delay_until(&prevWakeTime, 10);
  }

  leftController.flipDisable(true);
  rightController.flipDisable(true);
}

void ChassisControllerIntegrated::moveDistance(const double itarget) {
  moveDistance(itarget * meter);
}

void ChassisControllerIntegrated::turnAngle(const QAngle idegTarget) {
  auto enc = model->getSensorVals();
  int tryAgainCounter = 0;

  while (true) {
    if (enc[0] == PROS_ERR_F || enc[1] == PROS_ERR_F) {
      // If there was an error, check errno
      switch (errno) {
      case ENODEV: // Unplugged
      case EINVAL: // Invalid port number
        return;
      case EACCES: // Resource currently being used
      default:
        // Wait for access and try again
        pros::Task::delay(10);
        tryAgainCounter++;
        enc = model->getSensorVals();
        break;
      }

      if (tryAgainCounter > 4) {
        return;
      }
    } else {
      // No error, so we are good to go
      break;
    }
  }

  leftController.reset();
  rightController.reset();
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = idegTarget.convert(degree) * turnScale;
  leftController.setTarget(newTarget + enc[0]);
  rightController.setTarget(-1 * newTarget + enc[1]);

  std::uint32_t prevWakeTime = pros::millis();

  while (!leftController.isSettled() && !rightController.isSettled()) {
    pros::Task::delay_until(&prevWakeTime, 10);
  }

  leftController.flipDisable(true);
  rightController.flipDisable(true);
}

void ChassisControllerIntegrated::turnAngle(const double idegTarget) {
  turnAngle(idegTarget * degree);
}
} // namespace okapi
