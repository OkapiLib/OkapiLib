/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerIntegrated.hpp"
#include "api.h"
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/util/rate.hpp"

namespace okapi {
ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<ChassisModel> imodel, const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
  const AsyncPosIntegratedControllerArgs &irightControllerArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales)
  : ChassisController(imodel),
    leftController(ileftControllerArgs, SettledUtilFactory::createPtr(), std::make_unique<Rate>()),
    rightController(irightControllerArgs, SettledUtilFactory::createPtr(),
                    std::make_unique<Rate>()),
    lastTarget(0),
    gearRatio(igearset.ratio),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  if (igearset.ratio == 0) {
    throw std::invalid_argument("ChassisControllerIntegrated: The gear ratio cannot be zero! Check "
                                "if you are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

void ChassisControllerIntegrated::moveDistance(const QLength itarget) {
  leftController.reset();
  rightController.reset();
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = itarget.convert(meter) * straightScale * gearRatio;
  const auto enc = model->getSensorVals();
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
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / straightScale) * meter);
}

void ChassisControllerIntegrated::turnAngle(const QAngle idegTarget) {
  leftController.reset();
  rightController.reset();
  leftController.flipDisable(false);
  rightController.flipDisable(false);

  const double newTarget = idegTarget.convert(degree) * turnScale * gearRatio;
  const auto enc = model->getSensorVals();
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
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}
} // namespace okapi
