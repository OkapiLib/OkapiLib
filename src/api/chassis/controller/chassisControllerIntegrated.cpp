/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"

namespace okapi {
ChassisControllerIntegrated::ChassisControllerIntegrated(
  const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier,
  const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier,
  std::unique_ptr<ChassisModel> imodel, const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
  const AsyncPosIntegratedControllerArgs &irightControllerArgs,
  AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales)
  : ChassisController(std::move(imodel)),
    rate(std::move(irateSupplier.get())),
    leftController(ileftControllerArgs, isettledUtilSupplier.get(), irateSupplier.get()),
    rightController(irightControllerArgs, isettledUtilSupplier.get(), irateSupplier.get()),
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

  while (!leftController.isSettled() && !rightController.isSettled()) {
    rate->delayUntil(10_ms);
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

  while (!leftController.isSettled() && !rightController.isSettled()) {
    rate->delayUntil(10_ms);
  }

  leftController.flipDisable(true);
  rightController.flipDisable(true);
}

void ChassisControllerIntegrated::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}
} // namespace okapi
