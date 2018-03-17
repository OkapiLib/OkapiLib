/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/async/posIntegratedController.hpp"

namespace okapi {
PosIntegratedControllerParams::PosIntegratedControllerParams(const AbstractMotor &imotor)
  : motor(imotor) {
}

std::shared_ptr<AsyncPositionController> PosIntegratedControllerParams::make() const {
  return std::make_shared<PosIntegratedController>(PosIntegratedController(*this));
}

PosIntegratedController::PosIntegratedController(const AbstractMotor &imotor)
  : motor(imotor), lastTarget(0), offset(0) {
}

PosIntegratedController::PosIntegratedController(const PosIntegratedControllerParams &iparams)
  : motor(iparams.motor), lastTarget(0), offset(0) {
}

PosIntegratedController::~PosIntegratedController() = default;

/**
 * Sets the target for the controller.
 */
void PosIntegratedController::setTarget(const double itarget) {
  motor.move_absolute(itarget + offset, 100);
  lastTarget = itarget;
}

/**
 * Returns the last error of the controller.
 */
double PosIntegratedController::getError() const {
  return lastTarget - motor.get_position();
}

/**
 * Resets the controller so it can start from 0 again properly. Keeps configuration from
 * before.
 */
void PosIntegratedController::reset() {
  // Save the current target as an offset for the new targets so the current target becomes a
  // "zero position"
  offset = lastTarget;
}
} // namespace okapi
