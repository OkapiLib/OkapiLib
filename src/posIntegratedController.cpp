/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/posIntegratedController.hpp"

namespace okapi {
PosIntegratedControllerParams::PosIntegratedControllerParams(const AbstractMotor &imotor)
  : motor(imotor) {
}

PosIntegratedControllerParams::~PosIntegratedControllerParams() = default;

PosIntegratedController::PosIntegratedController(const AbstractMotor &imotor) : motor(imotor) {
}

PosIntegratedController::PosIntegratedController(const PosIntegratedControllerParams &iparams)
  : motor(iparams.motor) {
}

PosIntegratedController::~PosIntegratedController() = default;

void PosIntegratedController::setTarget(const double itarget) {
  motor.move_absolute(itarget + offset, 100);
  lastTarget = itarget;
}

double PosIntegratedController::getError() const {
  return lastTarget - motor.get_position();
}

void PosIntegratedController::reset() {
  // Save the current target as an offset for the new targets so the current target becomes a
  // "zero position"
  offset = lastTarget;
}
} // namespace okapi
