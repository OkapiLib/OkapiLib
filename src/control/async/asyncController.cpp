/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncController.hpp"

namespace okapi {
AsyncControllerArgs::~AsyncControllerArgs() = default;

double AsyncController::getOutput() const {
  return 0;
}

void AsyncController::setSampleTime(const std::uint32_t) {
}

void AsyncController::setOutputLimits(double, double) {
}
} // namespace okapi
