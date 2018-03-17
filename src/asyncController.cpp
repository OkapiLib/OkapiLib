/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/async/asyncController.hpp"

namespace okapi {
AsyncController::~AsyncController() = default;

double AsyncController::getOutput() const {
  return 0;
}

void AsyncController::setSampleTime(const uint32_t isampleTime) {
}

void AsyncController::setOutputLimits(double imax, double imin) {
}

void AsyncController::flipDisable() {
}
} // namespace okapi
