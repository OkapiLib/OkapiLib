/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/iterative/iterativeController.hpp"

namespace okapi {
IterativeControllerArgs::~IterativeControllerArgs() = default;

IterativeController::~IterativeController() = default;

void IterativeController::setSampleTime(const std::uint32_t isampleTime) {
}

void IterativeController::setOutputLimits(double imax, double imin) {
}

std::uint32_t IterativeController::getSampleTime() const {
  return 15;
}
} // namespace okapi
