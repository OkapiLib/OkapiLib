/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/util/timer.hpp"
#include "api.h"

namespace okapi {
Timer::Timer() : AbstractTimer(millis()) {
}

QTime Timer::millis() const {
  return pros::millis() * millisecond;
}
} // namespace okapi
