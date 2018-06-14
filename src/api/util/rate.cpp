/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/rate.hpp"
#include "api.h"

namespace okapi {
Rate::Rate() = default;

void Rate::delay(const QFrequency ihz) {
  delay(1000 / ihz.convert(Hz));
}

void Rate::delay(const int ims) {
  if (lastTime == 0) {
    // First call
    lastTime = pros::millis();
    pros::Task::delay(ims);
  } else {
    // Subsequent call
    pros::Task::delay_until(&lastTime, ims);
  }
}
} // namespace okapi
