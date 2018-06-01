/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/util/rate.hpp"
#include "okapi/units/QTime.hpp"

namespace okapi {
Rate::Rate() = default;

Rate::~Rate() = default;

void Rate::delayHz(const QFrequency ihz) {
  if (lastTime == 0) {
    // First call
    lastTime = pros::millis();
    pros::Task::delay(1000 / ihz.convert(Hz));
  } else {
    // Subsequent call
    pros::Task::delay_until(&lastTime, 1000 / ihz.convert(Hz));
  }
}
} // namespace okapi
