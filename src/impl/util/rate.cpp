/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/util/rate.hpp"
#include "api.h"

namespace okapi {
Rate::Rate() = default;

void Rate::delay(const QFrequency ihz) {
  delayUntil(1000 / ihz.convert(Hz));
}

void Rate::delay(const int ihz) {
  delayUntil(1000 / ihz);
}

void Rate::delayUntil(const QTime itime) {
  delayUntil(itime.convert(millisecond));
}

void Rate::delayUntil(const uint32_t ims) {
  if (lastTime == 0) {
    // First call
    lastTime = pros::millis();
  }

  pros::Task::delay_until(&lastTime, ims);
}
} // namespace okapi
