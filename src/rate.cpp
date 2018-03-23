/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/util/rate.hpp"

namespace okapi {
Rate::Rate() : lastTime(0) {
}

Rate::~Rate() = default;

void Rate::delayHz(const uint32_t ihz) {
  if (lastTime == 0) {
    // First call
    lastTime = millis();
    task_delay(static_cast<float>(1000) / ihz);
  } else {
    // Subsequent call
    task_delay_until(&lastTime, static_cast<float>(1000) / ihz);
  }
}
} // namespace okapi
