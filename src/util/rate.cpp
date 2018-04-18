/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/util/rate.hpp"

namespace okapi {
Rate::Rate() = default;

Rate::~Rate() = default;

void Rate::delayHz(const std::uint32_t ihz) {
  if (lastTime == 0) {
    // First call
    lastTime = pros::millis();
    pros::c::task_delay(
      static_cast<std::uint32_t>(static_cast<double>(1000) / static_cast<double>(ihz)));
  } else {
    // Subsequent call
    pros::c::task_delay_until(
      &lastTime, static_cast<std::uint32_t>(static_cast<double>(1000) / static_cast<double>(ihz)));
  }
}
} // namespace okapi
