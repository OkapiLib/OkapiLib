/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/util/timer.hpp"

namespace okapi {
Timer::Timer() : firstCalled(pros::millis()) {
}

Timer::~Timer() = default;

std::uint32_t Timer::getDt() {
  const std::uint32_t currTime = pros::millis();
  const std::uint32_t dt = currTime - lastCalled;
  lastCalled = currTime;
  return dt;
}

std::uint32_t Timer::getStartingTime() const {
  return firstCalled;
}

std::uint32_t Timer::getDtFromStart() const {
  return pros::millis() - firstCalled;
}

void Timer::placeMark() {
  mark = pros::millis();
}

void Timer::placeHardMark() {
  if (hardMark == 0)
    hardMark = pros::millis();
}

std::uint32_t Timer::clearHardMark() {
  const long old = hardMark;
  hardMark = 0;
  return old;
}

std::uint32_t Timer::getDtFromMark() const {
  return pros::millis() - mark;
}

std::uint32_t Timer::getDtFromHardMark() const {
  return hardMark == 0 ? 0 : pros::millis() - hardMark;
}

bool Timer::repeat(const std::uint32_t ms) {
  if (repeatMark == 0)
    repeatMark = pros::millis();

  if (pros::millis() - repeatMark >= ms) {
    repeatMark = 0;
    return true;
  }

  return false;
}
} // namespace okapi
