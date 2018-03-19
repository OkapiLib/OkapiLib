/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/util/timer.hpp"

namespace okapi {
Timer::Timer() : firstCalled(millis()), lastCalled(0), mark(0), hardMark(0), repeatMark(0) {
}

Timer::~Timer() = default;

uint32_t Timer::getDt() {
  const uint32_t currTime = millis();
  const uint32_t dt = currTime - lastCalled;
  lastCalled = currTime;
  return dt;
}

uint32_t Timer::getStartingTime() const {
  return firstCalled;
}

uint32_t Timer::getDtFromStart() const {
  return millis() - firstCalled;
}

void Timer::placeMark() {
  mark = millis();
}

void Timer::placeHardMark() {
  if (hardMark == 0)
    hardMark = millis();
}

uint32_t Timer::clearHardMark() {
  const long old = hardMark;
  hardMark = 0;
  return old;
}

uint32_t Timer::getDtFromMark() const {
  return millis() - mark;
}

uint32_t Timer::getDtFromHardMark() const {
  return hardMark == 0 ? 0 : millis() - hardMark;
}

bool Timer::repeat(const uint32_t ms) {
  if (repeatMark == 0)
    repeatMark = millis();

  if (millis() - repeatMark >= ms) {
    repeatMark = 0;
    return true;
  }

  return false;
}
} // namespace okapi
