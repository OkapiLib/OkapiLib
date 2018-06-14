/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/timer.hpp"
#include "api.h"

namespace okapi {
Timer::Timer() : firstCalled(millis()), lastCalled(firstCalled), mark(firstCalled) {
}

QTime Timer::millis() const {
  return pros::millis() * millisecond;
}

QTime Timer::getDt() {
  const QTime currTime = millis();
  const QTime dt = currTime - lastCalled;
  lastCalled = currTime;
  return dt;
}

QTime Timer::getStartingTime() const {
  return firstCalled;
}

QTime Timer::getDtFromStart() const {
  return millis() - firstCalled;
}

void Timer::placeMark() {
  mark = millis();
}

void Timer::placeHardMark() {
  if (hardMark == 0_ms)
    hardMark = millis();
}

QTime Timer::clearHardMark() {
  const QTime old = hardMark;
  hardMark = 0_ms;
  return old;
}

QTime Timer::getDtFromMark() const {
  return millis() - mark;
}

QTime Timer::getDtFromHardMark() const {
  return hardMark == 0_ms ? 0_ms : millis() - hardMark;
}

bool Timer::repeat(const QTime time) {
  if (repeatMark == 0_ms) {
    repeatMark = millis();
    return false;
  }

  if (millis() - repeatMark >= time) {
    repeatMark = 0_ms;
    return true;
  }

  return false;
}

bool Timer::repeat(const QFrequency frequency) {
  return repeat(QTime(1 / frequency.convert(Hz)));
}
} // namespace okapi
