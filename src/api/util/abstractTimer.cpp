/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/abstractTimer.hpp"

namespace okapi {
AbstractTimer::AbstractTimer(const QTime ifirstCalled)
  : firstCalled(ifirstCalled), lastCalled(firstCalled), mark(firstCalled) {
}

AbstractTimer::~AbstractTimer() = default;

QTime AbstractTimer::getDt() {
  const QTime currTime = millis();
  const QTime dt = currTime - lastCalled;
  lastCalled = currTime;
  return dt;
}

QTime AbstractTimer::readDt() const {
  return millis() - lastCalled;
}

QTime AbstractTimer::getStartingTime() const {
  return firstCalled;
}

QTime AbstractTimer::getDtFromStart() const {
  return millis() - firstCalled;
}

void AbstractTimer::placeMark() {
  mark = millis();
}

QTime AbstractTimer::clearMark() {
  const QTime old = mark;
  mark = 0_ms;
  return old;
}

void AbstractTimer::placeHardMark() {
  if (hardMark == 0_ms)
    hardMark = millis();
}

QTime AbstractTimer::clearHardMark() {
  const QTime old = hardMark;
  hardMark = 0_ms;
  return old;
}

QTime AbstractTimer::getDtFromMark() const {
  return mark == 0_ms ? 0_ms : millis() - mark;
}

QTime AbstractTimer::getDtFromHardMark() const {
  return hardMark == 0_ms ? 0_ms : millis() - hardMark;
}

bool AbstractTimer::repeat(const QTime time) {
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

bool AbstractTimer::repeat(const QFrequency frequency) {
  return repeat(QTime(1 / frequency.convert(Hz)));
}
} // namespace okapi
