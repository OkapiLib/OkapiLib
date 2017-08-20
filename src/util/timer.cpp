#include "util/timer.h"

namespace okapi {
  unsigned long Timer::getDt() {
    const unsigned long currTime = millis();
    const unsigned long dt = currTime - lastCalled;
    lastCalled = currTime;
    return dt;
  }

  unsigned long Timer::getStartingTime() const {
    return firstCalled;
  }

  unsigned long Timer::getDtFromStart() const {
    return millis() - firstCalled;
  }

  void Timer::placeMark() {
    mark = millis();
  }

  void Timer::placeHardMark() {
    if (hardMark == -1)
      hardMark = millis();
  }

  unsigned long Timer::clearHardMark() {
    const long old = hardMark;
    hardMark = -1;
    return old;
  }

  unsigned long Timer::getDtFromMark() const {
    return millis() - mark;
  }

  unsigned long Timer::getDtFromHardMark() const {
    return hardMark == -1 ? 0 : millis() - hardMark;
  }

  bool Timer::repeat(unsigned long ms) {
    if (repeatMark == -1)
      repeatMark = millis();

    if (millis() - repeatMark >= ms) {
      repeatMark = -1;
      return true;
    }

    return false;
  }
}
