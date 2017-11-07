#include "util/timer.h"
#include "PAL/PAL.h"

namespace okapi {
  unsigned long Timer::getDt() {
    const unsigned long currTime = PAL::millis();
    const unsigned long dt = currTime - lastCalled;
    lastCalled = currTime;
    return dt;
  }

  unsigned long Timer::getStartingTime() const {
    return firstCalled;
  }

  unsigned long Timer::getDtFromStart() const {
    return PAL::millis() - firstCalled;
  }

  void Timer::placeMark() {
    mark = PAL::millis();
  }

  void Timer::placeHardMark() {
    if (hardMark == -1)
      hardMark = PAL::millis();
  }

  unsigned long Timer::clearHardMark() {
    const long old = hardMark;
    hardMark = -1;
    return old;
  }

  unsigned long Timer::getDtFromMark() const {
    return PAL::millis() - mark;
  }

  unsigned long Timer::getDtFromHardMark() const {
    return hardMark == -1 ? 0 : PAL::millis() - hardMark;
  }

  bool Timer::repeat(unsigned long ms) {
    if (repeatMark == -1)
      repeatMark = PAL::millis();

    if (PAL::millis() - repeatMark >= ms) {
      repeatMark = -1;
      return true;
    }

    return false;
  }
}
