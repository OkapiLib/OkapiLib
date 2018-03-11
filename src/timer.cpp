#include "okapi/util/timer.h"

namespace okapi {
  unsigned long Timer::getDt() {
    const unsigned long currTime = pros::millis();
    const unsigned long dt = currTime - lastCalled;
    lastCalled = currTime;
    return dt;
  }

  unsigned long Timer::getStartingTime() const {
    return firstCalled;
  }

  unsigned long Timer::getDtFromStart() const {
    return pros::millis() - firstCalled;
  }

  void Timer::placeMark() {
    mark = pros::millis();
  }

  void Timer::placeHardMark() {
    if (hardMark == -1)
      hardMark = pros::millis();
  }

  unsigned long Timer::clearHardMark() {
    const long old = hardMark;
    hardMark = -1;
    return old;
  }

  unsigned long Timer::getDtFromMark() const {
    return pros::millis() - mark;
  }

  unsigned long Timer::getDtFromHardMark() const {
    return hardMark == -1 ? 0 : pros::millis() - hardMark;
  }

  bool Timer::repeat(unsigned long ms) {
    if (repeatMark == -1)
      repeatMark = pros::millis();

    if (pros::millis() - repeatMark >= ms) {
      repeatMark = -1;
      return true;
    }

    return false;
  }
}
