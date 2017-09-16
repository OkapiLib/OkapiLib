#include "control/nsPid.h"

namespace okapi {
  float NsPid::loop(const float inewReading) {
    Pid::loop(inewReading); //Main control loop

    //Check if velocity is sufficiently small
    if (velMath.loop(inewReading) < minVel) {
      return 0;
    }

    return Pid::output;
  }
}
