#include "control/nsPid.h"
#include <cmath>

namespace okapi {
  float NsPid::loop(const float inewReading) {
    using namespace std;

    Pid::loop(inewReading); //Main control loop

    //Check if velocity is sufficiently small
    if (abs(velMath.loop(inewReading)) < minVel) {
      return scale * Pid::output;
    }

    return Pid::output;
  }
}
