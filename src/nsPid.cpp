#include "okapi/control/nsPid.h"
#include <cmath>

namespace okapi {
  float NsPid::step(const float inewReading) {
    using namespace std;

    Pid::step(inewReading); //Main control loop

    //Check if velocity is sufficiently small
    if (fabs(velMath.step(inewReading)) < minVel) {
      return scale * Pid::output;
    }

    return Pid::output;
  }
}
