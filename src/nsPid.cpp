/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/nsPid.hpp"
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
