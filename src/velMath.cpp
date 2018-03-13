/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/velMath.hpp"
#include "api.h"

namespace okapi {
  double VelMath::step(const double inewPos) {
    const long now = pros::millis();

    vel = static_cast<double>((1000 / (now - lastTime))) * (inewPos - lastPos) * (60 / ticksPerRev);
    vel = filter.filter(vel);

    lastPos = inewPos;
    lastTime = now;

    return vel;
  }
}
