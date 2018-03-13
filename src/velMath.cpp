/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/velMath.hpp"
#include "api.h"

namespace okapi {
VelMath::VelMath(const double iticksPerRev, const double ialpha, const double ibeta)
  : lastTime(0), vel(0), lastVel(0), lastPos(0), ticksPerRev(iticksPerRev), filter(ialpha, ibeta) {
}

VelMath::VelMath(const VelMathParams &iparams)
  : lastTime(0),
    vel(0),
    lastVel(0),
    lastPos(0),
    ticksPerRev(iparams.ticksPerRev),
    filter(iparams.alpha, iparams.beta) {
}

double VelMath::step(const double inewPos) {
  const uint32_t now = pros::millis();

  vel = static_cast<double>((1000 / (now - lastTime))) * (inewPos - lastPos) * (60 / ticksPerRev);
  vel = filter.filter(vel);

  lastPos = inewPos;
  lastTime = now;

  return vel;
}

void VelMath::setGains(const double ialpha, const double ibeta) {
  filter.setGains(ialpha, ibeta);
}

void VelMath::setTicksPerRev(const double iTPR) {
  ticksPerRev = iTPR;
}

double VelMath::getOutput() const {
  return vel;
}

double VelMath::getDiff() const {
  return vel - lastVel;
}
}
