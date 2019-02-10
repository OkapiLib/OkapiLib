/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/medianFilter.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <utility>

namespace okapi {
VelMath::VelMath(const double iticksPerRev,
                 std::unique_ptr<Filter> ifilter,
                 QTime isampleTime,
                 std::unique_ptr<AbstractTimer> iloopDtTimer,
                 const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    ticksPerRev(iticksPerRev),
    sampleTime(isampleTime),
    loopDtTimer(std::move(iloopDtTimer)),
    filter(std::move(ifilter)) {
  if (iticksPerRev == 0) {
    LOG_ERROR_S("VelMath: The ticks per revolution cannot be zero! Check if you are using integer "
                "division.");
    throw std::invalid_argument(
      "VelMath: The ticks per revolution cannot be zero! Check if you are using integer division.");
  }
}

VelMath::~VelMath() = default;

QAngularSpeed VelMath::step(const double inewPos) {
  if (loopDtTimer->readDt() >= sampleTime) {
    const QTime dt = loopDtTimer->getDt();

    vel = filter->filter(((inewPos - lastPos) * (60 / ticksPerRev)) / dt.convert(second)) * rpm;
    accel = (vel - lastVel) / dt;

    lastVel = vel;
    lastPos = inewPos;
  }

  return vel;
}

void VelMath::setTicksPerRev(const double iTPR) {
  ticksPerRev = iTPR;
}

QAngularSpeed VelMath::getVelocity() const {
  return vel;
}

QAngularAcceleration VelMath::getAccel() const {
  return accel;
}
} // namespace okapi
