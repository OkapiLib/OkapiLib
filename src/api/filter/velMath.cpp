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
VelMathArgs::VelMathArgs(const double iticksPerRev, const QTime isampleTime)
  : VelMathArgs(iticksPerRev, std::make_shared<AverageFilter<2>>(), isampleTime) {
}

VelMathArgs::VelMathArgs(const double iticksPerRev,
                         const std::shared_ptr<Filter> &ifilter,
                         const QTime isampleTime)
  : ticksPerRev(iticksPerRev), filter(ifilter), sampleTime(isampleTime) {
}

VelMathArgs::~VelMathArgs() = default;

VelMath::VelMath(const VelMathArgs &iparams, std::unique_ptr<AbstractTimer> iloopDtTimer)
  : VelMath(iparams.ticksPerRev, iparams.filter, iparams.sampleTime, std::move(iloopDtTimer)) {
}

VelMath::VelMath(const double iticksPerRev,
                 const std::shared_ptr<Filter> &ifilter,
                 QTime isampleTime,
                 std::unique_ptr<AbstractTimer> iloopDtTimer)
  : logger(Logger::instance()),
    ticksPerRev(iticksPerRev),
    sampleTime(isampleTime),
    loopDtTimer(std::move(iloopDtTimer)),
    filter(ifilter) {
  if (iticksPerRev == 0) {
    logger->error(
      "VelMath: The ticks per revolution cannot be zero! Check if you are using integer division.");
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
