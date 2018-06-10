/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/filter/velMath.hpp"
#include "api.h"
#include "okapi/filter/averageFilter.hpp"
#include "okapi/filter/medianFilter.hpp"
#include "okapi/util/mathUtil.hpp"
#include <utility>

namespace okapi {
VelMathArgs::VelMathArgs(const double iticksPerRev)
  : VelMathArgs(iticksPerRev,
                std::make_shared<ComposableFilter>(std::initializer_list<std::shared_ptr<Filter>>(
                  {std::make_shared<MedianFilter<3>>(), std::make_shared<AverageFilter<5>>()}))) {
}

VelMathArgs::VelMathArgs(const double iticksPerRev, std::shared_ptr<Filter> ifilter)
  : ticksPerRev(iticksPerRev == 0 ? imev5TPR : iticksPerRev), filter(ifilter) {
}

VelMathArgs::~VelMathArgs() = default;

VelMath::VelMath(const double iticksPerRev)
  : VelMath(iticksPerRev,
            std::make_shared<ComposableFilter>(std::initializer_list<std::shared_ptr<Filter>>(
              {std::make_shared<MedianFilter<3>>(), std::make_shared<AverageFilter<5>>()})),
            std::make_unique<Timer>()) {
}

VelMath::VelMath(const double iticksPerRev, std::shared_ptr<Filter> ifilter)
  : VelMath(iticksPerRev, ifilter, std::make_unique<Timer>()) {
}

VelMath::VelMath(const VelMathArgs &iparams)
  : VelMath(iparams.ticksPerRev, iparams.filter, std::make_unique<Timer>()) {
}

VelMath::VelMath(const double iticksPerRev, std::shared_ptr<Filter> ifilter,
                 std::unique_ptr<Timer> iloopDtTimer)
  : ticksPerRev(iticksPerRev == 0 ? imev5TPR : iticksPerRev),
    loopDtTimer(std::move(iloopDtTimer)),
    filter(ifilter) {
}

VelMath::~VelMath() = default;

QAngularSpeed VelMath::step(const double inewPos) {
  const QTime dt = loopDtTimer->getDt();

  vel = filter->filter(((inewPos - lastPos) * (60 / ticksPerRev)) / dt.convert(second)) * rpm;
  accel = (vel - lastVel) / dt;

  lastVel = vel;
  lastPos = inewPos;

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
