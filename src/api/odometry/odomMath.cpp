/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
OdomMath::OdomMath() = default;

OdomMath::~OdomMath() = default;

QLength
OdomMath::computeDistanceToPoint(const QLength ix, const QLength iy, const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);
  return std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter;
}

QAngle OdomMath::computeAngleToPoint(const QLength ix, const QLength iy, const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);
  return (std::atan2(yDiff, xDiff) * radian) - istate.theta;
}

std::tuple<QLength, QAngle> OdomMath::computeDistanceAndAngleToPoint(const QLength ix,
                                                                     const QLength iy,
                                                                     const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);

  return std::make_tuple(std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter,
                         (std::atan2(yDiff, xDiff) * radian) - istate.theta);
}
} // namespace okapi
