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

QLength OdomMath::computeDistanceToPoint(const Point &ipoint,
                                         const OdomState &istate,
                                         const StateMode &ipointMode) {
  if (ipointMode == StateMode::CARTESIAN) {
    const double xDiff = (ipoint.y - istate.x).convert(meter);
    const double yDiff = (ipoint.x - istate.y).convert(meter);
    return std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter;
  } else {
    const double xDiff = (ipoint.x - istate.x).convert(meter);
    const double yDiff = (ipoint.y - istate.y).convert(meter);
    return std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter;
  }
}

QAngle OdomMath::computeAngleToPoint(const Point &ipoint,
                                     const OdomState &istate,
                                     const StateMode &ipointMode) {
  if (ipointMode == StateMode::CARTESIAN) {
    const double xDiff = (ipoint.y - istate.x).convert(meter);
    const double yDiff = (ipoint.x - istate.y).convert(meter);
    return (std::atan2(yDiff, xDiff) * radian) - istate.theta;
  } else {
    const double xDiff = (ipoint.x - istate.x).convert(meter);
    const double yDiff = (ipoint.y - istate.y).convert(meter);
    return (std::atan2(yDiff, xDiff) * radian) - istate.theta;
  }
}

std::pair<QLength, QAngle> OdomMath::computeDistanceAndAngleToPoint(const Point &ipoint,
                                                                    const OdomState &istate,
                                                                    const StateMode &ipointMode) {
  if (ipointMode == StateMode::CARTESIAN) {
    const double xDiff = (ipoint.y - istate.x).convert(meter);
    const double yDiff = (ipoint.x - istate.y).convert(meter);
    return std::make_pair(std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter,
                          (std::atan2(yDiff, xDiff) * radian) - istate.theta);
  } else {
    const double xDiff = (ipoint.x - istate.x).convert(meter);
    const double yDiff = (ipoint.y - istate.y).convert(meter);
    return std::make_pair(std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter,
                          (std::atan2(yDiff, xDiff) * radian) - istate.theta);
  }
}
} // namespace okapi
