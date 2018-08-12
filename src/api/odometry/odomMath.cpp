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

QLength OdomMath::computeDistanceToPoint(const QLength ix, const QLength iy,
                                         const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);
  return std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter;
}

QAngle OdomMath::computeAngleToPoint(const QLength ix, const QLength iy, const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);
  return (std::atan2(yDiff, xDiff) * radian) - istate.theta;
}

DistanceAndAngle OdomMath::computeDistanceAndAngleToPoint(const QLength ix, const QLength iy,
                                                          const OdomState &istate) {
  const double xDiff = (ix - istate.x).convert(meter);
  const double yDiff = (iy - istate.y).convert(meter);

  DistanceAndAngle out;
  out.length = std::sqrt((xDiff * xDiff) + (yDiff * yDiff)) * meter;
  out.theta = (std::atan2(yDiff, xDiff) * radian) - istate.theta;

  return out;
}

std::tuple<double, double> OdomMath::guessScales(const double chassisDiam, const double wheelDiam,
                                                 const double ticksPerRev) {
  const double scale = ((wheelDiam * pi * inchToMM) / ticksPerRev) *
                       0.9945483364; // The scale is usually off by this amount
  const double turnScale = (1.0 / (chassisDiam * inchToMM)) * radianToDegree * 2;
  return std::make_tuple(scale, turnScale);
}
} // namespace okapi
