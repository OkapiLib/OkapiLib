/*
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

QLength OdomMath::computeDistanceToPoint(const Point &ipoint, const OdomState &istate) {
  const auto [xDiff, yDiff] = computeDiffs(ipoint, istate);
  return computeDistance(xDiff, yDiff) * meter;
}

QAngle OdomMath::computeAngleToPoint(const Point &ipoint, const OdomState &istate) {
  const auto [xDiff, yDiff] = computeDiffs(ipoint, istate);
  return computeAngle(xDiff, yDiff, istate.theta.convert(radian)) * radian;
}

std::pair<QLength, QAngle> OdomMath::computeDistanceAndAngleToPoint(const Point &ipoint,
                                                                    const OdomState &istate) {
  const auto [xDiff, yDiff] = computeDiffs(ipoint, istate);
  return std::make_pair(computeDistance(xDiff, yDiff) * meter,
                        computeAngle(xDiff, yDiff, istate.theta.convert(radian)) * radian);
}

std::pair<double, double> OdomMath::computeDiffs(const Point &ipoint, const OdomState &istate) {
  const double xDiff = (ipoint.x - istate.x).convert(meter);
  const double yDiff = (ipoint.y - istate.y).convert(meter);
  return std::make_pair(xDiff, yDiff);
}

double OdomMath::computeDistance(double xDiff, double yDiff) {
  return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double OdomMath::computeAngle(double xDiff, double yDiff, double theta) {
  return std::atan2(yDiff, xDiff) - theta;
}

QAngle OdomMath::constrainAngle360(const QAngle &theta) {
  return theta - 360_deg * std::floor(theta.convert(degree) / 360.0);
}

QAngle OdomMath::constrainAngle180(const QAngle &theta) {
  return theta - 360_deg * std::floor((theta.convert(degree) + 180.0) / 360.0);
}
} // namespace okapi
