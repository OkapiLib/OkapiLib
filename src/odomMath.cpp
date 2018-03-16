/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/odometry/odomMath.hpp"
#include "okapi/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
OdomMath::OdomMath() {
}

OdomMath::~OdomMath() = default;

float OdomMath::computeDistanceToPoint(const float ix, const float iy, const OdomState &istate) {
  const float xDiff = ix - istate.x;
  const float yDiff = iy - istate.y;
  return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

float OdomMath::computeAngleToPoint(const float ix, const float iy, const OdomState &istate) {
  const float xDiff = ix - istate.x;
  const float yDiff = iy - istate.y;
  return (std::atan2(yDiff, xDiff) * radianToDegree) - istate.theta;
}

DistanceAndAngle OdomMath::computeDistanceAndAngleToPoint(const float ix, const float iy,
                                                          const OdomState &istate) {
  const float xDiff = ix - istate.x;
  const float yDiff = iy - istate.y;

  DistanceAndAngle out;
  out.length = std::sqrt((xDiff * xDiff) + (yDiff * yDiff));

  // Small xDiff is essentially dividing by zero, so avoid it and do custom math
  if (xDiff < 0.0001 && xDiff > -0.0001) {
    const int yDiffSign = static_cast<int>(copysign(1, yDiff));
    if (yDiffSign == 1) {
      out.theta = -1 * istate.theta;
    } else if (yDiffSign == -1) {
      out.theta = -180 - istate.theta;

      // Fix theta
      if (out.theta <= -360)
        out.theta += 360;
      else if (out.theta >= 360)
        out.theta -= 360;
    }
  } else {
    out.theta = (std::atan2(yDiff, xDiff) * radianToDegree) - istate.theta;
  }

  return out;
}

std::tuple<float, float> OdomMath::guessScales(const float chassisDiam, const float wheelDiam,
                                                 const float ticksPerRev) {
  const float scale = ((wheelDiam * pi * inchToMM) / ticksPerRev) *
                       0.9945483364; // The scale is usually off by this amount
  const float turnScale = (1.0 / (chassisDiam * inchToMM)) * radianToDegree * 2;
  return std::make_tuple(scale, turnScale);
}
} // namespace okapi
