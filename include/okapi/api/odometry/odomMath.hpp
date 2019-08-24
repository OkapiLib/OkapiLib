/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/odometry/point2d.h"
#include "okapi/api/util/logging.hpp"
#include <tuple>

namespace okapi {
class OdomMath {
  public:
  /**
   * Computes the distance from the given Odometry state to the given point. The point and the
   * OdomState must be in the same StateMode.
   *
   * @param ipoint The point.
   * @param istate The odometry state.
   * @return The distance between the odometry state and the point.
   */
  static QLength computeDistanceToPoint(const Point &ipoint, const OdomState &istate);

  /**
   * Computes the angle from the given Odometry state to the given point. The point and the
   * OdomState must be in the same StateMode.
   *
   * @param ipoint The point.
   * @param istate The odometry state.
   * @return The angle between the odometry state and the point.
   */
  static QAngle computeAngleToPoint(const Point &ipoint, const OdomState &istate);

  /**
   * Computes the distance and angle from the given Odometry state to the given point. The point and
   * the OdomState must be in the same StateMode.
   *
   * @param ipoint The point.
   * @param istate The odometry state.
   * @return The distance and angle between the odometry state and the point.
   */
  static std::pair<QLength, QAngle> computeDistanceAndAngleToPoint(const Point &ipoint,
                                                                   const OdomState &istate);

  private:
  OdomMath();
  ~OdomMath();
};
} // namespace okapi
