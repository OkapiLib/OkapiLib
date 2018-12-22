/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/logging.hpp"
#include <tuple>

namespace okapi {
class OdomMath {
  public:
  /**
   * Computes the distance from the given Odometry state to the given point.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param istate odometry state
   * @return distance between the points
   */
  static QLength computeDistanceToPoint(QLength ix, QLength iy, const OdomState &istate);

  /**
   * Computes the angle from the given Odometry state to the given point.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param istate odometry state
   * @return angle to the point
   */
  static QAngle computeAngleToPoint(QLength ix, QLength iy, const OdomState &istate);

  /**
   * Computes the distance and angle from the given Odometry state to the given point.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param istate odometry state
   * @return distance and angle to the point
   */
  static std::tuple<QLength, QAngle>
  computeDistanceAndAngleToPoint(QLength ix, QLength iy, const OdomState &istate);

  private:
  OdomMath();
  ~OdomMath();

  Logger *logger;
};
} // namespace okapi
