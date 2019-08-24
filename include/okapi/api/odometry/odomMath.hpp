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
   * Computes the distance from the given Odometry state to the given point.
   *
   * @param ipoint The point.
   * @param istate The odometry state, assumed to be in `StateMode::FRAME_TRANSFORMATION`.
   * @param ipointMode The StateMode that `ipoint` is in.
   * @return The distance between the odometry state and the point.
   */
  static QLength
  computeDistanceToPoint(const Point &ipoint,
                         const OdomState &istate,
                         const StateMode &ipointMode = StateMode::FRAME_TRANSFORMATION);

  /**
   * Computes the angle from the given Odometry state to the given point.
   *
   * @param ipoint The point.
   * @param istate The odometry state, assumed to be in `StateMode::FRAME_TRANSFORMATION`.
   * @param ipointMode The StateMode that `ipoint` is in.
   * @return The angle between the odometry state and the point.
   */
  static QAngle computeAngleToPoint(const Point &ipoint,
                                    const OdomState &istate,
                                    const StateMode &ipointMode = StateMode::FRAME_TRANSFORMATION);

  /**
   * Computes the distance and angle from the given Odometry state to the given point.
   *
   * @param ipoint The point.
   * @param istate The odometry state, assumed to be in `StateMode::FRAME_TRANSFORMATION`.
   * @param ipointMode The StateMode that `ipoint` is in.
   * @return The distance and angle between the odometry state and the point.
   */
  static std::pair<QLength, QAngle>
  computeDistanceAndAngleToPoint(const Point &ipoint,
                                 const OdomState &istate,
                                 const StateMode &ipointMode = StateMode::FRAME_TRANSFORMATION);

  private:
  OdomMath();
  ~OdomMath();
};
} // namespace okapi
