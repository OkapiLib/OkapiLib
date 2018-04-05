/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ODOMMATH_HPP_
#define _OKAPI_ODOMMATH_HPP_

#include "okapi/odometry/odometry.hpp"
#include <tuple>

namespace okapi {
class DistanceAndAngle {
  public:
  DistanceAndAngle();
  DistanceAndAngle(const double ilength, const double itheta);

  virtual ~DistanceAndAngle();

  double length = 0;
  double theta = 0;
};

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
  static double computeDistanceToPoint(const double ix, const double iy, const OdomState &istate);

  /**
   * Computes the angle from the given Odometry state to the given point.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param istate odometry state
   * @return angle to the point
   */
  static double computeAngleToPoint(const double ix, const double iy, const OdomState &istate);

  /**
   * Computes the distance and angle from the given Odometry state to the given point.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param istate odometry state
   * @return distance and angle to the point
   */
  static DistanceAndAngle computeDistanceAndAngleToPoint(const double ix, const double iy,
                                                         const OdomState &istate);

  /**
   * Attempt to guess scales based on robot dimensions.
   *
   * @param chassisDiam center-to-center wheelbase diameter in inches
   * @param wheelDiam edge-to-edge wheel diameter in inches
   * @param ticksPerRev ticks per revolution (default is 360)
   * @return scales in the format {straight scale, turn scale}
   */
  static std::tuple<double, double> guessScales(const double chassisDiam, const double wheelDiam,
                                                const double ticksPerRev = 1800.0);

  private:
  OdomMath();
  ~OdomMath();
};
} // namespace okapi

#endif
