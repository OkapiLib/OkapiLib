/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <functional>

namespace okapi {
class ThreeEncoderOdometry : public Odometry {
  public:
  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0)).
   *
   * @param itimeUtil The TimeUtil.
   * @param imodel The chassis model for reading sensors.
   * @param ichassisScales See ChassisScales docs (the middle wheel scale is the third member)
   * @param iwheelVelDelta The maximum delta between wheel velocities to consider the robot as
   * driving straight.
   * @param ilogger The logger this instance will log to.
   */
  ThreeEncoderOdometry(const TimeUtil &itimeUtil,
                       std::shared_ptr<ReadOnlyChassisModel> imodel,
                       const ChassisScales &ichassisScales,
                       const QSpeed &iwheelVelDelta = 0.0001_mps,
                       const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  protected:
  std::shared_ptr<ReadOnlyChassisModel> model;

  /**
   * Does the math, side-effect free, for one odom step.
   *
   * @param tickDiff The tick difference from the previous step to this step.
   * @param deltaT The time difference from the previous step to this step.
   * @return The estimated position/orientation offset.
   */
  OdomState odomMathStep(std::valarray<std::int32_t> &tickDiff, const QTime &deltaT) override;
};
} // namespace okapi