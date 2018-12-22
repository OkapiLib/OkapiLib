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
   * @param imodelArgs ChassisModel for reading sensors
   * @param ichassisScales See ChassisScales docs (the middle wheel scale is the third member)
   * @param irateSupplier a supplier of AbstractRate implementations
   */
  ThreeEncoderOdometry(const TimeUtil &itimeUtil,
                       std::shared_ptr<ReadOnlyChassisModel> imodel,
                       const ChassisScales &ichassisScales,
                       const QSpeed &iwheelVelDelta = 0.0001_mps,
                       const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  /**
   * Treat the input as a ThreeEncoderOdometry pointer and call loop. Meant to be used to bounce
   * into a thread because loop runs forever.
   *
   * @param context pointer to a ThreeEncoderOdometry object
   */
  static void trampoline(void *context);

  protected:
  std::shared_ptr<Logger> logger;
  std::shared_ptr<ReadOnlyChassisModel> model;
  std::unique_ptr<AbstractRate> rate;

  /**
   * Does the math, side-effect free, for one odom step.
   *
   * @param tickDiff The tick difference from the previous step to this step.
   * @param deltaT The time difference from the previous step to this step.
   * @return The estimated position/orientation offset.
   */
  std::tuple<OdomState, double, double> odomMathStep(std::valarray<std::int32_t> &tickDiff,
                                                     const QTime &deltaT) override;
};
} // namespace okapi
