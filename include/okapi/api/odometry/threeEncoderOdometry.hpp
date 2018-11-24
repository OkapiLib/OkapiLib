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
  ThreeEncoderOdometry(std::shared_ptr<ReadOnlyChassisModel> imodel,
                       const ChassisScales &ichassisScales,
                       const TimeUtil &itimeUtil);

  /**
   * Do odometry math in an infinite loop.
   */
  void step() override;

  /**
   * Treat the input as a ThreeEncoderOdometry pointer and call loop. Meant to be used to bounce
   * into a thread because loop runs forever.
   *
   * @param context pointer to a ThreeEncoderOdometry object
   */
  static void trampoline(void *context);

  protected:
  std::shared_ptr<ReadOnlyChassisModel> model;
  std::unique_ptr<AbstractRate> rate;
  std::valarray<std::int32_t> newTicks{0, 0, 0}, tickDiff{0, 0, 0}, lastTicks{0, 0, 0};
};
} // namespace okapi
