/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/readOnlyChassisModel.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <atomic>
#include <memory>
#include <valarray>

namespace okapi {
class Odometry {
  public:
  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0, 0)).
   *
   * @param itimeUtil The TimeUtil.
   * @param imodel The chassis model for reading sensors.
   * @param ichassisScales The chassis dimensions.
   * @param iwheelVelDelta The maximum delta between wheel velocities to consider the robot as
   * driving straight.
   * @param ilogger The logger this instance will log to.
   */
  Odometry(const TimeUtil &itimeUtil,
           const std::shared_ptr<ReadOnlyChassisModel> &imodel,
           const ChassisScales &ichassisScales,
           const QSpeed &iwheelVelDelta = 0.0001_mps,
           const std::shared_ptr<Logger> &ilogger = Logger::getDefaultLogger());

  virtual ~Odometry() = default;

  /**
   * Sets the drive and turn scales.
   */
  virtual void setScales(const ChassisScales &ichassisScales);

  /**
   * Do one odometry step.
   */
  virtual void step();

  /**
   * Returns the current state.
   *
   * @param imode The mode to return the state in.
   * @return The current state in the given format.
   */
  virtual OdomState getState(const StateMode &imode = StateMode::FRAME_TRANSFORMATION) const;

  /**
   * Sets a new state to be the current state.
   *
   * @param istate The new state in the given format.
   * @param imode The mode to treat the input state as.
   */
  virtual void setState(const OdomState &istate,
                        const StateMode &imode = StateMode::FRAME_TRANSFORMATION);

  protected:
  std::shared_ptr<Logger> logger;
  std::unique_ptr<AbstractRate> rate;
  std::unique_ptr<AbstractTimer> timer;
  std::shared_ptr<ReadOnlyChassisModel> model;
  ChassisScales chassisScales;
  QSpeed wheelVelDelta;
  OdomState state;
  std::valarray<std::int32_t> newTicks{0, 0, 0}, tickDiff{0, 0, 0}, lastTicks{0, 0, 0};

  /**
   * Does the math, side-effect free, for one odom step.
   *
   * @param itickDiff The tick difference from the previous step to this step.
   * @param ideltaT The time difference from the previous step to this step.
   * @return The newly computed OdomState.
   */
  virtual OdomState odomMathStep(const std::valarray<std::int32_t> &itickDiff,
                                 const QTime &ideltaT);
};
} // namespace okapi
