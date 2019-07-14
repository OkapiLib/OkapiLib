/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/coreProsAPI.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
class OdomChassisController {
  public:
  /**
   * Odometry based chassis controller. Starts task at the default for odometry when constructed.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle relative to its starting position.
   *
   * @param itimeUtil The TimeUtil.
   * @param iparams odometry parameters for the internal odometry math
   * @param imoveThreshold minimum length movement (smaller movements will be skipped)
   * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
   */
  OdomChassisController(const TimeUtil &itimeUtil,
                        std::unique_ptr<Odometry> iodometry,
                        const QLength &imoveThreshold = 10_mm,
                        const QAngle &iturnThreshold = 1_deg);

  virtual ~OdomChassisController();

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction pointing towards the robot
   */
  virtual void driveToPoint(const QLength &ix,
                            const QLength &iy,
                            bool ibackwards = false,
                            const QLength &ioffset = 0_mm) = 0;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  virtual void turnToAngle(const QAngle &iangle) = 0;

  /**
   * Returns the current state.
   *
   * @param imode The mode to return the state in.
   * @return The current state in the given format.
   */
  virtual OdomState getState(const StateMode &imode = StateMode::FRAME_TRANSFORMATION) const;

  /**
   * Set a new state to be the current state.
   *
   * @param istate The new state in the given format.
   * @param imode The mode to treat the input state as.
   */
  virtual void setState(const OdomState &istate,
                        const StateMode &imode = StateMode::FRAME_TRANSFORMATION);

  /**
   * Set a new move threshold. Any requested movements smaller than this threshold will be skipped.
   *
   * @param imoveThreshold new move threshold
   */
  virtual void setMoveThreshold(const QLength &imoveThreshold);

  /**
   * Set a new turn threshold. Any requested turns smaller than this threshold will be skipped.
   *
   * @param iturnTreshold new turn threshold
   */
  virtual void setTurnThreshold(const QAngle &iturnTreshold);

  /**
   * @return The current move threshold.
   */
  virtual QLength getMoveThreshold() const;

  /**
   * @return The current turn threshold.
   */
  virtual QAngle getTurnThreshold() const;

  /**
   * Starts the internal odometry thread. This should not be called by normal users.
   */
  void startOdomThread();

  /**
   * Returns the underlying thread handle.
   *
   * @return The underlying thread handle.
   */
  CrossplatformThread *getOdomThread() const;

  protected:
  TimeUtil timeUtil;
  QLength moveThreshold;
  QAngle turnThreshold;
  std::unique_ptr<Odometry> odom;
  CrossplatformThread *odomTask{nullptr};
  std::atomic_bool dtorCalled{false};

  static void trampoline(void *context);
  void loop();
};
} // namespace okapi
