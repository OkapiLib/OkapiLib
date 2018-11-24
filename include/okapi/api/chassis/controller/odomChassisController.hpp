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

namespace okapi {
class OdomChassisController : public virtual ChassisController {
  public:
  /**
   * Odometry based chassis controller. Starts task at the default for odometry when constructed.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle relative to its starting position.
   *
   * @param iparams odometry parameters for the internal odometry math
   * @param imoveThreshold minimum length movement (smaller movements will be skipped)
   * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
   */
  OdomChassisController(const std::shared_ptr<SkidSteerModel> &imodel,
                        std::unique_ptr<Odometry> iodometry,
                        const QLength &imoveThreshold = 10_mm,
                        const QAngle &iturnThreshold = 1_deg);

  ~OdomChassisController() override;

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction pointing towards the robot
   */
  virtual void
  driveToPoint(QLength ix, QLength iy, bool ibackwards = false, QLength ioffset = 0_mm) = 0;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  virtual void turnToAngle(QAngle iangle) = 0;

  /**
   * Returns the current state.
   *
   * @return current state
   */
  virtual OdomState getState() const;

  /**
   * Set a new state to be the current state.
   *
   * @param istate new state
   */
  virtual void setState(const OdomState &istate);

  /**
   * Set a new move threshold. Any requested movements smaller than this threshold will be skipped.
   *
   * @param imoveThreshold new move threshold
   */
  virtual void setMoveThreshold(QLength imoveThreshold);

  /**
   * Set a new turn threshold. Any requested turns smaller than this threshold will be skipped.
   *
   * @param iturnTreshold new turn threshold
   */
  virtual void setTurnThreshold(QAngle iturnTreshold);

  protected:
  QLength moveThreshold;
  QAngle turnThreshold;
  std::unique_ptr<Odometry> odom;
  CrossplatformThread task;
};
} // namespace okapi
