/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ODOMCHASSISCONTROLLER_HPP_
#define _OKAPI_ODOMCHASSISCONTROLLER_HPP_

#include "api.h"
#include "okapi/chassis/controller/chassisController.hpp"
#include "okapi/odometry/odometry.hpp"

namespace okapi {
class OdomChassisController : public virtual ChassisController {
  public:
  /**
   * Odometry based chassis controller. Spins up a task at the default priority plus 1 for
   * odometry when constructed.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param iparams odometry parameters for the internal odometry math
   */
  OdomChassisController(const OdometryParams &iparams);

  virtual ~OdomChassisController();

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction point towards the robot
   */
  virtual void driveToPoint(const float ix, const float iy, const bool ibackwards = false,
                            const float ioffset = 0) = 0;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  virtual void turnToAngle(const float iangle) = 0;

  /**
   * Passthrough to internal Odometry object.
   *
   * @return state from internal Odometry object
   */
  OdomState getState() const;

  /**
   * Set a new move threshold. Any requested movements smaller than the move threshold will not be
   * performed.
   * 
   * @param imoveThreshold new move threshold
   */
  void setMoveThreshold(const float imoveThreshold);

  protected:
  float moveThreshold = 10; // Minimum length movement
  Odometry odom;
};
} // namespace okapi

#endif
