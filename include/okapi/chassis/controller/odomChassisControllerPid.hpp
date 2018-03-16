/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ODOMCHASSISCONTROLLERPID_HPP_
#define _OKAPI_ODOMCHASSISCONTROLLERPID_HPP_

#include "api.h"
#include "okapi/chassis/controller/chassisControllerPid.hpp"
#include "okapi/chassis/controller/odomChassisController.hpp"
#include "okapi/odometry/odometry.hpp"

namespace okapi {
class OdomChassisControllerPID : public OdomChassisController, public ChassisControllerPID {
  public:
  /**
   * Odometry based chassis controller that moves using PID control. Spins up a task at the default
   * priority plus 1 for odometry when constructed.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param iparams odometry parameters for the internal odometry math
   * @param idistanceParams distance PID controller params
   * @param iangleParams angle PID controller params (keeps the robot straight)
   */
  OdomChassisControllerPID(const OdometryParams &iparams,
                           const PIDControllerParams &idistanceParams,
                           const PIDControllerParams &iangleParams);

  virtual ~OdomChassisControllerPID();

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction point towards the robot
   */
  void driveToPoint(const float ix, const float iy, const bool ibackwards = false,
                    const float ioffset = 0) override;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  void turnToAngle(const float iangle) override;
};
} // namespace okapi

#endif
