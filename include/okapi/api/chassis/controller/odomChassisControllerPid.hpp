/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include <memory>

namespace okapi {
class OdomChassisControllerPID : public OdomChassisController, public ChassisControllerPID {
  public:
  /**
   * Odometry based chassis controller that moves using PID control. Spins up a task at the default
   * priority plus 1 for odometry when constructed.
   *
   * This constructor uses the encoders from the supplied chassis model.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param imodel chassis model to use
   * @param iscale straight scale
   * @param iturnScale turn scale
   * @param idistanceController distance PID controller
   * @param idistanceController angle PID controller (keeps the robot straight)
   * @param imoveThreshold minimum length movement (smaller movements will be skipped)
   * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
   */
  OdomChassisControllerPID(const TimeUtil &itimeUtil,
                           const std::shared_ptr<SkidSteerModel> &imodel,
                           std::unique_ptr<Odometry> iodometry,
                           std::unique_ptr<IterativePosPIDController> idistanceController,
                           std::unique_ptr<IterativePosPIDController> iturnController,
                           std::unique_ptr<IterativePosPIDController> iangleController,
                           const AbstractMotor::GearsetRatioPair &igearset,
                           const ChassisScales &iscales,
                           const QLength &imoveThreshold = 10_mm,
                           const QAngle &iturnThreshold = 1_deg,
                           const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction pointing towards the robot
   */
  void
  driveToPoint(QLength ix, QLength iy, bool ibackwards = false, QLength ioffset = 0_mm) override;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  void turnToAngle(QAngle iangle) override;

  protected:
  std::shared_ptr<Logger> logger;
};
} // namespace okapi
