/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ODOMCHASSISCONTROLLERPID_HPP_
#define _OKAPI_ODOMCHASSISCONTROLLERPID_HPP_

#include "api.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/impl/chassis/controller/chassisControllerPid.hpp"
#include "okapi/impl/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include <memory>

namespace okapi {
class OdomChassisControllerPID : public OdomChassisController, public ChassisControllerPID {
  public:
  /**
   * Odometry based chassis controller that moves using PID control. Spins up a task at the default
   * priority plus 1 for odometry when constructed.
   *
   * This constructor uses the V5 motor's integrated encoders.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param iscale straight scale
   * @param iturnScale turn scale
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param imoveThreshold minimum length movement
   */
  OdomChassisControllerPID(Motor ileftSideMotor, Motor irightSideMotor, const double iscale,
                           const double iturnScale,
                           const IterativePosPIDControllerArgs &idistanceArgs,
                           const IterativePosPIDControllerArgs &iangleArgs,
                           const float imoveThreshold = 10);

  /**
   * Odometry based chassis controller that moves using PID control. Spins up a task at the default
   * priority plus 1 for odometry when constructed.
   *
   * This constructor uses the V5 motor's integrated encoders.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param iscale straight scale
   * @param iturnScale turn scale
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param imoveThreshold minimum length movement
   */
  OdomChassisControllerPID(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                           const double iscale, const double iturnScale,
                           const IterativePosPIDControllerArgs &idistanceArgs,
                           const IterativePosPIDControllerArgs &iangleArgs,
                           const float imoveThreshold = 10);
  /**
   * Odometry based chassis controller that moves using PID control. Spins up a task at the default
   * priority plus 1 for odometry when constructed.
   *
   * This constructor uses the V5 motor's integrated encoders.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param iscale straight scale
   * @param iturnScale turn scale
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param imoveThreshold minimum length movement
   */
  OdomChassisControllerPID(std::shared_ptr<AbstractMotor> ileftSideMotor,
                           std::shared_ptr<AbstractMotor> irightSideMotor, const double iscale,
                           const double iturnScale,
                           const IterativePosPIDControllerArgs &idistanceArgs,
                           const IterativePosPIDControllerArgs &iangleArgs,
                           const float imoveThreshold = 10);

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
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param imoveThreshold minimum length movement
   */
  OdomChassisControllerPID(std::shared_ptr<SkidSteerModel> imodel, const double iscale,
                           const double iturnScale,
                           const IterativePosPIDControllerArgs &idistanceArgs,
                           const IterativePosPIDControllerArgs &iangleArgs,
                           const float imoveThreshold = 10);

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction pointing towards the robot
   */
  virtual void driveToPoint(const float ix, const float iy, const bool ibackwards = false,
                            const float ioffset = 0) override;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle angle to turn to
   */
  virtual void turnToAngle(const float iangle) override;
};
} // namespace okapi

#endif
