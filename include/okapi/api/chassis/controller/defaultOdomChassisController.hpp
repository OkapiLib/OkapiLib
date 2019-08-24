/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include <memory>

namespace okapi {
class DefaultOdomChassisController : public OdomChassisController {
  public:
  /**
   * Odometry based chassis controller that moves using the V5 motor's integrated control. Spins up
   * a task at the default priority plus 1 for odometry when constructed.
   *
   * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
   * turn some amount, you instead tell it to drive to a specific point on the field or turn to
   * a specific angle, relative to its starting position.
   *
   * @param itimeUtil The TimeUtil.
   * @param imodel chassis model to use
   * @param iodometry The odometry.
   * @param ileftController left side controller
   * @param irightController right side controller
   * @param igearset The motor gearset.
   * @param iscales The chassis scales.
   * @param imoveThreshold minimum length movement (smaller movements will be skipped)
   * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
   * @param ilogger The logger this instance will log to.
   */
  DefaultOdomChassisController(const TimeUtil &itimeUtil,
                               std::unique_ptr<Odometry> iodometry,
                               std::shared_ptr<ChassisController> icontroller,
                               QLength imoveThreshold = 10_mm,
                               QAngle iturnThreshold = 1_deg,
                               std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

  DefaultOdomChassisController(const DefaultOdomChassisController &) = delete;
  DefaultOdomChassisController(DefaultOdomChassisController &&other) = delete;
  DefaultOdomChassisController &operator=(const DefaultOdomChassisController &other) = delete;
  DefaultOdomChassisController &operator=(DefaultOdomChassisController &&other) = delete;

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ipoint The target point to navigate to.
   * @param imode The mode to read the target point in.
   * @param ibackwards Whether to drive to the target point backwards.
   * @param ioffset An offset from the target point in the direction pointing towards the robot. The
   * robot will stop this far away from the target point.
   */
  void driveToPoint(const Point2d &ipoint,
                    const StateMode &imode = StateMode::FRAME_TRANSFORMATION,
                    bool ibackwards = false,
                    const QLength &ioffset = 0_mm) override;

  /**
   * Turns the robot to face a point in the odom frame.
   *
   * @param ipoint The target point to turn to face.
   * @param imode The mode to read the target point in.
   */
  void turnToPoint(const Point2d &ipoint,
                   const StateMode &imode = StateMode::FRAME_TRANSFORMATION) override;

  /**
   * Turns the robot to face an angle in the odom frame.
   *
   * @param iangle The angle to turn to.
   */
  void turnToAngle(const QAngle &iangle) override;

  void moveDistance(QLength itarget) override;
  void moveDistance(double itarget) override;
  void moveDistanceAsync(QLength itarget) override;
  void moveDistanceAsync(double itarget) override;
  void turnAngle(QAngle idegTarget) override;
  void turnAngle(double idegTarget) override;
  void turnAngleAsync(QAngle idegTarget) override;
  void turnAngleAsync(double idegTarget) override;
  void setTurnsMirrored(bool ishouldMirror) override;
  void waitUntilSettled() override;
  void stop() override;
  ChassisScales getChassisScales() const override;
  AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;
  std::shared_ptr<ChassisModel> getModel() override;
  ChassisModel &model() override;

  /**
   * @return The internal chassis controller.
   */
  std::shared_ptr<ChassisController> getChassisController();

  /**
   * @return The internal chassis controller.
   */
  ChassisController &chassisController();

  protected:
  std::shared_ptr<Logger> logger;
  std::shared_ptr<ChassisController> controller;
};
} // namespace okapi
