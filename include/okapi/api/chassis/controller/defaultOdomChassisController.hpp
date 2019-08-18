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
   * @param iodometry The odometry to read state estimates from.
   * @param icontroller The chassis controller to delegate to.
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

  /**
   * Drives the robot straight to a point in the odom frame.
   *
   * @param ix x coordinate
   * @param iy y coordinate
   * @param ibackwards whether to drive to the target point backwards
   * @param ioffset offset from target point in the direction pointing towards the robot
   */
  void driveToPoint(const QLength &ix,
                    const QLength &iy,
                    bool ibackwards = false,
                    const QLength &ioffset = 0_mm) override;

  /**
   * @return The internal ChassisController.
   */
  std::shared_ptr<ChassisController> getChassisController();

  /**
   * @return The internal ChassisController.
   */
  ChassisController &chassisController();

  /**
   * This delegates to the input ChassisController.
   */
  void turnToAngle(const QAngle &iangle) override;

  /**
   * This delegates to the input ChassisController.
   */
  void moveDistance(QLength itarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void moveDistance(double itarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void moveDistanceAsync(QLength itarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void moveDistanceAsync(double itarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void turnAngle(QAngle idegTarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void turnAngle(double idegTarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void turnAngleAsync(QAngle idegTarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void turnAngleAsync(double idegTarget) override;

  /**
   * This delegates to the input ChassisController.
   */
  void setTurnsMirrored(bool ishouldMirror) override;

  /**
   * This delegates to the input ChassisController.
   */
  void waitUntilSettled() override;

  /**
   * This delegates to the input ChassisController.
   */
  void stop() override;

  /**
   * This delegates to the input ChassisController.
   */
  ChassisScales getChassisScales() const override;

  /**
   * This delegates to the input ChassisController.
   */
  AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;

  /**
   * This delegates to the input ChassisController.
   */
  std::shared_ptr<ChassisModel> getModel() override;

  /**
   * This delegates to the input ChassisController.
   */
  ChassisModel &model() override;

  protected:
  std::shared_ptr<Logger> logger;
  std::shared_ptr<ChassisController> controller;
};
} // namespace okapi
