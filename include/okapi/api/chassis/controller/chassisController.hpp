/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLER_HPP_
#define _OKAPI_CHASSISCONTROLLER_HPP_

#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class ChassisController {
  public:
  /**
   * A ChassisController adds a closed-loop layer on top of a ChassisModel. moveDistance and
   * turnAngle both use closed-loop control to move the robot. There are passthrough functions for
   * everything defined in ChassisModel.
   *
   * @param imodel underlying ChassisModel
   */
  explicit ChassisController(std::unique_ptr<ChassisModel> imodel);

  virtual ~ChassisController();

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistance(QLength itarget) = 0;

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  virtual void moveDistance(double itarget) = 0;

  /**
   * Sets the target distance for the robot to drive straight (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistanceAsync(QLength itarget) = 0;

  /**
   * Sets the target distance for the robot to drive straight (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  virtual void moveDistanceAsync(double itarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngle(QAngle idegTarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  virtual void turnAngle(double idegTarget) = 0;

  /**
   * Sets the target angle for the robot to turn clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngleAsync(QAngle idegTarget) = 0;

  /**
   * Sets the target angle for the robot to turn clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  virtual void turnAngleAsync(double idegTarget) = 0;

  /**
   * Delays until the currently executing movement completes.
   */
  virtual void waitUntilSettled() = 0;

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void forward(int ispeed) const;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  virtual void driveVector(double iySpeed, double izRotation) const;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void rotate(int ispeed) const;

  /**
   * Stop the robot (set all the motors to 0).
   */
  virtual void stop() const;

  /**
   * Drive the robot with a tank drive layout. Uses voltage mode.
   *
   * @param ileftSpeed left side speed
   * @param irightSpeed right side speed
   * @param ithreshold deadband on joystick values
   */
  virtual void tank(double ileftSpeed, double irightSpeed, double ithreshold = 0) const;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(double iySpeed, double izRotation, double ithreshold = 0) const;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(double ispeed) const;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(double ispeed) const;

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  virtual std::valarray<std::int32_t> getSensorVals() const;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const;

  /**
   * Set the brake mode for each motor.
   *
   * @param mode new brake mode
   */
  virtual void setBrakeMode(AbstractMotor::brakeMode mode) const;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(AbstractMotor::encoderUnits units) const;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(AbstractMotor::gearset gearset) const;

  protected:
  std::unique_ptr<ChassisModel> model;
};
} // namespace okapi

#endif
