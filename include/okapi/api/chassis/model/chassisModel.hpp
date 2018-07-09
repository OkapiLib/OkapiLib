/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISMODEL_HPP_
#define _OKAPI_CHASSISMODEL_HPP_

#include "okapi/api/device/motor/abstractMotor.hpp"
#include <array>
#include <initializer_list>
#include <memory>
#include <valarray>
#include <vector>

namespace okapi {
class ChassisModelArgs {
  public:
  virtual ~ChassisModelArgs();
};

class ChassisModel {
  public:
  virtual ~ChassisModel();

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void forward(double ispeed) const = 0;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  virtual void driveVector(double iySpeed, double izRotation) const = 0;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ispeed motor power
   */
  virtual void rotate(double ispeed) const = 0;

  /**
   * Stop the robot (set all the motors to 0).
   */
  virtual void stop() const = 0;

  /**
   * Drive the robot with a tank drive layout. Uses voltage mode.
   *
   * @param ileftSpeed left side speed
   * @param irightSpeed right side speed
   * @param ithreshold deadband on joystick values
   */
  virtual void tank(double ileftSpeed, double irightSpeed, double ithreshold = 0) const = 0;

  /**
   * Drive the robot with an arcade drive layout. Uses voltage mode.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(double iySpeed, double izRotation, double ithreshold = 0) const = 0;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(double ispeed) const = 0;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(double ispeed) const = 0;

  /**
   * Read the sensors.
   *
   * @return sensor readings (format is implementation dependent)
   */
  virtual std::valarray<std::int32_t> getSensorVals() const = 0;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const = 0;

  /**
   * Set the brake mode for each motor.
   *
   * @param mode new brake mode
   */
  virtual void setBrakeMode(AbstractMotor::brakeMode mode) const = 0;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(AbstractMotor::encoderUnits units) const = 0;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(AbstractMotor::gearset gearset) const = 0;
};
} // namespace okapi

#endif
