/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISMODEL_HPP_
#define _OKAPI_CHASSISMODEL_HPP_

#include "okapi/device/motor/abstractMotor.hpp"
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
  virtual void forward(const double ispeed) const = 0;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  virtual void driveVector(const double iySpeed, const double izRotation) const = 0;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ispeed motor power
   */
  virtual void rotate(const double ispeed) const = 0;

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
  virtual void tank(const double ileftSpeed, const double irightSpeed,
                    const double ithreshold = 0) const = 0;

  /**
   * Drive the robot with an arcade drive layout. Uses voltage mode.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(const double iySpeed, const double izRotation,
                      const double ithreshold = 0) const = 0;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(const double ispeed) const = 0;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(const double ispeed) const = 0;

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
  virtual void setBrakeMode(const pros::c::motor_brake_mode_e_t mode) const = 0;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(const pros::c::motor_encoder_units_e_t units) const = 0;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(const pros::c::motor_gearset_e_t gearset) const = 0;
};
} // namespace okapi

#endif
