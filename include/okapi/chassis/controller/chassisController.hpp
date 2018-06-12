/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLER_HPP_
#define _OKAPI_CHASSISCONTROLLER_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include "okapi/units/QAngle.hpp"
#include "okapi/units/QLength.hpp"
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
  ChassisController(std::shared_ptr<ChassisModel> imodel);

  virtual ~ChassisController();

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistance(const QLength itarget) = 0;

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  virtual void moveDistance(const double itarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngle(const QAngle idegTarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  virtual void turnAngle(const double idegTarget) = 0;

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void forward(const int ispeed) const;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  virtual void driveVector(const double iySpeed, const double izRotation) const;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void rotate(const int ispeed) const;

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
  virtual void tank(const double ileftSpeed, const double irightSpeed,
                    const double ithreshold = 0) const;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(const double iySpeed, const double izRotation,
                      const double ithreshold = 0) const;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(const double ispeed) const;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(const double ispeed) const;

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
  virtual void setBrakeMode(const pros::c::motor_brake_mode_e_t mode) const;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(const pros::c::motor_encoder_units_e_t units) const;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(const pros::c::motor_gearset_e_t gearset) const;

  protected:
  std::shared_ptr<ChassisModel> model;
};
} // namespace okapi

#endif
