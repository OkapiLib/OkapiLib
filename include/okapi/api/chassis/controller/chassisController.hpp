/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLER_HPP_
#define _OKAPI_CHASSISCONTROLLER_HPP_

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class ChassisController : public ChassisModel {
  public:
  /**
   * A ChassisController adds a closed-loop layer on top of a ChassisModel. moveDistance and
   * turnAngle both use closed-loop control to move the robot. There are passthrough functions for
   * everything defined in ChassisModel.
   *
   * @param imodel underlying ChassisModel
   */
  explicit ChassisController(std::shared_ptr<ChassisModel> imodel);

  ~ChassisController() override;

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
  void forward(double ispeed) const override;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  void driveVector(double iySpeed, double izRotation) const override;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  void rotate(double ispeed) const override;

  /**
   * Stop the robot (set all the motors to 0).
   */
  void stop() override;

  /**
   * Drive the robot with a tank drive layout. Uses voltage mode.
   *
   * @param ileftSpeed left side speed
   * @param irightSpeed right side speed
   * @param ithreshold deadband on joystick values
   */
  void tank(double ileftSpeed, double irightSpeed, double ithreshold = 0) const override;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  void arcade(double iySpeed, double izRotation, double ithreshold = 0) const override;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  void left(double ispeed) const override;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  void right(double ispeed) const override;

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  std::valarray<std::int32_t> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  void resetSensors() const override;

  /**
   * Set the brake mode for each motor.
   *
   * @param mode new brake mode
   */
  void setBrakeMode(AbstractMotor::brakeMode mode) const override;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  void setEncoderUnits(AbstractMotor::encoderUnits units) const override;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  void setGearing(AbstractMotor::gearset gearset) const override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   */
  void setPosPID(double ikF, double ikP, double ikI, double ikD) const override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @param ifilter a constant used for filtering the profile acceleration
   * @param ilimit the integral limit
   * @param ithreshold the threshold for determining if a position movement has reached its goal
   * @param iloopSpeed the rate at which the PID computation is run (in ms)
   */
  void setPosPIDFull(double ikF,
                     double ikP,
                     double ikI,
                     double ikD,
                     double ifilter,
                     double ilimit,
                     double ithreshold,
                     double iloopSpeed) const override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   */
  void setVelPID(double ikF, double ikP, double ikI, double ikD) const override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @param ifilter a constant used for filtering the profile acceleration
   * @param ilimit the integral limit
   * @param ithreshold the threshold for determining if a position movement has reached its goal
   * @param iloopSpeed the rate at which the PID computation is run (in ms)
   */
  void setVelPIDFull(double ikF,
                     double ikP,
                     double ikI,
                     double ikD,
                     double ifilter,
                     double ilimit,
                     double ithreshold,
                     double iloopSpeed) const override;

  /**
   * Get the underlying ChassisModel. This should be used sparingly and carefully because it can
   * result in multiple owners writing to the same set of motors.
   */
  std::shared_ptr<ChassisModel> getChassisModel() const;

  /**
   * Get the ChassisScales.
   */
  virtual ChassisScales getChassisScales() const = 0;

  protected:
  std::shared_ptr<ChassisModel> model;
};
} // namespace okapi

#endif
