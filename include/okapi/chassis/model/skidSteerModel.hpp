/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_SKIDSTEERMODEL_HPP_
#define _OKAPI_SKIDSTEERMODEL_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/rotarysensor/rotarySensor.hpp"

namespace okapi {
class SkidSteerModelArgs : public ChassisModelArgs {
  public:
  SkidSteerModelArgs(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                     const RotarySensor &ileftEnc, const RotarySensor &irightEnc,
                     const double imaxOutput = 100);

  // Create the sensors using the integrated encoder
  SkidSteerModelArgs(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                     const double imaxOutput = 100);

  const AbstractMotor &leftSideMotor;
  const AbstractMotor &rightSideMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
  const double maxOutput;
};

class SkidSteerModel : public ChassisModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc  left side encoder
   * @param irightEnc right side encoder
   */
  SkidSteerModel(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                 const RotarySensor &ileftEnc, const RotarySensor &irightEnc,
                 const double imaxOutput = 100);

  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the left and right motors (using the integrated
   * encoders).
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   */
  SkidSteerModel(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                 const double imaxOutput = 100);

  SkidSteerModel(const SkidSteerModelArgs &iparams);

  SkidSteerModel(const SkidSteerModel &other);

  /**
   * Drive the robot forwards (using open-loop control). Uses velocity mode.
   *
   * @param ispeed motor power
   */
  virtual void forward(const double ispeed) const override;

  /**
   * Drive the robot in an arc (using open-loop control). Uses velocity mode.
   * The algorithm is (approximately):
   *   leftPower = ySpeed + zRotation
   *   rightPower = ySpeed - zRotation
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   */
  virtual void driveVector(const double iySpeed, const double izRotation) const override;

  /**
   * Turn the robot clockwise (using open-loop control). Uses velocity mode.
   *
   * @param ispeed motor power
   */
  virtual void rotate(const double ispeed) const override;

  /**
   * Stop the robot (set all the motors to 0). Uses velocity mode.
   */
  virtual void stop() const override;

  /**
   * Drive the robot with a tank drive layout. Uses voltage mode.
   *
   * @param ileftSpeed left side speed
   * @param irightSpeed right side speed
   * @param ithreshold deadband on joystick values
   */
  virtual void tank(const double ileftSpeed, const double irightSpeed,
                    const double ithreshold = 0) const override;

  /**
   * Drive the robot with an arcade drive layout. Uses voltage mode.
   *
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(const double iySpeed, const double izRotation,
                      const double ithreshold = 0) const override;

  /**
   * Power the left side motors. Uses velocity mode.
   *
   * @param ispeed motor power
   */
  virtual void left(const double ispeed) const override;

  /**
   * Power the right side motors. Uses velocity mode.
   *
   * @param ispeed motor power
   */
  virtual void right(const double ispeed) const override;

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  virtual std::valarray<int> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const override;

  /**
   * Set the brake mode for each motor.
   *
   * @param mode new brake mode
   */
  virtual void setBrakeMode(const motor_brake_mode_e_t mode) const override;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(const motor_encoder_units_e_t units) const override;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(const motor_gearset_e_t gearset) const override;

  /**
   * Returns the left side motor.
   *
   * @return the left side motor
   */
  const AbstractMotor &getLeftSideMotor() const;

  /**
   * Returns the left side motor.
   *
   * @return the left side motor
   */
  const AbstractMotor &getRightSideMotor() const;

  protected:
  const AbstractMotor &leftSideMotor;
  const AbstractMotor &rightSideMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
  const double maxOutput;
};
} // namespace okapi

#endif
