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
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"
#include "okapi/device/rotarysensor/adiEncoder.hpp"
#include "okapi/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
class SkidSteerModelArgs : public ChassisModelArgs {
  public:
  // Create the sensors using the integrated encoder
  SkidSteerModelArgs(std::shared_ptr<AbstractMotor> ileftSideMotor,
                     std::shared_ptr<AbstractMotor> irightSideMotor, const double imaxOutput = 127);

  SkidSteerModelArgs(std::shared_ptr<AbstractMotor> ileftSideMotor,
                     std::shared_ptr<AbstractMotor> irightSideMotor,
                     std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                     std::shared_ptr<ContinuousRotarySensor> irightEnc,
                     const double imaxOutput = 127);

  std::shared_ptr<AbstractMotor> leftSideMotor;
  std::shared_ptr<AbstractMotor> rightSideMotor;
  std::shared_ptr<ContinuousRotarySensor> leftSensor;
  std::shared_ptr<ContinuousRotarySensor> rightSensor;
  const double maxOutput;
};

class SkidSteerModel : public ChassisModel {
  public:
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
  SkidSteerModel(Motor ileftSideMotor, Motor irightSideMotor, const double imaxOutput = 127);

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
  SkidSteerModel(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                 const double imaxOutput = 127);

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
  SkidSteerModel(MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc,
                 ADIEncoder irightEnc, const double imaxOutput = 127);

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
  SkidSteerModel(std::shared_ptr<AbstractMotor> ileftSideMotor,
                 std::shared_ptr<AbstractMotor> irightSideMotor, const double imaxOutput = 127);

  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc  left side encoder
   * @param irightEnc right side encoder
   */
  SkidSteerModel(std::shared_ptr<AbstractMotor> ileftSideMotor,
                 std::shared_ptr<AbstractMotor> irightSideMotor,
                 std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                 std::shared_ptr<ContinuousRotarySensor> irightEnc, const double imaxOutput = 127);

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
  virtual std::valarray<std::int32_t> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const override;

  /**
   * Set the brake mode for each motor.
   *
   * @param mode new brake mode
   */
  virtual void setBrakeMode(const pros::c::motor_brake_mode_e_t mode) const override;

  /**
   * Set the encoder units for each motor.
   *
   * @param units new motor encoder units
   */
  virtual void setEncoderUnits(const pros::c::motor_encoder_units_e_t units) const override;

  /**
   * Set the gearset for each motor.
   *
   * @param gearset new motor gearset
   */
  virtual void setGearing(const pros::c::motor_gearset_e_t gearset) const override;

  /**
   * Returns the left side motor.
   *
   * @return the left side motor
   */
  std::shared_ptr<AbstractMotor> getLeftSideMotor() const;

  /**
   * Returns the left side motor.
   *
   * @return the left side motor
   */
  std::shared_ptr<AbstractMotor> getRightSideMotor() const;

  protected:
  std::shared_ptr<AbstractMotor> leftSideMotor;
  std::shared_ptr<AbstractMotor> rightSideMotor;
  std::shared_ptr<ContinuousRotarySensor> leftSensor;
  std::shared_ptr<ContinuousRotarySensor> rightSensor;
  const double maxOutput;
};
} // namespace okapi

#endif
