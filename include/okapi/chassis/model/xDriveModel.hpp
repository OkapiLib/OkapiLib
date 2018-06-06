/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_XDRIVEMODEL_HPP_
#define _OKAPI_XDRIVEMODEL_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"
#include "okapi/device/rotarysensor/adiEncoder.hpp"
#include "okapi/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
class XDriveModelArgs : public ChassisModelArgs {
  public:
  XDriveModelArgs(std::shared_ptr<AbstractMotor> itopLeftMotor,
                  std::shared_ptr<AbstractMotor> itopRightMotor,
                  std::shared_ptr<AbstractMotor> ibottomRightMotor,
                  std::shared_ptr<AbstractMotor> ibottomLeftMotor, const double imaxOutput = 127);

  XDriveModelArgs(std::shared_ptr<AbstractMotor> itopLeftMotor,
                  std::shared_ptr<AbstractMotor> itopRightMotor,
                  std::shared_ptr<AbstractMotor> ibottomRightMotor,
                  std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                  std::shared_ptr<ContinuousRotarySensor> irightEnc, const double imaxOutput = 127);

  std::shared_ptr<AbstractMotor> topLeftMotor;
  std::shared_ptr<AbstractMotor> topRightMotor;
  std::shared_ptr<AbstractMotor> bottomRightMotor;
  std::shared_ptr<AbstractMotor> bottomLeftMotor;
  std::shared_ptr<ContinuousRotarySensor> leftSensor;
  std::shared_ptr<ContinuousRotarySensor> rightSensor;
  const double maxOutput;
};

class XDriveModel : public ChassisModel {
  public:
  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  XDriveModel(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor,
              Motor ibottomLeftMotor, const double imaxOutput = 127);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  XDriveModel(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor,
              Motor ibottomLeftMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
              const double imaxOutput = 127);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  XDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
              std::shared_ptr<AbstractMotor> itopRightMotor,
              std::shared_ptr<AbstractMotor> ibottomRightMotor,
              std::shared_ptr<AbstractMotor> ibottomLeftMotor, const double imaxOutput = 127);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param ileftEnc Left side encoder
   * @param irightEnc Right side encoder
   */
  XDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
              std::shared_ptr<AbstractMotor> itopRightMotor,
              std::shared_ptr<AbstractMotor> ibottomRightMotor,
              std::shared_ptr<AbstractMotor> ibottomLeftMotor,
              std::shared_ptr<ContinuousRotarySensor> ileftEnc,
              std::shared_ptr<ContinuousRotarySensor> irightEnc, const double imaxOutput = 127);

  XDriveModel(const XDriveModelArgs &iparams);

  XDriveModel(const XDriveModel &other);

  /**
   * Drive the robot forwards (using open-loop control). Uses velocity mode.
   *
   * @param ispeed motor power
   */
  virtual void forward(const double ipower) const override;

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
   * @param ipower motor power
   */
  virtual void rotate(const double ipower) const override;

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
   * Drive the robot with an arcade drive layout. Uses voltage mode.
   *
   * @param izSpeed speed on x axis (right)
   * @param iySpeed speed on y axis (forward)
   * @param izRotation speed around z axis (up)
   * @param ithreshold deadband on joystick values
   */
  virtual void xArcade(const double ixSpeed, const double iySpeed, const double izRotation,
                       const double ithreshold = 0) const;

  /**
   * Power the left side motors. Uses velocity mode.
   *
   * @param ipower motor power
   */
  virtual void left(const double ipower) const override;

  /**
   * Power the right side motors. Uses velocity mode.
   *
   * @param ipower motor power
   */
  virtual void right(const double ipower) const override;

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
   * Returns the top left motor.
   *
   * @return the top left motor
   */
  std::shared_ptr<AbstractMotor> getTopLeftMotor() const;

  /**
   * Returns the top right motor.
   *
   * @return the top right motor
   */
  std::shared_ptr<AbstractMotor> getTopRightMotor() const;

  /**
   * Returns the bottom right motor.
   *
   * @return the bottom right motor
   */
  std::shared_ptr<AbstractMotor> getBottomRightMotor() const;

  /**
   * Returns the bottom left motor.
   *
   * @return the bottom left motor
   */
  std::shared_ptr<AbstractMotor> getBottomLeftMotor() const;

  protected:
  std::shared_ptr<AbstractMotor> topLeftMotor;
  std::shared_ptr<AbstractMotor> topRightMotor;
  std::shared_ptr<AbstractMotor> bottomRightMotor;
  std::shared_ptr<AbstractMotor> bottomLeftMotor;
  std::shared_ptr<ContinuousRotarySensor> leftSensor;
  std::shared_ptr<ContinuousRotarySensor> rightSensor;
  const double maxOutput;
};
} // namespace okapi

#endif
